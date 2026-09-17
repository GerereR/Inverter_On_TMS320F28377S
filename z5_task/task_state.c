#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"
#include "invert.h"
#include "sched.h"
#include "variable.h"

//继电器自检状态
typedef struct
{
    // 自检计时器，单位 ms，从 0 开始累加
    Uint32 timerMs;
    // 时序步骤索引，0~9，对应继电器动作序列
    Uint16 seqStep;
    // 故障滤波计数器，单位 ms，用于连续判据消抖
    Uint16 faultFilt;
    // 自检通过标志：1 = 通过，0 = 未通过
    Uint16 selfTestPass;
    // 自检故障标志：1 = 检测到粘连/失效，0 = 正常
    Uint16 fault;
} RelayState;
static RelayState gRelayData = {0};

/* 电网丢失（过零看门狗）连续计数阈值，约 50 × 10ms = 500ms。 */
#define GRID_LOST_CNT_THRESHOLD 50U
// 打嗝恢复等待时间，单位 ms
#define STATE_RELOAD_DELAY_MS   300UL
//将 300ms 向上取整转换为状态任务周期数，用于 reloadCnt 比较
#define STATE_RELOAD_COUNT      ((STATE_RELOAD_DELAY_MS + TASK_STATE_PERIOD_MS - 1U) / TASK_STATE_PERIOD_MS)

/* 继电器自检时序（ms）。原工程 2ms 一拍（wWaitTime），此处按 1 拍 = 2ms 换算，独立于调度周期。 */
//每个步骤的时间
#define RELAY_SEQ_STEP1_MS      100U        /* 合 relay1 */
#define RELAY_SEQ_STEP2_MS      600U        /* 合 relay2 + relay3 */
#define RELAY_SEQ_STEP3_MS      1600U       /* 断 relay3 */
#define RELAY_SEQ_STEP4_MS      1800U       /* 合 relay4 */
#define RELAY_SEQ_STEP5_MS      3400U       /* 断 relay2 */
#define RELAY_SEQ_STEP6_MS      3600U       /* 合 relay3 */
#define RELAY_SEQ_STEP7_MS      5400U       /* 断 relay1 */
#define RELAY_SEQ_STEP8_MS      5600U       /* 合 relay2 */
#define RELAY_SEQ_STEP9_MIN_MS  7600U       /* 最终合 relay1 */
#define RELAY_SEQ_STEP9_MAX_MS  7800U
#define RELAY_SEQ_TIMEOUT_MS    10400U      /* 超时全断 */

/* 压差阈值 */
#define RELAY_DELTA_V_TRIP_V    60.0f       
/* 连续判据, ms */
#define RELAY_FAULT_FILT_MS     250U        

/* 继电器检测窗口（ms，由原 2ms 拍换算）. */
#define RELAY_WIN_A_MIN_MS      1100U    /* 原 550 拍 */
#define RELAY_WIN_A_MAX_MS      1600U    /* 原 800 拍 */
#define RELAY_WIN_B_MIN_MS      2300U    /* 原 1150 拍 */
#define RELAY_WIN_B_MAX_MS      2800U    /* 原 1400 拍 */
#define RELAY_WIN_C_MIN_MS      4100U    /* 原 2050 拍 */
#define RELAY_WIN_C_MAX_MS      4600U    /* 原 2300 拍 */
#define RELAY_WIN_D_MIN_MS      6100U    /* 原 3050 拍 */
#define RELAY_WIN_D_MAX_MS      6600U    /* 原 3300 拍 */
#define RELAY_WIN_E_MIN_MS      8400U    /* 原 4200 拍 失效检测窗 */
#define RELAY_WIN_E_MAX_MS      8900U    /* 原 4450 拍 */
#define RELAY_SEQ_DONE_MS       8900U    /* 检测窗口结束后可判通过 */

static Uint16 State_IsExist_DC(void);
static Uint16 State_IsReady_DC(void);

static Uint16 State_IsReady_BUS(void);

static Uint16 State_IsExist_AC(void);
static Uint16 State_IsReady_AC(void);

static Uint16 State_HasRecovFault(void);
static Uint16 State_HasPermaFault(void);

static void State_ResetStartupData(void);
static void State_Enter(SysState nextState);

static void State_RunWait(void);
static void State_RunCheck(void);
static void State_RunNormal(void);
static void State_RunFault(void);
static void State_RunPerma(void);

static void State_RelaySelfTestInit(void);
static void State_RelaySelfTest(Uint16 deltaMs);

//状态机任务初始化
void Task_State_Init(void)
{
    //等待态是一切的起点
    gSysData.state = SYS_STATE_WAIT;
    //准备复位启动数据
    gSysData.checkStage = SYS_CHECK_RESET;
    gSysData.startReq = 0U;
    gSysData.sourceReady = 0U;
    gSysData.gridReady = 0U;
    gSysData.busReady = 0U;
    gSysData.sourceStableMs = 0UL;
    gSysData.gridStableMs = 0UL;
    gSysData.reloadFlag = 0U;
    gSysData.reloadCnt = 0U;
    Invert_EnterSafeOutput();
}

void Task_State(void)
{
    Uint16 keyEvents;
    //我选择把按键检测任务放在state里面,是因为这里的时间合适
    //按照我一贯的原则, 大概率会选择外部触发或者单独的Key任务, 但是key任务太小了,没必要
    //假如未来需要重度使用UI的话, 到时候自然会把UItask配置为key触发, 然后key属于UItask
    keyEvents = GPIO_GetKeyEvents();
    /* TODO: consume keyEvents when local-key state control is implemented. */
    (void)keyEvents;
    gSysData.startReq = POWER_SW1();

    switch(gSysData.state)
    {
        case SYS_STATE_WAIT:        State_RunWait();        break;
        case SYS_STATE_CHECK:       State_RunCheck();       break;
        case SYS_STATE_NORMAL:      State_RunNormal();      break;
        case SYS_STATE_FAULT:       State_RunFault();       break;
        case SYS_STATE_PERMA:       State_RunPerma();       break;
        default:                    State_Enter(SYS_STATE_PERMA); break;
    }
}

// 检查直流源是否存在（低阈值
static Uint16 State_IsExist_DC(void)
{
    return 
        (
            (gMachineData.realAvg.pv1Volt >= PV_PRESENT_MIN_V) ||
            (gMachineData.realAvg.pv2Volt >= PV_PRESENT_MIN_V) ||
            (gMachineData.realAvg.dcBusVolt >= DC_BUS_MIN_V)
        ) ? 1U : 0U;
}

// 检查直流源是否稳定满足启动条件
static Uint16 State_IsReady_DC(void)
{
    Uint16 valid;

    valid = ((gMachineData.realAvg.pv1Volt >= PV_START_V) ||
             (gMachineData.realAvg.pv2Volt >= PV_START_V) ||
             ((gMachineData.realAvg.dcBusVolt >= DC_BUS_MIN_V) &&
              (gMachineData.realAvg.dcBusVolt <= DC_BUS_MAX_V))) ? 1U : 0U;

    if(valid != 0U)
    {
        if(gSysData.sourceStableMs < SOURCE_QUALIFY_DELAY_MS)
        {
            gSysData.sourceStableMs += (Uint32)TASK_STATE_PERIOD_MS;
            if(gSysData.sourceStableMs > SOURCE_QUALIFY_DELAY_MS)
            {
                gSysData.sourceStableMs = SOURCE_QUALIFY_DELAY_MS;
            }
        }
    }
    else
    {
        gSysData.sourceStableMs = 0UL;
    }

    gSysData.sourceReady = (gSysData.sourceStableMs >= SOURCE_QUALIFY_DELAY_MS) ? 1U : 0U;
    return gSysData.sourceReady;
}

// 检查母线电压是否在正常窗口
static Uint16 State_IsReady_BUS(void)
{
    float busVolt;

    busVolt = gMachineData.realAvg.dcBusVolt;
    gSysData.busReady = ((busVolt >= DC_BUS_MIN_V) &&
                         (busVolt <= DC_BUS_MAX_V)) ? 1U : 0U;
    return gSysData.busReady;
}

// 检查电网瞬时是否有效（电压/频率/存在标志）
static Uint16 State_IsExist_AC(void)
{
    float gridFreqHz;
    float gridVoltRms;
    Uint16 voltValid;
    Uint16 freqValid;

    gridFreqHz = (float)gMachineData.ecapFreqCent * 0.01f;
    gridVoltRms = gMachineData.realRms.gridVolt;

    voltValid =
        ((gridVoltRms >= gGridSafety.reconnMinVolt) &&
         (gridVoltRms <= gGridSafety.reconnMaxVolt)) ? 1U : 0U;
    freqValid =
        ((gridFreqHz >= gGridSafety.reconnMinFreq) &&
         (gridFreqHz <= gGridSafety.reconnMaxFreq)) ? 1U : 0U;
    return ((voltValid != 0U) &&
         (freqValid != 0U) &&
         (gGridData.fastPresent != 0U)) ? 1U : 0U;
}

// 检查电网是否持续稳定满足并网条件
static Uint16 State_IsReady_AC(void)
{
    if(State_IsExist_AC() != 0U)
    {
        if(gSysData.gridStableMs < GRID_RECONN_DELAY_MS)
        {
            gSysData.gridStableMs += (Uint32)TASK_STATE_PERIOD_MS;
            if(gSysData.gridStableMs > GRID_RECONN_DELAY_MS)
            {
                gSysData.gridStableMs = GRID_RECONN_DELAY_MS;
            }
        }
    }
    else
    {
        gSysData.gridStableMs = 0UL;
    }

    gSysData.gridReady =
        (gSysData.gridStableMs >= GRID_RECONN_DELAY_MS) ? 1U : 0U;
    return gSysData.gridReady;
}

// 是否存在可恢复故障
static Uint16 State_HasRecovFault(void)
{
    return (gSysProblem.recovFault != 0UL) ? 1U : 0U;
}

// 是否存在永久故障
static Uint16 State_HasPermaFault(void)
{
    return (gSysProblem.permaFault != 0UL) ? 1U : 0U;
}

// 复位启动相关的运行时数据
static void State_ResetStartupData(void)
{
    /* 重新校准 ADC 运行时零漂（开机/重连时信号本应为 0）。 */
    gAdcDrift.adjInit = 1U;
    gAdcDrift.checkCnt = 0U;
    gAdcDrift.drift.inductCurr = 0.0f;
    gAdcDrift.drift.gridVolt = 0.0f;
    gAdcDrift.drift.gfciCurr = 0.0f;
    gAdcDrift.drift.gridDcCurr = 0.0f;
    gAdcDrift.sum.inductCurr = 0.0f;
    gAdcDrift.sum.gridVolt = 0.0f;
    gAdcDrift.sum.gfciCurr = 0.0f;
    gAdcDrift.sum.gridDcCurr = 0.0f;

    gSysData.busReady = 0U;
    State_RelaySelfTestInit();

    /* 启动 GFCI 自检（并网前注入 50mA 验证硬件 + 静态/注入检测） */
    gGfciData.selfTestActive = 1U;
    gGfciData.selfTestIdx = 0U;
    gGfciData.deviceFilt1 = 0U;
    gGfciData.deviceFilt2 = 0U;
    GFCI_CHECK_OFF();

    /* Controller internals are reset through their task interfaces. */
    DC_Ctrl_Reset();

    AC_Ctrl_Disable();
}

// 状态切换统一入口
static void State_Enter(SysState nextState)
{
    if(nextState == gSysData.state)
    {
        return;
    }

    if((nextState == SYS_STATE_WAIT) ||
       (nextState == SYS_STATE_FAULT) ||
       (nextState == SYS_STATE_PERMA))
    {
        Invert_EnterSafeOutput();
        gSysData.sourceReady = 0U;
        gSysData.gridReady = 0U;
        gSysData.busReady = 0U;
        gSysData.sourceStableMs = 0UL;
        gSysData.gridStableMs = 0UL;
        gSysData.reloadFlag = 0U;
        gSysData.reloadCnt = 0U;
    }
    else if(nextState == SYS_STATE_CHECK)
    {
        /* Safe output once on entry; the RELAY stage drives the grid relays
         * during self-test, so it must not be re-asserted every cycle. */
        Invert_EnterSafeOutput();
        gSysData.checkStage = SYS_CHECK_RESET;
        gSysData.gridReady = 0U;
        gSysData.gridStableMs = 0UL;
    }

    gSysData.state = nextState;
}

static void State_RunWait(void)
{
    Invert_EnterSafeOutput();

    if(State_HasPermaFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMA);
    }
    else if(State_HasRecovFault() != 0U)
    {
        gSysData.sourceStableMs = 0UL;
        gSysData.sourceReady = 0U;
    }
    else if(gSysData.startReq == 0U)
    {
        gSysData.sourceStableMs = 0UL;
        gSysData.sourceReady = 0U;
    }
    else if(State_IsReady_DC() != 0U)
    {
        State_Enter(SYS_STATE_CHECK);
    }
}

static void State_RunCheck(void)
{
    if(State_HasPermaFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMA);
        return;
    }
    if(State_HasRecovFault() != 0U)
    {
        State_Enter(SYS_STATE_FAULT);
        return;
    }
    if(gSysData.startReq == 0U)
    {
        State_Enter(SYS_STATE_WAIT);
        return;
    }

    switch(gSysData.checkStage)
    {
        case SYS_CHECK_RESET:
            State_ResetStartupData();
            gSysData.checkStage = SYS_CHECK_SOURCE;
            break;

        case SYS_CHECK_SOURCE:
            if(State_IsReady_DC() != 0U)
            {
                gSysData.checkStage = SYS_CHECK_GRID;
            }
            else if(State_IsExist_DC() == 0U)
            {
                State_Enter(SYS_STATE_WAIT);
            }
            break;

        case SYS_CHECK_GRID:
            if(State_IsExist_DC() == 0U)
            {
                State_Enter(SYS_STATE_WAIT);
            }
            else if(State_IsReady_AC() != 0U)
            {
                DC_Ctrl_StartSoftStart();   /* 电网就绪，启动 Boost 软启动建母线 */
                gSysData.checkStage = SYS_CHECK_BUS;
            }
            break;

        case SYS_CHECK_BUS:
            if((State_IsExist_DC() == 0U) || (State_IsExist_AC() == 0U))
            {
                State_Enter(SYS_STATE_WAIT);
            }
            else if(State_IsReady_BUS() != 0U)
            {
                gSysData.checkStage = SYS_CHECK_RELAY;
            }
            break;

        case SYS_CHECK_RELAY:
            if((State_IsExist_DC() == 0U) || (State_IsExist_AC() == 0U))
            {
                State_Enter(SYS_STATE_WAIT);
            }
            else
            {
                State_RelaySelfTest((Uint16)TASK_STATE_PERIOD_MS);
                if (gRelayData.fault != 0U)
                {
                    /* 继电器粘连/失效：进 FAULT，恢复后重新走 CHECK 流程。 */
                    gSysProblem.recovFault |= RECOV_RELAY_SELFTEST;
                    State_Enter(SYS_STATE_FAULT);
                }
                else if (gRelayData.selfTestPass != 0U)
                {
                    gSysData.checkStage = SYS_CHECK_PREPARE;
                }
            }
            break;

        case SYS_CHECK_PREPARE:
            if((State_IsExist_DC() == 0U) ||
               (State_IsReady_BUS() == 0U) ||
               (State_IsExist_AC() == 0U))
            {
                gSysData.checkStage = SYS_CHECK_SOURCE;
                gSysData.sourceStableMs = 0UL;
                gSysData.gridStableMs = 0UL;
                gSysData.sourceReady = 0U;
                gSysData.gridReady = 0U;
            }
            else if(gGfciData.selfTestActive != 0U)
            {
                /* GFCI 自检未完成，停留在 PREPARE 等待。通常继电器自检(8.9s)
                 * 远长于 GFCI 自检(约1s)，此处为防御性门控：避免将来调整时序后
                 * 自检未完成就并网。 */
            }
            else
            {
                /* ADC/DMA flags are owned by their ISRs and are not cleared
                 * here. Only the verified-inactive TZ latches are eligible. */
                if(EPWM_Enable() != 0U)
                {
                    State_Enter(SYS_STATE_NORMAL);
                    AC_Ctrl_Enable();
                    DSP_STATE_HIGH();
                }
            }
            break;

        default:
            State_Enter(SYS_STATE_PERMA);
            break;
    }
}

static void State_RunNormal(void)
{
    /* 过零看门狗：快速掉网计数超阈值 → 电网丢失，进 FAULT（会全断输出） */
    if (gGridData.noGridCnt > GRID_LOST_CNT_THRESHOLD)
    {
        gSysProblem.recovFault |= RECOV_NO_UTILITY;
        State_Enter(SYS_STATE_FAULT);
        return;
    }

    /* 打嗝保护恢复：reloadFlag 由快速层（ISR）置位，此处计数到 300ms 后重新软启动。 */
    if(gSysData.reloadFlag != 0U)
    {
        gSysData.reloadCnt++;
        if(gSysData.reloadCnt >= STATE_RELOAD_COUNT)
        {
            gSysData.reloadCnt = 0U;
            gSysData.reloadFlag = 0U;
            DC_Ctrl_Reset();
            if(EPWM_Enable() != 0U)
            {
                AC_Ctrl_Enable();
            }
        }
    }
    else
    {
        gSysData.reloadCnt = 0U;
    }

    if(State_HasPermaFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMA);
    }
    else if(State_HasRecovFault() != 0U)
    {
        State_Enter(SYS_STATE_FAULT);
    }
    else if((gSysData.startReq == 0U) ||
            (State_IsExist_DC() == 0U) ||
            (State_IsReady_BUS() == 0U))
    {
        /* Normal PV depletion or loss of the DC source is not latched as a
         * protection failure. Stop safely and qualify again from WAIT. */
        State_Enter(SYS_STATE_WAIT);
    }
}

static void State_RunFault(void)
{
    Invert_EnterSafeOutput();

    if(State_HasPermaFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMA);
    }
    else if(State_HasRecovFault() == 0U)
    {
        State_Enter(SYS_STATE_WAIT);
    }
}

static void State_RunPerma(void)
{
    Invert_EnterSafeOutput();
}

/*============================================================================
 * 并网继电器自检（方案A：电网电压 - 逆变电压 压差判据）
 * 对应原 G83_SLAVE 的 sSlaveRelayControl() / sRelayCheck()。
 * 时序用毫秒表达，由 CHECK_RELAY 阶段按状态任务周期推进。
 *==========================================================================*/
static Uint16 State_RelayInWindow(Uint32 ms, Uint16 loMs, Uint16 hiMs)
{
    return ((ms >= (Uint32)loMs) && (ms <= (Uint32)hiMs)) ? 1U : 0U;
}

static void State_RelaySelfTestInit(void)
{
    gRelayData.timerMs = 0U;
    /* 单 DSP：本机即为并网决策者，进入自检即视为允许完成并网。 */
    gRelayData.seqStep = 0U;
    gRelayData.faultFilt = 0U;
    gRelayData.selfTestPass = 0U;
    gRelayData.fault = 0U;

    GRID_RELAY1_OFF();
    GRID_RELAY2_OFF();
    GRID_RELAY3_OFF();
    GRID_RELAY4_OFF();
}

static void State_RelaySelfTest(Uint16 deltaMs)
{
    float deltaV;
    float deltaVAbs;

    /* --- 时序推进 --- */
    gRelayData.timerMs += (Uint32)deltaMs;

    /* --- 时序执行（原 sSlaveRelayControl，拍换算为 ms） --- */
    switch(gRelayData.seqStep)
    {
        case 0U:
            if(gRelayData.timerMs >= RELAY_SEQ_STEP1_MS)
            {
                GRID_RELAY1_ON();
                gRelayData.seqStep = 1U;
            }
            break;
        case 1U:
            if(gRelayData.timerMs >= RELAY_SEQ_STEP2_MS)
            {
                GRID_RELAY2_ON();
                GRID_RELAY3_ON();
                gRelayData.seqStep = 2U;
            }
            break;
        case 2U:
            if(gRelayData.timerMs >= RELAY_SEQ_STEP3_MS)
            {
                GRID_RELAY3_OFF();
                gRelayData.seqStep = 3U;
            }
            break;
        case 3U:
            if(gRelayData.timerMs >= RELAY_SEQ_STEP4_MS)
            {
                GRID_RELAY4_ON();
                gRelayData.seqStep = 4U;
            }
            break;
        case 4U:
            if(gRelayData.timerMs >= RELAY_SEQ_STEP5_MS)
            {
                GRID_RELAY2_OFF();
                gRelayData.seqStep = 5U;
            }
            break;
        case 5U:
            if(gRelayData.timerMs >= RELAY_SEQ_STEP6_MS)
            {
                GRID_RELAY3_ON();
                gRelayData.seqStep = 6U;
            }
            break;
        case 6U:
            if(gRelayData.timerMs >= RELAY_SEQ_STEP7_MS)
            {
                GRID_RELAY1_OFF();
                gRelayData.seqStep = 7U;
            }
            break;
        case 7U:
            if(gRelayData.timerMs >= RELAY_SEQ_STEP8_MS)
            {
                GRID_RELAY2_ON();
                gRelayData.seqStep = 8U;
            }
            break;
        case 8U:
            if(gRelayData.timerMs >= RELAY_SEQ_STEP9_MIN_MS)
            {
                GRID_RELAY1_ON();
                gRelayData.seqStep = 9U;
            }
            break;
        default:
            break;
    }

    if(gRelayData.timerMs >= RELAY_SEQ_TIMEOUT_MS)
    {
        /* 超时未完成自检：全断并锁定故障，等待上层故障恢复流程。 */
        GRID_RELAY1_OFF();
        GRID_RELAY2_OFF();
        GRID_RELAY3_OFF();
        GRID_RELAY4_OFF();
        gRelayData.fault = 1U;
    }

    /* --- 粘连/失效检测（原 sRelayCheck，方案A：电压差） --- */
    deltaV = gMachineData.realRms.gridVolt - gMachineData.realRms.invertVolt;
    deltaVAbs = (deltaV < 0.0f) ? (-deltaV) : deltaV;

    if (State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_A_MIN_MS, RELAY_WIN_A_MAX_MS) ||
        State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_B_MIN_MS, RELAY_WIN_B_MAX_MS) ||
        State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_C_MIN_MS, RELAY_WIN_C_MAX_MS) ||
        State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_D_MIN_MS, RELAY_WIN_D_MAX_MS))
    {
        /* 隔离期：压差本应大，异常小则判粘连（该断的没断） */
        if (deltaVAbs < RELAY_DELTA_V_TRIP_V)
        {
            gRelayData.faultFilt += deltaMs;
            if (gRelayData.faultFilt >= RELAY_FAULT_FILT_MS)
            {
                gRelayData.fault = 1U;
            }
        }
        else
        {
            gRelayData.faultFilt = 0U;
        }
    }
    else if (State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_E_MIN_MS, RELAY_WIN_E_MAX_MS))
    {
        /* 导通期：压差本应小，异常大则判失效（该合的没合） */
        if (deltaVAbs > RELAY_DELTA_V_TRIP_V)
        {
            gRelayData.faultFilt += deltaMs;
            if (gRelayData.faultFilt >= RELAY_FAULT_FILT_MS)
            {
                gRelayData.fault = 1U;
            }
        }
        else
        {
            gRelayData.faultFilt = 0U;
        }
    }
    else
    {
        gRelayData.faultFilt = 0U;
    }

    /* 检测窗口全部结束后且无故障 → 自检通过 */
    if ((gRelayData.timerMs >= RELAY_SEQ_DONE_MS) && (gRelayData.fault == 0U))
    {
        gRelayData.selfTestPass = 1U;
    }
}
