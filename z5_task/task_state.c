#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"
#include "inverter.h"
#include "scheduler.h"

typedef struct
{
    Uint32 timerMs;
    Uint16 relayOnFlag;
    Uint16 faultFilter;
    Uint16 selfTestPassed;
    Uint16 fault;
} RelayState;

static RelayState gRelayData = {0};
#include "variable.h"

/* 电网丢失（过零看门狗）连续计数阈值，约 50 × 10ms = 500ms。 */
#define GRID_LOST_COUNT_THRESHOLD  50U

/* 继电器自检时序（ms）。原工程 2ms 一拍（wWaitTime），此处按 1 拍 = 2ms 换算，独立于调度周期。 */
#define RELAY_SEQ_STEP1_MS          100U    /* 原 50 拍    合 relay1 */
#define RELAY_SEQ_STEP2_MS          600U    /* 原 300 拍   合 relay2+relay3 */
#define RELAY_SEQ_STEP3_MS         1600U    /* 原 800 拍   断 relay3 */
#define RELAY_SEQ_STEP4_MS         1800U    /* 原 900 拍   合 relay4 */
#define RELAY_SEQ_STEP5_MS         3400U    /* 原 1700 拍  断 relay2 */
#define RELAY_SEQ_STEP6_MS         3600U    /* 原 1800 拍  合 relay3 */
#define RELAY_SEQ_STEP7_MS         5400U    /* 原 2700 拍  断 relay1 */
#define RELAY_SEQ_STEP8_MS         5600U    /* 原 2800 拍  合 relay2 */
#define RELAY_SEQ_STEP9_MIN_MS     7600U    /* 原 3800 拍  最终合 relay1 */
#define RELAY_SEQ_STEP9_MAX_MS     7800U    /* 原 3900 拍 */
#define RELAY_SEQ_TIMEOUT_MS      10400U    /* 原 >5200 拍 超时全断 */

/* 继电器自检压差判据（方案A：电压差判据）. */
#define RELAY_DELTA_V_TRIP_V        60.0f   /* 原 c60V 压差阈值 */
#define RELAY_FAULT_FILTER_MS      250U     /* 原 125 拍 连续判据 */

/* 继电器检测窗口（ms，由原 2ms 拍换算）. */
#define RELAY_WIN_A_MIN_MS         1100U    /* 原 550 拍 */
#define RELAY_WIN_A_MAX_MS         1600U    /* 原 800 拍 */
#define RELAY_WIN_B_MIN_MS         2300U    /* 原 1150 拍 */
#define RELAY_WIN_B_MAX_MS         2800U    /* 原 1400 拍 */
#define RELAY_WIN_C_MIN_MS         4100U    /* 原 2050 拍 */
#define RELAY_WIN_C_MAX_MS         4600U    /* 原 2300 拍 */
#define RELAY_WIN_D_MIN_MS         6100U    /* 原 3050 拍 */
#define RELAY_WIN_D_MAX_MS         6600U    /* 原 3300 拍 */
#define RELAY_WIN_E_MIN_MS         8400U    /* 原 4200 拍 失效检测窗 */
#define RELAY_WIN_E_MAX_MS         8900U    /* 原 4450 拍 */
#define RELAY_SEQ_DONE_MS          8900U    /* 检测窗口结束后可判通过 */

static Uint16 State_IsPresent_DC(void);
static Uint16 State_IsReady_DC(void);

static Uint16 State_IsReady_BUS(void);

static Uint16 State_IsPresent_AC(void);
static Uint16 State_IsReady_AC(void);

static Uint16 State_HasRecoverFault(void);
static Uint16 State_HasPermanentFault(void);

static void State_ResetStartupData(void);
static void State_Enter(SysState nextState);

static void State_RunWait(void);
static void State_RunCheck(void);
static void State_RunNormal(void);
static void State_RunFault(void);
static void State_RunPermanent(void);

static void State_RelaySelfTestInit(void);
static void State_RelaySelfTest(Uint16 deltaMs);

void Task_State_Init(void)
{
    gSysData.state = SYS_STATE_WAIT;
    gSysData.checkStage = SYS_CHECK_RESET;
    gSysData.startRequest = 0U;
    gSysData.sourceReady = 0U;
    gSysData.gridReady = 0U;
    gSysData.busReady = 0U;
    gSysData.sourceStableMs = 0UL;
    gSysData.gridStableMs = 0UL;
    Inverter_EnterSafeOutput();
}

void Task_State(void)
{
    Uint16 keyEvents;

    /* GPIO details and key debounce remain inside the BSP. Key actions will
     * be connected after the operator-control policy is finalized. */
    keyEvents = GPIO_GetKeyEvents();
    (void)keyEvents;
    gSysData.startRequest = POWER_SW1();

    switch(gSysData.state)
    {
        case SYS_STATE_WAIT:        State_RunWait();        break;
        case SYS_STATE_CHECK:       State_RunCheck();       break;
        case SYS_STATE_NORMAL:      State_RunNormal();      break;
        case SYS_STATE_FAULT:       State_RunFault();       break;
        case SYS_STATE_PERMANENT:   State_RunPermanent();   break;
        default:                    State_Enter(SYS_STATE_PERMANENT); break;
    }
}

/* A source is present at the lower run/hold threshold. This check is used
 * after startup and intentionally has hysteresis relative to PV_START_V. */
static Uint16 State_IsPresent_DC(void)
{
    return 
        (
            (gMachineData.realAvg.pv1Voltage >= PV_PRESENT_MIN_V) ||
            (gMachineData.realAvg.pv2Voltage >= PV_PRESENT_MIN_V) ||
            (gMachineData.realAvg.dcBusVoltage >= DC_BUS_MIN_V)
        ) ? 1U : 0U;
}

/* Require the source-start condition to remain valid continuously. A single
 * invalid state-task sample restarts the qualification interval. */
static Uint16 State_IsReady_DC(void)
{
    Uint16 valid;

    valid = ((gMachineData.realAvg.pv1Voltage >= PV_START_V) ||
             (gMachineData.realAvg.pv2Voltage >= PV_START_V) ||
             ((gMachineData.realAvg.dcBusVoltage >= DC_BUS_MIN_V) &&
              (gMachineData.realAvg.dcBusVoltage <= DC_BUS_MAX_V))) ? 1U : 0U;

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

/* The current framework does not start Boost. Therefore CHECK only accepts
 * a bus that is already inside the normal operating window. */
static Uint16 State_IsReady_BUS(void)
{
    float busVoltage;

    busVoltage = gMachineData.realAvg.dcBusVoltage;
    gSysData.busReady = ((busVoltage >= DC_BUS_MIN_V) &&
                         (busVoltage <= DC_BUS_MAX_V)) ? 1U : 0U;
    return gSysData.busReady;
}

/* Update instantaneous grid diagnostics without applying reconnect timing. */
static Uint16 State_IsPresent_AC(void)
{
    float gridFreqHz;
    float gridVoltageRms;
    Uint16 voltageValid;
    Uint16 freqValid;

    gridFreqHz = (float)gMachineData.ecapFreqCent * 0.01f;
    gridVoltageRms = gMachineData.realRms.gridVoltage;

    voltageValid =
        ((gridVoltageRms >= gGridSafety.reconnMinVolt) &&
         (gridVoltageRms <= gGridSafety.reconnMaxVolt)) ? 1U : 0U;
    freqValid =
        ((gridFreqHz >= gGridSafety.reconnMinFreq) &&
         (gridFreqHz <= gGridSafety.reconnMaxFreq)) ? 1U : 0U;
    return ((voltageValid != 0U) &&
         (freqValid != 0U) &&
         (gGridData.fastPresent != 0U)) ? 1U : 0U;
}

/* Reconnect timing belongs to CHECK only. The timer is reset whenever either
 * voltage or frequency leaves the permitted window. */
static Uint16 State_IsReady_AC(void)
{
    if(State_IsPresent_AC() != 0U)
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

static Uint16 State_HasRecoverFault(void)
{
    return (gSysFault.word.recoverable != 0U) ? 1U : 0U;
}

static Uint16 State_HasPermanentFault(void)
{
    return (gSysFault.word.permanent != 0U) ? 1U : 0U;
}

/* Reset only startup/control runtime values. Measurement history and active
 * protection bits remain owned by their producer modules. */
static void State_ResetStartupData(void)
{
    /* 重新校准 ADC 运行时零漂（开机/重连时信号本应为 0）。 */
    gAdcOffsetCal.adInitial = 1U;
    gAdcOffsetCal.checkCount = 0U;
    gAdcOffsetCal.offset.inductorCurrent = 0.0f;
    gAdcOffsetCal.offset.gridVoltage = 0.0f;
    gAdcOffsetCal.offset.gfciCurrent = 0.0f;
    gAdcOffsetCal.offset.gridDcCurrent = 0.0f;
    gAdcOffsetCal.sum.inductorCurrent = 0.0f;
    gAdcOffsetCal.sum.gridVoltage = 0.0f;
    gAdcOffsetCal.sum.gfciCurrent = 0.0f;
    gAdcOffsetCal.sum.gridDcCurrent = 0.0f;

    gSysData.busReady = 0U;
    State_RelaySelfTestInit();

    /* 启动 GFCI 自检（并网前注入 50mA 验证硬件 + 静态/注入检测） */
    gGfciData.selfTestActive = 1U;
    gGfciData.selfTestIndex = 0U;
    gGfciData.deviceFilter1 = 0U;
    gGfciData.deviceFilter2 = 0U;
    GFCI_CHECK_OFF();

    /* Controller internals are reset through their task interfaces. */
    DC_Ctrl_Reset();

    AC_Ctrl_Disable();
}

static void State_Enter(SysState nextState)
{
    if(nextState == gSysData.state)
    {
        return;
    }

    if((nextState == SYS_STATE_WAIT) ||
       (nextState == SYS_STATE_FAULT) ||
       (nextState == SYS_STATE_PERMANENT))
    {
        Inverter_EnterSafeOutput();
        gSysData.sourceReady = 0U;
        gSysData.gridReady = 0U;
        gSysData.busReady = 0U;
        gSysData.sourceStableMs = 0UL;
        gSysData.gridStableMs = 0UL;
    }
    else if(nextState == SYS_STATE_CHECK)
    {
        /* Safe output once on entry; the RELAY stage drives the grid relays
         * during self-test, so it must not be re-asserted every cycle. */
        Inverter_EnterSafeOutput();
        gSysData.checkStage = SYS_CHECK_RESET;
        gSysData.gridReady = 0U;
        gSysData.gridStableMs = 0UL;
    }

    gSysData.state = nextState;
}

static void State_RunWait(void)
{
    Inverter_EnterSafeOutput();

    if(State_HasPermanentFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMANENT);
    }
    else if(State_HasRecoverFault() != 0U)
    {
        gSysData.sourceStableMs = 0UL;
        gSysData.sourceReady = 0U;
    }
    else if(gSysData.startRequest == 0U)
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
    if(State_HasPermanentFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMANENT);
        return;
    }
    if(State_HasRecoverFault() != 0U)
    {
        State_Enter(SYS_STATE_FAULT);
        return;
    }
    if(gSysData.startRequest == 0U)
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
            else if(State_IsPresent_DC() == 0U)
            {
                State_Enter(SYS_STATE_WAIT);
            }
            break;

        case SYS_CHECK_GRID:
            if(State_IsPresent_DC() == 0U)
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
            if(State_IsPresent_DC() == 0U)
            {
                State_Enter(SYS_STATE_WAIT);
            }
            else if(State_IsReady_BUS() != 0U)
            {
                gSysData.checkStage = SYS_CHECK_RELAY;
            }
            break;

        case SYS_CHECK_RELAY:
            State_RelaySelfTest((Uint16)TASK_STATE_PERIOD_MS);
            if (gRelayData.fault != 0U)
            {
                /* 继电器粘连/失效：进 FAULT，恢复后重新走 CHECK 流程。 */
                State_Enter(SYS_STATE_FAULT);
            }
            else if (gRelayData.selfTestPassed != 0U)
            {
                gSysData.checkStage = SYS_CHECK_PREPARE;
            }
            break;

        case SYS_CHECK_PREPARE:
            if((State_IsPresent_DC() == 0U) ||
               (State_IsReady_BUS() == 0U) ||
               (State_IsPresent_AC() == 0U))
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
                    AC_Ctrl_Enable();
                    DSP_STATE_HIGH();
                    State_Enter(SYS_STATE_NORMAL);
                }
            }
            break;

        default:
            State_Enter(SYS_STATE_PERMANENT);
            break;
    }
}

static void State_RunNormal(void)
{
    /* 过零看门狗：快速掉网计数超阈值 → 电网丢失，进 FAULT（会全断输出） */
    if (gGridData.noGridCount > GRID_LOST_COUNT_THRESHOLD)
    {
        gSysFault.bit.noUtility = 1U;
        State_Enter(SYS_STATE_FAULT);
        return;
    }

    /* 打嗝保护恢复：reloadFlag 由快速层（ISR）置位，此处计数到 300ms 后重新软启动。 */
    if(gSysData.reloadFlag != 0U)
    {
        gSysData.reloadCount++;
        if(gSysData.reloadCount > 150U)   /* 150 × 2ms = 300ms */
        {
            gSysData.reloadCount = 0U;
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
        gSysData.reloadCount = 0U;
    }

    if(State_HasPermanentFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMANENT);
    }
    else if(State_HasRecoverFault() != 0U)
    {
        State_Enter(SYS_STATE_FAULT);
    }
    else if((gSysData.startRequest == 0U) ||
            (State_IsPresent_DC() == 0U) ||
            (State_IsReady_BUS() == 0U))
    {
        /* Normal PV depletion or loss of the DC source is not latched as a
         * protection failure. Stop safely and qualify again from WAIT. */
        State_Enter(SYS_STATE_WAIT);
    }
}

static void State_RunFault(void)
{
    Inverter_EnterSafeOutput();

    if(State_HasPermanentFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMANENT);
    }
    else if(State_HasRecoverFault() == 0U)
    {
        State_Enter(SYS_STATE_WAIT);
    }
}

static void State_RunPermanent(void)
{
    Inverter_EnterSafeOutput();
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
    gRelayData.relayOnFlag = 1U;
    gRelayData.faultFilter = 0U;
    gRelayData.selfTestPassed = 0U;
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
    if (gRelayData.timerMs == RELAY_SEQ_STEP1_MS)
    {
        GRID_RELAY1_ON();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP2_MS)
    {
        GRID_RELAY2_ON();
        GRID_RELAY3_ON();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP3_MS)
    {
        GRID_RELAY3_OFF();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP4_MS)
    {
        GRID_RELAY4_ON();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP5_MS)
    {
        GRID_RELAY2_OFF();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP6_MS)
    {
        GRID_RELAY3_ON();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP7_MS)
    {
        GRID_RELAY1_OFF();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP8_MS)
    {
        GRID_RELAY2_ON();
    }
    else if ((gRelayData.timerMs >= RELAY_SEQ_STEP9_MIN_MS) &&
             (gRelayData.timerMs <= RELAY_SEQ_STEP9_MAX_MS))
    {
        if (gRelayData.relayOnFlag == 1U)
        {
            GRID_RELAY1_ON();
            gRelayData.relayOnFlag = 2U;
        }
    }
    else if (gRelayData.timerMs > RELAY_SEQ_TIMEOUT_MS)
    {
        /* 超时未进 Normal：全断 + 复位 */
        GRID_RELAY1_OFF();
        GRID_RELAY2_OFF();
        GRID_RELAY3_OFF();
        GRID_RELAY4_OFF();
        gRelayData.timerMs = 0U;
        gRelayData.relayOnFlag = 1U;
    }

    /* --- 粘连/失效检测（原 sRelayCheck，方案A：电压差） --- */
    deltaV = gMachineData.realRms.gridVoltage - gMachineData.realRms.inverterVoltage;
    deltaVAbs = (deltaV < 0.0f) ? (-deltaV) : deltaV;

    if (State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_A_MIN_MS, RELAY_WIN_A_MAX_MS) ||
        State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_B_MIN_MS, RELAY_WIN_B_MAX_MS) ||
        State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_C_MIN_MS, RELAY_WIN_C_MAX_MS) ||
        State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_D_MIN_MS, RELAY_WIN_D_MAX_MS))
    {
        /* 隔离期：压差本应大，异常小则判粘连（该断的没断） */
        if (deltaVAbs < RELAY_DELTA_V_TRIP_V)
        {
            gRelayData.faultFilter += deltaMs;
            if (gRelayData.faultFilter > RELAY_FAULT_FILTER_MS)
            {
                gRelayData.fault = 1U;
            }
        }
        else
        {
            gRelayData.faultFilter = 0U;
        }
    }
    else if (State_RelayInWindow(gRelayData.timerMs, RELAY_WIN_E_MIN_MS, RELAY_WIN_E_MAX_MS))
    {
        /* 导通期：压差本应小，异常大则判失效（该合的没合） */
        if (deltaVAbs > RELAY_DELTA_V_TRIP_V)
        {
            gRelayData.faultFilter += deltaMs;
            if (gRelayData.faultFilter > RELAY_FAULT_FILTER_MS)
            {
                gRelayData.fault = 1U;
            }
        }
        else
        {
            gRelayData.faultFilter = 0U;
        }
    }
    else
    {
        gRelayData.faultFilter = 0U;
    }

    /* 检测窗口全部结束后且无故障 → 自检通过 */
    if ((gRelayData.timerMs > RELAY_SEQ_DONE_MS) && (gRelayData.fault == 0U))
    {
        gRelayData.selfTestPassed = 1U;
    }
}
