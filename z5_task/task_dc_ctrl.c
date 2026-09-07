#include "F28x_Project.h"

#include "task.h"
#include "bsp.h"
#include "variable.h"
#include "system.h"

/* --- 母线环 / Boost 环私有常量（原 constant.h 迁入） --- */
/* Initial DC-bus outer-loop settings. The gains are bring-up values and
 * should be retuned after the power stage and bus capacitance are verified. */
#define BUS_CTRL_PERIOD_S                0.01f
#define BUS_VOLT_REF_V                 400.0f
#define BUS_PI_KP                        0.002f
#define BUS_PI_KI                        0.02f

/* Initial PV-voltage Boost-loop settings. The loop runs at the grid-peak
 * control event rate (approximately 100 Hz for a 50 Hz grid). */
#define BOOST_CTRL_PERIOD_S              0.01f
#define BOOST_PI_KP                      0.0005f
#define BOOST_PI_KI                      0.01f
#define BOOST_DUTY_MIN                   0.0f
#define BOOST_DUTY_MAX                   0.98f

/* Boost 软启动（建母线）占空比每拍爬升步长（峰值触发约 10ms 一拍）。 */
#define BOOST_SOFT_START_STEP            0.02f

/* 自适应母线目标电压（原 stand_volt / stable_ref）：
 * 母线跟随电网峰值 + 余量，clamp 370~430V，并保证不低于 MPPT 目标 PV 电压 + 余量
 * （至少一路 Boost 直通，减少 Boost 开关损耗）。 */
#define BUS_VOLT_GRID_MARGIN_V         30.0f   /* 母线高于电网峰值的最小余量 */
#define BUS_VOLT_MPPT_MARGIN_V         25.0f   /* 母线高于 MPPT 目标 PV 电压的余量 */
#define GRID_RMS_TO_PEAK               1.414f  /* 电网 RMS → 峰值 */

/* 母线/PV 保护滤波计数（峰值触发约 10ms 一拍）。 */
#define BUS_OV_FILTER_COUNT              100U    /* 母线过压连续判定（约 1s） */
#define BUS_UV_FILTER_COUNT              500U    /* 母线欠压连续判定（约 5s，仅并网态） */
#define PV_OV_FILTER_COUNT                30U    /* PV 过压连续判定（约 300ms） */
#define PV_OV_RECOVER_COUNT               30U    /* PV 过压恢复连续正常 */

static void DC_Ctrl_BoostResetChannels(void);

/* One coherent input snapshot for the DC-control task. The producers are
 * cooperative tasks today; keeping this snapshot local also makes the control
 * decision independent of later global updates during the same invocation. */
typedef struct
{
    float busVoltage;
    float pv1Voltage;
    float pv2Voltage;
    float pv1VoltageRef;
    float pv2VoltageRef;
    float currentAmpLimit;
    float gridVoltageRms;
    Uint16 pv1Enabled;
    Uint16 pv2Enabled;
} DC_CtrlInput;

typedef struct
{
    float integral;
    float output;
    Uint16 initialized;
} DC_BusLoopState;

typedef struct
{
    float integral;
    float duty;
} DC_BoostLoopState;

typedef struct
{
    DC_BusLoopState bus;
    DC_BoostLoopState boost1;
    DC_BoostLoopState boost2;
    Uint16 softStartActive;
} DC_CtrlState;

static DC_CtrlState DC_State = {0};

static void DC_Ctrl_ApplyBoostDuty(void)
{
    gBusCtrlData.boost1Duty = DC_State.boost1.duty;
    gBusCtrlData.boost2Duty = DC_State.boost2.duty;
    EPWM_SetBoostDuty(gBusCtrlData.boost1Duty, gBusCtrlData.boost2Duty);
}

/*============================================================================
 * 直流控制任务（峰值触发）：母线电压外环 + 双路 Boost 环。
 * 原 z8_control/bus_ctrl.c + boost_ctrl.c 的实现已内联至此。
 * 母线环输出电流幅值指令给电流环（task_fast），Boost 环输出占空比给 PWM。
 *==========================================================================*/

/* 自适应母线目标电压（原 stand_volt + sUpdateSBusRef）：
 * stableVoltRef = max(电网峰值 + 余量, MPPT 目标 + 余量)，clamp 370~430V。 */
static void BusVoltRef_Adapt(const DC_CtrlInput *input)
{
    float gridPeak;
    float standVolt;
    float mpptTarget;
    float stableRef;

    /* 电网峰值 = RMS × √2 */
    gridPeak = input->gridVoltageRms * GRID_RMS_TO_PEAK;

    /* 母线目标下限：跟随电网峰值 + 余量，clamp 370~430V */
    standVolt = gridPeak + BUS_VOLT_GRID_MARGIN_V;
    standVolt = System_Clamp(standVolt, DC_BUS_MIN_V, DC_BUS_MAX_V);

    /* MPPT 目标 PV 电压（取两路较大者）+ 余量 → 至少一路 Boost 直通 */
    mpptTarget = input->pv1VoltageRef;
    if (input->pv2VoltageRef > mpptTarget)
    {
        mpptTarget = input->pv2VoltageRef;
    }
    mpptTarget += BUS_VOLT_MPPT_MARGIN_V;

    /* 最终稳定参考 = max(电网自适应, MPPT 目标)，clamp 370~430V */
    stableRef = standVolt;
    if (mpptTarget > stableRef)
    {
        stableRef = mpptTarget;
    }
    stableRef = System_Clamp(stableRef, DC_BUS_MIN_V, DC_BUS_MAX_V);

    gBusCtrlData.stableVoltRef = stableRef;
}

/* --- 母线电压外环（原 Ctrl_BusRun） --- */
static void DC_Ctrl_BusRun(const DC_CtrlInput *input)
{
    float proportional;
    float candidate;
    float currentAmpLimit;

    if(DC_State.bus.initialized == 0U)
    {
        gBusCtrlData.stableVoltRef = BUS_VOLT_REF_V;
        DC_State.bus.integral = 0.0f;
        DC_State.bus.output = 0.0f;
        gBusCtrlData.currentAmpRef = 0.0f;
        DC_State.bus.initialized = 1U;
    }

    currentAmpLimit = System_Clamp(input->currentAmpLimit,
                                   BUS_CURRENT_AMP_MIN_NORM,
                                   BUS_CURRENT_AMP_MAX_NORM);

    proportional = BUS_PI_KP * (input->busVoltage - gBusCtrlData.stableVoltRef);

    // 积分反算抗饱和算法
    // 说人话就是在积分项提前加上积分值
    DC_State.bus.integral += BUS_PI_KI * BUS_CTRL_PERIOD_S * (input->busVoltage - gBusCtrlData.stableVoltRef);

    candidate = proportional + DC_State.bus.integral;
    DC_State.bus.output = System_Clamp(candidate, BUS_CURRENT_AMP_MIN_NORM, currentAmpLimit);

    /* 反算：piIntegral = clamp后输出 - 比例项，锚定到限幅处。 */
    DC_State.bus.integral = DC_State.bus.output - proportional;

    gBusCtrlData.currentAmpRef = DC_State.bus.output;
}

/* --- Boost 环（原 Ctrl_BoostUpdateChannel / Ctrl_BoostRun） --- */
/* 单路 Boost PI：让 PV 电压跟踪目标电压（MPPT 参考），占空比写入 ch->duty。
 * ch 指向该通道状态（boost1/boost2），pvVoltage 实测、pvVoltageRef 目标。 */
static void DC_Ctrl_BoostUpdateChannel
(
    DC_BoostLoopState *channel,
    float pvVoltage,
    float pvVoltageRef
)
{
    float error;
    float proportional;
    float candidate;

    /* PV 电压误差：实际 - 目标（Boost 升压，误差正 → 占空比↑ → 抽更多 → 电压回落） */
    error = pvVoltage - pvVoltageRef;
    proportional = BOOST_PI_KP * error;
    candidate = proportional + channel->integral;

    /* 条件积分抗饱和：输出饱和在上/下限且误差还在推高/推低时冻结积分。 */
    if (!((candidate >= BOOST_DUTY_MAX && error > 0.0f) ||
          (candidate <= BOOST_DUTY_MIN && error < 0.0f)))
    {
        channel->integral += BOOST_PI_KI * BOOST_CTRL_PERIOD_S * error;
    }

    candidate = proportional + channel->integral;
    channel->duty = System_Clamp(candidate, BOOST_DUTY_MIN, BOOST_DUTY_MAX);
}

static void DC_Ctrl_BoostRun(const DC_CtrlInput *input)
{
    /* 两路都没使能：全部复位 */
    if((input->pv1Enabled == 0U) && (input->pv2Enabled == 0U))
    {
        DC_Ctrl_BoostResetChannels();
        return;
    }

    /* 每路独立：使能 && 电压有效 && 目标有效 → 跑 PI；否则复位该通道 */
    if((input->pv1Enabled != 0U) && (input->pv1Voltage > 0.0f) && (input->pv1VoltageRef >= PV_PRESENT_MIN_V))
    {
        DC_Ctrl_BoostUpdateChannel(&DC_State.boost1,
                                   input->pv1Voltage,
                                   input->pv1VoltageRef);
    }
    else
    {
        DC_State.boost1.integral = 0.0f;
        DC_State.boost1.duty = 0.0f;
    }

    if((input->pv2Enabled != 0U) && (input->pv2Voltage > 0.0f) && (input->pv2VoltageRef >= PV_PRESENT_MIN_V))
    {
        DC_Ctrl_BoostUpdateChannel(&DC_State.boost2,
                                   input->pv2Voltage,
                                   input->pv2VoltageRef);
    }
    else
    {
        DC_State.boost2.integral = 0.0f;
        DC_State.boost2.duty = 0.0f;
    }
}

static void DC_Ctrl_BoostResetChannels(void)
{
    DC_State.boost1.integral = 0.0f;
    DC_State.boost1.duty = 0.0f;
    DC_State.boost2.integral = 0.0f;
    DC_State.boost2.duty = 0.0f;
    gBusCtrlData.boost1Duty = 0.0f;
    gBusCtrlData.boost2Duty = 0.0f;
}


/* 对外：复位母线环 + Boost（供状态机 State_ResetStartupData 调用）。 */
void DC_Ctrl_Reset(void)
{
    DC_State.bus.integral = 0.0f;
    DC_State.bus.output = 0.0f;
    DC_State.bus.initialized = 0U;
    DC_State.softStartActive = 0U;
    gBusCtrlData.currentAmpRef = 0.0f;
    DC_Ctrl_BoostResetChannels();
    DC_Ctrl_ApplyBoostDuty();
}

void DC_Ctrl_StartSoftStart(void)
{
    DC_State.softStartActive = 1U;
}

/* Boost 软启动（原 sStartBoostControl）：占空比从 0 平滑爬升，把母线从 PV
 * 电压泵到最低目标。母线达标后停止爬升、保持占空比，等并网后 MPPT 接管。 */
static void DC_Ctrl_BoostSoftStart(const DC_CtrlInput *input)
{
    if (DC_State.softStartActive == 0U)
    {
        return;
    }

    /* 只对已接入 PV 的通道爬占空比，未接通道保持 0（单路调试场景）。 */
    if (input->pv1Voltage >= PV_PRESENT_MIN_V)
    {
        DC_State.boost1.duty += BOOST_SOFT_START_STEP;
        if (DC_State.boost1.duty > BOOST_DUTY_MAX)
        {
            DC_State.boost1.duty = BOOST_DUTY_MAX;
        }
    }

    if (input->pv2Voltage >= PV_PRESENT_MIN_V)
    {
        DC_State.boost2.duty += BOOST_SOFT_START_STEP;
        if (DC_State.boost2.duty > BOOST_DUTY_MAX)
        {
            DC_State.boost2.duty = BOOST_DUTY_MAX;
        }
    }

    //我自己加的5V,保险一点
    if (input->busVoltage >= DC_BUS_MIN_V + 5.0f)
    {
        DC_State.softStartActive = 0U;
    }
}

/* 母线过压保护（原 VBUSCheck 过压）+ 母线欠压打嗝：
 * 过压任何态判、置 permanent 故障（Boost 失控/硬件损坏不可恢复）；
 * 欠压仅并网态判、置打嗝标志（reloadFlag，重新软启动，不停机）。 */
static void DC_Ctrl_CheckBus(const DC_CtrlInput *input)
{
    static Uint16 busOvpFilter = 0U;
    static Uint16 busUvpFilter = 0U;

    if (input->busVoltage > DC_BUS_OV_TRIP_V)
    {
        busOvpFilter++;
        if (busOvpFilter >= BUS_OV_FILTER_COUNT)
        {
            busOvpFilter = 0U;
            gSysFault.bit.dcBusOverVolt = 1U;
        }
    }
    else
    {
        busOvpFilter = 0U;
    }

    if (gSysData.state == SYS_STATE_NORMAL)
    {
        if ((input->busVoltage < DC_BUS_MIN_V) && (input->busVoltage > 0.0f))
        {
            busUvpFilter++;
            if (busUvpFilter >= BUS_UV_FILTER_COUNT)
            {
                busUvpFilter = 0U;
                gSysData.reloadFlag = 1U;   /* 母线欠压：打嗝（重新软启动），不停机 */
            }
        }
        else
        {
            busUvpFilter = 0U;
        }
    }
    else
    {
        busUvpFilter = 0U;
    }
}

/* PV 过压保护（原 VPVCheck）：双路独立判定 + 恢复。 */
static void DC_Ctrl_CheckPv(const DC_CtrlInput *input)
{
    static Uint16 pv1OvpFilter = 0U;
    static Uint16 pv2OvpFilter = 0U;
    static Uint16 pv1OvpBackFilter = 0U;
    static Uint16 pv2OvpBackFilter = 0U;

    if (input->pv1Voltage > PV_OV_TRIP_V)
    {
        pv1OvpFilter++;
        if (pv1OvpFilter >= PV_OV_FILTER_COUNT)
        {
            pv1OvpFilter = 0U;
            gSysFault.bit.pv1OverVolt = 1U;
        }
    }
    else
    {
        pv1OvpFilter = 0U;
    }


    if (input->pv2Voltage > PV_OV_TRIP_V)
    {
        pv2OvpFilter++;
        if (pv2OvpFilter >= PV_OV_FILTER_COUNT)
        {
            pv2OvpFilter = 0U;
            gSysFault.bit.pv2OverVolt = 1U;
        }
    }
    else
    {
        pv2OvpFilter = 0U;
    }

    if (gSysFault.bit.pv1OverVolt != 0U)
    {
        if (input->pv1Voltage < PV_OV_RECOVER_V)
        {
            pv1OvpBackFilter++;
            if (pv1OvpBackFilter >= PV_OV_RECOVER_COUNT)
            {
                pv1OvpBackFilter = 0U;
                gSysFault.bit.pv1OverVolt = 0U;
            }
        }
        else
        {
            pv1OvpBackFilter = 0U;
        }
    }

    if (gSysFault.bit.pv2OverVolt != 0U)
    {
        if (input->pv2Voltage < PV_OV_RECOVER_V)
        {
            pv2OvpBackFilter++;
            if (pv2OvpBackFilter >= PV_OV_RECOVER_COUNT)
            {
                pv2OvpBackFilter = 0U;
                gSysFault.bit.pv2OverVolt = 0U;
            }
        }
        else
        {
            pv2OvpBackFilter = 0U;
        }
    }
}

/* --- 任务编排（峰值触发） --- */
void Task_DC_Ctrl(void)
{
    DC_CtrlInput input;

    input.busVoltage = gMachineData.realAvg.dcBusVoltage;
    input.pv1Voltage = gMachineData.realAvg.pv1Voltage;
    input.pv2Voltage = gMachineData.realAvg.pv2Voltage;
    input.pv1VoltageRef = gMpptData.pv1.voltRef;
    input.pv2VoltageRef = gMpptData.pv2.voltRef;
    input.pv1Enabled = gMpptData.pv1.enabled;
    input.pv2Enabled = gMpptData.pv2.enabled;
    input.currentAmpLimit = gPowerLimitData.currentAmpLimit;
    input.gridVoltageRms = gMachineData.realRms.gridVoltage;

    /* 保护判定前置：置位故障后本周期即封波。 */
    DC_Ctrl_CheckBus(&input);
    DC_Ctrl_CheckPv(&input);

    /* 存在故障或母线电压无效时，复位并封波。 */
    if
    (
        (gSysFault.word.recoverable != 0U) ||
        (gSysFault.word.permanent != 0U) ||
        (input.busVoltage <= 0.0f)
    )
    {
        DC_Ctrl_Reset();
        AC_Ctrl_Disable();
        return;
    }

    if (gSysData.state == SYS_STATE_NORMAL)
    {
        /* 并网态：母线环 + Boost 环（MPPT）。 */
        BusVoltRef_Adapt(&input);
        DC_Ctrl_BusRun(&input);
        DC_Ctrl_BoostRun(&input);
        DC_Ctrl_ApplyBoostDuty();
    }
    else if (gSysData.state == SYS_STATE_CHECK)
    {
        /* CHECK 阶段：Boost 软启动建母线（软启动完成后占空比保持）。 */
        DC_Ctrl_BoostSoftStart(&input);
        DC_Ctrl_ApplyBoostDuty();
        AC_Ctrl_Disable();
    }
    else
    {
        /* WAIT/FAULT/PERMANENT：复位封波。 */
        DC_Ctrl_Reset();
        AC_Ctrl_Disable();
    }
}
