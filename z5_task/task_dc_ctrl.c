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

/* 母线/PV 保护滤波计数（峰值触发约 10ms 一拍）。 */
#define BUS_OV_FILTER_COUNT              100U    /* 母线过压连续判定（约 1s） */
#define BUS_UV_FILTER_COUNT              500U    /* 母线欠压连续判定（约 5s，仅并网态） */
#define PV_OV_FILTER_COUNT                30U    /* PV 过压连续判定（约 300ms） */
#define PV_OV_RECOVER_COUNT               30U    /* PV 过压恢复连续正常 */

/*============================================================================
 * 直流控制任务（峰值触发）：母线电压外环 + 双路 Boost 环。
 * 原 z8_control/bus_ctrl.c + boost_ctrl.c 的实现已内联至此。
 * 母线环输出电流幅值指令给电流环（task_fast），Boost 环输出占空比给 PWM。
 *==========================================================================*/

/* --- 母线电压外环（原 Ctrl_BusRun） --- */
static void DC_Ctrl_BusRun(float busVoltage, float currentAmpLimit)
{
    float proportional;
    float candidate;
    float error;

    if(gBusCtrlData.initialized == 0U)
    {
        gBusCtrlData.voltRef = BUS_VOLT_REF_V;
        gBusCtrlData.stableVoltRef = BUS_VOLT_REF_V;
        gBusCtrlData.softStartVoltRef = BUS_VOLT_REF_V;
        gBusCtrlData.voltErr = 0.0f;
        gBusCtrlData.voltErrPrev = 0.0f;
        gBusCtrlData.piIntegral = 0.0f;
        gBusCtrlData.piOut = 0.0f;
        gBusCtrlData.piOutPrev = 0.0f;
        gBusCtrlData.currentAmpRef = 0.0f;
        gBusCtrlData.initialized = 1U;
    }

    currentAmpLimit = System_Clamp(currentAmpLimit,
                                   BUS_CURRENT_AMP_MIN_NORM,
                                   BUS_CURRENT_AMP_MAX_NORM);

    error = busVoltage - gBusCtrlData.stableVoltRef;
    gBusCtrlData.voltErrPrev = gBusCtrlData.voltErr;
    gBusCtrlData.voltErr = error;

    proportional = BUS_PI_KP * error;

    /* 积分反算（back-calculation）抗饱和：先正常累加积分，
     * 再用 clamp 后的输出反算积分，把积分锚定在限幅处。
     * 输出被限流上限压住时，积分自动收缩，不会持续累积（饱和）。 */
    gBusCtrlData.piIntegral += BUS_PI_KI * BUS_CTRL_PERIOD_S * error;

    candidate = proportional + gBusCtrlData.piIntegral;
    gBusCtrlData.piOutPrev = gBusCtrlData.piOut;
    gBusCtrlData.piOut = System_Clamp(candidate,
                                      BUS_CURRENT_AMP_MIN_NORM,
                                      currentAmpLimit);

    /* 反算：piIntegral = clamp后输出 - 比例项，锚定到限幅处。 */
    gBusCtrlData.piIntegral = gBusCtrlData.piOut - proportional;

    gBusCtrlData.currentAmpRef = gBusCtrlData.piOut;
}

/* --- Boost 环（原 Ctrl_BoostUpdateChannel / Ctrl_BoostRun） --- */
static float DC_Ctrl_BoostUpdateChannel
(
    volatile float *voltError,
    volatile float *piIntegral,
    float pvVoltage,
    float pvVoltageRef
)
{
    float error;
    float proportional;
    float candidate;

    error = pvVoltage - pvVoltageRef;
    *voltError = error;
    proportional = BOOST_PI_KP * error;
    candidate = proportional + *piIntegral;

    if(!(((candidate >= BOOST_DUTY_MAX) && (error > 0.0f)) ||
         ((candidate <= BOOST_DUTY_MIN) && (error < 0.0f))))
    {
        *piIntegral += BOOST_PI_KI * BOOST_CTRL_PERIOD_S * error;
    }

    candidate = proportional + *piIntegral;
    return System_Clamp(candidate, BOOST_DUTY_MIN, BOOST_DUTY_MAX);
}

static void DC_Ctrl_BoostRun
(
    float pv1Voltage,
    float pv1VoltageRef,
    Uint16 pv1Enabled,
    float pv2Voltage,
    float pv2VoltageRef,
    Uint16 pv2Enabled
)
{
    if((pv1Enabled == 0U) && (pv2Enabled == 0U))
    {
        DC_Ctrl_BoostResetChannels();
        return;
    }

    if((pv1Enabled != 0U) && (pv1Voltage > 0.0f) && (pv1VoltageRef >= PV_PRESENT_MIN_V))
    {
        gBusCtrlData.boost1Duty = DC_Ctrl_BoostUpdateChannel(
            &gBusCtrlData.boost1VoltErr, &gBusCtrlData.boost1PiIntegral,
            pv1Voltage, pv1VoltageRef);
    }
    else
    {
        gBusCtrlData.boost1VoltErr = 0.0f;
        gBusCtrlData.boost1PiIntegral = 0.0f;
        gBusCtrlData.boost1Duty = 0.0f;
    }

    if((pv2Enabled != 0U) && (pv2Voltage > 0.0f) && (pv2VoltageRef >= PV_PRESENT_MIN_V))
    {
        gBusCtrlData.boost2Duty = DC_Ctrl_BoostUpdateChannel(
            &gBusCtrlData.boost2VoltErr, &gBusCtrlData.boost2PiIntegral,
            pv2Voltage, pv2VoltageRef);
    }
    else
    {
        gBusCtrlData.boost2VoltErr = 0.0f;
        gBusCtrlData.boost2PiIntegral = 0.0f;
        gBusCtrlData.boost2Duty = 0.0f;
    }
}

static void DC_Ctrl_BoostResetChannels(void)
{
    gBusCtrlData.boost1VoltErr = 0.0f;
    gBusCtrlData.boost2VoltErr = 0.0f;
    gBusCtrlData.boost1PiIntegral = 0.0f;
    gBusCtrlData.boost2PiIntegral = 0.0f;
    gBusCtrlData.boost1Duty = 0.0f;
    gBusCtrlData.boost2Duty = 0.0f;
}


/* 对外：复位母线环 + Boost（供状态机 State_ResetStartupData 调用）。 */
void DC_Ctrl_Reset(void)
{
    gBusCtrlData.voltErr = 0.0f;
    gBusCtrlData.voltErrPrev = 0.0f;
    gBusCtrlData.piIntegral = 0.0f;
    gBusCtrlData.piOut = 0.0f;
    gBusCtrlData.piOutPrev = 0.0f;
    gBusCtrlData.currentAmpRef = 0.0f;
    gBusCtrlData.initialized = 0U;
}

/* Boost 软启动（原 sStartBoostControl）：占空比从 0 平滑爬升，把母线从 PV
 * 电压泵到最低目标。母线达标后停止爬升、保持占空比，等并网后 MPPT 接管。 */
static void DC_Ctrl_BoostSoftStart(float busVoltage)
{
    if (gBusCtrlData.softStartActive == 0U)
    {
        return;
    }
    
    gBusCtrlData.boost1Duty += BOOST_SOFT_START_STEP; 
    if (gBusCtrlData.boost1Duty > BOOST_DUTY_MAX)
    {
        gBusCtrlData.boost1Duty = BOOST_DUTY_MAX;
    } 

    gBusCtrlData.boost2Duty += BOOST_SOFT_START_STEP;
    if (gBusCtrlData.boost2Duty > BOOST_DUTY_MAX)
    {
        gBusCtrlData.boost2Duty = BOOST_DUTY_MAX;
    } 

    //我自己加的5V,保险一点
    if (busVoltage >= DC_BUS_MIN_V + 5.0f)
    {
        gBusCtrlData.softStartActive = 0U;
    }
}

/* 母线过压保护（原 VBUSCheck 过压）+ 母线欠压打嗝：
 * 过压任何态判、置 permanent 故障（Boost 失控/硬件损坏不可恢复）；
 * 欠压仅并网态判、置打嗝标志（reloadFlag，重新软启动，不停机）。 */
static void DC_Ctrl_CheckBus(float busVoltage)
{
    static Uint16 busOvpFilter = 0U;
    static Uint16 busUvpFilter = 0U;

    if (busVoltage > DC_BUS_OV_TRIP_V)
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
        if ((busVoltage < DC_BUS_MIN_V) && (busVoltage > 0.0f))
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
static void DC_Ctrl_CheckPv(float pv1Voltage, float pv2Voltage)
{
    static Uint16 pv1OvpFilter = 0U;
    static Uint16 pv2OvpFilter = 0U;
    static Uint16 pv1OvpBackFilter = 0U;
    static Uint16 pv2OvpBackFilter = 0U;

    if (pv1Voltage > PV_OV_TRIP_V)
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


    if (pv2Voltage > PV_OV_TRIP_V)
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
        if (pv1Voltage < PV_OV_RECOVER_V)
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
        if (pv2Voltage < PV_OV_RECOVER_V)
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
    float busVoltage;
    float pv1Voltage;
    float pv2Voltage;

    busVoltage = gMachineData.realAvg.dcBusVoltage;
    pv1Voltage = gMachineData.realAvg.pv1Voltage;
    pv2Voltage = gMachineData.realAvg.pv2Voltage;

    /* 保护判定前置：置位故障后本周期即封波。 */
    DC_Ctrl_CheckBus(busVoltage);
    DC_Ctrl_CheckPv(pv1Voltage, pv2Voltage);

    /* 存在故障或母线电压无效时，复位并封波。 */
    if
    (
        (gSysFault.word.recoverable != 0U) ||
        (gSysFault.word.permanent != 0U) ||
        (busVoltage <= 0.0f)
    )
    {
        DC_Ctrl_Reset();
        
        DC_Ctrl_BoostResetChannels();
        EPWM_SetBoostDuty(0.0f, 0.0f);

        AC_Ctrl_Disable();
        return;
    }

    if (gSysData.state == SYS_STATE_NORMAL)
    {
        /* 并网态：母线环 + Boost 环（MPPT）。 */
        DC_Ctrl_BusRun(busVoltage, gPowerLimitData.currentAmpLimit);
        DC_Ctrl_BoostRun
        (
            pv1Voltage, 
            gMpptData.pv1.voltRef, 
            gMpptData.pv1.enabled,
            pv2Voltage, 
            gMpptData.pv2.voltRef, 
            gMpptData.pv2.enabled
        );
        EPWM_SetBoostDuty(gBusCtrlData.boost1Duty, gBusCtrlData.boost2Duty);
    }
    else if (gSysData.state == SYS_STATE_CHECK)
    {
        /* CHECK 阶段：Boost 软启动建母线（软启动完成后占空比保持）。 */
        DC_Ctrl_BoostSoftStart(busVoltage);
        EPWM_SetBoostDuty(gBusCtrlData.boost1Duty, gBusCtrlData.boost2Duty);
        AC_Ctrl_Disable();
    }
    else
    {
        /* WAIT/FAULT/PERMANENT：复位封波。 */
        DC_Ctrl_Reset();

        DC_Ctrl_BoostResetChannels();
        EPWM_SetBoostDuty(0.0f, 0.0f);

        AC_Ctrl_Disable();
    }
}
