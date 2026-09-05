#include "F28x_Project.h"

#include "task.h"
#include "bsp.h"
#include "variable.h"
#include "system.h"

/*============================================================================
 * 直流控制任务（峰值触发）：母线电压外环 + 双路 Boost 环。
 * 原 z8_control/bus_ctrl.c + boost_ctrl.c 的实现已内联至此。
 * 母线环输出电流幅值指令给电流环（task_fast），Boost 环输出占空比给 PWM。
 *==========================================================================*/

/* --- 母线电压外环（原 Ctrl_BusRun） --- */
static void DcCtrl_BusRun(float busVoltage, float currentAmpLimit)
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
    candidate = proportional + gBusCtrlData.piIntegral;

    if(!(((candidate >= currentAmpLimit) && (error > 0.0f)) ||
         ((candidate <= BUS_CURRENT_AMP_MIN_NORM) && (error < 0.0f))))
    {
        gBusCtrlData.piIntegral += BUS_PI_KI * BUS_CTRL_PERIOD_S * error;
    }

    candidate = proportional + gBusCtrlData.piIntegral;
    gBusCtrlData.piOutPrev = gBusCtrlData.piOut;
    gBusCtrlData.piOut = System_Clamp(candidate,
                                      BUS_CURRENT_AMP_MIN_NORM,
                                      currentAmpLimit);
    gBusCtrlData.currentAmpRef = gBusCtrlData.piOut;
}

/* --- Boost 环（原 Ctrl_BoostUpdateChannel / Ctrl_BoostRun） --- */
static float DcCtrl_BoostUpdateChannel(volatile float *voltError,
                                       volatile float *piIntegral,
                                       float pvVoltage,
                                       float pvVoltageRef)
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

static void DcCtrl_BoostResetChannels(void)
{
    gBusCtrlData.boost1VoltErr = 0.0f;
    gBusCtrlData.boost2VoltErr = 0.0f;
    gBusCtrlData.boost1PiIntegral = 0.0f;
    gBusCtrlData.boost2PiIntegral = 0.0f;
    gBusCtrlData.boost1Duty = 0.0f;
    gBusCtrlData.boost2Duty = 0.0f;
}

static void DcCtrl_BoostRun(float pv1Voltage,
                            float pv1VoltageRef,
                            Uint16 pv1Enabled,
                            float pv2Voltage,
                            float pv2VoltageRef,
                            Uint16 pv2Enabled)
{
    if((pv1Enabled == 0U) && (pv2Enabled == 0U))
    {
        DcCtrl_BoostResetChannels();
        return;
    }

    if((pv1Enabled != 0U) && (pv1Voltage > 0.0f) && (pv1VoltageRef >= PV_PRESENT_MIN_V))
    {
        gBusCtrlData.boost1Duty = DcCtrl_BoostUpdateChannel(
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
        gBusCtrlData.boost2Duty = DcCtrl_BoostUpdateChannel(
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

/* 对外：复位母线环 + Boost（供状态机 State_ResetStartupData 调用）。 */
void DcCtrl_Reset(void)
{
    gBusCtrlData.voltErr = 0.0f;
    gBusCtrlData.voltErrPrev = 0.0f;
    gBusCtrlData.piIntegral = 0.0f;
    gBusCtrlData.piOut = 0.0f;
    gBusCtrlData.piOutPrev = 0.0f;
    gBusCtrlData.currentAmpRef = 0.0f;
    gBusCtrlData.initialized = 0U;

    DcCtrl_BoostResetChannels();
}

/* --- 任务编排（峰值触发） --- */
void Task_DcCtrl(void)
{
    float busVoltage;

    busVoltage = gMachineData.realAvg.dcBusVoltage;

    /* 不在并网态、存在故障、或母线电压无效时，复位并封波。 */
    if((gSysData.state != SYS_STATE_NORMAL) ||
       (gSysFault.word.recoverable != 0U) ||
       (gSysFault.word.permanent != 0U) ||
       (busVoltage <= 0.0f))
    {
        DcCtrl_Reset();
        EPWM_SetBoostDuty(0.0f, 0.0f);
        Fast_SetCurrentAmp(0.0f);
        return;
    }

    /* 母线环：母线电压误差 -> 电流幅值指令。 */
    DcCtrl_BusRun(busVoltage, gPowerLimitData.currentAmpLimit);

    /* 把母线环结果交给电流环（task_fast）。 */
    Fast_SetCurrentAmp(gBusCtrlData.currentAmpRef);

    /* Boost 环：PV 电压误差 -> 占空比。 */
    DcCtrl_BoostRun(gMachineData.realAvg.pv1Voltage, gMpptData.pv1.voltRef, gMpptData.pv1.enabled,
                    gMachineData.realAvg.pv2Voltage, gMpptData.pv2.voltRef, gMpptData.pv2.enabled);

    EPWM_SetBoostDuty(gBusCtrlData.boost1Duty, gBusCtrlData.boost2Duty);
}
