#include "F28x_Project.h"

#include "control.h"
#include "system.h"
#include "variable.h"

static float Ctrl_BoostUpdateChannel
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

    /* A PV voltage above its reference requires more Boost duty. Freeze the
     * integrator only when that same action would exceed a duty limit. */
    if(!(((candidate >= BOOST_DUTY_MAX) && (error > 0.0f)) ||
         ((candidate <= BOOST_DUTY_MIN) && (error < 0.0f))))
    {
        *piIntegral += BOOST_PI_KI * BOOST_CTRL_PERIOD_S * error;
    }

    candidate = proportional + *piIntegral;
    return System_Clamp(candidate, BOOST_DUTY_MIN, BOOST_DUTY_MAX);
}

void Ctrl_BoostReset(void)
{
    gBusCtrlData.boost1VoltErr = 0.0f;
    gBusCtrlData.boost2VoltErr = 0.0f;
    gBusCtrlData.boost1PiIntegral = 0.0f;
    gBusCtrlData.boost2PiIntegral = 0.0f;
    gBusCtrlData.boost1Duty = 0.0f;
    gBusCtrlData.boost2Duty = 0.0f;
}

void Ctrl_BoostRun(float pv1Voltage,
                   float pv1VoltageRef,
                   Uint16 pv1Enabled,
                   float pv2Voltage,
                   float pv2VoltageRef,
                   Uint16 pv2Enabled)
{
    /* MPPT owns the voltage references. Until a channel has a valid
     * reference, keep its Boost switch off instead of driving toward zero. */
    if((pv1Enabled == 0U) && (pv2Enabled == 0U))
    {
        Ctrl_BoostReset();
        return;
    }

    if((pv1Enabled != 0U) &&
       (pv1Voltage > 0.0f) &&
       (pv1VoltageRef >= PV_PRESENT_MIN_V))
    {
        gBusCtrlData.boost1Duty = Ctrl_BoostUpdateChannel(
            &gBusCtrlData.boost1VoltErr,
            &gBusCtrlData.boost1PiIntegral,
            pv1Voltage,
            pv1VoltageRef);
    }
    else
    {
        gBusCtrlData.boost1VoltErr = 0.0f;
        gBusCtrlData.boost1PiIntegral = 0.0f;
        gBusCtrlData.boost1Duty = 0.0f;
    }

    if((pv2Enabled != 0U) &&
       (pv2Voltage > 0.0f) &&
       (pv2VoltageRef >= PV_PRESENT_MIN_V))
    {
        gBusCtrlData.boost2Duty = Ctrl_BoostUpdateChannel(
            &gBusCtrlData.boost2VoltErr,
            &gBusCtrlData.boost2PiIntegral,
            pv2Voltage,
            pv2VoltageRef);
    }
    else
    {
        gBusCtrlData.boost2VoltErr = 0.0f;
        gBusCtrlData.boost2PiIntegral = 0.0f;
        gBusCtrlData.boost2Duty = 0.0f;
    }
}
