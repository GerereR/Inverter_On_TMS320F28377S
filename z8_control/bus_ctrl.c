#include "F28x_Project.h"

#include "control.h"
#include "system.h"
#include "variable.h"

void Ctrl_BusReset(void)
{
    gBusCtrlData.voltErr = 0.0f;
    gBusCtrlData.voltErrPrev = 0.0f;
    gBusCtrlData.piIntegral = 0.0f;
    gBusCtrlData.piOut = 0.0f;
    gBusCtrlData.piOutPrev = 0.0f;
    gBusCtrlData.currentAmpRef = 0.0f;
    gBusCtrlData.initialized = 0U;
}

void Ctrl_BusRun(float busVoltage, float currentAmpLimit)
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

    /* A high bus voltage means excess energy is accumulating. Therefore a
     * positive error must increase exported inverter current. */
    error = busVoltage - gBusCtrlData.stableVoltRef;
    gBusCtrlData.voltErrPrev = gBusCtrlData.voltErr;
    gBusCtrlData.voltErr = error;

    proportional = BUS_PI_KP * error;
    candidate = proportional + gBusCtrlData.piIntegral;

    /* Freeze the integrator only when it would drive an already saturated
     * output farther into saturation. */
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
