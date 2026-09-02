#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"
#include "variable.h"

static void MPPT_ResetChannel(volatile MpptChannelData *channel);

static void MPPT_UpdateChannel
(
    volatile MpptChannelData *channel,
    float pvVoltage,
    float pvCurrent,
    float pvCurrentLimit,
    float pvMinVoltage,
    float smallPowerDelta,
    float largePowerDelta
);

static float MPPT_ClampReference
(
    float reference,
    float pvVoltage,
    float openCircuitVoltage,
    float minimumVoltage
);

static void MPPT_ResetChannel(volatile MpptChannelData *channel)
{
    channel->voltRef = 0.0f;
    channel->voltRefPrev = 0.0f;
    channel->openCircuitVolt = 0.0f;
    channel->power = 0.0f;
    channel->powerPrev = 0.0f;
    channel->voltStep = MPPT_VOLT_STEP_V;
    channel->duty = 0.0f;
    channel->dutyPrev = 0.0f;
    channel->direction = -1;
    channel->enabled = 0U;
    channel->fastSearch = 1U;
}

static float MPPT_ClampReference
(
    float reference,
    float pvVoltage,
    float openCircuitVoltage,
    float minimumVoltage
)
{
    float maximumVoltage = openCircuitVoltage;

    /* The old tracker quickly pulls an obsolete reference back below the
     * measured PV voltage after a source change. */
    if((pvVoltage > 0.0f) && (reference > (pvVoltage + 30.0f)))
    {
        reference = pvVoltage - MPPT_VOLT_STEP_V;
    }

    if(reference < minimumVoltage)
    {
        reference = minimumVoltage;
    }

    /* Do not request a voltage above the last known open-circuit voltage. */
    if(maximumVoltage >= minimumVoltage)
    {
        if(reference > maximumVoltage)
        {
            reference = maximumVoltage;
        }
    }

    return reference;
}

static void MPPT_UpdateChannel
(
    volatile MpptChannelData *channel,
    float pvVoltage,
    float pvCurrent,
    float pvCurrentLimit,
    float pvMinVoltage,
    float smallPowerDelta,
    float largePowerDelta
)
{
    float powerDelta;
    float powerDeadband;
    float step;

    if((pvVoltage <= 0.0f) || (pvCurrent <= 0.0f))
    {
        MPPT_ResetChannel(channel);
        return;
    }

    channel->power = pvVoltage * pvCurrent;

    if(channel->enabled == 0U)
    {
        /* Use the first valid sample as the open-circuit estimate. The real
         * Boost startup code can replace this with a dedicated Voc sample. */
        channel->openCircuitVolt = pvVoltage;
        channel->voltRef = pvVoltage * 0.98f;
        channel->voltRefPrev = channel->voltRef;
        channel->voltStep = pvVoltage * 0.01f;
        if(channel->voltStep < MPPT_VOLT_STEP_V)
        {
            channel->voltStep = MPPT_VOLT_STEP_V;
        }
        channel->powerPrev = channel->power;
        channel->direction = -1;
        channel->fastSearch = 1U;
        channel->enabled = 1U;
        return;
    }

    step = channel->voltStep;
    if(step < MPPT_VOLT_STEP_V)
    {
        step = MPPT_VOLT_STEP_V;
    }

    powerDeadband = (channel->fastSearch != 0U) ? largePowerDelta : smallPowerDelta;
    powerDelta = channel->power - channel->powerPrev;

    if(channel->fastSearch != 0U)
    {
        /* Start below Voc and move down quickly while power is improving. */
        if(powerDelta > MPPT_FAST_POWER_DELTA_W)
        {
            channel->voltRef -= step;
        }
        else
        {
            channel->voltRef += MPPT_VOLT_STEP_V;
            channel->fastSearch = 0U;
        }
    }
    else if(powerDelta > powerDeadband)
    {
        /* Power increased: keep perturbing in the same direction. */
        channel->voltRef += (float)channel->direction * step;
    }
    else if(powerDelta < -powerDeadband)
    {
        /* Power decreased: reverse the perturbation direction. */
        channel->direction = -channel->direction;
        channel->voltRef += (float)channel->direction * step;
    }
    else
    {
        /* Near the MPP, retain the direction but use a smaller movement. */
        channel->voltRef += (float)channel->direction * MPPT_VOLT_FINE_STEP_V;
    }

    /* More PV current than allowed: raise the voltage reference to reduce
     * current, matching the safety branch in New_Master's MPPT code. */
    if(pvCurrent > (pvCurrentLimit + MPPT_CURRENT_LIMIT_MARGIN_A))
    {
        channel->voltRef += MPPT_VOLT_STEP_V;
    }

    channel->voltRef = MPPT_ClampReference(channel->voltRef, pvVoltage, channel->openCircuitVolt, pvMinVoltage);
    channel->voltRefPrev = channel->voltRef;
    channel->powerPrev = channel->power;
}

void Task_MPPT(void)
{
    float pv1CurrentLimit;
    float pv1MinVoltage;
    float pv1SmallDelta;
    float pv1LargeDelta;
    float pv2CurrentLimit;
    float pv2MinVoltage;
    float pv2SmallDelta;
    float pv2LargeDelta;

    /* MPPT is deliberately inactive outside NORMAL. It only updates target
     * PV voltages; the Boost PI consumes those references in Task_DcCtrl(). */
    if(gSysData.state != SYS_STATE_NORMAL)
    {
        MPPT_ResetChannel(&gMpptData.pv1);
        MPPT_ResetChannel(&gMpptData.pv2);
        gMpptData.inputMode = MPPT_INPUT_NONE;
        return;
    }

    if(gSysData.model == MODEL_4KW)
    {
        pv1CurrentLimit = MODEL_4KW_PV_CUR_LIMIT_A;
        pv1MinVoltage = MODEL_4KW_MPPT_MIN_V;
        pv1SmallDelta = MODEL_4KW_MPPT_SMALL_DELTA_W;
        pv1LargeDelta = MODEL_4KW_MPPT_LARGE_DELTA_W;
        pv2CurrentLimit = MODEL_4KW_PV_CUR_LIMIT_A;
        pv2MinVoltage = MODEL_4KW_MPPT_MIN_V;
        pv2SmallDelta = MODEL_4KW_MPPT_SMALL_DELTA_W;
        pv2LargeDelta = MODEL_4KW_MPPT_LARGE_DELTA_W;
    }
    else
    {
        pv1CurrentLimit = MODEL_3KW_PV_CUR_LIMIT_A;
        pv1MinVoltage = MODEL_3KW_MPPT_MIN_V;
        pv1SmallDelta = MODEL_3KW_MPPT_SMALL_DELTA_W;
        pv1LargeDelta = MODEL_3KW_MPPT_LARGE_DELTA_W;
        pv2CurrentLimit = MODEL_3KW_PV_CUR_LIMIT_A;
        pv2MinVoltage = MODEL_3KW_MPPT_MIN_V;
        pv2SmallDelta = MODEL_3KW_MPPT_SMALL_DELTA_W;
        pv2LargeDelta = MODEL_3KW_MPPT_LARGE_DELTA_W;
    }

    MPPT_UpdateChannel(&gMpptData.pv1,
                       gMachineData.realAvg.pv1Voltage,
                       gMachineData.realAvg.pv1Current,
                       pv1CurrentLimit,
                       pv1MinVoltage,
                       pv1SmallDelta,
                       pv1LargeDelta);
    MPPT_UpdateChannel(&gMpptData.pv2,
                       gMachineData.realAvg.pv2Voltage,
                       gMachineData.realAvg.pv2Current,
                       pv2CurrentLimit,
                       pv2MinVoltage,
                       pv2SmallDelta,
                       pv2LargeDelta);

    if((gMpptData.pv1.enabled != 0U) &&
       (gMpptData.pv2.enabled != 0U))
    {
        gMpptData.inputMode = MPPT_INPUT_DUAL;
    }
    else if(gMpptData.pv1.enabled != 0U)
    {
        gMpptData.inputMode = MPPT_INPUT_PV1_ONLY;
    }
    else if(gMpptData.pv2.enabled != 0U)
    {
        gMpptData.inputMode = MPPT_INPUT_PV2_ONLY;
    }
    else
    {
        gMpptData.inputMode = MPPT_INPUT_NONE;
    }
}
