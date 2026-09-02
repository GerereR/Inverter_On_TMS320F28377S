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
    float pvPower,
    float pvCurrentLimit,
    float pvMinVoltage,
    float smallPowerDelta,
    float largePowerDelta
);

static void MPPT_LoadModelConfig(void)
{
    if(gSysData.model == MODEL_4KW)
    {
        gMpptData.pv1Config.currentLimit = MODEL_4KW_PV_CUR_LIMIT_A;
        gMpptData.pv1Config.minVoltage = MODEL_4KW_MPPT_MIN_V;
        gMpptData.pv1Config.smallPowerDelta = MODEL_4KW_MPPT_SMALL_DELTA_W;
        gMpptData.pv1Config.largePowerDelta = MODEL_4KW_MPPT_LARGE_DELTA_W;
    }
    else
    {
        gMpptData.pv1Config.currentLimit = MODEL_3KW_PV_CUR_LIMIT_A;
        gMpptData.pv1Config.minVoltage = MODEL_3KW_MPPT_MIN_V;
        gMpptData.pv1Config.smallPowerDelta = MODEL_3KW_MPPT_SMALL_DELTA_W;
        gMpptData.pv1Config.largePowerDelta = MODEL_3KW_MPPT_LARGE_DELTA_W;
    }

    /* Both physical inputs use the same model limits in the current design. */
    gMpptData.pv2Config = gMpptData.pv1Config;
}

void Task_MPPT_Init(void)
{
    MPPT_LoadModelConfig();
    MPPT_ResetChannel(&gMpptData.pv1);
    MPPT_ResetChannel(&gMpptData.pv2);
    gMpptData.inputMode = MPPT_INPUT_NONE;
    gMpptData.masterChannel = 0U;
    gMpptData.topologyStage = 0U;
    gMpptData.topologyCount = 0U;
    gMpptData.powerAvgCount = 0U;
}

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


static void MPPT_UpdateChannel
(
    volatile MpptChannelData *channel,
    float pvVoltage,
    float pvCurrent,
    float pvPower,

    float pvCurrentLimit,
    float pvMinVoltage,
    float smallPowerDelta,
    float largePowerDelta
)
{
    float powerDelta;
    //功率滞缓
    float powerDeadband;
    //PV参考电压扰动步长
    float step;

    if((pvVoltage <= 0.0f) || (pvCurrent <= 0.0f))
    {
        MPPT_ResetChannel(channel);
        return;
    }

    /* Power is calculated once by Task_Measure and shared through MachineData. */
    channel->power = pvPower;

    if(channel->enabled == 0U)
    {
        /* Use the first valid sample as the open-circuit estimate. The real
         * Boost startup code can replace this with a dedicated Voc sample. */
        //重启MPPT时,给初始参考电压和扰动步长
        channel->openCircuitVolt = pvVoltage;
        channel->voltRef = pvVoltage * 0.98f;
        channel->voltStep = pvVoltage * 0.01f;
        if(channel->voltStep < MPPT_VOLT_STEP_V)
        {
            channel->voltStep = MPPT_VOLT_STEP_V;
        }

        channel->voltRefPrev = channel->voltRef;
        channel->powerPrev = channel->power;

        //第一次减小参考电压
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

    //功率明显变大
    else if(powerDelta > powerDeadband)
    {
        /* Power increased: keep perturbing in the same direction. */
        //继续保持原来的扰动方向
        channel->voltRef += (float)channel->direction * step;
    }

    //功率明显变小
    else if(powerDelta < -powerDeadband)
    {
        /* Power decreased: reverse the perturbation direction. */
        //变换方向
        channel->direction = -channel->direction;
        channel->voltRef += (float)channel->direction * step;
    }
    //功率变化不大,理论上很接近MPP
    else
    {
        /* Near the MPP, retain the direction but use a smaller movement. */
        //很妙,它把小步长放在direct上面
        channel->voltRef += (float)channel->direction * MPPT_VOLT_FINE_STEP_V;
    }


    /* More PV current than allowed: raise the voltage reference to reduce
     * current, matching the safety branch in New_Master's MPPT code. */
     //如果电流限制的话,那么增大参考电压值
    if(pvCurrent > (pvCurrentLimit + MPPT_CURRENT_LIMIT_MARGIN_A))
    {
        channel->voltRef += MPPT_VOLT_STEP_V;
    }
    //如果参考值比当前 PV 电压高出 30 V 以上，则拉回当前值附近
    if((pvVoltage > 0.0f) && (channel->voltRef > (pvVoltage + 30.0f)))
    {
        channel->voltRef = pvVoltage - MPPT_VOLT_STEP_V;
    }
    //不能低于机型设定的 MPPT 最低电压
    if(channel->voltRef < pvMinVoltage)
    {
        channel->voltRef = pvMinVoltage;
    }
    //不能高于记录到的开路电压
    if((channel->openCircuitVolt >= pvMinVoltage) && (channel->voltRef > channel->openCircuitVolt))
    {
        channel->voltRef = channel->openCircuitVolt;
    }
    channel->voltRefPrev = channel->voltRef;
    channel->powerPrev = channel->power;
}

void Task_MPPT(void)
{
    /* MPPT is deliberately inactive outside NORMAL. It only updates target
     * PV voltages; the Boost PI consumes those references in Task_DcCtrl(). */
     //我们这个好多了,老代码是直接一个超级大IF
    if(gSysData.state != SYS_STATE_NORMAL)
    {
        MPPT_ResetChannel(&gMpptData.pv1);
        MPPT_ResetChannel(&gMpptData.pv2);
        gMpptData.inputMode = MPPT_INPUT_NONE;
        return;
    }

    MPPT_UpdateChannel
    (
        &gMpptData.pv1,
        gMachineData.realAvg.pv1Voltage,
        gMachineData.realAvg.pv1Current,
        gMachineData.powerData.pv1Power,

        gMpptData.pv1Config.currentLimit,
        gMpptData.pv1Config.minVoltage,
        gMpptData.pv1Config.smallPowerDelta,
        gMpptData.pv1Config.largePowerDelta
    );
    
    MPPT_UpdateChannel
    (
        &gMpptData.pv2,
        gMachineData.realAvg.pv2Voltage,
        gMachineData.realAvg.pv2Current,
        gMachineData.powerData.pv2Power,

        gMpptData.pv2Config.currentLimit,
        gMpptData.pv2Config.minVoltage,
        gMpptData.pv2Config.smallPowerDelta,
        gMpptData.pv2Config.largePowerDelta
    );

    if((gMpptData.pv1.enabled != 0U) && (gMpptData.pv2.enabled != 0U))
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
