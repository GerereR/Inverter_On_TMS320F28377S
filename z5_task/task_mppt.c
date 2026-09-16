#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"
#include "variable.h"

typedef struct
{
    float volt;
    float curr;
    float power;
    float currLim;
    float minVolt;
    float smallPowerDelta;
    float largePowerDelta;
} MPPT_Input;

typedef struct
{
    float voltRef;
    float openCircuitVolt;
    float power;
    float powerPrev;
    float voltStep;
    int16 direction;
    Uint16 enabled;
    Uint16 fastSearch;
} MPPT_ChState;

typedef struct
{
    float currLim;
    float minVolt;
    float smallPowerDelta;
    float largePowerDelta;
} MPPT_ChConfig;

typedef struct
{
    MPPT_ChState pv1;
    MPPT_ChState pv2;
    MPPT_ChConfig config;
} MPPT_State;

static MPPT_State MPPT_StateData = {0};

static void MPPT_PublishOutputs(void)
{
    gMpptData.pv1.voltRef = MPPT_StateData.pv1.voltRef;
    gMpptData.pv1.enabled = MPPT_StateData.pv1.enabled;
    gMpptData.pv2.voltRef = MPPT_StateData.pv2.voltRef;
    gMpptData.pv2.enabled = MPPT_StateData.pv2.enabled;
}

static void MPPT_ResetCh(MPPT_ChState *channel);

static void MPPT_UpdateCh
(
    MPPT_ChState *channel,
    const MPPT_Input *input
);

static void MPPT_LoadModelConfig(void)
{
    if(gSysData.model == MODEL_4KW)
    {
        MPPT_StateData.config.currLim = MODEL_4KW_PV_CUR_LIM_A;
        MPPT_StateData.config.minVolt = MODEL_4KW_MPPT_MIN_V;
        MPPT_StateData.config.smallPowerDelta = MODEL_4KW_MPPT_SMALL_DELTA_W;
        MPPT_StateData.config.largePowerDelta = MODEL_4KW_MPPT_LARGE_DELTA_W;
    }
    else
    {
        MPPT_StateData.config.currLim = MODEL_3KW_PV_CUR_LIM_A;
        MPPT_StateData.config.minVolt = MODEL_3KW_MPPT_MIN_V;
        MPPT_StateData.config.smallPowerDelta = MODEL_3KW_MPPT_SMALL_DELTA_W;
        MPPT_StateData.config.largePowerDelta = MODEL_3KW_MPPT_LARGE_DELTA_W;
    }
}

void Task_MPPT_Init(void)
{
    MPPT_LoadModelConfig();
    MPPT_ResetCh(&MPPT_StateData.pv1);
    MPPT_ResetCh(&MPPT_StateData.pv2);
    MPPT_PublishOutputs();
    gMpptData.inputMode = MPPT_INPUT_NONE;
}

static void MPPT_ResetCh(MPPT_ChState *channel)
{
    channel->voltRef = 0.0f;
    channel->openCircuitVolt = 0.0f;
    channel->power = 0.0f;
    channel->powerPrev = 0.0f;
    channel->voltStep = MPPT_VOLT_STEP_V;
    channel->direction = -1;
    channel->enabled = 0U;
    channel->fastSearch = 1U;
}


static void MPPT_UpdateCh
(
    MPPT_ChState *channel,
    const MPPT_Input *input
)
{
    float powerDelta;
    //功率滞缓
    float powerDeadband;
    //PV参考电压扰动步长
    float step;

    if((input->volt <= 0.0f) || (input->curr <= 0.0f))
    {
        MPPT_ResetCh(channel);
        return;
    }

    /* Power is calculated once by Task_Measu and shared through MachineData. */
    channel->power = input->power;

    if(channel->enabled == 0U)
    {
        /* Use the first valid sample as the open-circuit estimate. The real
         * Boost startup code can replace this with a dedicated Voc sample. */
        //重启MPPT时,给初始参考电压和扰动步长
        channel->openCircuitVolt = input->volt;
        channel->voltRef = input->volt * 0.98f;
        channel->voltStep = input->volt * 0.01f;
        if(channel->voltStep < MPPT_VOLT_STEP_V)
        {
            channel->voltStep = MPPT_VOLT_STEP_V;
        }

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

    powerDeadband = (channel->fastSearch != 0U) ?
                    input->largePowerDelta : input->smallPowerDelta;
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


    /* More PV curr than allowed: raise the volt reference to reduce
     * curr, matching the safety branch in New_Master's MPPT code. */
     //如果电流限制的话,那么增大参考电压值
    if(input->curr > (input->currLim + MPPT_CURR_LIM_MARGIN_A))
    {
        channel->voltRef += MPPT_VOLT_STEP_V;
    }
    //如果参考值比当前 PV 电压高出 30 V 以上，则拉回当前值附近
    if((input->volt > 0.0f) &&
       (channel->voltRef > (input->volt + 30.0f)))
    {
        channel->voltRef = input->volt - MPPT_VOLT_STEP_V;
    }
    //不能低于机型设定的 MPPT 最低电压
    if(channel->voltRef < input->minVolt)
    {
        channel->voltRef = input->minVolt;
    }
    //不能高于记录到的开路电压
    if((channel->openCircuitVolt >= input->minVolt) &&
       (channel->voltRef > channel->openCircuitVolt))
    {
        channel->voltRef = channel->openCircuitVolt;
    }
    channel->powerPrev = channel->power;
}

void Task_MPPT(void)
{
    MPPT_Input pv1Input;
    MPPT_Input pv2Input;

    /* MPPT is deliberately inactive outside NORMAL. It only updates target
     * PV volts; the Boost PI consumes those references in Task_DC_Ctrl(). */
     //我们这个好多了,老代码是直接一个超级大IF
    if(gSysData.state != SYS_STATE_NORMAL)
    {
        MPPT_ResetCh(&MPPT_StateData.pv1);
        MPPT_ResetCh(&MPPT_StateData.pv2);
        MPPT_PublishOutputs();
        gMpptData.inputMode = MPPT_INPUT_NONE;
        return;
    }

    pv1Input.volt = gMachineData.realAvg.pv1Volt;
    pv1Input.curr = gMachineData.realAvg.pv1Curr;
    pv1Input.power = gMachineData.powerData.pv1Power;
    pv1Input.currLim = MPPT_StateData.config.currLim;
    pv1Input.minVolt = MPPT_StateData.config.minVolt;
    pv1Input.smallPowerDelta = MPPT_StateData.config.smallPowerDelta;
    pv1Input.largePowerDelta = MPPT_StateData.config.largePowerDelta;

    pv2Input.volt = gMachineData.realAvg.pv2Volt;
    pv2Input.curr = gMachineData.realAvg.pv2Curr;
    pv2Input.power = gMachineData.powerData.pv2Power;
    pv2Input.currLim = MPPT_StateData.config.currLim;
    pv2Input.minVolt = MPPT_StateData.config.minVolt;
    pv2Input.smallPowerDelta = MPPT_StateData.config.smallPowerDelta;
    pv2Input.largePowerDelta = MPPT_StateData.config.largePowerDelta;

    MPPT_UpdateCh(&MPPT_StateData.pv1, &pv1Input);
    MPPT_UpdateCh(&MPPT_StateData.pv2, &pv2Input);

    MPPT_PublishOutputs();

    if((MPPT_StateData.pv1.enabled != 0U) && (MPPT_StateData.pv2.enabled != 0U))
    {
        gMpptData.inputMode = MPPT_INPUT_DUAL;
    }
    else if(MPPT_StateData.pv1.enabled != 0U)
    {
        gMpptData.inputMode = MPPT_INPUT_PV1_ONLY;
    }
    else if(MPPT_StateData.pv2.enabled != 0U)
    {
        gMpptData.inputMode = MPPT_INPUT_PV2_ONLY;
    }
    else
    {
        gMpptData.inputMode = MPPT_INPUT_NONE;
    }
}
