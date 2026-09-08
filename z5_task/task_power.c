#include "F28x_Project.h"

#include "task.h"
#include "constant.h"
#include "variable.h"
#include "system.h"

/* --- 功率限幅常量（原 powercalc.c / power_mgr.c，按机型配置，当前为默认值） ---
 * 单位：功率 W、温度 ℃、电压 V、频率 Hz；电流上限归一化 0..1。
 * 数值为 bring-up 初值，上机后需按实际机型/校准/收敛速度调。 */
#define POWER_OVERLOAD_W                 3300.0f   /* 过载功率上限 */
#define POWER_RATED_W                    3000.0f   /* 额定功率（过载降档目标） */

/* 温度降额 */
#define POWER_TEMP_DERATE_START_C        65.0f     /* 温度降额起始点（℃） */
#define POWER_TEMP_DERATE_RATE_W_PER_C   40.0f     /* 温度降额斜率（W/℃） */
#define POWER_TEMP_DERATE_DELAY_MS       5000U     /* 超阈值持续多久才降额（迟滞） */

/* PV 过压降额 */
#define POWER_PV_DERATE_START_V          500.0f    /* PV 过压降额起始（V） */
#define POWER_PV_DERATE_SLOPE_W_PER_V    8.0f      /* PV 降额斜率（W/V） */

/* 过载管理 */
#define POWER_OVERLOAD_TIME_MS           3000U     /* 超额定持续多久才降档 */
#define POWER_OVERLOAD_BACK_TIME_MS      30000U    /* 回落多久才升档（迟滞） */

/* 频率降额 + 无功调度上限：见下方 Power_FreqDerate / Power_ReactiveDerate 接口，
 * 待安规/国家码接入时实现，当前不降额。 */

/* 功率环积分增益：老代码 500ms 一次 amp_limit += delta_watt/7500；
 * 本工程 Task_Power 10ms 一次，等效增益 = 7500*(500/10) = 375000。 */
#define POWER_INT_GAIN_W                 375000.0f

/* 功率任务私有状态（不暴露给外部，外部只读 gPowerLimitData 里的结果）。 */
typedef struct
{
    Uint16 overloadTimerMs;    /* 过载持续计时 */
    Uint16 overloadActive;     /* 过载降档激活（1 = 当前限额定功率） */
    Uint16 overloadBackMs;     /* 过载恢复计时 */
    Uint16 tempDerateTimerMs;  /* 温度降额迟滞计时 */
} PowerState;

static PowerState Power_State = {0};

/* 功率任务的输入快照（慢任务串行，快照保证任务内一致）。 */
typedef struct
{
    float boostTemperature;
    float pv1Voltage;
    float pv2Voltage;
    float gridActivePower;
    float gridFreqHz;
    float currentAmpMax;
} PowerInput;

void Task_Power_Init(void)
{
    /* 电流上限默认满：电流由母线环 + 功率环自动产生。
     * currentAmpMax 供 SCI 手动降载（未来实现），现在默认不限制。 */
    gPowerLimitData.currentAmpMax = BUS_CURRENT_AMP_MAX_NORM;
    gPowerLimitData.currentAmpLimit = BUS_CURRENT_AMP_MAX_NORM;
    Power_State.overloadTimerMs = 0U;
    Power_State.overloadActive = 0U;
    Power_State.overloadBackMs = 0U;
    Power_State.tempDerateTimerMs = 0U;
}

/* 频率降额接口（预留）：电网频率 → 功率上限 W。
 * 老代码 power_calc_freq_derate（德国/比利时过频曲线 VDE-AR-N 4105），依赖国家码。
 * 待安规/国家码接入时在此实现，当前不降额。 */
static float Power_FreqDerate(float freqHz)
{
    (void)freqHz;
    return POWER_OVERLOAD_W;
}

/* 无功调度上限接口（预留）：有功功率 → 有功上限 W（cosφ(P) 曲线）。
 * 老代码 reactive.c 的 full_load_q_limit，依赖国家码。
 * 待安规/国家码接入时在此实现，当前不降额。 */
static float Power_ReactiveDerate(float activePower)
{
    (void)activePower;
    return POWER_OVERLOAD_W;
}

/*============================================================================
 * 功能层：各路约束降额取 min → 功率环积分器 → 电流限幅 clamp。
 *==========================================================================*/
static void Power_Compute(const PowerInput *input)
{
    float thermalLimit;
    float pvLimit;
    float overloadLimit;
    float freqLimit;
    float reactiveLimit;
    float targetPower;
    float ampMax;

    /* 1. 温度降额（带迟滞）：超阈值持续 POWER_TEMP_DERATE_DELAY_MS 才降额 */
    if (input->boostTemperature > POWER_TEMP_DERATE_START_C)
    {
        Power_State.tempDerateTimerMs += TASK_POWER_PERIOD_MS;
        if (Power_State.tempDerateTimerMs >= POWER_TEMP_DERATE_DELAY_MS)
        {
            thermalLimit =  POWER_OVERLOAD_W - 
                            POWER_TEMP_DERATE_RATE_W_PER_C * (input->boostTemperature - POWER_TEMP_DERATE_START_C);
        }
        else
        {
            thermalLimit = POWER_OVERLOAD_W;   /* 迟滞期内不降额 */
        }
    }
    else
    {
        Power_State.tempDerateTimerMs = 0U;
        thermalLimit = POWER_OVERLOAD_W;
    }

    /* 2. PV 过压降额（双路独立，取 min） */
    pvLimit = POWER_OVERLOAD_W;
    if (input->pv1Voltage > POWER_PV_DERATE_START_V)
    {
        float pv1Limit = POWER_OVERLOAD_W -
                         POWER_PV_DERATE_SLOPE_W_PER_V *
                         (input->pv1Voltage - POWER_PV_DERATE_START_V);
        if (pv1Limit < pvLimit) 
        { 
            pvLimit = pv1Limit; 
        }
    }
    if (input->pv2Voltage > POWER_PV_DERATE_START_V)
    {
        float pv2Limit = POWER_OVERLOAD_W -
                         POWER_PV_DERATE_SLOPE_W_PER_V *
                         (input->pv2Voltage - POWER_PV_DERATE_START_V);
        if (pv2Limit < pvLimit) 
        { 
            pvLimit = pv2Limit; 
        }
    }

    /* 3. 过载管理：超额定持续 → 降档到额定；回落持续 → 升档回过载 */
    if (input->gridActivePower > POWER_RATED_W)
    {
        Power_State.overloadBackMs = 0U;
        if (Power_State.overloadActive == 0U)
        {
            Power_State.overloadTimerMs += TASK_POWER_PERIOD_MS;
            if (Power_State.overloadTimerMs >= POWER_OVERLOAD_TIME_MS)
            {
                Power_State.overloadActive = 1U;   /* 降档 */
            }
        }
    }
    else
    {
        Power_State.overloadTimerMs = 0U;
        if (Power_State.overloadActive != 0U)
        {
            Power_State.overloadBackMs += TASK_POWER_PERIOD_MS;
            if (Power_State.overloadBackMs >= POWER_OVERLOAD_BACK_TIME_MS)
            {
                Power_State.overloadActive = 0U;   /* 升档 */
            }
        }
    }
    overloadLimit = (Power_State.overloadActive != 0U) ? POWER_RATED_W : POWER_OVERLOAD_W;

    /* 4. 频率降额（接口预留，当前不降额） */
    freqLimit = Power_FreqDerate(input->gridFreqHz);

    /* 5. 无功调度上限（接口预留，当前不降额） */
    reactiveLimit = Power_ReactiveDerate(input->gridActivePower);

    /* 6. 各路取 min 得目标功率 */
    targetPower = POWER_OVERLOAD_W;
    targetPower = (thermalLimit  < targetPower) ? thermalLimit  : targetPower;
    targetPower = (pvLimit       < targetPower) ? pvLimit       : targetPower;
    targetPower = (overloadLimit < targetPower) ? overloadLimit : targetPower;
    targetPower = (freqLimit     < targetPower) ? freqLimit     : targetPower;
    targetPower = (reactiveLimit < targetPower) ? reactiveLimit : targetPower;

    /* 7. 功率环（纯积分器）：功率误差 → 电流限幅 */
    gPowerLimitData.currentAmpLimit += (targetPower - input->gridActivePower) / POWER_INT_GAIN_W;

    /* 8. 最终 clamp：上限 = SCI 手动上限 currentAmpMax */
    ampMax = System_Clamp(input->currentAmpMax, BUS_CURRENT_AMP_MIN_NORM, BUS_CURRENT_AMP_MAX_NORM);
    gPowerLimitData.currentAmpLimit = System_Clamp(gPowerLimitData.currentAmpLimit, BUS_CURRENT_AMP_MIN_NORM, ampMax);
}

/* 任务入口：先调度（判断是否该干活），再功能（限幅计算）。 */
void Task_Power(void)
{
    PowerInput input;

    input.boostTemperature = gMachineData.realAvg.boostTemperature;
    input.pv1Voltage = gMachineData.realAvg.pv1Voltage;
    input.pv2Voltage = gMachineData.realAvg.pv2Voltage;
    input.gridActivePower = gMachineData.powerData.gridActivePower;
    input.gridFreqHz = (float)gMachineData.ecapFreqCent * 0.01f;
    input.currentAmpMax = gPowerLimitData.currentAmpMax;

    /* 调度层：仅并网态限功率；脱离并网复位限流和私有状态。 */
    if (gSysData.state != SYS_STATE_NORMAL)
    {
        gPowerLimitData.currentAmpLimit = 0.0f;
        Power_State.overloadTimerMs = 0U;
        Power_State.overloadActive = 0U;
        Power_State.overloadBackMs = 0U;
        Power_State.tempDerateTimerMs = 0U;
        return;
    }

    /* 功能层：限幅计算。 */
    Power_Compute(&input);
}
