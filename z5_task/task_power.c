#include "F28x_Project.h"

#include "task.h"
#include "constant.h"
#include "variable.h"
#include "system.h"

/* --- 功率限幅常量（原 powercalc.c / power_mgr.c，按机型配置，当前为默认值） ---
 * 单位：功率 W、温度 ℃、电压 V；电流上限归一化 0..1。
 * 数值为 bring-up 初值，上机后需按实际机型/校准/收敛速度调。 */
#define POWER_OVERLOAD_W                  3300.0f   /* 过载功率上限 */
#define POWER_TEMP_DERATE_START_C         65.0f     /* 温度降额起始点（℃） */
#define POWER_TEMP_DERATE_RATE_W_PER_C    40.0f     /* 温度降额斜率（W/℃） */
#define POWER_PV_DERATE_START_V           500.0f    /* PV 过压降额起始（V） */
#define POWER_PV_DERATE_SLOPE_W_PER_V     8.0f      /* PV 降额斜率（W/V） */
/* 功率环积分增益：老代码 500ms 一次 amp_limit += delta_watt/7500；
 * 本工程 Task_Power 10ms 一次，等效增益 = 7500*(500/10) = 375000。 */
#define POWER_INT_GAIN_W                  375000.0f

void Task_Power_Init(void)
{
    /* 电流上限默认满：电流由母线环 + 功率环自动产生。
     * currentAmpMax 供 SCI 手动降载（未来实现），现在默认不限制。 */
    gPowerLimitData.currentAmpMax = BUS_CURRENT_AMP_MAX_NORM;
    gPowerLimitData.currentAmpLimit = BUS_CURRENT_AMP_MAX_NORM;
    gPowerLimitData.outputPowerCmd = POWER_OVERLOAD_W;
    gPowerLimitData.thermalPowerLimit = POWER_OVERLOAD_W;
    gPowerLimitData.pvPowerLimit = POWER_OVERLOAD_W;
}

void Task_Power(void)
{
    float thermalLimit;
    float pvLimit;
    float targetPower;
    float powerError;
    float ampMax;

    /* 仅并网态限功率；脱离并网复位限流，进 NORMAL 后功率环从 0 积分爬起。 */
    if (gSysData.state != SYS_STATE_NORMAL)
    {
        gPowerLimitData.currentAmpLimit = 0.0f;
        return;
    }

    /* --- 1. 各路降额：减法算出功率上限（W） --- */

    /* 温度降额：overload - rate*(temp - start)。
     * 老代码带 1 分钟迟滞 + 5s 滤波，此处先直接计算，迟滞后续补。 */
    if (gMachineData.realAvg.boostTemperature > POWER_TEMP_DERATE_START_C)
    {
        thermalLimit = POWER_OVERLOAD_W -
                       POWER_TEMP_DERATE_RATE_W_PER_C *
                       (gMachineData.realAvg.boostTemperature - POWER_TEMP_DERATE_START_C);
    }
    else
    {
        thermalLimit = POWER_OVERLOAD_W;
    }

    /* PV 过压降额：overload - slope*(vpv - start)。 */
    if (gMachineData.realAvg.pv1Voltage > POWER_PV_DERATE_START_V)
    {
        pvLimit = POWER_OVERLOAD_W -
                  POWER_PV_DERATE_SLOPE_W_PER_V *
                  (gMachineData.realAvg.pv1Voltage - POWER_PV_DERATE_START_V);
    }
    else
    {
        pvLimit = POWER_OVERLOAD_W;
    }

    /* --- 2. 各路取 min 得目标功率 ---
     * 频率降额、无功调度、过载计时：待安规/国家码接入后补。 */
    targetPower = POWER_OVERLOAD_W;
    if (thermalLimit < targetPower) { targetPower = thermalLimit; }
    if (pvLimit < targetPower)      { targetPower = pvLimit; }

    gPowerLimitData.thermalPowerLimit = thermalLimit;
    gPowerLimitData.pvPowerLimit = pvLimit;
    gPowerLimitData.outputPowerCmd = targetPower;

    /* --- 3. 功率环（纯积分器）：功率误差 → 电流限幅 --- */
    powerError = targetPower - gMachineData.powerData.gridActivePower;
    gPowerLimitData.currentAmpLimit += powerError / POWER_INT_GAIN_W;

    /* --- 4. 最终 clamp：上限 = SCI 手动上限 currentAmpMax（钳到 [0,满]） --- */
    ampMax = System_Clamp(gPowerLimitData.currentAmpMax,
                          BUS_CURRENT_AMP_MIN_NORM,
                          BUS_CURRENT_AMP_MAX_NORM);
    gPowerLimitData.currentAmpLimit = System_Clamp(gPowerLimitData.currentAmpLimit,
                                                   BUS_CURRENT_AMP_MIN_NORM,
                                                   ampMax);
}
