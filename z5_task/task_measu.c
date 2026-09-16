#include "F28x_Project.h"
#include <math.h>
#include "task.h"
#include "bsp.h"
#include "variable.h"

/* 老代码 4.7kΩ NTC 分压电阻 + 分段温度曲线常量。 */
#define ADC_TEMP_DIVIDER_RESIST        4700.0f
#define ADC_DECI_C_TO_C                    0.1f

static float ADC_ToReal(Uint16 raw, const ADC_CalParam *cal);
static float ADC_MeanSqToRms(float meanSq, const ADC_CalParam *cal);
static float ADC_ConvertTemp(Uint16 raw);



static float ADC_ToReal(Uint16 raw, const ADC_CalParam *cal)
{
    return ((float)raw - cal->Bias) * cal->gain;
}

/* 带运行时零漂的物理值换算：先减出厂校准再减运行时零漂，最后乘增益。 */
static float ADC_ToRealBias(Uint16 raw, const ADC_CalParam *cal, float Bias)
{
    return ((float)raw - cal->Bias - Bias) * cal->gain;
}

/* ADC 运行时零漂校准（原 signal_check_adc_Bias）。
 * 开机/重连时（adInitial==1）信号本应为 0，采 32 组码值平均得到采样链路零漂。 */
static void Measu_CheckAdcBias(const ADC_UintData *rawAvg)
{
    if (gAdcBiasCal.adInitial == 0U)
    {
        return;
    }

    if (gAdcBiasCal.checkCnt != 0U && gAdcBiasCal.checkCnt <= 32U)
    {
        gAdcBiasCal.sum.inductCurr += (float)rawAvg->inductCurr;
        gAdcBiasCal.sum.gridVolt     += (float)rawAvg->gridVolt;
        gAdcBiasCal.sum.gfciCurr     += (float)rawAvg->gfciCurr;
        gAdcBiasCal.sum.gridDcCurr   += (float)rawAvg->gridDcCurr;
    }

    if (gAdcBiasCal.checkCnt <= 32U)
    {
        gAdcBiasCal.checkCnt++;
    }
    else
    {
        gAdcBiasCal.Bias.inductCurr = gAdcBiasCal.sum.inductCurr / 32.0f;
        gAdcBiasCal.Bias.gridVolt     = gAdcBiasCal.sum.gridVolt     / 32.0f;
        gAdcBiasCal.Bias.gfciCurr     = gAdcBiasCal.sum.gfciCurr     / 32.0f;
        gAdcBiasCal.Bias.gridDcCurr   = gAdcBiasCal.sum.gridDcCurr   / 32.0f;


        gAdcBiasCal.sum.inductCurr = 0.0f;
        gAdcBiasCal.sum.gridVolt     = 0.0f;
        gAdcBiasCal.sum.gfciCurr     = 0.0f;
        gAdcBiasCal.sum.gridDcCurr   = 0.0f;
        gAdcBiasCal.checkCnt          = 0U;
        gAdcBiasCal.adInitial           = 0U;
    }
}

static float ADC_MeanSqToRms(float meanSq, const ADC_CalParam *cal)
{
    float gain = cal->gain;

    /* 保护 sqrtf：防止浮点舍入产生的小负值导致开方异常。 */
    if (meanSq <= 0.0f)
    {
        return 0.0f;
    }
    if (gain < 0.0f)
    {
        gain = -gain;
    }
    return sqrtf(meanSq) * gain;
}

static float ADC_ConvertTemp(Uint16 raw)
{
    float rawFloat = (float)raw;
    float resist;
    float tempDeciC;

    /* 满量程码值会让分压公式出现除零奇异，先钳位到满量程-1。 */
    if (rawFloat >= ADC_FULL_SCALE)
    {
        rawFloat = ADC_FULL_SCALE - 1.0f;
    }

    resist = rawFloat * ADC_TEMP_DIVIDER_RESIST / (ADC_FULL_SCALE - rawFloat);

    /* 保留老代码的 NTC 分段曲线（现在由 FPU 计算）。 */
    tempDeciC = 
            (resist > 50000.0f) ? (71.0f - resist / 557.0f)  :
            (resist > 19000.0f) ? (306.0f - resist / 157.0f) :
            (resist > 7700.0f)  ? (529.0f - resist / 54.0f)  :
            (resist > 3450.0f)  ? (758.0f - resist / 20.0f)  :
                                      (937.0f - resist / 10.0f)  ;

    return tempDeciC * ADC_DECI_C_TO_C;
}

/*============================================================================
 * Task_Measu —— 测量任务（3ms 周期 + 电网过零事件触发）
 *
 * 调度层：消费 DMA 已完成的采样块，做"开机凑齐 + PV 配对"门控，
 *         判断本周期是否该执行测量。
 * 功能层：ADC 零漂校准 → 物理值换算（AVG/RMS）→ 功率计算 → 发布到 gMachineData。
 *==========================================================================*/

/* 调度层：消费 DMA 块，判断本周期是否该执行测量。
 * 返回 0 = 本周期无需执行（无 DMA 完成 / 启动凑齐中）。
 * 返回非 0 = 该执行，返回值即 updatedMask，*pvUpdated 输出 PV 是否配对完成。 */
static Uint16 Measu_TrySchedule
(
    const ADC_Calibrate *cal,
    ADC_UintData *rawAvg,
    ADC_FloatData *rawMeanSq,
    float *gridVoltCurrMeanRaw,
    Uint16 *pvUpdated
)
{
    static Uint16 validMask = 0U;
    Uint16 updatedMask;

    /* 消费 DMA 已完成的块（AVG/RMS 累加已在 DMA 里算好）。 */
    updatedMask = DMA_ProcessBlocks(rawAvg, rawMeanSq, gridVoltCurrMeanRaw, cal);
    if (updatedMask == 0U)
    {
        return 0U;   /* 本周期无 DMA 块完成 */
    }

    /* 开机凑齐：所有 DMA 块都至少到过一次，才允许发布（防半成品）。 */
    validMask |= updatedMask;
    if (validMask != DMA_UPDATE_ALL)
    {
        return 0U;
    }

    /* PV 配对：PV 电流块 + 电压块可能分属不同 tick，凑成一对才算有效。 */
    *pvUpdated = ((updatedMask & DMA_UPDATE_PV) != 0U) ? 1U : 0U;

    return updatedMask;
}

/* 功能层：执行测量换算并发布（零漂 → AVG/RMS → 功率 → gMachineData）。 */
static void Measu_Compute
(
    const ADC_Calibrate *cal,
    const ADC_UintData *rawAvg,
    const ADC_FloatData *rawMeanSq,
    float gridVoltCurrMeanRaw,
    Uint16 updatedMask,
    Uint16 pvUpdated
)
{
    ADC_FloatData realAvg;
    ADC_FloatData realRms = {0};
    PowerData powerData;

    /* ADC 运行时零漂校准（开机/重连时采 32 组码值平均）。 */
    Measu_CheckAdcBias(rawAvg);

    /* 原始码值 → 物理值（平均值）：4 个交流/测零通道减运行时零漂，其余只减出厂校准。 */
    realAvg.gridVolt =           ADC_ToRealBias(rawAvg->gridVolt, &cal->gridVolt, gAdcBiasCal.Bias.gridVolt);
    realAvg.inductCurr =       ADC_ToRealBias(rawAvg->inductCurr, &cal->inductCurr, gAdcBiasCal.Bias.inductCurr);
    realAvg.gfciCurr =           ADC_ToRealBias(rawAvg->gfciCurr, &cal->gfciCurr, gAdcBiasCal.Bias.gfciCurr);
    realAvg.gridDcCurr =         ADC_ToRealBias(rawAvg->gridDcCurr, &cal->gridDcCurr, gAdcBiasCal.Bias.gridDcCurr);
    realAvg.dcBusVolt =          ADC_ToReal(rawAvg->dcBusVolt, &cal->dcBusVolt);
    realAvg.invertVolt =       ADC_ToReal(rawAvg->invertVolt, &cal->invertVolt);
    realAvg.pv1Curr =            ADC_ToReal(rawAvg->pv1Curr, &cal->pv1Curr);
    realAvg.pv2Curr =            ADC_ToReal(rawAvg->pv2Curr, &cal->pv2Curr);
    realAvg.pv1Volt =            ADC_ToReal(rawAvg->pv1Volt, &cal->pv1Volt);
    realAvg.pv2Volt =            ADC_ToReal(rawAvg->pv2Volt, &cal->pv2Volt);
    realAvg.pv1Insul =          ADC_ToReal(rawAvg->pv1Insul, &cal->pv1Insul);
    realAvg.pv2Insul =          ADC_ToReal(rawAvg->pv2Insul, &cal->pv2Insul);
    realAvg.invertTemp =   ADC_ConvertTemp(rawAvg->invertTemp);
    realAvg.boostTemp =      ADC_ConvertTemp(rawAvg->boostTemp);

    /* 均方值 → RMS。 */
    realRms.gridVolt =           ADC_MeanSqToRms(rawMeanSq->gridVolt, &cal->gridVolt);
    realRms.inductCurr =       ADC_MeanSqToRms(rawMeanSq->inductCurr, &cal->inductCurr);
    realRms.gfciCurr =           ADC_MeanSqToRms(rawMeanSq->gfciCurr, &cal->gfciCurr);
    realRms.dcBusVolt =          ADC_MeanSqToRms(rawMeanSq->dcBusVolt, &cal->dcBusVolt);
    realRms.gridDcCurr =         ADC_MeanSqToRms(rawMeanSq->gridDcCurr, &cal->gridDcCurr);
    realRms.invertVolt =       ADC_MeanSqToRms(rawMeanSq->invertVolt, &cal->invertVolt);
    realRms.pv1Curr =            ADC_MeanSqToRms(rawMeanSq->pv1Curr, &cal->pv1Curr);
    realRms.pv2Curr =            ADC_MeanSqToRms(rawMeanSq->pv2Curr, &cal->pv2Curr);
    realRms.pv1Volt =            ADC_MeanSqToRms(rawMeanSq->pv1Volt, &cal->pv1Volt);
    realRms.pv2Volt =            ADC_MeanSqToRms(rawMeanSq->pv2Volt, &cal->pv2Volt);
    realRms.pv1Insul =          ADC_MeanSqToRms(rawMeanSq->pv1Insul, &cal->pv1Insul);
    realRms.pv2Insul =          ADC_MeanSqToRms(rawMeanSq->pv2Insul, &cal->pv2Insul);

    /* 功率计算（PV 功率 + 电网有功，读-改-写保留累计能量等其它字段）。 */
    powerData = gMachineData.powerData;
    if (pvUpdated != 0U)
    {
        powerData.pv1Power = realAvg.pv1Volt * realAvg.pv1Curr;
        powerData.pv2Power = realAvg.pv2Volt * realAvg.pv2Curr;
    }
    if ((updatedMask & DMA_UPDATE_FAST) != 0U)
    {
        powerData.gridActivePower = gridVoltCurrMeanRaw * cal->gridVolt.gain * cal->inductCurr.gain;
    }

    /* 发布校准后的平均值、RMS 和功率，供控制/任务消费。 */
    gMachineData.realAvg = realAvg;
    gMachineData.realRms = realRms;
    gMachineData.powerData = powerData;
    gMachineData.measuSeq++;
}

/* 任务入口：先调度（判断是否该干活），再功能（换算 + 发布）。 */
void Task_Measu(void)
{
    static          ADC_UintData  rawAvg = {0};
    static          ADC_FloatData rawMeanSq = {0};
    static float    gridVoltCurrMeanRaw = 0.0f;
    ADC_Calibrate   cal;
    Uint16          updatedMask;
    Uint16          pvUpdated;
    Uint16          interruptState;

    /* 用一份校准快照同时供 DMA 统计和物理值换算。 */
    interruptState = CPU_InterruptSaveDisable();
    cal = gAdcCal;
    CPU_InterruptRestore(interruptState);

    /* 调度层：消费 DMA 块，判断本周期是否该执行。 */
    updatedMask = Measu_TrySchedule(&cal, &rawAvg, &rawMeanSq, &gridVoltCurrMeanRaw, &pvUpdated);
    if (updatedMask == 0U)
    {
        return;
    }

    /* 功能层：换算 + 功率 + 发布。 */
    Measu_Compute(&cal, &rawAvg, &rawMeanSq, gridVoltCurrMeanRaw, updatedMask, pvUpdated);
}
