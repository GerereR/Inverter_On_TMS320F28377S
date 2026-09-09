#include "F28x_Project.h"
#include <math.h>
#include "task.h"
#include "bsp.h"
#include "variable.h"

/* 老代码 4.7kΩ NTC 分压电阻 + 分段温度曲线常量。 */
#define ADC_TEMP_DIVIDER_RESISTANCE        4700.0f
#define ADC_DECI_C_TO_C                    0.1f

static float ADC_ToReal(Uint16 raw, const ADC_CalParam *cal);
static float ADC_MeanSqToRms(float meanSq, const ADC_CalParam *cal);
static float ADC_ConvertTemp(Uint16 raw);



static float ADC_ToReal(Uint16 raw, const ADC_CalParam *cal)
{
    return ((float)raw - cal->offset) * cal->gain;
}

/* 带运行时零漂的物理值换算：先减出厂校准再减运行时零漂，最后乘增益。 */
static float ADC_ToRealOffset(Uint16 raw, const ADC_CalParam *cal, float offset)
{
    return ((float)raw - cal->offset - offset) * cal->gain;
}

/* ADC 运行时零漂校准（原 signal_check_adc_offset）。
 * 开机/重连时（adInitial==1）信号本应为 0，采 32 组码值平均得到采样链路零漂。 */
static void Measure_CheckAdcOffset(const ADC_UintData *rawAvg)
{
    if (gAdcOffsetCal.adInitial == 0U)
    {
        return;
    }

    if (gAdcOffsetCal.checkCount != 0U && gAdcOffsetCal.checkCount <= 32U)
    {
        gAdcOffsetCal.sum.inductorCurrent += (float)rawAvg->inductorCurrent;
        gAdcOffsetCal.sum.gridVoltage     += (float)rawAvg->gridVoltage;
        gAdcOffsetCal.sum.gfciCurrent     += (float)rawAvg->gfciCurrent;
        gAdcOffsetCal.sum.gridDcCurrent   += (float)rawAvg->gridDcCurrent;
    }

    if (gAdcOffsetCal.checkCount <= 32U)
    {
        gAdcOffsetCal.checkCount++;
    }
    else
    {
        gAdcOffsetCal.offset.inductorCurrent = gAdcOffsetCal.sum.inductorCurrent / 32.0f;
        gAdcOffsetCal.offset.gridVoltage     = gAdcOffsetCal.sum.gridVoltage     / 32.0f;
        gAdcOffsetCal.offset.gfciCurrent     = gAdcOffsetCal.sum.gfciCurrent     / 32.0f;
        gAdcOffsetCal.offset.gridDcCurrent   = gAdcOffsetCal.sum.gridDcCurrent   / 32.0f;


        gAdcOffsetCal.sum.inductorCurrent = 0.0f;
        gAdcOffsetCal.sum.gridVoltage     = 0.0f;
        gAdcOffsetCal.sum.gfciCurrent     = 0.0f;
        gAdcOffsetCal.sum.gridDcCurrent   = 0.0f;
        gAdcOffsetCal.checkCount          = 0U;
        gAdcOffsetCal.adInitial           = 0U;
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
    float resistance;
    float tempDeciC;

    /* 满量程码值会让分压公式出现除零奇异，先钳位到满量程-1。 */
    if (rawFloat >= ADC_FULL_SCALE)
    {
        rawFloat = ADC_FULL_SCALE - 1.0f;
    }

    resistance = rawFloat * ADC_TEMP_DIVIDER_RESISTANCE / (ADC_FULL_SCALE - rawFloat);

    /* 保留老代码的 NTC 分段曲线（现在由 FPU 计算）。 */
    tempDeciC = 
            (resistance > 50000.0f) ? (71.0f - resistance / 557.0f)  :
            (resistance > 19000.0f) ? (306.0f - resistance / 157.0f) :
            (resistance > 7700.0f)  ? (529.0f - resistance / 54.0f)  :
            (resistance > 3450.0f)  ? (758.0f - resistance / 20.0f)  :
                                      (937.0f - resistance / 10.0f)  ;

    return tempDeciC * ADC_DECI_C_TO_C;
}

/*============================================================================
 * Task_Measure —— 测量任务（3ms 周期 + 电网过零事件触发）
 *
 * 调度层：消费 DMA 已完成的采样块，做"开机凑齐 + PV 配对"门控，
 *         判断本周期是否该执行测量。
 * 功能层：ADC 零漂校准 → 物理值换算（AVG/RMS）→ 功率计算 → 发布到 gMachineData。
 *==========================================================================*/

/* 调度层：消费 DMA 块，判断本周期是否该执行测量。
 * 返回 0 = 本周期无需执行（无 DMA 完成 / 启动凑齐中）。
 * 返回非 0 = 该执行，返回值即 updatedMask，*pvUpdated 输出 PV 是否配对完成。 */
static Uint16 Measure_TrySchedule
(
    const ADC_Calibrate *cal,
    ADC_UintData *rawAvg,
    ADC_FloatData *rawMeanSq,
    float *gridVoltCurrentMeanRaw,
    Uint16 *pvUpdated
)
{
    static Uint16 validMask = 0U;
    static Uint16 pvUpdateMask = 0U;
    Uint16 updatedMask;

    /* 消费 DMA 已完成的块（AVG/RMS 累加已在 DMA 里算好）。 */
    updatedMask = DMA_ProcessBlocks(rawAvg, rawMeanSq, gridVoltCurrentMeanRaw, cal);
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
    pvUpdateMask |= (updatedMask & DMA_UPDATE_PV_ALL);
    *pvUpdated = ((pvUpdateMask & DMA_UPDATE_PV_ALL) == DMA_UPDATE_PV_ALL) ? 1U : 0U;
    if (*pvUpdated != 0U)
    {
        pvUpdateMask &= (Uint16)(~DMA_UPDATE_PV_ALL);
    }

    return updatedMask;
}

/* 功能层：执行测量换算并发布（零漂 → AVG/RMS → 功率 → gMachineData）。 */
static void Measure_Compute
(
    const ADC_Calibrate *cal,
    const ADC_UintData *rawAvg,
    const ADC_FloatData *rawMeanSq,
    float gridVoltCurrentMeanRaw,
    Uint16 updatedMask,
    Uint16 pvUpdated
)
{
    ADC_FloatData realAvg;
    ADC_FloatData realRms = {0};
    PowerData powerData;

    /* ADC 运行时零漂校准（开机/重连时采 32 组码值平均）。 */
    Measure_CheckAdcOffset(rawAvg);

    /* 原始码值 → 物理值（平均值）：4 个交流/测零通道减运行时零漂，其余只减出厂校准。 */
    realAvg.gridVoltage =           ADC_ToRealOffset(rawAvg->gridVoltage, &cal->gridVoltage, gAdcOffsetCal.offset.gridVoltage);
    realAvg.inductorCurrent =       ADC_ToRealOffset(rawAvg->inductorCurrent, &cal->inductorCurrent, gAdcOffsetCal.offset.inductorCurrent);
    realAvg.gfciCurrent =           ADC_ToRealOffset(rawAvg->gfciCurrent, &cal->gfciCurrent, gAdcOffsetCal.offset.gfciCurrent);
    realAvg.gridDcCurrent =         ADC_ToRealOffset(rawAvg->gridDcCurrent, &cal->gridDcCurrent, gAdcOffsetCal.offset.gridDcCurrent);
    realAvg.dcBusVoltage =          ADC_ToReal(rawAvg->dcBusVoltage, &cal->dcBusVoltage);
    realAvg.inverterVoltage =       ADC_ToReal(rawAvg->inverterVoltage, &cal->inverterVoltage);
    realAvg.pv1Current =            ADC_ToReal(rawAvg->pv1Current, &cal->pv1Current);
    realAvg.pv2Current =            ADC_ToReal(rawAvg->pv2Current, &cal->pv2Current);
    realAvg.pv1Voltage =            ADC_ToReal(rawAvg->pv1Voltage, &cal->pv1Voltage);
    realAvg.pv2Voltage =            ADC_ToReal(rawAvg->pv2Voltage, &cal->pv2Voltage);
    realAvg.pv1Isolation =          ADC_ToReal(rawAvg->pv1Isolation, &cal->pv1Isolation);
    realAvg.pv2Isolation =          ADC_ToReal(rawAvg->pv2Isolation, &cal->pv2Isolation);
    realAvg.inverterTemperature =   ADC_ConvertTemp(rawAvg->inverterTemperature);
    realAvg.boostTemperature =      ADC_ConvertTemp(rawAvg->boostTemperature);

    /* 均方值 → RMS。 */
    realRms.gridVoltage =           ADC_MeanSqToRms(rawMeanSq->gridVoltage, &cal->gridVoltage);
    realRms.inductorCurrent =       ADC_MeanSqToRms(rawMeanSq->inductorCurrent, &cal->inductorCurrent);
    realRms.gfciCurrent =           ADC_MeanSqToRms(rawMeanSq->gfciCurrent, &cal->gfciCurrent);
    realRms.dcBusVoltage =          ADC_MeanSqToRms(rawMeanSq->dcBusVoltage, &cal->dcBusVoltage);
    realRms.gridDcCurrent =         ADC_MeanSqToRms(rawMeanSq->gridDcCurrent, &cal->gridDcCurrent);
    realRms.inverterVoltage =       ADC_MeanSqToRms(rawMeanSq->inverterVoltage, &cal->inverterVoltage);
    realRms.pv1Current =            ADC_MeanSqToRms(rawMeanSq->pv1Current, &cal->pv1Current);
    realRms.pv2Current =            ADC_MeanSqToRms(rawMeanSq->pv2Current, &cal->pv2Current);
    realRms.pv1Voltage =            ADC_MeanSqToRms(rawMeanSq->pv1Voltage, &cal->pv1Voltage);
    realRms.pv2Voltage =            ADC_MeanSqToRms(rawMeanSq->pv2Voltage, &cal->pv2Voltage);
    realRms.pv1Isolation =          ADC_MeanSqToRms(rawMeanSq->pv1Isolation, &cal->pv1Isolation);
    realRms.pv2Isolation =          ADC_MeanSqToRms(rawMeanSq->pv2Isolation, &cal->pv2Isolation);

    /* 功率计算（PV 功率 + 电网有功，读-改-写保留累计能量等其它字段）。 */
    powerData = gMachineData.powerData;
    if (pvUpdated != 0U)
    {
        powerData.pv1Power = realAvg.pv1Voltage * realAvg.pv1Current;
        powerData.pv2Power = realAvg.pv2Voltage * realAvg.pv2Current;
    }
    if ((updatedMask & DMA_UPDATE_FAST) != 0U)
    {
        powerData.gridActivePower = gridVoltCurrentMeanRaw * cal->gridVoltage.gain * cal->inductorCurrent.gain;
    }

    /* 发布校准后的平均值、RMS 和功率，供控制/任务消费。 */
    gMachineData.realAvg = realAvg;
    gMachineData.realRms = realRms;
    gMachineData.powerData = powerData;
    gMachineData.measureSeq++;
}

/* 任务入口：先调度（判断是否该干活），再功能（换算 + 发布）。 */
void Task_Measure(void)
{
    static          ADC_UintData  rawAvg = {0};
    static          ADC_FloatData rawMeanSq = {0};
    static float    gridVoltCurrentMeanRaw = 0.0f;
    ADC_Calibrate   cal;
    Uint16          updatedMask;
    Uint16          pvUpdated;
    Uint16          interruptState;

    /* 用一份校准快照同时供 DMA 统计和物理值换算。 */
    interruptState = CPU_InterruptSaveDisable();
    cal = gAdcCal;
    CPU_InterruptRestore(interruptState);

    /* 调度层：消费 DMA 块，判断本周期是否该执行。 */
    updatedMask = Measure_TrySchedule(&cal, &rawAvg, &rawMeanSq, &gridVoltCurrentMeanRaw, &pvUpdated);
    if (updatedMask == 0U)
    {
        return;
    }

    /* 功能层：换算 + 功率 + 发布。 */
    Measure_Compute(&cal, &rawAvg, &rawMeanSq, gridVoltCurrentMeanRaw, updatedMask, pvUpdated);
}
