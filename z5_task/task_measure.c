#include "F28x_Project.h"
#include <math.h>
#include "task.h"
#include "bsp.h"
#include "variable.h"
#include "system.h"

/* Legacy 4.7 kOhm NTC divider and piecewise temperature-curve constants. */
#define ADC_TEMP_DIVIDER_RESISTANCE        4700.0f
#define ADC_DECI_C_TO_C                    0.1f

static float ADC_ToReal(Uint16 raw, const ADC_CalParam *cal);
static float ADC_MeanSqToRms(float meanSq, const ADC_CalParam *cal);
static float ADC_ConvertTemp(Uint16 raw);
static void ADC_RawToReal(const ADC_UintData *rawData, const ADC_Calibrate *cal, ADC_FloatData *realData);
static float Measure_Abs(float value);
static void Measure_UpdatePower(PowerData *powerData,
                                const ADC_FloatData *realAvg,
                                const ADC_FloatData *realRms,
                                float gridVoltCurrentMeanRaw,
                                float fastWindowSec,
                                const ADC_Calibrate *cal,
                                Uint16 fastUpdated,
                                Uint16 pvUpdated);

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
        gAdcOffsetCal.checkCount = 0U;
        gAdcOffsetCal.adInitial  = 0U;
    }
}

static float ADC_MeanSqToRms(float meanSq, const ADC_CalParam *cal)
{
    float gain = cal->gain;

    /* Protect sqrtf from a small negative value caused by float roundoff. */
    if(meanSq <= 0.0f)
    {
        return 0.0f;
    }
    if(gain < 0.0f)
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

    /* A full-scale code would make the divider equation singular. */
    if(rawFloat >= ADC_FULL_SCALE)
    {
        rawFloat = ADC_FULL_SCALE - 1.0f;
    }

    resistance = rawFloat * ADC_TEMP_DIVIDER_RESISTANCE /
                 (ADC_FULL_SCALE - rawFloat);

    /* Preserve the legacy NTC piecewise curve, now evaluated by the FPU. */
    if(resistance > 50000.0f)
    {
        tempDeciC = 71.0f - resistance / 557.0f;
    }
    else if(resistance > 19000.0f)
    {
        tempDeciC = 306.0f - resistance / 157.0f;
    }
    else if(resistance > 7700.0f)
    {
        tempDeciC = 529.0f - resistance / 54.0f;
    }
    else if(resistance > 3450.0f)
    {
        tempDeciC = 758.0f - resistance / 20.0f;
    }
    else
    {
        tempDeciC = 937.0f - resistance / 10.0f;
    }

    return tempDeciC * ADC_DECI_C_TO_C;
}

static void ADC_RawToReal(const ADC_UintData *rawData, const ADC_Calibrate *cal, ADC_FloatData *realData)
{
    /* 4 个交流/测零通道减运行时零漂，其余通道只减出厂校准。 */
    realData->gridVoltage =         ADC_ToRealOffset(rawData->gridVoltage, &cal->gridVoltage, gAdcOffsetCal.offset.gridVoltage);
    realData->inductorCurrent =     ADC_ToRealOffset(rawData->inductorCurrent, &cal->inductorCurrent, gAdcOffsetCal.offset.inductorCurrent);
    realData->gfciCurrent =         ADC_ToRealOffset(rawData->gfciCurrent, &cal->gfciCurrent, gAdcOffsetCal.offset.gfciCurrent);
    realData->gridDcCurrent =       ADC_ToRealOffset(rawData->gridDcCurrent, &cal->gridDcCurrent, gAdcOffsetCal.offset.gridDcCurrent);
    realData->dcBusVoltage =        ADC_ToReal(rawData->dcBusVoltage, &cal->dcBusVoltage);
    realData->inverterVoltage =     ADC_ToReal(rawData->inverterVoltage, &cal->inverterVoltage);
    realData->pv1Current =          ADC_ToReal(rawData->pv1Current, &cal->pv1Current);
    realData->pv2Current =          ADC_ToReal(rawData->pv2Current, &cal->pv2Current);
    realData->pv1Voltage =          ADC_ToReal(rawData->pv1Voltage, &cal->pv1Voltage);
    realData->pv2Voltage =          ADC_ToReal(rawData->pv2Voltage, &cal->pv2Voltage);
    realData->pv1Isolation =        ADC_ToReal(rawData->pv1Isolation, &cal->pv1Isolation);
    realData->pv2Isolation =        ADC_ToReal(rawData->pv2Isolation, &cal->pv2Isolation);
    realData->inverterTemperature = ADC_ConvertTemp(rawData->inverterTemperature);
    realData->boostTemperature =    ADC_ConvertTemp(rawData->boostTemperature);
}

static float Measure_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void Measure_UpdatePower(PowerData *powerData,
                                const ADC_FloatData *realAvg,
                                const ADC_FloatData *realRms,
                                float gridVoltCurrentMeanRaw,
                                float fastWindowSec,
                                const ADC_Calibrate *cal,
                                Uint16 fastUpdated,
                                Uint16 pvUpdated)
{
    float apparentPower;
    float reactivePowerSquared;
    float reactivePowerMagnitude;
    float gridActivePower;

    /* PV is DC, so the block-average voltage and current form its power.
     * Recalculate only after both PV DMA blocks have arrived. */
    if(pvUpdated != 0U)
    {
        powerData->pv1Power = realAvg->pv1Voltage * realAvg->pv1Current;
        powerData->pv2Power = realAvg->pv2Voltage * realAvg->pv2Current;
        powerData->totalPvPower = powerData->pv1Power + powerData->pv2Power;
    }

    if(fastUpdated != 0U)
    {
        /* DMA accumulated centered ADC-code products. Convert both axes to
         * engineering units only once after the block has been consumed. */
        gridActivePower = gridVoltCurrentMeanRaw *
                          cal->gridVoltage.gain *
                          cal->inductorCurrent.gain;
        powerData->gridActivePower = gridActivePower;

        /* A complete block contains a stable RMS estimate for both axes. */
        apparentPower = Measure_Abs(realRms->gridVoltage) *
                        Measure_Abs(realRms->inductorCurrent);
        powerData->gridApparentPower = apparentPower;

        if(apparentPower > 0.001f)
        {
            powerData->powerFactor = gridActivePower / apparentPower;
            powerData->powerFactor = System_Clamp(powerData->powerFactor,
                                                  -1.0f,
                                                  1.0f);
        }
        else
        {
            powerData->powerFactor = 0.0f;
        }

        /* Without a quadrature-current accumulator, Q can only be estimated
         * from S^2-P^2. The reactive command supplies its intended sign. */
        reactivePowerSquared = apparentPower * apparentPower -
                               gridActivePower * gridActivePower;
        if(reactivePowerSquared < 0.0f)
        {
            reactivePowerSquared = 0.0f;
        }
        reactivePowerMagnitude = sqrtf(reactivePowerSquared);
        if(gReactiveData.phaseShiftRad < 0.0f)
        {
            powerData->gridReactivePower = -reactivePowerMagnitude;
        }
        else if(gReactiveData.phaseShiftRad > 0.0f)
        {
            powerData->gridReactivePower = reactivePowerMagnitude;
        }
        else
        {
            powerData->gridReactivePower = 0.0f;
        }

        /* Integrate only newly completed fast blocks. Wh = W * seconds / 3600. */
        if((gridActivePower > 0.0f) && (fastWindowSec > 0.0f))
        {
            powerData->totalEnergyWh += gridActivePower * fastWindowSec / 3600.0f;
        }
    }
}

void Task_Measure(void)
{
    static ADC_UintData rawInstant = {0};
    static ADC_UintData rawAvg = {0};
    static ADC_FloatData rawMeanSq = {0};
    static float gridVoltCurrentMeanRaw = 0.0f;
    static float fastWindowSec = 0.0f;
    static Uint16 validMask = 0U;
    static Uint16 pvUpdateMask = 0U;

    Uint16 updatedMask;
    ADC_Calibrate cal;
    ADC_FloatData realInstant;
    ADC_FloatData realAvg;
    ADC_FloatData realRms = {0};
    PowerData powerData;
    Uint16 pvUpdated;

    /* Use one calibration snapshot for DMA statistics and real conversion. */
    DINT;
    cal = gAdcCal;
    EINT;

    /* DMA has already moved ADC results; this step consumes completed blocks. */
    updatedMask = DMA_ProcessBlocks(&rawInstant,
                                    &rawAvg,
                                    &rawMeanSq,
                                    &gridVoltCurrentMeanRaw,
                                    &fastWindowSec,
                                    &cal);
    if(updatedMask == 0U)
    {
        return;
    }

    /* Do not publish partially initialized measurements during startup. */
    validMask |= updatedMask;
    if(validMask != DMA_UPDATE_ALL)
    {
        return;
    }

    /* The two PV DMA channels may complete in different scheduler ticks.
     * Accumulate their notifications until a matched pair is available. */
    pvUpdateMask |= (updatedMask & DMA_UPDATE_PV_ALL);
    pvUpdated = ((pvUpdateMask & DMA_UPDATE_PV_ALL) == DMA_UPDATE_PV_ALL) ? 1U : 0U;
    if(pvUpdated != 0U)
    {
        pvUpdateMask &= (Uint16)(~DMA_UPDATE_PV_ALL);
    }

    /* ADC 运行时零漂校准（开机/重连时采 32 组码值平均）。 */
    Measure_CheckAdcOffset(&rawAvg);

    ADC_RawToReal(&rawInstant, &cal, &realInstant);
    ADC_RawToReal(&rawAvg, &cal, &realAvg);

    realRms.gridVoltage = ADC_MeanSqToRms(rawMeanSq.gridVoltage, &cal.gridVoltage);
    realRms.inductorCurrent = ADC_MeanSqToRms(rawMeanSq.inductorCurrent, &cal.inductorCurrent);
    realRms.gfciCurrent = ADC_MeanSqToRms(rawMeanSq.gfciCurrent, &cal.gfciCurrent);
    realRms.dcBusVoltage = ADC_MeanSqToRms(rawMeanSq.dcBusVoltage, &cal.dcBusVoltage);
    realRms.gridDcCurrent = ADC_MeanSqToRms(rawMeanSq.gridDcCurrent, &cal.gridDcCurrent);
    realRms.inverterVoltage = ADC_MeanSqToRms(rawMeanSq.inverterVoltage, &cal.inverterVoltage);
    realRms.pv1Current = ADC_MeanSqToRms(rawMeanSq.pv1Current, &cal.pv1Current);
    realRms.pv2Current = ADC_MeanSqToRms(rawMeanSq.pv2Current, &cal.pv2Current);
    realRms.pv1Voltage = ADC_MeanSqToRms(rawMeanSq.pv1Voltage, &cal.pv1Voltage);
    realRms.pv2Voltage = ADC_MeanSqToRms(rawMeanSq.pv2Voltage, &cal.pv2Voltage);
    realRms.pv1Isolation = ADC_MeanSqToRms(rawMeanSq.pv1Isolation, &cal.pv1Isolation);
    realRms.pv2Isolation = ADC_MeanSqToRms(rawMeanSq.pv2Isolation, &cal.pv2Isolation);

    /* Publish only the raw instantaneous value and calibrated results. */
    gMachineData.realInstant = realInstant;
    gMachineData.realAvg = realAvg;
    gMachineData.realRms = realRms;
    gMachineData.rawInstant = rawInstant;
    powerData = gMachineData.powerData;
    Measure_UpdatePower(&powerData,
                        &realAvg,
                        &realRms,
                        gridVoltCurrentMeanRaw,
                        fastWindowSec,
                        &cal,
                        (updatedMask & DMA_UPDATE_FAST),
                        pvUpdated);
    gMachineData.powerData = powerData;
    gMachineData.measureSeq++;
}
