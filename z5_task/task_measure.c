#include "F28x_Project.h"
#include <math.h>
#include "task.h"
#include "bsp.h"
#include "variable.h"

static float ADC_ToReal(Uint16 raw, const ADC_CalParam *cal);
static float ADC_MeanSqToRms(float meanSq, const ADC_CalParam *cal);
static float ADC_ConvertTemp(Uint16 raw);
static void ADC_RawToReal(const ADC_RawData *rawData, const ADC_Calibrate *cal, ADC_Real *realData);

static float ADC_ToReal(Uint16 raw, const ADC_CalParam *cal)
{
    return ((float)raw - cal->offset) * cal->gain;
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

static void ADC_RawToReal(const ADC_RawData *rawData, const ADC_Calibrate *cal, ADC_Real *realData)
{
    realData->gridVoltage =         ADC_ToReal(rawData->gridVoltage, &cal->gridVoltage);
    realData->inductorCurrent =     ADC_ToReal(rawData->inductorCurrent, &cal->inductorCurrent);
    realData->gfciCurrent =         ADC_ToReal(rawData->gfciCurrent, &cal->gfciCurrent);
    realData->dcBusVoltage =        ADC_ToReal(rawData->dcBusVoltage, &cal->dcBusVoltage);
    realData->gridDcCurrent =       ADC_ToReal(rawData->gridDcCurrent, &cal->gridDcCurrent);
    realData->inverterVoltage =     ADC_ToReal(rawData->inverterVoltage, &cal->inverterVoltage);
    realData->pv1Current =          ADC_ToReal(rawData->pv1Current, &cal->pv1Current);
    realData->pv2Current =          ADC_ToReal(rawData->pv2Current, &cal->pv2Current);
    realData->pv1Voltage =          ADC_ToReal(rawData->pv1Voltage, &cal->pv1Voltage);
    realData->pv2Voltage =          ADC_ToReal(rawData->pv2Voltage, &cal->pv2Voltage);
    realData->pv1IsolationVoltage = ADC_ToReal(rawData->pv1Isolation, &cal->pv1Isolation);
    realData->pv2IsolationVoltage = ADC_ToReal(rawData->pv2Isolation, &cal->pv2Isolation);
    realData->inverterTemp =        ADC_ConvertTemp(rawData->inverterTemperature);
    realData->boostTemp =           ADC_ConvertTemp(rawData->boostTemperature);
}

void Task_Measure(void)
{
    static ADC_RawData rawInstant = {0};
    static ADC_RawData rawAvg = {0};
    static ADC_RawMeanSq rawMeanSq = {0};
    static Uint16 validMask = 0U;

    Uint16 updatedMask;
    ADC_Calibrate cal;
    ADC_Real realInstant;
    ADC_Real realAvg;
    ADC_RealRms realRms;

    /* Use one calibration snapshot for DMA statistics and real conversion. */
    DINT;
    cal = gAdcCal;
    EINT;

    /* DMA has already moved ADC results; this step consumes completed blocks. */
    updatedMask = DMA_ProcessBlocks(&rawInstant, &rawAvg, &rawMeanSq, &cal);
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
    realRms.pv1IsolationVoltage = ADC_MeanSqToRms(rawMeanSq.pv1Isolation, &cal.pv1Isolation);
    realRms.pv2IsolationVoltage = ADC_MeanSqToRms(rawMeanSq.pv2Isolation, &cal.pv2Isolation);

    /* Publish only the raw instantaneous value and calibrated results. */
    gMachineData.realInstant = realInstant;
    gMachineData.realAvg = realAvg;
    gMachineData.realRms = realRms;
    gMachineData.rawInstant = rawInstant;
    gMachineData.measureSeq++;
}
