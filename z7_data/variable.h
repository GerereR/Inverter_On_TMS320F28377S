#ifndef VARIABLE_H_
#define VARIABLE_H_

#include "constant.h"

/*
 * Raw ADC values use signal names rather than ADC module/channel names.
 * This keeps the control and communication layers independent of pin routing.
 */
typedef struct
{
    Uint16 gridVoltage;
    Uint16 inductorCurrent;
    Uint16 gfciCurrent;
    Uint16 dcBusVoltage;
    Uint16 gridDcCurrent;
    Uint16 inverterVoltage;
    Uint16 pv1Current;
    Uint16 pv2Current;
    Uint16 pv1Voltage;
    Uint16 pv2Voltage;
    Uint16 pv1Isolation;
    Uint16 pv2Isolation;
    Uint16 inverterTemperature;
    Uint16 boostTemperature;
} ADC_RawData;

/*
 * Mean-square values remain in ADC-count-squared units. The configured
 * channel offset is removed before squaring, so RMS conversion needs only
 * a square root and the channel gain.
 */
typedef struct
{
    float gridVoltage;
    float inductorCurrent;
    float gfciCurrent;
    float dcBusVoltage;
    float gridDcCurrent;
    float inverterVoltage;
    float pv1Current;
    float pv2Current;
    float pv1Voltage;
    float pv2Voltage;
    float pv1Isolation;
    float pv2Isolation;
} ADC_RawMeanSqData;

/* Real values use volts, amperes and degrees Celsius. */
typedef struct
{
    float gridVoltage;
    float inductorCurrent;
    float gfciCurrent;
    float dcBusVoltage;
    float gridDcCurrent;
    float inverterVoltage;
    float pv1Current;
    float pv2Current;
    float pv1Voltage;
    float pv2Voltage;
    float pv1IsolationVoltage;
    float pv2IsolationVoltage;
    float inverterTemp;
    float boostTemp;
} ADC_RealData;

/* RMS is defined only for the linear electrical measurement channels. */
typedef struct
{
    float gridVoltage;
    float inductorCurrent;
    float gfciCurrent;
    float dcBusVoltage;
    float gridDcCurrent;
    float inverterVoltage;
    float pv1Current;
    float pv2Current;
    float pv1Voltage;
    float pv2Voltage;
    float pv1IsolationVoltage;
    float pv2IsolationVoltage;
} ADC_RealRmsData;

/* Each linear channel owns an independently adjustable zero and gain. */
typedef struct
{
    float offset;
    float gain;
} ADC_CalParam;



typedef struct
{
    ADC_CalParam gridVoltage;
    ADC_CalParam inductorCurrent;
    ADC_CalParam gfciCurrent;
    ADC_CalParam dcBusVoltage;
    ADC_CalParam gridDcCurrent;
    ADC_CalParam inverterVoltage;
    ADC_CalParam pv1Current;
    ADC_CalParam pv2Current;
    ADC_CalParam pv1Voltage;
    ADC_CalParam pv2Voltage;
    ADC_CalParam pv1Isolation;
    ADC_CalParam pv2Isolation;
} ADC_Calibrate;

/* ADCA RESULT0 through RESULT5 form one complete 20 kHz fast frame. */
typedef struct
{
    Uint16 inductorCurrent;
    Uint16 gridVoltage;
    Uint16 gfciCurrent;
    Uint16 dcBusVoltage;
    Uint16 gridDcCurrent;
    Uint16 inverterVoltage;
} ADC_FastRawFrame;

/* ADCB RESULT0 and RESULT1 form one synchronized PV-current frame. */
typedef struct
{
    Uint16 pv1Current;
    Uint16 pv2Current;
} ADC_PvCurrentRawFrame;

/* ADCD RESULT0 and RESULT1 form one synchronized PV-voltage frame. */
typedef struct
{
    Uint16 pv1Voltage;
    Uint16 pv2Voltage;
} ADC_PvVoltageRawFrame;

/* ADCD RESULT2 and RESULT3 form one slow isolation-monitoring frame. */
typedef struct
{
    Uint16 pv1Isolation;
    Uint16 pv2Isolation;
} ADC_IsolationRawFrame;

/* ADCC RESULT0 and RESULT1 form one slow temperature frame. */
typedef struct
{
    Uint16 inverterTemperature;
    Uint16 boostTemperature;
} ADC_TemperatureRawFrame;

/*
 * Central fault state for protection and control decisions.
 * Independent words are used while the fault list is still evolving; a
 * packed communication bitmap can be added later without exposing bitfields.
 */
typedef struct
{
    Uint16 pllFault;
    Uint16 tzFault;
} SysFault;

/*
 * Runtime machine data shared by acquisition, control and communication.
 * ECAP and PLL frequencies are stored in 0.01 Hz units.
 */
typedef struct
{
    ADC_RawData rawInstant;
    ADC_RawData rawAvg;
    ADC_RawMeanSqData rawMeanSq;
    ADC_RealData realInstant;
    ADC_RealData realAvg;
    ADC_RealRmsData realRms;
    Uint32 measureSeq;
    Uint16 ecapFreqCent;
    Uint16 pllFreqCent;
} MachineData;

extern volatile MachineData gMachineData;
extern volatile SysFault gSysFault;
extern volatile ADC_Calibrate gAdcCal;

#endif
