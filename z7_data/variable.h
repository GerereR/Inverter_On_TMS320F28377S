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
} ADC_RawMeanSq;

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
} ADC_Real;

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
} ADC_RealRms;

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
 * Fault bits can be accessed individually or checked as recoverable/permanent
 * groups. Keep each group exactly 16 bits so its word view stays stable.
 */
typedef union
{
    Uint32 all;

    struct
    {
        Uint16 recoverable;
        Uint16 permanent;
    } word;

    struct
    {
        /* Recoverable fault word, bits 0..15. */
        Uint16 pllFault : 1;
        Uint16 tzFault : 1;
        Uint16 gridOverVolt : 1;
        Uint16 gridUnderVolt : 1;
        Uint16 gridOverFreq : 1;
        Uint16 gridUnderFreq : 1;
        Uint16 dcBusOverVolt : 1;
        Uint16 dcBusUnderVolt : 1;
        Uint16 pv1OverVolt : 1;
        Uint16 pv2OverVolt : 1;
        Uint16 pv1OverCurrent : 1;
        Uint16 pv2OverCurrent : 1;
        Uint16 inductorOverCurrent : 1;
        Uint16 gridDcCurrentFault : 1;
        Uint16 gfciFault : 1;
        Uint16 isolationFault : 1;

        /* Permanent fault word, bits 16..31. */
        Uint16 inverterOverTemp : 1;
        Uint16 boostOverTemp : 1;
        Uint16 adcFault : 1;
        Uint16 eepromFault : 1;
        Uint16 reserved20 : 1;
        Uint16 reserved21 : 1;
        Uint16 reserved22 : 1;
        Uint16 reserved23 : 1;
        Uint16 reserved24 : 1;
        Uint16 reserved25 : 1;
        Uint16 reserved26 : 1;
        Uint16 reserved27 : 1;
        Uint16 reserved28 : 1;
        Uint16 reserved29 : 1;
        Uint16 reserved30 : 1;
        Uint16 reserved31 : 1;
    } bit;
} SysFault;

/*
 * Runtime machine data shared by acquisition, control and communication.
 * ECAP and PLL frequencies are stored in 0.01 Hz units.
 */
typedef struct
{
    ADC_RawData rawInstant;
    ADC_Real realInstant;
    ADC_Real realAvg;
    ADC_RealRms realRms;
    Uint32 measureSeq;
    Uint16 ecapFreqCent;
    Uint16 pllFreqCent;
} MachineData;

/* State-machine data shared by startup, protection and supervisory tasks. */
typedef struct
{
    SysState state;
    Uint16 model;
    SysCheckStage checkStage;
    Uint16 startRequest;
    Uint16 sourceReady;
    Uint16 gridReady;
    Uint16 busReady;
    Uint16 boostReady;
    Uint16 inverterReady;
    Uint16 relayReady;
    Uint16 restartRequest;
    Uint32 sourceStableMs;
    Uint32 gridStableMs;
    Uint32 restartCount;
} SysData;

/* One PV input's MPPT history and Boost command. */
typedef struct
{
    float voltRef;
    float voltRefPrev;
    float openCircuitVolt;
    float power;
    float powerPrev;
    float voltStep;
    float duty;
    float dutyPrev;
    int16 direction;
    Uint16 enabled;
    Uint16 fastSearch;
} MpptChannelData;

/* Shared state for dual-input topology management and both MPPT trackers. */
typedef struct
{
    MpptChannelData pv1;
    MpptChannelData pv2;
    Uint16 inputMode;
    Uint16 masterChannel;
    Uint16 topologyStage;
    Uint16 topologyCount;
    Uint16 powerAvgCount;
} MpptData;

/* DC-bus outer-loop and Boost soft-start state. */
typedef struct
{
    float voltRef;
    float stableVoltRef;
    float softStartVoltRef;
    float voltErr;
    float voltErrPrev;
    float piIntegral;
    float piOut;
    float piOutPrev;
    float currentAmpRef;
    float boost1Duty;
    float boost2Duty;
    Uint16 initialized;
    Uint16 softStartActive;
    Uint16 softStartStage;
    Uint32 softStartTimerMs;
} BusCtrlData;

/* Inverter current-loop command, feedback and diagnostic state.
 * currentAmpCmd/currentAmpApplied are normalized peak commands (0..1).
 * currentRef/currentFeedback/currentErr and the compensation/limit fields
 * use centered ADC-code units. PI, feed-forward and modulation are normalized
 * bridge commands (-1..1). */
typedef struct
{
    float currentAmpCmd;
    float currentAmpApplied;
    float currentRef;
    float currentFeedback;
    float currentErr;
    float currentErrPrev;
    float piIntegral;
    float piOut;
    float gridVoltFeedForward;
    float modulation;
    float dcCurrentComp;
    float currentLimit;
    Uint16 enabled;
    Uint16 zeroCrossUpdatePending;
} InvCtrlData;

/* Calculated power, energy and supervisory derating limits. */
typedef struct
{
    float gridActivePower;
    float gridReactivePower;
    float gridApparentPower;
    float powerFactor;
    float pv1Power;
    float pv2Power;
    float totalPvPower;
    float energyWh;
    float outputPowerCmd;
    float outputPowerLimit;
    float currentAmpLimit;
    float thermalPowerLimit;
    float pvPowerLimit;
    float freqPowerLimit;
    Uint32 calcCount;
    Uint32 overloadTimerMs;
} PowerData;

/* Grid qualification and long-window monitoring state. */
typedef struct
{
    float freqHz;
    float freqAvgHz;
    float voltageRmsAvg;
    float voltageRmsAvg10Min;
    float fastPeakVolt;
    Uint32 periodTicks;
    Uint32 validCycleCount;
    Uint32 noGridCount;
    Uint16 gridPresent;
    Uint16 fastPresent;
    Uint16 voltageValid;
    Uint16 freqValid;
} GridMonitorData;

/* Reactive-power command and phase compensation shared with control.
 * Both phase offsets are signed final current-reference offsets: positive
 * leads the PLL grid phase and negative lags it. */
typedef struct
{
    Uint16 mode;
    float powerFactorCmd;
    float reactivePowerCmd;
    float phaseShiftRad;
    float capCompRad;
    float activePowerLimit;
    float curvePowerLow;
    float curvePowerHigh;
    float curvePfLow;
    float curvePfHigh;
} ReactiveCtrlData;

extern volatile MachineData gMachineData;
extern volatile SysFault gSysFault;
extern volatile SysData gSysData;
extern volatile ADC_Calibrate gAdcCal;
extern volatile MpptData gMpptData;
extern volatile BusCtrlData gBusCtrlData;
extern volatile InvCtrlData gInvCtrlData;
extern volatile PowerData gPowerData;
extern volatile GridMonitorData gGridData;
extern volatile ReactiveCtrlData gReactiveData;

#endif
