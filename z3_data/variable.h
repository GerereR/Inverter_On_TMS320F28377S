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
} ADC_UintData;

/* Converted values use volts, amperes and degrees Celsius. */
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
    float inverterTemperature;
    float boostTemperature;
} ADC_FloatData;

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

/* 运行时 ADC 零漂值（码值，采样时减去）。 */
typedef struct
{
    float inductorCurrent;
    float gridVoltage;
    float gfciCurrent;
    float gridDcCurrent;
} AdcOffsetValues;

/* ADC 运行时零漂校准（开机/重连时采 32 组码值平均，补偿温度漂移）。 */
typedef struct
{
    AdcOffsetValues offset;   /* 运行时零漂 */
    AdcOffsetValues sum;      /* 校准累积器 */
    Uint16 checkCount;        /* 校准采样计数 */
    Uint16 adInitial;         /* 校准标志：1=校准中 */
} AdcOffsetCal;

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
 * groups. Keep each group exactly 32 bits so its word view stays stable.
 */
typedef union
{
    Uint64 all;

    struct
    {
        Uint32 recoverable;
        Uint32 permanent;
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
        Uint16 reserved6 : 1;
        Uint16 reserved7 : 1;
        Uint16 pv1OverVolt : 1;
        Uint16 pv2OverVolt : 1;
        Uint16 pv1OverCurrent : 1;
        Uint16 pv2OverCurrent : 1;
        Uint16 inductorOverCurrent : 1;
        Uint16 gridDcCurrentFault : 1;
        Uint16 gfciFault : 1;
        Uint16 isolationFault : 1;

        /* Recoverable fault word, bits 16..31. */
        Uint16 noUtility : 1;
        Uint16 reserved17 : 1;
        Uint16 reserved18 : 1;
        Uint16 reserved19 : 1;
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

        /* Permanent fault word, bits 32..47. */
        Uint16 inverterOverTemp : 1;
        Uint16 boostOverTemp : 1;
        Uint16 adcFault : 1;
        Uint16 eepromFault : 1;
        Uint16 gfciDeviceFault : 1;   /* 原 GFCIDeviceFault：GFCI 自检硬件故障 */
        Uint16 dcBusOverVolt : 1;     /* 母线过压：permanent，Boost 失控/硬件损坏不可恢复 */
        Uint16 reserved38 : 1;
        Uint16 reserved39 : 1;
        Uint16 reserved40 : 1;
        Uint16 reserved41 : 1;
        Uint16 reserved42 : 1;
        Uint16 reserved43 : 1;
        Uint16 reserved44 : 1;
        Uint16 reserved45 : 1;
        Uint16 reserved46 : 1;
        Uint16 reserved47 : 1;

        /* Reserved, bits 48..63. */
        Uint16 reserved48 : 1;
        Uint16 reserved49 : 1;
        Uint16 reserved50 : 1;
        Uint16 reserved51 : 1;
        Uint16 reserved52 : 1;
        Uint16 reserved53 : 1;
        Uint16 reserved54 : 1;
        Uint16 reserved55 : 1;
        Uint16 reserved56 : 1;
        Uint16 reserved57 : 1;
        Uint16 reserved58 : 1;
        Uint16 reserved59 : 1;
        Uint16 reserved60 : 1;
        Uint16 reserved61 : 1;
        Uint16 reserved62 : 1;
        Uint16 reserved63 : 1;
    } bit;
} SysFault;

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
    Uint16 reloadFlag;    /* 打嗝保护标志：快速层(ISR)置位，状态机恢复 */
    Uint16 reloadCount;   /* 打嗝恢复计数（NORMAL 态累加，>150 即 300ms 后恢复） */
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

/* Runtime MPPT limits selected once from the installed inverter model. */
typedef struct
{
    float currentLimit;
    float minVoltage;
    float smallPowerDelta;
    float largePowerDelta;
} MpptChannelConfig;

/* Shared state for dual-input topology management and both MPPT trackers. */
typedef struct
{
    MpptChannelData pv1;
    MpptChannelData pv2;
    MpptChannelConfig pv1Config;
    MpptChannelConfig pv2Config;
    Uint16 inputMode;
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
    /* Normalized inverter-current amplitude command, range 0.0 to 1.0. */
    float currentAmpRef;
    /* Independent PV-voltage PI states for the two Boost channels. */
    float boost1VoltErr;
    float boost2VoltErr;
    float boost1PiIntegral;
    float boost2PiIntegral;
    float boost1Duty;
    float boost2Duty;
    Uint16 initialized;
    Uint16 softStartActive;
    Uint16 softStartStage;
    Uint32 softStartTimerMs;
} BusCtrlData;

/* Inverter current-loop command, feedback and diagnostic state.
 * The applied current amplitude comes from the bus loop directly
 * (gBusCtrlData.currentAmpRef).
 * currentRef/currentFeedback/currentErr and the compensation/limit fields
 * use centered ADC-code units. PI, feed-forward and modulation are normalized
 * bridge commands (-1..1). */
typedef struct
{
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
    Uint16 dciAdjCount;       /* 直流补偿激活计数（并网态每个电网周期 +1） */
} InvCtrlData;

/* Calculated grid/PV power and accumulated output energy. */
typedef struct
{
    float gridActivePower;
    float gridReactivePower;
    float gridApparentPower;
    float powerFactor;

    float pv1Power;
    float pv2Power;
    float totalPvPower;

    float totalEnergyWh;
    float dailyEnergyWh;
    float monthlyEnergyWh;
} PowerData;

/* Output command and the limits currently constraining delivered power. */
typedef struct
{
    float outputPowerCmd;
    float outputPowerLimit;
    float currentAmpLimit;
    float currentAmpMax;       /* SCI 手动设置的电流上限（0..1，默认满） */
    float thermalPowerLimit;
    float pvPowerLimit;
    float freqPowerLimit;
    Uint32 overloadTimerMs;
} PowerLimitData;

/*
 * Runtime machine data shared by acquisition, control and communication.
 * ADC data has two forms only: raw unsigned counts and converted
 * floating-point values. DMA mean-square intermediates use the same float
 * type and remain in centered ADC-count-squared units until RMS conversion.
 * Fields that are not meaningful for a given statistic remain zero.
 * ECAP and PLL frequencies are stored in 0.01 Hz units.
 */
typedef struct
{
    Uint32 measureSeq;

    ADC_UintData  rawInstant;
    ADC_FloatData realInstant;
    ADC_FloatData realAvg;
    ADC_FloatData realRms;

    PowerData powerData;

    Uint16 ecapFreqCent;
    Uint16 pllFreqCent;
} MachineData;

/* Grid qualification and long-window monitoring state. */
typedef struct
{
    float freqHz;
    float freqAvgHz;
    float voltageRmsAvg;
    float voltageRmsAvg10Min;
    float voltageRmsBuf[20];   /* 10 分钟窗口：20 槽，每槽 30 秒 */
    Uint16 voltageRmsBufIdx;   /* 环形索引 */
    Uint16 halfMinCnt;         /* 30 秒采样计数（电网周期数） */
    float fastPeakVolt;
    Uint32 periodTicks;
    Uint32 validCycleCount;
    Uint32 noGridCount;
    Uint16 gridPresent;
    Uint16 fastPresent;
    Uint16 voltageValid;
    Uint16 freqValid;
} GridMonitorData;

/* 并网安规参数（集中一个数据域，便于按国标/机型配置，将来可存 EEPROM）。 */
typedef struct
{
    float nomVoltRms;          /* 额定电压(V) */
    float nomFreqHz;           /* 额定频率(Hz) */

    float voltOverLevel1;      /* 一级过压(V) */
    float voltOverLevel2;      /* 二级过压(V) */
    float voltUnderLevel1;     /* 一级欠压(V) */
    float voltUnderLevel2;     /* 二级欠压(V) */

    float freqOverLevel1;      /* 一级过频(Hz) */
    float freqOverLevel2;      /* 二级过频(Hz) */
    float freqUnderLevel1;     /* 一级欠频(Hz) */
    float freqUnderLevel2;     /* 二级欠频(Hz) */

    float voltOver10Min;       /* 10 分钟平均过压(V) */

    float reconnMaxVolt;       /* 重连电压上限(V) */
    float reconnMinVolt;       /* 重连电压下限(V) */
    float reconnMaxFreq;       /* 重连频率上限(Hz) */
    float reconnMinFreq;       /* 重连频率下限(Hz) */

    Uint16 faultFilterCount1;  /* 一级判定计数 */
    Uint16 faultFilterCount2;  /* 二级判定计数 */
    Uint16 backFilterCount;    /* 恢复计数 */
} GridSafetyParams;

/* GFCI 漏电保护状态（自检 + 运行保护差分跳变 + 多级反时限）。 */
typedef struct
{
    float rmsBuf[3];         /* 最近 3 周期漏电流 RMS */
    float avgBuf[3];         /* 最近 3 周期漏电流平均值 */
    float deltaBaseAvg;         /* 突变基准（平均值，原 delta_base） */
    float deltaBaseRms;      /* 突变基准（RMS，原 delta_base_rms） */
    Uint16 breakFlag;        /* 突变标志（原 ubBreakFlag） */
    Uint16 breakSwFlag;      /* 突变基准锁定（bit0=30mA档, bit1=60mA档） */
    Uint16 noBreakCnt;       /* 无突变计数（原 gfci_cnt） */
    Uint16 filter30ma;       /* 30mA 档滤波 */
    Uint16 filter60ma;       /* 60mA 档滤波 */
    Uint16 filter300ma;      /* 300mA 档滤波 */
    Uint16 deviceFilter1;    /* 自检静态检测滤波（原 gfci_fault_filter1） */
    Uint16 deviceFilter2;    /* 自检注入检测滤波（原 gfci_fault_filter2） */
    Uint16 backFilter;       /* 恢复滤波 */
    Uint16 checkDelay;       /* 自检后保护静默期计数（原 wCheckGFCIDelay） */
    Uint16 selfTestIndex;    /* 自检计数（原 gfci_50ma_index） */
    Uint16 selfTestActive;   /* 自检进行中 */
} GfciData;

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

/* Grid-relay self-test state owned by system.c. */
typedef struct
{
    Uint32 timerMs;          /* 继电器时序计时（ms） */
    Uint16 relayOnFlag;      /* 原 RelayOnFlag：允许完成并网（1=允许，2=已吸合） */
    Uint16 faultFilter;      /* 粘连/失效连续判据（ms） */
    Uint16 selfTestPassed;   /* 自检通过 */
    Uint16 fault;            /* 继电器粘连/失效故障 */
} RelayCtrlData;

/* PLL state and algorithms belong to the control layer, not system startup. */
typedef struct
{
    float input;
    float phase;
    float freqHz;
    float nomFreqHz;
    float phaseDet;
    float notchOut;
    float piInt;
    float loopOut;
    float sampleTs;
    float kp;
    float ki;
    float minFreqHz;
    float maxFreqHz;
    float notchB0;
    float notchB1;
    float notchB2;
    float notchA1;
    float notchA2;
    float sogiAlpha;
    float sogiBeta;
    float sogiK;
    float detHist[3];
    float notchHist[3];
} SPLL_1ph;

extern volatile MachineData gMachineData;
extern volatile SysFault gSysFault;
extern volatile SysData gSysData;
extern volatile ADC_Calibrate gAdcCal;
extern volatile AdcOffsetCal gAdcOffsetCal;
extern volatile MpptData gMpptData;
extern volatile BusCtrlData gBusCtrlData;
extern volatile InvCtrlData gInvCtrlData;
extern volatile PowerLimitData gPowerLimitData;
extern volatile GridMonitorData gGridData;
extern volatile GridSafetyParams gGridSafety;
extern volatile GfciData gGfciData;
extern volatile ReactiveCtrlData gReactiveData;
extern volatile RelayCtrlData gRelayData;
extern volatile SPLL_1ph GridSPLL;

#endif
