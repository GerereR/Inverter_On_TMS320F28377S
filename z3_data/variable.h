#ifndef VARIABLE_H_
#define VARIABLE_H_

#include "constant.h"

//Uint16数据, 主要用来记录ADC码值数据
typedef struct
{
    Uint16 gridVolt;
    Uint16 inductCurr;
    Uint16 gfciCurr;
    Uint16 dcBusVolt;
    Uint16 gridDcCurr;
    Uint16 invertVolt;
    Uint16 pv1Curr;
    Uint16 pv2Curr;
    Uint16 pv1Volt;
    Uint16 pv2Volt;
    Uint16 pv1Insul;
    Uint16 pv2Insul;
    Uint16 invertTemp;
    Uint16 boostTemp;
} ADC_UintData;

//Float数据, 主要用来记录信号真实数据
typedef struct
{
    float gridVolt;
    float inductCurr;
    float gfciCurr;
    float dcBusVolt;
    float gridDcCurr;
    float invertVolt;
    float pv1Curr;
    float pv2Curr;
    float pv1Volt;
    float pv2Volt;
    float pv1Insul;
    float pv2Insul;
    float invertTemp;
    float boostTemp;
} ADC_FloatData;

/* Each linear channel owns an independently adjustable zero and gain. */
typedef struct
{
    float bias;
    float gain;
} AdcCalParam;

//ADC各信号调理值
typedef struct
{
    AdcCalParam gridVolt;
    AdcCalParam inductCurr;
    AdcCalParam gfciCurr;
    AdcCalParam dcBusVolt;
    AdcCalParam gridDcCurr;
    AdcCalParam invertVolt;
    AdcCalParam pv1Curr;
    AdcCalParam pv2Curr;
    AdcCalParam pv1Volt;
    AdcCalParam pv2Volt;
    AdcCalParam pv1Insul;
    AdcCalParam pv2Insul;
} AdcCal;

//ADC交流信号零漂值
typedef struct
{
    float inductCurr;
    float gridVolt;
    float gfciCurr;
    float gridDcCurr;
} AdcDriftParam;

//ADC 运行时零漂校准（开机/重连时采 32 组码值平均，补偿温度漂移）
typedef struct
{
    AdcDriftParam drift;    /* 运行时零漂 */
    AdcDriftParam sum;      /* 校准累积器 */
    Uint16 checkCnt;        /* 校准采样计数 */
    Uint16 adjInit;         /* 校准标志：1=校准中 */
} AdcDrift;

/* ADCA RESULT0 through RESULT5 form one complete 20 kHz fast frame. */
typedef struct
{
    Uint16 inductCurr;
    Uint16 gridVolt;
    Uint16 gfciCurr;
    Uint16 dcBusVolt;
    Uint16 gridDcCurr;
    Uint16 invertVolt;
} ADC_FastRawFrame;

/* ADCD RESULT0 through RESULT3 form one synchronized PV frame. */
typedef struct
{
    Uint16 pv1Volt;
    Uint16 pv1Curr;
    Uint16 pv2Volt;
    Uint16 pv2Curr;
} ADC_PvRawFrame;

/* ADCB RESULT0 and RESULT1 form one slow INSUL-Guarding frame. */
typedef struct
{
    Uint16 pv1Insul;
    Uint16 pv2Insul;
} ADC_InsulRawFrame;

/* ADCC RESULT0 and RESULT1 form one slow temperature frame. */
typedef struct
{
    Uint16 invertTemp;
    Uint16 boostTemp;
} ADC_TempRawFrame;

/* Machine problems are grouped by handling policy, not compiler bitfields. */
typedef struct
{
    Uint32 warning;
    Uint32 recovFault;
    Uint32 permaFault;
} SysProblem;

/* State-machine data shared by startup, protection and supervinsulry tasks. */
typedef struct
{
    SysState state;
    Uint16 model;
    SysCheckStage checkStage;
    Uint16 startReq;
    Uint16 sourceReady;
    Uint16 gridReady;
    Uint16 busReady;
    Uint32 sourceStableMs;
    Uint32 gridStableMs;
    Uint16 reloadFlag;    /* 打嗝保护标志：快速层(ISR)置位，状态机恢复 */
    Uint16 reloadCnt;   /* 打嗝恢复计数（NORMAL 态按状态任务周期累加） */
} SysData;

/* MPPT result consumed by the DC-control task. */
typedef struct
{
    float voltRef;
    Uint16 enabled;
} MpptChOutput;

/* MPPT results consumed outside task_mppt.c. */
typedef struct
{
    MpptChOutput pv1;
    MpptChOutput pv2;
    Uint16 inputMode;
} MpptData;

/* DC-control values consumed outside task_dc_ctrl.c. */
typedef struct
{
    float stableVoltRef;
    float currAmpRef;
    float boost1Duty;
    float boost2Duty;
} BusCtrlData;

/* Invert values shared with the slower AC Guard task. The curr-loop
 * working values remain private to task_ac_ctrl.c. */
typedef struct
{
    float dcCurrComp;
} InvCtrlData;

/* Cal power values consumed by power limiting and MPPT. */
typedef struct
{
    float gridActivePower;
    float gridReactivePower;
    float gridApparPower;
    float gridPF;
    float pv1Power;
    float pv2Power;
} PowerData;

/* Output command and the limits currly constraining delivered power. */
typedef struct
{
    float currAmpLim;
    float currAmpMax;       /* SCI 手动设置的电流上限（0..1，默认满） */
} PowerLimData;

/* 无功调度公共契约。
 * mode/setValue 是未来 Scalpel 的设定入口；phaseShiftPu 是电流环唯一消费的输出。
 * 电容系数、弧度中间量和诊断量均由无功任务私有持有，避免把实现细节扩散到全局域。 */
typedef struct
{
    /* 设定（通讯/Scalpel 将来写入） */
    Uint16 mode;            /* REACTIVE_MODE_OFF/PF/Q */
    float  setValue;        /* mode=PF: 带符号 cosφ(正=超前, 负=滞后); mode=Q: 带符号 Q(Var) */
    float  phaseShiftPu;    /* 总相移的周标幺值，电流环直接加进 PLL 相位 */
} ReactiveData;

/*
 * Runtime machine data shared by acquisition, control and communication.
 * ADC data has two forms only: raw unsigned cnts and converted
 * floating-point values. DMA mean-square intermediates use the same float
 * type and remain in cent ADC-cnt-squared units until RMS conversion.
 * Fields that are not meaningful for a given statistic remain zero.
 * ECAP and PLL frequencies are stored in 0.01 Hz units.
 */
typedef struct
{
    //测量任务心跳, 说明数据是否更新, 或者漏掉
    Uint32 measuSeq;

    ADC_FloatData realAvg;
    ADC_FloatData realRms;

    PowerData powerData;

    Uint16 ecapFreqCent;
    Uint16 pllFreqCent;
} MachineData;

/* Fast grid-presence state shared by the AC control and state tasks. */
typedef struct
{
    Uint32 noGridCnt;
    Uint16 fastPresent;
} GridGuardData;

/* 并网安规参数（集中一个数据域，便于按国标/机型配置，将来可存 EEPROM）。 */
typedef struct
{
    float voltOverLV1;      /* 一级过压(V) */
    float voltOverLV2;      /* 二级过压(V) */
    float voltUnderLV1;     /* 一级欠压(V) */
    float voltUnderLV2;     /* 二级欠压(V) */

    float freqOverLV1;      /* 一级过频(Hz) */
    float freqOverLV2;      /* 二级过频(Hz) */
    float freqUnderLV1;     /* 一级欠频(Hz) */
    float freqUnderLV2;     /* 二级欠频(Hz) */

    float voltOver10Min;       /* 10 分钟平均过压(V) */

    float reconnMaxVolt;       /* 重连电压上限(V) */
    float reconnMinVolt;       /* 重连电压下限(V) */
    float reconnMaxFreq;       /* 重连频率上限(Hz) */
    float reconnMinFreq;       /* 重连频率下限(Hz) */

    Uint16 faultFiltCnt1;  /* 一级判定计数 */
    Uint16 faultFiltCnt2;  /* 二级判定计数 */
    Uint16 backFiltCnt;    /* 恢复计数 */
} GridSafetyParam;

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
    Uint16 filt30ma;       /* 30mA 档滤波 */
    Uint16 filt60ma;       /* 60mA 档滤波 */
    Uint16 filt300ma;      /* 300mA 档滤波 */
    Uint16 deviceFilt1;    /* 自检静态检测滤波（原 gfci_fault_filt1） */
    Uint16 deviceFilt2;    /* 自检注入检测滤波（原 gfci_fault_filt2） */
    Uint16 backFilt;       /* 恢复滤波 */
    Uint16 checkDelay;       /* 自检后保护静默期计数（原 wCheckGFCIDelay） */
    Uint16 selfTestIdx;    /* 自检计数（原 gfci_50ma_index） */
    Uint16 selfTestActive;   /* 自检进行中 */
} GfciData;

/* PLL state and algorithms belong to the control layer, not system startup. */
typedef struct
{
    float input;
    float phase;
    float freqHz;
    float nomFreqHz;
    float phaseDet;
    float piInt;
    float loopOut;
    float sampleTs;
    float kp;
    float ki;
    float minFreqHz;
    float maxFreqHz;
    float notchOut;
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
} PllData;

extern volatile MachineData     gMachineData;
extern volatile SysProblem      gSysProblem;
extern volatile SysData         gSysData;
extern volatile AdcCal          gAdcCal;
extern volatile AdcDrift        gAdcDrift;
extern volatile MpptData        gMpptData;
extern volatile BusCtrlData     gBusCtrlData;
extern volatile InvCtrlData     gInvCtrlData;
extern volatile PowerLimData    gPowerLimData;
extern volatile ReactiveData    gReactiveData;
extern volatile GridGuardData   gGridData;
extern volatile GridSafetyParam gGridSafety;
extern volatile GfciData        gGfciData;
extern volatile PllData         gPllData;

#endif
