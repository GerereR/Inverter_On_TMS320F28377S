#include "F28x_Project.h"

#include "task.h"
#include "constant.h"
#include "bsp.h"
#include "variable.h"
#include "system.h"

/* 前置声明：电网电压瞬时异常检测（定义见文件末尾）。 */
static void CheckGridVoltAbnormal(float gridVoltAdc);

/* Fast ADC-domain grid-presence check. A 205-sample window is about one
 * half-cycle at the 20 kHz control rate used by this project. */
#define GRID_FAST_PRESENT_PEAK_V      180.0f
#define GRID_FAST_WINDOW_SAMPLES        205U

//PLL宏定义
#define SPLL_MAX_DEVIATION_HZ       5.0f

#define SRF_PLL_DEFAULT_KP          60.0f
#define SRF_PLL_DEFAULT_KI          2000.0f

#define SOGI_PLL_DEFAULT_KP         90.0f
#define SOGI_PLL_DEFAULT_KI         4000.0f

#define SRF_PLL_DEFAULT_NOTCH_B0    1.3853181f
#define SRF_PLL_DEFAULT_NOTCH_B1   -2.7692690f
#define SRF_PLL_DEFAULT_NOTCH_B2    1.3853181f
#define SRF_PLL_DEFAULT_NOTCH_A1   -1.9590329f
#define SRF_PLL_DEFAULT_NOTCH_A2    0.9604000f

/* Temporary PLL lock thresholds for the current bring-up stage. */
#define PLL_INPUT_ABS_FILTER_COEFF       0.001f

#define PLL_LOCK_INPUT_ABS_MIN           0.10f
#define PLL_UNLOCK_INPUT_ABS_MIN         0.05f

#define PLL_LOCK_PHASE_ERROR_MAX         0.05f
#define PLL_UNLOCK_PHASE_ERROR_MAX       0.10f

#define PLL_LOCK_FREQ_MARGIN_HZ          0.25f
#define PLL_UNLOCK_FREQ_MARGIN_HZ        0.05f

#define PLL_LOCK_CONFIRM_SAMPLES         2000U
#define PLL_UNLOCK_CONFIRM_SAMPLES       200U   //50us*200=10ms

/* Legacy 20 kHz current-loop tuning. The PI result is normalized below, so
 * the present F28377S TBPRD does not change these discrete coefficients. */
#define INV_CURRENT_KP                    300.0f
#define INV_CURRENT_KI                     30.0f

#define INV_LEGACY_BUS_GAIN              6935.0f

#define INV_PI_BUS_SCALE                 1024.0f

#define INV_LEGACY_PWM_PERIOD            1500.0f

/* With centered raw ADC values, 1.203 is the raw-domain equivalent of the
 * legacy physical feed-forward coefficient 0.8 after the voltage gain ratio. */
#define INV_GRID_FEED_FORWARD             1.203f

typedef struct
{
    float currentRef;
    float currentFeedback;
    float currentErr;
    float currentErrPrev;
    float piOut;
    float gridVoltFeedForward;
    float modulation;
    Uint16 enabled;
} AC_CtrlState;

static volatile AC_CtrlState AC_State = {0};

static void SRF_PLL_Init(volatile PLL_Data *pll, float nomFreqHz, float sampleFreqHz)
{
    Uint16 index;

    pll->input = 0.0f;
    pll->phase = 0.0f;
    pll->freqHz = nomFreqHz;
    pll->nomFreqHz = nomFreqHz;
    pll->phaseDet = 0.0f;
    pll->notchOut = 0.0f;
    pll->piInt = 0.0f;
    pll->loopOut = 0.0f;
    pll->sampleTs = 1.0f / sampleFreqHz;
    pll->kp = SRF_PLL_DEFAULT_KP;
    pll->ki = SRF_PLL_DEFAULT_KI;
    pll->minFreqHz = nomFreqHz - SPLL_MAX_DEVIATION_HZ;
    pll->maxFreqHz = nomFreqHz + SPLL_MAX_DEVIATION_HZ;

    pll->notchB0 = SRF_PLL_DEFAULT_NOTCH_B0;
    pll->notchB1 = SRF_PLL_DEFAULT_NOTCH_B1;
    pll->notchB2 = SRF_PLL_DEFAULT_NOTCH_B2;
    pll->notchA1 = SRF_PLL_DEFAULT_NOTCH_A1;
    pll->notchA2 = SRF_PLL_DEFAULT_NOTCH_A2;
    pll->sogiAlpha = 0.0f;
    pll->sogiBeta = 0.0f;
    pll->sogiK = 1.41421356f;

    for(index = 0U; index < 3U; index++)
    {
        pll->detHist[index] = 0.0f;
        pll->notchHist[index] = 0.0f;
    }
}

static void SOGI_PLL_Init(volatile PLL_Data *pll, float nomFreqHz, float sampleFreqHz)
{
    Uint16 index;

    pll->input = 0.0f;
    pll->phase = 0.0f;
    pll->freqHz = nomFreqHz;
    pll->nomFreqHz = nomFreqHz;
    pll->phaseDet = 0.0f;
    pll->notchOut = 0.0f;
    pll->piInt = 0.0f;
    pll->loopOut = 0.0f;
    pll->sampleTs = 1.0f / sampleFreqHz;
    pll->kp = SOGI_PLL_DEFAULT_KP;
    pll->ki = SOGI_PLL_DEFAULT_KI;
    pll->minFreqHz = nomFreqHz - SPLL_MAX_DEVIATION_HZ;
    pll->maxFreqHz = nomFreqHz + SPLL_MAX_DEVIATION_HZ;

    for(index = 0U; index < 3U; index++)
    {
        pll->detHist[index] = 0.0f;
        pll->notchHist[index] = 0.0f;
    }

    pll->sogiAlpha = 0.0f;
    pll->sogiBeta = 0.0f;
    pll->sogiK = 1.41421356f;
}

static void SRF_PLL_Run(volatile PLL_Data *pll, float input)
{
    float omega;
    float maxCorrection;
    float phasePu;
    float phaseCos;

    pll->input = input;
    phasePu = pll->phase * MATH_INV_TWO_PI_F;
    phaseCos = __cospuf32(phasePu);
    pll->detHist[0] = input * phaseCos;

    pll->notchHist[0] =
        -pll->notchA1 * pll->notchHist[1]
        -pll->notchA2 * pll->notchHist[2]
        +pll->notchB0 * pll->detHist[0]
        +pll->notchB1 * pll->detHist[1]
        +pll->notchB2 * pll->detHist[2];

    pll->phaseDet = pll->detHist[0];
    pll->notchOut = pll->notchHist[0];

    pll->piInt += pll->ki * pll->sampleTs * pll->notchOut;
    maxCorrection = MATH_TWO_PI_F * (pll->maxFreqHz - pll->nomFreqHz);
    if(pll->piInt > maxCorrection)
    {
        pll->piInt = maxCorrection;
    }
    else if(pll->piInt < -maxCorrection)
    {
        pll->piInt = -maxCorrection;
    }
    pll->loopOut = pll->kp * pll->notchOut + pll->piInt;

    omega = MATH_TWO_PI_F * pll->nomFreqHz + pll->loopOut;
    if(omega < MATH_TWO_PI_F * pll->minFreqHz)
    {
        omega = MATH_TWO_PI_F * pll->minFreqHz;
    }
    else if(omega > MATH_TWO_PI_F * pll->maxFreqHz)
    {
        omega = MATH_TWO_PI_F * pll->maxFreqHz;
    }
    pll->freqHz = omega * MATH_INV_TWO_PI_F;

    pll->phase += omega * pll->sampleTs;
    while(pll->phase >= MATH_TWO_PI_F)
    {
        pll->phase -= MATH_TWO_PI_F;
    }
    while(pll->phase < 0.0f)
    {
        pll->phase += MATH_TWO_PI_F;
    }

    pll->detHist[2] = pll->detHist[1];
    pll->detHist[1] = pll->detHist[0];
    pll->notchHist[2] = pll->notchHist[1];
    pll->notchHist[1] = pll->notchHist[0];
}

static void SOGI_PLL_Run(volatile PLL_Data *pll, float input)
{
    float omega;
    float error;
    float alphaOld;
    float vq;
    float maxCorrection;
    float phasePu;
    float phaseSin;
    float phaseCos;

    pll->input = input;
    omega = MATH_TWO_PI_F * pll->freqHz;
    phasePu = pll->phase * MATH_INV_TWO_PI_F;
    phaseSin = __sinpuf32(phasePu);
    phaseCos = __cospuf32(phasePu);

    error = input - pll->sogiAlpha;
    alphaOld = pll->sogiAlpha;
    pll->sogiAlpha = alphaOld +
                     pll->sampleTs *
                     (pll->sogiK * omega * error - omega * pll->sogiBeta);
    pll->sogiBeta += pll->sampleTs * (omega * alphaOld);

    vq = pll->sogiAlpha * phaseCos + pll->sogiBeta * phaseSin;
    pll->phaseDet = vq;

    pll->piInt += pll->ki * pll->sampleTs * vq;
    maxCorrection = MATH_TWO_PI_F * (pll->maxFreqHz - pll->nomFreqHz);
    if(pll->piInt > maxCorrection)
    {
        pll->piInt = maxCorrection;
    }
    else if(pll->piInt < -maxCorrection)
    {
        pll->piInt = -maxCorrection;
    }
    pll->loopOut = pll->kp * vq + pll->piInt;

    omega = MATH_TWO_PI_F * pll->nomFreqHz + pll->loopOut;
    if(omega < MATH_TWO_PI_F * pll->minFreqHz)
    {
        omega = MATH_TWO_PI_F * pll->minFreqHz;
    }
    else if(omega > MATH_TWO_PI_F * pll->maxFreqHz)
    {
        omega = MATH_TWO_PI_F * pll->maxFreqHz;
    }
    pll->freqHz = omega * MATH_INV_TWO_PI_F;

    pll->phase += omega * pll->sampleTs;
    while(pll->phase >= MATH_TWO_PI_F)
    {
        pll->phase -= MATH_TWO_PI_F;
    }
    while(pll->phase < 0.0f)
    {
        pll->phase += MATH_TWO_PI_F;
    }
}


//之所以峰值检测放在这里,是因为我把ADC的快采放到电流环里面去了
static Uint16 DetectGridPeak(void)
{
    static Uint16 phasePrimed = 0U;
    static float phasePrev = 0.0f;
    float phase;
    Uint16 peakCrossed = 0U;

    /* Phase scheduling is valid only after eCAP and PLL have become valid. */
    if((gMachineData.ecapFreqCent == 0U) || (gSysFault.bit.pllFault != 0U))
    {
        phasePrimed = 0U;
        return FAST_EVENT_NONE;
    }

    phase = GridPLL.phase;
    if(phasePrimed == 0U)
    {
        phasePrev = phase;
        phasePrimed = 1U;
        return FAST_EVENT_NONE;
    }

    /* Crossing pi/2 or 3*pi/2 identifies each voltage peak once. */
    if(phase >= phasePrev)
    {
        if
        (
            ((phasePrev < MATH_HALF_PI_F) &&
            (phase >= MATH_HALF_PI_F)) ||
           ((phasePrev < MATH_THREE_HALF_PI_F) &&
            (phase >= MATH_THREE_HALF_PI_F))
        )
        {
            peakCrossed = 1U;
        }
    }
    phasePrev = phase;

    return (peakCrossed != 0U) ? FAST_EVENT_GRID_PEAK : FAST_EVENT_NONE;
}

static void UpdatePllLock(float pllInput)
{
    float inputAbs;
    float phaseErrorAbs;
    static float inputAbsFiltered = 0.0f;
    static Uint16 lockCounter = 0U;
    static Uint16 unlockCounter = 0U;
    Uint16 lockCondition;
    Uint16 unlockCondition;

    inputAbs = (pllInput >= 0.0f) ? pllInput : -pllInput;

    /* SOGI-PLL stores its q-axis phase error in phaseDet. */
    phaseErrorAbs = (GridPLL.phaseDet >= 0.0f) ? GridPLL.phaseDet : -GridPLL.phaseDet;
    inputAbsFiltered += PLL_INPUT_ABS_FILTER_COEFF * (inputAbs - inputAbsFiltered);

    lockCondition =
        (inputAbsFiltered >= PLL_LOCK_INPUT_ABS_MIN) &&
        (phaseErrorAbs <= PLL_LOCK_PHASE_ERROR_MAX) &&
        (GridPLL.freqHz > (GridPLL.minFreqHz + PLL_LOCK_FREQ_MARGIN_HZ)) &&
        (GridPLL.freqHz < (GridPLL.maxFreqHz - PLL_LOCK_FREQ_MARGIN_HZ));

    unlockCondition =
        (inputAbsFiltered < PLL_UNLOCK_INPUT_ABS_MIN) ||
        (phaseErrorAbs > PLL_UNLOCK_PHASE_ERROR_MAX) ||
        (GridPLL.freqHz <= (GridPLL.minFreqHz + PLL_UNLOCK_FREQ_MARGIN_HZ)) ||
        (GridPLL.freqHz >= (GridPLL.maxFreqHz - PLL_UNLOCK_FREQ_MARGIN_HZ));

    if(gSysFault.bit.pllFault != 0U)
    {
        unlockCounter = 0U;
        if(lockCondition != 0U)
        {
            if(lockCounter < PLL_LOCK_CONFIRM_SAMPLES)
            {
                lockCounter++;
            }
            if(lockCounter >= PLL_LOCK_CONFIRM_SAMPLES)
            {
                gSysFault.bit.pllFault = 0U;
                lockCounter = 0U;
            }
        }
        else
        {
            lockCounter = 0U;
        }
    }
    else
    {
        lockCounter = 0U;
        if(unlockCondition != 0U)
        {
            if(unlockCounter < PLL_UNLOCK_CONFIRM_SAMPLES)
            {
                unlockCounter++;
            }
            if(unlockCounter >= PLL_UNLOCK_CONFIRM_SAMPLES)
            {
                gSysFault.bit.pllFault = 1U;
                unlockCounter = 0U;
            }
        }
        else
        {
            unlockCounter = 0U;
        }
    }
}

static void AC_Ctrl_ResetCurrentLoop(void)
{
    AC_State.currentRef = 0.0f;
    AC_State.currentFeedback = 0.0f;
    AC_State.currentErr = 0.0f;
    AC_State.currentErrPrev = 0.0f;
    AC_State.piOut = 0.0f;
    AC_State.gridVoltFeedForward = 0.0f;
    AC_State.modulation = 0.0f;
}

void AC_Ctrl_Enable(void)
{
    /* Establish a clean controller state before the fast ISR can use it. */
    AC_State.enabled = 0U;
    AC_Ctrl_ResetCurrentLoop();
    
    EPWM_SetInverterMode(0.0f);
    AC_State.enabled = 1U;
}

void AC_Ctrl_Disable(void)
{
    /* Preserve the CPU command, but reset all applied loop state. */
    AC_State.enabled = 0U;
    AC_Ctrl_ResetCurrentLoop();
}

/* 快速电网存在性检测（原 adc.c 的 ADC_CheckGridPresence，迁入快速层）。
 * 在 20kHz ISR 里对电网电压做峰值窗口判断，给出快速掉网指示。
 * 正式 RMS 判定与恢复时序仍在 Task_AcMonitor()/状态机里。 */
void CheckGridPresence(Uint16 gridVoltRaw)
{
    static Uint16 windowSamples = 0U;
    static float windowPeakVolt = 0.0f;
    float gridVolt;

    gridVolt = ((float)gridVoltRaw - gAdcCal.gridVoltage.offset) * gAdcCal.gridVoltage.gain;
    if(gridVolt < 0.0f)
    {
        gridVolt = -gridVolt;
    }
    if(gridVolt > windowPeakVolt)
    {
        windowPeakVolt = gridVolt;
    }

    windowSamples++;
    if(windowSamples >= GRID_FAST_WINDOW_SAMPLES)
    {
        if(windowPeakVolt >= GRID_FAST_PRESENT_PEAK_V)
        {
            gGridData.fastPresent = 1U;
            gGridData.noGridCount = 0UL;
        }
        else
        {
            gGridData.fastPresent = 0U;
            if(gGridData.noGridCount < 0xFFFFFFFFUL)
            {
                gGridData.noGridCount++;
            }
        }

        windowSamples = 0U;
        windowPeakVolt = 0.0f;
    }
}

/* 电网电压瞬时值异常检测（原 adc_isr.c 的打嗝保护）。
 * 并网态下，电网电压瞬时值若比母线折算值高 15V 以上，连续 3 次则打嗝封波。
 * 0.663 为母线到电网的调制比折算系数，需按实际硬件校准。 */
static void CheckGridVoltAbnormal(float gridVoltAdc)
{
    static Uint16 abnormalCount = 0U;
    float gridVoltV;
    float threshold;

    if((gSysData.state != SYS_STATE_NORMAL) || (gSysData.reloadFlag != 0U))
    {
        abnormalCount = 0U;
        return;
    }

    /* 电网电压物理值（V）；阈值 = 母线电压 × 0.663（调制比）+ 15V 裕量 */
    gridVoltV = gridVoltAdc * gAdcCal.gridVoltage.gain;
    threshold = gBusCtrlData.stableVoltRef * 0.663f + 15.0f;

    if((gridVoltV > threshold) || (gridVoltV < -threshold))
    {
        abnormalCount++;
    }
    else
    {
        abnormalCount = 0U;
    }

    if(abnormalCount >= 3U)
    {
        abnormalCount = 0U;
        gSysData.reloadFlag = 1U;
        EPWM_Disable();
    }
}

void AC_Ctrl_Init(void)
{
    /* PLL 初始化收进电流环假任务，main 不再直接接触 PLL。 */
    SOGI_PLL_Init(&GridPLL, 50.0f, 20000.0f);

    AC_State.enabled = 0U;
    /* Until the power-limit manager is active, allow the full normalized
     * current range. A later zero limit must remain effective. */
    gPowerLimitData.currentAmpLimit = BUS_CURRENT_AMP_MAX_NORM;
    gPowerLimitData.currentAmpMax = BUS_CURRENT_AMP_MAX_NORM;
    AC_Ctrl_ResetCurrentLoop();
}

/* 电流环假任务入口：由 ADC ISR 直接调用。
 * 内部读取 ADC 结果、去零漂、做快速电网存在性检测，然后跑 PLL + 电流环。 */
Uint16 Task_AC_Ctrl(void)
{
    Uint16 gridVoltRaw;
    float gridVoltAdc;
    float inductorCurrentAdc;
    float dcBusVoltAdc;
    Uint16 events;
    float gridVoltUnif;
    float currentPhase;
    float currentRefSine;
    float refMaxCode;
    float piIncrement;

    /* 读取当前 ADC 帧并去除零漂（原 ISR 中的采样逻辑收进任务）。 */
    gridVoltRaw = AdcaResultRegs.ADCRESULT1;
    inductorCurrentAdc = (float)AdcaResultRegs.ADCRESULT0 - gAdcCal.inductorCurrent.offset - gAdcOffsetCal.offset.inductorCurrent;
    gridVoltAdc = (float)gridVoltRaw - gAdcCal.gridVoltage.offset - gAdcOffsetCal.offset.gridVoltage;
    dcBusVoltAdc = (float)AdcaResultRegs.ADCRESULT3 - gAdcCal.dcBusVoltage.offset;//其实是否用BUS瞬时值,有待商榷,因为这样的话BUS瞬变会导致电流环不稳定

    /* 快速电网存在性检测（快速掉网指示）。 */
    CheckGridPresence(gridVoltRaw);

    /* 电网电压瞬时值异常 -> 打嗝保护。 */
    CheckGridVoltAbnormal(gridVoltAdc);

    /* SOGI input is normalized to the centered 12-bit ADC half-range. */
    gridVoltUnif = gridVoltAdc / ADC_BIPOLAR_ZERO;

    SOGI_PLL_Run(&GridPLL, gridVoltUnif);
    UpdatePllLock(gridVoltUnif);

    gMachineData.pllFreqCent = (Uint16)(GridPLL.freqHz * 100.0f);
    events = DetectGridPeak();

    /* Feedback remains in centered ADC-code units to match the legacy loop. */
    AC_State.currentFeedback = inductorCurrentAdc;

    /* 打嗝保护期间不输出电流环。 */
    if((AC_State.enabled != 0U) && (gSysData.reloadFlag == 0U))
    {
        currentPhase = GridPLL.phase;
        while(currentPhase >= MATH_TWO_PI_F)
        {
            currentPhase -= MATH_TWO_PI_F;
        }
        while(currentPhase < 0.0f)
        {
            currentPhase += MATH_TWO_PI_F;
        }
        /* 电流参考：幅值 × sin；正半周幅值减直流分量补偿（原 DCcurrentAdj），
         * 负半周不变，从而产生反向直流抵消电网电流里的直流分量。 */
        currentRefSine = __sinpuf32(currentPhase * MATH_INV_TWO_PI_F);
        if(currentRefSine > 0.0f)
        {
            refMaxCode = gBusCtrlData.currentAmpRef * ADC_BIPOLAR_ZERO - gInvCtrlData.dcCurrentComp;
        }
        else
        {
            refMaxCode = gBusCtrlData.currentAmpRef * ADC_BIPOLAR_ZERO;
        }
        AC_State.currentRef = refMaxCode * currentRefSine;

        /* This guard is only for a valid division denominator. Bus operating
         * range qualification belongs to the state machine. */
        if(dcBusVoltAdc > 0.0f)
        {
            AC_State.currentErrPrev = AC_State.currentErr;
            AC_State.currentErr = AC_State.currentRef - AC_State.currentFeedback;

            /* Incremental PI: scale the new increment before accumulating it. */
            piIncrement =
                (AC_State.currentErr *
                 (INV_CURRENT_KP + INV_CURRENT_KI) -
                 AC_State.currentErrPrev * INV_CURRENT_KP) *
                INV_LEGACY_BUS_GAIN /
                (dcBusVoltAdc * INV_PI_BUS_SCALE *
                 INV_LEGACY_PWM_PERIOD);
            AC_State.piOut = System_Clamp(AC_State.piOut + piIncrement, -1.0f, 1.0f);

            AC_State.gridVoltFeedForward = INV_GRID_FEED_FORWARD * gridVoltAdc / dcBusVoltAdc;
            AC_State.modulation = System_Clamp(AC_State.piOut+AC_State.gridVoltFeedForward,-1.0f,1.0f);
        }
        else
        {
            /* Do not let an invalid denominator produce a PWM command. */
            AC_State.currentErr = 0.0f;
            AC_State.currentErrPrev = 0.0f;
            AC_State.piOut = 0.0f;
            AC_State.gridVoltFeedForward = 0.0f;
            AC_State.modulation = 0.0f;
        }

        EPWM_SetInverterMode(AC_State.modulation);
    }
    else
    {
        AC_State.currentRef = 0.0f;
        AC_State.currentErr = 0.0f;
        AC_State.currentErrPrev = 0.0f;
        AC_State.piOut = 0.0f;
        AC_State.gridVoltFeedForward = 0.0f;
        AC_State.modulation = 0.0f;
    }

    return events;
}
