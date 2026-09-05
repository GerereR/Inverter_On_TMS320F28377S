#include "F28x_Project.h"

#include "task.h"
#include "constant.h"
#include "bsp.h"
#include "variable.h"
#include "system.h"

/*============================================================================
 * task_fast.c —— 电流环假任务（非调度器触发，由 ADC ISR 直接调用）
 *
 * 合并原 z8_control/pll.c + current_ctrl.c：
 *   - PLL：SRF/SOGI 单相锁相环
 *   - 电流环：50us 快速闭环（PLL + 电流环 PI + PWM 更新 + 峰值检测）
 *==========================================================================*/

/* --- PLL（原 pll.c） --- */
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

static void SRF_PLL_Init(volatile SPLL_1ph *pll,
                  float nomFreqHz,
                  float sampleFreqHz)
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

static void SOGI_PLL_Init(volatile SPLL_1ph *pll,
                   float nomFreqHz,
                   float sampleFreqHz)
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

static void SRF_PLL_Run(volatile SPLL_1ph *pll, float input)
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

static void SOGI_PLL_Run(volatile SPLL_1ph *pll, float input)
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

/* --- 电流环（原 current_ctrl.c，接口改名 Fast_*） --- */

/* Temporary PLL lock thresholds for the current bring-up stage. */
#define PLL_INPUT_ABS_FILTER_COEFF       0.001f
#define PLL_LOCK_INPUT_ABS_MIN           0.10f
#define PLL_UNLOCK_INPUT_ABS_MIN         0.05f
#define PLL_LOCK_PHASE_ERROR_MAX         0.05f
#define PLL_UNLOCK_PHASE_ERROR_MAX       0.10f
#define PLL_LOCK_FREQ_MARGIN_HZ          0.25f
#define PLL_UNLOCK_FREQ_MARGIN_HZ        0.05f
#define PLL_LOCK_CONFIRM_SAMPLES         2000U
#define PLL_UNLOCK_CONFIRM_SAMPLES       200U

/* Legacy 20 kHz current-loop tuning. The PI result is normalized below, so
 * the present F28377S TBPRD does not change these discrete coefficients. */
#define INV_CURRENT_KP                    300.0f
#define INV_CURRENT_KI                     30.0f
#define INV_LEGACY_BUS_GAIN              6935.0f
#define INV_LEGACY_PWM_PERIOD            1500.0f
#define INV_PI_BUS_SCALE                 1024.0f

/* With centered raw ADC values, 1.203 is the raw-domain equivalent of the
 * legacy physical feed-forward coefficient 0.8 after the voltage gain ratio. */
#define INV_GRID_FEED_FORWARD             1.203f

static float Fast_WrapPhase(float phase)
{
    while(phase >= MATH_TWO_PI_F)
    {
        phase -= MATH_TWO_PI_F;
    }
    while(phase < 0.0f)
    {
        phase += MATH_TWO_PI_F;
    }
    return phase;
}

static void Fast_ResetCurrentLoop(void)
{
    gInvCtrlData.currentAmpApplied = 0.0f;
    gInvCtrlData.currentRef = 0.0f;
    gInvCtrlData.currentFeedback = 0.0f;
    gInvCtrlData.currentErr = 0.0f;
    gInvCtrlData.currentErrPrev = 0.0f;
    gInvCtrlData.piIntegral = 0.0f;
    gInvCtrlData.piOut = 0.0f;
    gInvCtrlData.gridVoltFeedForward = 0.0f;
    gInvCtrlData.modulation = 0.0f;
}

static Uint16 Fast_DetectGridPeak(void)
{
    static Uint16 phasePrimed = 0U;
    static float phasePrev = 0.0f;
    float phase;
    Uint16 peakCrossed = 0U;

    /* Phase scheduling is valid only after eCAP and PLL have become valid. */
    if((gMachineData.ecapFreqCent == 0U) ||
       (gSysFault.bit.pllFault != 0U))
    {
        phasePrimed = 0U;
        return FAST_EVENT_NONE;
    }

    phase = GridSPLL.phase;
    if(phasePrimed == 0U)
    {
        phasePrev = phase;
        phasePrimed = 1U;
        return FAST_EVENT_NONE;
    }

    /* Crossing pi/2 or 3*pi/2 identifies each voltage peak once. */
    if(phase >= phasePrev)
    {
        if(((phasePrev < MATH_HALF_PI_F) &&
            (phase >= MATH_HALF_PI_F)) ||
           ((phasePrev < MATH_THREE_HALF_PI_F) &&
            (phase >= MATH_THREE_HALF_PI_F)))
        {
            peakCrossed = 1U;
        }
    }
    phasePrev = phase;

    return (peakCrossed != 0U) ? FAST_EVENT_GRID_PEAK : FAST_EVENT_NONE;
}

static void Fast_UpdatePllLock(float pllInput)
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
    phaseErrorAbs = (GridSPLL.phaseDet >= 0.0f) ?
                    GridSPLL.phaseDet : -GridSPLL.phaseDet;
    inputAbsFiltered += PLL_INPUT_ABS_FILTER_COEFF *
                        (inputAbs - inputAbsFiltered);

    lockCondition =
        (inputAbsFiltered >= PLL_LOCK_INPUT_ABS_MIN) &&
        (phaseErrorAbs <= PLL_LOCK_PHASE_ERROR_MAX) &&
        (GridSPLL.freqHz > (GridSPLL.minFreqHz + PLL_LOCK_FREQ_MARGIN_HZ)) &&
        (GridSPLL.freqHz < (GridSPLL.maxFreqHz - PLL_LOCK_FREQ_MARGIN_HZ));

    unlockCondition =
        (inputAbsFiltered < PLL_UNLOCK_INPUT_ABS_MIN) ||
        (phaseErrorAbs > PLL_UNLOCK_PHASE_ERROR_MAX) ||
        (GridSPLL.freqHz <= (GridSPLL.minFreqHz + PLL_UNLOCK_FREQ_MARGIN_HZ)) ||
        (GridSPLL.freqHz >= (GridSPLL.maxFreqHz - PLL_UNLOCK_FREQ_MARGIN_HZ));

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

void Fast_Init(void)
{
    /* PLL 初始化收进电流环假任务，main 不再直接接触 PLL。 */
    SOGI_PLL_Init(&GridSPLL, 50.0f, 20000.0f);

    /* Startup does not invent a nonzero current command. */
    gInvCtrlData.currentAmpCmd = 0.0f;
    gInvCtrlData.enabled = 0U;
    /* Until the power-limit manager is active, allow the full normalized
     * current range. A later zero limit must remain effective. */
    gPowerLimitData.currentAmpLimit = BUS_CURRENT_AMP_MAX_NORM;
    Fast_ResetCurrentLoop();
}

void Fast_Enable(void)
{
    /* Establish a clean controller state before the fast ISR can use it. */
    gInvCtrlData.enabled = 0U;
    Fast_ResetCurrentLoop();
    gInvCtrlData.currentAmpApplied = gInvCtrlData.currentAmpCmd;
    EPWM_SetInverterMode(0.0f);
    gInvCtrlData.enabled = 1U;
}

void Fast_Disable(void)
{
    /* Preserve the CPU command, but reset all applied loop state. */
    gInvCtrlData.enabled = 0U;
    Fast_ResetCurrentLoop();
}

void Fast_SetCurrentAmp(float amp)
{
    /* The external command is a normalized peak-current request. */
    if(amp < 0.0f)
    {
        amp = 0.0f;
    }
    else if(amp > 1.0f)
    {
        amp = 1.0f;
    }

    gInvCtrlData.currentAmpCmd = amp;
    if(gInvCtrlData.enabled != 0U)
    {
        gInvCtrlData.currentAmpApplied = amp;
    }
}

float Fast_GetCurrentAmp(void)
{
    return gInvCtrlData.currentAmpCmd;
}

/* All three arguments are centered ADC values after total offset removal. */
Uint16 Fast_Run(float gridVoltAdc, float inductorCurrentAdc, float dcBusVoltAdc)
{
    Uint16 events;
    float gridVoltUnif;
    float currentPhase;
    float currentRefSine;
    float piIncrement;

    /* SOGI input is normalized to the centered 12-bit ADC half-range. */
    gridVoltUnif = gridVoltAdc / ADC_BIPOLAR_ZERO;

    SOGI_PLL_Run(&GridSPLL, gridVoltUnif);
    Fast_UpdatePllLock(gridVoltUnif);
    gMachineData.pllFreqCent = (Uint16)(GridSPLL.freqHz * 100.0f);
    events = Fast_DetectGridPeak();

    /* Feedback remains in centered ADC-code units to match the legacy loop. */
    gInvCtrlData.currentFeedback = inductorCurrentAdc;

    if(gInvCtrlData.enabled != 0U)
    {
        currentPhase = Fast_WrapPhase(GridSPLL.phase + gReactiveData.phaseShiftRad + gReactiveData.capCompRad);
        currentRefSine = __sinpuf32(currentPhase * MATH_INV_TWO_PI_F);
        gInvCtrlData.currentRef = gInvCtrlData.currentAmpApplied * ADC_BIPOLAR_ZERO * currentRefSine;

        /* This guard is only for a valid division denominator. Bus operating
         * range qualification belongs to the state machine. */
        if(dcBusVoltAdc > 0.0f)
        {
            gInvCtrlData.currentErrPrev = gInvCtrlData.currentErr;
            gInvCtrlData.currentErr = gInvCtrlData.currentRef - gInvCtrlData.currentFeedback;

            /* Incremental PI: scale the new increment before accumulating it. */
            piIncrement =
                (gInvCtrlData.currentErr *
                 (INV_CURRENT_KP + INV_CURRENT_KI) -
                 gInvCtrlData.currentErrPrev * INV_CURRENT_KP) *
                INV_LEGACY_BUS_GAIN /
                (dcBusVoltAdc * INV_PI_BUS_SCALE *
                 INV_LEGACY_PWM_PERIOD);
            gInvCtrlData.piOut =
                System_Clamp(gInvCtrlData.piOut + piIncrement, -1.0f, 1.0f);

            gInvCtrlData.gridVoltFeedForward = INV_GRID_FEED_FORWARD * gridVoltAdc / dcBusVoltAdc;
            gInvCtrlData.modulation = System_Clamp(
                gInvCtrlData.piOut + gInvCtrlData.gridVoltFeedForward,
                -1.0f,
                1.0f);
        }
        else
        {
            /* Do not let an invalid denominator produce a PWM command. */
            gInvCtrlData.currentErr = 0.0f;
            gInvCtrlData.currentErrPrev = 0.0f;
            gInvCtrlData.piOut = 0.0f;
            gInvCtrlData.gridVoltFeedForward = 0.0f;
            gInvCtrlData.modulation = 0.0f;
        }

        EPWM_SetInverterMode(gInvCtrlData.modulation);
    }
    else
    {
        gInvCtrlData.currentRef = 0.0f;
        gInvCtrlData.currentErr = 0.0f;
        gInvCtrlData.currentErrPrev = 0.0f;
        gInvCtrlData.piOut = 0.0f;
        gInvCtrlData.gridVoltFeedForward = 0.0f;
        gInvCtrlData.modulation = 0.0f;
    }

    return events;
}