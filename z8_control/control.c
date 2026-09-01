#include "F28x_Project.h"

#include "control.h"
#include "pll.h"
#include "bsp.h"
#include "variable.h"

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

static float Ctrl_ClampUnit(float value, float maxValue)
{
    if(value > maxValue)
    {
        return maxValue;
    }
    if(value < -maxValue)
    {
        return -maxValue;
    }
    return value;
}

static float Ctrl_WrapPhase(float phase)
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

static void Ctrl_ResetCurrentLoop(void)
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

static Uint16 Ctrl_DetectGridPeak(void)
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
        return CTRL_EVENT_NONE;
    }

    phase = GridSPLL.phase;
    if(phasePrimed == 0U)
    {
        phasePrev = phase;
        phasePrimed = 1U;
        return CTRL_EVENT_NONE;
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

    return (peakCrossed != 0U) ? CTRL_EVENT_GRID_PEAK : CTRL_EVENT_NONE;
}

static void Ctrl_UpdatePllLock(float pllInput)
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

void Ctrl_Init(void)
{
    /* Startup does not invent a nonzero current command. */
    gInvCtrlData.currentAmpCmd = 0.0f;
    gInvCtrlData.enabled = 0U;
    Ctrl_ResetCurrentLoop();
}

void Ctrl_Enable(void)
{
    /* Establish a clean controller state before the fast ISR can use it. */
    gInvCtrlData.enabled = 0U;
    Ctrl_ResetCurrentLoop();
    gInvCtrlData.currentAmpApplied = gInvCtrlData.currentAmpCmd;
    EPWM_SetInverterMode(0.0f);
    gInvCtrlData.enabled = 1U;
}

void Ctrl_Disable(void)
{
    /* Preserve the CPU command, but reset all applied loop state. */
    gInvCtrlData.enabled = 0U;
    Ctrl_ResetCurrentLoop();
}

void Ctrl_SetInductorCurrentAmp(float amp)
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

float Ctrl_GetInductorCurrentAmp(void)
{
    return gInvCtrlData.currentAmpCmd;
}

/* All three arguments are centered ADC values after total offset removal. */
Uint16 Ctrl_FastRun(float gridVoltAdc,
                    float inductorCurrentAdc,
                    float dcBusVoltAdc)
{
    Uint16 events;
    float gridVoltUnif;
    float currentPhase;
    float currentRefSine;
    float piIncrement;

    /* SOGI input is normalized to the centered 12-bit ADC half-range. */
    gridVoltUnif = gridVoltAdc / ADC_BIPOLAR_ZERO;

    SOGI_PLL_Run(&GridSPLL, gridVoltUnif);
    Ctrl_UpdatePllLock(gridVoltUnif);
    gMachineData.pllFreqCent = (Uint16)(GridSPLL.freqHz * 100.0f);
    events = Ctrl_DetectGridPeak();

    /* Feedback remains in centered ADC-code units to match the legacy loop. */
    gInvCtrlData.currentFeedback = inductorCurrentAdc;

    if(gInvCtrlData.enabled != 0U)
    {
        currentPhase = Ctrl_WrapPhase(GridSPLL.phase +
                                       gReactiveData.phaseShiftRad +
                                       gReactiveData.capCompRad);
        currentRefSine = __sinpuf32(currentPhase * MATH_INV_TWO_PI_F);
        gInvCtrlData.currentRef =
            gInvCtrlData.currentAmpApplied * ADC_BIPOLAR_ZERO *
            currentRefSine;

        /* This guard is only for a valid division denominator. Bus operating
         * range qualification belongs to the state machine. */
        if(dcBusVoltAdc > 0.0f)
        {
            gInvCtrlData.currentErrPrev = gInvCtrlData.currentErr;
            gInvCtrlData.currentErr = gInvCtrlData.currentRef -
                                      gInvCtrlData.currentFeedback;

            /* Incremental PI: scale the new increment before accumulating it. */
            piIncrement =
                (gInvCtrlData.currentErr *
                 (INV_CURRENT_KP + INV_CURRENT_KI) -
                 gInvCtrlData.currentErrPrev * INV_CURRENT_KP) *
                INV_LEGACY_BUS_GAIN /
                (dcBusVoltAdc * INV_PI_BUS_SCALE *
                 INV_LEGACY_PWM_PERIOD);
            gInvCtrlData.piOut =
                Ctrl_ClampUnit(gInvCtrlData.piOut + piIncrement, 1.0f);

            /* This ratio uses centered raw ADC values; 1.203 is the raw-domain
             * form of the legacy 0.8 physical feed-forward coefficient. */
            gInvCtrlData.gridVoltFeedForward =
                INV_GRID_FEED_FORWARD * gridVoltAdc / dcBusVoltAdc;
            gInvCtrlData.modulation =
                Ctrl_ClampUnit(gInvCtrlData.piOut +
                               gInvCtrlData.gridVoltFeedForward, 1.0f);
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
