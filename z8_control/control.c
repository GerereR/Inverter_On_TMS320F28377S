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

static Uint16 Ctrl_DetectGridPeak(void)
{
    static Uint16 phasePrimed = 0U;
    static float phasePrev = 0.0f;
    float phase;
    Uint16 peakCrossed = 0U;

    /* Do not phase-schedule DC control until both the external grid-period
     * measurement and the PLL have become valid. */
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

    /* With forward-running PLL phase, crossing pi/2 or 3*pi/2 identifies the
     * positive or negative voltage peak exactly once. A 2*pi wrap is not a
     * peak and therefore needs no special event. */
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

static void Ctrl_UpdatePllLock(float pllInput)
{
    static float inputAbsFiltered = 0.0f;
    static Uint16 lockCounter = 0U;
    static Uint16 unlockCounter = 0U;
    float inputAbs;
    float phaseErrorAbs;
    Uint16 lockCondition;
    Uint16 unlockCondition;

    inputAbs = (pllInput >= 0.0f) ? pllInput : -pllInput;
    phaseErrorAbs = (GridSPLL.notchOut >= 0.0f) ?
                    GridSPLL.notchOut : -GridSPLL.notchOut;
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
    /* The CPU/communication layer owns the requested amplitude. Startup must
     * not invent a nonzero current command. */
    gInvCtrlData.currentAmpCmd = 0.0f;
    gInvCtrlData.currentAmpApplied = 0.0f;
    gInvCtrlData.currentRef = 0.0f;
    gInvCtrlData.duty = 0.5f;
    gInvCtrlData.enabled = 0U;
}

void Ctrl_Enable(void)
{
    gInvCtrlData.currentAmpApplied = gInvCtrlData.currentAmpCmd;
    gInvCtrlData.enabled = 1U;
}

void Ctrl_Disable(void)
{
    /* Preserve currentAmpCmd so a CPU-provided request survives WAIT/FAULT;
     * only the value applied by the fast loop is forced to zero. */
    gInvCtrlData.enabled = 0U;
    gInvCtrlData.currentAmpApplied = 0.0f;
    gInvCtrlData.currentRef = 0.0f;
    gInvCtrlData.currentFeedback = 0.0f;
    gInvCtrlData.currentErr = 0.0f;
    gInvCtrlData.currentErrPrev = 0.0f;
    gInvCtrlData.piIntegral = 0.0f;
    gInvCtrlData.piOut = 0.0f;
    gInvCtrlData.duty = 0.5f;
}

void Ctrl_SetInductorCurrentAmp(float amp)
{
    gInvCtrlData.currentAmpCmd = amp;

    if(gInvCtrlData.enabled != 0U)
    {
        /* The temporary open-loop path applies an enabled command directly.
         * The closed-loop version can defer this copy to a zero crossing. */
        gInvCtrlData.currentAmpApplied = amp;
    }
}

float Ctrl_GetInductorCurrentAmp(void)
{
    return gInvCtrlData.currentAmpCmd;
}

Uint16 Ctrl_FastRun(float gridVoltPllIn)
{
    Uint16 events;
    float currentPhase;
    float currentRefSine;

    /* The closed-loop inductor-current controller is intentionally pending. */
    SOGI_PLL_Run(&GridSPLL, gridVoltPllIn);
    gMachineData.pllFreqCent = (Uint16)(GridSPLL.freqHz * 100.0f);
    Ctrl_UpdatePllLock(gridVoltPllIn);
    events = Ctrl_DetectGridPeak();

    if(gInvCtrlData.enabled != 0U)
    {
        /* The PLL owns only the grid phase. Reactive control supplies signed
         * phase offsets; the fast current loop creates its final waveform at
         * the point of use so every sample includes the latest compensation. */
        currentPhase = GridSPLL.phase +
                       gReactiveData.phaseShiftRad +
                       gReactiveData.capCompRad;
        currentPhase = Ctrl_WrapPhase(currentPhase);
        currentRefSine = __sinpuf32(currentPhase * MATH_INV_TWO_PI_F);
        gInvCtrlData.currentRef =
            gInvCtrlData.currentAmpApplied * currentRefSine;
        gInvCtrlData.duty = 0.5f * (1.0f + gInvCtrlData.currentRef);
    }
    else
    {
        gInvCtrlData.currentRef = 0.0f;
        gInvCtrlData.duty = 0.5f;
    }
    EPWM_SetDuty(gInvCtrlData.duty);

    return events;
}
