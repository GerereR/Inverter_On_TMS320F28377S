#include "F28x_Project.h"
#include "bsp.h"
#include "system.h"
#include "variable.h"

volatile float InductorCurrentAmp_temporal = 0.8f;
static volatile float InductorCurrentRef_temporal = 0.0f;

/* PLL锁定门限均基于归一化输入,后续应结合实机波形和SCI记录继续整定。 */
#define PLL_INPUT_ABS_FILTER_COEFF       0.001f
#define PLL_LOCK_INPUT_ABS_MIN           0.10f
#define PLL_UNLOCK_INPUT_ABS_MIN         0.05f
#define PLL_LOCK_PHASE_ERROR_MAX         0.05f
#define PLL_UNLOCK_PHASE_ERROR_MAX       0.10f
#define PLL_LOCK_FREQ_MARGIN_HZ          0.25f
#define PLL_UNLOCK_FREQ_MARGIN_HZ        0.05f
#define PLL_LOCK_CONFIRM_SAMPLES         2000U  /* 20kHz下连续100ms合格后判定锁定 */
#define PLL_UNLOCK_CONFIRM_SAMPLES       200U   /* 20kHz下连续10ms异常后判定失锁 */

static void ADC_UpdateGridPllLock(float pllInput)
{
    static float inputAbsFiltered = 0.0f;
    static Uint16 lockCounter = 0U;
    static Uint16 unlockCounter = 0U;
    float inputAbs;
    float phaseErrorAbs;
    Uint16 lockCondition;
    Uint16 unlockCondition;

    inputAbs = (pllInput >= 0.0f) ? pllInput : -pllInput;
    phaseErrorAbs = (GridSPLL.notchOutput >= 0.0f) ?
                    GridSPLL.notchOutput : -GridSPLL.notchOutput;
    inputAbsFiltered += PLL_INPUT_ABS_FILTER_COEFF *
                        (inputAbs - inputAbsFiltered);

    lockCondition =
        (inputAbsFiltered >= PLL_LOCK_INPUT_ABS_MIN) &&
        (phaseErrorAbs <= PLL_LOCK_PHASE_ERROR_MAX) &&
        (GridSPLL.frequencyHz > (GridSPLL.minFrequencyHz + PLL_LOCK_FREQ_MARGIN_HZ)) &&
        (GridSPLL.frequencyHz < (GridSPLL.maxFrequencyHz - PLL_LOCK_FREQ_MARGIN_HZ));

    unlockCondition =
        (inputAbsFiltered < PLL_UNLOCK_INPUT_ABS_MIN) ||
        (phaseErrorAbs > PLL_UNLOCK_PHASE_ERROR_MAX) ||
        (GridSPLL.frequencyHz <= (GridSPLL.minFrequencyHz + PLL_UNLOCK_FREQ_MARGIN_HZ)) ||
        (GridSPLL.frequencyHz >= (GridSPLL.maxFrequencyHz - PLL_UNLOCK_FREQ_MARGIN_HZ));

    if(gSysFault.pllFault != 0U)
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
                gSysFault.pllFault = 0U;
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
                gSysFault.pllFault = 1U;
                unlockCounter = 0U;
            }
        }
        else
        {
            unlockCounter = 0U;
        }
    }
}

void ADC_Config(void)
{
    EALLOW;

    /* All four ADC modules use 12-bit single-ended conversion at the same clock. */
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    /* Generate ADC interrupt pulses after result registers are updated. */
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1U;

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1U;

    EDIS;
    DELAY_US(1000);
    EALLOW;

    /*
     * ADCA fast frame, triggered once per 20 kHz PWM period:
     * RESULT0 I_Grid_Fin, RESULT1 V_Grid_Fin, RESULT2 GFCI_Fin,
     * RESULT3 V_BUS_Fin, RESULT4 Idc_Grid_Fin, RESULT5 V_INV_Fin.
     */
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 14U;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 5U;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 4U;
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 15U;
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 2U;
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;

    /* ADCB records the synchronized PV1/PV2 current pair at 20 kHz. */
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2U;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;

    /* ADCD RESULT0/1 records the synchronized PV1/PV2 voltage pair at 20 kHz. */
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 4U;
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM3_SOCA;

    /* ADCD RESULT2/3 records PV1/PV2 isolation signals at 100 Hz. */
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 1U;
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGGER_CPU_TIMER1;
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 2U;
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ADC_TRIGGER_CPU_TIMER1;

    /* SOC0 and SOC1 retain priority when fast and slow ADCD triggers overlap. */
    AdcdRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 2U;

    /* ADCC records inverter and boost temperatures at 100 Hz. */
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 4U;
    AdccRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_CPU_TIMER1;
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdccRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_CPU_TIMER1;

    /* ADCA EOC5 drives both the fast CPU loop and DMA CH1. */
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5U;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1U;
    AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 5U;
    AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1U;

    /* The remaining ADC interrupts are continuous DMA trigger sources only. */
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1U;
    AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1U;
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1U;
    AdcdRegs.ADCINTSEL1N2.bit.INT1CONT = 1U;
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1U;
    AdcdRegs.ADCINTSEL1N2.bit.INT2SEL = 3U;
    AdcdRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;
    AdcdRegs.ADCINTSEL1N2.bit.INT2E = 1U;
    AdccRegs.ADCINTSEL1N2.bit.INT2SEL = 1U;
    AdccRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;
    AdccRegs.ADCINTSEL1N2.bit.INT2E = 1U;

    /* Start every interrupt source from a known, non-overflowed state. */
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdccRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdcdRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    AdcdRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;

    PieVectTable.ADCA1_INT = &ADCA1_CPU_ISR;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1U;
    IER |= M_INT1;

    /* Timer1 runs freely at 100 Hz and triggers ADC SOCs without a CPU ISR. */
    CpuTimer1Regs.TCR.bit.TSS = 1U;
    CpuTimer1Regs.PRD.all = ADC_SLOW_TRIGGER_COUNTS - 1UL;
    CpuTimer1Regs.TPR.all = 0U;
    CpuTimer1Regs.TPRH.all = 0U;
    CpuTimer1Regs.TCR.bit.TIE = 0U;
    CpuTimer1Regs.TCR.bit.TIF = 1U;
    CpuTimer1Regs.TCR.bit.TRB = 1U;
    CpuTimer1Regs.TCR.bit.TSS = 0U;

    EDIS;
}

__interrupt void ADCA1_CPU_ISR(void)
{
    Uint16 gridVoltageRawSample;
    float gridVoltagePllInput;

    /* The control loop consumes the current ADC frame directly. */
    gridVoltageRawSample = AdcaResultRegs.ADCRESULT1;

    /* Convert the grid-voltage sample to the normalized PLL input. */
    gridVoltagePllInput = ((float)gridVoltageRawSample - gAdcCal.gridVoltage.offset) / ADC_BIPOLAR_ZERO;
    //SRF_PLL_Run(&GridSPLL, gridVoltagePllInput);
    SOGI_PLL_Run(&GridSPLL, gridVoltagePllInput);
    gMachineData.pllFreqCent = (Uint16)(GridSPLL.frequencyHz * 100.0f);
    ADC_UpdateGridPllLock(gridVoltagePllInput);

    /* Open-loop placeholder until calibrated current feedback is implemented. */
    InductorCurrentRef_temporal = InductorCurrentAmp_temporal * GridSPLL.sine;
    EPWM_SetDuty(0.5f * (1.0f + InductorCurrentRef_temporal));

    /* Clear the fast-loop request after all six results have been consumed. */
    if(AdcaRegs.ADCINTOVF.bit.ADCINT1 != 0U)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
