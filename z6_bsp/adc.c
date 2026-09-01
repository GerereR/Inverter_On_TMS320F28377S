#include "F28x_Project.h"
#include "bsp.h"
#include "variable.h"
#include "scheduler.h"
#include "../z8_control/control.h"

static void ADC_UpdateGridPresence(Uint16 gridVoltRaw)
{
    static Uint16 windowSamples = 0U;
    static float windowPeakVolt = 0.0f;
    float gridVolt;

    /* This is only a fast loss-of-grid indication. RMS qualification and
     * reconnect timing remain in Task_State(). */
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
        gGridData.fastPeakVolt = windowPeakVolt;
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

/* The fast ISR normalizes grid voltage before handing it to the control layer. */
void ADC_Config(void)
{
    EALLOW;

    /* All four ADC modules use 12-bit single-ended conversion at the same clock. */
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);//实在是有点长,直接用driver库了
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    /* Generate ADC interrupt pulses after result registers are updated. */
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1U;//ADC自身中断配置,转换完成后中断
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1U;

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1U;//ADC上电
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
     //采样顺序配置
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 14U;//ADC采样通道,要和引脚对应
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;//采样窗口
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;//触发源
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 5U;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 4U;
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 15U;
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 2U;
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;

    /* ADCB records the synchronized PV1/PV2 current pair at 20 kHz. */
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2U;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;

    /* ADCD RESULT0/1 records the synchronized PV1/PV2 voltage pair at 20 kHz. */
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 4U;
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;

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
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5U;//SOCx完成后触发中断
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;//非连续模式
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1U;//启用中断
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
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;//清除中断标志
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;//清除中断溢出标志
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
    Uint16 gridVoltRaw;
    float inductorCurrentHalfRaw;
    float gridVoltHalfRaw;
    float dcBusVoltHalfRaw;
    Uint16 ctrlEvents;

    /* The control loop consumes the current ADC frame directly. */
    gridVoltRaw = AdcaResultRegs.ADCRESULT1;
    inductorCurrentHalfRaw = (float)AdcaResultRegs.ADCRESULT0 - gAdcCal.inductorCurrent.offset;
    gridVoltHalfRaw = (float)gridVoltRaw - gAdcCal.gridVoltage.offset;
    dcBusVoltHalfRaw = (float)AdcaResultRegs.ADCRESULT3 - gAdcCal.dcBusVoltage.offset;//其实是否用BUS瞬时值,有待商榷,因为这样的话BUS瞬变会导致电流环不稳定

    /* ADCINT2 triggers DMA from the same EOC5 event. Count this ADC frame so
     * eCAP can close the active DMA block at the next grid-cycle boundary. */
    DMA_NotifyFastFrameEoc();

    ADC_UpdateGridPresence(gridVoltRaw);

    ctrlEvents = Ctrl_FastRun(gridVoltHalfRaw, inductorCurrentHalfRaw, dcBusVoltHalfRaw);
    if((ctrlEvents & CTRL_EVENT_GRID_PEAK) != 0U)
    {
        Scheduler_NotifyGridPeak();
    }

    /* The CPU consumed grid voltage; DMA handles the six-result data frame.
     * Clear only the CPU fast-loop interrupt request here. */
    if(AdcaRegs.ADCINTOVF.bit.ADCINT1 != 0U)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
