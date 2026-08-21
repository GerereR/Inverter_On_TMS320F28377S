#include "F28x_Project.h"
#include "bsp.h"
#include "system.h"
#include "variable.h"

static volatile Uint16 ADC_GridVoltageBuffer[ADC_SAMPLE_BUFFER_SIZE];
static volatile Uint16 ADC_InductorCurrentBuffer[ADC_SAMPLE_BUFFER_SIZE];
static volatile Uint16 ADC_SampleIndex = 0U;
static volatile Uint32 ADC_SampleCount = 0UL;
volatile float OpenLoopInductorCurrentAmplitude = 0.8f;
static volatile float OpenLoopInductorCurrentReference = 0.0f;

void ADC_Config(void)
{
    EALLOW;

    // Power ADCA and ADCB in 12-bit single-ended mode with the same clock.
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1U;

    EDIS;
    DELAY_US(1000);
    EALLOW;

    // SOC0 samples grid voltage and SOC1 samples inductor current.
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0U;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14U;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 9U;

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1U;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14U;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 9U;

    // ADCB0 samples PV voltage and ADCB1 samples PV current.
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0U;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14U;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 9U;

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1U;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14U;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 9U;

    //以下是ADC-A中断配置
    // ADCINT1 drives the fast CPU control loop after SOC1 completes.
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1U;//EOC1触发ADCAINT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;//快环肯定不能连续
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1U;//使能中断

    // ADCINT2 is a continuous DMA trigger for the same two-result frame.
    AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 1U;//EOC1也触发ADCAINT2
    AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;//慢环肯定连续
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1U;

    //以下是ADC-B中断配置
    // ADCBINT2 continuously triggers DMA CH2 after both PV results are ready.
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1U;//EOC1触发ADCBINT2
    AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;//慢环使能连续
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1U;
    //总结:电网电压和电感电流同时触发快环和慢环,而PV电流和PV电压只触发慢环

    //清除中断标志和中断溢出标志
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;

    // Route ADCAINT1 to the CPU; DMA is initialized independently by System_Init.
    PieVectTable.ADCA1_INT = &ADCA1_CPU_ISR;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1U;
    IER |= M_INT1;

    EDIS;
}

__interrupt void ADCA1_CPU_ISR(void)
{
    Uint16 gridVoltageSample = AdcaResultRegs.ADCRESULT0;
    Uint16 inductorCurrentSample = AdcaResultRegs.ADCRESULT1;
    Uint16 sampleIndex = ADC_SampleIndex;
    float duty;

    // Keep a full-rate CPU history for monitoring and future feedback control.
    ADC_GridVoltageBuffer[sampleIndex] = gridVoltageSample;
    ADC_InductorCurrentBuffer[sampleIndex] = inductorCurrentSample;
    sampleIndex++;
    if(sampleIndex >= ADC_SAMPLE_BUFFER_SIZE)
    {
        sampleIndex = 0U;
    }
    ADC_SampleIndex = sampleIndex;
    ADC_SampleCount++;

    // Convert the grid-voltage sample to the normalized PLL input.
    SRF_PLL_Run(&GridSPLL, ((float)gridVoltageSample - 2048.0f) / 2048.0f);
    gMachineData.pllPhaseMilliradian = (Uint16)(GridSPLL.phase * 1000.0f);
    gMachineData.pllLocked =
        (GridSPLL.frequencyHz >= GridSPLL.minFrequencyHz &&
         GridSPLL.frequencyHz <= GridSPLL.maxFrequencyHz) ? 1U : 0U;

    //这里是未来放电流环的地方
    // The current-control stage is intentionally open-loop until calibration
    // and a current-feedback controller are defined.
    OpenLoopInductorCurrentReference = OpenLoopInductorCurrentAmplitude * GridSPLL.sine;
    duty = 0.5f * (1.0f + OpenLoopInductorCurrentReference);
    EPWM_SetDuty(duty);

    // Clear overflow before the interrupt flag so the next EOC is not lost.
    // 其实ADC的中断只有ADCAINT1触发,所有只清除它
    if(AdcaRegs.ADCINTOVF.bit.ADCINT1 != 0U)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
