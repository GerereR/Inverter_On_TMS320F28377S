#include "F28x_Project.h"
#include "bsp.h"
#include "variable.h"
#include "scheduler.h"
#include "task.h"

/*
 * Configure the four F28377S ADCs for 12-bit single-ended conversion.
 *
 * The C2000Ware AdcSetMode() helper does two separate jobs: it writes the
 * mode bits and loads the device's OTP calibration. Keep both parts here so
 * this BSP does not depend on that helper while retaining factory accuracy.
 * The caller must hold EALLOW while this function runs.
 */
static void ADC_SetMode(void)
{
    Uint16 offsetTrim;

    /* Load per-module INL calibration when the device provides it. */
    if(*((Uint16 *)CalAdcaINL) != 0xFFFFU)
    {
        (*CalAdcaINL)();
    }
    if(*((Uint16 *)CalAdcbINL) != 0xFFFFU)
    {
        (*CalAdcbINL)();
    }
    if(*((Uint16 *)CalAdccINL) != 0xFFFFU)
    {
        (*CalAdccINL)();
    }
    if(*((Uint16 *)CalAdcdINL) != 0xFFFFU)
    {
        (*CalAdcdINL)();
    }

    /* The OTP offset table uses four entries per ADC; single-ended 12-bit is
     * the first mode entry for each module: ADCA=0, ADCB=4, ADCC=8, ADCD=12. */
    if(*((Uint16 *)GetAdcOffsetTrimOTP) != 0xFFFFU)
    {
        offsetTrim = (*GetAdcOffsetTrimOTP)(0U);
        if(offsetTrim != 0U)
        {
            AdcaRegs.ADCOFFTRIM.all = offsetTrim;
        }

        offsetTrim = (*GetAdcOffsetTrimOTP)(4U);
        if(offsetTrim != 0U)
        {
            AdcbRegs.ADCOFFTRIM.all = offsetTrim;
        }

        offsetTrim = (*GetAdcOffsetTrimOTP)(8U);
        if(offsetTrim != 0U)
        {
            AdccRegs.ADCOFFTRIM.all = offsetTrim;
        }

        offsetTrim = (*GetAdcOffsetTrimOTP)(12U);
        if(offsetTrim != 0U)
        {
            AdcdRegs.ADCOFFTRIM.all = offsetTrim;
        }
    }

    /* RESOLUTION=0 selects 12-bit; SIGNALMODE=0 selects single-ended. */
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0U;
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0U;
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0U;
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0U;
    AdccRegs.ADCCTL2.bit.RESOLUTION = 0U;
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0U;
    AdcdRegs.ADCCTL2.bit.RESOLUTION = 0U;
    AdcdRegs.ADCCTL2.bit.SIGNALMODE = 0U;

    /* F2837xS 12-bit linearity trim workaround: retain the upper half of
     * each trim register as required by the device reference implementation. */
    AdcaRegs.ADCINLTRIM1 &= 0xFFFF0000UL;
    AdcaRegs.ADCINLTRIM2 &= 0xFFFF0000UL;
    AdcaRegs.ADCINLTRIM4 &= 0xFFFF0000UL;
    AdcaRegs.ADCINLTRIM5 &= 0xFFFF0000UL;
    AdcbRegs.ADCINLTRIM1 &= 0xFFFF0000UL;
    AdcbRegs.ADCINLTRIM2 &= 0xFFFF0000UL;
    AdcbRegs.ADCINLTRIM4 &= 0xFFFF0000UL;
    AdcbRegs.ADCINLTRIM5 &= 0xFFFF0000UL;
    AdccRegs.ADCINLTRIM1 &= 0xFFFF0000UL;
    AdccRegs.ADCINLTRIM2 &= 0xFFFF0000UL;
    AdccRegs.ADCINLTRIM4 &= 0xFFFF0000UL;
    AdccRegs.ADCINLTRIM5 &= 0xFFFF0000UL;
    AdcdRegs.ADCINLTRIM1 &= 0xFFFF0000UL;
    AdcdRegs.ADCINLTRIM2 &= 0xFFFF0000UL;
    AdcdRegs.ADCINLTRIM4 &= 0xFFFF0000UL;
    AdcdRegs.ADCINLTRIM5 &= 0xFFFF0000UL;
}

//四个ADC模块配置函数
void ADC_Config(void)
{
    EALLOW;

    //四个 ADC 模块使用相同的 ADC 时钟预分频：PRESCALE=6
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6U;
    //单独配置ADC
    ADC_SetMode();
    //ADC自身中断配置,转换完成后中断
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    //ADC上电
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1U;

    EDIS;
    DELAY_US(1000);
    EALLOW;

    /*ADCA配置, 采样窗口均为14, 以下均由EPWM1 SOCA 触发*/
    // ADCA SOC0: 通道 14, 对应 I_Grid_Fin
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 14U;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    // ADCA SOC1: 通道 3, 对应 V_Grid_Fin
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    // ADCA SOC2: 通道 5, 对应 GFCI_Fin
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 5U;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    // ADCA SOC3: 通道 4, 对应 V_BUS_Fin
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 4U;
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    // ADCA SOC4: 通道 15, 对应 Idc_Grid_Fin
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 15U;
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    // ADCA SOC5: 通道 2, 对应 V_INV_Fin
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 2U;
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;

    /*ADCB配置, 采样窗口均为14, 以下均由EPWM1 SOCA 触发*/
    // ADCB SOC0: 通道 2, 对应 I_PV1_Fin
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2U;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    // ADCB SOC1: 通道 3, 对应 I_PV2_Fin
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;

    /*ADCC配置, 采样窗口均为14, 以下均由Timer1 100Hz触发*/
    // ADCC SOC0: 通道 4, 对应 TEMP_INV_Fin
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 4U;
    AdccRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_CPU_TIMER1;
    // ADCC SOC1: 通道 3, 对应 TEMP_BST_Fin
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdccRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_CPU_TIMER1;

    /*ADCD配置, 采样窗口均为14, 有的是EPWM触发, 有的是*Timer1触发*/
    // ADCD SOC0: 通道 4, 对应 V_PV1_Fin
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 4U;
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    // ADCD SOC1: 通道 3, 对应 V_PV2_Fin
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 3U;
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    // ADCD SOC2: 通道 1, 对应 ISO_PV1_Fin
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 1U;
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGGER_CPU_TIMER1;
    // ADCD SOC3: 通道 2, 对应 ISO_PV2_Fin
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 2U;
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = ADC_ACQUISITION_WINDOW;
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ADC_TRIGGER_CPU_TIMER1;
    //如果ePWM触发和Timer1同时触发,那就优先SCO0和SOC1
    AdcdRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 2U;


    /*ADCA 中断配置*/
    //SOC5 完成后触发 ADCA1 中断, 给电流环用
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5U;
    //非连续模式, 中断标志位不清除就阻塞
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;
    //使能中断
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1U;
    //SOC5 完成后触发 ADCA2 中断, 给DMA采集用
    AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 5U;
    //连续模式, 中断标志位不清除也不阻塞
    AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;
    //使能中断
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1U;

    /* ADCB中断配置 */
    //SOC1完成后触发ADCB2中断, 给DMA采集用, 连续模式
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1U;
    AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1U;

    /* ADCC中断配置 */
    //SOC1完成后触发ADCC2中断, 给DMA采集用, 连续模式
    AdccRegs.ADCINTSEL1N2.bit.INT2SEL = 1U;
    AdccRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;
    AdccRegs.ADCINTSEL1N2.bit.INT2E = 1U;

    /* ADCD中断配置 */
    //SOC1完成后触发ADCD1中断, 给DMA采集用, 连续模式
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1U;
    AdcdRegs.ADCINTSEL1N2.bit.INT1CONT = 1U;
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1U;
    //SOC3完成后触发ADCD2中断, 给DMA采集用, 连续模式
    AdcdRegs.ADCINTSEL1N2.bit.INT2SEL = 3U;
    AdcdRegs.ADCINTSEL1N2.bit.INT2CONT = 1U;
    AdcdRegs.ADCINTSEL1N2.bit.INT2E = 1U;

    //清除所有ADC的中断标志位
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT2 = 1U;

    //清除所有ADC的中断溢出标志位
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;
    AdccRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;
    AdcdRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    AdcdRegs.ADCINTOVFCLR.bit.ADCINT2 = 1U;

    //事实上, 我们只注册一个ADCA1的中断到PIE
    PieVectTable.ADCA1_INT = &ADCA1_CPU_ISR;
    //它位于PIE的1.1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1U;
    IER |= M_INT1;

    EDIS;
}

//ADC中断->电流环
__interrupt void ADCA1_CPU_ISR(void)
{
    Uint16 ctrlEvents;
    AC_CtrlRawInput rawInput;
    //其实它的任务很简单,就是统计当前"我采了多少次电流环数据"
    DMA_NotifyFastFrameEoc();
    //实际上,电流环只要这三个的数据
    rawInput.inductorCurrent = AdcaResultRegs.ADCRESULT0;
    rawInput.gridVoltage = AdcaResultRegs.ADCRESULT1;
    rawInput.dcBusVoltage = AdcaResultRegs.ADCRESULT3;
    //实际上,电流环任务是"假任务"它由ADC中断直接调度,不接受调度器调度
    ctrlEvents = Task_AC_Ctrl(&rawInput);
    //通知调度器电网到达峰值
    if((ctrlEvents & FAST_EVENT_GRID_PEAK) != 0U)
    {
        //直接通知调度器可以进行峰值任务,也就是限幅任务
        Scheduler_NotifyGridPeak();
    }
    //清除ADCA1的中断标志位和溢出标志位
    if(AdcaRegs.ADCINTOVF.bit.ADCINT1 != 0U)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    //通知PIE"我已处理好中断"
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
