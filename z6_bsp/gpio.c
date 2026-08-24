#include "F28x_Project.h"
#include "bsp.h"

// Keep all pin mux, direction, pull-up, and qualification settings here.
// Peripheral register settings remain in their corresponding BSP source files.

void GPIO_Config(void)
{
    EALLOW;

    // LED1: GPIO1, active low.
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;

    // LED2: GPIO2, active low.
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;

    // LED3: GPIO3, active low.
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;

    // EPWM3A/EPWM3B: GPIO4/GPIO5.
    // EPWM_Config() must run first with TBCLKSYNC disabled. The pins are muxed
    // only after action qualifier and dead-band registers have valid values.
    // EPWM_Start() releases TBCLKSYNC after all peripheral setup is complete.
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;

    // ECAP1 input: asynchronous GPIO19 routed through INPUT X-BAR 7.
    GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;
    InputXbarRegs.INPUT7SELECT = 19;

    // SCIA: GPIO28 = RX, GPIO29 = TX. These are the controlCARD defaults.
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO28 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO29 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;

    // I2CA: GPIO32 = SDA, GPIO33 = SCL. External pull-ups are still required.
    // 说明:
    //   - GPBPUD=0:使能内部上拉(仅弱上拉,400kHz 下通常仍需外部 4.7k 上拉)
    //   - GPBQSEL1=3:异步输入、无去抖——I2C 需要亚微秒级信号沿,不能加滤波
    //   - GMUX=0/MUX=1:选中 I2CA 外设功能(GPIO32/33 为 F28377S 的 I2CA 默认引脚)
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO33 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;

    EDIS;
}


// LED outputs are active low: clearing the GPIO output turns an LED on.
void LED_Ctrl(Uint16 ledNumber, Uint16 state)
{
    Uint32 ledMask;

    switch(ledNumber)
    {
        case LED_NUMBER_1:
            ledMask = (1UL << 1U);
            break;
        case LED_NUMBER_2:
            ledMask = (1UL << 2U);
            break;
        case LED_NUMBER_3:
            ledMask = (1UL << 3U);
            break;
        default:
            return;
    }

    switch(state)
    {
        case LED_STATE_ON:
            GpioDataRegs.GPACLEAR.all = ledMask;
            break;
        case LED_STATE_OFF:
            GpioDataRegs.GPASET.all = ledMask;
            break;
        case LED_STATE_TOGGLE:
            GpioDataRegs.GPATOGGLE.all = ledMask;
            break;
        default:
            break;
    }
}
