#include "F28x_Project.h"
#include "bsp.h"

/* Board pin map from 引脚配置.xlsx.  ADC pin/SOC settings remain in adc.c. */

void GPIO_Config(void)
{
    EALLOW;

    /* Gate-driver PWM outputs: GPIO0/1=EPWM1, GPIO2/3=EPWM2,
     * GPIO4/5=EPWM3, GPIO6/7=EPWM4.  Only EPWM3 is configured today. */
    GpioCtrlRegs.GPAPUD.all |= 0x000000FFUL;
    GpioCtrlRegs.GPAGMUX1.all &= (Uint16)~0x00FFU;
    GpioCtrlRegs.GPAMUX1.all &= (Uint16)~0x00FFU;
    /* Only the active EPWM3 pair is muxed now; future PWM pairs stay GPIO. */
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1U;
    GpioCtrlRegs.GPADIR.all |= 0x000000FFUL;
    GpioDataRegs.GPASET.all = 0x000000FFUL;

    /* Board digital inputs: POWER_SW1..3, KEY1..4, OVP_BUS, FAN_STATE.
     * The switches and keys pull to ground when pressed, so their internal
     * pull-ups are enabled (PUD=0). */
    GpioCtrlRegs.GPCPUD.bit.GPIO76 = 0U;
    GpioCtrlRegs.GPCPUD.bit.GPIO77 = 0U;
    GpioCtrlRegs.GPCPUD.bit.GPIO78 = 0U;
    GpioCtrlRegs.GPCPUD.bit.GPIO79 = 0U;
    GpioCtrlRegs.GPCPUD.bit.GPIO80 = 0U;
    GpioCtrlRegs.GPCPUD.bit.GPIO81 = 0U;
    GpioCtrlRegs.GPCPUD.bit.GPIO82 = 0U;
    GpioCtrlRegs.GPCPUD.bit.GPIO83 = 0U;
    GpioCtrlRegs.GPCPUD.bit.GPIO85 = 0U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO76 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 0U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO77 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 0U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO78 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 0U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO79 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO80 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO81 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO82 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO83 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO76 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO77 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO78 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO79 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO80 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO81 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO82 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO83 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO85 = 0U;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO76 = 3U;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO77 = 3U;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO78 = 3U;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO79 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO80 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO81 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO82 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO83 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO85 = 3U;

    /* Board digital outputs.  Signals ending in _L are released high. */
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 1U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO53 = 0U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1U;
    GpioDataRegs.GPBSET.bit.GPIO53 = 1U; /* FAN_PWM_L */

    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0U;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1U;
    GpioDataRegs.GPASET.bit.GPIO31 = 1U; /* LED1 */

    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1U;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0U;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1U;
    GpioDataRegs.GPBSET.bit.GPIO34 = 1U; /* LED2 */

    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 1U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO56 = 0U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO56 = 1U;
    GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1U; /* BEEP off */

    /* GPIO67 is the M24C64 WP pin. Low keeps write-protect deasserted so the
     * EEPROM task can save parameters; software can drive it high later. */
    GpioCtrlRegs.GPCPUD.bit.GPIO67 = 1U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO67 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1U;
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1U; /* EEPROM WP low: writes enabled */

    GpioCtrlRegs.GPCPUD.bit.GPIO88 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO89 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO90 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO91 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO92 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO93 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO94 = 1U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO88 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO88 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO89 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO89 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO90 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO90 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO91 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO91 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO92 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO92 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO93 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO93 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO94 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO94 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO88 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO89 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO90 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO91 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO92 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO93 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO94 = 1U;
    GpioDataRegs.GPCSET.all = (1UL << 24U) | (1UL << 25U) | (1UL << 26U) |
                               (1UL << 27U) | (1UL << 28U) | (1UL << 29U) |
                               (1UL << 30U);

    /* Remaining active-low status/control outputs on GPIO18..21. */
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1U;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1U;
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1U;
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0U;
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1U;
    GpioDataRegs.GPASET.all = (1UL << 18U) | (1UL << 19U) |
                               (1UL << 20U) | (1UL << 21U);

    /* SCIB: GPIO54=SCITXDB, GPIO55=SCIRXDB (function 6: GMUX=1, MUX=2). */
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0U;
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0U;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 3U;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 1U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 2U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 1U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 2U;
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1U;
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0U;

    /* I2CA: GPIO42=SDAA, GPIO43=SCLA (function 6: GMUX=1, MUX=2). */
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0U;
    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0U;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO42 = 3U;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO43 = 3U;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO42 = 1U;
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 2U;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO43 = 1U;
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 2U;

    /* ECAP input: GPIO61 -> INPUT X-BAR 7 -> ECAP1. */
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1U;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 0U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 0U;
    InputXbarRegs.INPUT7SELECT = 61U;

    /* Protection inputs: GPIO62/63/64 -> INPUT X-BAR 1/2/3 -> TZ1/2/3. */
    GpioCtrlRegs.GPBPUD.bit.GPIO62 = 1U;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 1U;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3U;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3U;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO62 = 0U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO62 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO63 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO64 = 0U;
    InputXbarRegs.INPUT1SELECT = 62U;
    InputXbarRegs.INPUT2SELECT = 63U;
    InputXbarRegs.INPUT3SELECT = 64U;

    /* Clear a stale PWM trip after routing the new TZ1 input. */
    EPwm3Regs.TZCLR.bit.OST = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm3Regs.TZCLR.bit.INT = 1U;

    EDIS;
}

/* The table defines two common-anode LEDs. Driving their GPIO low turns them on. */
void LED_Ctrl(Uint16 ledNumber, Uint16 state)
{
    if(ledNumber == LED_NUMBER_1)
    {
        if(state == LED_STATE_ON) GpioDataRegs.GPACLEAR.bit.GPIO31 = 1U;
        else if(state == LED_STATE_OFF) GpioDataRegs.GPASET.bit.GPIO31 = 1U;
        else if(state == LED_STATE_TOGGLE) GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1U;
    }
    else if(ledNumber == LED_NUMBER_2)
    {
        if(state == LED_STATE_ON) GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1U;
        else if(state == LED_STATE_OFF) GpioDataRegs.GPBSET.bit.GPIO34 = 1U;
        else if(state == LED_STATE_TOGGLE) GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1U;
    }
}
