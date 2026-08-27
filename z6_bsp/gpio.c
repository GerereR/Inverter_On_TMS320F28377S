#include "F28x_Project.h"
#include "bsp.h"

#define GPIO_KEY_COUNT             4U
#define GPIO_KEY_DEBOUNCE_SAMPLES  4U  /* Task_State runs every 5 ms: about 20 ms. */

static Uint16 GPIO_ReadKeyPressed(Uint16 keyNumber);
static void GPIO_KeyDebounceInit(void);

static Uint16 GPIO_KeyStablePressed[GPIO_KEY_COUNT] = {0U, 0U, 0U, 0U};
static Uint16 GPIO_KeyLastPressed[GPIO_KEY_COUNT] = {0U, 0U, 0U, 0U};
static Uint16 GPIO_KeyDebounceCount[GPIO_KEY_COUNT] = {0U, 0U, 0U, 0U};
static Uint16 GPIO_KeyPendingEvents = 0U;


/* Board pin map from 引脚配置.xlsx.  ADC pin/SOC settings remain in adc.c. */
void GPIO_Config(void)
{
    EALLOW;

    /* Gate-driver PWM outputs: GPIO0/1=EPWM1, GPIO2/3=EPWM2,
     * GPIO4/5=EPWM3 (dual Boost). GPIO6/7=EPWM4 remain reserved for the
     * old EPWM6-style ZVT function. */
    GpioCtrlRegs.GPAPUD.all |= 0x000000FFUL;
    GpioCtrlRegs.GPAGMUX1.all &= ~0x0000FFFFUL;
    GpioCtrlRegs.GPAMUX1.all &= ~0x0000FFFFUL;
    /* GPIO0..7 use mux function 1 for EPWMxA/EPWMxB on F28377S. */
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1U;
    GpioCtrlRegs.GPADIR.all |= 0x000000FFUL;
    GpioDataRegs.GPASET.all = 0x000000FFUL;

    /* Passive buzzer: GPIO8 is EPWM5A (the tone is generated in epwm.c). */
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1U;

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

    /* Grid_Relay4_L and Grid_Relay_OFF_L use GPIO10 and GPIO11. */
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1U;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0U;
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1U;
    GpioDataRegs.GPASET.bit.GPIO10 = 1U;
    GpioDataRegs.GPASET.bit.GPIO11 = 1U;

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

    /* Clear any startup trip flags after routing the three TZ inputs. */
    EPwm1Regs.TZCLR.bit.OST = 1U;
    EPwm1Regs.TZCLR.bit.INT = 1U;
    EPwm3Regs.TZCLR.bit.OST = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm3Regs.TZCLR.bit.INT = 1U;
    EPwm4Regs.TZCLR.bit.OST = 1U;
    EPwm4Regs.TZCLR.bit.INT = 1U;

    GPIO_KeyDebounceInit();

    EDIS;
}

/* Read one active-low key. The GPIO register access is kept inside the BSP. */
static Uint16 GPIO_ReadKeyPressed(Uint16 keyNumber)
{
    Uint16 isPressed = 0U;

    switch(keyNumber)
    {
        case KEY_NUMBER_1:
            isPressed = (GpioDataRegs.GPCDAT.bit.GPIO85 == 0U) ? 1U : 0U;
            break;
        case KEY_NUMBER_2:
            isPressed = (GpioDataRegs.GPCDAT.bit.GPIO83 == 0U) ? 1U : 0U;
            break;
        case KEY_NUMBER_3:
            isPressed = (GpioDataRegs.GPCDAT.bit.GPIO81 == 0U) ? 1U : 0U;
            break;
        case KEY_NUMBER_4:
            isPressed = (GpioDataRegs.GPCDAT.bit.GPIO79 == 0U) ? 1U : 0U;
            break;
        default:
            isPressed = 0U;
            break;
    }

    return isPressed;
}

static void GPIO_KeyDebounceInit(void)
{
    Uint16 keyIndex;
    Uint16 pressed;

    for(keyIndex = 0U; keyIndex < GPIO_KEY_COUNT; keyIndex++)
    {
        pressed = GPIO_ReadKeyPressed((Uint16)(keyIndex + 1U));
        GPIO_KeyStablePressed[keyIndex] = pressed;
        GPIO_KeyLastPressed[keyIndex] = pressed;
        GPIO_KeyDebounceCount[keyIndex] = 0U;
    }
    GPIO_KeyPendingEvents = 0U;
}

/* Scan all keys and return only newly confirmed press events. */
Uint16 GPIO_GetKeyEvents(void)
{
    Uint16 keyIndex;
    Uint16 pressed;
    Uint16 eventMask = 0U;
    const Uint16 keyEventMasks[GPIO_KEY_COUNT] =
        {KEY_EVENT_1, KEY_EVENT_2, KEY_EVENT_3, KEY_EVENT_4};

    for(keyIndex = 0U; keyIndex < GPIO_KEY_COUNT; keyIndex++)
    {
        pressed = GPIO_ReadKeyPressed((Uint16)(keyIndex + 1U));

        if(pressed == GPIO_KeyStablePressed[keyIndex])
        {
            GPIO_KeyDebounceCount[keyIndex] = 0U;
            GPIO_KeyLastPressed[keyIndex] = pressed;
        }
        else if(pressed == GPIO_KeyLastPressed[keyIndex])
        {
            if(GPIO_KeyDebounceCount[keyIndex] < GPIO_KEY_DEBOUNCE_SAMPLES)
            {
                GPIO_KeyDebounceCount[keyIndex]++;
            }

            if(GPIO_KeyDebounceCount[keyIndex] >= GPIO_KEY_DEBOUNCE_SAMPLES)
            {
                GPIO_KeyStablePressed[keyIndex] = pressed;
                GPIO_KeyDebounceCount[keyIndex] = 0U;

                if(pressed != 0U)
                {
                    eventMask |= keyEventMasks[keyIndex];
                }
            }
        }
        else
        {
            GPIO_KeyLastPressed[keyIndex] = pressed;
            GPIO_KeyDebounceCount[keyIndex] = 1U;
        }
    }

    GPIO_KeyPendingEvents |= eventMask;
    eventMask = GPIO_KeyPendingEvents;
    GPIO_KeyPendingEvents = 0U;
    return eventMask;
}
