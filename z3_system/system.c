#include "F28x_Project.h"

#include "system.h"
#include "bsp.h"
#include "../z8_control/control.h"

float System_Clamp(float value, float minimum, float maximum)
{
    if(value > maximum)
    {
        return maximum;
    }
    if(value < minimum)
    {
        return minimum;
    }
    return value;
}

void System_Init(void)
{
    /* Establish clocks and a known interrupt state before configuring BSPs. */
    InitSysCtrl();
    InitGpio();
    InitPieCtrl();
    DINT;
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    /* Configure peripherals while the power-stage time bases are stopped. */
    EPWM_Config();
    GPIO_Config();
    ADC_Config();
    DMA_Config();
    ECAP_Config();
    SCI_Config();
    I2C_Config();

    /* EPWM_Start() is called explicitly by main() after ISR setup. */
}

void System_EnterSafeOutput(void)
{
    /* This is the coordinated software shutdown path for task context. A TZ
     * event still clamps PWM in hardware before the state task reaches here. */
    EPWM_Disable();
    BOOST_OFF();
    INVERTER_OFF();
    GRID_RELAY1_OFF();
    GRID_RELAY2_OFF();
    GRID_RELAY3_OFF();
    GRID_RELAY4_OFF();
    GRID_RELAY_ALL_OFF();
    ISO_RELAY1_OFF();
    ISO_RELAY2_OFF();
    GFCI_CHECK_OFF();
    Ctrl_Disable();
    DSP_STATE_LOW();
}
