#include "F28x_Project.h"

#include "inverter.h"
#include "bsp.h"
#include "task.h"

static void Inverter_ForceHardwareSafe(void);

float Inverter_Clamp(float value, float minimum, float maximum)
{
    /* Every current caller uses zero as its physically inactive command. */
    return ((value != value) || (minimum != minimum) || (maximum != maximum) || (minimum > maximum)) ? 0.0f :
    (value > maximum) ? maximum : (value < minimum) ? minimum : value;
}

void Inverter_Init(void)
{
    /* Establish clocks and a known interrupt state before configuring BSPs. */
    InitSysCtrl();
    InitGpio();
    InitPieCtrl();
    CPU_InterruptInit();
    InitPieVectTable();

    /* Configure peripherals while the power-stage time bases are stopped. */
    EPWM_Config();
    GPIO_Config();
    Inverter_ForceHardwareSafe();
    ADC_Config();
    DMA_Config();
    ECAP_Config();
    SCI_Config();
    I2C_Config();

    /* EPWM_Start() is called explicitly by main() after ISR setup. */
}

void Inverter_EnterSafeOutput(void)
{
    /* This is the coordinated software shutdown path for task context. A TZ
     * event still clamps PWM in hardware before the state task reaches here. */
    Inverter_ForceHardwareSafe();
    AC_Ctrl_Disable();
}

static void Inverter_ForceHardwareSafe(void)
{
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
    DSP_STATE_LOW();
}
