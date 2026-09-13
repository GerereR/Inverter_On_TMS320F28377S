#include "F28x_Project.h"

#include "inverter.h"
#include "bsp.h"
#include "task.h"

static void Inverter_ForceHardwareSafe(void);

//限幅器
float Inverter_Clamp(float value, float minimum, float maximum)
{
    return ((value != value) || (minimum != minimum) || (maximum != maximum) || (minimum > maximum)) ? 0.0f :
    (value > maximum) ? maximum : (value < minimum) ? minimum : value;
}

//逆变器初始化
void Inverter_Init(void)
{
    //SYSTEM配置
    InitSysCtrl();
    InitGpio();
    InitPieCtrl();
    CPU_InterruptInit();
    InitPieVectTable();

    //BSP配置
    EPWM_Config();
    GPIO_Config();
    Inverter_ForceHardwareSafe();
    ADC_Config();
    DMA_Config();
    ECAP_Config();
    SCI_Config();
    I2C_Config();
}

//软件强制保护
void Inverter_EnterSafeOutput(void)
{
    Inverter_ForceHardwareSafe();
    AC_Ctrl_Disable();
}

//硬件强制保护
static void Inverter_ForceHardwareSafe(void)
{
    EPWM_Disable();
    BOOST_OFF();
    INVERTER_OFF();
    
    ISO_RELAY1_OFF();
    ISO_RELAY2_OFF();
    GRID_RELAY1_OFF();
    GRID_RELAY2_OFF();
    GRID_RELAY3_OFF();
    GRID_RELAY4_OFF();
    GRID_RELAY_ALL_OFF();
    
    GFCI_CHECK_OFF();
    DSP_STATE_LOW();
}
