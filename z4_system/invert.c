#include "F28x_Project.h"

#include "invert.h"
#include "bsp.h"
#include "task.h"

static void Invert_ForceHwSafe(void);

//限幅器
float Invert_Clamp(float value, float minimum, float maximum)
{
    return ((value != value) || (minimum != minimum) || (maximum != maximum) || (minimum > maximum)) ? 0.0f :
    (value > maximum) ? maximum : (value < minimum) ? minimum : value;
}

//逆变器初始化
void Invert_Init(void)
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
    Invert_ForceHwSafe();
    ADC_Config();
    Timer1_Config();
    DMA_Config();
    ECAP_Config();
    SCI_Config();
    I2C_Config();
}

//软件强制保护
void Invert_EnterSafeOutput(void)
{
    Invert_ForceHwSafe();
    AC_Ctrl_Disable();
}

//硬件强制保护
static void Invert_ForceHwSafe(void)
{
    EPWM_Disable();
    BOOST_OFF();
    INVERT_OFF();
    
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
