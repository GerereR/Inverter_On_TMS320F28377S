#include "F28x_Project.h"

#include "system.h"
#include "bsp.h"
#include "../z5_task/task.h"

float System_Clamp(float value, float minimum, float maximum)
{
float System_Clamp(float value, float minimum, float maximum)
{
    return (value > maximum) ? maximum :
           (value < minimum) ? minimum :
                               value;
}
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
    AC_Ctrl_Disable();
    DSP_STATE_LOW();
}

/*============================================================================
 * 协作式调度器（原 scheduler.c 并入）
 * 1ms tick 派生各定时任务标志；过零/峰值由 ISR 通知。
 *==========================================================================*/
static volatile Uint16 Scheduler_Flags = 0U;

void Scheduler_Config(void)
{
    Scheduler_Flags = 0U;
    SchedulerTimer_Config();
}

Uint16 Scheduler_TakeFlags(void)
{
    Uint16 flags;

    /* Claim every pending event as one snapshot. An ISR that runs after EINT
     * posts into a fresh flag word for the next main-loop pass. */
    DINT;
    flags = Scheduler_Flags;
    Scheduler_Flags = 0U;
    EINT;

    return flags;
}

void Scheduler_NotifyGridZeroCross(void)
{
    /* Grid calculations are phase-aligned; measurement also gets an immediate
     * opportunity to consume the DMA block closed at this boundary. */
    Scheduler_Flags |= (TASK_AC_MONITOR_FLAG | TASK_MEASURE_FLAG);
}

void Scheduler_NotifyGridPeak(void)
{
    /* The positive and negative peaks produce two DC-control events per cycle. */
    Scheduler_Flags |= TASK_DC_CTRL_FLAG;
}

void Scheduler_Tick1ms(void)
{
    static Uint16 cntMeasure = 0U;
    static Uint16 cntState = 0U;
    static Uint16 cntMppt = 0U;
    static Uint16 cntUi = 0U;
    static Uint16 cntComm = 0U;
    static Uint16 cntEeprom = 0U;
    static Uint16 cntReactive = 0U;
    static Uint16 cntPower = 0U;

    // Derive all cooperative task rates from the common 1 ms tick.
    if(++cntMeasure >= TASK_MEASURE_PERIOD_MS)
    {
        cntMeasure = 0U;
        Scheduler_Flags |= TASK_MEASURE_FLAG;
    }

    if(++cntState >= TASK_STATE_PERIOD_MS)
    {
        cntState = 0U;
        Scheduler_Flags |= TASK_STATE_FLAG;
    }

    if(++cntMppt >= TASK_MPPT_PERIOD_MS)
    {
        cntMppt = 0U;
        Scheduler_Flags |= TASK_MPPT_FLAG;
    }

    if(++cntUi >= TASK_UI_PERIOD_MS)
    {
        cntUi = 0U;
        Scheduler_Flags |= TASK_UI_FLAG;
    }

    if(++cntComm >= TASK_COMM_PERIOD_MS)
    {
        cntComm = 0U;
        Scheduler_Flags |= TASK_COMM_FLAG;
    }

    if(++cntEeprom >= TASK_EEPROM_PERIOD_MS)
    {
        cntEeprom = 0U;
        Scheduler_Flags |= TASK_EEPROM_FLAG;
    }

    if(++cntReactive >= TASK_REACTIVE_PERIOD_MS)
    {
        cntReactive = 0U;
        Scheduler_Flags |= TASK_REACTIVE_FLAG;
    }

    if(++cntPower >= TASK_POWER_PERIOD_MS)
    {
        cntPower = 0U;
        Scheduler_Flags |= TASK_POWER_FLAG;
    }
}
