#include "F28x_Project.h"

#include "scheduler.h"
#include "bsp.h"

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
    Scheduler_Flags |= (TASK_GRID_FLAG | TASK_MEASURE_FLAG);
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
    static Uint16 cntReactiveCtrl = 0U;
    static Uint16 cntPowerLimit = 0U;

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

    if(++cntReactiveCtrl >= TASK_REACTIVE_CTRL_PERIOD_MS)
    {
        cntReactiveCtrl = 0U;
        Scheduler_Flags |= TASK_REACTIVE_CTRL_FLAG;
    }

    if(++cntPowerLimit >= TASK_POWER_LIMIT_PERIOD_MS)
    {
        cntPowerLimit = 0U;
        Scheduler_Flags |= TASK_POWER_LIMIT_FLAG;
    }

}
