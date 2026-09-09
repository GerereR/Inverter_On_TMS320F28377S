#include "F28x_Project.h"

#include "scheduler.h"
#include "bsp.h"

#define SCHEDULER_PENDING_MAX  0xFFFFU

/* Periodic work may coalesce; grid-synchronous events must retain occurrence count. */
static volatile Uint16 Scheduler_PeriodicFlags = 0U;
static volatile Uint16 Scheduler_GridZeroCrossPending = 0U;
static volatile Uint16 Scheduler_GridPeakPending = 0U;
static volatile SchedulerProbe Scheduler_ProbeData = {0};

const volatile SchedulerProbe * const gSchedulerProbe = &Scheduler_ProbeData;

void Scheduler_Init(void)
{
    Uint16 taskId;

    Scheduler_PeriodicFlags = 0U;
    Scheduler_GridZeroCrossPending = 0U;
    Scheduler_GridPeakPending = 0U;

    for(taskId = 0U; taskId < SCHED_TASK_COUNT; taskId++)
    {
        Scheduler_ProbeData.taskLastCycles[taskId] = 0UL;
        Scheduler_ProbeData.taskMaxCycles[taskId] = 0UL;
    }
    Scheduler_ProbeData.batchLastCycles = 0UL;
    Scheduler_ProbeData.batchMaxCycles = 0UL;

    Timer0_Init();
    
    #if (SCHEDULER_PROFILE_ENABLE != 0U)
    Timer2_Init();
    #endif  
}

#if (SCHEDULER_PROFILE_ENABLE != 0U)

    Uint32 Scheduler_ProfileBegin(void)
    {
        return Timer2_GetCount();
    }

    void Scheduler_ProfileTaskEnd(SchedulerTaskId taskId, Uint32 startCycles)
    {
        Uint32 elapsedCycles;

        if(taskId >= SCHED_TASK_COUNT)
        {
            return;
        }

        /* Timer2 counts down; unsigned subtraction also handles one wraparound. */
        elapsedCycles = startCycles - Timer2_GetCount();
        Scheduler_ProbeData.taskLastCycles[taskId] = elapsedCycles;

        if(elapsedCycles > Scheduler_ProbeData.taskMaxCycles[taskId])
        {
            Scheduler_ProbeData.taskMaxCycles[taskId] = elapsedCycles;
        }
    }

    void Scheduler_ProfileBatchEnd(Uint32 startCycles)
    {
        Uint32 elapsedCycles;

        elapsedCycles = startCycles - Timer2_GetCount();
        Scheduler_ProbeData.batchLastCycles = elapsedCycles;

        if(elapsedCycles > Scheduler_ProbeData.batchMaxCycles)
        {
            Scheduler_ProbeData.batchMaxCycles = elapsedCycles;
        }
    }
    
#endif

Uint16 Scheduler_TakeFlags(void)
{
    Uint16 flags;
    Uint16 interruptState;

    interruptState = CPU_InterruptSaveDisable();
    flags = Scheduler_PeriodicFlags;
    Scheduler_PeriodicFlags = 0U;

    if(Scheduler_GridZeroCrossPending != 0U)
    {
        Scheduler_GridZeroCrossPending--;
        flags |= TASK_AC_MONITOR_FLAG;
    }
    if(Scheduler_GridPeakPending != 0U)
    {
        Scheduler_GridPeakPending--;
        flags |= TASK_DC_CTRL_FLAG;
    }

    CPU_InterruptRestore(interruptState);

    return flags;
}

void Scheduler_NotifyGridZeroCross(void)
{
    if(Scheduler_GridZeroCrossPending < SCHEDULER_PENDING_MAX)
    {
        Scheduler_GridZeroCrossPending++;
    }

    /* Give the completed DMA grid block an immediate processing opportunity. */
    Scheduler_PeriodicFlags |= TASK_MEASURE_FLAG;
}

void Scheduler_NotifyGridPeak(void)
{
    if(Scheduler_GridPeakPending < SCHEDULER_PENDING_MAX)
    {
        Scheduler_GridPeakPending++;
    }
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

    if(++cntMeasure >= TASK_MEASURE_PERIOD_MS)
    {
        cntMeasure = 0U;
        Scheduler_PeriodicFlags |= TASK_MEASURE_FLAG;
    }

    if(++cntState >= TASK_STATE_PERIOD_MS)
    {
        cntState = 0U;
        Scheduler_PeriodicFlags |= TASK_STATE_FLAG;
    }

    if(++cntMppt >= TASK_MPPT_PERIOD_MS)
    {
        cntMppt = 0U;
        Scheduler_PeriodicFlags |= TASK_MPPT_FLAG;
    }

    if(++cntUi >= TASK_UI_PERIOD_MS)
    {
        cntUi = 0U;
        Scheduler_PeriodicFlags |= TASK_UI_FLAG;
    }

    if(++cntComm >= TASK_COMM_PERIOD_MS)
    {
        cntComm = 0U;
        Scheduler_PeriodicFlags |= TASK_COMM_FLAG;
    }

    if(++cntEeprom >= TASK_EEPROM_PERIOD_MS)
    {
        cntEeprom = 0U;
        Scheduler_PeriodicFlags |= TASK_EEPROM_FLAG;
    }

    if(++cntReactive >= TASK_REACTIVE_PERIOD_MS)
    {
        cntReactive = 0U;
        Scheduler_PeriodicFlags |= TASK_REACTIVE_FLAG;
    }

    if(++cntPower >= TASK_POWER_PERIOD_MS)
    {
        cntPower = 0U;
        Scheduler_PeriodicFlags |= TASK_POWER_FLAG;
    }
}
