#include "F28x_Project.h"

#include "inverter.h"
#include "scheduler.h"
#include "task.h"
#include "bsp.h"

int main(void)
{
    Uint16 schedulerFlags;
#if (SCHEDULER_PROFILE_ENABLE != 0U)
    Uint32 taskStartCycles;
    Uint32 batchStartCycles;
#endif

    // Peripheral interrupts are configured first and enabled together here.
    AC_Ctrl_Init();  // PLL 初始化已收进 AC_Ctrl_Init，main 不再直接接触 PLL。

    Inverter_Init();

    /* Initialize task-owned state after the peripherals are ready. Each task
     * owns its own init; there is no aggregate task_init translation unit. */
    Task_Eeprom_Init();
    Task_Comm_Init();
    Task_State_Init();
    Task_MPPT_Init();
    Task_Reactive_Init();
    Task_Power_Init();
    Task_UI_Init();
    /* Start the ePWM time bases for ADC triggering. Power outputs remain
     * software-clamped until the state machine completes CHECK. */
    EPWM_Start();

    Scheduler_Init();

    CPU_InterruptEnable();

    // Cooperative tasks run in the main context; ISRs only acquire data and
    // set scheduler flags so time-critical interrupt latency stays bounded.
    while(1)
    {
        schedulerFlags = Scheduler_TakeFlags();

        if(schedulerFlags == 0U)
        {
            continue;
        }

        SCHEDULER_PROFILE_BEGIN(batchStartCycles);

        if(schedulerFlags & TASK_MEASURE_FLAG)     // 3 ms
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Measure();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_MEASURE, taskStartCycles);
        }

        if(schedulerFlags & TASK_STATE_FLAG)       // 5 ms
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_State();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_STATE, taskStartCycles);
        }

        if(schedulerFlags & TASK_AC_MONITOR_FLAG)  // valid eCAP grid boundary
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_AC_Monitor();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_AC_MONITOR, taskStartCycles);
        }

        if(schedulerFlags & TASK_POWER_FLAG)        // 10 ms framework task
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Power();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_POWER, taskStartCycles);
        }

        if(schedulerFlags & TASK_REACTIVE_FLAG)     // 10 ms framework task
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Reactive();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_REACTIVE, taskStartCycles);
        }

        if(schedulerFlags & TASK_DC_CTRL_FLAG)     // positive/negative grid peak
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_DC_Ctrl();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_DC_CTRL, taskStartCycles);
        }

        if(schedulerFlags & TASK_MPPT_FLAG)        // 750 ms
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_MPPT();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_MPPT, taskStartCycles);
        }

        if(schedulerFlags & TASK_COMM_FLAG)        // 500 ms
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Comm();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_COMM, taskStartCycles);
        }
        if(schedulerFlags & TASK_EEPROM_FLAG)      // 1 s check; writes are request-driven
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Eeprom();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_EEPROM, taskStartCycles);
        }
        
        if(schedulerFlags & TASK_UI_FLAG)          // 2 s, refresh one complete screen
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_UI();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_UI, taskStartCycles);
        }

        SCHEDULER_PROFILE_BATCH_END(batchStartCycles);
    }
}
