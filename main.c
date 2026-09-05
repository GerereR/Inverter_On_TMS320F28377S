#include "F28x_Project.h"

#include "system.h"
#include "task.h"
#include "bsp.h"

int main(void)
{
    Uint16 schedulerFlags;

    // Peripheral interrupts are configured first and enabled together here.
    Fast_Init();  // PLL 初始化已收进 Fast_Init，main 不再直接接触 PLL。

    System_Init();

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

    Scheduler_Config();

    EINT;
    ERTM;

    // Cooperative tasks run in the main context; ISRs only acquire data and
    // set scheduler flags so time-critical interrupt latency stays bounded.
    while(1)
    {
        schedulerFlags = Scheduler_TakeFlags();

        if(schedulerFlags & TASK_MEASURE_FLAG)     // 3 ms
        {
            Task_Measure();
        }

        if(schedulerFlags & TASK_STATE_FLAG)       // 5 ms
        {
            Task_State();
        }

        if(schedulerFlags & TASK_AC_MONITOR_FLAG)  // valid eCAP grid boundary
        {
            Task_AcMonitor();
        }

        if(schedulerFlags & TASK_POWER_FLAG) // 10 ms framework task
        {
            Task_Power();
        }

        if(schedulerFlags & TASK_REACTIVE_FLAG) // 10 ms framework task
        {
            Task_Reactive();
        }

        if(schedulerFlags & TASK_DC_CTRL_FLAG)     // positive/negative grid peak
        {
            Task_DcCtrl();
        }

        if(schedulerFlags & TASK_MPPT_FLAG)        // 500 ms
        {
            Task_MPPT();
        }

        if(schedulerFlags & TASK_COMM_FLAG)        // 500 ms
        {
            Task_Comm();
        }
        if(schedulerFlags & TASK_EEPROM_FLAG)      // 1 s check; writes are request-driven
        {
            Task_Eeprom();
        }
        
        if(schedulerFlags & TASK_UI_FLAG)          // 1.5 s, refresh one complete screen
        {
            Task_UI();
        }
    }
}
