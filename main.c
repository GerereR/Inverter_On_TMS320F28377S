// Prefer direct register access throughout the application layer.
//testing git
//基石版本,即现在引脚配置是虚假的,下一步就是配置zhen

#include "F28x_Project.h"

#include "system.h"
#include "scheduler.h"
#include "task.h"
#include "bsp.h"

int main(void)
{
    Uint16 schedulerFlags;

    // Peripheral interrupts are configured first and enabled together here.
    SRF_PLL_Init(&GridSPLL, 50.0f, 20000.0f);//PLL state is consumed by ADCA1 ISR

    System_Init();

    Task_Comm_Init();
    // Polled OLED bring-up runs before the power stage starts; failure must not block startup.
    Task_UI_Init();
    // Start PWM only after the CPU ADC interrupt path is configured.
    EPWM_Start();

    Scheduler_Config();

    EINT;
    ERTM;

    // Cooperative tasks run in the main context; ISRs only acquire data and
    // set scheduler flags so time-critical interrupt latency stays bounded.
    while(1)
    {
        schedulerFlags = Scheduler_GetFlags();

        if(schedulerFlags & TASK_MEASURE_FLAG)     // 3 ms
        {
            Scheduler_ClearFlags(TASK_MEASURE_FLAG);
            Task_Measure();
        }

        if(schedulerFlags & TASK_STATE_FLAG)       // 5 ms
        {
            Scheduler_ClearFlags(TASK_STATE_FLAG);
            Task_State();
        }

        if(schedulerFlags & TASK_GRID_FLAG)        // 20 ms
        {
            Scheduler_ClearFlags(TASK_GRID_FLAG);
            Task_Grid();
        }

        if(schedulerFlags & TASK_POWER_FLAG)       // 50 ms
        {
            Scheduler_ClearFlags(TASK_POWER_FLAG);
            Task_Power();
        }

        if(schedulerFlags & TASK_MPPT_FLAG)        // 100 ms
        {
            Scheduler_ClearFlags(TASK_MPPT_FLAG);
            Task_MPPT();
        }

        if(schedulerFlags & TASK_COMM_FLAG)        // 500 ms
        {
            Scheduler_ClearFlags(TASK_COMM_FLAG);
            Task_Comm();
        }

        if(schedulerFlags & TASK_UI_FLAG)          // 200 ms, refresh is page-limited
        {
            Scheduler_ClearFlags(TASK_UI_FLAG);
            Task_UI();
        }
    }
}
