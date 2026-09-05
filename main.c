#include "F28x_Project.h"

#include "system.h"
#include "scheduler.h"
#include "task.h"
#include "bsp.h"
#include "z8_control/control.h"
#include "z8_control/pll.h"

int main(void)
{
    Uint16 schedulerFlags;

    // Peripheral interrupts are configured first and enabled together here.
    //SRF_PLL_Init(&GridSPLL, 50.0f, 20000.0f);//PLL state is consumed by ADCA1 ISR
    SOGI_PLL_Init(&GridSPLL, 50.0f, 20000.0f);//PLL state is consumed by ADCA1 ISR
    Ctrl_Init();

    System_Init();

    /* Initialize all task-owned state after the peripherals are ready. */
    Task_Init();
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

        if(schedulerFlags & TASK_GRID_FLAG)        // valid eCAP grid boundary
        {
            Task_Grid();
        }

        if(schedulerFlags & TASK_POWER_LIMIT_FLAG) // 10 ms framework task
        {
            Task_PowerLimit();
        }

        if(schedulerFlags & TASK_REACTIVE_CTRL_FLAG) // 10 ms framework task
        {
            Task_ReactiveCtrl();
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
