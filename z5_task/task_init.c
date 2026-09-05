#include "F28x_Project.h"

#include "task.h"

/* Initialize task-owned runtime state after System_Init() has configured the
 * peripherals and before the scheduler enables cooperative execution. */
void Task_Init(void)
{
    Task_Eeprom_Init();
    Task_Comm_Init();
    Task_State_Init();
    Task_MPPT_Init();
    Task_ReactiveCtrl_Init();
    Task_PowerLimit_Init();
    Task_UI_Init();
}
