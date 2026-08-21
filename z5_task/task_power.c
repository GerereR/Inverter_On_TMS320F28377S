#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"

void Task_Power(void)
{
    // Temporary heartbeat until power management is implemented.
    LED_Ctrl(LED_NUMBER_2, LED_STATE_TOGGLE);
}
