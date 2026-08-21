#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"

void Task_MPPT(void)
{
    // Temporary heartbeat until the MPPT algorithm is implemented.
    LED_Ctrl(LED_NUMBER_3, LED_STATE_TOGGLE);
}
