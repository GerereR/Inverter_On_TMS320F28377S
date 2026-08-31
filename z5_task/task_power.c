#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"

void Task_Power(void)
{
    // Temporary heartbeat until power management is implemented.
    LED2_TOGGLE();
}

void Task_DcCtrl(void)
{
    /* Phase-aligned 10 ms hook for the future DC-bus outer loop. The actual
     * controller remains intentionally absent until its startup policy and
     * limits are defined. */
}
