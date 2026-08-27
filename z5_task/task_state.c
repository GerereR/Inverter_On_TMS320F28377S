#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"

void Task_State(void)
{
    Uint16 keyEvents;

    /* GPIO details and key debounce are handled inside the BSP. */
    keyEvents = GPIO_GetKeyEvents();

    /* Key actions will be connected to the inverter state machine later. */
    if(keyEvents != 0U)
    {
        /* Reserved for KEY1..KEY4 state transitions. */
    }

    /* Reserved for inverter state transitions and fault-state handling. */
}
