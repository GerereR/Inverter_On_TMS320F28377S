#include "F28x_Project.h"

#include "task.h"

void Task_Power_Init(void)
{
    /* Framework only: keep the existing default limit until the manager is
     * implemented. Thermal, PV, frequency and user limits are not active yet. */
}

void Task_Power(void)
{
    /* Framework only: update power/current limits from measurements and
     * operating rules once the limit manager is implemented. */
}
