#include "F28x_Project.h"

#include "task.h"
#include "../z8_control/power_limit.h"

void Task_PowerLimit_Init(void)
{
    PowerLimit_Init();
}

void Task_PowerLimit(void)
{
    /* Keep task scheduling separate from the future limit calculations. */
    PowerLimit_Update();
}
