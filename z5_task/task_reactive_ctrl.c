#include "F28x_Project.h"

#include "task.h"
#include "../z8_control/reactive_ctrl.h"

void Task_ReactiveCtrl_Init(void)
{
    ReactiveCtrl_Init();
}

void Task_ReactiveCtrl(void)
{
    /* Keep task scheduling separate from the future reactive-control logic. */
    ReactiveCtrl_Update();
}
