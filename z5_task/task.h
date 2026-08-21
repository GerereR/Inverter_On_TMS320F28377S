#ifndef TASK_H
#define TASK_H

void Task_Init(void);

// Cooperative task periods are assigned by scheduler.c.
void Task_State(void);
void Task_Measure(void);
void Task_Grid(void);
void Task_MPPT(void);
void Task_Power(void);
void Task_Comm_Init(void);
void Task_Comm(void);

#endif
