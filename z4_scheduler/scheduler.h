#ifndef SCHEDULER_H_
#define SCHEDULER_H_


#define TASK_STATE_FLAG       0x0001
#define TASK_MEASURE_FLAG     0x0002
#define TASK_GRID_FLAG        0x0004
#define TASK_MPPT_FLAG        0x0008
#define TASK_POWER_FLAG       0x0010
#define TASK_COMM_FLAG        0x0020

void Scheduler_Config(void);
Uint16 Scheduler_GetFlags(void);
void Scheduler_ClearFlags(Uint16 flags);
__interrupt void Scheduler_ISR(void);


#endif
