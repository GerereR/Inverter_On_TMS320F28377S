#ifndef SCHEDULER_H_
#define SCHEDULER_H_


#define TASK_STATE_FLAG       0x0001
#define TASK_MEASURE_FLAG     0x0002
#define TASK_GRID_FLAG        0x0004
#define TASK_MPPT_FLAG        0x0008
#define TASK_POWER_FLAG       0x0010
#define TASK_COMM_FLAG        0x0020
#define TASK_UI_FLAG          0x0040

/* Cooperative task periods derived from the common 1 ms scheduler tick. */
#define TASK_MEASURE_PERIOD_MS     3U
#define TASK_STATE_PERIOD_MS       5U
#define TASK_GRID_PERIOD_MS       20U
#define TASK_POWER_PERIOD_MS      50U
#define TASK_MPPT_PERIOD_MS      100U
#define TASK_UI_PERIOD_MS        200U
#define TASK_COMM_PERIOD_MS      500U

void Scheduler_Config(void);
Uint16 Scheduler_GetFlags(void);
void Scheduler_ClearFlags(Uint16 flags);
__interrupt void Scheduler_ISR(void);


#endif
