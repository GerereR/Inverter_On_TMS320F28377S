#ifndef SCHEDULER_H_
#define SCHEDULER_H_


#define TASK_STATE_FLAG       0x0001
#define TASK_MEASURE_FLAG     0x0002
#define TASK_GRID_FLAG        0x0004
#define TASK_MPPT_FLAG        0x0008
#define TASK_COMM_FLAG        0x0020
#define TASK_UI_FLAG          0x0040
#define TASK_EEPROM_FLAG      0x0080
#define TASK_DC_CTRL_FLAG     0x0100

/* Cooperative task periods derived from the common 1 ms scheduler tick. */
#define TASK_MEASURE_PERIOD_MS     3U
#define TASK_STATE_PERIOD_MS       5U
#define TASK_MPPT_PERIOD_MS      500U
#define TASK_UI_PERIOD_MS       1500U
#define TASK_COMM_PERIOD_MS      500U
#define TASK_EEPROM_PERIOD_MS   1000U

void Scheduler_Config(void);
Uint16 Scheduler_TakeFlags(void);
void Scheduler_Tick1ms(void);
void Scheduler_NotifyGridZeroCross(void);
void Scheduler_NotifyGridPeak(void);


#endif
