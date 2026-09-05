#ifndef SYSTEM_H_
#define SYSTEM_H_

/* System startup and peripheral initialization entry point. */
void System_Init(void);
void System_EnterSafeOutput(void);

/* Shared floating-point limiter for control and supervisory algorithms. */
float System_Clamp(float value, float minimum, float maximum);

/* ---- 协作式调度器（1ms tick 派生 + 过零/峰值事件） ---- */
#define TASK_STATE_FLAG       0x0001
#define TASK_MEASURE_FLAG     0x0002
#define TASK_AC_MONITOR_FLAG  0x0004
#define TASK_MPPT_FLAG        0x0008
#define TASK_COMM_FLAG        0x0020
#define TASK_UI_FLAG          0x0040
#define TASK_EEPROM_FLAG      0x0080
#define TASK_DC_CTRL_FLAG     0x0100
#define TASK_REACTIVE_FLAG    0x0200
#define TASK_POWER_FLAG       0x0400

/* Cooperative task periods derived from the common 1 ms scheduler tick. */
#define TASK_MEASURE_PERIOD_MS     3U
#define TASK_STATE_PERIOD_MS       5U
#define TASK_MPPT_PERIOD_MS      500U
#define TASK_UI_PERIOD_MS       1500U
#define TASK_COMM_PERIOD_MS      500U
#define TASK_EEPROM_PERIOD_MS   1000U
#define TASK_REACTIVE_PERIOD_MS 10U
#define TASK_POWER_PERIOD_MS    10U

void Scheduler_Config(void);
Uint16 Scheduler_TakeFlags(void);
void Scheduler_Tick1ms(void);
void Scheduler_NotifyGridZeroCross(void);
void Scheduler_NotifyGridPeak(void);

#endif /* SYSTEM_H_ */
