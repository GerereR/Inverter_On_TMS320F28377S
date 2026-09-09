#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "F28x_Project.h"

#ifndef SCHEDULER_PROFILE_ENABLE
    #define SCHEDULER_PROFILE_ENABLE  1U
#endif

#define SCHEDULER_PROFILE_CYCLES_PER_US  200UL

/* Cooperative periodic tasks and grid-synchronous events. */
#define TASK_STATE_FLAG       0x0001U
#define TASK_MEASURE_FLAG     0x0002U
#define TASK_AC_MONITOR_FLAG  0x0004U
#define TASK_MPPT_FLAG        0x0008U
#define TASK_COMM_FLAG        0x0020U
#define TASK_UI_FLAG          0x0040U
#define TASK_EEPROM_FLAG      0x0080U
#define TASK_DC_CTRL_FLAG     0x0100U
#define TASK_REACTIVE_FLAG    0x0200U
#define TASK_POWER_FLAG       0x0400U

/* Periods derived from the common 1 ms scheduler tick. */
#define TASK_MEASURE_PERIOD_MS  3U
#define TASK_STATE_PERIOD_MS    5U
#define TASK_MPPT_PERIOD_MS     750U
#define TASK_UI_PERIOD_MS       2000U
#define TASK_COMM_PERIOD_MS     500U
#define TASK_EEPROM_PERIOD_MS   1000U
#define TASK_REACTIVE_PERIOD_MS 10U
#define TASK_POWER_PERIOD_MS    10U

typedef enum
{
    SCHED_TASK_MEASURE = 0U,
    SCHED_TASK_STATE,
    SCHED_TASK_AC_MONITOR,
    SCHED_TASK_POWER,
    SCHED_TASK_REACTIVE,
    SCHED_TASK_DC_CTRL,
    SCHED_TASK_MPPT,
    SCHED_TASK_COMM,
    SCHED_TASK_EEPROM,
    SCHED_TASK_UI,
    SCHED_TASK_COUNT
} SchedulerTaskId;

typedef struct
{
    Uint32 taskLastCycles[SCHED_TASK_COUNT];
    Uint32 taskMaxCycles[SCHED_TASK_COUNT];
    Uint32 batchLastCycles;
    Uint32 batchMaxCycles;
} SchedulerProbe;

/* Scheduler owns the writable object; other modules receive a read-only view. */
extern const volatile SchedulerProbe * const gSchedulerProbe;

void Scheduler_Init(void);
Uint16 Scheduler_TakeFlags(void);
void Scheduler_Tick1ms(void);
void Scheduler_NotifyGridZeroCross(void);
void Scheduler_NotifyGridPeak(void);


#if (SCHEDULER_PROFILE_ENABLE != 0U)

    Uint32 Scheduler_ProfileBegin(void);
    void Scheduler_ProfileTaskEnd(SchedulerTaskId taskId, Uint32 startCycles);
    void Scheduler_ProfileBatchEnd(Uint32 startCycles);

    #define SCHEDULER_PROFILE_BEGIN(startCycles)            ((startCycles) = Scheduler_ProfileBegin())
    #define SCHEDULER_PROFILE_TASK_END(taskId, startCycles) Scheduler_ProfileTaskEnd((taskId), (startCycles))
    #define SCHEDULER_PROFILE_BATCH_END(startCycles)        Scheduler_ProfileBatchEnd((startCycles))

#else

    #define SCHEDULER_PROFILE_BEGIN(startCycles)               ((void)0)
    #define SCHEDULER_PROFILE_TASK_END(taskId, startCycles)    ((void)0)
    #define SCHEDULER_PROFILE_BATCH_END(startCycles)           ((void)0)

#endif /*SCHEDULER_PROFILE_ENABLE*/

#endif /* SCHEDULER_H_ */
