#ifndef SCHED_H_
#define SCHED_H_

#include "F28x_Project.h"
#include "constant.h"

#ifndef SCHED_SLICE_ENABLE
    #define SCHED_SLICE_ENABLE  1U
#endif

//这里的timer2不分频
#define SCHED_SLICE_CYCLE_PER_US  SYSCLK_CYCLE_PER_US

/* A scheduled batch should finish within one 1 ms scheduler tick. */
#define SCHED_MAIN_LOOP_WARNING_US      1000UL
#define SCHED_MAIN_LOOP_WARNING_CYCLE  (SCHED_MAIN_LOOP_WARNING_US * SCHED_SLICE_CYCLE_PER_US)

//每个标志位占据的字节是不一样的
#define TASK_STATE_FLAG       (1UL << 0U)
#define TASK_MEASU_FLAG     (1UL << 1U)
#define TASK_AC_GUARD_FLAG  (1UL << 2U)
#define TASK_MPPT_FLAG        (1UL << 3U)
#define TASK_COMM_FLAG        (1UL << 4U)
#define TASK_UI_FLAG          (1UL << 5U)
#define TASK_EEPROM_FLAG      (1UL << 6U)
#define TASK_DC_CTRL_FLAG     (1UL << 7U)
#define TASK_REACTIVE_FLAG    (1UL << 8U)
#define TASK_POWER_FLAG       (1UL << 9U)

//每个任务的触发事件是不一样的
#define TASK_MEASU_PERIOD_MS  3U
#define TASK_STATE_PERIOD_MS    5U
#define TASK_MPPT_PERIOD_MS     750U
#define TASK_UI_PERIOD_MS       2000U
#define TASK_COMM_PERIOD_MS     500U
#define TASK_EEPROM_PERIOD_MS   1000U
#define TASK_REACTIVE_PERIOD_MS 10U
#define TASK_POWER_PERIOD_MS    10U

typedef enum
{
    SCHED_TASK_MEASU = 0U,
    SCHED_TASK_STATE,
    SCHED_TASK_AC_Guard,
    SCHED_TASK_POWER,
    SCHED_TASK_REACTIVE,
    SCHED_TASK_DC_CTRL,
    SCHED_TASK_MPPT,
    SCHED_TASK_COMM,
    SCHED_TASK_EEPROM,
    SCHED_TASK_UI,
    SCHED_TASK_CNT
} SchedTaskId;

typedef struct
{
    Uint32 taskLastCycle[SCHED_TASK_CNT];
    Uint32 taskMaxCycle[SCHED_TASK_CNT];
    Uint32 batchLastCycle;
    Uint32 batchMaxCycle;
} SchedProbe;

extern const volatile SchedProbe * const gSchedProbe;

void Sched_Init(void);
Uint16 Sched_TakeFlags(void);
void Sched_Tick1ms(void);
void Sched_NoteGridZCross(void);
void Sched_NoteGridPeak(void);


#if (SCHED_SLICE_ENABLE != 0U)

    Uint32 Sched_SliceBegin(void);
    void Sched_SliceTaskEnd(SchedTaskId taskId, Uint32 startCycle);
    void Sched_SliceBatchEnd(Uint32 startCycle);

    #define SCHED_SLICE_BEGIN(startCycle)            ((startCycle) = Sched_SliceBegin())
    #define SCHED_SLICE_TASK_END(taskId, startCycle) Sched_SliceTaskEnd((taskId), (startCycle))
    #define SCHED_SLICE_BATCH_END(startCycle)        Sched_SliceBatchEnd((startCycle))

#else

    #define SCHED_SLICE_BEGIN(startCycle)               ((void)0)
    #define SCHED_SLICE_TASK_END(taskId, startCycle)    ((void)0)
    #define SCHED_SLICE_BATCH_END(startCycle)           ((void)0)

#endif /*SCHED_SLICE_ENABLE*/

#endif /* SCHED_H_ */
