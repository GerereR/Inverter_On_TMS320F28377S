#include "F28x_Project.h"

#include "sched.h"
#include "bsp.h"

//最长挂起次数4次, 超过就警告
#define SCHED_PEND_MAX  0x0004U

//周期任务标志位,负责记录所有任务的标志位
static volatile Uint16 Sched_PeriodFlags = 0U;
//eCAP下降沿过零标志位,用于唤醒AC监测任务
static volatile Uint16 Sched_GridZCrossPend = 0U;
//电网峰谷值标志位,用于唤醒限幅任务
static volatile Uint16 Sched_GridPeakPend = 0U;
//调度器探针数据域,用于存放一次循环各个任务花费的时间,注意:这儿是只读的!//定义一个 只读的 易变的 (SchedProbe类型)结构体 指针, 名叫(* gSchedProbe), 这个指针只能指向

/*这两句话的意思是
第一行: 定义一个静态的,易变的SchedProbe类型的数据, 名叫:Sched_ProbeData, 它的数据类型是一个结构体, 初始赋值全为零
第二行: 定义一个只读的,易变的SchedProbe类型的指针, 名叫gSchedProbe, 这个指针指向Sched_ProbeData, 同时且只能指向它,不可更改*/
static volatile SchedProbe Sched_ProbeData = {0};
const volatile SchedProbe * const gSchedProbe = &Sched_ProbeData;


//调度初始化
void Sched_Init(void)
{
    Uint16 taskId;

    Sched_PeriodFlags = 0U;
    Sched_GridZCrossPend = 0U;
    Sched_GridPeakPend = 0U;

    //调度器探针数据域初始化(ac控制任务dc监测任务不算,所以只有10个)
    for(taskId = 0U; taskId < SCHED_TASK_CNT; taskId++)
    {
        Sched_ProbeData.taskLastCycle[taskId] = 0UL;
        Sched_ProbeData.taskMaxCycle[taskId] = 0UL;
    }
    Sched_ProbeData.batchLastCycle = 0UL;
    Sched_ProbeData.batchMaxCycle = 0UL;

    //定时器初始化
    Timer0_Config();
    #if (SCHED_SLICE_ENABLE != 0U)
    Timer2_Config();
    #endif  
}

#if (SCHED_SLICE_ENABLE != 0U)

    //返回定时器当前计数器值
    Uint32 Sched_SliceBegin(void)
    {
        return Timer2_GetCnt();
    }

    //测量单个任务的耗时,采用向下计数,即使发生了回绕,相减之后的结果也依然正确
    void Sched_SliceTaskEnd(SchedTaskId taskId, Uint32 startCycle)
    {
        Uint32 deltaCycle;

        if(taskId >= SCHED_TASK_CNT)
        {
            return;
        }

        //算出并存放当前任务的耗时
        deltaCycle = startCycle - Timer2_GetCnt();
        Sched_ProbeData.taskLastCycle[taskId] = deltaCycle;

        //记录最长的一次耗时
        if(deltaCycle > Sched_ProbeData.taskMaxCycle[taskId])
        {
            Sched_ProbeData.taskMaxCycle[taskId] = deltaCycle;
        }
    }

    //测量一次while循环的耗时
    void Sched_SliceBatchEnd(Uint32 startCycle)
    {
        Uint32 deltaCycle;

        deltaCycle = startCycle - Timer2_GetCnt();
        Sched_ProbeData.batchLastCycle = deltaCycle;

        if(deltaCycle > Sched_ProbeData.batchMaxCycle)
        {
            Sched_ProbeData.batchMaxCycle = deltaCycle;
        }

        //如果一次循环超过了1ms, 那就发出警告, 此时说明CPU负担很重,有漏任务的风险!
        if(deltaCycle > SCHED_MAIN_LOOP_WARNING_CYCLE)
        {
            gSysProblem.warning |= WARNING_MAIN_LOOP_OVERRUN;
        }
    }
    
#endif

//消费取得的任务标志位
Uint16 Sched_TakeFlags(void)
{
    Uint16 flags;
    Uint16 interruptState;

    //关CPU中断
    interruptState = CPU_InterruptSaveDisable();
    //交接所有任务的标志位,然后清零准备下一次收集
    flags = Sched_PeriodFlags;
    Sched_PeriodFlags = 0U;

    //这两个标志位和其他标志位不同的是,它们自生不仅是标志位,也是计数器
    //这恰恰说明这两个任务比较重要,不能遗漏!
    if(Sched_GridZCrossPend != 0U)
    {
        Sched_GridZCrossPend--;
        flags |= TASK_AC_GUARD_FLAG;
    }
    if(Sched_GridPeakPend != 0U)
    {
        Sched_GridPeakPend--;
        flags |= TASK_DC_CTRL_FLAG;
    }

    //恢复CPU中断
    CPU_InterruptRestore(interruptState);

    return flags;
}

//过零标志位生产者
void Sched_NoteGridZCross(void)
{
    if(Sched_GridZCrossPend < SCHED_PEND_MAX)
    {
        Sched_GridZCrossPend++;
    }
    //挂起超过WARNING_SCHED_PEND_OVERFLOW次就警告,说明CPU负担太重
    else
    {
        gSysProblem.warning |= WARNING_SCHED_PEND_OVERFLOW;
    }

    //注意:测量任务其实是有两个触发源的,一个是3ms触发, 还有一个是过零触发
    //过零触发意味着我们应该立即把DMA里面的数据取走,否则会影响DMA的传输!
    //具体细节详见DMA模块
    Sched_PeriodFlags |= TASK_MEASU_FLAG;
}

//峰值标志位生产者
void Sched_NoteGridPeak(void)
{
    if(Sched_GridPeakPend < SCHED_PEND_MAX)
    {
        Sched_GridPeakPend++;
    }
    //挂起超过WARNING_SCHED_PEND_OVERFLOW次就警告,说明CPU负担太重
    else
    {
        gSysProblem.warning |= WARNING_SCHED_PEND_OVERFLOW;
    }
}

//1ms节拍产生器,绝大多数任务标志位大多数都是在这里被生产
void Sched_Tick1ms(void)
{
    static Uint16 cntMeasu = 0U;
    static Uint16 cntState = 0U;
    static Uint16 cntMppt = 0U;
#if (TASK_UI_ENABLE != 0U)
    static Uint16 cntUi = 0U;
#endif
    static Uint16 cntComm = 0U;
    static Uint16 cntEeprom = 0U;
    static Uint16 cntReactive = 0U;
    static Uint16 cntPower = 0U;

    //到时间就挂起标志位
    if(++cntMeasu >= TASK_MEASU_PERIOD_MS)
    {
        cntMeasu = 0U;
        Sched_PeriodFlags |= TASK_MEASU_FLAG;
    }

    if(++cntState >= TASK_STATE_PERIOD_MS)
    {
        cntState = 0U;
        Sched_PeriodFlags |= TASK_STATE_FLAG;
    }

    if(++cntMppt >= TASK_MPPT_PERIOD_MS)
    {
        cntMppt = 0U;
        Sched_PeriodFlags |= TASK_MPPT_FLAG;
    }

#if (TASK_UI_ENABLE != 0U)
    if(++cntUi >= TASK_UI_PERIOD_MS)
    {
        cntUi = 0U;
        Sched_PeriodFlags |= TASK_UI_FLAG;
    }
#endif

    if(++cntComm >= TASK_COMM_PERIOD_MS)
    {
        cntComm = 0U;
        Sched_PeriodFlags |= TASK_COMM_FLAG;
    }

    if(++cntEeprom >= TASK_EEPROM_PERIOD_MS)
    {
        cntEeprom = 0U;
        Sched_PeriodFlags |= TASK_EEPROM_FLAG;
    }

    if(++cntReactive >= TASK_REACTIVE_PERIOD_MS)
    {
        cntReactive = 0U;
        Sched_PeriodFlags |= TASK_REACTIVE_FLAG;
    }

    if(++cntPower >= TASK_POWER_PERIOD_MS)
    {
        cntPower = 0U;
        Sched_PeriodFlags |= TASK_POWER_FLAG;
    }
}
