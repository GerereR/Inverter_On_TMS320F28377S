#include "F28x_Project.h"

#include "invert.h"
#include "sched.h"
#include "task.h"
#include "bsp.h"

int main(void)
{
    Uint16 schedFlags;
#if (SCHED_SLICE_ENABLE != 0U)
    Uint32 taskStartCycle;
    Uint32 batchStartCycle;
#endif

    //电流环"任务"初始化.
    AC_Ctrl_Init();  

    //逆变器级初始化
    Invert_Init();

    //各个任务初始化
    Task_Eeprom_Init();
    Task_Comm_Init();
    Task_State_Init();
    Task_MPPT_Init();
    Task_Reactive_Init();
    Task_Power_Init();
#if (TASK_UI_ENABLE != 0U)
    Task_UI_Init();
#endif
    
    //EPWM使能,也意味着ADC开始采集
    //但是并不代表EPWM开始发波!
    EPWM_Start();

    //调度器初始化
    Sched_Init();

    //CPU中断使能
    CPU_InterruptEnable();

    while(1)
    {
        //获取任务标志位
        schedFlags = Sched_TakeFlags();

        if(schedFlags == 0U)
        {
            continue;
        }

        //统计一次while循环所需时间(如果使能的话)
        SCHED_SLICE_BEGIN(batchStartCycle);

        //测量任务,用于把ADC原码转换为真实的有效值和平均值,同时算出当前功率
        if(schedFlags & TASK_MEASU_FLAG)     // 3 ms
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_Measu();
            SCHED_SLICE_TASK_END(SCHED_TASK_MEASU, taskStartCycle);
        }

        //状态机任务
        if(schedFlags & TASK_STATE_FLAG)       // 5 ms
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_State();
            SCHED_SLICE_TASK_END(SCHED_TASK_STATE, taskStartCycle);
        }

        //交流测保护任务
        if(schedFlags & TASK_AC_GUARD_FLAG)  // 20ms 电网过零点下降沿触发
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_AC_Guard();
            SCHED_SLICE_TASK_END(SCHED_TASK_AC_Guard, taskStartCycle);
        }

        //功率限额任务
        if(schedFlags & TASK_POWER_FLAG)        // 10 ms 
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_Power();
            SCHED_SLICE_TASK_END(SCHED_TASK_POWER, taskStartCycle);
        }

        //无功控制任务
        if(schedFlags & TASK_REACTIVE_FLAG)     // 10 ms 
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_Reactive();
            SCHED_SLICE_TASK_END(SCHED_TASK_REACTIVE, taskStartCycle);
        }

        //直流侧控制任务
        if(schedFlags & TASK_DC_CTRL_FLAG)     // 10ms 电网峰值谷值触发
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_DC_Ctrl();
            SCHED_SLICE_TASK_END(SCHED_TASK_DC_CTRL, taskStartCycle);
        }

        //MPPT任务
        if(schedFlags & TASK_MPPT_FLAG)        // 750 ms
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_MPPT();
            SCHED_SLICE_TASK_END(SCHED_TASK_MPPT, taskStartCycle);
        }

        //UART通讯任务,需使用专用工具
        if(schedFlags & TASK_COMM_FLAG)        // 500 ms  被动触发
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_Comm();
            SCHED_SLICE_TASK_END(SCHED_TASK_COMM, taskStartCycle);
        }

        //EEPROM任务
        if(schedFlags & TASK_EEPROM_FLAG)      // 1s 被动触发
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_Eeprom();
            SCHED_SLICE_TASK_END(SCHED_TASK_EEPROM, taskStartCycle);
        }
        
        //UI显示任务
#if (TASK_UI_ENABLE != 0U)
        if(schedFlags & TASK_UI_FLAG)          // 2s, 
        {
            SCHED_SLICE_BEGIN(taskStartCycle);
            Task_UI();
            SCHED_SLICE_TASK_END(SCHED_TASK_UI, taskStartCycle);
        }
#endif

        //计算一次while循环耗时
        SCHED_SLICE_BATCH_END(batchStartCycle);
    }
}
