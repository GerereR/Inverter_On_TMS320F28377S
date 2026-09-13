#include "F28x_Project.h"

#include "inverter.h"
#include "scheduler.h"
#include "task.h"
#include "bsp.h"

int main(void)
{
    Uint16 schedulerFlags;
#if (SCHEDULER_PROFILE_ENABLE != 0U)
    Uint32 taskStartCycles;
    Uint32 batchStartCycles;
#endif

    //电流环"任务"初始化.
    AC_Ctrl_Init();  

    //逆变器级初始化
    Inverter_Init();

    //各个任务初始化
    Task_Eeprom_Init();
    Task_Comm_Init();
    Task_State_Init();
    Task_MPPT_Init();
    Task_Reactive_Init();
    Task_Power_Init();
    Task_UI_Init();
    
    //EPWM使能,也意味着ADC开始采用
    EPWM_Start();

    //调度器初始化
    Scheduler_Init();

    //CPU中断使能
    CPU_InterruptEnable();

    while(1)
    {
        //获取任务标志位
        schedulerFlags = Scheduler_TakeFlags();

        if(schedulerFlags == 0U)
        {
            continue;
        }

        //统计一次while循环所需时间(如果使能的话)
        SCHEDULER_PROFILE_BEGIN(batchStartCycles);

        //测量任务,用于把ADC原码转换为真实的有效值和平均值,同时算出当前功率
        if(schedulerFlags & TASK_MEASURE_FLAG)     // 3 ms
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Measure();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_MEASURE, taskStartCycles);
        }

        //状态机任务
        if(schedulerFlags & TASK_STATE_FLAG)       // 5 ms
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_State();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_STATE, taskStartCycles);
        }

        //交流测保护任务
        if(schedulerFlags & TASK_AC_MONITOR_FLAG)  // 20ms 电网过零点下降沿触发
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_AC_Monitor();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_AC_MONITOR, taskStartCycles);
        }

        //功率限额任务
        if(schedulerFlags & TASK_POWER_FLAG)        // 10 ms 
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Power();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_POWER, taskStartCycles);
        }

        //无功控制任务
        if(schedulerFlags & TASK_REACTIVE_FLAG)     // 10 ms 
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Reactive();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_REACTIVE, taskStartCycles);
        }

        //直流侧控制任务
        if(schedulerFlags & TASK_DC_CTRL_FLAG)     // 10ms 电网峰值谷值触发
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_DC_Ctrl();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_DC_CTRL, taskStartCycles);
        }

        //MPPT任务
        if(schedulerFlags & TASK_MPPT_FLAG)        // 750 ms
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_MPPT();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_MPPT, taskStartCycles);
        }

        //UART通讯任务,需使用专用工具
        if(schedulerFlags & TASK_COMM_FLAG)        // 500 ms  被动触发
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Comm();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_COMM, taskStartCycles);
        }

        //EEPROM任务
        if(schedulerFlags & TASK_EEPROM_FLAG)      // 1s 被动触发
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_Eeprom();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_EEPROM, taskStartCycles);
        }
        
        //UI显示任务
        if(schedulerFlags & TASK_UI_FLAG)          // 2s, 
        {
            SCHEDULER_PROFILE_BEGIN(taskStartCycles);
            Task_UI();
            SCHEDULER_PROFILE_TASK_END(SCHED_TASK_UI, taskStartCycles);
        }

        //计算一次while循环耗时
        SCHEDULER_PROFILE_BATCH_END(batchStartCycles);
    }
}
