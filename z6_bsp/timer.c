#include "F28x_Project.h"

#include "bsp.h"
#include "scheduler.h"

//配置timer0时间,1ms
#define TIMER0_PERIOD_US       1000UL
#define TIMER0_PERIOD_COUNTS   (SYSCLK_CYCLES_PER_US * TIMER0_PERIOD_US)

static __interrupt void Timer0_BSP_ISR(void);

void Timer2_Init(void)
{
    //失能timer2
    CpuTimer2Regs.TCR.bit.TSS = 1U;
    EALLOW;
    //采用系统时钟,不分频
    CpuSysRegs.TMR2CLKCTL.bit.TMR2CLKSRCSEL = 0U;
    CpuSysRegs.TMR2CLKCTL.bit.TMR2CLKPRESCALE = 0U;
    EDIS;
    //周期计数器
    CpuTimer2Regs.PRD.all = 0xFFFFFFFFUL;
    //预分频清零
    CpuTimer2Regs.TPR.all = 0U;
    CpuTimer2Regs.TPRH.all = 0U;
    //禁止中断
    CpuTimer2Regs.TCR.bit.TIE = 0U;
    //调试相关
    CpuTimer2Regs.TCR.bit.SOFT = 0U;
    CpuTimer2Regs.TCR.bit.FREE = 0U;
    //清除中断
    CpuTimer2Regs.TCR.bit.TIF = 1U;
    //PRD加载到之前寄存器
    CpuTimer2Regs.TCR.bit.TRB = 1U;
    //启用timer2
    CpuTimer2Regs.TCR.bit.TSS = 0U;
}

Uint32 Timer2_GetCount(void)
{
    //读取计数器
    return CpuTimer2Regs.TIM.all;
}

void Timer0_Init(void)
{
    //失能timer0和它的中断
    CpuTimer0Regs.TCR.bit.TSS = 1U;
    CpuTimer0Regs.TCR.bit.TIE = 0U;
    EALLOW;
    //计数器记完代表1ms时间
    CpuTimer0Regs.PRD.all = TIMER0_PERIOD_COUNTS - 1UL;
    //手动清零
    CpuTimer0Regs.TPR.all = 0U;
    CpuTimer0Regs.TPRH.all = 0U;
    //中断挂接到中断函数
    PieVectTable.TIMER0_INT = &Timer0_BSP_ISR;
    EDIS;
    //查表发现timer0中断位于PIE的Group 1，ch 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1U;
    IER |= M_INT1;
    //清除中断标志
    CpuTimer0Regs.TCR.bit.TIF = 1U;
    //装载周期寄存器
    CpuTimer0Regs.TCR.bit.TRB = 1U;
    //调试相关
    CpuTimer0Regs.TCR.bit.SOFT = 0U;
    CpuTimer0Regs.TCR.bit.FREE = 0U;
    //中断使能
    CpuTimer0Regs.TCR.bit.TIE = 1U;
    //启动timer0
    CpuTimer0Regs.TCR.bit.TSS = 0U;
}

static __interrupt void Timer0_BSP_ISR(void)
{
    //给调度器一个1ms节拍
    Scheduler_Tick1ms();
    //清除寄存器中断
    CpuTimer0Regs.TCR.bit.TIF = 1U;
    //清除PIE中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
