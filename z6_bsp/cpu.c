#include "F28x_Project.h"

#include "bsp.h"

//CPU级中断清除函数
void CPU_InterruptInit(void)
{
    // INTM = 1，关全局中断
    DINT;
    // 清所有中断使能位
    IER = 0x0000U;
    // 清所有中断标志位
    IFR = 0x0000U;
}

//CPU级中断使能函数
void CPU_InterruptEnable(void)
{
    // INTM = 0，开全局中断
    EINT;
    // DBGM = 0，允许调试
    ERTM;
}

//保存当前中断状态，同时关中断
Uint16 CPU_InterruptSaveDisable(void)
{
    //TI 编译器提供的内联函数
    //读取 ST1 寄存器，把当前 INTM/DBGM 状态保存下来, 
    //然后执行 DINT，关全局中断
    //最后返回保存的状态值
    return __disable_interrupts();
}

//根据保存的状态，恢复中断使能
void CPU_InterruptRestore(Uint16 interruptState)
{
    //如果原状态是 0（中断开着），就执行 EINT 恢复开中断
    if((interruptState & 0x0001U) == 0U)
    {
        EINT;
    }
    //如果原状态是 1（中断关着），就什么都不做，保持关中断
    if((interruptState & 0x0002U) == 0U)
    {
        ERTM;
    }
}
