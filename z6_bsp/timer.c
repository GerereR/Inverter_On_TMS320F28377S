#include "F28x_Project.h"

#include "bsp.h"
#include "scheduler.h"

/* Timer0 runs from the 200 MHz system clock. */
#define SCHEDULER_TICK_COUNTS  200000UL

static __interrupt void Timer0_BSP_ISR(void);

void Timer2_Init(void)
{
    /* Free-running SYSCLK counter for scheduler profiling; no interrupt. */
    CpuTimer2Regs.TCR.bit.TSS = 1U;
    CpuTimer2Regs.PRD.all = 0xFFFFFFFFUL;
    CpuTimer2Regs.TPR.all = 0U;
    CpuTimer2Regs.TPRH.all = 0U;
    CpuTimer2Regs.TCR.bit.TIE = 0U;
    CpuTimer2Regs.TCR.bit.TIF = 1U;
    CpuTimer2Regs.TCR.bit.TRB = 1U;
    CpuTimer2Regs.TCR.bit.TSS = 0U;
}

Uint32 Timer2_GetCount(void)
{
    return CpuTimer2Regs.TIM.all;
}

void Timer0_Init(void)
{
    EALLOW;
    CpuTimer0Regs.PRD.all = SCHEDULER_TICK_COUNTS - 1UL;
    CpuTimer0Regs.TPR.all = 0U;
    CpuTimer0Regs.TPRH.all = 0U;
    PieVectTable.TIMER0_INT = &Timer0_BSP_ISR;
    EDIS;

    /* CPU Timer0 is PIE group 1, channel 7. */
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1U;
    IER |= M_INT1;

    CpuTimer0Regs.TCR.bit.TIF = 1U;
    CpuTimer0Regs.TCR.bit.TRB = 1U;
    CpuTimer0Regs.TCR.bit.TIE = 1U;
    CpuTimer0Regs.TCR.bit.TSS = 0U;
}

static __interrupt void Timer0_BSP_ISR(void)
{
    Scheduler_Tick1ms();
    CpuTimer0Regs.TCR.bit.TIF = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
