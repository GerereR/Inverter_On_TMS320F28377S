#include "F28x_Project.h"

#include "bsp.h"
#include "scheduler.h"

/* Timer0 runs from the 200 MHz system clock. */
#define SCHEDULER_TICK_COUNTS  200000UL

static __interrupt void SchedulerTimer_BSP_ISR(void);

void SchedulerTimer_Config(void)
{
    EALLOW;
    CpuTimer0Regs.PRD.all = SCHEDULER_TICK_COUNTS - 1UL;
    CpuTimer0Regs.TPR.all = 0U;
    CpuTimer0Regs.TPRH.all = 0U;
    PieVectTable.TIMER0_INT = &SchedulerTimer_BSP_ISR;
    EDIS;

    /* CPU Timer0 is PIE group 1, channel 7. */
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1U;
    IER |= M_INT1;

    CpuTimer0Regs.TCR.bit.TIF = 1U;
    CpuTimer0Regs.TCR.bit.TRB = 1U;
    CpuTimer0Regs.TCR.bit.TIE = 1U;
    CpuTimer0Regs.TCR.bit.TSS = 0U;
}

static __interrupt void SchedulerTimer_BSP_ISR(void)
{
    Scheduler_Tick1ms();
    CpuTimer0Regs.TCR.bit.TIF = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
