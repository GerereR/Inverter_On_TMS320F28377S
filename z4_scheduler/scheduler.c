#include "F28x_Project.h"

#include "scheduler.h"

// Timer0 runs from the 200 MHz system clock. 200000 counts produces a 1 ms tick.
#define SCHEDULER_TICK_COUNTS  200000UL

volatile Uint16 Scheduler_Flags = 0;

void Scheduler_Config(void)
{
    EALLOW;

    CpuTimer0Regs.PRD.all = SCHEDULER_TICK_COUNTS - 1UL;
    CpuTimer0Regs.TPR.all = 0;
    CpuTimer0Regs.TPRH.all = 0;
    PieVectTable.TIMER0_INT = &Scheduler_ISR;

    EDIS;

    // CPU Timer0 is PIE group 1, channel 7.
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    IER |= M_INT1;

    CpuTimer0Regs.TCR.bit.TIF = 1;
    CpuTimer0Regs.TCR.bit.TRB = 1;
    CpuTimer0Regs.TCR.bit.TIE = 1;
    CpuTimer0Regs.TCR.bit.TSS = 0;
}

Uint16 Scheduler_GetFlags(void)
{
    return Scheduler_Flags;
}

void Scheduler_ClearFlags(Uint16 flags)
{
    Scheduler_Flags &= ~flags;
}

__interrupt void Scheduler_ISR(void)
{
    static Uint16 cnt10ms   = 0;
    static Uint16 cnt20ms   = 0;
    static Uint16 cnt50ms   = 0;
    static Uint16 cnt100ms  = 0;
    static Uint16 cnt500ms  = 0;
    static Uint16 cntComm   = 0;

    // Derive all cooperative task rates from the common 1 ms tick.
    if(++cnt10ms >= 10)
    {
        cnt10ms = 0;
        Scheduler_Flags |= TASK_STATE_FLAG;
    }

    if(++cnt20ms >= 20)
    {
        cnt20ms = 0;
        Scheduler_Flags |= TASK_MEASURE_FLAG;
    }

    if(++cnt50ms >= 50)
    {
        cnt50ms = 0;
        Scheduler_Flags |= TASK_GRID_FLAG;
    }

    if(++cnt100ms >= 100)
    {
        cnt100ms = 0;
        Scheduler_Flags |= TASK_MPPT_FLAG;
    }

    if(++cnt500ms >= 500)
    {
        cnt500ms = 0;
        Scheduler_Flags |= TASK_POWER_FLAG;
    }

    // Communication is a supervisory task and is serviced every 500 ms.
    if(++cntComm >= 500)
    {
        cntComm = 0;
        Scheduler_Flags |= TASK_COMM_FLAG;
    }

    CpuTimer0Regs.TCR.bit.TIF = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
