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
    static Uint16 cntMeasure = 0U;
    static Uint16 cntState = 0U;
    static Uint16 cntGrid = 0U;
    static Uint16 cntPower = 0U;
    static Uint16 cntMppt = 0U;
    static Uint16 cntUi = 0U;
    static Uint16 cntComm = 0U;
    static Uint16 cntEeprom = 0U;

    // Derive all cooperative task rates from the common 1 ms tick.
    if(++cntMeasure >= TASK_MEASURE_PERIOD_MS)
    {
        cntMeasure = 0U;
        Scheduler_Flags |= TASK_MEASURE_FLAG;
    }

    if(++cntState >= TASK_STATE_PERIOD_MS)
    {
        cntState = 0U;
        Scheduler_Flags |= TASK_STATE_FLAG;
    }

    if(++cntGrid >= TASK_GRID_PERIOD_MS)
    {
        cntGrid = 0U;
        Scheduler_Flags |= TASK_GRID_FLAG;
    }

    if(++cntPower >= TASK_POWER_PERIOD_MS)
    {
        cntPower = 0U;
        Scheduler_Flags |= TASK_POWER_FLAG;
    }

    if(++cntMppt >= TASK_MPPT_PERIOD_MS)
    {
        cntMppt = 0U;
        Scheduler_Flags |= TASK_MPPT_FLAG;
    }

    if(++cntUi >= TASK_UI_PERIOD_MS)
    {
        cntUi = 0U;
        Scheduler_Flags |= TASK_UI_FLAG;
    }

    if(++cntComm >= TASK_COMM_PERIOD_MS)
    {
        cntComm = 0U;
        Scheduler_Flags |= TASK_COMM_FLAG;
    }

    if(++cntEeprom >= TASK_EEPROM_PERIOD_MS)
    {
        cntEeprom = 0U;
        Scheduler_Flags |= TASK_EEPROM_FLAG;
    }

    CpuTimer0Regs.TCR.bit.TIF = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
