#include "F28x_Project.h"

#include "bsp.h"

void CPU_InterruptInit(void)
{
    DINT;
    IER = 0x0000U;
    IFR = 0x0000U;
}

void CPU_InterruptEnable(void)
{
    EINT;
    ERTM;
}

Uint16 CPU_InterruptSaveDisable(void)
{
    return __disable_interrupts();
}

void CPU_InterruptRestore(Uint16 interruptState)
{
    if((interruptState & 0x0001U) == 0U)
    {
        EINT;
    }
    if((interruptState & 0x0002U) == 0U)
    {
        ERTM;
    }
}
