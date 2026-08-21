#ifndef VARIABLE_H_
#define VARIABLE_H_

#include "constant.h"

/*
 * Runtime machine snapshot shared by measurement and communication tasks.
 * Frequency is stored in 0.01 Hz units.
 */
typedef struct
{
    Uint16 gridVoltage;
    Uint16 inductorCurrent;
    Uint16 pvVoltage;
    Uint16 pvCurrent;
    Uint16 gridFrequencyCentihertz;
    Uint16 pllPhaseMilliradian;
    Uint16 pllLocked;
    Uint16 tripZoneFaulted;
} MachineDataSnapshot;

extern volatile MachineDataSnapshot gMachineData;

#endif
