#ifndef CONTROL_H_
#define CONTROL_H_

#include "pll.h"

#define CTRL_EVENT_NONE          0x0000U
#define CTRL_EVENT_GRID_PEAK     0x0001U

void Ctrl_Init(void);
Uint16 Ctrl_FastRun(float gridVoltPllIn);
void Ctrl_Enable(void);
void Ctrl_Disable(void);
void Ctrl_SetInductorCurrentAmp(float amp);
float Ctrl_GetInductorCurrentAmp(void);

#endif /* CONTROL_H_ */
