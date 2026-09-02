#ifndef CONTROL_H_
#define CONTROL_H_

#include "pll.h"

#define CTRL_EVENT_NONE          0x0000U
#define CTRL_EVENT_GRID_PEAK     0x0001U

void Ctrl_Init(void);
Uint16 Ctrl_FastRun(float gridVoltAdc, float inductorCurrentAdc, float dcBusVoltAdc);
void Ctrl_Enable(void);
void Ctrl_Disable(void);
void Ctrl_SetInductorCurrentAmp(float amp);
float Ctrl_GetInductorCurrentAmp(void);

/* Supervisory control algorithms called by Task_DcCtrl(). */
void Ctrl_BusReset(void);
void Ctrl_BusRun(float busVoltage, float currentAmpLimit);
void Ctrl_BoostReset(void);
void Ctrl_BoostRun(float pv1Voltage,
                   float pv1VoltageRef,
                   Uint16 pv1Enabled,
                   float pv2Voltage,
                   float pv2VoltageRef,
                   Uint16 pv2Enabled);

#endif /* CONTROL_H_ */
