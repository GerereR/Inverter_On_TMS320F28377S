#ifndef TASK_H
#define TASK_H

// Cooperative task periods are assigned by scheduler.c.
void Task_State_Init(void);
void Task_State(void);

/* Reset the DC-side loops (bus + boost). Owned by task_dc_ctrl.c. */
void DC_Ctrl_Reset(void);
void DC_Ctrl_StartSoftStart(void);

/* 电流环假任务（由 ADC ISR 直接调用，非调度器触发）。 */
#define FAST_EVENT_NONE          0x0000U
#define FAST_EVENT_GRID_PEAK     0x0001U

void AC_Ctrl_Init(void);
Uint16 Task_AC_Ctrl(void);
void CheckGridPresence(Uint16 gridVoltRaw);
void AC_Ctrl_Enable(void);
void AC_Ctrl_Disable(void);

void Task_Measure(void);

void Task_AC_Monitor(void);

void Task_MPPT_Init(void);
void Task_MPPT(void);

void Task_DC_Ctrl(void);

void Task_Reactive_Init(void);
void Task_Reactive(void);

void Task_Power_Init(void);
void Task_Power(void);

void Task_Comm_Init(void);
void Task_Comm(void);

void Task_UI_Init(void);
void Task_UI(void);

void Task_Eeprom_Init(void);
void Task_Eeprom(void);
#define EEPROM_SAVE_CALIBRATION  1U
void EEPROM_RequestSave(Uint16 saveGroup);

#endif
