#ifndef TASK_H
#define TASK_H

void Task_Init(void);

// Cooperative task periods are assigned by scheduler.c.
void Task_State_Init(void);
void Task_State(void);

void Task_Measure(void);

void Task_Grid(void);

void Task_MPPT_Init(void);
void Task_MPPT(void);

void Task_DcCtrl(void);

void Task_ReactiveCtrl_Init(void);
void Task_ReactiveCtrl(void);

void Task_PowerLimit_Init(void);
void Task_PowerLimit(void);

void Task_Comm_Init(void);
void Task_Comm(void);

void Task_UI_Init(void);
void Task_UI(void);

void Task_Eeprom_Init(void);
void Task_Eeprom(void);
#define EEPROM_SAVE_CALIBRATION  1U
void EEPROM_RequestSave(Uint16 saveGroup);

#endif
