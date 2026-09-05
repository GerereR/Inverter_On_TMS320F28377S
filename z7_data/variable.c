#include "F28x_Project.h"
#include "variable.h"

volatile MachineData gMachineData = {0};

/* PLL starts faulted until the fast loop confirms a valid lock condition. */
volatile SysFault gSysFault = {1U};
volatile SysData gSysData = {SYS_STATE_WAIT, MODEL_3KW};

/* Shared runtime data is grouped by the function that produces it. */
volatile MpptData gMpptData = {0};
volatile BusCtrlData gBusCtrlData = {0};
volatile InvCtrlData gInvCtrlData = {0};
volatile PowerLimitData gPowerLimitData = {0};
volatile GridMonitorData gGridData = {0};
volatile ReactiveCtrlData gReactiveData = {0};
volatile RelayCtrlData gRelayData = {0};

volatile ADC_Calibrate gAdcCal =
{
    {ADC_BIPOLAR_ZERO, ADC_GRID_VOLTAGE_GAIN},
    {ADC_BIPOLAR_ZERO, ADC_INDUCTOR_CURRENT_GAIN},
    {ADC_BIPOLAR_ZERO, ADC_GFCI_CURRENT_GAIN},
    {0.0f, ADC_DC_BUS_VOLTAGE_GAIN},
    {ADC_BIPOLAR_ZERO, ADC_INVERTER_DC_CURRENT_GAIN},
    {ADC_BIPOLAR_ZERO, ADC_INVERTER_VOLTAGE_GAIN},
    {0.0f, ADC_PV_CURRENT_GAIN},
    {0.0f, ADC_PV_CURRENT_GAIN},
    {0.0f, ADC_PV_VOLTAGE_GAIN},
    {0.0f, ADC_PV_VOLTAGE_GAIN},
    {0.0f, ADC_ISOLATION_VOLTAGE_GAIN},
    {0.0f, ADC_ISOLATION_VOLTAGE_GAIN}
};
