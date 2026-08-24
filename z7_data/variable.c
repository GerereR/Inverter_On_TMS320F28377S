#include "F28x_Project.h"
#include "variable.h"

volatile MachineData gMachineData = {0};

/* PLL starts faulted until the fast loop confirms a valid lock condition. */
volatile SysFault gSysFault = {1U, 0U};


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
