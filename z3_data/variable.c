#include "F28x_Project.h"
#include "variable.h"



/* Keep the PLL fault asserted until the fast loop confirms a valid lock. */
volatile SysFault gSysFault =
{
    .all = SYS_FAULT_PLL_STARTUP_MASK
};

volatile SysData gSysData =
{
    .state = SYS_STATE_WAIT,
    .model = MODEL_3KW
};

/* Defaults are named in constant.h so the active grid code has no magic values. */
volatile GridSafetyParams gGridSafety =
{
    .voltOverLevel1 = GRID_VOLT_OVER_LEVEL1_V,
    .voltOverLevel2 = GRID_VOLT_OVER_LEVEL2_V,
    .voltUnderLevel1 = GRID_VOLT_UNDER_LEVEL1_V,
    .voltUnderLevel2 = GRID_VOLT_UNDER_LEVEL2_V,
    .freqOverLevel1 = GRID_FREQ_OVER_LEVEL1_HZ,
    .freqOverLevel2 = GRID_FREQ_OVER_LEVEL2_HZ,
    .freqUnderLevel1 = GRID_FREQ_UNDER_LEVEL1_HZ,
    .freqUnderLevel2 = GRID_FREQ_UNDER_LEVEL2_HZ,
    .voltOver10Min = GRID_VOLT_OVER_10MIN_V,
    .reconnMaxVolt = GRID_RECONN_MAX_V,
    .reconnMinVolt = GRID_RECONN_MIN_V,
    .reconnMaxFreq = GRID_RECONN_MAX_FREQ_HZ,
    .reconnMinFreq = GRID_RECONN_MIN_FREQ_HZ,
    .faultFilterCount1 = GRID_FAULT_FILTER_COUNT_LEVEL1,
    .faultFilterCount2 = GRID_FAULT_FILTER_COUNT_LEVEL2,
    .backFilterCount = GRID_FAULT_BACK_FILTER_COUNT
};

//存放理论的偏置和增益
volatile ADC_Calibrate gAdcCal =
{
    .gridVoltage = {ADC_BIPOLAR_ZERO, ADC_GRID_VOLTAGE_GAIN},
    .inductorCurrent = {ADC_BIPOLAR_ZERO, ADC_INDUCTOR_CURRENT_GAIN},
    .gfciCurrent = {ADC_BIPOLAR_ZERO, ADC_GFCI_CURRENT_GAIN},
    .dcBusVoltage = {ADC_UNIPOLAR_ZERO, ADC_DC_BUS_VOLTAGE_GAIN},
    .gridDcCurrent = {ADC_BIPOLAR_ZERO, ADC_INVERTER_DC_CURRENT_GAIN},
    .inverterVoltage = {ADC_BIPOLAR_ZERO, ADC_INVERTER_VOLTAGE_GAIN},
    .pv1Current = {ADC_UNIPOLAR_ZERO, ADC_PV_CURRENT_GAIN},
    .pv2Current = {ADC_UNIPOLAR_ZERO, ADC_PV_CURRENT_GAIN},
    .pv1Voltage = {ADC_UNIPOLAR_ZERO, ADC_PV_VOLTAGE_GAIN},
    .pv2Voltage = {ADC_UNIPOLAR_ZERO, ADC_PV_VOLTAGE_GAIN},
    .pv1Isolation = {ADC_UNIPOLAR_ZERO, ADC_ISOLATION_VOLTAGE_GAIN},
    .pv2Isolation = {ADC_UNIPOLAR_ZERO, ADC_ISOLATION_VOLTAGE_GAIN}
};


volatile AdcOffsetCal   gAdcOffsetCal = {0}; //存放零漂校准值
volatile GfciData       gGfciData = {0};
volatile PLL_Data       GridPLL = {0};
volatile MachineData    gMachineData = {0};
volatile MpptData       gMpptData = {0};
volatile BusCtrlData    gBusCtrlData = {0};
volatile InvCtrlData    gInvCtrlData = {0};
volatile PowerLimitData gPowerLimitData = {0};
volatile ReactiveData   gReactiveData = {0};
volatile GridMonitorData gGridData = {0};
