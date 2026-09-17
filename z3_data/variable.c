#include "F28x_Project.h"
#include "variable.h"



/* Keep the PLL fault asserted until the fast loop confirms a valid lock. */
volatile SysProblem gSysProblem =
{
    .warning = 0UL,
    .recovFault = RECOV_PLL_FAULT,
    .permaFault = 0UL
};

volatile SysData gSysData =
{
    .state = SYS_STATE_WAIT,
    .model = MODEL_3KW
};

/* Defaults are named in constant.h so the active grid code has no magic values. */
volatile GridSafetyParam gGridSafety =
{
    .voltOverLV1 =   GRID_VOLT_OVER_LV1_V,
    .voltOverLV2 =   GRID_VOLT_OVER_LV2_V,
    .voltUnderLV1 =  GRID_VOLT_UNDER_LV1_V,
    .voltUnderLV2 =  GRID_VOLT_UNDER_LV2_V,
    .freqOverLV1 =   GRID_FREQ_OVER_LV1_HZ,
    .freqOverLV2 =   GRID_FREQ_OVER_LV2_HZ,
    .freqUnderLV1 =  GRID_FREQ_UNDER_LV1_HZ,
    .freqUnderLV2 =  GRID_FREQ_UNDER_LV2_HZ,
    .voltOver10Min =    GRID_VOLT_OVER_10MIN_V,
    .reconnMaxVolt =    GRID_RECONN_MAX_V,
    .reconnMinVolt =    GRID_RECONN_MIN_V,
    .reconnMaxFreq =    GRID_RECONN_MAX_FREQ_HZ,
    .reconnMinFreq =    GRID_RECONN_MIN_FREQ_HZ,
    .faultFiltCnt1 = GRID_FAULT_FILT_CNT_LV1,
    .faultFiltCnt2 = GRID_FAULT_FILT_CNT_LV2,
    .backFiltCnt =  GRID_FAULT_BACK_FILT_CNT
};

//存放理论的偏置和增益
volatile AdcCal gAdcCal =
{
    .gridVolt =      {ADC_BIPOLAR_ZERO, ADC_GRID_VOLT_GAIN},
    .inductCurr =  {ADC_BIPOLAR_ZERO, ADC_INDUCT_CURR_GAIN},
    .gfciCurr =      {ADC_BIPOLAR_ZERO, ADC_GFCI_CURR_GAIN},
    .dcBusVolt =     {ADC_UNIPOLAR_ZERO, ADC_DC_BUS_VOLT_GAIN},
    .gridDcCurr =    {ADC_BIPOLAR_ZERO, ADC_INVERT_DC_CURR_GAIN},
    .invertVolt =  {ADC_BIPOLAR_ZERO, ADC_INVERT_VOLT_GAIN},
    .pv1Curr =       {ADC_UNIPOLAR_ZERO, ADC_PV_CURR_GAIN},
    .pv2Curr =       {ADC_UNIPOLAR_ZERO, ADC_PV_CURR_GAIN},
    .pv1Volt =       {ADC_UNIPOLAR_ZERO, ADC_PV_VOLT_GAIN},
    .pv2Volt =       {ADC_UNIPOLAR_ZERO, ADC_PV_VOLT_GAIN},
    .pv1Insul =     {ADC_UNIPOLAR_ZERO, ADC_INSUL_VOLT_GAIN},
    .pv2Insul =     {ADC_UNIPOLAR_ZERO, ADC_INSUL_VOLT_GAIN}
};


volatile AdcDrift       gAdcDrift = {0}; //存放零漂校准值
volatile GfciData       gGfciData = {0};
volatile PllData        gPllData = {0};
volatile MachineData    gMachineData = {0};
volatile MpptData       gMpptData = {0};
volatile BusCtrlData    gBusCtrlData = {0};
volatile InvCtrlData    gInvCtrlData = {0};
volatile PowerLimData   gPowerLimData = {0};
volatile ReactiveData   gReactiveData = {0};
volatile GridGuardData  gGridData = {0};
