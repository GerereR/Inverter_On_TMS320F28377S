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
volatile GridSafetyParams gGridSafety =
{
    220.0f, 50.0f,                         /* 额定电压/频率 */
    242.0f, 264.0f, 187.0f, 176.0f,        /* 过/欠压一级/二级 */
    50.5f, 51.0f, 49.5f, 49.0f,            /* 过/欠频一级/二级 */
    253.0f,                                /* 10 分钟平均过压 */
    242.0f, 187.0f, 50.5f, 49.5f,          /* 重连阈值 */
    3U, 3U, 300U                           /* 一级/二级判定计数 + 恢复计数 */
};
volatile GfciData gGfciData = {0};
volatile ReactiveCtrlData gReactiveData = {0};
volatile RelayCtrlData gRelayData = {0};
volatile SPLL_1ph GridSPLL = {0};

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

/* ADC 运行时零漂校准状态，开机/重连时采 32 组码值平均得到零漂。 */
volatile AdcOffsetCal gAdcOffsetCal = {0};
