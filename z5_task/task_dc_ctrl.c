#include "F28x_Project.h"

#include "task.h"
#include "bsp.h"
#include "variable.h"
#include "control.h"

void Task_DcCtrl(void)
{
    float busVoltage;
    busVoltage = gMachineData.realAvg.dcBusVoltage;
    /* Supervisory gating stays in the task layer. The control modules only
     * calculate commands and do not decide whether the power stage may run. */
     //如果不在并网态, 或者有任何的错误, 或者BUS不正常
    if
    (
        (gSysData.state != SYS_STATE_NORMAL) ||     
        (gSysFault.word.recoverable != 0U) ||       
        (gSysFault.word.permanent != 0U) ||
        (busVoltage <= 0.0f)           
    )
    {
        Ctrl_BusReset();
        Ctrl_BoostReset();
        EPWM_SetBoostDuty(0.0f, 0.0f);
        Ctrl_SetInductorCurrentAmp(0.0f);
        return;
    }

    //母线环根据当前母线电压与目标值的误差，计算逆变器应该输出多大的电流幅值。
    Ctrl_BusRun(busVoltage, gPowerLimitData.currentAmpLimit);

    //把母线环结果交给电流环
    Ctrl_SetInductorCurrentAmp(gBusCtrlData.currentAmpRef);

    Ctrl_BoostRun
    (
        gMachineData.realAvg.pv1Voltage,
        gMpptData.pv1.voltRef,
        gMpptData.pv1.enabled,
        gMachineData.realAvg.pv2Voltage,
        gMpptData.pv2.voltRef,
        gMpptData.pv2.enabled
    );

    EPWM_SetBoostDuty(gBusCtrlData.boost1Duty, gBusCtrlData.boost2Duty);
}
