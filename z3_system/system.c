#include "F28x_Project.h"

#include "system.h"
#include "bsp.h"
#include "../z8_control/control.h"

float System_Clamp(float value, float minimum, float maximum)
{
    if(value > maximum)
    {
        return maximum;
    }
    if(value < minimum)
    {
        return minimum;
    }
    return value;
}

void System_Init(void)
{
    /* Establish clocks and a known interrupt state before configuring BSPs. */
    InitSysCtrl();
    InitGpio();
    InitPieCtrl();
    DINT;
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    /* Configure peripherals while the power-stage time bases are stopped. */
    EPWM_Config();
    GPIO_Config();
    ADC_Config();
    DMA_Config();
    ECAP_Config();
    SCI_Config();
    I2C_Config();

    /* EPWM_Start() is called explicitly by main() after ISR setup. */
}

void System_EnterSafeOutput(void)
{
    /* This is the coordinated software shutdown path for task context. A TZ
     * event still clamps PWM in hardware before the state task reaches here. */
    EPWM_Disable();
    BOOST_OFF();
    INVERTER_OFF();
    GRID_RELAY1_OFF();
    GRID_RELAY2_OFF();
    GRID_RELAY3_OFF();
    GRID_RELAY4_OFF();
    GRID_RELAY_ALL_OFF();
    ISO_RELAY1_OFF();
    ISO_RELAY2_OFF();
    GFCI_CHECK_OFF();
    Ctrl_Disable();
    DSP_STATE_LOW();
}

/*============================================================================
 * 并网继电器自检（方案A：电网电压 - 逆变电压 压差判据）
 *
 * 对应原 G83_SLAVE 的 sSlaveRelayControl() / sRelayCheck()。
 * 原工程为 2ms 一拍（wWaitTime），此处用毫秒表达时序，由调用方按周期推进。
 * 4 个继电器分时吸合/断开，在特定时间窗用 |Vgrid - Vinv| 判粘连/失效。
 *==========================================================================*/
void System_RelaySelfTestInit(void)
{
    gRelayData.timerMs = 0U;
    /* 单 DSP：本机即为并网决策者，进入自检即视为允许完成并网。
     * 原工程此标志来自 Master 通过 SPI 下发的并网许可。 */
    gRelayData.relayOnFlag = 1U;
    gRelayData.faultFilter = 0U;
    gRelayData.selfTestPassed = 0U;
    gRelayData.fault = 0U;

    GRID_RELAY1_OFF();
    GRID_RELAY2_OFF();
    GRID_RELAY3_OFF();
    GRID_RELAY4_OFF();
}

static Uint16 System_RelayInWindow(Uint32 ms, Uint16 loMs, Uint16 hiMs)
{
    return ((ms >= (Uint32)loMs) && (ms <= (Uint32)hiMs)) ? 1U : 0U;
}

void System_RelaySelfTest(Uint16 deltaMs)
{
    float deltaV;
    float deltaVAbs;

    /* --- 时序推进 --- */
    gRelayData.timerMs += (Uint32)deltaMs;

    /* --- 时序执行（原 sSlaveRelayControl，拍换算为 ms） --- */
    if (gRelayData.timerMs == RELAY_SEQ_STEP1_MS)
    {
        GRID_RELAY1_ON();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP2_MS)
    {
        GRID_RELAY2_ON();
        GRID_RELAY3_ON();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP3_MS)
    {
        GRID_RELAY3_OFF();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP4_MS)
    {
        GRID_RELAY4_ON();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP5_MS)
    {
        GRID_RELAY2_OFF();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP6_MS)
    {
        GRID_RELAY3_ON();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP7_MS)
    {
        GRID_RELAY1_OFF();
    }
    else if (gRelayData.timerMs == RELAY_SEQ_STEP8_MS)
    {
        GRID_RELAY2_ON();
    }
    else if ((gRelayData.timerMs >= RELAY_SEQ_STEP9_MIN_MS) &&
             (gRelayData.timerMs <= RELAY_SEQ_STEP9_MAX_MS))
    {
        if (gRelayData.relayOnFlag == 1U)
        {
            GRID_RELAY1_ON();
            gRelayData.relayOnFlag = 2U;
        }
    }
    else if (gRelayData.timerMs > RELAY_SEQ_TIMEOUT_MS)
    {
        /* 超时未进 Normal：全断 + 复位 */
        GRID_RELAY1_OFF();
        GRID_RELAY2_OFF();
        GRID_RELAY3_OFF();
        GRID_RELAY4_OFF();
        gRelayData.timerMs = 0U;
        gRelayData.relayOnFlag = 1U;
    }

    /* --- 粘连/失效检测（原 sRelayCheck，方案A：电压差） --- */
    deltaV = gMachineData.realRms.gridVoltage - gMachineData.realRms.inverterVoltage;
    deltaVAbs = (deltaV < 0.0f) ? (-deltaV) : deltaV;

    if (System_RelayInWindow(gRelayData.timerMs, RELAY_WIN_A_MIN_MS, RELAY_WIN_A_MAX_MS) ||
        System_RelayInWindow(gRelayData.timerMs, RELAY_WIN_B_MIN_MS, RELAY_WIN_B_MAX_MS) ||
        System_RelayInWindow(gRelayData.timerMs, RELAY_WIN_C_MIN_MS, RELAY_WIN_C_MAX_MS) ||
        System_RelayInWindow(gRelayData.timerMs, RELAY_WIN_D_MIN_MS, RELAY_WIN_D_MAX_MS))
    {
        /* 隔离期：压差本应大，异常小则判粘连（该断的没断） */
        if (deltaVAbs < RELAY_DELTA_V_TRIP_V)
        {
            gRelayData.faultFilter += deltaMs;
            if (gRelayData.faultFilter > RELAY_FAULT_FILTER_MS)
            {
                gRelayData.fault = 1U;
            }
        }
        else
        {
            gRelayData.faultFilter = 0U;
        }
    }
    else if (System_RelayInWindow(gRelayData.timerMs, RELAY_WIN_E_MIN_MS, RELAY_WIN_E_MAX_MS))
    {
        /* 导通期：压差本应小，异常大则判失效（该合的没合） */
        if (deltaVAbs > RELAY_DELTA_V_TRIP_V)
        {
            gRelayData.faultFilter += deltaMs;
            if (gRelayData.faultFilter > RELAY_FAULT_FILTER_MS)
            {
                gRelayData.fault = 1U;
            }
        }
        else
        {
            gRelayData.faultFilter = 0U;
        }
    }
    else
    {
        gRelayData.faultFilter = 0U;
    }

    /* 检测窗口全部结束后且无故障 → 自检通过 */
    if ((gRelayData.timerMs > RELAY_SEQ_DONE_MS) && (gRelayData.fault == 0U))
    {
        gRelayData.selfTestPassed = 1U;
    }
}
