#include "F28x_Project.h"
#include "task.h"
#include "scheduler.h"
#include "bsp.h"
#include "system.h"
#include "variable.h"
#include "../z8_control/control.h"

static Uint16 State_SourcePresent(void);
static Uint16 State_SourceReady(void);
static Uint16 State_BusReady(void);
static Uint16 State_GridPresent(void);
static Uint16 State_GridReady(void);
static Uint16 State_HasRecoverableFault(void);
static Uint16 State_HasPermanentFault(void);

static void State_UpdateGridFaults(void);
static void State_ResetStartupData(void);
static void State_Enter(SysState nextState);

static void State_RunWait(void);
static void State_RunCheck(void);
static void State_RunNormal(void);
static void State_RunFault(void);
static void State_RunPermanent(void);

void Task_State_Init(void)
{
    gSysData.state = SYS_STATE_WAIT;
    gSysData.checkStage = SYS_CHECK_RESET;
    gSysData.startRequest = 0U;
    gSysData.sourceReady = 0U;
    gSysData.gridReady = 0U;
    gSysData.busReady = 0U;
    gSysData.boostReady = 0U;
    gSysData.inverterReady = 0U;
    gSysData.relayReady = 0U;
    gSysData.sourceStableMs = 0UL;
    gSysData.gridStableMs = 0UL;
    System_EnterSafeOutput();
}

void Task_State(void)
{
    Uint16 keyEvents;

    /* GPIO details and key debounce remain inside the BSP. Key actions will
     * be connected after the operator-control policy is finalized. */
    keyEvents = GPIO_GetKeyEvents();
    (void)keyEvents;
    gSysData.startRequest = POWER_SW1();

    switch(gSysData.state)
    {
        case SYS_STATE_WAIT:        State_RunWait();        break;
        case SYS_STATE_CHECK:       State_RunCheck();       break;
        case SYS_STATE_NORMAL:      State_RunNormal();      break;
        case SYS_STATE_FAULT:       State_RunFault();       break;
        case SYS_STATE_PERMANENT:   State_RunPermanent();   break;
        default:                    State_Enter(SYS_STATE_PERMANENT); break;
    }
}

/* A source is present at the lower run/hold threshold. This check is used
 * after startup and intentionally has hysteresis relative to PV_START_V. */
static Uint16 State_SourcePresent(void)
{
    return ((gMachineData.realAvg.pv1Voltage >= PV_PRESENT_MIN_V) ||
            (gMachineData.realAvg.pv2Voltage >= PV_PRESENT_MIN_V) ||
            (gMachineData.realAvg.dcBusVoltage >= DC_BUS_MIN_V)) ? 1U : 0U;
}

/* Require the source-start condition to remain valid continuously. A single
 * invalid state-task sample restarts the qualification interval. */
static Uint16 State_SourceReady(void)
{
    Uint16 valid;

    valid = ((gMachineData.realAvg.pv1Voltage >= PV_START_V) ||
             (gMachineData.realAvg.pv2Voltage >= PV_START_V) ||
             ((gMachineData.realAvg.dcBusVoltage >= DC_BUS_MIN_V) &&
              (gMachineData.realAvg.dcBusVoltage <= DC_BUS_MAX_V))) ? 1U : 0U;

    if(valid != 0U)
    {
        if(gSysData.sourceStableMs < SOURCE_QUALIFY_DELAY_MS)
        {
            gSysData.sourceStableMs += (Uint32)TASK_STATE_PERIOD_MS;
            if(gSysData.sourceStableMs > SOURCE_QUALIFY_DELAY_MS)
            {
                gSysData.sourceStableMs = SOURCE_QUALIFY_DELAY_MS;
            }
        }
    }
    else
    {
        gSysData.sourceStableMs = 0UL;
    }

    gSysData.sourceReady =
        (gSysData.sourceStableMs >= SOURCE_QUALIFY_DELAY_MS) ? 1U : 0U;
    return gSysData.sourceReady;
}

/* The current framework does not start Boost. Therefore CHECK only accepts
 * a bus that is already inside the normal operating window. */
static Uint16 State_BusReady(void)
{
    float busVoltage;

    busVoltage = gMachineData.realAvg.dcBusVoltage;
    gSysData.busReady = ((busVoltage >= DC_BUS_MIN_V) &&
                         (busVoltage <= DC_BUS_MAX_V)) ? 1U : 0U;
    return gSysData.busReady;
}

/* Update instantaneous grid diagnostics without applying reconnect timing. */
static Uint16 State_GridPresent(void)
{
    float gridFreqHz;
    float gridVoltageRms;

    gridFreqHz = (float)gMachineData.ecapFreqCent * 0.01f;
    gridVoltageRms = gMachineData.realRms.gridVoltage;

    gGridData.freqHz = gridFreqHz;
    gGridData.voltageValid =
        ((gridVoltageRms >= GRID_RECONN_MIN_RMS_V) &&
         (gridVoltageRms <= GRID_RECONN_MAX_RMS_V)) ? 1U : 0U;
    gGridData.freqValid =
        ((gridFreqHz >= GRID_RECONN_MIN_FREQ_HZ) &&
         (gridFreqHz <= GRID_RECONN_MAX_FREQ_HZ)) ? 1U : 0U;
    gGridData.gridPresent =
        ((gGridData.voltageValid != 0U) &&
         (gGridData.freqValid != 0U) &&
         (gGridData.fastPresent != 0U)) ? 1U : 0U;
    return gGridData.gridPresent;
}

/* Reconnect timing belongs to CHECK only. The timer is reset whenever either
 * voltage or frequency leaves the permitted window. */
static Uint16 State_GridReady(void)
{
    if(State_GridPresent() != 0U)
    {
        if(gSysData.gridStableMs < GRID_RECONN_DELAY_MS)
        {
            gSysData.gridStableMs += (Uint32)TASK_STATE_PERIOD_MS;
            if(gSysData.gridStableMs > GRID_RECONN_DELAY_MS)
            {
                gSysData.gridStableMs = GRID_RECONN_DELAY_MS;
            }
        }
    }
    else
    {
        gSysData.gridStableMs = 0UL;
    }

    gSysData.gridReady =
        (gSysData.gridStableMs >= GRID_RECONN_DELAY_MS) ? 1U : 0U;
    return gSysData.gridReady;
}

static Uint16 State_HasRecoverableFault(void)
{
    return (gSysFault.word.recoverable != 0U) ? 1U : 0U;
}

static Uint16 State_HasPermanentFault(void)
{
    return (gSysFault.word.permanent != 0U) ? 1U : 0U;
}

/* Grid fault bits are asserted only after the unit has reached NORMAL. In
 * FAULT they continue following the measurements so recovery can be seen. */
static void State_UpdateGridFaults(void)
{
    float gridFreqHz;
    float gridVoltageRms;

    gridFreqHz = (float)gMachineData.ecapFreqCent * 0.01f;
    gridVoltageRms = gMachineData.realRms.gridVoltage;

    gSysFault.bit.gridOverVolt =
        (gridVoltageRms > GRID_OV_TRIP_RMS_V) ? 1U : 0U;
    gSysFault.bit.gridUnderVolt =
        ((gridVoltageRms < GRID_UV_TRIP_RMS_V) ||
         (gGridData.fastPresent == 0U)) ? 1U : 0U;
    gSysFault.bit.gridOverFreq =
        (gridFreqHz > GRID_OF_TRIP_HZ) ? 1U : 0U;
    gSysFault.bit.gridUnderFreq =
        (gridFreqHz < GRID_UF_TRIP_HZ) ? 1U : 0U;

    (void)State_GridPresent();
}

/* Reset only startup/control runtime values. Measurement history and active
 * protection bits remain owned by their producer modules. */
static void State_ResetStartupData(void)
{
    gSysData.busReady = 0U;
    gSysData.boostReady = 0U;
    gSysData.inverterReady = 0U;
    gSysData.relayReady = 0U;

    gBusCtrlData.piIntegral = 0.0f;
    gBusCtrlData.piOut = 0.0f;
    gBusCtrlData.piOutPrev = 0.0f;
    gBusCtrlData.currentAmpRef = 0.0f;
    gBusCtrlData.boost1Duty = 0.0f;
    gBusCtrlData.boost2Duty = 0.0f;
    gBusCtrlData.initialized = 0U;
    gBusCtrlData.softStartActive = 0U;
    gBusCtrlData.softStartStage = 0U;
    gBusCtrlData.softStartTimerMs = 0UL;

    gInvCtrlData.currentErr = 0.0f;
    gInvCtrlData.currentErrPrev = 0.0f;
    gInvCtrlData.piIntegral = 0.0f;
    gInvCtrlData.piOut = 0.0f;
    gInvCtrlData.zeroCrossUpdatePending = 0U;
    Ctrl_Disable();
    EPWM_SetBoostDuty(0.0f, 0.0f);
}

static void State_Enter(SysState nextState)
{
    if(nextState == gSysData.state)
    {
        return;
    }

    if((nextState == SYS_STATE_WAIT) ||
       (nextState == SYS_STATE_FAULT) ||
       (nextState == SYS_STATE_PERMANENT))
    {
        System_EnterSafeOutput();
        gSysData.sourceReady = 0U;
        gSysData.gridReady = 0U;
        gSysData.busReady = 0U;
        gSysData.sourceStableMs = 0UL;
        gSysData.gridStableMs = 0UL;
    }
    else if(nextState == SYS_STATE_CHECK)
    {
        gSysData.checkStage = SYS_CHECK_RESET;
        gSysData.gridReady = 0U;
        gSysData.gridStableMs = 0UL;
    }

    gSysData.state = nextState;
}

static void State_RunWait(void)
{
    System_EnterSafeOutput();

    if(State_HasPermanentFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMANENT);
    }
    else if(State_HasRecoverableFault() != 0U)
    {
        gSysData.sourceStableMs = 0UL;
        gSysData.sourceReady = 0U;
    }
    else if(gSysData.startRequest == 0U)
    {
        gSysData.sourceStableMs = 0UL;
        gSysData.sourceReady = 0U;
    }
    else if(State_SourceReady() != 0U)
    {
        State_Enter(SYS_STATE_CHECK);
    }
}

static void State_RunCheck(void)
{
    System_EnterSafeOutput();

    if(State_HasPermanentFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMANENT);
        return;
    }
    if(State_HasRecoverableFault() != 0U)
    {
        State_Enter(SYS_STATE_FAULT);
        return;
    }
    if(gSysData.startRequest == 0U)
    {
        State_Enter(SYS_STATE_WAIT);
        return;
    }

    switch(gSysData.checkStage)
    {
        case SYS_CHECK_RESET:
            State_ResetStartupData();
            gSysData.checkStage = SYS_CHECK_SOURCE;
            break;

        case SYS_CHECK_SOURCE:
            if(State_SourceReady() != 0U)
            {
                gSysData.checkStage = SYS_CHECK_GRID;
            }
            else if(State_SourcePresent() == 0U)
            {
                State_Enter(SYS_STATE_WAIT);
            }
            break;

        case SYS_CHECK_GRID:
            if(State_SourcePresent() == 0U)
            {
                State_Enter(SYS_STATE_WAIT);
            }
            else if(State_GridReady() != 0U)
            {
                gSysData.checkStage = SYS_CHECK_BUS;
            }
            break;

        case SYS_CHECK_BUS:
            if(State_SourcePresent() == 0U)
            {
                State_Enter(SYS_STATE_WAIT);
            }
            else if(State_BusReady() != 0U)
            {
                gSysData.checkStage = SYS_CHECK_RELAY;
            }
            break;

        case SYS_CHECK_RELAY:
            /* Relay outputs have no feedback input in the current pin map.
             * Keep them open and do not report a successful self-test. */
            gSysData.relayReady = 0U;
            gSysData.checkStage = SYS_CHECK_PREPARE;
            break;

        case SYS_CHECK_PREPARE:
            if((State_SourcePresent() == 0U) ||
               (State_BusReady() == 0U) ||
               (State_GridPresent() == 0U))
            {
                gSysData.checkStage = SYS_CHECK_SOURCE;
                gSysData.sourceStableMs = 0UL;
                gSysData.gridStableMs = 0UL;
                gSysData.sourceReady = 0U;
                gSysData.gridReady = 0U;
            }
            else
            {
                /* ADC/DMA flags are owned by their ISRs and are not cleared
                 * here. Only the verified-inactive TZ latches are eligible. */
                if(EPWM_Enable() != 0U)
                {
                    Ctrl_Enable();
                    DSP_STATE_HIGH();
                    State_Enter(SYS_STATE_NORMAL);
                }
            }
            break;

        default:
            State_Enter(SYS_STATE_PERMANENT);
            break;
    }
}

static void State_RunNormal(void)
{
    State_UpdateGridFaults();

    if(State_HasPermanentFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMANENT);
    }
    else if(State_HasRecoverableFault() != 0U)
    {
        State_Enter(SYS_STATE_FAULT);
    }
    else if((gSysData.startRequest == 0U) ||
            (State_SourcePresent() == 0U) ||
            (State_BusReady() == 0U))
    {
        /* Normal PV depletion or loss of the DC source is not latched as a
         * protection failure. Stop safely and qualify again from WAIT. */
        State_Enter(SYS_STATE_WAIT);
    }
}

static void State_RunFault(void)
{
    System_EnterSafeOutput();
    State_UpdateGridFaults();

    if(State_HasPermanentFault() != 0U)
    {
        State_Enter(SYS_STATE_PERMANENT);
    }
    else if(State_HasRecoverableFault() == 0U)
    {
        gSysData.restartCount++;
        State_Enter(SYS_STATE_WAIT);
    }
}

static void State_RunPermanent(void)
{
    System_EnterSafeOutput();
}
