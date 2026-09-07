#include "F28x_Project.h"
#include "bsp.h"
#include "variable.h"
#include "system.h"

#define EPWM_PERIOD_TICKS          2500U
#define EPWM_DEADBAND_TICKS        100U
#define EPWM_AQ_FORCE_DISABLED      0U
#define EPWM_AQ_FORCE_LOW           1U
#define EPWM_AQ_FORCE_HIGH          2U
#define EPWM_AQ_FORCE_IMMEDIATE     3U

/* Time-base synchronization modes used by the power-stage channels. */
#define EPWM_SYNC_INDEPENDENT       0U
#define EPWM_SYNC_MASTER            1U
#define EPWM_SYNC_SLAVE             2U

#define BEEP_PWM_CLOCK_HZ           25000000UL
#define BEEP_DEFAULT_FREQUENCY_HZ   2000U
#define BEEP_MIN_FREQUENCY_HZ       400U
#define BEEP_MAX_FREQUENCY_HZ       10000U

static volatile Uint16 EPWM_TripZoneFaulted = 0U;

static void EPWM_ConfigPowerStage
(
    volatile struct EPWM_REGS *pwm,
    Uint16 syncMode,
    Uint16 useDeadband,
    Uint16 useTz1,
    Uint16 useTz2,
    Uint16 useTz3
)
{
    pwm->TBCTL.all = 0U;
    pwm->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    pwm->TBCTL.bit.PHSEN = (syncMode == EPWM_SYNC_SLAVE) ? TB_ENABLE : TB_DISABLE;
    if(syncMode == EPWM_SYNC_MASTER)
    {
        pwm->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    }
    else if(syncMode == EPWM_SYNC_SLAVE)
    {
        pwm->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    }
    else
    {
        pwm->TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    }
    pwm->TBCTL.bit.PRDLD = TB_SHADOW;
    pwm->TBCTL.bit.HSPCLKDIV = TB_DIV1;
    pwm->TBCTL.bit.CLKDIV = TB_DIV1;
    pwm->TBCTL.bit.FREE_SOFT = 2U;
    pwm->TBPRD = EPWM_PERIOD_TICKS;
    pwm->TBPHS.all = 0U;
    pwm->TBCTR = 0U;

    pwm->CMPCTL.all = 0U;
    pwm->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    pwm->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    pwm->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    pwm->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
    pwm->CMPA.bit.CMPA = EPWM_PERIOD_TICKS / 2U;

    pwm->AQSFRC.bit.RLDCSF = EPWM_AQ_FORCE_IMMEDIATE;
    pwm->AQCTLA.all = 0U;
    pwm->AQCTLA.bit.CAU = AQ_SET;
    pwm->AQCTLA.bit.CAD = AQ_CLEAR;
    pwm->AQCTLB.all = 0U;
    if(useDeadband != 0U)
    {
        /* Inverter bridge: B is generated as the delayed complement of A. */
        pwm->DBCTL.all = 0U;
        pwm->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        pwm->DBCTL.bit.IN_MODE = DBA_ALL;
        pwm->DBCTL.bit.POLSEL = DB_ACTV_HIC;
        pwm->DBRED.bit.DBRED = EPWM_DEADBAND_TICKS;
        pwm->DBFED.bit.DBFED = EPWM_DEADBAND_TICKS;
    }
    else
    {
        /* Dual Boost: A and B are independent outputs on one shared timer. */
        pwm->DBCTL.all = 0U;
        pwm->AQCTLB.bit.CBU = AQ_CLEAR;
        pwm->AQCTLB.bit.CBD = AQ_SET;
        pwm->CMPB.bit.CMPB = EPWM_PERIOD_TICKS / 2U;
    }

    pwm->TZSEL.all = 0U;
    pwm->TZSEL.bit.OSHT1 = useTz1;
    pwm->TZSEL.bit.OSHT2 = useTz2;
    pwm->TZSEL.bit.OSHT3 = useTz3;
    pwm->TZCTL.bit.TZA = TZ_FORCE_LO;
    pwm->TZCTL.bit.TZB = TZ_FORCE_LO;
    pwm->TZEINT.bit.OST = ((useTz1 != 0U) || (useTz2 != 0U) ||
                           (useTz3 != 0U)) ? 1U : 0U;
    pwm->TZCLR.bit.OST = 1U;
    pwm->TZOSTCLR.bit.OST1 = 1U;
    pwm->TZOSTCLR.bit.OST2 = 1U;
    pwm->TZOSTCLR.bit.OST3 = 1U;
    pwm->TZCLR.bit.INT = 1U;
    pwm->AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    pwm->AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
}

void EPWM_Config(void)
{
    EALLOW;

    /* Freeze all ePWM time bases while configuring the power stage. */
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1U;

    /* Functional mapping: old EPWM2/3 -> inverter EPWM1/2; old EPWM4 ->
     * dual-Boost EPWM3; old EPWM6 remains a TZ2/ZVT support module. */
    EPWM_ConfigPowerStage(&EPwm1Regs, EPWM_SYNC_MASTER,      1U, 0U, 0U, 1U); /* old EPWM2, TZ3 */
    EPWM_ConfigPowerStage(&EPwm2Regs, EPWM_SYNC_SLAVE,       1U, 0U, 0U, 1U); /* old EPWM3, TZ3 */
    EPWM_ConfigPowerStage(&EPwm3Regs, EPWM_SYNC_INDEPENDENT, 0U, 1U, 0U, 0U); /* old EPWM4, dual Boost */

    /* Old EPWM6 was not a power PWM. Keep EPWM4 as a TZ2 monitor/reserved
     * ZVT carrier so its dedicated interrupt remains available. */
    EPwm4Regs.TBCTL.all = 0U;
    EPwm4Regs.TZSEL.all = 0U;
    EPwm4Regs.TZSEL.bit.OSHT2 = 1U;
    EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPwm4Regs.TZEINT.bit.OST = 1U;
    EPwm4Regs.TZCLR.bit.OST = 1U;
    EPwm4Regs.TZCLR.bit.INT = 1U;

    /* TZ3 is shared by EPWM1/2. EPWM1 owns the single CPU interrupt so the
     * same physical trip is not reported twice and EPWM2 cannot leave a
     * pending, unserviced TZ interrupt flag. */
    EPwm2Regs.TZEINT.bit.OST = 0U;
    EPwm2Regs.TZCLR.bit.INT = 1U;

    /* EPWM1 is the master ADC trigger at CTR=ZERO. */
    EPwm1Regs.ETSEL.bit.SOCAEN = 0U;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;
    EPwm1Regs.ETCLR.bit.SOCA = 1U;
    EPwm1Regs.ETSEL.bit.SOCAEN = 1U;

    /* One ISR per physical protection group. TZ3 is shared by EPWM1/2. */
    PieVectTable.EPWM1_TZ_INT = &EPWM1_TZ_BSP_ISR;
    PieVectTable.EPWM3_TZ_INT = &EPWM3_TZ_BSP_ISR;
    PieVectTable.EPWM4_TZ_INT = &EPWM4_TZ_BSP_ISR;
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1U;
    PieCtrlRegs.PIEIER2.bit.INTx3 = 1U;
    PieCtrlRegs.PIEIER2.bit.INTx4 = 1U;
    IER |= M_INT2;

    /* EPWM5A drives the passive buzzer on GPIO8 and starts silent. */
    EPwm5Regs.TBCTL.all = 0U;
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm5Regs.TBPRD = (Uint16)(BEEP_PWM_CLOCK_HZ / BEEP_DEFAULT_FREQUENCY_HZ - 1UL);
    EPwm5Regs.CMPCTL.all = 0U;
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPA.bit.CMPA = EPwm5Regs.TBPRD / 2U;
    EPwm5Regs.AQCTLA.all = 0U;
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQSFRC.bit.RLDCSF = EPWM_AQ_FORCE_IMMEDIATE;
    EPwm5Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;

    EDIS;
}

void BEEP_SetFreq(Uint16 freqHz)
{
    Uint32 periodTicks;

    if(freqHz < BEEP_MIN_FREQUENCY_HZ)
    {
        freqHz = BEEP_MIN_FREQUENCY_HZ;
    }
    else if(freqHz > BEEP_MAX_FREQUENCY_HZ)
    {
        freqHz = BEEP_MAX_FREQUENCY_HZ;
    }

    periodTicks = BEEP_PWM_CLOCK_HZ / (Uint32)freqHz;
    if(periodTicks > 0UL)
    {
        periodTicks--;
    }
    if(periodTicks > 65535UL)
    {
        periodTicks = 65535UL;
    }

    EALLOW;
    EPwm5Regs.TBPRD = (Uint16)periodTicks;
    EPwm5Regs.CMPA.bit.CMPA = (Uint16)(periodTicks / 2UL);
    EDIS;
}

void EPWM_Start(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    /* Keep the power outputs clamped. The running time base still supplies
     * EPWM1 SOCA to the ADC while the state machine performs CHECK. */
    EDIS;
}

void EPWM_Disable(void)
{
    EALLOW;
    EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
    EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
    EPwm3Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    EPwm3Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
    EDIS;
}

Uint16 EPWM_Enable(void)
{
    /* A low TZ input is an active hardware trip. Never release the software
     * clamps until all three physical protection inputs are inactive. */
    if(EPWM_TripZoneClear() == 0U)
    {
        return 0U;
    }

    EALLOW;
    EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
    EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
    EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    EPwm3Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
    EPwm3Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    EPwm4Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
    EPwm4Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    EDIS;
    return 1U;
}

void EPWM_TripZoneForce(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1U;
    EPwm2Regs.TZFRC.bit.OST = 1U;
    EPwm3Regs.TZFRC.bit.OST = 1U;
    EPwm4Regs.TZFRC.bit.OST = 1U;
    EDIS;
}

Uint16 EPWM_TripZoneClear(void)
{
    /* GPIO62/63/64 are the active-low TZ1/TZ2/TZ3 inputs. Clearing a
     * one-shot latch while any input is low would only hide an active fault. */
    if((GpioDataRegs.GPBDAT.bit.GPIO62 == 0U) ||
       (GpioDataRegs.GPBDAT.bit.GPIO63 == 0U) ||
       (GpioDataRegs.GPCDAT.bit.GPIO64 == 0U))
    {
        EPWM_TripZoneFaulted = 1U;
        gSysFault.bit.tzFault = 1U;
        return 0U;
    }

    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1U;
    EPwm1Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm1Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm1Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm1Regs.TZCLR.bit.INT = 1U;
    EPwm2Regs.TZCLR.bit.OST = 1U;
    EPwm2Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm2Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm2Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm2Regs.TZCLR.bit.INT = 1U;
    EPwm3Regs.TZCLR.bit.OST = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm3Regs.TZCLR.bit.INT = 1U;
    EPwm4Regs.TZCLR.bit.OST = 1U;
    EPwm4Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm4Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm4Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm4Regs.TZCLR.bit.INT = 1U;
    EDIS;

    EPWM_TripZoneFaulted = 0U;
    gSysFault.bit.tzFault = 0U;
    return 1U;
}

static void EPWM_RecordTrip(volatile struct EPWM_REGS *pwm)
{
    EPWM_TripZoneFaulted = 1U;
    gSysFault.bit.tzFault = 1U;
    pwm->TZCLR.bit.INT = 1U;
}

__interrupt void EPWM1_TZ_BSP_ISR(void)
{
    EPWM_RecordTrip(&EPwm1Regs);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

__interrupt void EPWM3_TZ_BSP_ISR(void)
{
    EPWM_RecordTrip(&EPwm3Regs);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

__interrupt void EPWM4_TZ_BSP_ISR(void)
{
    EPWM_RecordTrip(&EPwm4Regs);
    /* EPWM4 is the old EPWM6-style TZ2 monitor. Its ISR must disable the
     * real Boost outputs on EPWM3 because EPWM4 itself has no power PWM. */
    EPWM_Disable();
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

void EPWM_SetInverterMode(float modulation)
{
    Uint16 compareValue;

    modulation = System_Clamp(modulation, -1.0f, 1.0f);
    
    compareValue = (Uint16)(((modulation >= 0.0f) ? modulation : -modulation) * (float)EPWM_PERIOD_TICKS);
    EPwm1Regs.CMPA.bit.CMPA = compareValue;
    EPwm2Regs.CMPA.bit.CMPA = compareValue;

    /* Match the legacy unipolar full-bridge strategy. The inactive leg is
     * fixed high while the other leg is modulated with its complementary
     * dead-band output. Both legs use the same fixed state at zero, producing
     * zero differential bridge voltage. */
    if(modulation > 0.0f)
    {
        EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_HIGH;
        EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
        EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
        EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    }
    else if(modulation < 0.0f)
    {
        EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_HIGH;
        EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
        EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
        EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    }
    else
    {
        EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_HIGH;
        EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
        EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_HIGH;
        EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    }
}

void EPWM_SetBoostDuty(float boost1Duty, float boost2Duty)
{
    Uint16 boost1Compare;
    Uint16 boost2Compare;

    boost1Duty = System_Clamp(boost1Duty, 0.0f, 0.98f);
    boost1Duty = System_Clamp(boost2Duty, 0.0f, 0.98f);

    /* Match the old EPWM4 formulas for its two independent Boost outputs. */
    boost1Compare = (Uint16)((1.0f - boost1Duty) * (float)EPWM_PERIOD_TICKS);
    boost2Compare = (Uint16)(boost2Duty * (float)EPWM_PERIOD_TICKS);
    
    EPwm3Regs.CMPA.bit.CMPA = boost1Compare;
    EPwm3Regs.CMPB.bit.CMPB = boost2Compare;
}
