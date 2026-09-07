#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"
#include "variable.h"

/* 电网过/欠压、过/欠频、DCI、GFCI 的阈值与计数常量（原 constant.h 迁入，本文件私有）。 */

/* 10 分钟平均电压窗（长期过压保护）。每 30 秒采一个 RMS，20 槽环形平均。 */
#define GRID_10MIN_SLOT_COUNT         20U
#define GRID_10MIN_HALF_MIN_COUNT   1500U

static float GridVoltageRmsAvg10Min = 0.0f;
static float GridVoltageRmsBuf[GRID_10MIN_SLOT_COUNT] = {0};
static Uint16 GridVoltageRmsBufIdx = 0U;
static Uint16 GridHalfMinCount = 0U;

/* DCI（直流分量）检测与直流注入补偿。 */
#define DCI_DEADBAND_A           0.02f
#define DCI_ADJ_LIMIT            65.0f
#define DCI_FAULT_LIMIT_A        0.8f
#define DCI_ADJ_ACTIVE_COUNT     8U
#define DCI_ACTIVE_POWER_W       200.0f

/* GFCI 漏电保护阈值（物理值 A，对应原 G83 码值 GFCI*）。 */
#define GFCI_15MA_A                      0.015f   /* 注入检测下限：注入漏电流过小=硬件坏 */
#define GFCI_24MA_A                      0.024f   /* 30mA 档差分跳变阈值（原 GFCI24mA） */
#define GFCI_35MA_A                      0.035f   /* 静态漏电流检测阈值（原 GFCI35mA） */
#define GFCI_48MA_A                      0.048f   /* 60mA 档差分跳变阈值（原 GFCI48mA） */
#define GFCI_50MA_A                      0.050f   /* 恢复阈值（原 GFCI50mA） */
#define GFCI_85MA_A                      0.085f   /* 注入检测上限：注入漏电流过大（原 GFCI85mA） */
#define GFCI_120MA_A                     0.120f   /* 150mA 档差分跳变阈值，立即停（原 GFCI120mA） */
#define GFCI_280MA_A                     0.280f   /* 300mA 档绝对大漏电流（原 GFCI280mA） */

#define GFCI_DELTA_MIN_A                 0.001f   /* 缓慢变化最小差分（原 delta_gfi > 2 码） */
#define GFCI_PROTECT_DELAY_CYCLES        150U     /* 自检后保护静默期（原 check_delay >= 150） */
#define GFCI_FILTER_300MA                12U      /* 300mA 档连续判定（原 > 11） */
#define GFCI_FILTER_60MA                 8U       /* 60mA 档连续判定 */
#define GFCI_FILTER_30MA                 8U       /* 30mA 档连续判定 */
#define GFCI_DEV_FILTER                  6U       /* 自检设备故障连续判定（原 > 5） */
#define GFCI_BACK_FILTER                 276U     /* 恢复计数（原 > 275） */
#define GFCI_NOBREAK_CNT                 26U      /* 无突变复位计数（原 > 25） */
#define GFCI_SELFTEST_STATIC_END         30U      /* 自检静态检测阶段结束（原 index <= 30） */
#define GFCI_SELFTEST_INJECT_START       31U      /* 自检注入 50mA 起点（原 index == 31） */
#define GFCI_SELFTEST_INJECT_CHECK       33U      /* 自检注入检测起点（原 index >= 33） */
#define GFCI_SELFTEST_DONE               46U      /* 自检完成（注入断开） */

/*============================================================================
 * Task_AC_Monitor —— 交流侧慢速监测任务（过零触发，约 20ms）
 * 职责：电网过/欠压、过/欠频判定；DCI 检测 + 直流注入补偿 + 直流超标保护；
 *       GFCI 漏电自检 + 运行保护；过零时清电网丢失故障位。
 * 电网丢失的"检测"由状态机过零看门狗完成（本任务过零时只负责清标志）。
 *==========================================================================*/

/* DCI 检测 + 直流注入补偿 + 超标保护（原 signal_calc_dci + DciCheck）。 */
static void DCI_Inject(void)
{
    static Uint16 adjustmentCount = 0U;
    float dci = gMachineData.realAvg.gridDcCurrent;   /* 直流分量（物理值 A） */

    /* 激活计数：并网态每个电网周期 +1，脱离并网清零 */
    if (gSysData.state == SYS_STATE_NORMAL)
    {
        if (adjustmentCount < 9U)
        {
            adjustmentCount++;
        }
    }
    else
    {
        adjustmentCount = 0U;
    }

    /* 直流注入补偿：并网 + 稳定(>=8 周期) + 有功率，死区积分器调 dcCurrentComp */
    if 
    (
        (gSysData.state == SYS_STATE_NORMAL) &&
        (adjustmentCount >= DCI_ADJ_ACTIVE_COUNT) &&
        (gMachineData.powerData.gridActivePower > DCI_ACTIVE_POWER_W)
    )
    {
        if (dci > DCI_DEADBAND_A)
        {
            if (gInvCtrlData.dcCurrentComp < DCI_ADJ_LIMIT)
            {
                gInvCtrlData.dcCurrentComp += 1.0f;
            }
        }
        else if (dci < -DCI_DEADBAND_A)
        {
            if (gInvCtrlData.dcCurrentComp > -DCI_ADJ_LIMIT)
            {
                gInvCtrlData.dcCurrentComp -= 1.0f;
            }
        }
    }
}
static void DCI_Protect(void)
{
    float dci = gMachineData.realAvg.gridDcCurrent;
    float dciAbs = (dci < 0.0f) ? -dci : dci;
    static Uint16 dciFaultFilter = 0U;
    static Uint16 dciBackFilter = 0U;
    /* DCI 超标保护：并网 + 非打嗝 + 有功率，|DCI| 连续 3 次超阈值 → 故障 */
    if 
    (
        (gSysData.state == SYS_STATE_NORMAL) &&
        (gSysData.reloadFlag == 0U) &&
        (gMachineData.powerData.gridActivePower > DCI_ACTIVE_POWER_W)
    )
    {
        if (dciAbs > DCI_FAULT_LIMIT_A)
        {
            dciFaultFilter++;
            if (dciFaultFilter >= 3U)
            {
                dciFaultFilter = 0U;
                gSysFault.bit.gridDcCurrentFault = 1U;
            }
        }
        else
        {
            dciFaultFilter = 0U;
        }
    }
    else
    {
        dciFaultFilter = 0U;
    }

    /* DCI 恢复：|DCI| 持续低于阈值 250 周期(约 5s) → 清故障 */
    if (gSysFault.bit.gridDcCurrentFault != 0U)
    {
        if (dciAbs < DCI_FAULT_LIMIT_A)
        {
            dciBackFilter++;
            if (dciBackFilter >= 250U)
            {
                dciBackFilter = 0U;
                gSysFault.bit.gridDcCurrentFault = 0U;
            }
        }
        else
        {
            dciBackFilter = 0U;
        }
    }
}

/* 电网过/欠压、过/欠频判定（一级/二级阈值 + 独立计数滤波 + 恢复确认）。
 * 判定：连续 faultFilterCount 个电网周期超标 → 置故障；
 * 恢复：连续 backFilterCount 个周期回到正常范围 → 清故障。
 * 一级/二级使用独立滤波计数器，预留反时限（不同时间窗）。 */
static void GridVolt_Protect(void)
{
    static Uint16 voltOverFilter1 = 0U;   /* 一级过压滤波 */
    static Uint16 voltOverFilter2 = 0U;   /* 二级过压滤波 */
    static Uint16 voltUnderFilter1 = 0U;  /* 一级欠压滤波 */
    static Uint16 voltUnderFilter2 = 0U;  /* 二级欠压滤波 */
    static Uint16 voltBackFilter = 0U;
    float gridVoltRms = gMachineData.realRms.gridVoltage;

        /* --- 电压：过/欠压 --- */
    if (gSysFault.bit.gridOverVolt == 0U && gSysFault.bit.gridUnderVolt == 0U)
    {
        if (gridVoltRms > gGridSafety.voltOverLevel2)          /* 二级过压 */
        {
            voltOverFilter2++;
            voltOverFilter1 = 0U; 
            voltUnderFilter1 = 0U; 
            voltUnderFilter2 = 0U;
        }
        else if (gridVoltRms < gGridSafety.voltUnderLevel2)    /* 二级欠压 */
        {
            voltUnderFilter2++;
            voltOverFilter1 = 0U; 
            voltOverFilter2 = 0U; 
            voltUnderFilter1 = 0U;
        }
        else if (gridVoltRms > gGridSafety.voltOverLevel1)     /* 一级过压 */
        {
            voltOverFilter1++;
            voltOverFilter2 = 0U; 
            voltUnderFilter1 = 0U; 
            voltUnderFilter2 = 0U;
        }
        else if (gridVoltRms < gGridSafety.voltUnderLevel1)    /* 一级欠压 */
        {
            voltUnderFilter1++;
            voltOverFilter1 = 0U; 
            voltOverFilter2 = 0U; 
            voltUnderFilter2 = 0U;
        }
        else if (GridVoltageRmsAvg10Min > gGridSafety.voltOver10Min)  /* 10 分钟长期过压，立即置故障 */
        {
            gSysFault.bit.gridOverVolt = 1U;
            voltOverFilter1 = 0U; 
            voltOverFilter2 = 0U;
            voltUnderFilter1 = 0U; 
            voltUnderFilter2 = 0U;
        }
        else
        {
            voltOverFilter1 = 0U; 
            voltOverFilter2 = 0U;
            voltUnderFilter1 = 0U; 
            voltUnderFilter2 = 0U;
        }

        if (voltOverFilter2 >= gGridSafety.faultFilterCount2)
        {
            voltOverFilter2 = 0U;
            gSysFault.bit.gridOverVolt = 1U;
        }
        if (voltOverFilter1 >= gGridSafety.faultFilterCount1)
        {
            voltOverFilter1 = 0U;
            gSysFault.bit.gridOverVolt = 1U;
        }
        if (voltUnderFilter2 >= gGridSafety.faultFilterCount2)
        {
            voltUnderFilter2 = 0U;
            gSysFault.bit.gridUnderVolt = 1U;
        }
        if (voltUnderFilter1 >= gGridSafety.faultFilterCount1)
        {
            voltUnderFilter1 = 0U;
            gSysFault.bit.gridUnderVolt = 1U;
        }
    }
    else
    {
        if ((gridVoltRms > gGridSafety.voltUnderLevel1) && (gridVoltRms < gGridSafety.voltOverLevel1))
        {
            voltBackFilter++;
            if (voltBackFilter > gGridSafety.backFilterCount)
            {
                voltBackFilter = 0U;
                gSysFault.bit.gridOverVolt = 0U;
                gSysFault.bit.gridUnderVolt = 0U;
            }
        }
        else
        {
            voltBackFilter = 0U;
        }
    }
}

static void GridFreq_Protect(void)
{
    static Uint16 freqOverFilter1 = 0U;   /* 一级过频滤波 */
    static Uint16 freqOverFilter2 = 0U;   /* 二级过频滤波 */
    static Uint16 freqUnderFilter1 = 0U;  /* 一级欠频滤波 */
    static Uint16 freqUnderFilter2 = 0U;  /* 二级欠频滤波 */
    static Uint16 freqBackFilter = 0U;  
    float gridFreqHz = (float)gMachineData.ecapFreqCent * 0.01f;

    /* --- 频率：过/欠频 --- */
    if (gSysFault.bit.gridOverFreq == 0U && gSysFault.bit.gridUnderFreq == 0U)
    {
        if (gridFreqHz > gGridSafety.freqOverLevel2)           /* 二级过频 */
        {
            freqOverFilter2++;
            freqOverFilter1 = 0U; 
            freqUnderFilter1 = 0U; 
            freqUnderFilter2 = 0U;
        }
        else if (gridFreqHz < gGridSafety.freqUnderLevel2)     /* 二级欠频 */
        {
            freqUnderFilter2++;
            freqOverFilter1 = 0U; 
            freqOverFilter2 = 0U; 
            freqUnderFilter1 = 0U;
        }
        else if (gridFreqHz > gGridSafety.freqOverLevel1)      /* 一级过频 */
        {
            freqOverFilter1++;
            freqOverFilter2 = 0U; 
            freqUnderFilter1 = 0U; 
            freqUnderFilter2 = 0U;
        }
        else if (gridFreqHz < gGridSafety.freqUnderLevel1)     /* 一级欠频 */
        {
            freqUnderFilter1++;
            freqOverFilter1 = 0U; 
            freqOverFilter2 = 0U; 
            freqUnderFilter2 = 0U;
        }
        else
        {
            freqOverFilter1 = 0U; 
            freqOverFilter2 = 0U;
            freqUnderFilter1 = 0U; 
            freqUnderFilter2 = 0U;
        }

        if (freqOverFilter2 >= gGridSafety.faultFilterCount2)
        {
            freqOverFilter2 = 0U;
            gSysFault.bit.gridOverFreq = 1U;
        }
        if (freqOverFilter1 >= gGridSafety.faultFilterCount1)
        {
            freqOverFilter1 = 0U;
            gSysFault.bit.gridOverFreq = 1U;
        }
        if (freqUnderFilter2 >= gGridSafety.faultFilterCount2)
        {
            freqUnderFilter2 = 0U;
            gSysFault.bit.gridUnderFreq = 1U;
        }
        if (freqUnderFilter1 >= gGridSafety.faultFilterCount1)
        {
            freqUnderFilter1 = 0U;
            gSysFault.bit.gridUnderFreq = 1U;
        }
    }
    else
    {
        if ((gridFreqHz > gGridSafety.freqUnderLevel1) && (gridFreqHz < gGridSafety.freqOverLevel1))
        {
            freqBackFilter++;
            if (freqBackFilter > gGridSafety.backFilterCount)
            {
                freqBackFilter = 0U;
                gSysFault.bit.gridOverFreq = 0U;
                gSysFault.bit.gridUnderFreq = 0U;
            }
        }
        else
        {
            freqBackFilter = 0U;
        }
    }
}


/* 10 分钟平均电压窗：每 30 秒采一个 RMS，20 槽环形平均（原 vgrid_20_buf 机制）。 */
static void GridVolt_10minWindow(void)
{
    GridHalfMinCount++;
    if (GridHalfMinCount >= GRID_10MIN_HALF_MIN_COUNT)
    {
        Uint16 i;
        float sum = 0.0f;

        GridHalfMinCount = 0U;
        GridVoltageRmsBuf[GridVoltageRmsBufIdx] = gMachineData.realRms.gridVoltage;
        GridVoltageRmsBufIdx++;
        if (GridVoltageRmsBufIdx >= GRID_10MIN_SLOT_COUNT)
        {
            GridVoltageRmsBufIdx = 0U;
        }

        for (i = 0U; i < GRID_10MIN_SLOT_COUNT; i++)
        {
            sum += GridVoltageRmsBuf[i];
        }
        GridVoltageRmsAvg10Min = sum / (float)GRID_10MIN_SLOT_COUNT;
    }
}


/* GFCI 自检时序（原 GFCICheck 自检部分）：状态机进 CHECK 时置 selfTestActive=1，
 * 本函数在过零节奏下推进：静态检测 → 注入 50mA → 注入检测 → 断开注入。 */
static void Gfci_SelfTest(void)
{
    if (gGfciData.selfTestActive == 0U)
    {
        return;
    }

    gGfciData.selfTestIndex++;

    /* GFCI 静态漏电流检测（原 GFCIDeviceCheck1）：未注入时漏电流本底长期过大判硬件故障。 */
    if (gGfciData.selfTestIndex <= GFCI_SELFTEST_STATIC_END)
    {
        if (gMachineData.realRms.gfciCurrent > GFCI_35MA_A)
        {
            gGfciData.deviceFilter1++;
            if (gGfciData.deviceFilter1 >= GFCI_DEV_FILTER)
            {
                gSysFault.bit.gfciDeviceFault = 1U;
                gGfciData.deviceFilter1 = 0U;
            }
        }
        else
        {
            gGfciData.deviceFilter1 = 0U;
        }
    }
    else if (gGfciData.selfTestIndex == GFCI_SELFTEST_INJECT_START)
    {
        GFCI_CHECK_ON();
    }
    /* GFCI 注入漏电流检测（原 GFCIDeviceCheck2）：注入 50mA 后测量值应落在合理区间，否则硬件故障。 */
    else if (gGfciData.selfTestIndex >= GFCI_SELFTEST_INJECT_CHECK)
    {
        if ((gMachineData.realRms.gfciCurrent > GFCI_85MA_A) ||
            (gMachineData.realRms.gfciCurrent < GFCI_15MA_A))
        {
            gGfciData.deviceFilter2++;
            if (gGfciData.deviceFilter2 >= GFCI_DEV_FILTER)
            {
                gSysFault.bit.gfciDeviceFault = 1U;
                gGfciData.deviceFilter2 = 0U;
            }
        }
        else
        {
            gGfciData.deviceFilter2 = 0U;
        }
    }

    if (gGfciData.selfTestIndex >= GFCI_SELFTEST_DONE)
    {
        GFCI_CHECK_OFF();
        gGfciData.selfTestActive = 0U;
        gGfciData.selfTestIndex = 0U;
        gGfciData.checkDelay = 0U;   /* 自检完成，开始保护静默期 */
    }
}

/* GFCI 运行保护（原 GFCIProtect）：差分跳变检测 + 多级反时限 30/60/150/300mA。
 * 差分跳变区分"缓慢漂移"与"突变"：突变时锁定基准，后续相对基准追踪。 */
static void Gfci_Protect(void)
{
    float deltaGfi;
    float differenceGfiAvg = 0.0f;
    float differenceGfiRms = 0.0f;
    float gfciRms = gMachineData.realRms.gfciCurrent;
    float gfciAvg = gMachineData.realAvg.gfciCurrent;

    /* 存储三个周期的漏电流值（原 gfi_buf / gfi_ave_buf） */
    gGfciData.rmsBuf[0] = gGfciData.rmsBuf[1];
    gGfciData.rmsBuf[1] = gGfciData.rmsBuf[2];
    gGfciData.rmsBuf[2] = gfciRms;

    gGfciData.avgBuf[0] = gGfciData.avgBuf[1];
    gGfciData.avgBuf[1] = gGfciData.avgBuf[2];
    gGfciData.avgBuf[2] = gfciAvg;

    if (gSysData.state == SYS_STATE_NORMAL)
    {
        if (gGfciData.checkDelay < GFCI_PROTECT_DELAY_CYCLES)
        {
            gGfciData.checkDelay++;
        }

        //自检保护静默期
        else if (gGfciData.checkDelay >= GFCI_PROTECT_DELAY_CYCLES)
        {
            deltaGfi = gGfciData.avgBuf[2] - gGfciData.avgBuf[0];
            deltaGfi = deltaGfi >= 0.0f ? deltaGfi : -deltaGfi ;

            /* 绝对大漏电流（300mA 档，未修正 250mA 档） */
            if (gfciRms > GFCI_280MA_A)
            {
                gGfciData.filter300ma++;
                if (gGfciData.filter300ma >= GFCI_FILTER_300MA)
                {
                    gSysFault.bit.gfciFault = 1U;
                    gGfciData.filter300ma = 0U;
                }
            }
            else
            {
                gGfciData.filter300ma = 0U;
            }

            /* 差分跳变取值：缓慢变化 or 突变基准追踪 */
            if ((gGfciData.breakFlag == 0U) && (deltaGfi > GFCI_DELTA_MIN_A))
            {
                differenceGfiAvg = (gGfciData.avgBuf[2] > gGfciData.avgBuf[0]) ? 
                    (gGfciData.avgBuf[2] - gGfciData.avgBuf[0]) : 
                    (gGfciData.avgBuf[0] - gGfciData.avgBuf[2]) ;

                differenceGfiRms = (gGfciData.rmsBuf[2] > gGfciData.rmsBuf[0]) ? 
                    (gGfciData.rmsBuf[2] - gGfciData.rmsBuf[0]) : 
                    (gGfciData.rmsBuf[0] - gGfciData.rmsBuf[2]) ;
            }
            else if (gGfciData.breakFlag == 1U)
            {
                differenceGfiAvg = (gGfciData.avgBuf[2] > gGfciData.deltaBaseAvg) ? 
                    (gGfciData.avgBuf[2] - gGfciData.deltaBaseAvg) : 
                    (gGfciData.deltaBaseAvg - gGfciData.avgBuf[2]) ;

                differenceGfiRms = (gGfciData.rmsBuf[2] > gGfciData.deltaBaseRms) ? 
                    (gGfciData.rmsBuf[2] - gGfciData.deltaBaseRms) : 
                    (gGfciData.deltaBaseRms - gGfciData.rmsBuf[2]) ;
            }

            /* 多级反时限：150mA 立即，60mA/30mA 连续 8 周期 */
            if (differenceGfiRms >= GFCI_120MA_A)
            {
                gGfciData.filter30ma = 0U;
                gGfciData.filter60ma = 0U;

                gGfciData.breakFlag = 0U;
                gGfciData.breakSwFlag = 0U;

                gGfciData.deltaBaseAvg = 0.0f;
                gGfciData.deltaBaseRms = 0.0f;
                
                gSysFault.bit.gfciFault = 1U;
            }
            else if (differenceGfiAvg >= GFCI_48MA_A)
            {
                gGfciData.filter30ma = 0U;
                gGfciData.breakFlag = 1U;
                if ((gGfciData.breakSwFlag & 0x02U) == 0U)
                {
                    gGfciData.deltaBaseAvg = gGfciData.avgBuf[0];
                    gGfciData.deltaBaseRms = gGfciData.rmsBuf[0];
                    gGfciData.breakSwFlag = 2U;
                }
                
                gGfciData.filter60ma++;
                if (gGfciData.filter60ma >= GFCI_FILTER_60MA)
                {
                    gGfciData.filter60ma = 0U;
                    gGfciData.breakSwFlag = 0U;
                    gGfciData.breakFlag = 0U;
                    gGfciData.deltaBaseAvg = 0.0f;
                    gSysFault.bit.gfciFault = 1U;
                }
            }
            else if (differenceGfiAvg >= GFCI_24MA_A)
            {
                gGfciData.filter60ma = 0U;
                gGfciData.breakFlag = 1U;
                if ((gGfciData.breakSwFlag & 0x01U) == 0U)
                {
                    gGfciData.deltaBaseAvg = gGfciData.avgBuf[0];
                    gGfciData.deltaBaseRms = gGfciData.rmsBuf[0];
                    gGfciData.breakSwFlag = 1U;
                }
                gGfciData.filter30ma++;
                if (gGfciData.filter30ma >= GFCI_FILTER_30MA)
                {
                    gGfciData.filter30ma = 0U;
                    gGfciData.deltaBaseAvg = 0.0f;
                    gGfciData.breakSwFlag = 0U;
                    gSysFault.bit.gfciFault = 1U;
                    gGfciData.breakFlag = 0U;
                }
            }
            else
            {
                gGfciData.filter30ma = 0U;
                gGfciData.filter60ma = 0U;
                gGfciData.noBreakCnt++;
                if (gGfciData.noBreakCnt >= GFCI_NOBREAK_CNT)
                {
                    gGfciData.noBreakCnt = 0U;
                    gGfciData.breakFlag = 0U;
                    gGfciData.breakSwFlag = 0U;
                    gGfciData.deltaBaseAvg = 0.0f;
                    gGfciData.deltaBaseRms = 0.0f;
                }
            }
        }
    }
    
    else
    {
        gGfciData.checkDelay = 0U;
    }

    /* 恢复：GFCI 故障置位后，漏电流持续低于 50mA 若干周期才清除 */
    if (gSysFault.bit.gfciFault != 0U)
    {
        if (gfciRms < GFCI_50MA_A)
        {
            gGfciData.backFilter++;
            if (gGfciData.backFilter >= GFCI_BACK_FILTER)
            {
                gGfciData.backFilter = 0U;
                gSysFault.bit.gfciFault = 0U;
            }
        }
        else
        {
            gGfciData.backFilter = 0U;
        }
    }
}

void Task_AC_Monitor(void)
{
    /* 过零正常 → 清电网丢失故障位（丢失检测由状态机的过零看门狗完成） */
    gSysFault.bit.noUtility = 0U;

    /* 10 分钟平均电压窗更新 */
    GridVolt_10minWindow();

    /* 电网过/欠压、过/欠频判定 */
    GridVolt_Protect();
    GridFreq_Protect();
    /* DCI 检测 + 直流注入补偿 + 超标保护 */

    DCI_Inject();
    DCI_Protect();

    /* GFCI 自检 + 运行保护 */
    Gfci_SelfTest();
    Gfci_Protect();
}
