#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"
#include "variable.h"
#include "invert.h"

/* 4.7kΩ NTC 分压电阻 + 分段温度曲线常量。 */
#define ADC_TEMP_DIVIDER_RESIST       4700.0f
// 0.1℃ 转换为 ℃ 的系数
#define ADC_DECI_C_TO_C               0.1f
// 计算功率因数时的最小视在功率阈值，防止除零
#define GRID_POWER_FACTOR_MIN_VA      1.0f

static float ADC_RawToAvg(Uint16 raw, const volatile AdcCalParam *calParam, float Drift);
static float ADC_RawToRms(float meanSq, const volatile AdcCalParam *calParam);
static float ADC_ConvertTemp(Uint16 raw);

//码值平均值转换为真实平均值, 减去bias和drift
static float ADC_RawToAvg(Uint16 raw, const volatile AdcCalParam *calParam, float Drift)
{
    return ((float)raw - (calParam->bias) - Drift) * (calParam->gain);
}

//码值均方值转换为真实均方值
static float ADC_RawToRms(float meanSq, const volatile AdcCalParam *calParam)
{
    float gain = calParam->gain;
    /* 保护 __sqrt：防止浮点舍入产生的小负值导致开方异常。 */
    if (meanSq <= 0.0f)
    {
        return 0.0f;
    }
    if (gain < 0.0f)
    {
        gain = -gain;
    }
    //使用TMU内联函数__sqrt
    return __sqrt(meanSq) * gain;
}

//机器开机零漂校准函数
//主要校准电感电流, 电网电压, 漏电流, 直流分量
static void Measu_CheckAdcDrift(const ADC_UintData *rawAvg)
{
    //如果正在校准,忽视
    if (gAdcDrift.adjInit == 0U)
    {
        return;
    }
    //采样32个数据进行校准
    if (gAdcDrift.checkCnt != 0U && gAdcDrift.checkCnt <= 32U)
    {
        gAdcDrift.sum.inductCurr    += (float)rawAvg->inductCurr;
        gAdcDrift.sum.gridVolt      += (float)rawAvg->gridVolt;
        gAdcDrift.sum.gfciCurr      += (float)rawAvg->gfciCurr;
        gAdcDrift.sum.gridDcCurr    += (float)rawAvg->gridDcCurr;
    }
    //计数器自增
    if (gAdcDrift.checkCnt <= 32U)
    {
        gAdcDrift.checkCnt++;
    }
    //如果采满了32个数据
    else
    {
        //就记录下来这些数据, 正式发布零漂校准值
        gAdcDrift.drift.inductCurr  = gAdcDrift.sum.inductCurr  / 32.0f;
        gAdcDrift.drift.gridVolt    = gAdcDrift.sum.gridVolt    / 32.0f;
        gAdcDrift.drift.gfciCurr    = gAdcDrift.sum.gfciCurr    / 32.0f;
        gAdcDrift.drift.gridDcCurr  = gAdcDrift.sum.gridDcCurr  / 32.0f;
        //清除中间参数, 清除校准标志位, 之后少用了
        gAdcDrift.sum.inductCurr    = 0.0f;
        gAdcDrift.sum.gridVolt      = 0.0f;
        gAdcDrift.sum.gfciCurr      = 0.0f;
        gAdcDrift.sum.gridDcCurr    = 0.0f;
        gAdcDrift.checkCnt          = 0U;
        gAdcDrift.adjInit           = 0U;
    }
}

//热敏电阻非线性,需要分段线性化
static float ADC_ConvertTemp(Uint16 raw)
{
    float rawFloat = (float)raw;
    float resist;
    float tempDeciC;
    /* 满量程码值会让分压公式出现除零奇异，先钳位到满量程-1。 */
    if (rawFloat >= ADC_FULL_SCALE)
    {
        rawFloat = ADC_FULL_SCALE - 1.0f;
    }
    // 根据分压公式计算 NTC 电阻值, 也就是说满量程下, 等效的电阻是4700ohm
    resist = rawFloat * ADC_TEMP_DIVIDER_RESIST / (ADC_FULL_SCALE - rawFloat);
    // 根据电阻值落在不同区间，用线性公式计算 0.1℃ 为单位的温度
    tempDeciC = 
            //-1.8℃
            (resist > 50000.0f) ? (71.0f -  resist / 557.0f) :
            //18.4℃
            (resist > 19000.0f) ? (306.0f - resist / 157.0f) :
            //38.6℃
            (resist > 7700.0f)  ? (529.0f - resist / 54.0f)  :
            //58.55℃
            (resist > 3450.0f)  ? (758.0f - resist / 20.0f)  :
            //>58.55℃
                                  (937.0f - resist / 10.0f)  ;

    return tempDeciC * ADC_DECI_C_TO_C;
}

//调度层：消费 DMA 块，判断本周期是否该执行测量。
//返回 0 = 本周期无需执行
//注意这个函数的形参是指针, 那没事了
static Uint16 Measu_TrySched
(
    ADC_UintData        *rawAvg,
    ADC_FloatData       *rawMeanSq,
    DMA_GridPowerRaw    *gridPowerRaw
)
{
    static Uint16 validMask = 0U;
    Uint16 updateMask;
    // 获取已经完成好累加的原始码值, 和它对应的block
    updateMask = DMA_ProcessBlocks(rawAvg, rawMeanSq, gridPowerRaw);
    /* 本周期无 DMA 块完成的话, 就直接返回0: 无block完成*/
    if (updateMask == 0U)
    {
        return 0U;   
    }

    /* 开机凑齐：所有 DMA 块都至少到过一次，才允许发布（防半成品）。 */
    validMask |= updateMask;
    if (validMask != DMA_UPDATE_ALL)
    {
        return 0U;
    }
    return updateMask;
}

/* 功能层：执行测量换算并发布（零漂 → AVG/RMS → 功率 → gMachineData）。 */
static void Measu_Compute
(
    const ADC_UintData *rawAvg,
    const ADC_FloatData *rawMeanSq,
    const DMA_GridPowerRaw *gridPowerRaw,
    Uint16 updateMask
)
{
    //先创建几个临时值,最后都会传入MachineData
    ADC_FloatData   realAvg = {0};
    ADC_FloatData   realRms = {0};
    PowerData       powerData = {0};

    /* ADC 运行时零漂校准（开机/重连时采 32 组码值平均）。 */
    Measu_CheckAdcDrift(rawAvg);

    /* 原始码值 → 物理值（平均值）：4 个交流/测零通道减运行时零漂，其余只减出厂校准。*/
    //realAvg.gridVolt =      ADC_RawToAvg(rawAvg->gridVolt,    &gAdcCal.gridVolt,     gAdcDrift.drift.gridVolt);
    //realAvg.inductCurr =    ADC_RawToAvg(rawAvg->inductCurr,  &gAdcCal.inductCurr,   gAdcDrift.drift.inductCurr);
    realAvg.gfciCurr =      ADC_RawToAvg(rawAvg->gfciCurr,    &gAdcCal.gfciCurr,     gAdcDrift.drift.gfciCurr);
    realAvg.gridDcCurr =    ADC_RawToAvg(rawAvg->gridDcCurr,  &gAdcCal.gridDcCurr,   gAdcDrift.drift.gridDcCurr);
    realAvg.dcBusVolt =     ADC_RawToAvg(rawAvg->dcBusVolt,   &gAdcCal.dcBusVolt,    0.0f);
    //realAvg.invertVolt =    ADC_RawToAvg(rawAvg->invertVolt,  &gAdcCal.invertVolt,   0.0f);
    realAvg.pv1Curr =       ADC_RawToAvg(rawAvg->pv1Curr,     &gAdcCal.pv1Curr,      0.0f);
    realAvg.pv2Curr =       ADC_RawToAvg(rawAvg->pv2Curr,     &gAdcCal.pv2Curr,      0.0f);
    realAvg.pv1Volt =       ADC_RawToAvg(rawAvg->pv1Volt,     &gAdcCal.pv1Volt,      0.0f);
    realAvg.pv2Volt =       ADC_RawToAvg(rawAvg->pv2Volt,     &gAdcCal.pv2Volt,      0.0f);
    realAvg.pv1Insul =      ADC_RawToAvg(rawAvg->pv1Insul,    &gAdcCal.pv1Insul,     0.0f);
    realAvg.pv2Insul =      ADC_RawToAvg(rawAvg->pv2Insul,    &gAdcCal.pv2Insul,     0.0f);
    realAvg.invertTemp =    ADC_ConvertTemp(rawAvg->invertTemp);
    realAvg.boostTemp =     ADC_ConvertTemp(rawAvg->boostTemp);

    /* 均方值 → RMS。 */
    realRms.gridVolt =      ADC_RawToRms(rawMeanSq->gridVolt,    &gAdcCal.gridVolt);
    realRms.inductCurr =    ADC_RawToRms(rawMeanSq->inductCurr,  &gAdcCal.inductCurr);
    realRms.gfciCurr =      ADC_RawToRms(rawMeanSq->gfciCurr,    &gAdcCal.gfciCurr);
    //realRms.dcBusVolt =     ADC_RawToRms(rawMeanSq->dcBusVolt,   &gAdcCal.dcBusVolt);
    //realRms.gridDcCurr =    ADC_RawToRms(rawMeanSq->gridDcCurr,  &gAdcCal.gridDcCurr);
    realRms.invertVolt =    ADC_RawToRms(rawMeanSq->invertVolt,  &gAdcCal.invertVolt);
    //realRms.pv1Curr =       ADC_RawToRms(rawMeanSq->pv1Curr,     &gAdcCal.pv1Curr);
    //realRms.pv2Curr =       ADC_RawToRms(rawMeanSq->pv2Curr,     &gAdcCal.pv2Curr);
    //realRms.pv1Volt =       ADC_RawToRms(rawMeanSq->pv1Volt,     &gAdcCal.pv1Volt);
    //realRms.pv2Volt =       ADC_RawToRms(rawMeanSq->pv2Volt,     &gAdcCal.pv2Volt);
    //realRms.pv1Insul =      ADC_RawToRms(rawMeanSq->pv1Insul,    &gAdcCal.pv1Insul);
    //realRms.pv2Insul =      ADC_RawToRms(rawMeanSq->pv2Insul,    &gAdcCal.pv2Insul);

    //保存以前功率数据, 防止当前某些数据无效
    powerData = gMachineData.powerData;

    //PV数据没更新的话,pass
    if ((updateMask & DMA_UPDATE_PV) != 0U)
    {
        //反之计算PV功率, 很简单
        powerData.pv1Power = realAvg.pv1Volt * realAvg.pv1Curr;
        powerData.pv2Power = realAvg.pv2Volt * realAvg.pv2Curr;
    }
    //快环数据没更新的话,pass
    if ((updateMask & DMA_UPDATE_FAST) != 0U)
    {
        //如果更新了,就要计算这些数据和功率
        //增益值提前算
        float powerScale = gAdcCal.gridVolt.gain * gAdcCal.inductCurr.gain;
        //有功功率和无功功率实际上已经累加过了, 只需转为实际值即可
        powerData.gridActivePower = gridPowerRaw->activeMean * powerScale;
        powerData.gridReactivePower = gridPowerRaw->reactiveMean * powerScale;
        //视在功率用RMS直接算
        powerData.gridApparPower = realRms.gridVolt * realRms.inductCurr;
        //只有S > 1VA,计算才有意义, 否则在MCU里可能直接约掉了
        if (powerData.gridApparPower > GRID_POWER_FACTOR_MIN_VA)
        {
            //PF = P / S
            powerData.gridPF = Invert_Clamp(powerData.gridActivePower / powerData.gridApparPower, -1.0f, 1.0f);
        }
        else
        {
            powerData.gridPF = 0.0f;
        }
    }

    /* 发布校准后的平均值、RMS 和功率，供控制/任务消费。 */
    gMachineData.realAvg = realAvg;
    gMachineData.realRms = realRms;
    gMachineData.powerData = powerData;
    //测量心跳
    gMachineData.measuSeq++;
}

/* 任务入口：先调度（判断是否该干活），再功能（换算 + 发布）。 */
void Task_Measu(void)
{
    // 静态变量临时保存DMA 平均原始值
    static  ADC_UintData     rawAvg = {0};
    // 静态变量临时保存 DMA 均方值
    static  ADC_FloatData    rawMeanSq = {0};
    // 静态变量临时保存电网功率原始量
    static  DMA_GridPowerRaw gridPowerRaw = {0};
    // DMA 更新掩码
    Uint16  updateMask;

    /* 调度层：消费 DMA 块，判断本周期是否该执行。 */
    //注意: 临时量的指针传入函数
    updateMask = Measu_TrySched(&rawAvg, &rawMeanSq, &gridPowerRaw);
    //DMA_ProcessBlocks会把rawMeanSq, gridPowerRaw里面填满数据的,如果有的话
    //如果没有数据更新
    if (updateMask == 0U)
    {
        //说明没有浮点计算任务, 直接跳过计算任务
        return;
    }
    //如果有浮点计算任务
    /* 功能层：换算 + 功率 + 发布。 */
    //各种原始码值数据指针被传入, 而计算任务最终会把数据发布到MachineData, 所以没有输出
    Measu_Compute(&rawAvg, &rawMeanSq, &gridPowerRaw, updateMask);
}
