#include "F28x_Project.h"
#include "bsp.h"
#include "variable.h"

//DMA基本思想就是起点(一般是ADC结果寄存器) --- 搬运 ---> 终点(我们指定的Buffer)

//由于快环比较特殊, 它的数据会受到电网频率的影响, 所以它的burst长度不固定
//但是再不固定, 你至少要有250个burst, 也就是12.5ms
#define DMA_FAST_CYCLE_MIN_BURSTS  250U
//当然也不能超过50ms
#define DMA_FAST_HALT_WAIT_LIMIT   1000U

//DMA状态只供本文件的ISR和后台处理函数使用
typedef struct
{
    //刚完成的 buffer 索引：0 或 1
    Uint16 doneBuffer;
    //刚完成的 block 里实际有多少个 burst
    Uint16 doneBursts;
    //是否有完成的 block 等待前台处理
    Uint16 ready;
    //前台还没处理完，又有新 block 完成，导致覆盖
    Uint16 overrun;
    //完成的 block 流水号，用于检测丢块
    Uint32 sequence;
    //完成的 block 总数，心跳监控
    Uint32 doneBlockCount;
} DMA_BlockState;

//存放四个DMA当前状态
static volatile DMA_BlockState ADC_FastDmaState = {0};
static volatile DMA_BlockState ADC_PvDmaState = {0};
static volatile DMA_BlockState ADC_IsoDmaState = {0};
static volatile DMA_BlockState ADC_TempDmaState = {0};

//单独为这些buffer定义段的地硬件址, 不要放在.ebss
//具体段可在CMD文件找到
#pragma DATA_SECTION(ADC_FastRawBuffer0,"adcFastDmaBuffer0")
#pragma DATA_SECTION(ADC_FastRawBuffer1,"adcFastDmaBuffer1")
#pragma DATA_SECTION(ADC_PvRawBuffer0,  "adcPvDmaBuffer0")
#pragma DATA_SECTION(ADC_PvRawBuffer1,  "adcPvDmaBuffer1")
#pragma DATA_SECTION(ADC_IsoRawBuffer0, "adcIsoDmaBuffer0")
#pragma DATA_SECTION(ADC_IsoRawBuffer1, "adcIsoDmaBuffer1")
#pragma DATA_SECTION(ADC_TempRawBuffer0,"adcTempDmaBuffer0")
#pragma DATA_SECTION(ADC_TempRawBuffer1,"adcTempDmaBuffer1")

//为了乒乓传输,所以定义了两个buffer
//快环数据buffer, 多为电网数据
volatile ADC_FastRawFrame   ADC_FastRawBuffer0[ADC_FAST_BLOCK_MAX_BURSTS];
volatile ADC_FastRawFrame   ADC_FastRawBuffer1[ADC_FAST_BLOCK_MAX_BURSTS];
//PV数据buffer
volatile ADC_PvRawFrame     ADC_PvRawBuffer0[ADC_PV_BLOCK_BURSTS];
volatile ADC_PvRawFrame     ADC_PvRawBuffer1[ADC_PV_BLOCK_BURSTS];
//绝缘阻抗数据buffer
volatile ADC_IsoRawFrame    ADC_IsoRawBuffer0[ADC_ISO_BLOCK_BURSTS];
volatile ADC_IsoRawFrame    ADC_IsoRawBuffer1[ADC_ISO_BLOCK_BURSTS];
//温度数据buffer
volatile ADC_TempRawFrame   ADC_TempRawBuffer0[ADC_TEMP_BLOCK_BURSTS];
volatile ADC_TempRawFrame   ADC_TempRawBuffer1[ADC_TEMP_BLOCK_BURSTS];

//记录当前共记录了多少次burst
static volatile Uint16  ADC_FastDmaFrameCount = 0U;
//记录当前是否是第一次完整记录过一次电网数据
static volatile Uint16  ADC_FastDmaCyclePrimed = 0U;

//记录当前正在使用哪一个buffer?
static volatile Uint16  ADC_FastDmaActiveBuffer = 0U;
static          Uint16  ADC_PvDmaActiveBuffer = 0U;
static          Uint16  ADC_IsoDmaActiveBuffer = 0U;
static          Uint16  ADC_TempDmaActiveBuffer = 0U;


static void DMA_Config_CHx
(
    //选择要配置的DMA通道
    volatile struct CH_REGS *channel,
    //对应DMA的触发源
    Uint16 channelNumber,        
    //一个burst由几个result组成? 比如快环就有6个
    Uint16 wordsPerBurst,   
    //一个buffer由几个burst组成? 比如快环就不确定
    Uint16 burstsPerBlock,     
    //起点地址,也就是ADC结果寄存器
    volatile Uint16 *source,     
    //终点地址,也就是对应的buffer
    volatile Uint16 *destination          
)
{
    //先清除要配置的DMA寄存器
    channel->MODE.all = 0U;
    
    //十六位数据
    channel->MODE.bit.DATASIZE = 0U;    
    //选择对应DMA的触发源, 比如快环就是ADCA2
    //但是数据手册明确说了这些是遗留位，应该设置为通道号。
    //真正的选择在DMACHSRCSEL1.bit.CHx
    channel->MODE.bit.PERINTSEL = channelNumber;
    
    //使能外部信号触发DMA
    channel->MODE.bit.PERINTE = 1U;

    //关闭单次模式,允许连续搬运burst
    channel->MODE.bit.ONESHOT = 0U;
    //失能连续模式,不允许连续搬运block, 得先处理中断
    channel->MODE.bit.CONTINUOUS = 0U;

    // 关闭溢出中断，不因为 DMA 溢出而触发中断
    channel->MODE.bit.OVRINTE = 0U;
    //块传输完才中断,也就是buffer填满才中断
    channel->MODE.bit.CHINTMODE = 1U;
    //使能通道中断（允许向CPU发PIE中断）
    channel->MODE.bit.CHINTE = 1U;
  
    //设置源起始地址
    channel->SRC_BEG_ADDR_SHADOW = (Uint32)source;
    channel->SRC_ADDR_SHADOW = (Uint32)source;
    //设置目的起始地址
    channel->DST_BEG_ADDR_SHADOW = (Uint32)destination;
    channel->DST_ADDR_SHADOW = (Uint32)destination;
    
    //告诉对应DMA, 一个burst有几个result
    channel->BURST_SIZE.all = wordsPerBurst - 1U;
    //一个 burst 内每搬 1 个字，源地址 +1, 也就是SOC0->SOC1
    channel->SRC_BURST_STEP = 1;
    //一个 burst 内每搬 1 个字，目的地址 +1 也就是buffer[0]->buffer[1]
    channel->DST_BURST_STEP = 1;

    //告诉对应DMA, 一个block有几个burst
    channel->TRANSFER_SIZE = burstsPerBlock - 1U;
    //重点!回跳配置,终点地址一直加一没事,但是起点地址有且只有ADC结果寄存器. 因此:SOC0+(N-1)+(1-N)=SOC0
    channel->SRC_TRANSFER_STEP = 1 - (int16)wordsPerBurst;
    //burst 结束后目的地址继续 +1，保证 buffer 连续填充
    channel->DST_TRANSFER_STEP = 1;

    //起点和终点地址都设置不回绕, 因为正常运行不可能会到达边界
    channel->SRC_WRAP_SIZE = 0xFFFFU;
    channel->SRC_WRAP_STEP = 0;
    channel->DST_WRAP_SIZE = 0xFFFFU;
    channel->DST_WRAP_STEP = 0;

    // 清除外设中断挂起标志（清掉ADC的旧触发信号）
    channel->CONTROL.bit.PERINTCLR = 1U;
    // 清除错误标志（确保启动时无历史故障）
    channel->CONTROL.bit.ERRCLR = 1U;
}

void DMA_Config(void)
{
    // 快环当前写入 buffer 索引清零，从 Buffer0 开始
    ADC_FastDmaActiveBuffer = 0U;
    // 快环当前周期已采集帧数清零
    ADC_FastDmaFrameCount = 0U;
    // 快环周期预热标志清零，表示还没有完成一个完整周期
    ADC_FastDmaCyclePrimed = 0U;
    // PV 通道当前写入 buffer 索引清零
    ADC_PvDmaActiveBuffer = 0U;
    // ISO道当前写入 buffer 索引清零
    ADC_IsoDmaActiveBuffer = 0U;
    // TEMP 通道当前写入 buffer 索引清零
    ADC_TempDmaActiveBuffer = 0U;

    EALLOW;

    //对 DMA 模块做一次硬复位，所有通道回到默认状态
    DmaRegs.DMACTRL.bit.HARDRESET = 1U;
    __asm(" NOP");

    // 调试模式下 DMA 自由运行，不被仿真断点暂停
    DmaRegs.DEBUGCTRL.bit.FREE = 1U;

    //DMA四个通道的触发源选择, 参见adc.c
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = DMA_ADCAINT2;
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = DMA_ADCDINT1;
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH3 = DMA_ADCBINT1;
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH4 = DMA_ADCCINT1;
    // 设置 CH1 为高优先级通道, 因为CH1是都是快环数据
    DmaRegs.PRIORITYCTRL1.bit.CH1PRIORITY = 1U;
    
    //配置CH1, 一个burst 6个数据, 一个block有450个burst
    DMA_Config_CHx 
    (
        &DmaRegs.CH1, 1U, 6U, ADC_FAST_BLOCK_MAX_BURSTS,
        (volatile Uint16 *)&AdcaResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_FastRawBuffer0 
    );
    //配置CH2, 一个burst 4个数据, 一个block有160个burst
    DMA_Config_CHx
    (
        &DmaRegs.CH2, 2U, 4U, ADC_PV_BLOCK_BURSTS,
        (volatile Uint16 *)&AdcdResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_PvRawBuffer0
    );
    //配置CH3, 一个burst 2个数据, 一个block有10个burst
    DMA_Config_CHx
    (
        &DmaRegs.CH3, 3U, 2U, ADC_ISO_BLOCK_BURSTS,
        (volatile Uint16 *)&AdcbResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_IsoRawBuffer0
    );
    //配置CH4, 一个burst 2个数据, 一个block有20个burst
    DMA_Config_CHx
    (
        &DmaRegs.CH4, 4U, 2U, ADC_TEMP_BLOCK_BURSTS,
        (volatile Uint16 *)&AdccResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_TempRawBuffer0
    );

    //注册中断向量表到PIE
    PieVectTable.DMA_CH1_INT = &DMA_CH1_CPU_ISR;
    PieVectTable.DMA_CH2_INT = &DMA_CH2_CPU_ISR;
    PieVectTable.DMA_CH3_INT = &DMA_CH3_CPU_ISR;
    PieVectTable.DMA_CH4_INT = &DMA_CH4_CPU_ISR;
    //使能 PIE 组 7 的 INTx，对应 DMA CHx
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1U;
    PieCtrlRegs.PIEIER7.bit.INTx2 = 1U;
    PieCtrlRegs.PIEIER7.bit.INTx3 = 1U;
    PieCtrlRegs.PIEIER7.bit.INTx4 = 1U;
    IER |= M_INT7;

    //正式启动DMA，开始等待外部触发!
    DmaRegs.CH1.CONTROL.bit.RUN = 1U;
    DmaRegs.CH2.CONTROL.bit.RUN = 1U;
    DmaRegs.CH3.CONTROL.bit.RUN = 1U;
    DmaRegs.CH4.CONTROL.bit.RUN = 1U;

    EDIS;
}

static void DMA_RecordCompletion(volatile DMA_BlockState *state, Uint16 doneBuffer, Uint16 doneBursts)
{
    /* A still-set ready flag means the foreground did not consume the prior block. */
    if(state->ready != 0U)//DMA数据丢包计数
    {
        state->overrun++;
    }

    state->doneBuffer = doneBuffer;//记录刚刚是哪一个buffer传输完成了
    state->doneBursts = doneBursts;
    state->sequence++;//流水号,主要看有没有掉包,这个比丢包严重,因为丢包好歹CPU知道有这回事,但是掉包CPU根本没发觉
    state->doneBlockCount++;//相当于心跳监控
    state->ready = 1U;
}

//切换乒乓 buffer 并重启通道
static void DMA_StartNextBlock(volatile struct CH_REGS *channel, volatile Uint16 *destination)
{
    //切换新的终点地址
    channel->DST_BEG_ADDR_SHADOW = (Uint32)destination;
    channel->DST_ADDR_SHADOW = (Uint32)destination;

    // 清除外设中断挂起标志，避免旧触发残留
    channel->CONTROL.bit.PERINTCLR = 1U;
    // 清除错误标志
    channel->CONTROL.bit.ERRCLR = 1U;
     //取消暂停, 这个主要是应对快环的, 快环会主动暂停DMA
    channel->CONTROL.bit.HALT = 0U;
    //然后等待下次ADC中断
    channel->CONTROL.bit.RUN = 1U;
}

/* Stop after the current six-word burst so the completed prefix is stable. */
static Uint16 DMA_HaltFastBlock(void)
{
    Uint16 waitCount = DMA_FAST_HALT_WAIT_LIMIT;

    DmaRegs.CH1.CONTROL.bit.HALT = 1U;
    while((DmaRegs.CH1.CONTROL.bit.RUNSTS != 0U) && (waitCount > 0U))
    {
        waitCount--;
    }

    return (DmaRegs.CH1.CONTROL.bit.RUNSTS == 0U) ? 1U : 0U;
}

/* An early boundary needs a channel reset so transfer count returns to zero. */
static void DMA_RestartFastBlock(volatile Uint16 *destination)
{
    DmaRegs.CH1.CONTROL.bit.SOFTRESET = 1U;
    __asm(" NOP");

    DMA_Config_CHx
    (
        &DmaRegs.CH1,
        1U,
        (volatile Uint16 *)&AdcaResultRegs.ADCRESULT0,
        destination,
        6U,
        ADC_FAST_BLOCK_MAX_BURSTS
    );
    DmaRegs.CH1.CONTROL.bit.HALT = 0U;
    DmaRegs.CH1.CONTROL.bit.RUN = 1U;
}

/* Count one ADCA EOC5 event. DMA CH1 moves the corresponding six-result frame
 * independently; this notification only tracks the current cycle length. */
void DMA_NotifyFastFrameEoc(void)
{
    if(ADC_FastDmaFrameCount < ADC_FAST_BLOCK_MAX_BURSTS)
    {
        ADC_FastDmaFrameCount++;
    }
}

/* Freeze the current grid-cycle prefix and immediately start the other half. */
void DMA_GridCycleBoundary(void)
{
    Uint16 doneBursts = ADC_FastDmaFrameCount;
    Uint16 haltedCleanly = DMA_HaltFastBlock();

    if((ADC_FastDmaCyclePrimed != 0U) &&
       (haltedCleanly != 0U) &&
       (doneBursts >= DMA_FAST_CYCLE_MIN_BURSTS) &&
       (doneBursts <= ADC_FAST_BLOCK_MAX_BURSTS))
    {
        DMA_RecordCompletion(&ADC_FastDmaState,
                             ADC_FastDmaActiveBuffer,
                             doneBursts);
    }

    ADC_FastDmaActiveBuffer ^= 1U;
    ADC_FastDmaFrameCount = 0U;
    ADC_FastDmaCyclePrimed = 1U;

    DMA_RestartFastBlock
    (
        (ADC_FastDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_FastRawBuffer0 :
        (volatile Uint16 *)ADC_FastRawBuffer1
    );
}

/*进入了DMA中断,就说明当前的buffer满了, 准确来说是block到达对应数量了*/

//DMA CH1 中断函数, 正常来说, 我们都是不会进入的, 因为被提前截断了
__interrupt void DMA_CH1_CPU_ISR(void)
{
    // 帧数清零，本次采集无效
    ADC_FastDmaFrameCount = 0U;
    // 取消预热，等待下一次采集
    ADC_FastDmaCyclePrimed = 0U;
    // 电网频率相关变量清零，表示没有捕获到有效边界
    gMachineData.ecapFreqCent = 0U;
    //但还是老老实实切换buffer
    ADC_FastDmaActiveBuffer ^= 1U;
    //然后换挡,这段期间CPU可以取刚刚完成的buffer数据,处理数据和读取数据实现解耦
    DMA_StartNextBlock
    (
        &DmaRegs.CH1,
        (ADC_FastDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_FastRawBuffer0 :
        (volatile Uint16 *)ADC_FastRawBuffer1
    );
    //告诉PIE, "我已经处理好了这个中断"
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

__interrupt void DMA_CH2_CPU_ISR(void)
{
    DMA_RecordCompletion(&ADC_PvDmaState,
                         ADC_PvDmaActiveBuffer,
                         ADC_PV_BLOCK_BURSTS);
    ADC_PvDmaActiveBuffer ^= 1U;
    DMA_StartNextBlock
    (
        &DmaRegs.CH2,
        (ADC_PvDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_PvRawBuffer0 :
        (volatile Uint16 *)ADC_PvRawBuffer1
    );
    //告诉PIE, "我已经处理好了这个中断"
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

__interrupt void DMA_CH3_CPU_ISR(void)
{
    DMA_RecordCompletion(&ADC_IsoDmaState,
                         ADC_IsoDmaActiveBuffer,
                         ADC_ISO_BLOCK_BURSTS);
    ADC_IsoDmaActiveBuffer ^= 1U;
    DMA_StartNextBlock
    (
        &DmaRegs.CH3,
        (ADC_IsoDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_IsoRawBuffer0 :
        (volatile Uint16 *)ADC_IsoRawBuffer1
    );
    //告诉PIE, "我已经处理好了这个中断"
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

__interrupt void DMA_CH4_CPU_ISR(void)
{
    DMA_RecordCompletion(&ADC_TempDmaState,
                         ADC_TempDmaActiveBuffer,
                         ADC_TEMP_BLOCK_BURSTS);
    ADC_TempDmaActiveBuffer ^= 1U;
    DMA_StartNextBlock
    (
        &DmaRegs.CH4,
        (ADC_TempDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_TempRawBuffer0 :
        (volatile Uint16 *)ADC_TempRawBuffer1
    );
    //告诉PIE, "我已经处理好了这个中断"
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}




//以下是数据传输的函数=========================================================================================================================

static Uint16 DMA_ClaimBuffer(volatile DMA_BlockState *state,
                              Uint16 *doneBuffer,
                              Uint16 *doneBursts)//确认数据,更新状态
{
    Uint16 blockReady;

    /* Keep the ISR from changing ready/doneBuffer between the two reads. */
    DINT;//关中断????
    blockReady = state->ready;
    if(blockReady != 0U)
    {
        //注意,这一个函数不负责更新刚刚完成传输的buffer, 而且甚至不直接读取当前的已完成,因为它需要读取的是旧数据,管你当前是哪个buffer在工作
        *doneBuffer = state->doneBuffer;
        if(doneBursts != 0)
        {
            *doneBursts = state->doneBursts;
        }
        state->ready = 0U;
    }
    EINT;

    return blockReady;//这里显式告诉你是否准备好,隐式告诉你一个用哪个buffer
}

Uint16 DMA_ProcessBlocks
(
    ADC_UintData *rawAvg,
    ADC_FloatData *rawMeanSq,
    float *gridVoltCurrentMeanRaw,
    const ADC_Calibrate *cal
)
{
    Uint16 bufferIndex;
    Uint16 fastBlockBursts;
    Uint16 sampleIndex;
    Uint16 updated = 0U;

    Uint32 sum0;
    Uint32 sum1;
    Uint32 sum2;
    Uint32 sum3;
    Uint32 sum4;
    Uint32 sum5;

    float squareSum0;
    float squareSum1;
    float squareSum2;
    float squareSum3;
    float squareSum4;
    float squareSum5;

    float centeredSample;
    float centeredGridVoltage;
    float centeredInductorCurrent;
    float gridVoltCurrentSum;

    //以下就是轮询5个DMA通道的状态,看是否有数据更新

    if(DMA_ClaimBuffer(&ADC_FastDmaState, &bufferIndex, &fastBlockBursts) != 0U)//如果快环数据更新
    {
        //这里的bufferIndex就是刚刚问DMA"你有哪个buffer是准备好的?" 这是DMA"回答"的结果
        volatile ADC_FastRawFrame *buffer = (bufferIndex == 0U) ? ADC_FastRawBuffer0 : ADC_FastRawBuffer1;

        sum0 = 0UL;
        sum1 = 0UL;
        sum2 = 0UL;
        sum3 = 0UL;
        sum4 = 0UL;
        sum5 = 0UL;
        squareSum0 = 0.0f;
        squareSum1 = 0.0f;
        squareSum2 = 0.0f;
        squareSum3 = 0.0f;
        squareSum4 = 0.0f;
        squareSum5 = 0.0f;
        gridVoltCurrentSum = 0.0f;
        for(sampleIndex = 0U; sampleIndex < fastBlockBursts; sampleIndex++)
        {
            //注意,buffer里可全是原始数据
            sum0 += buffer[sampleIndex].inductorCurrent;
            sum1 += buffer[sampleIndex].gridVoltage;
            sum2 += buffer[sampleIndex].gfciCurrent;
            sum3 += buffer[sampleIndex].dcBusVoltage;
            sum4 += buffer[sampleIndex].gridDcCurrent;
            sum5 += buffer[sampleIndex].inverterVoltage;

            //偏置
            centeredSample = (float)buffer[sampleIndex].inductorCurrent - cal->inductorCurrent.offset;
            centeredInductorCurrent = centeredSample;
            squareSum0 += centeredSample * centeredSample;

            centeredSample = (float)buffer[sampleIndex].gridVoltage - cal->gridVoltage.offset;
            squareSum1 += centeredSample * centeredSample;

            centeredSample = (float)buffer[sampleIndex].gfciCurrent - cal->gfciCurrent.offset;
            centeredGridVoltage = centeredSample;
            squareSum2 += centeredSample * centeredSample;

            centeredSample = (float)buffer[sampleIndex].dcBusVoltage - cal->dcBusVoltage.offset;
            squareSum3 += centeredSample * centeredSample;

            centeredSample = (float)buffer[sampleIndex].gridDcCurrent - cal->gridDcCurrent.offset;
            squareSum4 += centeredSample * centeredSample;

            centeredSample = (float)buffer[sampleIndex].inverterVoltage - cal->inverterVoltage.offset;
            squareSum5 += centeredSample * centeredSample;

            /* Keep the voltage/current samples paired so active power is
             * calculated from average(v*i), rather than Vavg*Iavg. */
            gridVoltCurrentSum += centeredGridVoltage * centeredInductorCurrent;
        }

        //保存本数据块最后一次采样和直流分量(avg)
        rawAvg->inductorCurrent =   (Uint16)(sum0 / fastBlockBursts);
        rawAvg->gridVoltage =       (Uint16)(sum1 / fastBlockBursts);
        rawAvg->gfciCurrent =       (Uint16)(sum2 / fastBlockBursts);
        rawAvg->dcBusVoltage =      (Uint16)(sum3 / fastBlockBursts);
        rawAvg->gridDcCurrent =     (Uint16)(sum4 / fastBlockBursts);
        rawAvg->inverterVoltage =   (Uint16)(sum5 / fastBlockBursts);

        //算直流等效值(rms)
        rawMeanSq->inductorCurrent =    squareSum0 / (float)fastBlockBursts;
        rawMeanSq->gridVoltage =        squareSum1 / (float)fastBlockBursts;
        rawMeanSq->gfciCurrent =        squareSum2 / (float)fastBlockBursts;
        rawMeanSq->dcBusVoltage =       squareSum3 / (float)fastBlockBursts;
        rawMeanSq->gridDcCurrent =      squareSum4 / (float)fastBlockBursts;
        rawMeanSq->inverterVoltage =    squareSum5 / (float)fastBlockBursts;
        *gridVoltCurrentMeanRaw =       gridVoltCurrentSum / (float)fastBlockBursts;
        updated |= DMA_UPDATE_FAST;
    }

    if(DMA_ClaimBuffer(&ADC_PvDmaState, &bufferIndex, 0) != 0U)
    {
        volatile ADC_PvRawFrame *buffer =
            (bufferIndex == 0U) ? ADC_PvRawBuffer0 : ADC_PvRawBuffer1;

        sum0 = 0UL;
        sum1 = 0UL;
        sum2 = 0UL;
        sum3 = 0UL;
        squareSum0 = 0.0f;
        squareSum1 = 0.0f;
        squareSum2 = 0.0f;
        squareSum3 = 0.0f;
        for(sampleIndex = 0U; sampleIndex < ADC_PV_BLOCK_BURSTS; sampleIndex++)
        {
            sum0 += buffer[sampleIndex].pv1Voltage;
            sum1 += buffer[sampleIndex].pv1Current;
            sum2 += buffer[sampleIndex].pv2Voltage;
            sum3 += buffer[sampleIndex].pv2Current;
            centeredSample = (float)buffer[sampleIndex].pv1Voltage - cal->pv1Voltage.offset;
            squareSum0 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].pv1Current - cal->pv1Current.offset;
            squareSum1 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].pv2Voltage - cal->pv2Voltage.offset;
            squareSum2 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].pv2Current - cal->pv2Current.offset;
            squareSum3 += centeredSample * centeredSample;
        }
        rawAvg->pv1Voltage = (Uint16)(sum0 / ADC_PV_BLOCK_BURSTS);
        rawAvg->pv1Current = (Uint16)(sum1 / ADC_PV_BLOCK_BURSTS);
        rawAvg->pv2Voltage = (Uint16)(sum2 / ADC_PV_BLOCK_BURSTS);
        rawAvg->pv2Current = (Uint16)(sum3 / ADC_PV_BLOCK_BURSTS);
        rawMeanSq->pv1Voltage = squareSum0 / (float)ADC_PV_BLOCK_BURSTS;
        rawMeanSq->pv1Current = squareSum1 / (float)ADC_PV_BLOCK_BURSTS;
        rawMeanSq->pv2Voltage = squareSum2 / (float)ADC_PV_BLOCK_BURSTS;
        rawMeanSq->pv2Current = squareSum3 / (float)ADC_PV_BLOCK_BURSTS;
        updated |= DMA_UPDATE_PV;
    }

    if(DMA_ClaimBuffer(&ADC_IsoDmaState, &bufferIndex, 0) != 0U)
    {
        volatile ADC_IsoRawFrame *buffer = (bufferIndex == 0U) ? ADC_IsoRawBuffer0 : ADC_IsoRawBuffer1;

        sum0 = 0UL;
        sum1 = 0UL;
        squareSum0 = 0.0f;
        squareSum1 = 0.0f;
        for(sampleIndex = 0U; sampleIndex < ADC_ISO_BLOCK_BURSTS; sampleIndex++)
        {
            sum0 += buffer[sampleIndex].pv1Isolation;
            sum1 += buffer[sampleIndex].pv2Isolation;
            centeredSample = (float)buffer[sampleIndex].pv1Isolation - cal->pv1Isolation.offset;
            squareSum0 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].pv2Isolation - cal->pv2Isolation.offset;
            squareSum1 += centeredSample * centeredSample;
        }
        rawAvg->pv1Isolation = (Uint16)(sum0 / ADC_ISO_BLOCK_BURSTS);
        rawAvg->pv2Isolation = (Uint16)(sum1 / ADC_ISO_BLOCK_BURSTS);
        rawMeanSq->pv1Isolation = squareSum0 / (float)ADC_ISO_BLOCK_BURSTS;
        rawMeanSq->pv2Isolation = squareSum1 / (float)ADC_ISO_BLOCK_BURSTS;
        updated |= DMA_UPDATE_ISOLATION;
    }

    if(DMA_ClaimBuffer(&ADC_TempDmaState, &bufferIndex, 0) != 0U)
    {
        volatile ADC_TempRawFrame *buffer = (bufferIndex == 0U) ? ADC_TempRawBuffer0 : ADC_TempRawBuffer1;

        sum0 = 0UL;
        sum1 = 0UL;
        for(sampleIndex = 0U; sampleIndex < ADC_TEMP_BLOCK_BURSTS; sampleIndex++)
        {
            sum0 += buffer[sampleIndex].inverterTemperature;
            sum1 += buffer[sampleIndex].boostTemperature;
        }
        rawAvg->inverterTemperature = (Uint16)(sum0 / ADC_TEMP_BLOCK_BURSTS);
        rawAvg->boostTemperature = (Uint16)(sum1 / ADC_TEMP_BLOCK_BURSTS);
        updated |= DMA_UPDATE_TEMP;
    }

    return updated;
}
