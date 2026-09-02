#include "F28x_Project.h"
#include "bsp.h"
#include "variable.h"
//DMA
//起点(一般是ADC结果寄存器) --- 搬运 ---> 终点(我们指定的Buffer)

#define DMA_FAST_CYCLE_MIN_BURSTS  250U
#define DMA_FAST_HALT_WAIT_LIMIT   1000U

/* DMA状态只供本文件的ISR和后台处理函数使用。 */
typedef struct
{
    Uint16 completedBuffer;
    Uint16 completedBursts;
    Uint16 ready;
    Uint16 overrun;
    Uint32 sequence;
    Uint32 completedBlockCount;
} DMA_BlockState;


/* Each ping-pong half has its own linker section so large arrays never overlap. */
//单独为这些buffer定义段的地硬件址
#pragma DATA_SECTION(ADC_FastRawBuffer0, "adcFastDmaBuffer0")
#pragma DATA_SECTION(ADC_FastRawBuffer1, "adcFastDmaBuffer1")
#pragma DATA_SECTION(ADC_PvCurrentRawBuffer0, "adcPvCurrentDmaBuffer0")
#pragma DATA_SECTION(ADC_PvCurrentRawBuffer1, "adcPvCurrentDmaBuffer1")
#pragma DATA_SECTION(ADC_PvVoltageRawBuffer0, "adcPvVoltageDmaBuffer0")
#pragma DATA_SECTION(ADC_PvVoltageRawBuffer1, "adcPvVoltageDmaBuffer1")
#pragma DATA_SECTION(ADC_IsolationRawBuffer0, "adcIsolationDmaBuffer0")
#pragma DATA_SECTION(ADC_IsolationRawBuffer1, "adcIsolationDmaBuffer1")
#pragma DATA_SECTION(ADC_TemperatureRawBuffer0, "adcTemperatureDmaBuffer0")
#pragma DATA_SECTION(ADC_TemperatureRawBuffer1, "adcTemperatureDmaBuffer1")

//为了乒乓传输,所以定义了两个buffer,我建议不要太在意名字,因为是硬件设计不够完美,没有硬件上把快环,慢环分开
//快环,正巧全在ADCA内,不需要拆分了
volatile ADC_FastRawFrame ADC_FastRawBuffer0[ADC_FAST_BLOCK_MAX_BURSTS];
volatile ADC_FastRawFrame ADC_FastRawBuffer1[ADC_FAST_BLOCK_MAX_BURSTS];

//中环,一个在ADCB,一个在ADCD,不得不分开了
volatile ADC_PvCurrentRawFrame ADC_PvCurrentRawBuffer0[ADC_PV_BLOCK_BURSTS];
volatile ADC_PvCurrentRawFrame ADC_PvCurrentRawBuffer1[ADC_PV_BLOCK_BURSTS];

volatile ADC_PvVoltageRawFrame ADC_PvVoltageRawBuffer0[ADC_PV_BLOCK_BURSTS];
volatile ADC_PvVoltageRawFrame ADC_PvVoltageRawBuffer1[ADC_PV_BLOCK_BURSTS];

//慢环,一个在ADCC,一个在ADCD,不得不分开了
volatile ADC_IsolationRawFrame ADC_IsolationRawBuffer0[ADC_SLOW_BLOCK_BURSTS];
volatile ADC_IsolationRawFrame ADC_IsolationRawBuffer1[ADC_SLOW_BLOCK_BURSTS];

volatile ADC_TemperatureRawFrame ADC_TemperatureRawBuffer0[ADC_SLOW_BLOCK_BURSTS];
volatile ADC_TemperatureRawFrame ADC_TemperatureRawBuffer1[ADC_SLOW_BLOCK_BURSTS];

//五路DMA的完成状态由dma.c内部独占管理
static volatile DMA_BlockState ADC_FastDmaState = {0};
static volatile DMA_BlockState ADC_PvCurrentDmaState = {0};
static volatile DMA_BlockState ADC_PvVoltageDmaState = {0};
static volatile DMA_BlockState ADC_IsolationDmaState = {0};
static volatile DMA_BlockState ADC_TemperatureDmaState = {0};

//决定了到底使用哪一个buffer,也就是乒乓传输的选择
static volatile Uint16 ADC_FastDmaActiveBuffer = 0U;
static volatile Uint16 ADC_FastDmaFrameCount = 0U;
static volatile Uint16 ADC_FastDmaCyclePrimed = 0U;
static Uint16 ADC_PvCurrentDmaActiveBuffer = 0U;
static Uint16 ADC_PvVoltageDmaActiveBuffer = 0U;
static Uint16 ADC_IsolationDmaActiveBuffer = 0U;
static Uint16 ADC_TemperatureDmaActiveBuffer = 0U;

//一定要记住,各速度环路的数据buffer,状态结构体和buffer选择都已经定义好了




//一些是初始化用到的函数========================================================================================================================
static void DMA_Config_CHx
(
    volatile struct CH_REGS *channel,//选择DMA通道
    Uint16 channelNumber,           //触发源
    volatile Uint16 *source,        //起点地址,也就是ADC结果寄存器
    volatile Uint16 *destination,   //终点地址,也就是对应的buffer
    Uint16 wordsPerBurst,           //一个burst由几个result组成?
    Uint16 burstsPerBlock           //一个buffer由几个burst组成?
)
{
    channel->MODE.all = 0U;//全部复位
    channel->MODE.bit.PERINTSEL = channelNumber;
    channel->MODE.bit.PERINTE = 1U;//使能中断
    channel->MODE.bit.ONESHOT = 0U;//失能单次模式,允许连续响应多次触发

    /* Stop after each block so the ISR can select the other ping-pong half. */
    channel->MODE.bit.CONTINUOUS = 0U;//失能连续模式,搬完一个块就停，等CPU重启
    channel->MODE.bit.OVRINTE = 0U;//溢出不触发中断
    channel->MODE.bit.DATASIZE = 0U;//十六位数据
    channel->MODE.bit.CHINTMODE = 1U;//块传输完才中断,也就是buffer填满才中断
    channel->MODE.bit.CHINTE = 1U;//使能通道中断（允许向CPU发PIE中断）
    //以上的配置是非常一般的配置
  
    channel->SRC_BEG_ADDR_SHADOW = (Uint32)source;
    channel->SRC_ADDR_SHADOW = (Uint32)source;//当前起点地址,一般也确实是保持一致即可
    channel->DST_BEG_ADDR_SHADOW = (Uint32)destination;
    channel->DST_ADDR_SHADOW = (Uint32)destination;//当前终点地址,一般也确实是保持一致即可

    /* DMA count registers store the requested quantity minus one. */
    channel->BURST_SIZE.all = wordsPerBurst - 1U;//注意这里要减一,因为默认有一个data
    channel->SRC_BURST_STEP = 1;//按顺序依次传输SOC1,SOC2....
    channel->DST_BURST_STEP = 1;
    channel->TRANSFER_SIZE = burstsPerBlock - 1U;//注意这里要减一,因为默认有一个burst

    /* Return from the last ADCRESULT in one burst to RESULT0 for the next. */
    //重点!回跳配置,因为终点地址一直加一没毛病,但是起点地址有且只有ADC结果寄存器
    //Base + (N-1) - (N-1) = Base
    channel->SRC_TRANSFER_STEP = -((int16)wordsPerBurst - 1);
    channel->DST_TRANSFER_STEP = 1;

    channel->SRC_WRAP_SIZE = 0xFFFFU;//不回绕
    channel->SRC_WRAP_STEP = 0;
    channel->DST_WRAP_SIZE = 0xFFFFU;
    channel->DST_WRAP_STEP = 0;

    channel->CONTROL.bit.PERINTCLR = 1U;// 清除外设中断挂起标志（清掉ADC的旧触发信号）
    channel->CONTROL.bit.ERRCLR = 1U;// 清除错误标志（确保启动时无历史故障）
}

void DMA_Config(void)
{
    ADC_FastDmaActiveBuffer = 0U;
    ADC_FastDmaFrameCount = 0U;
    ADC_FastDmaCyclePrimed = 0U;

    EALLOW;

    /* Reset the DMA once before configuring any channel. */
    DmaRegs.DMACTRL.bit.HARDRESET = 1U;
    __asm(" NOP");
    DmaRegs.DEBUGCTRL.bit.FREE = 1U;
    DmaRegs.PRIORITYCTRL1.bit.CH1PRIORITY = 1U;

    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = DMA_ADCAINT2;
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = DMA_ADCBINT2;
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH3 = DMA_ADCDINT1;
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH4 = DMA_ADCDINT2;
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH5 = DMA_ADCCINT2;

    DMA_Config_CHx
    (
        &DmaRegs.CH1, 
        1U,
        (volatile Uint16 *)&AdcaResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_FastRawBuffer0,
        6U, 
        ADC_FAST_BLOCK_MAX_BURSTS
    );

    DMA_Config_CHx
    (
        &DmaRegs.CH2, 
        2U,
        (volatile Uint16 *)&AdcbResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_PvCurrentRawBuffer0,
        2U, 
        ADC_PV_BLOCK_BURSTS
    );

    DMA_Config_CHx
    (
        &DmaRegs.CH3, 
        3U,
        (volatile Uint16 *)&AdcdResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_PvVoltageRawBuffer0,
        2U, 
        ADC_PV_BLOCK_BURSTS
    );

    DMA_Config_CHx
    (
        &DmaRegs.CH4, 
        4U,
        (volatile Uint16 *)&AdcdResultRegs.ADCRESULT2,
        (volatile Uint16 *)ADC_IsolationRawBuffer0,
        2U, 
        ADC_SLOW_BLOCK_BURSTS
    );

    DMA_Config_CHx
    (
        &DmaRegs.CH5, 
        5U,
        (volatile Uint16 *)&AdccResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_TemperatureRawBuffer0,
        2U, 
        ADC_SLOW_BLOCK_BURSTS
    );

    PieVectTable.DMA_CH1_INT = &DMA_CH1_CPU_ISR;
    PieVectTable.DMA_CH2_INT = &DMA_CH2_CPU_ISR;
    PieVectTable.DMA_CH3_INT = &DMA_CH3_CPU_ISR;
    PieVectTable.DMA_CH4_INT = &DMA_CH4_CPU_ISR;
    PieVectTable.DMA_CH5_INT = &DMA_CH5_CPU_ISR;
    
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1U;
    PieCtrlRegs.PIEIER7.bit.INTx2 = 1U;
    PieCtrlRegs.PIEIER7.bit.INTx3 = 1U;
    PieCtrlRegs.PIEIER7.bit.INTx4 = 1U;
    PieCtrlRegs.PIEIER7.bit.INTx5 = 1U;
    IER |= M_INT7;

    DmaRegs.CH1.CONTROL.bit.RUN = 1U;
    DmaRegs.CH2.CONTROL.bit.RUN = 1U;
    DmaRegs.CH3.CONTROL.bit.RUN = 1U;
    DmaRegs.CH4.CONTROL.bit.RUN = 1U;
    DmaRegs.CH5.CONTROL.bit.RUN = 1U;

    EDIS;
}




//以下是中断用到的函数=========================================================================================================================

static void DMA_RecordCompletion(volatile DMA_BlockState *state,
                                 Uint16 completedBuffer,
                                 Uint16 completedBursts)
{
    /* A still-set ready flag means the foreground did not consume the prior block. */
    if(state->ready != 0U)//DMA数据丢包计数
    {
        state->overrun++;
    }

    state->completedBuffer = completedBuffer;//记录刚刚是哪一个buffer传输完成了
    state->completedBursts = completedBursts;
    state->sequence++;//流水号,主要看有没有掉包,这个比丢包严重,因为丢包好歹CPU知道有这回事,但是掉包CPU根本没发觉
    state->completedBlockCount++;//相当于心跳监控
    state->ready = 1U;
}

static void DMA_StartNextBlock(volatile struct CH_REGS *channel, volatile Uint16 *destination)
{
    channel->DST_BEG_ADDR_SHADOW = (Uint32)destination;//切换新的终点地址
    channel->DST_ADDR_SHADOW = (Uint32)destination;

    channel->CONTROL.bit.PERINTCLR = 1U;
    channel->CONTROL.bit.ERRCLR = 1U;
    channel->CONTROL.bit.HALT = 0U;
    channel->CONTROL.bit.RUN = 1U;//等待下次ADC中断,把影子寄存器写进去
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
    Uint16 completedBursts = ADC_FastDmaFrameCount;
    Uint16 haltedCleanly = DMA_HaltFastBlock();

    if((ADC_FastDmaCyclePrimed != 0U) &&
       (haltedCleanly != 0U) &&
       (completedBursts >= DMA_FAST_CYCLE_MIN_BURSTS) &&
       (completedBursts <= ADC_FAST_BLOCK_MAX_BURSTS))
    {
        DMA_RecordCompletion(&ADC_FastDmaState,
                             ADC_FastDmaActiveBuffer,
                             completedBursts);
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


__interrupt void DMA_CH1_CPU_ISR(void)//进入了DMA中断,就说明当前的buffer满了
{
    /* Reaching 450 frames means no valid grid boundary arrived in time. */
    ADC_FastDmaFrameCount = 0U;
    ADC_FastDmaCyclePrimed = 0U;
    gMachineData.ecapFreqCent = 0U;
    ADC_FastDmaActiveBuffer ^= 1U;//这里(指的是中断)才是决定更新采样buffer的地方
    //然后换挡,这段期间CPU可以取刚刚完成的buffer数据,处理数据和读取数据实现解耦
    DMA_StartNextBlock
    (
        &DmaRegs.CH1,
        (ADC_FastDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_FastRawBuffer0 :
        (volatile Uint16 *)ADC_FastRawBuffer1
    );
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

__interrupt void DMA_CH2_CPU_ISR(void)
{
    DMA_RecordCompletion(&ADC_PvCurrentDmaState,
                         ADC_PvCurrentDmaActiveBuffer,
                         ADC_PV_BLOCK_BURSTS);
    ADC_PvCurrentDmaActiveBuffer ^= 1U;
    DMA_StartNextBlock
    (
        &DmaRegs.CH2,
        (ADC_PvCurrentDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_PvCurrentRawBuffer0 :
        (volatile Uint16 *)ADC_PvCurrentRawBuffer1
    );
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

__interrupt void DMA_CH3_CPU_ISR(void)
{
    DMA_RecordCompletion(&ADC_PvVoltageDmaState,
                         ADC_PvVoltageDmaActiveBuffer,
                         ADC_PV_BLOCK_BURSTS);
    ADC_PvVoltageDmaActiveBuffer ^= 1U;
    DMA_StartNextBlock
    (
        &DmaRegs.CH3,
        (ADC_PvVoltageDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_PvVoltageRawBuffer0 :
        (volatile Uint16 *)ADC_PvVoltageRawBuffer1
    );
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

__interrupt void DMA_CH4_CPU_ISR(void)
{
    DMA_RecordCompletion(&ADC_IsolationDmaState,
                         ADC_IsolationDmaActiveBuffer,
                         ADC_SLOW_BLOCK_BURSTS);
    ADC_IsolationDmaActiveBuffer ^= 1U;
    DMA_StartNextBlock
    (
        &DmaRegs.CH4,
        (ADC_IsolationDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_IsolationRawBuffer0 :
        (volatile Uint16 *)ADC_IsolationRawBuffer1
    );
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

__interrupt void DMA_CH5_CPU_ISR(void)
{
    DMA_RecordCompletion(&ADC_TemperatureDmaState,
                         ADC_TemperatureDmaActiveBuffer,
                         ADC_SLOW_BLOCK_BURSTS);
    ADC_TemperatureDmaActiveBuffer ^= 1U;
    DMA_StartNextBlock
    (
        &DmaRegs.CH5,
        (ADC_TemperatureDmaActiveBuffer == 0U) ?
        (volatile Uint16 *)ADC_TemperatureRawBuffer0 :
        (volatile Uint16 *)ADC_TemperatureRawBuffer1
    );
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}




//以下是数据传输的函数=========================================================================================================================

static Uint16 DMA_ClaimBuffer(volatile DMA_BlockState *state,
                              Uint16 *completedBuffer,
                              Uint16 *completedBursts)//确认数据,更新状态
{
    Uint16 blockReady;

    /* Keep the ISR from changing ready/completedBuffer between the two reads. */
    DINT;//关中断????
    blockReady = state->ready;
    if(blockReady != 0U)
    {
        //注意,这一个函数不负责更新刚刚完成传输的buffer, 而且甚至不直接读取当前的已完成,因为它需要读取的是旧数据,管你当前是哪个buffer在工作
        *completedBuffer = state->completedBuffer;
        if(completedBursts != 0)
        {
            *completedBursts = state->completedBursts;
        }
        state->ready = 0U;
    }
    EINT;

    return blockReady;//这里显式告诉你是否准备好,隐式告诉你一个用哪个buffer
}

Uint16 DMA_ProcessBlocks
(
    ADC_UintData *rawInstant,
    ADC_UintData *rawAvg,
                                 ADC_FloatData *rawMeanSq,
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

    //以下就是轮询5个DMA通道的状态,看是否有数据更新

    if(DMA_ClaimBuffer(&ADC_FastDmaState,
                       &bufferIndex,
                       &fastBlockBursts) != 0U)//如果快环数据更新
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
            squareSum0 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].gridVoltage - cal->gridVoltage.offset;
            squareSum1 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].gfciCurrent - cal->gfciCurrent.offset;
            squareSum2 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].dcBusVoltage - cal->dcBusVoltage.offset;
            squareSum3 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].gridDcCurrent - cal->gridDcCurrent.offset;
            squareSum4 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].inverterVoltage - cal->inverterVoltage.offset;
            squareSum5 += centeredSample * centeredSample;
        }

        //保存本数据块最后一次采样和直流分量(avg)
        rawInstant->inductorCurrent = buffer[fastBlockBursts - 1U].inductorCurrent;
        rawInstant->gridVoltage = buffer[fastBlockBursts - 1U].gridVoltage;
        rawInstant->gfciCurrent = buffer[fastBlockBursts - 1U].gfciCurrent;
        rawInstant->dcBusVoltage = buffer[fastBlockBursts - 1U].dcBusVoltage;
        rawInstant->gridDcCurrent = buffer[fastBlockBursts - 1U].gridDcCurrent;
        rawInstant->inverterVoltage = buffer[fastBlockBursts - 1U].inverterVoltage;
        rawAvg->inductorCurrent = (Uint16)(sum0 / fastBlockBursts);
        rawAvg->gridVoltage = (Uint16)(sum1 / fastBlockBursts);
        rawAvg->gfciCurrent = (Uint16)(sum2 / fastBlockBursts);
        rawAvg->dcBusVoltage = (Uint16)(sum3 / fastBlockBursts);
        rawAvg->gridDcCurrent = (Uint16)(sum4 / fastBlockBursts);
        rawAvg->inverterVoltage = (Uint16)(sum5 / fastBlockBursts);

        //算直流等效值(rms)
        rawMeanSq->inductorCurrent = squareSum0 / (float)fastBlockBursts;
        rawMeanSq->gridVoltage = squareSum1 / (float)fastBlockBursts;
        rawMeanSq->gfciCurrent = squareSum2 / (float)fastBlockBursts;
        rawMeanSq->dcBusVoltage = squareSum3 / (float)fastBlockBursts;
        rawMeanSq->gridDcCurrent = squareSum4 / (float)fastBlockBursts;
        rawMeanSq->inverterVoltage = squareSum5 / (float)fastBlockBursts;
        updated |= DMA_UPDATE_FAST;
    }

    if(DMA_ClaimBuffer(&ADC_PvCurrentDmaState, &bufferIndex, 0) != 0U)
    {
        volatile ADC_PvCurrentRawFrame *buffer =
            (bufferIndex == 0U) ? ADC_PvCurrentRawBuffer0 : ADC_PvCurrentRawBuffer1;

        sum0 = 0UL;
        sum1 = 0UL;
        squareSum0 = 0.0f;
        squareSum1 = 0.0f;
        for(sampleIndex = 0U; sampleIndex < ADC_PV_BLOCK_BURSTS; sampleIndex++)
        {
            sum0 += buffer[sampleIndex].pv1Current;
            sum1 += buffer[sampleIndex].pv2Current;
            centeredSample = (float)buffer[sampleIndex].pv1Current - cal->pv1Current.offset;
            squareSum0 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].pv2Current - cal->pv2Current.offset;
            squareSum1 += centeredSample * centeredSample;
        }
        rawInstant->pv1Current = buffer[ADC_PV_BLOCK_BURSTS - 1U].pv1Current;
        rawInstant->pv2Current = buffer[ADC_PV_BLOCK_BURSTS - 1U].pv2Current;
        rawAvg->pv1Current = (Uint16)(sum0 / ADC_PV_BLOCK_BURSTS);
        rawAvg->pv2Current = (Uint16)(sum1 / ADC_PV_BLOCK_BURSTS);
        rawMeanSq->pv1Current = squareSum0 / (float)ADC_PV_BLOCK_BURSTS;
        rawMeanSq->pv2Current = squareSum1 / (float)ADC_PV_BLOCK_BURSTS;
        updated |= DMA_UPDATE_PV_CURRENT;
    }

    if(DMA_ClaimBuffer(&ADC_PvVoltageDmaState, &bufferIndex, 0) != 0U)
    {
        volatile ADC_PvVoltageRawFrame *buffer = (bufferIndex == 0U) ? ADC_PvVoltageRawBuffer0 : ADC_PvVoltageRawBuffer1;

        sum0 = 0UL;
        sum1 = 0UL;
        squareSum0 = 0.0f;
        squareSum1 = 0.0f;
        for(sampleIndex = 0U; sampleIndex < ADC_PV_BLOCK_BURSTS; sampleIndex++)
        {
            sum0 += buffer[sampleIndex].pv1Voltage;
            sum1 += buffer[sampleIndex].pv2Voltage;
            centeredSample = (float)buffer[sampleIndex].pv1Voltage - cal->pv1Voltage.offset;
            squareSum0 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].pv2Voltage - cal->pv2Voltage.offset;
            squareSum1 += centeredSample * centeredSample;
        }
        rawInstant->pv1Voltage = buffer[ADC_PV_BLOCK_BURSTS - 1U].pv1Voltage;
        rawInstant->pv2Voltage = buffer[ADC_PV_BLOCK_BURSTS - 1U].pv2Voltage;
        rawAvg->pv1Voltage = (Uint16)(sum0 / ADC_PV_BLOCK_BURSTS);
        rawAvg->pv2Voltage = (Uint16)(sum1 / ADC_PV_BLOCK_BURSTS);
        rawMeanSq->pv1Voltage = squareSum0 / (float)ADC_PV_BLOCK_BURSTS;
        rawMeanSq->pv2Voltage = squareSum1 / (float)ADC_PV_BLOCK_BURSTS;
        updated |= DMA_UPDATE_PV_VOLTAGE;
    }

    if(DMA_ClaimBuffer(&ADC_IsolationDmaState, &bufferIndex, 0) != 0U)
    {
        volatile ADC_IsolationRawFrame *buffer = (bufferIndex == 0U) ? ADC_IsolationRawBuffer0 : ADC_IsolationRawBuffer1;

        sum0 = 0UL;
        sum1 = 0UL;
        squareSum0 = 0.0f;
        squareSum1 = 0.0f;
        for(sampleIndex = 0U; sampleIndex < ADC_SLOW_BLOCK_BURSTS; sampleIndex++)
        {
            sum0 += buffer[sampleIndex].pv1Isolation;
            sum1 += buffer[sampleIndex].pv2Isolation;
            centeredSample = (float)buffer[sampleIndex].pv1Isolation - cal->pv1Isolation.offset;
            squareSum0 += centeredSample * centeredSample;
            centeredSample = (float)buffer[sampleIndex].pv2Isolation - cal->pv2Isolation.offset;
            squareSum1 += centeredSample * centeredSample;
        }
        rawInstant->pv1Isolation = buffer[ADC_SLOW_BLOCK_BURSTS - 1U].pv1Isolation;
        rawInstant->pv2Isolation = buffer[ADC_SLOW_BLOCK_BURSTS - 1U].pv2Isolation;
        rawAvg->pv1Isolation = (Uint16)(sum0 / ADC_SLOW_BLOCK_BURSTS);
        rawAvg->pv2Isolation = (Uint16)(sum1 / ADC_SLOW_BLOCK_BURSTS);
        rawMeanSq->pv1Isolation = squareSum0 / (float)ADC_SLOW_BLOCK_BURSTS;
        rawMeanSq->pv2Isolation = squareSum1 / (float)ADC_SLOW_BLOCK_BURSTS;
        updated |= DMA_UPDATE_ISOLATION;
    }

    if(DMA_ClaimBuffer(&ADC_TemperatureDmaState, &bufferIndex, 0) != 0U)
    {
        volatile ADC_TemperatureRawFrame *buffer = (bufferIndex == 0U) ? ADC_TemperatureRawBuffer0 : ADC_TemperatureRawBuffer1;

        sum0 = 0UL;
        sum1 = 0UL;
        for(sampleIndex = 0U; sampleIndex < ADC_SLOW_BLOCK_BURSTS; sampleIndex++)
        {
            sum0 += buffer[sampleIndex].inverterTemperature;
            sum1 += buffer[sampleIndex].boostTemperature;
        }
        rawInstant->inverterTemperature = buffer[ADC_SLOW_BLOCK_BURSTS - 1U].inverterTemperature;
        rawInstant->boostTemperature = buffer[ADC_SLOW_BLOCK_BURSTS - 1U].boostTemperature;
        rawAvg->inverterTemperature = (Uint16)(sum0 / ADC_SLOW_BLOCK_BURSTS);
        rawAvg->boostTemperature = (Uint16)(sum1 / ADC_SLOW_BLOCK_BURSTS);
        updated |= DMA_UPDATE_TEMP;
    }

    return updated;
}
