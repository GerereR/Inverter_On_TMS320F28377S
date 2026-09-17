#include "F28x_Project.h"
#include "bsp.h"
#include "variable.h"

//DMA基本思想就是起点(一般是ADC结果寄存器) --- 搬运 ---> 终点(我们指定的Buf)

//由于快环比较特殊, 它的数据会受到电网频率的影响, 所以它的burst长度不固定
//但是再不固定, 你至少要有250个burst, 也就是12.5ms
#define DMA_FAST_CYCLE_MIN_BURST  250U
//DMA快环冻结超时时间
#define DMA_FAST_HALT_WAIT_LIM   1000U

//DMA状态只供本文件的ISR和后台处理函数使用
typedef struct
{
    //刚完成的 buffer 索引：0 或 1
    Uint16 doneBuf;
    //刚完成的 block 里实际有多少个 burst
    Uint16 doneBurst;
    //是否有完成的 block 等待前台处理
    Uint16 ready;
    //前台还没处理完，又有新 block 完成，导致覆盖
    Uint16 overrun;
} DMA_BlockState;

//存放四个DMA当前状态
static volatile DMA_BlockState ADC_FastDmaState = {0};
static volatile DMA_BlockState ADC_PvDmaState = {0};
static volatile DMA_BlockState ADC_InsulDmaState = {0};
static volatile DMA_BlockState ADC_TempDmaState = {0};

//单独为这些buffer定义段的地硬件址, 不要放在.ebss
//具体段可在CMD文件找到
#pragma DATA_SECTION(ADC_FastRawBuf0,"adcFastDmaBuf0")
#pragma DATA_SECTION(ADC_FastRawBuf1,"adcFastDmaBuf1")
#pragma DATA_SECTION(ADC_PvRawBuf0,  "adcPvDmaBuf0")
#pragma DATA_SECTION(ADC_PvRawBuf1,  "adcPvDmaBuf1")
#pragma DATA_SECTION(ADC_InsulRawBuf0, "adcInsulDmaBuf0")
#pragma DATA_SECTION(ADC_InsulRawBuf1, "adcInsulDmaBuf1")
#pragma DATA_SECTION(ADC_TempRawBuf0,"adcTempDmaBuf0")
#pragma DATA_SECTION(ADC_TempRawBuf1,"adcTempDmaBuf1")

//为了乒乓传输,所以定义了两个buffer
//快环数据buffer, 多为电网数据
volatile ADC_FastRawFrame   ADC_FastRawBuf0[ADC_FAST_BLOCK_MAX_BURST];
volatile ADC_FastRawFrame   ADC_FastRawBuf1[ADC_FAST_BLOCK_MAX_BURST];
//PV数据buffer
volatile ADC_PvRawFrame     ADC_PvRawBuf0[ADC_PV_BLOCK_BURST];
volatile ADC_PvRawFrame     ADC_PvRawBuf1[ADC_PV_BLOCK_BURST];
//绝缘阻抗数据buffer
volatile ADC_InsulRawFrame    ADC_InsulRawBuf0[ADC_INSUL_BLOCK_BURST];
volatile ADC_InsulRawFrame    ADC_InsulRawBuf1[ADC_INSUL_BLOCK_BURST];
//温度数据buffer
volatile ADC_TempRawFrame   ADC_TempRawBuf0[ADC_TEMP_BLOCK_BURST];
volatile ADC_TempRawFrame   ADC_TempRawBuf1[ADC_TEMP_BLOCK_BURST];

//记录当前共记录了多少次burst
static volatile Uint16  ADC_FastDmaFrameCnt = 0U;
//记录当前是否是第一次完整记录过一次电网数据
static volatile Uint16  ADC_FastDmaCyclePrimed = 0U;

//记录当前正在使用哪一个buffer?
static volatile Uint16  ADC_FastDmaActiveBuf = 0U;
static          Uint16  ADC_PvDmaActiveBuf = 0U;
static          Uint16  ADC_InsulDmaActiveBuf = 0U;
static          Uint16  ADC_TempDmaActiveBuf = 0U;


static void DMA_Config_CHx
(
    //选择要配置的DMA通道
    volatile struct CH_REGS *dmaCh,
    //对应DMA的触发源
    Uint16 dmaChNum,        
    //一个burst由几个result组成? 比如快环就有6个
    Uint16 wordsPerBurst,   
    //一个buffer由几个burst组成? 比如快环就不确定
    Uint16 burstPerBlock,     
    //起点地址,也就是ADC结果寄存器
    volatile Uint16 *source,     
    //终点地址,也就是对应的buffer
    volatile Uint16 *dest          
)
{
    //先清除要配置的DMA寄存器
    dmaCh->MODE.all = 0U;
    
    //十六位数据
    dmaCh->MODE.bit.DATASIZE = 0U;    
    //选择对应DMA的触发源, 比如快环就是ADCA2
    //但是数据手册明确说了这些是遗留位，应该设置为通道号。
    //真正的选择在DMACHSRCSEL1.bit.CHx
    dmaCh->MODE.bit.PERINTSEL = dmaChNum;
    
    //使能外部信号触发DMA
    dmaCh->MODE.bit.PERINTE = 1U;

    //关闭单次模式,允许连续搬运burst
    dmaCh->MODE.bit.ONESHOT = 0U;
    //失能连续模式,不允许连续搬运block, 得先处理中断
    dmaCh->MODE.bit.CONTINUOUS = 0U;

    // 关闭溢出中断，不因为 DMA 溢出而触发中断
    dmaCh->MODE.bit.OVRINTE = 0U;
    //块传输完才中断,也就是buffer填满才中断
    dmaCh->MODE.bit.CHINTMODE = 1U;
    //使能通道中断（允许向CPU发PIE中断）
    dmaCh->MODE.bit.CHINTE = 1U;
  
    //设置源起始地址
    dmaCh->SRC_BEG_ADDR_SHADOW = (Uint32)source;
    dmaCh->SRC_ADDR_SHADOW = (Uint32)source;
    //设置目的起始地址
    dmaCh->DST_BEG_ADDR_SHADOW = (Uint32)dest;
    dmaCh->DST_ADDR_SHADOW = (Uint32)dest;
    
    //告诉对应DMA, 一个burst有几个result
    dmaCh->BURST_SIZE.all = wordsPerBurst - 1U;
    //一个 burst 内每搬 1 个字，源地址 +1, 也就是SOC0->SOC1
    dmaCh->SRC_BURST_STEP = 1;
    //一个 burst 内每搬 1 个字，目的地址 +1 也就是buffer[0]->buffer[1]
    dmaCh->DST_BURST_STEP = 1;

    //告诉对应DMA, 一个block有几个burst
    dmaCh->TRANSFER_SIZE = burstPerBlock - 1U;
    //重点!回跳配置,终点地址一直加一没事,但是起点地址有且只有ADC结果寄存器. 因此:SOC0+(N-1)+(1-N)=SOC0
    dmaCh->SRC_TRANSFER_STEP = 1 - (int16)wordsPerBurst;
    //burst 结束后目的地址继续 +1，保证 buffer 连续填充
    dmaCh->DST_TRANSFER_STEP = 1;

    //起点和终点地址都设置不回绕, 因为正常运行不可能会到达边界
    dmaCh->SRC_WRAP_SIZE = 0xFFFFU;
    dmaCh->SRC_WRAP_STEP = 0;
    dmaCh->DST_WRAP_SIZE = 0xFFFFU;
    dmaCh->DST_WRAP_STEP = 0;

    // 清除外设中断挂起标志（清掉ADC的旧触发信号）
    dmaCh->CONTROL.bit.PERINTCLR = 1U;
    // 清除错误标志（确保启动时无历史故障）
    dmaCh->CONTROL.bit.ERRCLR = 1U;
}

void DMA_Config(void)
{
    // 快环当前写入 buffer 索引清零，从 Buf0 开始
    ADC_FastDmaActiveBuf = 0U;
    // 快环当前周期已采集帧数清零
    ADC_FastDmaFrameCnt = 0U;
    // 快环周期预热标志清零，表示还没有完成一个完整周期
    ADC_FastDmaCyclePrimed = 0U;
    // PV 通道当前写入 buffer 索引清零
    ADC_PvDmaActiveBuf = 0U;
    // ISO道当前写入 buffer 索引清零
    ADC_InsulDmaActiveBuf = 0U;
    // TEMP 通道当前写入 buffer 索引清零
    ADC_TempDmaActiveBuf = 0U;

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
        &DmaRegs.CH1, 1U, 6U, ADC_FAST_BLOCK_MAX_BURST,
        (volatile Uint16 *)&AdcaResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_FastRawBuf0 
    );
    //配置CH2, 一个burst 4个数据, 一个block有160个burst
    DMA_Config_CHx
    (
        &DmaRegs.CH2, 2U, 4U, ADC_PV_BLOCK_BURST,
        (volatile Uint16 *)&AdcdResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_PvRawBuf0
    );
    //配置CH3, 一个burst 2个数据, 一个block有10个burst
    DMA_Config_CHx
    (
        &DmaRegs.CH3, 3U, 2U, ADC_INSUL_BLOCK_BURST,
        (volatile Uint16 *)&AdcbResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_InsulRawBuf0
    );
    //配置CH4, 一个burst 2个数据, 一个block有20个burst
    DMA_Config_CHx
    (
        &DmaRegs.CH4, 4U, 2U, ADC_TEMP_BLOCK_BURST,
        (volatile Uint16 *)&AdccResultRegs.ADCRESULT0,
        (volatile Uint16 *)ADC_TempRawBuf0
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

//DMA 完成一个 block 后，更新状态结构体，通知前台
static void DMA_RecordComplete(volatile DMA_BlockState *state, Uint16 doneBuf, Uint16 doneBurst)
{
    // 如果上一次完成的 block 还没被前台取走, DMA数据丢包计数更新
    if(state->ready != 0U) { state->overrun++; }
    //记录刚刚是哪一个buffer传输完成了
    state->doneBuf = doneBuf;
    // 记录这个 block 里实际有多少个 burst, 实际上这个还是针对快环开发的
    state->doneBurst = doneBurst; 
    // 标记有新的完成块等待前台处理
    state->ready = 1U;
}

//切换乒乓 buffer 并重启通道
static void DMA_StartNextBlock(volatile struct CH_REGS *dmaCh, volatile Uint16 *dest)
{
    //切换新的终点地址
    dmaCh->DST_BEG_ADDR_SHADOW = (Uint32)dest;
    dmaCh->DST_ADDR_SHADOW = (Uint32)dest;
    // 清除外设中断挂起标志，避免旧触发残留
    dmaCh->CONTROL.bit.PERINTCLR = 1U;
    // 清除错误标志
    dmaCh->CONTROL.bit.ERRCLR = 1U;
     //取消暂停, 这个主要是应对快环的, 快环会主动暂停DMA
    dmaCh->CONTROL.bit.HALT = 0U;
    //然后等待下次ADC中断
    dmaCh->CONTROL.bit.RUN = 1U;
}

//在ADC中断函数处被触发, 只是告诉DMA当前快环帧+1
void DMA_NoteFastFrameEoc(void)
{
    if(ADC_FastDmaFrameCnt < ADC_FAST_BLOCK_MAX_BURST)
    {
        ADC_FastDmaFrameCnt++;
    }
}

//这个函数在ecap被调用
//主要进行电网周期边界处理
void DMA_GridCycleBound(void)
{
    // 本周期实际帧数
    Uint16 doneBurst = ADC_FastDmaFrameCnt;
    //设置超时
    Uint16 waitCnt = DMA_FAST_HALT_WAIT_LIM;
    //冻结DMA CH1, 但是到时候这个会在最后解冻
    DmaRegs.CH1.CONTROL.bit.HALT = 1U;
    //一直等,等到超时或者真正暂停了
    while((DmaRegs.CH1.CONTROL.bit.RUNSTS != 0U) && (waitCnt > 0U))
    {
        waitCnt--;
    }
    //快环鉴定一次数据传输是否成功比较严苛, 其他CH都是直接记录的, 只有当
    if
    (
        //不是第一次记录完整的周期
        (ADC_FastDmaCyclePrimed != 0U) &&
        //真正冻结了CH1
        (DmaRegs.CH1.CONTROL.bit.RUNSTS == 0U) &&
        //传输时间小于12.5ms
        (doneBurst >= DMA_FAST_CYCLE_MIN_BURST) &&
        //或者传输时间超过22.5ms(实际上超过了早就进中断了)
        (doneBurst <= ADC_FAST_BLOCK_MAX_BURST)
    )
    {
        //才真正记录一次传输成功
        DMA_RecordComplete(&ADC_FastDmaState, ADC_FastDmaActiveBuf, doneBurst);
    }
    //切buffer
    ADC_FastDmaActiveBuf ^= 1U;
    // 当前周期帧数清零
    ADC_FastDmaFrameCnt = 0U;
    // 标记已预热
    ADC_FastDmaCyclePrimed = 1U;
    // 软复位 CH1，把传输计数等归零
    DmaRegs.CH1.CONTROL.bit.SOFTRESET = 1U;
    __asm(" NOP");
    //重新配置DMA CH1 这和其他CH不同, 它们都用DMA_StartNextBlock
    //究其原因,快环是动态帧数,如果只用 DMA_StartNextBlock，
    //DMA 会接着上次的剩余计数继续搬，而不是从新 block 的开头开始
    DMA_Config_CHx
    (
        &DmaRegs.CH1, 1U, 6U, ADC_FAST_BLOCK_MAX_BURST,
        (volatile Uint16 *)&AdcaResultRegs.ADCRESULT0,
        (ADC_FastDmaActiveBuf == 0U) ?
        (volatile Uint16 *)ADC_FastRawBuf0 :
        (volatile Uint16 *)ADC_FastRawBuf1
    );
    //解冻CH1
    DmaRegs.CH1.CONTROL.bit.HALT = 0U;
    //启动CH1
    DmaRegs.CH1.CONTROL.bit.RUN = 1U;
}

/*进入了DMA中断,就说明当前的buffer满了, 准确来说是block到达对应数量了*/

//DMA CH1 中断函数, 正常来说, 我们都是不会进入的, 因为被提前截断了
__interrupt void DMA_CH1_CPU_ISR(void)
{
    // 帧数清零，本次采集无效
    ADC_FastDmaFrameCnt = 0U;
    // 取消预热，等待下一次采集
    ADC_FastDmaCyclePrimed = 0U;
    // 电网频率相关变量清零，表示没有捕获到有效边界
    gMachineData.ecapFreqCent = 0U;
    //但还是老老实实切换buffer
    ADC_FastDmaActiveBuf ^= 1U;
    //然后换挡,这段期间CPU可以取刚刚完成的buffer数据,处理数据和读取数据实现解耦
    DMA_StartNextBlock
    (
        &DmaRegs.CH1,
        (ADC_FastDmaActiveBuf == 0U) ?
        (volatile Uint16 *)ADC_FastRawBuf0 :
        (volatile Uint16 *)ADC_FastRawBuf1
    );
    //告诉PIE, "我已经处理好了这个中断"
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

//DMA CH2 中断函数, 剩下的就是正常中断了
__interrupt void DMA_CH2_CPU_ISR(void)
{
    // 记录一次block完成
    DMA_RecordComplete(&ADC_PvDmaState, ADC_PvDmaActiveBuf, ADC_PV_BLOCK_BURST);
    //切换buffer
    ADC_PvDmaActiveBuf ^= 1U;
    //换挡
    DMA_StartNextBlock
    (
        &DmaRegs.CH2,
        (ADC_PvDmaActiveBuf == 0U) ?
        (volatile Uint16 *)ADC_PvRawBuf0 :
        (volatile Uint16 *)ADC_PvRawBuf1
    );
    //告诉PIE, "我已经处理好了这个中断"
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

//DMA CH3 中断函数
__interrupt void DMA_CH3_CPU_ISR(void)
{
    // 记录一次block完成
    DMA_RecordComplete(&ADC_InsulDmaState, ADC_InsulDmaActiveBuf, ADC_INSUL_BLOCK_BURST);
    //切换buffer
    ADC_InsulDmaActiveBuf ^= 1U;
    //换挡
    DMA_StartNextBlock
    (
        &DmaRegs.CH3,
        (ADC_InsulDmaActiveBuf == 0U) ?
        (volatile Uint16 *)ADC_InsulRawBuf0 :
        (volatile Uint16 *)ADC_InsulRawBuf1
    );
    //告诉PIE, "我已经处理好了这个中断"
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

//DMA CH4 中断函数
__interrupt void DMA_CH4_CPU_ISR(void)
{
    // 记录一次block完成
    DMA_RecordComplete(&ADC_TempDmaState, ADC_TempDmaActiveBuf, ADC_TEMP_BLOCK_BURST);
    //切换buffer
    ADC_TempDmaActiveBuf ^= 1U;
    //换挡
    DMA_StartNextBlock
    (
        &DmaRegs.CH4,
        (ADC_TempDmaActiveBuf == 0U) ?
        (volatile Uint16 *)ADC_TempRawBuf0 :
        (volatile Uint16 *)ADC_TempRawBuf1
    );
    //告诉PIE, "我已经处理好了这个中断"
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

//检查当前DMA CH是否有可用的buffer
static Uint16 DMA_ClaimBuf(volatile DMA_BlockState *state, Uint16 *doneBuf, Uint16 *doneBurst)//确认数据,更新状态
{
    Uint16 blockReady;
    //关中断, 什么这个工作很重要, 要么不做,要么就全做完
    DINT;
    //冻结住完成状态
    blockReady = state->ready;
    if(blockReady != 0U)
    {
        //注意,这一个函数不负责更新刚刚完成传输的buffer, 而且甚至不直接读取当前的已完成,
        //因为它需要读取的是旧数据,不管你当前是哪个buffer在工作
        //冻结住完成的buffer
        *doneBuf = state->doneBuf;
        if(doneBurst != 0)
        {
            //针对快环数据,还返回当前多少个burst
            *doneBurst = state->doneBurst;
        }
        //清除ready, 表示已经取走
        state->ready = 0U;
    }
    EINT;
    //这里显式告诉你是否准备好,隐式告诉你一个用哪个buf
    return blockReady;
}

//被measure任务调用, 先把码值数据进行简单的累加, 复杂的浮点计算任务放在测量任务
//但是本质上, 他就是任务层, 是底层和中层的桥梁
//其返回值是当前已经完成处理的block
Uint16 DMA_ProcessBlocks
(
    //同时输出码值平均值和码值均方值
    ADC_UintData *rawAvg,
    ADC_FloatData *rawMeanSq,
    //输出完整电网周期内的有功/无功原始统计量
    DMA_GridPowerRaw *gridPowerRaw
)
{
    //冻结后的, 刚完成的是哪个 buffer?
    Uint16 bufIdx;
    //冻结后的, 本 block 实际有多少帧?
    Uint16 fastBlockBurst;
    //通用下表
    Uint16 sampleIdx;
    //记录当前有哪些DMA的数据块更新, 返回给测量任务
    Uint16 update = 0U;
    //临时累加器
    Uint32 sum0;
    Uint32 sum1;
    Uint32 sum2;
    Uint32 sum3;
    Uint32 sum4;
    Uint32 sum5;
    float sqSum0;
    float sqSum1;
    float sqSum2;
    //float sqSum3;
    //float sqSum4;
    float sqSum5;
    //电网功率原始累加量
    float gridActiveSum;
    float gridReactiveSum;
    //各种临时变量
    float centSample;
    //电网电压去偏置值，用于功率计算
    float centGridVolt;
    //电感电流去偏置值，用于功率计算
    float centInductCurr;
    //约四分之一周期后的电网电压样本，用于构造正交电压
    Uint16 orthIdx;
    //orthIdx对应下的Burst, 也就是滞后90°的Burst
    Uint16 quartCycleBurst;
    float orthGridVolt;

    //以下就是轮询5个DMA通道的状态,看是否有数据更新

    if(DMA_ClaimBuf(&ADC_FastDmaState, &bufIdx, &fastBlockBurst) != 0U)//如果快环数据更新
    {
        //这里的bufferIdx就是刚刚问DMA"你有哪个buf是准备好的?" 这是DMA"回答"的结果
        volatile ADC_FastRawFrame *buf = (bufIdx == 0U) ? ADC_FastRawBuf0 : ADC_FastRawBuf1;
        //初始化参数
        sum0 = 0UL;
        sum1 = 0UL;
        sum2 = 0UL;
        sum3 = 0UL;
        sum4 = 0UL;
        sum5 = 0UL;
        sqSum0 = 0.0f;
        sqSum1 = 0.0f;
        sqSum2 = 0.0f;
        //sqSum3 = 0.0f;
        //sqSum4 = 0.0f;
        sqSum5 = 0.0f;
        gridActiveSum = 0.0f;
        gridReactiveSum = 0.0f;
        //这一步加上2再除以4, 你可以想一想是为什么🤨
        quartCycleBurst = (fastBlockBurst + 2U) / 4U;
        //快环的burst是不固定的
        for(sampleIdx = 0U; sampleIdx < fastBlockBurst; sampleIdx++)
        {
            //累加码值
            sum0 += buf[sampleIdx].inductCurr;
            sum1 += buf[sampleIdx].gridVolt;
            sum2 += buf[sampleIdx].gfciCurr;
            sum3 += buf[sampleIdx].dcBusVolt;
            sum4 += buf[sampleIdx].gridDcCurr;
            sum5 += buf[sampleIdx].invertVolt;
            //累加减去偏置的码值平方
            centSample = (float)buf[sampleIdx].inductCurr - gAdcCal.inductCurr.bias;
            centInductCurr = centSample;
            sqSum0 += centSample * centSample;
            centSample = (float)buf[sampleIdx].gridVolt - gAdcCal.gridVolt.bias;
            centGridVolt = centSample;
            sqSum1 += centSample * centSample;
            centSample = (float)buf[sampleIdx].gfciCurr - gAdcCal.gfciCurr.bias;
            sqSum2 += centSample * centSample;
            //centSample = (float)buf[sampleIdx].dcBusVolt - gAdcCal.dcBusVolt.bias;
            //sqSum3 += centSample * centSample;
            //centSample = (float)buf[sampleIdx].gridDcCurr - gAdcCal.gridDcCurr.bias;
            //sqSum4 += centSample * centSample;
            centSample = (float)buf[sampleIdx].invertVolt - gAdcCal.invertVolt.bias;
            sqSum5 += centSample * centSample;
            //P = average(v * i)，暂用电感电流近似并网电流
            //累加有功功率码值
            gridActiveSum += centGridVolt * centInductCurr; 
            //theta + 90deg
            orthIdx = sampleIdx + quartCycleBurst;
            if(orthIdx >= fastBlockBurst)
            {
                //绕回
                orthIdx -= fastBlockBurst;
            }
            //Q = average(v(theta + 90deg) * i)，正值表示电流超前（容性）
            orthGridVolt = (float)buf[orthIdx].gridVolt - gAdcCal.gridVolt.bias;
            //累加无功功率码值
            gridReactiveSum += orthGridVolt * centInductCurr;
        }
        //算码值平均值
        rawAvg->inductCurr =    (Uint16)(sum0 / fastBlockBurst);
        rawAvg->gridVolt =      (Uint16)(sum1 / fastBlockBurst);
        rawAvg->gfciCurr =      (Uint16)(sum2 / fastBlockBurst);
        rawAvg->dcBusVolt =     (Uint16)(sum3 / fastBlockBurst);
        rawAvg->gridDcCurr =    (Uint16)(sum4 / fastBlockBurst);
        rawAvg->invertVolt =    (Uint16)(sum5 / fastBlockBurst);
        //算码值均方值
        rawMeanSq->inductCurr = sqSum0 / (float)fastBlockBurst;
        rawMeanSq->gridVolt =   sqSum1 / (float)fastBlockBurst;
        rawMeanSq->gfciCurr =   sqSum2 / (float)fastBlockBurst;
        //rawMeanSq->dcBusVolt =  sqSum3 / (float)fastBlockBurst;
        //rawMeanSq->gridDcCurr = sqSum4 / (float)fastBlockBurst;
        rawMeanSq->invertVolt = sqSum5 / (float)fastBlockBurst;
        gridPowerRaw->activeMean = gridActiveSum / (float)fastBlockBurst;
        gridPowerRaw->reactiveMean = gridReactiveSum / (float)fastBlockBurst;
        update |= DMA_UPDATE_FAST;
    }

    if(DMA_ClaimBuf(&ADC_PvDmaState, &bufIdx, 0) != 0U)
    {
        volatile ADC_PvRawFrame *buf = (bufIdx == 0U) ? ADC_PvRawBuf0 : ADC_PvRawBuf1;
        sum0 = 0UL;
        sum1 = 0UL;
        sum2 = 0UL;
        sum3 = 0UL;
        //sqSum0 = 0.0f;
        //sqSum1 = 0.0f;
        //sqSum2 = 0.0f;
        //sqSum3 = 0.0f;
        for(sampleIdx = 0U; sampleIdx < ADC_PV_BLOCK_BURST; sampleIdx++)
        {
            sum0 += buf[sampleIdx].pv1Volt;
            sum1 += buf[sampleIdx].pv1Curr;
            sum2 += buf[sampleIdx].pv2Volt;
            sum3 += buf[sampleIdx].pv2Curr;
            //centSample = (float)buf[sampleIdx].pv1Volt - gAdcCal.pv1Volt.bias;
            //sqSum0 += centSample * centSample;
            //centSample = (float)buf[sampleIdx].pv1Curr - gAdcCal.pv1Curr.bias;
            //sqSum1 += centSample * centSample;
            //centSample = (float)buf[sampleIdx].pv2Volt - gAdcCal.pv2Volt.bias;
            //sqSum2 += centSample * centSample;
            //centSample = (float)buf[sampleIdx].pv2Curr - gAdcCal.pv2Curr.bias;
            //sqSum3 += centSample * centSample;
        }
        rawAvg->pv1Volt = (Uint16)(sum0 / ADC_PV_BLOCK_BURST);
        rawAvg->pv1Curr = (Uint16)(sum1 / ADC_PV_BLOCK_BURST);
        rawAvg->pv2Volt = (Uint16)(sum2 / ADC_PV_BLOCK_BURST);
        rawAvg->pv2Curr = (Uint16)(sum3 / ADC_PV_BLOCK_BURST);
        //rawMeanSq->pv1Volt = sqSum0 / (float)ADC_PV_BLOCK_BURST;
        //rawMeanSq->pv1Curr = sqSum1 / (float)ADC_PV_BLOCK_BURST;
        //rawMeanSq->pv2Volt = sqSum2 / (float)ADC_PV_BLOCK_BURST;
        //rawMeanSq->pv2Curr = sqSum3 / (float)ADC_PV_BLOCK_BURST;
        update |= DMA_UPDATE_PV;
    }

    if(DMA_ClaimBuf(&ADC_InsulDmaState, &bufIdx, 0) != 0U)
    {
        volatile ADC_InsulRawFrame *buf = (bufIdx == 0U) ? ADC_InsulRawBuf0 : ADC_InsulRawBuf1;
        sum0 = 0UL;
        sum1 = 0UL;
        //sqSum0 = 0.0f;
        //sqSum1 = 0.0f;
        for(sampleIdx = 0U; sampleIdx < ADC_INSUL_BLOCK_BURST; sampleIdx++)
        {
            sum0 += buf[sampleIdx].pv1Insul;
            sum1 += buf[sampleIdx].pv2Insul;
            //centSample = (float)buf[sampleIdx].pv1Insul - gAdcCal.pv1Insul.bias;
            //sqSum0 += centSample * centSample;
            //centSample = (float)buf[sampleIdx].pv2Insul - gAdcCal.pv2Insul.bias;
            //sqSum1 += centSample * centSample;
        }
        rawAvg->pv1Insul = (Uint16)(sum0 / ADC_INSUL_BLOCK_BURST);
        rawAvg->pv2Insul = (Uint16)(sum1 / ADC_INSUL_BLOCK_BURST);
        //rawMeanSq->pv1Insul = sqSum0 / (float)ADC_INSUL_BLOCK_BURST;
        //rawMeanSq->pv2Insul = sqSum1 / (float)ADC_INSUL_BLOCK_BURST;
        update |= DMA_UPDATE_INSUL;
    }

    if(DMA_ClaimBuf(&ADC_TempDmaState, &bufIdx, 0) != 0U)
    {
        volatile ADC_TempRawFrame *buf = (bufIdx == 0U) ? ADC_TempRawBuf0 : ADC_TempRawBuf1;
        sum0 = 0UL;
        sum1 = 0UL;
        for(sampleIdx = 0U; sampleIdx < ADC_TEMP_BLOCK_BURST; sampleIdx++)
        {
            sum0 += buf[sampleIdx].invertTemp;
            sum1 += buf[sampleIdx].boostTemp;
        }
        rawAvg->invertTemp = (Uint16)(sum0 / ADC_TEMP_BLOCK_BURST);
        rawAvg->boostTemp = (Uint16)(sum1 / ADC_TEMP_BLOCK_BURST);
        update |= DMA_UPDATE_TEMP;
    }
    //告诉测量任务哪些数据块更新了
    return update;
}
