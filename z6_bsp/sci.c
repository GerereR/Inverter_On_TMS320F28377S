#include "F28x_Project.h"
#include "bsp.h"

//SCI时钟四分频, 为50MHz
#define SCI_LSPCLK_DIV      4UL
#define SCI_LSPCLK_HZ       (SYSCLK_FREQ_HZ / SCI_LSPCLK_DIV)
//SCI通讯波特率9600
#define SCI_BAUDRATE        9600UL
//根据波特率需求配置BRR寄存器, 数据手册都有的
#define SCI_BRR_VALUE       ((SCI_LSPCLK_HZ / (SCI_BAUDRATE * 8UL)) - 1UL)
//软件环形buffer大小
#define SCI_RX_BUFFER_SIZE  512U
#define SCI_TX_BUFFER_SIZE  256U

//创建 SCI 软件环形缓冲区。
static volatile Uint16 SCI_RxSwBuf[SCI_RX_BUFFER_SIZE];
static volatile Uint16 SCI_TxSwBuf[SCI_TX_BUFFER_SIZE];


//对于发送和接收软件buffer来说,都有两个指针进行管理
//之所以要配备两个指针,是因为软件buffer同时进行生产和消费
//对于RxBuffer: 上游生产者肯定是FIFO传入的,在软件里表现为SCI中断函数,所以是写指针
//              下游的消费者肯定是CPU进行处理,在软件里表现为SCI通讯接收任务,所以是读指针
//对于TxBuffer: 上游生产者肯定是CPU的请求,在软件里表现为SCI通讯发送任务,所以是写指针
//              下游的消费者肯定是FIFO传出,在软件里表现为FIFO填充函数,所以是读指针

//RX软件环形缓冲区写指针，由接收 ISR 更新
static volatile Uint16 SCI_RxSwBufWriteIndex = 0U;
//RX软件环形缓冲区读指针，由通信任务调用 SCI_ReadByte() 更新
static volatile Uint16 SCI_RxSwBufReadIndex = 0U;
//TX软件环形缓冲区写指针，由通信任务提交数据后更新
static volatile Uint16 SCI_TxSwBufWriteIndex = 0U;
//TX软件环形缓冲区读指针，由发送逻辑搬入硬件 FIFO 后更新
static volatile Uint16 SCI_TxSwBufReadIndex = 0U;

//记录软件缓冲区满时丢弃了多少字符
static volatile Uint32 SCI_RxOverflowCount = 0UL;
//队列放不下完整响应帧时的丢帧次数。
static volatile Uint32 SCI_TxDroppedFrameCount = 0UL;

static void SCI_FillTxFifo(void);
static __interrupt void SCIB_BSP_RX_ISR(void);
static __interrupt void SCIB_BSP_TX_ISR(void);

//SCI配置
void SCI_Config(void)
{
    SCI_RxSwBufWriteIndex = 0U;
    SCI_RxSwBufReadIndex = 0U;
    SCI_TxSwBufWriteIndex = 0U;
    SCI_TxSwBufReadIndex = 0U;

    SCI_RxOverflowCount = 0UL;
    SCI_TxDroppedFrameCount = 0UL;

    EALLOW;

    //配置期间先让 SCI 保持复位
    ScibRegs.SCICTL1.bit.SWRESET = 0;

    //清除之前通讯配置
    ScibRegs.SCICCR.all = 0U;
    //8位数据,其中一位是停止位
    ScibRegs.SCICCR.bit.SCICHAR = 7U;

    //清除之前收发配置
    ScibRegs.SCICTL1.all = 0U;
    //发送和接收使能
    ScibRegs.SCICTL1.bit.RXENA = 1U;
    ScibRegs.SCICTL1.bit.TXENA = 1U;
    //波特率设置,9600
    ScibRegs.SCIHBAUD.all = (Uint16)(SCI_BRR_VALUE >> 8U);
    ScibRegs.SCILBAUD.all = (Uint16)(SCI_BRR_VALUE & 0x00FFU);

    //清除之前TxFIFO配置
    ScibRegs.SCIFFTX.all = 0U;
    //开启增强 FIFO 模式
    ScibRegs.SCIFFTX.bit.SCIFFENA = 1U;
    //释放 SCI FIFO 通道复位
    ScibRegs.SCIFFTX.bit.SCIRST = 1U;
    //TX FIFO 空时产生中断条件
    ScibRegs.SCIFFTX.bit.TXFFIL = 0U;
    //初始化阶段暂时关闭 TX FIFO 中断
    ScibRegs.SCIFFTX.bit.TXFFIENA = 0U;
    //TxFIFO复位
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1U;
    //清除TxFIFO中断标志
    ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1U;

    //清除之前RxFIFO配置
    ScibRegs.SCIFFRX.all = 0U;
    //配置RxFIFO中断: 内有一字节时中断
    ScibRegs.SCIFFRX.bit.RXFFIL = 1U;
    //开启增强 FIFO 模式
    ScibRegs.SCIFFRX.bit.RXFFIENA = 1U;
    //清除之前TxFIFO中断
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1U;
    //RxFIFO复位
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1U;
    //清除RxFIFO溢出标志
    ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1U;

    //清除FIFO控制寄存器数据
    ScibRegs.SCIFFCT.all = 0U;

    //清除SCI控制寄存器2数据
    ScibRegs.SCICTL2.all = 0U;

    //软件调试配置
    ScibRegs.SCIPRI.bit.FREESOFT = 3U;

    //中断接口
    PieVectTable.SCIB_RX_INT = &SCIB_BSP_RX_ISR;
    PieVectTable.SCIB_TX_INT = &SCIB_BSP_TX_ISR;

    //SCI开始工作
    ScibRegs.SCICTL1.bit.SWRESET = 1;

    //SCIB RX 是 PIE 9.3，TX 是 PIE 9.4
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1U;
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1U;
    IER |= M_INT9;

    EDIS;
}

//非阻塞发送函数
Uint16 SCI_TrySend(const Uint16 *data, Uint16 length)
{
    Uint16 interruptState;
    Uint16 readIndex;
    Uint16 writeIndex;
    Uint16 freeSlots;
    Uint16 index;

    //return 0: 参数非法或者队列空间不足
    if((data == 0) || (length == 0U) || (length >= SCI_TX_BUFFER_SIZE))
    {
        return 0U;
    }

    readIndex = SCI_TxSwBufReadIndex;
    writeIndex = SCI_TxSwBufWriteIndex;

    //下面这个判断是为了算出buffer剩余可用空间,这得发挥一下想象力😓
    //第一种情况: 写指针尚未绕会队首，空闲区跨越队列尾部。
    //▯▯▯▯▯▯▯...▯▯▯▮▮▮▮▮▮▮▮▮▮▯▯▯
    //                     ↑               ↑
    //                   Read-→         Write-→
    if(writeIndex >= readIndex)
    {
        //由于第256故意保留一个空槽,用来区分"空"和"满",所以要减一
        freeSlots = (SCI_TX_BUFFER_SIZE - 1U) - (writeIndex - readIndex);
    }
    //第二种情况: 写指针已经绕会队首，空闲区位于读写指针之间
    //▮▮▮▮▯▯▯...▯▯▯▯▯▯▯▯▯▯▯▮▮▮▮▮
    //      ↑                            ↑               
    //    Write-→                       Read-→         
    else
    {
        freeSlots = (readIndex - writeIndex) - 1U;
    }

    //剩余空间不够,说明丢帧了
    if(length > freeSlots)
    {
        SCI_TxDroppedFrameCount++;
        gSysProblem.warning |= WARNING_SCI_FRAME_DROPPED;
        return 0U;
    }

    //如果剩余空间足够,那就把数据一个个放在buffer里面
    for(index = 0U; index < length; index++)
    {
        //buffer只能存16位数据
        SCI_TxSwBuf[writeIndex] = data[index] & 0x00FFU;
        writeIndex++;
        //绕回
        if(writeIndex >= SCI_TX_BUFFER_SIZE)
        {
            writeIndex = 0U;
        }
    }

    //禁止CPU中断
    interruptState = CPU_InterruptSaveDisable();
    //写完之后实时同步一下当前的写指针落在哪个位置
    SCI_TxSwBufWriteIndex = writeIndex;
    //接下来就就交给FIFO填充函数来把数据放在FIFO里面
    SCI_FillTxFifo();
    //使能CPU中断
    CPU_InterruptRestore(interruptState);

    //return 1: 完整数据已经进入发送系统
    return 1U;
}

//非阻塞读取函数
Uint16 SCI_ReadByte(Uint16 *data)
{
    //接收读取是由CPU执行
    Uint16 readIndex = SCI_RxSwBufReadIndex;

    if(data == 0)
    {
        return 0U;
    }
    //如果当前FIFO还没写好,就直接返回0
    if(readIndex == SCI_RxSwBufWriteIndex)
    {
        return 0U;
    }
    //缓存区放到目标指针里面
    *data = SCI_RxSwBuf[readIndex];
    //准备读下一个缓存区数据
    readIndex++;
    //绕回
    if(readIndex >= SCI_RX_BUFFER_SIZE)
    {
        readIndex = 0U;
    }
    //全局读指针更新
    SCI_RxSwBufReadIndex = readIndex;

    return 1U;
}

//判断当前接收缓存是否有数据
Uint16 SCI_HasRxData(void)
{
    //判断的标准就是读指针追上了写指针
    return (SCI_RxSwBufReadIndex != SCI_RxSwBufWriteIndex) ? 1U : 0U;
}

//FIFO填充函数,这是发送函数的主体
static void SCI_FillTxFifo(void)
{
    //发送读指针,就是FIFO的要取的数据的指针
    Uint16 readIndex = SCI_TxSwBufReadIndex;
    //如果软件Buffer还有数据,且硬件FIFO还有空位(哪怕只有一个)
    while((readIndex != SCI_TxSwBufWriteIndex) && (ScibRegs.SCIFFTX.bit.TXFFST < 16U))
    {
        //将软件buffer里面数据推进FIFO里面
        ScibRegs.SCITXBUF.bit.TXDT = SCI_TxSwBuf[readIndex];
        readIndex++;
        //绕回
        if(readIndex >= SCI_TX_BUFFER_SIZE)
        {
            readIndex = 0U;
        }
    }

    //更新全局发送读指针
    SCI_TxSwBufReadIndex = readIndex;
    //清除TxFIFO中断标志
    ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1U;
    //很巧妙的一点,当软件buffer没有数据了,就不再需要发送中断了
    ScibRegs.SCIFFTX.bit.TXFFIENA = (readIndex != SCI_TxSwBufWriteIndex) ? 1U : 0U;
}

//SCI接收中断条件:FIFO内有一字节数据
static __interrupt void SCIB_BSP_RX_ISR(void)
{
    //临时保存数据
    Uint16 data;
    //索引
    Uint16 nextIndex;

    //搬运直到FIFO彻底空
    while(ScibRegs.SCIFFRX.bit.RXFFST != 0U)
    {
        //直接读取FIFO硬件数据
        data = ScibRegs.SCIRXBUF.bit.SAR;
        //这个指针是CPU通讯任务写进去的
        nextIndex = SCI_RxSwBufWriteIndex + 1U;

        //环形缓冲的实现
        if(nextIndex >= SCI_RX_BUFFER_SIZE)
        {
            nextIndex = 0U;
        }
        //软件FIFO是不是满了?
        if(nextIndex != SCI_RxSwBufReadIndex)    
        {
            SCI_RxSwBuf[SCI_RxSwBufWriteIndex] = data;  
            SCI_RxSwBufWriteIndex = nextIndex;
        }
        //满了的话就记录当前丢失了多少次
        else
        {
            SCI_RxOverflowCount++;
            gSysProblem.warning |= WARNING_SCI_FRAME_DROPPED;
        }
    }

    //当 SCI 接收 FIFO 已满，并且又收到了新的数据字时，RXFFOVF 会被硬件置 1
    //这是硬件级的溢出检测
    if(ScibRegs.SCIFFRX.bit.RXFFOVF != 0U)
    {
        SCI_RxOverflowCount++;
        gSysProblem.warning |= WARNING_SCI_FRAME_DROPPED;
    }

    //清除中断溢出标志
    ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    //清除中断标志
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    //通知PIE已经处理中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

//SCI发送中断条件:FIFO空了就发出中断
static __interrupt void SCIB_BSP_TX_ISR(void)
{
    //有两个FIFO填充函数,虽然会在功能上造成轻微的耦合,但好处是降低了CPU的负担
    //因为之前TxFIFO配置是空中断,假如很长时间都没有数据,岂不是时时刻刻都要中断?
    //为了解决这个问题,就让TrySend任务唤醒它一次,之后就是中断通知搬运
    //某种意义上就是双向通知功能
    SCI_FillTxFifo();
    //通知PIE:我已处理中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}
