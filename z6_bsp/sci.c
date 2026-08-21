#include "F28x_Project.h"
#include "bsp.h"

// InitSysCtrl() leaves LSPCLK at SYSCLK/4 = 50 MHz in this project.
#define SCI_LSPCLK_HZ       50000000UL      //SCI时钟设置为50MHz
#define SCI_BAUDRATE        9600UL      //波特率9600
#define SCI_BRR_VALUE       ((SCI_LSPCLK_HZ / (SCI_BAUDRATE * 8UL)) - 1UL)  //其实波特率是BRR决定的
#define SCI_RX_BUFFER_SIZE  512U    //500 ms 通信周期下的接收缓冲区大小

static volatile Uint16 SCI_RxBuffer[SCI_RX_BUFFER_SIZE];  //创建 SCI 软件接收环形缓冲区。
static volatile Uint16 SCI_RxWriteIndex = 0U;   //写指针，由接收 ISR 更新
static volatile Uint16 SCI_RxReadIndex = 0U;    //读指针，由主程序调用 SCI_ReadByte() 更新
volatile Uint16 SCI_RxDataPending = 0U;         //RX ISR 置位，通信任务处理后重新检查缓冲区
static volatile Uint32 SCI_RxOverflowCount = 0UL;  //记录软件缓冲区满时丢弃了多少字符

void SCI_Config(void)
{
    EALLOW;

    // Hold SCI in reset while format, baud rate, and FIFO are configured.
    SciaRegs.SCICTL1.bit.SWRESET = 0;   //说明配置期间先让 SCI 保持复位
    SciaRegs.SCICCR.all = 0x0007;       // 8 data bits, no parity, 1 stop bit. 普通异步串口
    SciaRegs.SCICTL1.all = 0x0003;     // 打开接收器和发送器,但 SWRESET 仍然为 0，因此 SCI 还没有真正运行
    SciaRegs.SCIHBAUD.all = (Uint16)(SCI_BRR_VALUE >> 8);//把 BRR 的高 8 位写入高波特率寄存器。
    SciaRegs.SCILBAUD.all = (Uint16)(SCI_BRR_VALUE & 0xFFU);//把 BRR 的低 8 位写入低波特率寄存器。

    SciaRegs.SCIFFTX.all = 0xE040;  //配置发送FIFO,发送不使用中断
    SciaRegs.SCIFFRX.all = 0x2061;  //配置接收FIFO,RX FIFO 里面只要有 1 个字节，就触发 RX FIFO 中断

    SciaRegs.SCIFFCT.all = 0x0000;  //FIFO相关特殊控制功能关闭

    SciaRegs.SCICTL2.all = 0x0000;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;//打开RX 接收中断

    SciaRegs.SCIPRI.bit.FREESOFT = 3;//仿真器暂停 CPU

    PieVectTable.SCIA_RX_INT = &SCIA_BSP_RX_ISR;

    SciaRegs.SCICTL1.bit.SWRESET = 1;//SCI开始工作

    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
    IER |= M_INT9;

    EDIS;
}

void SCI_SendByte(Uint16 data)
{
    while(SciaRegs.SCIFFTX.bit.TXFFST >= 16U)//如果FIFO满了
    {
        //里面可以加上超时保护
    }
    SciaRegs.SCITXBUF.bit.TXDT = data & 0x00FFU;//发送低八位
}

void SCI_SendString(const char *text)
{
    while(*text != '\0')
    {
        SCI_SendByte((Uint16)*text);
        text++;
    }
}

Uint16 SCI_ReadByte(Uint16 *data)
{
    Uint16 readIndex = SCI_RxReadIndex;

    if(readIndex == SCI_RxWriteIndex)//如果当前
    {
        return 0U;
    }

    *data = SCI_RxBuffer[readIndex];//放到缓存区里面
    readIndex++;
    if(readIndex >= SCI_RX_BUFFER_SIZE)
    {
        readIndex = 0U;
    }
    SCI_RxReadIndex = readIndex;

    return 1U;
}

__interrupt void SCIA_BSP_RX_ISR(void)//存进软件buffer,至于用不用,需要task_comm判断
{
    Uint16 data;  //临时保存数据
    Uint16 nextIndex;   //索引

    while(SciaRegs.SCIFFRX.bit.RXFFST != 0U)//读数据知道FIFO彻底空了
    {
        data = SciaRegs.SCIRXBUF.bit.SAR; //直接读取FIFO硬件数据
        nextIndex = SCI_RxWriteIndex + 1U;  //索引++
        if(nextIndex >= SCI_RX_BUFFER_SIZE) //从头开始
        {
            nextIndex = 0U;
        }

        if(nextIndex != SCI_RxReadIndex)    //软件FIFO是不是满了?
        {
            SCI_RxBuffer[SCI_RxWriteIndex] = data;  //
            SCI_RxWriteIndex = nextIndex;
        }
        else//满了的话
        {
            SCI_RxOverflowCount++;
        }
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    SCI_RxDataPending = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}
