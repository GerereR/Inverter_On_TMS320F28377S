#include "F28x_Project.h"
#include "bsp.h"


//I2C配置层========================================================================================
// 200 MHz SYSCLK -> 10 MHz I2C module clock; these dividers target 400 kHz SCL.
// First OLED bring-up should still be validated on a scope/analyzer; drop to
// 100 kHz if the wiring/pull-ups cannot sustain 400 kHz.

#define I2C_PRESCALER            19U  /* IPSC:200MHz / (19+1) = 10MHz 模块时钟 */
#define I2C_CLOCK_LOW            10U  /* ICCL:低电平 15 个模块周期 = 1.5us    */
#define I2C_CLOCK_HIGH           5U   /* ICCH:高电平 10 个模块周期 = 1.0us    */
#define I2C_WAIT_LOOPS_PER_US    20UL /* 空转延时估算系数:约 20 圈 ≈ 1us,只用于超时兜底 */
#define I2C_DEFAULT_TIMEOUT_US   2000U /* 单次等待的默认超时2ms               */
#define I2C_TX_FIFO_DEPTH        16U  /* F2837xS I2C发送FIFO固定为16字节     */
#define I2C_TX_FIFO_REFILL_LEVEL 8U   /* 降到半满时批量补充,给高优先级ISR留余量 */

//把现实时间换算成CPU轮询次数；只作为总线异常时的退出兜底。
static Uint32 I2C_WaitLoopsFromUs(Uint16 timeoutUs)
{
    Uint32 loops = (Uint32)timeoutUs * I2C_WAIT_LOOPS_PER_US;
    return (loops == 0UL) ? 1UL : loops;
}

//这些状态位是锁存型标志，必须由软件写1清除。
static void I2C_ClearStatusFlags(void)
{
    I2caRegs.I2CSTR.bit.ARBL = 1U;
    I2caRegs.I2CSTR.bit.NACK = 1U;
    I2caRegs.I2CSTR.bit.ARDY = 1U;
    I2caRegs.I2CSTR.bit.SCD  = 1U;
}

/* 读取错误状态并映射为bsp.h中的公共错误码。 */
static Uint16 I2C_GetErrorStatus(void)
{
    if(I2caRegs.I2CSTR.bit.ARBL != 0U)
    {
        return I2C_STATUS_ARBITRATION_LOST;
    }
    if(I2caRegs.I2CSTR.bit.NACK != 0U)
    {
        return I2C_STATUS_NACK;
    }
    return I2C_STATUS_OK;
}

/* 丢弃发送FIFO中的残留字节，同时保留FIFO使能和轮询工作方式。 */
static void I2C_ResetTxFifo(void)
{
    I2caRegs.I2CFFTX.bit.TXFFRST = 0U;
    I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1U;
    I2caRegs.I2CFFTX.bit.TXFFRST = 1U;
}

/* 出错统一收尾：请求STOP、清空FIFO并清除锁存状态。 */
static Uint16 I2C_FinishWithError(Uint16 status)
{
    I2caRegs.I2CMDR.bit.STP = 1U;
    I2C_ResetTxFifo();
    I2C_ClearStatusFlags();
    return status;
}

/* 每次轮询同时检查总线错误和软件超时。 */
static Uint16 I2C_WaitPoll(Uint32 *waitLoops)
{
    Uint16 status = I2C_GetErrorStatus();
    if(status != I2C_STATUS_OK)
    {
        return I2C_FinishWithError(status);
    }
    if(--(*waitLoops) == 0UL)
    {
        return I2C_FinishWithError(I2C_STATUS_TIMEOUT);
    }
    return I2C_STATUS_OK;
}

/* I2CA初始化：400kHz SCL、主机模式、FIFO使能、纯轮询。 */
void I2C_Config(void)
{
    EALLOW;

    I2caRegs.I2CMDR.bit.IRS = 0U;
    DELAY_US(1000);
    I2caRegs.I2CFFTX.all = 0x0000;
    I2caRegs.I2CFFRX.all = 0x0040;
    I2caRegs.I2CPSC.bit.IPSC = I2C_PRESCALER;
    I2caRegs.I2CCLKL = I2C_CLOCK_LOW;
    I2caRegs.I2CCLKH = I2C_CLOCK_HIGH;
    I2caRegs.I2COAR.bit.OAR = 0x0000;
    I2caRegs.I2CSAR.bit.SAR = 0x0000;
    I2C_ClearStatusFlags();

    /* 不使用I2C模块中断；UI任务通过FIFO状态进行低优先级轮询。 */
    I2caRegs.I2CIER.all = 0x0000;

    I2caRegs.I2CMDR.all = 0x0000;
    I2caRegs.I2CMDR.bit.IRS = 1U;
    I2caRegs.I2CMDR.bit.FREE = 1U;

    /* 释放并使能收发FIFO；FIFO中断保持关闭。 */
    I2caRegs.I2CFFTX.all = 0x6040;
    I2caRegs.I2CFFRX.all = 0x2040;

    EDIS;
}

/* I2C轮询式主机发送：
 *   1. 等待总线空闲并配置一次完整事务；
 *   2. START前预装最多16字节；
 *   3. FIFO降至半满时按空余槽位批量补充；
 *   4. 等待FIFO、移位器和总线全部空闲后返回。*/
Uint16 I2C_MasterWrite
(
    Uint16 slaveAddr7,
    const unsigned char *data,
    Uint16 length,
    Uint16 timeoutUs
)
{
    Uint16 index;
    Uint16 status;
    Uint16 fifoUsed;
    Uint16 fifoFree;
    Uint32 waitLoops;

    if((data == 0) || (length == 0U) || (slaveAddr7 > 0x7FU))
    {
        return I2C_STATUS_BAD_PARAMETER;
    }
    if(timeoutUs == 0U)
    {
        timeoutUs = I2C_DEFAULT_TIMEOUT_US;
    }

    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        if(waitLoops == 0UL)
        {
            return I2C_STATUS_BUS_BUSY;
        }
        waitLoops--;
    }

    I2C_ClearStatusFlags();
    I2caRegs.I2CSAR.bit.SAR = slaveAddr7;
    I2caRegs.I2CCNT = length;

    I2caRegs.I2CMDR.all = 0x0000;
    I2caRegs.I2CMDR.bit.IRS = 1U;
    I2caRegs.I2CMDR.bit.TRX = 1U;
    I2caRegs.I2CMDR.bit.MST = 1U;
    I2caRegs.I2CMDR.bit.FREE = 1U;
    I2caRegs.I2CMDR.bit.STP = 1U;

    /* 每次事务先清空FIFO，避免上次异常遗留的数据混入本帧。 */
    I2C_ResetTxFifo();

    index = 0U;
    while((index < length) && (index < I2C_TX_FIFO_DEPTH))
    {
        I2caRegs.I2CDXR.bit.DATA = data[index];
        index++;
    }
    I2caRegs.I2CMDR.bit.STT = 1U;

    while(index < length)
    {
        waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
        while(I2caRegs.I2CFFTX.bit.TXFFST > I2C_TX_FIFO_REFILL_LEVEL)
        {
            status = I2C_WaitPoll(&waitLoops);
            if(status != I2C_STATUS_OK)
            {
                return status;
            }
        }

        status = I2C_GetErrorStatus();
        if(status != I2C_STATUS_OK)
        {
            return I2C_FinishWithError(status);
        }

        fifoUsed = I2caRegs.I2CFFTX.bit.TXFFST;
        fifoFree = I2C_TX_FIFO_DEPTH - fifoUsed;
        while((fifoFree > 0U) && (index < length))
        {
            I2caRegs.I2CDXR.bit.DATA = data[index];
            index++;
            fifoFree--;
        }
    }

    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while((I2caRegs.I2CFFTX.bit.TXFFST != 0U) ||
          (I2caRegs.I2CSTR.bit.XSMT == 0U) ||
          (I2caRegs.I2CSTR.bit.BB != 0U))
    {
        status = I2C_WaitPoll(&waitLoops);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }

    status = I2C_GetErrorStatus();
    if(status != I2C_STATUS_OK)
    {
        return I2C_FinishWithError(status);
    }

    I2C_ClearStatusFlags();
    return I2C_STATUS_OK;
}

/* Probe a slave address without writing a data byte.  EEPROM ACK polling
 * uses this transaction while the device completes its internal write cycle. */
Uint16 I2C_MasterProbe(Uint16 slaveAddr7, Uint16 timeoutUs)
{
    Uint16 status;
    Uint32 waitLoops;

    if(slaveAddr7 > 0x7FU)
    {
        return I2C_STATUS_BAD_PARAMETER;
    }
    if(timeoutUs == 0U)
    {
        timeoutUs = I2C_DEFAULT_TIMEOUT_US;
    }

    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        if(waitLoops == 0UL)
        {
            return I2C_STATUS_BUS_BUSY;
        }
        waitLoops--;
    }

    I2C_ClearStatusFlags();
    I2caRegs.I2CSAR.bit.SAR = slaveAddr7;
    I2caRegs.I2CCNT = 0U;
    I2caRegs.I2CMDR.all = 0x0000;
    I2caRegs.I2CMDR.bit.IRS = 1U;
    I2caRegs.I2CMDR.bit.TRX = 1U;
    I2caRegs.I2CMDR.bit.MST = 1U;
    I2caRegs.I2CMDR.bit.FREE = 1U;
    I2caRegs.I2CMDR.bit.STP = 1U;
    I2caRegs.I2CMDR.bit.STT = 1U;

    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        status = I2C_WaitPoll(&waitLoops);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }

    status = I2C_GetErrorStatus();
    I2C_ClearStatusFlags();
    return status;
}

/* Receive a fixed number of bytes from a slave after a START condition. */
Uint16 I2C_MasterRead
(
    Uint16 slaveAddr7,  
    unsigned char *data,    //待填数据块
    Uint16 length,          //长度
    Uint16 timeoutUs
)
{
    Uint16 index;
    Uint16 status;
    Uint32 waitLoops;

    if((data == 0) || (length == 0U) || (slaveAddr7 > 0x7FU))
    {
        return I2C_STATUS_BAD_PARAMETER;
    }
    if(timeoutUs == 0U)
    {
        timeoutUs = I2C_DEFAULT_TIMEOUT_US;
    }

    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        if(waitLoops == 0UL)
        {
            return I2C_STATUS_BUS_BUSY;
        }
        waitLoops--;
    }

    I2C_ClearStatusFlags();
    I2caRegs.I2CSAR.bit.SAR = slaveAddr7;
    I2caRegs.I2CCNT = length;
    I2caRegs.I2CMDR.all = 0x0000;
    I2caRegs.I2CMDR.bit.IRS = 1U;
    I2caRegs.I2CMDR.bit.MST = 1U;
    I2caRegs.I2CMDR.bit.FREE = 1U;
    I2caRegs.I2CMDR.bit.STP = 1U;

    I2caRegs.I2CFFRX.bit.RXFFRST = 0U;
    I2caRegs.I2CFFRX.bit.RXFFINTCLR = 1U;
    I2caRegs.I2CFFRX.bit.RXFFRST = 1U;
    I2caRegs.I2CMDR.bit.STT = 1U;

    for(index = 0U; index < length; index++)
    {
        waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
        while((I2caRegs.I2CFFRX.bit.RXFFST == 0U) &&
              (I2caRegs.I2CSTR.bit.RRDY == 0U))
        {
            status = I2C_WaitPoll(&waitLoops);
            if(status != I2C_STATUS_OK)
            {
                return status;
            }
        }
        data[index] = (unsigned char)I2caRegs.I2CDRR.bit.DATA;
    }

    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        status = I2C_WaitPoll(&waitLoops);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }
    status = I2C_GetErrorStatus();
    I2C_ClearStatusFlags();
    return status;
}

/* Write an EEPROM/register address and immediately issue a repeated START
 * for the read phase.  The write phase deliberately omits STOP. */
Uint16 I2C_MasterWriteRead
(
    Uint16 slaveAddr7,
    const unsigned char *writeData,
    Uint16 writeLength,
    unsigned char *readData,
    Uint16 readLength,
    Uint16 timeoutUs
)
{
    Uint16 index;
    Uint16 status;
    Uint32 waitLoops;

    if((writeData == 0) || (readData == 0) ||
       (writeLength == 0U) || (readLength == 0U) ||
       (writeLength > I2C_TX_FIFO_DEPTH) || (slaveAddr7 > 0x7FU))
    {
        return I2C_STATUS_BAD_PARAMETER;
    }
    if(timeoutUs == 0U)
    {
        timeoutUs = I2C_DEFAULT_TIMEOUT_US;
    }

    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        if(waitLoops == 0UL)
        {
            return I2C_STATUS_BUS_BUSY;
        }
        waitLoops--;
    }

    I2C_ClearStatusFlags();
    I2caRegs.I2CSAR.bit.SAR = slaveAddr7;
    I2caRegs.I2CCNT = writeLength;
    I2caRegs.I2CMDR.all = 0x0000;
    I2caRegs.I2CMDR.bit.IRS = 1U;
    I2caRegs.I2CMDR.bit.TRX = 1U;
    I2caRegs.I2CMDR.bit.MST = 1U;
    I2caRegs.I2CMDR.bit.FREE = 1U;
    I2caRegs.I2CFFTX.bit.TXFFRST = 0U;
    I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1U;
    I2caRegs.I2CFFTX.bit.TXFFRST = 1U;

    for(index = 0U; index < writeLength; index++)
    {
        I2caRegs.I2CDXR.bit.DATA = writeData[index];
    }
    /* No STOP: ARDY marks the end of the address phase. */
    I2caRegs.I2CMDR.bit.STT = 1U;

    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.ARDY == 0U)
    {
        status = I2C_WaitPoll(&waitLoops);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }
    I2caRegs.I2CSTR.bit.ARDY = 1U;

    I2caRegs.I2CCNT = readLength;
    I2caRegs.I2CMDR.bit.TRX = 0U;
    I2caRegs.I2CMDR.bit.STP = 1U;
    I2caRegs.I2CFFRX.bit.RXFFRST = 0U;
    I2caRegs.I2CFFRX.bit.RXFFINTCLR = 1U;
    I2caRegs.I2CFFRX.bit.RXFFRST = 1U;
    I2caRegs.I2CMDR.bit.STT = 1U;

    for(index = 0U; index < readLength; index++)
    {
        waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
        while((I2caRegs.I2CFFRX.bit.RXFFST == 0U) &&
              (I2caRegs.I2CSTR.bit.RRDY == 0U))
        {
            status = I2C_WaitPoll(&waitLoops);
            if(status != I2C_STATUS_OK)
            {
                return status;
            }
        }
        readData[index] = (unsigned char)I2caRegs.I2CDRR.bit.DATA;
    }

    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        status = I2C_WaitPoll(&waitLoops);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }
    status = I2C_GetErrorStatus();
    I2C_ClearStatusFlags();
    return status;
}
