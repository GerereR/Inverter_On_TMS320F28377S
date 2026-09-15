#include "F28x_Project.h"
#include "bsp.h"

//IIC模块时钟200MHz/20 = 10MHz
#define I2C_PRESCALER            19U
//低电平 15 个模块周期 = 1.5us
#define I2C_CLOCK_LOW            10U
//高电平 10 个模块周期 = 1.0us
//加起来2.5us,也就是400KHz
#define I2C_CLOCK_HIGH           5U
//空转延时估算系数:约 20 圈 ≈ 1us,只用于超时兜底
#define I2C_WAIT_LOOPS_PER_US    20UL
//单次等待的默认超时2ms
#define I2C_DEFAULT_TIMEOUT_US   2000U
//F2837xS I2CTxFIFO固定为16字节
#define I2C_TX_FIFO_DEPTH        16U
//降到半满时批量补充,给高优先级ISR留余量
#define I2C_TX_FIFO_REFILL_LEVEL 8U

//把现实时间换算成CPU轮询次数；只作为总线异常时的退出兜底。
//因为I2C对接的是UI任务,对于实时性要求没这么高,就不需要精准计算
static Uint32 I2C_WaitLoopsFromUs(Uint16 timeoutUs)
{
    Uint32 loops = (Uint32)timeoutUs * I2C_WAIT_LOOPS_PER_US;
    //保底返回1
    return (loops == 0UL) ? 1UL : loops;
}

//这些状态位是锁存型标志，必须由软件写1清除。
//一定要注意,是写1清除这些标志!
static void I2C_ClearStatusFlags(void)
{
    //仲裁失败标志位,多主机竞争 I2C 总线时，本机抢总线失败
    I2caRegs.I2CSTR.bit.ARBL = 1U;
    //无确认中断标志位,发送后没有收到接收方的 ACK 应答
    I2caRegs.I2CSTR.bit.NACK = 1U;
    //寄存器可访问就绪标志位,I2C 模块准备好，CPU 可以访问它的寄存器
    I2caRegs.I2CSTR.bit.ARDY = 1U;
    //检测到停止条件位, 总线上出现了 I2C STOP 条件,说明本次传输完成
    I2caRegs.I2CSTR.bit.SCD  = 1U;
}

//读取错误状态并映射为bsp.h中的公共错误码。
//这里主要探查无应答错误和竞争失败错误
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

//每个事务开始前清空 TX FIFO，防止上次异常残留的数据混进本帧。
static void I2C_ResetTxFifo(void)
{
    //TxFIFO复位
    I2caRegs.I2CFFTX.bit.TXFFRST = 0U;
    //清除TxFIFO中断标志位
    I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1U;
    //释放TxFIFO复位,启动
    I2caRegs.I2CFFTX.bit.TXFFRST = 1U;
}

//上报IIC警告
static void I2C_ReportFrameDrop(Uint16 status, Uint16 frameActive)
{
    //报丢帧警告
    if((frameActive != 0U) && (status != I2C_STATUS_OK))
    {
        gSysProblem.warning |= WARNING_I2C_FRAME_DROPPED;
    }
}

//出错统一收尾：请求STOP、清空FIFO并清除锁存状态。
static Uint16 I2C_FinishWithError(Uint16 status, Uint16 frameActive)
{
    //告诉IIC模块:你发送完成后不要有其他动作,发送一个STOP就行,把总线释放了
    I2caRegs.I2CMDR.bit.STP = 1U;
    //清空FIFO数据
    I2C_ResetTxFifo();
    //清除锁存寄存器
    I2C_ClearStatusFlags();
    //报丢帧警告
    I2C_ReportFrameDrop(status, frameActive);
    return status;
}

//每次轮询同时检查总线错误和软件超时。 
static Uint16 I2C_WaitPoll(Uint32 *waitLoops, Uint16 frameActive)
{
    //先获取传输过程当中的错误
    Uint16 status = I2C_GetErrorStatus();
    //如果没有问题,正常传输
    if(status != I2C_STATUS_OK)
    {
        //如实返回OK,然后收尾
        return I2C_FinishWithError(status, frameActive);
    }
    if(--(*waitLoops) == 0UL)
    {
        //如实返回超时问题,然后收尾
        return I2C_FinishWithError(I2C_STATUS_TIMEOUT, frameActive);
    }
    return I2C_STATUS_OK;
}

/* I2CA初始化：400KHz SCL、主机模式、FIFO使能、纯轮询。 */
void I2C_Config(void)
{
    EALLOW;

    //IIC软复位中...🐌
    I2caRegs.I2CMDR.bit.IRS = 0U;
    //等待1ms
    DELAY_US(1000);

    //直接清零并禁用TxFIFO
    I2caRegs.I2CFFTX.all = 0x0000;
    //清零RxFIFO,并清除中断标志位
    I2caRegs.I2CFFRX.all = 0x0040;

    //20分频,10MHz
    I2caRegs.I2CPSC.bit.IPSC = I2C_PRESCALER;

    //配置逻辑高位和逻辑低位时间
    I2caRegs.I2CCLKL = I2C_CLOCK_LOW;
    I2caRegs.I2CCLKH = I2C_CLOCK_HIGH;

    //本机地址,由于我们是主机,所以随便写
    I2caRegs.I2COAR.bit.OAR = 0x0000;
    //从机地址,由于我们有俩从机,所以到时候还会重新赋值,也随便写
    I2caRegs.I2CSAR.bit.SAR = 0x0000;
    
    //清除所有锁存状态标志
    I2C_ClearStatusFlags();
    //不使用I2C模块中断；但之后会开FIFO中断
    I2caRegs.I2CIER.all = 0x0000;
    //清零IIC配置寄存器
    I2caRegs.I2CMDR.all = 0x0000;
    //和之前的代码呼应,现在正式启用IIC模块🚀
    I2caRegs.I2CMDR.bit.IRS = 1U;
    //软件调试时继续跑
    I2caRegs.I2CMDR.bit.FREE = 1U;

    //配置TxFIFO
    I2caRegs.I2CFFTX.all = 0x0000;
    //清除中断标志位
    I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1U;
    //释放TxFIFO复位
    I2caRegs.I2CFFTX.bit.TXFFRST = 1U;
    //使能TxFIFO
    I2caRegs.I2CFFTX.bit.I2CFFEN = 1U;

    //配置RxFIFO
    I2caRegs.I2CFFRX.all = 0x0000;
    //清除中断标志位
    I2caRegs.I2CFFRX.bit.RXFFINTCLR = 1U;
    //RxFIFO复位
    I2caRegs.I2CFFRX.bit.RXFFRST = 1U;

    //也就是说也不用FIFO中断,慢速任务,一个个发就行

    EDIS;
}

/* I2C轮询式主机发送：
 *   1. 等待总线空闲并配置一次完整事务；
 *   2. START前预装最多16字节；
 *   3. FIFO降至半满时按空余槽位批量补充；
 *   4. 等待FIFO、移位器和总线全部空闲后返回。*/
Uint16 I2C_MasterTransfer
(
    //从机地址
    Uint16 slaveAddr7,
    //待发送数据首地址          
    const Uint16 *data, 
    //发送数据长度,写零就是Probe模式
    Uint16 length,     
     //设定超时时间         
    Uint16 timeoutUs   
)
{
    //记录这一帧已传字节数
    Uint16 index;
    Uint16 status;
    //当前FIFO已经用了多少
    Uint16 fifoUsed;
    //当前FIFO还剩多少
    Uint16 fifoFree;
    Uint32 waitLoops;

    //非法地址,非法数据,非法长度,非法设定时间就返回错误
    if(slaveAddr7 > 0x7FU)
    {
        return I2C_STATUS_BAD_PARAMETER;
    } 
    if((length > 0U) && (data == 0))
    {
        return I2C_STATUS_BAD_PARAMETER;
    }
    if(timeoutUs == 0U)
    {
        timeoutUs = I2C_DEFAULT_TIMEOUT_US;
    }
        
    //粗算"我大概需要多少次循环就到达了超时时间🧐"
    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    //一般的代码看到忙线就直接掠过去了
    //但是对于IIC任务来说,是要计算超时任务的,所以不介意多等一会儿,等到你总线释放了出来
    //但是不代表这段代码就不需要优化了,后续要是发现真的超时的话就需要加定时器计数了
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        if(waitLoops == 0UL)
        {
            I2C_ReportFrameDrop(I2C_STATUS_BUS_BUSY, (length != 0U) ? 1U : 0U);
            return I2C_STATUS_BUS_BUSY;
        }
        waitLoops--;
    }

    //清除锁存状态标志位
    I2C_ClearStatusFlags();
    //设置从机地址
    I2caRegs.I2CSAR.bit.SAR = slaveAddr7;
    //设置待发送数据长度,发完自动发STOP标志位
    I2caRegs.I2CCNT = length;

    //配置I2C控制寄存器
    I2caRegs.I2CMDR.all = 0x0000;
    //释放复位,启用IIC
    I2caRegs.I2CMDR.bit.IRS = 1U;
    //设置为发送模式
    I2caRegs.I2CMDR.bit.TRX = 1U;
    //设置本芯片为主机
    I2caRegs.I2CMDR.bit.MST = 1U;
    //软件调试不停止
    I2caRegs.I2CMDR.bit.FREE = 1U;
    //FIFO没有数据了就发出stop标志位
    I2caRegs.I2CMDR.bit.STP = 1U;

    //每次事务先清空FIFO，避免上次异常遗留的数据混入本帧
    I2C_ResetTxFifo();

    // 预填（length==0 时不执行）
    index = 0U;
    //把数据填充至发送FIFO,这里暗示了我们一次最多传16个16位数据
    //一直填充,直到填完或者填满
    while((index < length) && (index < I2C_TX_FIFO_DEPTH))
    {
        I2caRegs.I2CDXR.bit.DATA = data[index];
        index++;
    }
    //给总线一个START标志位,代表我已经开始发送了
    I2caRegs.I2CMDR.bit.STT = 1U;


    // 续填（length==0 时不执行）
    //下面的代码是应对(index < I2C_TX_FIFO_DEPTH)的情况,也就是之前的数据没全放到FIFO的情况
    while(index < length)
    {
        //每轮续填前，重新算一次等待上限，作为本轮的超时预算
        waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
        //等待直到FIFO空间足够,超过8字节
        while(I2caRegs.I2CFFTX.bit.TXFFST > I2C_TX_FIFO_REFILL_LEVEL)
        {
            //检查这几次传输有没有问题
            status = I2C_WaitPoll(&waitLoops, (length != 0U) ? 1U : 0U);
            if(status != I2C_STATUS_OK)
            {
                return status;
            }
        }
        //检查本次传输是否出现错误
        status = I2C_GetErrorStatus();
        if(status != I2C_STATUS_OK)
        {
            //有错误就收尾
            return I2C_FinishWithError(status, (length != 0U) ? 1U : 0U);
        }

        //统计当前FIFO使用情况
        fifoUsed = I2caRegs.I2CFFTX.bit.TXFFST;
        fifoFree = I2C_TX_FIFO_DEPTH - fifoUsed;
        while((fifoFree > 0U) && (index < length))
        {
            //空间够的话就是一直填充,直到没有空位或者数据发完
            //下次如果再发送的话就得等到FIFO还剩下8字节了
            I2caRegs.I2CDXR.bit.DATA = data[index];
            index++;
            fifoFree--;
        }
    }

    // //所有数据都放到FIFO了之后,再等一会,等最后的几个字节发送完
    //（length==0 时前两个条件天然成立，只剩 BB==0）
    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    //一直等到:
    while
    (
        ////FIFO彻底空了
        (I2caRegs.I2CFFTX.bit.TXFFST != 0U) ||
        //移位寄存器也空了
        (I2caRegs.I2CSTR.bit.XSMT == 0U) ||
        //总线都不再繁忙,说明STOP都发出
        (I2caRegs.I2CSTR.bit.BB != 0U)
    )
    //以上条件都满足,说明这一帧彻底发送成功
    {
        status = I2C_WaitPoll(&waitLoops, (length != 0U) ? 1U : 0U);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }
    //收尾前再检查一次错误状态,此时最重要的就是检查从机是否发送ACK被主机接收到
    status = I2C_GetErrorStatus();
    if(status != I2C_STATUS_OK)
    {
        //错误自有错误的收尾方法
        return I2C_FinishWithError(status, (length != 0U) ? 1U : 0U);
    }
    //清除标志位,收尾
    I2C_ClearStatusFlags();
    return I2C_STATUS_OK;
}

//IIC读取函数
Uint16 I2C_MasterRead
(
    //从机地址
    Uint16 slaveAddr7,  
    //Rx Buffer
    Uint16 *data, 
    //要读取的字节数
    Uint16 length,    
    //超时时间设置
    Uint16 timeoutUs
)
{
    Uint16 index;
    Uint16 status;
    Uint32 waitLoops;
    //检查参数是否正确
    if((data == 0) || (length == 0U) || (slaveAddr7 > 0x7FU))
    {
        return I2C_STATUS_BAD_PARAMETER;
    }
    if(timeoutUs == 0U)
    {
        timeoutUs = I2C_DEFAULT_TIMEOUT_US;
    }
    //等待,直到到母线空闲
    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        if(waitLoops == 0UL)
        {
            I2C_ReportFrameDrop(I2C_STATUS_BUS_BUSY, 1U);
            return I2C_STATUS_BUS_BUSY;
        }
        waitLoops--;
    }
    //清除之前状态标志
    I2C_ClearStatusFlags();
    //写入从机地址
    I2caRegs.I2CSAR.bit.SAR = slaveAddr7;
    //告诉硬件本次要接收多少字节。读满这么多字节后，硬件按 STP 设置自动发 STOP
    I2caRegs.I2CCNT = length;

    //配置IIC
    I2caRegs.I2CMDR.all = 0x0000;
    I2caRegs.I2CMDR.bit.IRS = 1U;
    //设为接收模式
    I2caRegs.I2CMDR.bit.TRX = 0U;
    I2caRegs.I2CMDR.bit.MST = 1U;
    I2caRegs.I2CMDR.bit.FREE = 1U;
    //收满 I2CCNT 字节后自动发 STOP，结束本帧
    I2caRegs.I2CMDR.bit.STP = 1U;

    //复位Rx FIFO,停止活动
    I2caRegs.I2CFFRX.bit.RXFFRST = 0U;
    //清除中断标志位
    I2caRegs.I2CFFRX.bit.RXFFINTCLR = 1U;
    //使能Rx FIFO,启动
    I2caRegs.I2CFFRX.bit.RXFFRST = 1U;
    //发出 START 条件，开始本次读传输,硬件会先发从机地址 + 读方向位（R/W=1），等从机 ACK 后开始收数据
    //具体参见数据手册
    I2caRegs.I2CMDR.bit.STT = 1U;
    //逐字节读取
    for(index = 0U; index < length; index++)
    {
        //注意,这里的超时是直接放在循环内部的,说明接收会比发送慢
        waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
        //等待,直到Rx FIFO里面有几个字节的数据,且硬件表示自己:"接收数据完成"
        while((I2caRegs.I2CFFRX.bit.RXFFST == 0U) && (I2caRegs.I2CSTR.bit.RRDY == 0U))
        {
            //期间也不断检查超时错误和其他错误
            status = I2C_WaitPoll(&waitLoops, 1U);
            if(status != I2C_STATUS_OK)
            {
                return status;
            }
        }
        //将数据存入Buffer里面
        data[index] = (Uint16)I2caRegs.I2CDRR.bit.DATA;
    }
    //等待最后一个数据都读取完毕,进入收尾状态
    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        status = I2C_WaitPoll(&waitLoops, 1U);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }
    //收尾
    status = I2C_GetErrorStatus();
    I2C_ClearStatusFlags();
    I2C_ReportFrameDrop(status, 1U);
    return status;
}

//这个函数同时具备读写功能,先写再读,一般只在EEPROM.c里面用到,是比较上层的函数
//它没有一处是死等的,和上两个函数相反
Uint16 I2C_MasterWriteRead
(
    //从机地址
    Uint16 slaveAddr7,
    //待写数据buffer
    const Uint16 *writeData,
    //写入字节数
    Uint16 writeLength,
    //读数据buffer
    Uint16 *readData,
    //读取的字节数
    Uint16 readLength,
    //超时时间
    Uint16 timeoutUs
)
{
    Uint16 index;
    Uint16 status;
    Uint32 waitLoops;
    //错误参数识别
    if
    (
        (writeData == 0) || 
        (readData == 0) ||
        (writeLength == 0U) || 
        (readLength == 0U) ||
        (writeLength > I2C_TX_FIFO_DEPTH) || 
        (slaveAddr7 > 0x7FU)
    )
    {
        return I2C_STATUS_BAD_PARAMETER;
    }
    if(timeoutUs == 0U)
    {
        timeoutUs = I2C_DEFAULT_TIMEOUT_US;
    }

    //等待总线空闲
    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        if(waitLoops == 0UL)
        {
            I2C_ReportFrameDrop(I2C_STATUS_BUS_BUSY, 1U);
            return I2C_STATUS_BUS_BUSY;
        }
        waitLoops--;
    }
    //清除残留标志位
    I2C_ClearStatusFlags();
    //设置从机地址
    I2caRegs.I2CSAR.bit.SAR = slaveAddr7;
    //设置写数据长度
    I2caRegs.I2CCNT = writeLength;
    //配置IIC控制寄存器
    I2caRegs.I2CMDR.all = 0x0000;
    //释放模块复位,启动IIC
    I2caRegs.I2CMDR.bit.IRS = 1U;
    //先设置为发送模式
    I2caRegs.I2CMDR.bit.TRX = 1U;
    I2caRegs.I2CMDR.bit.MST = 1U;
    I2caRegs.I2CMDR.bit.FREE = 1U;
    //Tx FIFO复位
    I2caRegs.I2CFFTX.bit.TXFFRST = 0U;
    //清除Tx FIFO中断志位
    I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1U;
    //释放Tx FIFO复位,启动
    I2caRegs.I2CFFTX.bit.TXFFRST = 1U;
    //逐字节填充Tx FIFO
    for(index = 0U; index < writeLength; index++)
    {
        I2caRegs.I2CDXR.bit.DATA = writeData[index];
    }
    //填充完毕,给总线发送START,开始传输
    I2caRegs.I2CMDR.bit.STT = 1U;
    //等待传输完毕
    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    //注意,这里不用BB判断是否发送完毕,是因为写完了之后还要读,所以用ARDY判断
    //ARDY变高表示从机已应答地址，且Tx FIFO 已准备好接收新命令
    while(I2caRegs.I2CSTR.bit.ARDY == 0U)
    {
        status = I2C_WaitPoll(&waitLoops, 1U);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }
    //上面的while跳出条件本就是ARDY = 1U, 为啥这里再赋值一次呢?
    //因为我说过,ARDY他是锁存型bit,也就是W1C型变量
    //其S端连在外部硬件,R端连在内部CPU
    I2caRegs.I2CSTR.bit.ARDY = 1U;

    
    //待读取数据长度
    I2caRegs.I2CCNT = readLength;
    //IIC接收模式
    I2caRegs.I2CMDR.bit.TRX = 0U;
    //读取完成发一个STOP给总线,代表已经接收完成
    I2caRegs.I2CMDR.bit.STP = 1U;
    //Rx FIFO复位
    I2caRegs.I2CFFRX.bit.RXFFRST = 0U;
    //清除Rx FIFO中断标志位
    I2caRegs.I2CFFRX.bit.RXFFINTCLR = 1U;
    //释放Rx FIFO复位,启动
    I2caRegs.I2CFFRX.bit.RXFFRST = 1U;
    //正式接收
    I2caRegs.I2CMDR.bit.STT = 1U;
    //循环读取数据
    for(index = 0U; index < readLength; index++)
    {
        waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
        //一直等,等到有数据,且IIC表示"可以接收数据"
        while((I2caRegs.I2CFFRX.bit.RXFFST == 0U) && (I2caRegs.I2CSTR.bit.RRDY == 0U))
        {
            status = I2C_WaitPoll(&waitLoops, 1U);
            if(status != I2C_STATUS_OK)
            {
                return status;
            }
        }
        //放置到buffer里面
        readData[index] = (Uint16)I2caRegs.I2CDRR.bit.DATA;
    }

    //这个时候才等总线空闲,总线空闲代表着这一次的读写彻底完成,进行收尾工作
    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        status = I2C_WaitPoll(&waitLoops, 1U);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }
    //结束前,最后检查一次传输错误
    status = I2C_GetErrorStatus();
    //收尾
    I2C_ClearStatusFlags();
    I2C_ReportFrameDrop(status, 1U);
    return status;
}
