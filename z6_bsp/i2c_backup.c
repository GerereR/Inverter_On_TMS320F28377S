#if 0 /* User backup kept for reference only: not compiled. */

/* ============================================================================
 * i2c_backup.c —— 旧版 I2C 驱动备份(中断驱动版),仅作参考,不参与编译。
 *
 * 与现行版(i2c.c)的关键差异:
 *   1. 使用中断:I2CIER 开启 SCD(STOP 条件)+ ARDY(寄存器就绪)中断,
 *      并打开 PIE8.1 / CPU IER 的 INT8。
 *   2. 但 ISR 里读 I2CISRC 的动作会顺带清除 ARDY/SCD 状态位——
 *      在轮询传输中这属于"偷清标志",可能破坏正在等待的状态位。
 *      这就是现行版改为纯轮询、关闭全部中断的原因。
 *   3. I2CMDR 只写 0x0020(IRS=1)不含 FREE/STP/STT 等位,配置不如现行版完整。
 * ==========================================================================*/

#include "F28x_Project.h"
#include "bsp.h"

// 200 MHz SYSCLK -> 10 MHz I2C module clock; these dividers give 400 kHz SCL.
#define I2C_PRESCALER       19U
#define I2C_CLOCK_LOW       10U
#define I2C_CLOCK_HIGH      5U

volatile Uint16 I2C_LastInterruptSource = I2C_NO_ISRC;//保存最后一次 I2C 中断源
volatile Uint32 I2C_InterruptCount = 0UL;//中断计数器

void I2C_Config(void)
{
    EALLOW;

    // Keep the module and FIFOs reset while changing timing registers.
    I2caRegs.I2CMDR.bit.IRS = 0;//先复位模块
    DELAY_US(1000);
    I2caRegs.I2CFFTX.all = 0x0000;//关闭发送 FIFO 使能，并清除 FIFO 复位、中断标志等
    I2caRegs.I2CFFRX.all = 0x0040;//接收 FIFO 的 RXFFINTCLR 位,清除可能残留的 FIFO 中断标志。
    I2caRegs.I2CPSC.bit.IPSC = I2C_PRESCALER;//IIC时钟频率10MHz
    I2caRegs.I2CCLKL = I2C_CLOCK_LOW;//【修正】ICCLKL=SCL 低电平时间:(10+5)x100ns=1.5us
    I2caRegs.I2CCLKH = I2C_CLOCK_HIGH;//【修正】ICCLKH=SCL 高电平时间:(5+5)x100ns=1.0us
    //加起来不就是400kHz了吗
    I2caRegs.I2COAR.bit.OAR = 0x0000;   // Own address is unused in master mode.本机地址寄存器
    I2caRegs.I2CSAR.bit.SAR = 0x0000;   // Set the target address before a transfer.
    I2caRegs.I2CSTR.all = 0x0023;       //清除一下三种状态:1.检测到 STOP  2.从机没有应答  3.多主机仲裁失败
    
    I2caRegs.I2CIER.all = 0x0024;       // 开启模块级中断:1.STOP 条件中断  2.寄存器访问就绪中断
    /*Bit 5（SCD）：使能停止条件中断（当发送 STOP 后触发）。
    Bit 2（ARDY）：使能寄存器访问就绪中断（当 I2C 模块完成当前操作，准备好接收新的命令时触发）。*/

    PieVectTable.I2CA_INT = &I2CA_BSP_ISR;
    I2caRegs.I2CMDR.all = 0x0020;       // 释放 I2C 模块复位，但暂时不启动任何传输

    //FIFO设置
    I2caRegs.I2CFFTX.all = 0x6040;      // 释放发送 FIFO 复位,清除发送 FIFO 中断标志, 启用 I2C FIFO
    I2caRegs.I2CFFRX.all = 0x2040;      // 释放接收 FIFO 复位, 清除接收 FIFO 中断标志

    PieCtrlRegs.PIEIER8.bit.INTx1 = 1;  // 使能 PIE 第 8 组第 1 路(I2CA)中断
    IER |= M_INT8;                      // 使能 CPU 级 INT8

    EDIS;
}

__interrupt void I2CA_BSP_ISR(void)
{
    // Reading I2CISRC identifies and clears the current I2C interrupt source.
    I2C_LastInterruptSource = I2caRegs.I2CISRC.all;//读取中断源
    /*数值含义
    0	没有中断        1	仲裁失败
    2	收到 NACK       3	ARDY 
    4	接收就绪        5	发送就绪
    6	检测到 STOP     */

    I2C_InterruptCount++;//中断计数
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

#endif /* i2c_backup.c reference copy */
