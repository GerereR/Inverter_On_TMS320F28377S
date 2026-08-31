#include "F28x_Project.h"
#include "bsp.h"
#include "variable.h"
#include "scheduler.h"

// ECAP计数器直接使用200 MHz系统时钟。
#define ECAP_CLOCK_HZ  200000000.0f
#define ECAP_CYCLE_FREQ_MIN_HZ  45.0f
#define ECAP_CYCLE_FREQ_MAX_HZ  70.0f

static volatile Uint32 ECAP_PeriodTicks = 0;
static volatile float ECAP_FreqHz = 0.0f;
static Uint16 ECAP_CapturePrimed = 0;

void ECAP_Config(void)
{
    EALLOW;

    PieVectTable.ECAP1_INT = &ECAP1_BSP_ISR;// 将ECAP1硬件中断连接到ECAP1_BSP_ISR中断服务函数。

    ECap1Regs.ECEINT.all = 0;//关闭 ECAP1内部的全部中断源。还没有配置完,不能中断
    ECap1Regs.ECCLR.all = 0xFFFF;//清除 ECAP1内部所有残留标志
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;//暂时禁止捕获寄存器装载
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;//停止 ECAP1的时间戳计数器  0:停止计数 1:运行计数
    ECap1Regs.TSCTR = 0;//把向上计数间戳计数器清零
    ECap1Regs.CTRPHS = 0;//把 ECAP 相位寄存器清零
    ECAP_CapturePrimed = 0;

    ECap1Regs.ECCTL2.bit.CAP_APWM = 0;//捕获外部信号边沿
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0;//选择连续捕获模式
    ECap1Regs.ECCTL2.bit.STOP_WRAP = 0;//回绕模式
    ECap1Regs.ECCTL1.bit.CAP1POL = 1;//将捕获事件 1配置为下降沿,下降沿触发事件1
    ECap1Regs.ECCTL1.bit.CTRRST1 = 1;// 每个下降沿发生后自动将TSCTR清零，因此后续CAP1就是一个输入周期的计数值。    
    ECap1Regs.ECCTL1.bit.PRESCALE = 0;// 不进行输入边沿预分频，每个下降沿都参与测量。
    ECap1Regs.ECCTL1.bit.FREE_SOFT = 2;// 调试暂停CPU时，ECAP计数器仍保持运行    
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;// 当前不使用ECAP同步输入和同步输出。
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = 2;//禁止 ECAP1输出同步信号。    
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;// 允许捕获事件将TSCTR值锁存到CAP1。    
    ECap1Regs.ECEINT.bit.CEVT1 = 1;// 每个下降沿触发一次CEVT1中断，实现每个输入周期更新频率。  
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1; // 启动ECAP时间戳计数器。
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;// 打开PIE第4组第1路以及CPU第4组中断。
    IER |= M_INT4;

    EDIS;
}

__interrupt void ECAP1_BSP_ISR(void)
{
    Uint32 periodTicks = ECap1Regs.CAP1;

    // 第一个下降沿只建立计时参考，不能代表完整输入周期。
    if(ECAP_CapturePrimed == 0U)
    {
        ECAP_CapturePrimed = 1U;
        /* The first edge discards startup data and starts an aligned block. */
        DMA_GridCycleBoundary();
    }
    else
    {
        // 后续每个下降沿对应一个完整周期，保存周期计数并计算频率。
        ECAP_PeriodTicks = periodTicks;
        if(periodTicks != 0UL)
        {
            ECAP_FreqHz = ECAP_CLOCK_HZ / (float)periodTicks;
            /* Ignore noise edges that cannot represent a valid grid cycle. */
            if((ECAP_FreqHz >= ECAP_CYCLE_FREQ_MIN_HZ) &&
               (ECAP_FreqHz <= ECAP_CYCLE_FREQ_MAX_HZ))
            {
                gMachineData.ecapFreqCent = (Uint16)(ECAP_FreqHz * 100.0f);
                DMA_GridCycleBoundary();
                Scheduler_NotifyGridZeroCross();
            }
            else
            {
                gMachineData.ecapFreqCent = 0U;
            }
        }
    }

    // 清除ECAP事件、中断和PIE标志，允许下一次捕获继续进入ISR。
    ECap1Regs.ECCLR.bit.CEVT1 = 1;
    ECap1Regs.ECCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
