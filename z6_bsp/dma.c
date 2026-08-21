#include "F28x_Project.h"
#include "bsp.h"
#include "variable.h"

// DMA stores ADCRESULT0/ADCRESULT1 pairs and interrupts after one block.
volatile Uint16 ADC_DMA_Buffer[ADC_DMA_BUFFER_WORDS];//定义 DMA 目标缓冲区
volatile Uint16 ADC_DMA_FrameReady = 0U;//DMA 数据块完成标志
volatile Uint32 ADC_DMA_BlockCount = 0UL;//记录 DMA 完成了多少个数据块。
volatile Uint16 ADC_DMA_LastGridAverage = 0U;//保存最近一个 DMA 数据块中，电网电压 ADC 原始码的平均值。
volatile Uint16 ADC_DMA_LastInductorCurrentAverage = 0U;//保存最近一个 DMA 数据块中，电感电流 ADC 原始码的平均值。

// Each array element matches one DMA burst: PV voltage followed by PV current.
volatile ADC_PV_DMA_Burst ADC_PV_DMA_Buffer[ADC_PV_DMA_BURST_COUNT];
volatile Uint16 ADC_PV_DMA_BlockReady = 0U;
volatile Uint32 ADC_PV_DMA_BlockCount = 0UL;

void DMA_Config(void)
{
    EALLOW;

    // Reset DMA and let transfers continue while the CPU is halted by a debugger.
    DmaRegs.DMACTRL.bit.HARDRESET = 1U;//对 DMA 模块执行硬复位
    __asm(" NOP");//文档要求至少一个NOP
    DmaRegs.DEBUGCTRL.bit.FREE = 1U;//设置调试运行模式。

    // ADCINT2 is the DMA trigger; ADCINT1 remains dedicated to the control ISR.
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = DMA_ADCAINT2;//选择 DMA 通道 1 的触发源为 ADCINT2

    // Each ADCINT2 trigger copies two 16-bit results: voltage, then current.
    DmaRegs.CH1.MODE.bit.PERINTSEL = 1U;//设置 DMA 通道 1 使用自身的外设触发选择方式
    DmaRegs.CH1.MODE.bit.PERINTE = 1U;//使能外设触发
    DmaRegs.CH1.MODE.bit.ONESHOT = 0U;//关闭单次模式
    DmaRegs.CH1.MODE.bit.CONTINUOUS = 1U;//开启连续模式
    DmaRegs.CH1.MODE.bit.OVRINTE = 0U;//关闭 DMA 溢出中断
    DmaRegs.CH1.MODE.bit.DATASIZE = 0U;//设置数据宽度为 16 位
    DmaRegs.CH1.MODE.bit.CHINTMODE = 1U;//设置 DMA 通道中断在传输结束时产生
    DmaRegs.CH1.MODE.bit.CHINTE = 1U;//使能 DMA 通道中断

    DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32)&AdcaResultRegs.ADCRESULT0;//重装载起始地址
    DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32)&AdcaResultRegs.ADCRESULT0;//第一次装载起始地址
    DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32)ADC_DMA_Buffer;//数据从 ADC_DMA_Buffer[0] 开始写
    DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)ADC_DMA_Buffer;//第一次传输从 ADC_DMA_Buffer[0] 开始

    DmaRegs.CH1.BURST_SIZE.all = 1U;//设置一次 burst 搬运 2 个字
    DmaRegs.CH1.SRC_BURST_STEP = 1;//burst 内源地址每次增加 1
    DmaRegs.CH1.DST_BURST_STEP = 1;//burst 内目标地址每次增加 1
    DmaRegs.CH1.TRANSFER_SIZE = ADC_DMA_FRAME_COUNT - 1U;//设置一次完整 DMA 传输包含多少个 burst
    DmaRegs.CH1.SRC_TRANSFER_STEP = -1;//高级功能,适配于一个数组存多个burst
    DmaRegs.CH1.DST_TRANSFER_STEP = 1;//

    // Disable wrapping; continuous mode reloads the shadow addresses per block.
    DmaRegs.CH1.SRC_WRAP_SIZE = 0xFFFFU;//设置源地址回绕长度为最大值，等效于关闭源地址回绕。
    DmaRegs.CH1.SRC_WRAP_STEP = 0;//即使发生源地址回绕，也不改变源地址,某种意义上这就是保护性代码
    DmaRegs.CH1.DST_WRAP_SIZE = 0xFFFFU;//关闭目标地址回绕。
    DmaRegs.CH1.DST_WRAP_STEP = 0;//目标地址回绕步长设置为 0。
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1U;//清除 DMA 可能残留的外设触发标志。
    DmaRegs.CH1.CONTROL.bit.ERRCLR = 1U;//清除 DMA 通道错误状态。

    // ADCBINT2 triggers DMA CH2 after PV voltage/current conversions finish.
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = DMA_ADCBINT2;
    DmaRegs.CH2.MODE.bit.PERINTSEL = 2U;
    DmaRegs.CH2.MODE.bit.PERINTE = 1U;
    DmaRegs.CH2.MODE.bit.ONESHOT = 0U;
    DmaRegs.CH2.MODE.bit.CONTINUOUS = 1U;
    DmaRegs.CH2.MODE.bit.OVRINTE = 0U;
    DmaRegs.CH2.MODE.bit.DATASIZE = 0U;
    DmaRegs.CH2.MODE.bit.CHINTMODE = 1U;
    DmaRegs.CH2.MODE.bit.CHINTE = 1U;

    DmaRegs.CH2.SRC_BEG_ADDR_SHADOW = (Uint32)&AdcbResultRegs.ADCRESULT0;
    DmaRegs.CH2.SRC_ADDR_SHADOW = (Uint32)&AdcbResultRegs.ADCRESULT0;
    DmaRegs.CH2.DST_BEG_ADDR_SHADOW =
        (Uint32)&ADC_PV_DMA_Buffer[0U].pvVoltage;
    DmaRegs.CH2.DST_ADDR_SHADOW =
        (Uint32)&ADC_PV_DMA_Buffer[0U].pvVoltage;

    // BURST_SIZE and TRANSFER_SIZE both store the desired count minus one.
    DmaRegs.CH2.BURST_SIZE.all = 1U;
    DmaRegs.CH2.SRC_BURST_STEP = 1;
    DmaRegs.CH2.DST_BURST_STEP = 1;
    DmaRegs.CH2.TRANSFER_SIZE = ADC_PV_DMA_BURST_COUNT - 1U;
    DmaRegs.CH2.SRC_TRANSFER_STEP = -1;
    DmaRegs.CH2.DST_TRANSFER_STEP = 1;

    // Disable wrapping; continuous mode reloads the block start addresses.
    DmaRegs.CH2.SRC_WRAP_SIZE = 0xFFFFU;
    DmaRegs.CH2.SRC_WRAP_STEP = 0;
    DmaRegs.CH2.DST_WRAP_SIZE = 0xFFFFU;
    DmaRegs.CH2.DST_WRAP_STEP = 0;
    DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1U;
    DmaRegs.CH2.CONTROL.bit.ERRCLR = 1U;

    PieVectTable.DMA_CH1_INT = &DMA_CH1_CPU_ISR;
    PieVectTable.DMA_CH2_INT = &DMA_CH2_CPU_ISR;
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1U;
    PieCtrlRegs.PIEIER7.bit.INTx2 = 1U;
    IER |= M_INT7;//使能 PIE 第 7 组第 1 个中断。

    DmaRegs.CH1.CONTROL.bit.RUN = 1U;//启动 DMA 通道 1
    DmaRegs.CH2.CONTROL.bit.RUN = 1U;//启动 DMA 通道 2

    EDIS;
}

__interrupt void DMA_CH1_CPU_ISR(void)
{
    Uint16 frameIndex;//定义数据帧索引
    Uint32 gridSum = 0UL;//保存 16 个电网电压电流采样值的总和
    Uint32 inductorCurrentSum = 0UL;

    // Consume the completed block before DMA reloads the destination buffer.
    for(frameIndex = 0U; frameIndex < ADC_DMA_FRAME_COUNT; frameIndex++)
    {
        gridSum += ADC_DMA_Buffer[2U * frameIndex];
        inductorCurrentSum += ADC_DMA_Buffer[2U * frameIndex + 1U];
    }

    ADC_DMA_LastGridAverage = (Uint16)(gridSum / ADC_DMA_FRAME_COUNT);
    ADC_DMA_LastInductorCurrentAverage =
        (Uint16)(inductorCurrentSum / ADC_DMA_FRAME_COUNT);
    gMachineData.gridVoltage = ADC_DMA_LastGridAverage;
    gMachineData.inductorCurrent = ADC_DMA_LastInductorCurrentAverage;
    ADC_DMA_BlockCount++;
    ADC_DMA_FrameReady = 1U;

    // Clear the DMA peripheral flag and release PIE group 7.
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1U;
    DmaRegs.CH1.CONTROL.bit.ERRCLR = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

__interrupt void DMA_CH2_CPU_ISR(void)
{
    Uint16 burstIndex;
    Uint32 pvVoltageSum = 0UL;
    Uint32 pvCurrentSum = 0UL;

    // Average the completed PV block before DMA starts filling it again.
    for(burstIndex = 0U; burstIndex < ADC_PV_DMA_BURST_COUNT; burstIndex++)
    {
        pvVoltageSum += ADC_PV_DMA_Buffer[burstIndex].pvVoltage;
        pvCurrentSum += ADC_PV_DMA_Buffer[burstIndex].pvCurrent;
    }
    gMachineData.pvVoltage = (Uint16)(pvVoltageSum / ADC_PV_DMA_BURST_COUNT);
    gMachineData.pvCurrent = (Uint16)(pvCurrentSum / ADC_PV_DMA_BURST_COUNT);

    ADC_PV_DMA_BlockCount++;
    ADC_PV_DMA_BlockReady = 1U;

    DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1U;
    DmaRegs.CH2.CONTROL.bit.ERRCLR = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}
