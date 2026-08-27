#ifndef BSP_H_
#define BSP_H_

#include "F28x_Project.h"
#include "variable.h"

/* ADC trigger selections from the TMS320F28377S ADC trigger table. */
#define ADC_TRIGGER_CPU_TIMER1      2U
#define ADC_TRIGGER_EPWM1_SOCA      5U

//其实快环的这些数据早在电流环那块就被用了,那些快环数据当场读取当场处理了,
//但是再怎么说逆变器还是需要告诉别人当前电压电流大小的,所以就以非常慢的速度更新了,
//而PV电压电流等数据是需要参与到未来的控制的,所以才160个大小,快环反而400个大小
#define ADC_ACQUISITION_WINDOW     14U
#define ADC_FAST_BLOCK_BURSTS     400U
#define ADC_PV_BLOCK_BURSTS       160U
#define ADC_SLOW_BLOCK_BURSTS      16U

/* DMA block groups reported to Task_Measure after buffer processing. */
#define DMA_UPDATE_FAST          0x0001U
#define DMA_UPDATE_PV_CURRENT    0x0002U
#define DMA_UPDATE_PV_VOLTAGE    0x0004U
#define DMA_UPDATE_ISOLATION     0x0008U
#define DMA_UPDATE_TEMP          0x0010U
#define DMA_UPDATE_ALL           0x001FU

/* CPU Timer1 supplies the independent 100 Hz slow ADC hardware trigger. */
#define ADC_SLOW_TRIGGER_COUNTS  2000000UL

void GPIO_Config(void);

/* Board digital inputs. These macros expose logical states, not GPIO numbers. */
#define POWER_SW1()        ((GpioDataRegs.GPCDAT.bit.GPIO82 == 0U) ? 1U : 0U)
#define POWER_SW2()        ((GpioDataRegs.GPCDAT.bit.GPIO80 == 0U) ? 1U : 0U)
#define POWER_SW3()        ((GpioDataRegs.GPCDAT.bit.GPIO78 == 0U) ? 1U : 0U)

/* Active-low user keys. GPIO numbers are intentionally hidden in gpio.c. */
#define KEY_NUMBER_1       1U
#define KEY_NUMBER_2       2U
#define KEY_NUMBER_3       3U
#define KEY_NUMBER_4       4U
#define KEY_EVENT_1        0x0001U
#define KEY_EVENT_2        0x0002U
#define KEY_EVENT_3        0x0004U
#define KEY_EVENT_4        0x0008U

/* Return and clear newly confirmed key-press events after software debounce. */
Uint16 GPIO_GetKeyEvents(void);

/* LEDs are common-anode: a low output turns an LED on. */
#define LED1_ON()          (GpioDataRegs.GPACLEAR.bit.GPIO31 = 1U)
#define LED1_OFF()         (GpioDataRegs.GPASET.bit.GPIO31 = 1U)
#define LED1_TOGGLE()      (GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1U)
#define LED2_ON()          (GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1U)
#define LED2_OFF()         (GpioDataRegs.GPBSET.bit.GPIO34 = 1U)
#define LED2_TOGGLE()      (GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1U)

/* Active-low relay and gate-driver controls. */
#define ISO_RELAY1_ON()    (GpioDataRegs.GPCCLEAR.bit.GPIO91 = 1U)
#define ISO_RELAY1_OFF()   (GpioDataRegs.GPCSET.bit.GPIO91 = 1U)
#define ISO_RELAY2_ON()    (GpioDataRegs.GPCCLEAR.bit.GPIO93 = 1U)
#define ISO_RELAY2_OFF()   (GpioDataRegs.GPCSET.bit.GPIO93 = 1U)

#define GRID_RELAY1_ON()   (GpioDataRegs.GPCCLEAR.bit.GPIO90 = 1U)
#define GRID_RELAY1_OFF()  (GpioDataRegs.GPCSET.bit.GPIO90 = 1U)
#define GRID_RELAY2_ON()   (GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1U)
#define GRID_RELAY2_OFF()  (GpioDataRegs.GPCSET.bit.GPIO92 = 1U)
#define GRID_RELAY3_ON()   (GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1U)
#define GRID_RELAY3_OFF()  (GpioDataRegs.GPCSET.bit.GPIO94 = 1U)
#define GRID_RELAY4_ON()   (GpioDataRegs.GPACLEAR.bit.GPIO10 = 1U)
#define GRID_RELAY4_OFF()  (GpioDataRegs.GPASET.bit.GPIO10 = 1U)
#define GRID_RELAY_ALL_OFF()   (GpioDataRegs.GPACLEAR.bit.GPIO11 = 1U)
#define GRID_RELAY_ALL_ON()    (GpioDataRegs.GPASET.bit.GPIO11 = 1U)

#define INVERTER_OFF()     (GpioDataRegs.GPCCLEAR.bit.GPIO89 = 1U)
#define INVERTER_ON()      (GpioDataRegs.GPCSET.bit.GPIO89 = 1U)

#define BOOST_OFF()        (GpioDataRegs.GPCCLEAR.bit.GPIO88 = 1U)
#define BOOST_ON()         (GpioDataRegs.GPCSET.bit.GPIO88 = 1U)

#define GFCI_CHECK_ON()    (GpioDataRegs.GPACLEAR.bit.GPIO19 = 1U)
#define GFCI_CHECK_OFF()   (GpioDataRegs.GPASET.bit.GPIO19 = 1U)

#define DSP_STATE_LOW()    (GpioDataRegs.GPACLEAR.bit.GPIO21 = 1U)
#define DSP_STATE_HIGH()   (GpioDataRegs.GPASET.bit.GPIO21 = 1U)

/* Passive buzzer on GPIO8/EPWM5A. The tone frequency is configurable. */
void BEEP_SetFrequency(Uint16 frequencyHz);
#define BEEP_ON()  (EPwm5Regs.AQCSFRC.bit.CSFA = 0U)
#define BEEP_OFF() (EPwm5Regs.AQCSFRC.bit.CSFA = 1U)

void EPWM_Config(void);
void EPWM_Start(void);
void EPWM_Enable(void);
void EPWM_TripZoneForce(void);
void EPWM_TripZoneClear(void);
void EPWM_Disable(void);
void EPWM_SetDuty(float duty);
void EPWM_SetBoostDuty(float boost1Duty, float boost2Duty);

void ADC_Config(void);
void DMA_Config(void);
Uint16 DMA_ProcessCompletedBlocks(ADC_RawData *rawInstant,
                                  ADC_RawData *rawAvg,
                                  ADC_RawMeanSqData *rawMeanSq,
                                  const ADC_Calibrate *cal);
void ECAP_Config(void);
void SCI_Config(void);
void I2C_Config(void);

/* Polled I2C master status used by the OLED bring-up path. */
/* I2C 轮询主机的公共状态码(i2c.c 的 I2C_MasterWrite 返回):
 *   OK               —— 传输完成、总线已释放
 *   BAD_PARAMETER    —— 空指针/零长度/地址越界
 *   BUS_BUSY         —— 等待总线空闲超时(总线上有异常 START 未结束)
 *   TIMEOUT          —— 等待 XRDY/XSMT/BB 超时(从机卡死或无响应)
 *   NACK             —— 从机对地址或数据无应答(最常见:OLED 没接好/地址错)
 *   ARBITRATION_LOST —— 仲裁丢失(本设计是单主机,实际几乎不会出现)  */
#define I2C_STATUS_OK              0U
#define I2C_STATUS_BAD_PARAMETER   1U
#define I2C_STATUS_BUS_BUSY        2U
#define I2C_STATUS_TIMEOUT         3U
#define I2C_STATUS_NACK            4U
#define I2C_STATUS_ARBITRATION_LOST 5U

/* I2C 轮询式主机发送:向 7 位地址从机写 length 字节,阻塞至完成/超时/出错。
 * timeoutUs=0 表示用默认超时(2ms)。返回上面的 I2C_STATUS_xxx。      */
Uint16 I2C_MasterWrite(Uint16 slaveAddr7, const unsigned char *data, Uint16 length, Uint16 timeoutUs);
Uint16 I2C_MasterRead(Uint16 slaveAddr7, unsigned char *data, Uint16 length, Uint16 timeoutUs);
Uint16 I2C_MasterProbe(Uint16 slaveAddr7, Uint16 timeoutUs);
Uint16 I2C_MasterWriteRead
(
    Uint16 slaveAddr7,
    const unsigned char *writeData,
    Uint16 writeLength,
    unsigned char *readData,
    Uint16 readLength,
    Uint16 timeoutUs
);

void SCI_SendByte(Uint16 data);
void SCI_SendString(const char *text);
Uint16 SCI_ReadByte(Uint16 *data);

extern volatile float InductorCurrentAmp_temporal;
extern volatile Uint16 SCI_RxDataPending;

__interrupt void ECAP1_BSP_ISR(void);
__interrupt void SCIB_BSP_RX_ISR(void);
__interrupt void ADCA1_CPU_ISR(void);
__interrupt void DMA_CH1_CPU_ISR(void);
__interrupt void DMA_CH2_CPU_ISR(void);
__interrupt void DMA_CH3_CPU_ISR(void);
__interrupt void DMA_CH4_CPU_ISR(void);
__interrupt void DMA_CH5_CPU_ISR(void);
__interrupt void EPWM1_TZ_BSP_ISR(void);
__interrupt void EPWM3_TZ_BSP_ISR(void);
__interrupt void EPWM4_TZ_BSP_ISR(void);

#endif
