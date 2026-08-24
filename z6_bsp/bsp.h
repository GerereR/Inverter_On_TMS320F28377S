#ifndef BSP_H_
#define BSP_H_

#include "F28x_Project.h"
#include "variable.h"

/* ADC trigger selections from the TMS320F28377S ADC trigger table. */
#define ADC_TRIGGER_CPU_TIMER1      2U
#define ADC_TRIGGER_EPWM3_SOCA      9U

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

/* LED identifiers and states used by the single LED control entry point. */
#define LED_NUMBER_1       1U
#define LED_NUMBER_2       2U
#define LED_NUMBER_3       3U
#define LED_STATE_OFF      0U
#define LED_STATE_ON       1U
#define LED_STATE_TOGGLE   2U

void LED_Ctrl(Uint16 ledNumber, Uint16 state);

void EPWM_Config(void);
void EPWM_Start(void);
void EPWM_Enable(void);
void EPWM_TripZoneForce(void);
void EPWM_TripZoneClear(void);
void EPWM_Disable(void);
void EPWM_SetDuty(float duty);

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

/* SSD1306 OLED over I2CA. Implementation lives in i2c.c to keep BSP file count low. */
/* ============= SSD1306 OLED 接口(i2c.c 实现)=============
 * 屏幕 128×64,页寻址,8 页 × 128 列;帧缓冲 + 脏页位图 + 限流刷新。 */
#define OLED_I2C_ADDR_7BIT     0x3CU   /* SSD1306 常见 7 位从地址(SA0=0)   */
#define OLED_WIDTH_COLUMNS     128U    /* 水平像素/列数                    */
#define OLED_HEIGHT_PAGES      8U      /* 垂直页数(每页 8 行像素)          */
#define OLED_LINE_CHARS        21U     /* 每行最多字符(128 列 ÷ 6 列/字符) */
#define OLED_I2C_TIMEOUT_US    3000U   /* 单条 OLED I2C 事务超时 3ms       */


 /*   - 左右镜像 → OLED_SEGMENT_REMAP 在 0xA0 / 0xA1 之间切换
 *   - 上下颠倒 → OLED_COM_SCAN      在 0xC0 / 0xC8 之间切换*/
#define OLED_SEGMENT_REMAP     0xA1U   //0xA0=正向 / 0xA1=水平镜像        
#define OLED_COM_SCAN          0xC8U   //0xC0=自上而下 / 0xC8=自下而上

Uint16 OLED_Init(void);                             /* 上电初始化命令序列 + 清软件缓冲 */
void OLED_Clear(void);                              /* 缓冲清零 + 光标归位 + 整屏标脏 */
Uint16 OLED_RefreshAll(void);                       /* 立即刷全部 8 页(阻塞)          */
Uint16 OLED_RefreshDirty(Uint16 maxPages);          /* 最多刷 maxPages 个脏页(限流)   */
void OLED_SetCursor(Uint16 column, Uint16 page);    /* 设置字符流输出光标              */
void OLED_WriteChar(char ch);                       /* 光标处写 1 字符(6 列步进)      */
void OLED_WriteString(const char *text);            /* 连续写字符串直到 '\0'          */
void OLED_WriteLine(Uint16 page, const char *text); /* 整行写(定长补空格,最多 21 字符)*/
void OLED_WriteFloat1Line(Uint16 page, const char *label, float value, const char *unit);
                                                    /* "标签 值 单位",1 位小数        */
Uint16 OLED_BringupTest(void);                      /* 开机自检画面并全量刷新         */

void SCI_SendByte(Uint16 data);
void SCI_SendString(const char *text);
Uint16 SCI_ReadByte(Uint16 *data);

extern volatile float InductorCurrentAmp_temporal;
extern volatile Uint16 SCI_RxDataPending;

__interrupt void ECAP1_BSP_ISR(void);
__interrupt void SCIA_BSP_RX_ISR(void);
__interrupt void ADCA1_CPU_ISR(void);
__interrupt void DMA_CH1_CPU_ISR(void);
__interrupt void DMA_CH2_CPU_ISR(void);
__interrupt void DMA_CH3_CPU_ISR(void);
__interrupt void DMA_CH4_CPU_ISR(void);
__interrupt void DMA_CH5_CPU_ISR(void);
__interrupt void EPWM3_TZ_BSP_ISR(void);

#endif
