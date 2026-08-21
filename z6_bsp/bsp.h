#ifndef BSP_H_
#define BSP_H_

// At 20 kSPS, 400 samples contain one complete 50 Hz input cycle.
#define ADC_SAMPLE_BUFFER_SIZE  400U

// DMA collects voltage/current pairs in a short block for lower-rate CPU work.
#define ADC_DMA_FRAME_COUNT     16U
#define ADC_DMA_BUFFER_WORDS    (2U * ADC_DMA_FRAME_COUNT)  //32个16位数据

// DMA CH2 stores 160 ADCB bursts; each burst contains PV voltage/current.
#define ADC_PV_DMA_BURST_COUNT  160U

typedef struct
{
    Uint16 pvVoltage;
    Uint16 pvCurrent;
} ADC_PV_DMA_Burst;

void GPIO_Config(void);

// LED identifiers and states used by the single LED control entry point.
#define LED_NUMBER_1       1U
#define LED_NUMBER_2       2U
#define LED_NUMBER_3       3U
#define LED_STATE_OFF      0U
#define LED_STATE_ON       1U
#define LED_STATE_TOGGLE   2U

// LED register access is kept inside gpio.c.
void LED_Ctrl(Uint16 ledNumber, Uint16 state);

void EPWM_Config(void);
void EPWM_Start(void);
void EPWM_Enable(void);

// Trip-zone control: a latched TZ1 event forces both PWM outputs low.
void EPWM_TripZoneForce(void);
void EPWM_TripZoneClear(void);

void EPWM_Disable(void);
void ADC_Config(void);
void DMA_Config(void);
void ECAP_Config(void);
void SCI_Config(void);
void I2C_Config(void);

void SCI_SendByte(Uint16 data);
void SCI_SendString(const char *text);
Uint16 SCI_ReadByte(Uint16 *data);

void EPWM_SetDuty(float duty);

extern volatile float OpenLoopInductorCurrentAmplitude;
extern volatile Uint16 SCI_RxDataPending;

__interrupt void ECAP1_BSP_ISR(void);
__interrupt void I2CA_BSP_ISR(void);
__interrupt void SCIA_BSP_RX_ISR(void);
__interrupt void ADCA1_CPU_ISR(void);
__interrupt void DMA_CH1_CPU_ISR(void);
__interrupt void DMA_CH2_CPU_ISR(void);
__interrupt void EPWM3_TZ_BSP_ISR(void);


#endif
