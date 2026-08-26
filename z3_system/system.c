#include "F28x_Project.h"

#include "system.h"
#include "bsp.h"
#include "constant.h"

#define SPLL_MAX_DEVIATION_HZ   5.0f//频率偏差限幅

//这个到时候自己调
#define SRF_PLL_DEFAULT_KP         60.0f
#define SRF_PLL_DEFAULT_KI         2000.0f

#define SOGI_PLL_DEFAULT_KP         90.0f
#define SOGI_PLL_DEFAULT_KI         4000.0f

#define SRF_PLL_DEFAULT_NOTCH_B0   1.3853181f
#define SRF_PLL_DEFAULT_NOTCH_B1   -2.7692690f
#define SRF_PLL_DEFAULT_NOTCH_B2   1.3853181f
#define SRF_PLL_DEFAULT_NOTCH_A1   -1.9590329f
#define SRF_PLL_DEFAULT_NOTCH_A2   0.9604000f

volatile SPLL_1ph GridSPLL;

void System_Init(void)
{
    // Establish clocks and a known interrupt state before configuring BSPs.
    InitSysCtrl();
    InitGpio();
    InitPieCtrl();
    DINT;
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // Configure PWM while its time base is stopped. Pin muxing is intentionally
    // delayed until the PWM output and dead-band registers contain safe values.
    EPWM_Config();
    GPIO_Config();
    ADC_Config();
    DMA_Config();
    ECAP_Config();
    SCI_Config();
    I2C_Config();

    // EPWM_Start() is called explicitly by main() after the CPU ADC ISR is ready.
}

void SRF_PLL_Init(volatile SPLL_1ph *pll, float nominalFrequencyHz, float sampleFrequencyHz)
{
    Uint16 index;

    pll->input = 0.0f;
    pll->phase = 0.0f;
    pll->sine = 0.0f;
    pll->cosine = 1.0f;//巧妙
    pll->frequencyHz = nominalFrequencyHz;
    pll->nominalFrequencyHz = nominalFrequencyHz;
    pll->phaseDetector = 0.0f;
    pll->notchOutput = 0.0f;
    pll->piIntegrator = 0.0f;
    pll->loopOutput = 0.0f;
    pll->samplePeriod = 1.0f / sampleFrequencyHz;
    pll->kp = SRF_PLL_DEFAULT_KP;
    pll->ki = SRF_PLL_DEFAULT_KI;
    pll->minFrequencyHz = nominalFrequencyHz - SPLL_MAX_DEVIATION_HZ;
    pll->maxFrequencyHz = nominalFrequencyHz + SPLL_MAX_DEVIATION_HZ;

    // 100 Hz notch coefficients for a 20 kHz sampled 50 Hz single-phase input.
    pll->notchB0 = SRF_PLL_DEFAULT_NOTCH_B0;
    pll->notchB1 = SRF_PLL_DEFAULT_NOTCH_B1;
    pll->notchB2 = SRF_PLL_DEFAULT_NOTCH_B2;
    pll->notchA1 = SRF_PLL_DEFAULT_NOTCH_A1;
    pll->notchA2 = SRF_PLL_DEFAULT_NOTCH_A2;
    pll->sogiAlpha = 0.0f;
    pll->sogiBeta = 0.0f;
    pll->sogiK = 1.41421356f;

    //清空历史缓存
    for(index = 0U; index < 3U; index++)
    {
        pll->detectorHistory[index] = 0.0f;
        pll->notchHistory[index] = 0.0f;
    }
}

void SOGI_PLL_Init(volatile SPLL_1ph *pll, float nominalFrequencyHz, float sampleFrequencyHz)
{
    Uint16 index;

    pll->input = 0.0f;
    pll->phase = 0.0f;
    pll->sine = 0.0f;
    pll->cosine = 1.0f;
    pll->frequencyHz = nominalFrequencyHz;
    pll->nominalFrequencyHz = nominalFrequencyHz;
    pll->phaseDetector = 0.0f;
    pll->notchOutput = 0.0f;
    pll->piIntegrator = 0.0f;
    pll->loopOutput = 0.0f;
    pll->samplePeriod = 1.0f / sampleFrequencyHz;
    pll->kp = SOGI_PLL_DEFAULT_KP;
    pll->ki = SOGI_PLL_DEFAULT_KI;
    pll->minFrequencyHz = nominalFrequencyHz - SPLL_MAX_DEVIATION_HZ;
    pll->maxFrequencyHz = nominalFrequencyHz + SPLL_MAX_DEVIATION_HZ;

    for(index = 0U; index < 3U; index++)
    {
        pll->detectorHistory[index] = 0.0f;
        pll->notchHistory[index] = 0.0f;
    }

    pll->sogiAlpha = 0.0f;  
    pll->sogiBeta = 0.0f;
    pll->sogiK = 1.41421356f;
}

void SRF_PLL_Run(volatile SPLL_1ph *pll, float input)
{
    float omega;
    float maxCorrection;

    pll->input = input;
    pll->detectorHistory[0] = input * pll->cosine;

    pll->notchHistory[0] =
        -pll->notchA1 * pll->notchHistory[1]
        -pll->notchA2 * pll->notchHistory[2]
        +pll->notchB0 * pll->detectorHistory[0]
        +pll->notchB1 * pll->detectorHistory[1]
        +pll->notchB2 * pll->detectorHistory[2];

    pll->phaseDetector = pll->detectorHistory[0];
    pll->notchOutput = pll->notchHistory[0];

    pll->piIntegrator += pll->ki * pll->samplePeriod * pll->notchOutput;
    maxCorrection = MATH_TWO_PI_F *
                    (pll->maxFrequencyHz - pll->nominalFrequencyHz);
    if(pll->piIntegrator > maxCorrection)
    {
        pll->piIntegrator = maxCorrection;
    }
    else if(pll->piIntegrator < -maxCorrection)
    {
        pll->piIntegrator = -maxCorrection;
    }
    pll->loopOutput = pll->kp * pll->notchOutput + pll->piIntegrator;

    omega = MATH_TWO_PI_F * pll->nominalFrequencyHz + pll->loopOutput;
    if(omega < MATH_TWO_PI_F * pll->minFrequencyHz)
    {
        omega = MATH_TWO_PI_F * pll->minFrequencyHz;
    }
    else if(omega > MATH_TWO_PI_F * pll->maxFrequencyHz)
    {
        omega = MATH_TWO_PI_F * pll->maxFrequencyHz;
    }
    pll->frequencyHz = omega * MATH_INV_TWO_PI_F;

    pll->phase += omega * pll->samplePeriod;
    while(pll->phase >= MATH_TWO_PI_F)
    {
        pll->phase -= MATH_TWO_PI_F;
    }
    while(pll->phase < 0.0f)
    {
        pll->phase += MATH_TWO_PI_F;
    }

    pll->sine = __sinpuf32(pll->phase * MATH_INV_TWO_PI_F);
    pll->cosine = __cospuf32(pll->phase * MATH_INV_TWO_PI_F);

    pll->detectorHistory[2] = pll->detectorHistory[1];
    pll->detectorHistory[1] = pll->detectorHistory[0];
    pll->notchHistory[2] = pll->notchHistory[1];
    pll->notchHistory[1] = pll->notchHistory[0];
}

void SOGI_PLL_Run(volatile SPLL_1ph *pll, float input)
{
    float omega;
    float error;
    float alphaOld;
    float vq;
    float maxCorrection;

    pll->input = input;
    omega = MATH_TWO_PI_F * pll->frequencyHz;

    error = input - pll->sogiAlpha;
    alphaOld = pll->sogiAlpha;
    pll->sogiAlpha = alphaOld + pll->samplePeriod * (pll->sogiK * omega * error - omega * pll->sogiBeta);
    pll->sogiBeta += pll->samplePeriod * (omega * alphaOld);

    vq = pll->sogiAlpha * pll->cosine + pll->sogiBeta * pll->sine;
    pll->phaseDetector = vq;
    pll->notchOutput = vq;

    pll->piIntegrator += pll->ki * pll->samplePeriod * vq;
    maxCorrection = MATH_TWO_PI_F * (pll->maxFrequencyHz - pll->nominalFrequencyHz);
    if(pll->piIntegrator > maxCorrection)
    {
        pll->piIntegrator = maxCorrection;
    }
    else if(pll->piIntegrator < -maxCorrection)
    {
        pll->piIntegrator = -maxCorrection;
    }
    pll->loopOutput = pll->kp * vq + pll->piIntegrator;

    omega = MATH_TWO_PI_F * pll->nominalFrequencyHz + pll->loopOutput;
    if(omega < MATH_TWO_PI_F * pll->minFrequencyHz)
    {
        omega = MATH_TWO_PI_F * pll->minFrequencyHz;
    }
    else if(omega > MATH_TWO_PI_F * pll->maxFrequencyHz)
    {
        omega = MATH_TWO_PI_F * pll->maxFrequencyHz;
    }
    pll->frequencyHz = omega * MATH_INV_TWO_PI_F;

    pll->phase += omega * pll->samplePeriod;
    while(pll->phase >= MATH_TWO_PI_F)
    {
        pll->phase -= MATH_TWO_PI_F;
    }
    while(pll->phase < 0.0f)
    {
        pll->phase += MATH_TWO_PI_F;
    }

    pll->sine = __sinpuf32(pll->phase * MATH_INV_TWO_PI_F);
    pll->cosine = __cospuf32(pll->phase * MATH_INV_TWO_PI_F);
}
