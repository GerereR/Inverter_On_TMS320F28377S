#include "F28x_Project.h"

#include "pll.h"
#include "constant.h"

#define SPLL_MAX_DEVIATION_HZ       5.0f

#define SRF_PLL_DEFAULT_KP          60.0f
#define SRF_PLL_DEFAULT_KI          2000.0f

#define SOGI_PLL_DEFAULT_KP         90.0f
#define SOGI_PLL_DEFAULT_KI         4000.0f

#define SRF_PLL_DEFAULT_NOTCH_B0    1.3853181f
#define SRF_PLL_DEFAULT_NOTCH_B1   -2.7692690f
#define SRF_PLL_DEFAULT_NOTCH_B2    1.3853181f
#define SRF_PLL_DEFAULT_NOTCH_A1   -1.9590329f
#define SRF_PLL_DEFAULT_NOTCH_A2    0.9604000f

volatile SPLL_1ph GridSPLL;

void SRF_PLL_Init(volatile SPLL_1ph *pll,
                  float nomFreqHz,
                  float sampleFreqHz)
{
    Uint16 index;

    pll->input = 0.0f;
    pll->phase = 0.0f;
    pll->freqHz = nomFreqHz;
    pll->nomFreqHz = nomFreqHz;
    pll->phaseDet = 0.0f;
    pll->notchOut = 0.0f;
    pll->piInt = 0.0f;
    pll->loopOut = 0.0f;
    pll->sampleTs = 1.0f / sampleFreqHz;
    pll->kp = SRF_PLL_DEFAULT_KP;
    pll->ki = SRF_PLL_DEFAULT_KI;
    pll->minFreqHz = nomFreqHz - SPLL_MAX_DEVIATION_HZ;
    pll->maxFreqHz = nomFreqHz + SPLL_MAX_DEVIATION_HZ;

    pll->notchB0 = SRF_PLL_DEFAULT_NOTCH_B0;
    pll->notchB1 = SRF_PLL_DEFAULT_NOTCH_B1;
    pll->notchB2 = SRF_PLL_DEFAULT_NOTCH_B2;
    pll->notchA1 = SRF_PLL_DEFAULT_NOTCH_A1;
    pll->notchA2 = SRF_PLL_DEFAULT_NOTCH_A2;
    pll->sogiAlpha = 0.0f;
    pll->sogiBeta = 0.0f;
    pll->sogiK = 1.41421356f;

    for(index = 0U; index < 3U; index++)
    {
        pll->detHist[index] = 0.0f;
        pll->notchHist[index] = 0.0f;
    }
}

void SOGI_PLL_Init(volatile SPLL_1ph *pll,
                   float nomFreqHz,
                   float sampleFreqHz)
{
    Uint16 index;

    pll->input = 0.0f;
    pll->phase = 0.0f;
    pll->freqHz = nomFreqHz;
    pll->nomFreqHz = nomFreqHz;
    pll->phaseDet = 0.0f;
    pll->notchOut = 0.0f;
    pll->piInt = 0.0f;
    pll->loopOut = 0.0f;
    pll->sampleTs = 1.0f / sampleFreqHz;
    pll->kp = SOGI_PLL_DEFAULT_KP;
    pll->ki = SOGI_PLL_DEFAULT_KI;
    pll->minFreqHz = nomFreqHz - SPLL_MAX_DEVIATION_HZ;
    pll->maxFreqHz = nomFreqHz + SPLL_MAX_DEVIATION_HZ;

    for(index = 0U; index < 3U; index++)
    {
        pll->detHist[index] = 0.0f;
        pll->notchHist[index] = 0.0f;
    }

    pll->sogiAlpha = 0.0f;
    pll->sogiBeta = 0.0f;
    pll->sogiK = 1.41421356f;
}

void SRF_PLL_Run(volatile SPLL_1ph *pll, float input)
{
    float omega;
    float maxCorrection;
    float phasePu;
    float phaseCos;

    pll->input = input;
    phasePu = pll->phase * MATH_INV_TWO_PI_F;
    phaseCos = __cospuf32(phasePu);
    pll->detHist[0] = input * phaseCos;

    pll->notchHist[0] =
        -pll->notchA1 * pll->notchHist[1]
        -pll->notchA2 * pll->notchHist[2]
        +pll->notchB0 * pll->detHist[0]
        +pll->notchB1 * pll->detHist[1]
        +pll->notchB2 * pll->detHist[2];

    pll->phaseDet = pll->detHist[0];
    pll->notchOut = pll->notchHist[0];

    pll->piInt += pll->ki * pll->sampleTs * pll->notchOut;
    maxCorrection = MATH_TWO_PI_F *
                    (pll->maxFreqHz - pll->nomFreqHz);
    if(pll->piInt > maxCorrection)
    {
        pll->piInt = maxCorrection;
    }
    else if(pll->piInt < -maxCorrection)
    {
        pll->piInt = -maxCorrection;
    }
    pll->loopOut = pll->kp * pll->notchOut + pll->piInt;

    omega = MATH_TWO_PI_F * pll->nomFreqHz + pll->loopOut;
    if(omega < MATH_TWO_PI_F * pll->minFreqHz)
    {
        omega = MATH_TWO_PI_F * pll->minFreqHz;
    }
    else if(omega > MATH_TWO_PI_F * pll->maxFreqHz)
    {
        omega = MATH_TWO_PI_F * pll->maxFreqHz;
    }
    pll->freqHz = omega * MATH_INV_TWO_PI_F;

    pll->phase += omega * pll->sampleTs;
    while(pll->phase >= MATH_TWO_PI_F)
    {
        pll->phase -= MATH_TWO_PI_F;
    }
    while(pll->phase < 0.0f)
    {
        pll->phase += MATH_TWO_PI_F;
    }

    pll->detHist[2] = pll->detHist[1];
    pll->detHist[1] = pll->detHist[0];
    pll->notchHist[2] = pll->notchHist[1];
    pll->notchHist[1] = pll->notchHist[0];
}

void SOGI_PLL_Run(volatile SPLL_1ph *pll, float input)
{
    float omega;
    float error;
    float alphaOld;
    float vq;
    float maxCorrection;
    float phasePu;
    float phaseSin;
    float phaseCos;

    pll->input = input;
    omega = MATH_TWO_PI_F * pll->freqHz;
    phasePu = pll->phase * MATH_INV_TWO_PI_F;
    phaseSin = __sinpuf32(phasePu);
    phaseCos = __cospuf32(phasePu);

    error = input - pll->sogiAlpha;
    alphaOld = pll->sogiAlpha;
    pll->sogiAlpha = alphaOld +
                     pll->sampleTs *
                     (pll->sogiK * omega * error - omega * pll->sogiBeta);
    pll->sogiBeta += pll->sampleTs * (omega * alphaOld);

    vq = pll->sogiAlpha * phaseCos + pll->sogiBeta * phaseSin;
    pll->phaseDet = vq;
    pll->notchOut = vq;

    pll->piInt += pll->ki * pll->sampleTs * vq;
    maxCorrection = MATH_TWO_PI_F *
                    (pll->maxFreqHz - pll->nomFreqHz);
    if(pll->piInt > maxCorrection)
    {
        pll->piInt = maxCorrection;
    }
    else if(pll->piInt < -maxCorrection)
    {
        pll->piInt = -maxCorrection;
    }
    pll->loopOut = pll->kp * vq + pll->piInt;

    omega = MATH_TWO_PI_F * pll->nomFreqHz + pll->loopOut;
    if(omega < MATH_TWO_PI_F * pll->minFreqHz)
    {
        omega = MATH_TWO_PI_F * pll->minFreqHz;
    }
    else if(omega > MATH_TWO_PI_F * pll->maxFreqHz)
    {
        omega = MATH_TWO_PI_F * pll->maxFreqHz;
    }
    pll->freqHz = omega * MATH_INV_TWO_PI_F;

    pll->phase += omega * pll->sampleTs;
    while(pll->phase >= MATH_TWO_PI_F)
    {
        pll->phase -= MATH_TWO_PI_F;
    }
    while(pll->phase < 0.0f)
    {
        pll->phase += MATH_TWO_PI_F;
    }

}
