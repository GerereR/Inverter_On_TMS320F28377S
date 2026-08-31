#ifndef PLL_H_
#define PLL_H_

/* PLL state and algorithms belong to the control layer, not system startup. */
typedef struct
{
    float input;
    float phase;

    float freqHz;
    float nomFreqHz;

    float phaseDet;
    float notchOut;
    float piInt;
    float loopOut;
    float sampleTs;

    float kp;
    float ki;
    float minFreqHz;
    float maxFreqHz;

    float notchB0;
    float notchB1;
    float notchB2;
    float notchA1;
    float notchA2;

    float sogiAlpha;
    float sogiBeta;
    float sogiK;

    float detHist[3];
    float notchHist[3];
} SPLL_1ph;

extern volatile SPLL_1ph GridSPLL;

void SRF_PLL_Init(volatile SPLL_1ph *pll, float nomFreqHz, float sampleFreqHz);
void SOGI_PLL_Init(volatile SPLL_1ph *pll, float nomFreqHz, float sampleFreqHz);

void SRF_PLL_Run(volatile SPLL_1ph *pll, float input);
void SOGI_PLL_Run(volatile SPLL_1ph *pll, float input);

#endif /* PLL_H_ */
