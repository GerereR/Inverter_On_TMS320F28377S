#ifndef SYSTEM_H_
#define SYSTEM_H_

//这里没有角频率
//像这种不是全局调用的,放在自身文件里就好
typedef struct
{
    float input;//输入
    float phase;//输出

    float sine;//正弦输出
    float cosine;//余弦输出

    float frequencyHz;//追踪的频率

    float nominalFrequencyHz;//额定频率

    float phaseDetector;//鉴相器输出
    float notchOutput;//陷波器输出
    float piIntegrator;//积分器输出
    float loopOutput;//PI输出
    
    float samplePeriod;//采样周期

    float kp;//PI控制器比例
    float ki;//PI控制器积分

    float minFrequencyHz;//PI下限
    float maxFrequencyHz;//PI上限

    float notchB0;//陷波器分子系数
    float notchB1;
    float notchB2;

    float notchA1;//陷波器分母系数
    float notchA2;

    float sogiAlpha;//SOGI α轴
    float sogiBeta;//SOGI β轴
    float sogiK;

    //历史输出,离散控制用
    float detectorHistory[3];
    float notchHistory[3];

} SPLL_1ph;

extern volatile SPLL_1ph GridSPLL;

void System_Init(void);

void SRF_PLL_Init(volatile SPLL_1ph *pll, float nominalFrequencyHz, float sampleFrequencyHz);
void SOGI_PLL_Init(volatile SPLL_1ph *pll, float nominalFrequencyHz, float sampleFrequencyHz);

void SRF_PLL_Run(volatile SPLL_1ph *pll, float input);
void SOGI_PLL_Run(volatile SPLL_1ph *pll, float input);


#endif
