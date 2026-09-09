#include "F28x_Project.h"

#include "task.h"
#include "constant.h"
#include "variable.h"

/* 有功功率低于此值不给无功（老代码约 20% 额定，防小功率下角度发散）。 */
#define REACTIVE_ACTIVE_MIN_W       600.0f

/*============================================================================
 * 无功调度 + 电容补偿（浮点化自 New_Master/ctrl/reactive.c）。
 *
 * 两种控制方式（开环）：
 *   1. 固定功率因数 cosφ：φ = acos(|setValue|)，符号由 setValue 定（正=超前/负=滞后）
 *   2. 固定无功功率 Q   ：φ = atan(setValue / P)，setValue 为带符号 Q(Var)
 * 另加电容补偿角 θc，抵消输出滤波电容的相位滞后（沿用老代码公式）。
 *
 * TMU 的 atan2puf32 返回周标幺角，φ、θc 和 phaseShiftPu 全部统一使用该单位。
 * 电流环直接把 phaseShiftPu 加进 PLL 相位得到 sin(θ+φ)。
 *
 * 符号约定统一：正 = 超前(容性)，负 = 滞后(感性)。
 *==========================================================================*/
static float Reactive_ValueCap = REACTIVE_VALUE_CAP_3K;

void Task_Reactive_Init(void)
{
    gReactiveData.mode = REACTIVE_MODE_OFF;      /* 禁用无功调度，但保留电容补偿 */
    gReactiveData.setValue = 1.0f;               /* mode=PF 时默认 cosφ=1（无相移） */
    gReactiveData.phaseShiftPu = 0.0f;
    Reactive_ValueCap = REACTIVE_VALUE_CAP_3K;
}

void Task_Reactive(void)
{
    /* 输入：测量层发布的电网量。 */
    float gridActivePower   = gMachineData.powerData.gridActivePower;
    float gridFreqHz        = (float)gMachineData.pllFreqCent * 0.01f;
    float vgridRms          = gMachineData.realRms.gridVoltage;
    float igridRms          = gMachineData.realRms.inductorCurrent;
    Uint16 mode              = gReactiveData.mode;
    float setValue           = gReactiveData.setValue;

    float phiReactive = 0.0f;   /* 无功相移角 */
    float thetaCap = 0.0f;      /* 电容补偿角 */
    float phaseShiftPu;
    float phiMax;
    float pfMag;                /* 固定 cosφ 模式的 |cosφ| 幅值 */

    /* 1) 电容补偿角：抵消输出滤波电容的相位滞后。
     *    老代码：theta_c = atan((2π/2366000)*(freq_centiHz/10000)*Vrms*cap/Irms)
     *    浮点化：theta_c = atan(2π*freqHz*Vrms*cap / (scale*Irms)) */
    if (igridRms > 0.001f)
    {
        /* atan(分子/分母) = atan2(分子, 分母)，走 TMU ATANPUF32 指令。 */
        thetaCap = __atan2puf32(MATH_TWO_PI_F * gridFreqHz * vgridRms * Reactive_ValueCap, REACTIVE_THETA_C_SCALE * igridRms);
    }

    /* 2) 无功相移角：仅在有功足够时给无功（避免小功率下 atan(Q/P) 发散）。 */
    if (gridActivePower >= REACTIVE_ACTIVE_MIN_W)
    {
        if (mode == REACTIVE_MODE_PF)
        {
            /* 固定 cosφ：|setValue| clamp 到 [PF_min, 1]，符号表超前(+) / 滞后(-)。 */
            pfMag = (setValue < 0.0f) ? -setValue : setValue;

            if (pfMag < REACTIVE_PF_MIN)
            {
                pfMag = REACTIVE_PF_MIN;
            }
            if (pfMag > 1.0f)
            {
                pfMag = 1.0f;
            }
            /* TMU 无 acos，用 acos(x) = atan2(sqrt(1-x²), x) 推导。 */
            phiReactive = __atan2puf32(__sqrt(1.0f - pfMag * pfMag), pfMag);
            if (setValue < 0.0f)
            {
                phiReactive = -phiReactive;  /* 负 = 滞后(感性) */
            }
        }
        else if (mode == REACTIVE_MODE_Q)
        {
            /* 固定 Q：setValue = 目标 Q(Var，带符号)。φ = atan2(Q, P)。 */
            phiReactive = __atan2puf32(setValue, gridActivePower);
        }
        /* REACTIVE_MODE_OFF：phiReactive 保持 0（纯有功）。 */
    }

    /* 3) 相移角限幅：|φ| <= acos(PF_min)，保证总相移对应 PF 不低于 0.8。 */
    phiMax = __atan2puf32(__sqrt(1.0f - REACTIVE_PF_MIN * REACTIVE_PF_MIN), REACTIVE_PF_MIN);
    if (phiReactive > phiMax) phiReactive = phiMax;
    else if (phiReactive < -phiMax) phiReactive = -phiMax;

    /* 4) 总相移 = 无功角 + 电容补偿角，仍保持为周标幺值。 */
    phaseShiftPu = phiReactive + thetaCap;
    if (phaseShiftPu > phiMax)
    {
        phaseShiftPu = phiMax;
    }
    else if (phaseShiftPu < -phiMax)
    {
        phaseShiftPu = -phiMax;
    }

    /* 5) 电流环直接消费统一单位的相移输出。 */
    gReactiveData.phaseShiftPu = phaseShiftPu;
}
