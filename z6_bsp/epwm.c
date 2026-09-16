#include "F28x_Project.h"
#include "bsp.h"
#include "variable.h"
#include "invert.h"

//周期计数器值,向上计数到2500开始向下计数
#define EPWM_PERIOD_TICK          2500U
//死区时间
#define EPWM_DEADBAND_TICK        100U
//AQ不强制
#define EPWM_AQ_FORCE_DISABLED      0U
//AQ强制低
#define EPWM_AQ_FORCE_LOW           1U
//AQ强制高
#define EPWM_AQ_FORCE_HIGH          2U
//AQ立即强制
#define EPWM_AQ_FORCE_IMMEDIATE     3U

//独立,不同步
#define EPWM_SYNC_INDEPENDENT       0U
//作为主同步
#define EPWM_SYNC_MASTER            1U
//作为副同步
#define EPWM_SYNC_SLAVE             2U

//蜂鸣器时钟频率,25MHz
#define BEEP_PWM_CLOCK_HZ           25000000UL
//蜂鸣器默认频率2KHz
#define BEEP_DEFAULT_FREQ_HZ   2000U
//蜂鸣器声音频率范围400Hz~10KHz
#define BEEP_MIN_FREQ_HZ       400U
#define BEEP_MAX_FREQ_HZ       10000U

//这个函数是为了单独配置功率级的ePWM模块的
static void EPWM_ConfigPowerStage
(
    //待配置ePWM模块
    volatile struct EPWM_REGS *pwm,
    //设置同步模式
    Uint16 syncMode,
    //死区使能配置
    Uint16 useDeadband,
    //TZ使能配置
    Uint16 useTz1,
    Uint16 useTz2,
    Uint16 useTz3
)
{
    //配置时基寄存器
    pwm->TBCTL.all = 0U;
    //几个ePWM都是上下计数模式
    pwm->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    //根据输入参数考虑是否启用同步
    pwm->TBCTL.bit.PHSEN = (syncMode == EPWM_SYNC_SLAVE) ? TB_ENABLE : TB_DISABLE;
    //如果选择了同步,且配置为主
    if(syncMode == EPWM_SYNC_MASTER)
    {
        //当CTR=0时,输出一个同步信号
        pwm->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    }
    //如果选择了同步,且配置为从
    else if(syncMode == EPWM_SYNC_SLAVE)
    {
        //接收外部的同步信号
        pwm->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    }
    //参数有误就不使能
    else
    {
        pwm->TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    }
    //启用周期影子寄存器
    pwm->TBCTL.bit.PRDLD = TB_SHADOW;
    //ePWM内部时钟不分频
    pwm->TBCTL.bit.HSPCLKDIV = TB_DIV1;
    pwm->TBCTL.bit.CLKDIV = TB_DIV1;
    //软件调试时停止输出
    pwm->TBCTL.bit.FREE_SOFT = 2U;
    //周期计数器配置为2500,
    pwm->TBPRD = EPWM_PERIOD_TICK;
    //告诉ePWM: "当你接收到同步之后,应该跳转到CTR=0,然后向上计数"
    pwm->TBPHS.all = 0U;
    //初始化周期计数器
    pwm->TBCTR = 0U;

    //配置比较寄存器
    pwm->CMPCTL.all = 0U;
    //A,B两路都配置为双缓冲操作,具体装载时间见下
    pwm->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    pwm->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    //A,B两路装载时刻都在CTR=0或CTR=2500时刻装载
    pwm->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    pwm->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
    //初始先给个0.5的占空比
    pwm->CMPA.bit.CMPA = EPWM_PERIOD_TICK / 2U;

    //配置动作限定寄存器,这个寄存器的作用就是处理处理PWM事件,把事件变成PWM波
    //对于大功率器件,当我发送强制低的信号时,就应该立即执行,不管你当前是什么状态
    pwm->AQSFRC.bit.RLDCSF = EPWM_AQ_FORCE_IMMEDIATE;
    //配置A路的AQ寄存器
    pwm->AQCTLA.all = 0U;
    //对于A路,当基波>载波的时候输出高
    pwm->AQCTLA.bit.CAU = AQ_SET;
    pwm->AQCTLA.bit.CAD = AQ_CLEAR;
    //对于B路,没配置,因为他将会在死区模块被配置为互补
    pwm->AQCTLB.all = 0U;

    //配置死区寄存器
    //如果有配置死区,暗示我们正在配置INV的开关管
    if(useDeadband != 0U)
    {
        pwm->DBCTL.all = 0U;
        //使能死区模块,而且提前关断,延时导通都启用
        pwm->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        //A路作为唯一的死区输入路,也就是说,延时导通,提前关断都体现在A路
        pwm->DBCTL.bit.IN_MODE = DBA_ALL;
        //设置为高电平有效,且AB互补
        pwm->DBCTL.bit.POLSEL = DB_ACTV_HIC;
        //FED和RED的时间,也就是死区时间
        pwm->DBRED.bit.DBRED = EPWM_DEADBAND_TICK;
        pwm->DBFED.bit.DBFED = EPWM_DEADBAND_TICK;
        //其实死区的配置很简单,一句话:
        //AB两路首先是互补的,但是每一路从低->高的时候都得乖乖等待设定的死区时间,外在表现都是延时导通.
        //之所以强调"外在"这一个词,是因为配置方法不止这一种
    }
    //如果没有配置死区,暗示我们正在配置BOOST的开关管
    else
    {
        //清除死区配置
        pwm->DBCTL.all = 0U;
        //接着回到AQ配置,B路是独立的,但是方向和A路相反:基波>载波输出低
        pwm->AQCTLB.bit.CBU = AQ_CLEAR;
        pwm->AQCTLB.bit.CBD = AQ_SET;
        //初始占空比也设置为0.5
        pwm->CMPB.bit.CMPB = EPWM_PERIOD_TICK / 2U;
    }
    //TZ 配置
    pwm->TZSEL.all = 0U;
    //选择使能TZ通道
    pwm->TZSEL.bit.OSHT1 = useTz1;
    pwm->TZSEL.bit.OSHT2 = useTz2;
    pwm->TZSEL.bit.OSHT3 = useTz3;
    //当触发了TZ之后,两路PWN都强制低,注意:这是硬件自动触发的,无条件的
    pwm->TZCTL.bit.TZA = TZ_FORCE_LO;
    pwm->TZCTL.bit.TZB = TZ_FORCE_LO;
    //启用中断
    pwm->TZEINT.bit.OST = ((useTz1 != 0U) || (useTz2 != 0U) || (useTz3 != 0U)) ? 1U : 0U;
    //清除所有可能的中断标志
    pwm->TZCLR.bit.OST = 1U;
    pwm->TZOSTCLR.bit.OST1 = 1U;
    pwm->TZOSTCLR.bit.OST2 = 1U;
    pwm->TZOSTCLR.bit.OST3 = 1U;
    pwm->TZCLR.bit.INT = 1U;
    //初始化时 A、B 都强制为低，保证上电时功率管是关断的
    pwm->AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    pwm->AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
}

void EPWM_Config(void)
{
    EALLOW;
    //冻结所有时间基
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;
    //系统时钟二分频进入EPWM,结合之前的配置,就可以知道(2MHz / 2) / (2 * 2500) = 20KHz
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1U;
    //从配置参数可以知道:
    //ePWM1 & ePWM2是逆变器PWM, 其中ePWN1作为主同步信号, 都开启了死区功能, 两个模块共用TZ3
    EPWM_ConfigPowerStage(&EPwm1Regs, EPWM_SYNC_MASTER,      1U, 0U, 0U, 1U); 
    EPWM_ConfigPowerStage(&EPwm2Regs, EPWM_SYNC_SLAVE,       1U, 0U, 0U, 1U);
    //ePWM3是BOOSt的PWM, 模块使用TZ1
    EPWM_ConfigPowerStage(&EPwm3Regs, EPWM_SYNC_INDEPENDENT, 0U, 1U, 0U, 0U); 

    //EPWM4是未使用的,不必关心
    EPwm4Regs.TBCTL.all = 0U;
    EPwm4Regs.TZSEL.all = 0U;
    EPwm4Regs.TZSEL.bit.OSHT2 = 1U;
    EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPwm4Regs.TZEINT.bit.OST = 1U;
    EPwm4Regs.TZCLR.bit.OST = 1U;
    EPwm4Regs.TZCLR.bit.INT = 1U;

    //ePWM2不启用TZ中断, 也就是说只有ePWM1接收TZ3中断
    EPwm2Regs.TZEINT.bit.OST = 0U;
    EPwm2Regs.TZCLR.bit.INT = 1U;
    //失能PWM2A触发ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 0U;
    //CTR到达0的时候通知ADC采集数据
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    //每次到达设定CTR都通知ADC
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;
    //清除残留事件
    EPwm1Regs.ETCLR.bit.SOCA = 1U;
    //使能PWM2A触发ADC!
    EPwm1Regs.ETSEL.bit.SOCAEN = 1U;

    //注册中断表
    PieVectTable.EPWM1_TZ_INT = &EPWM1_TZ_BSP_ISR;
    PieVectTable.EPWM3_TZ_INT = &EPWM3_TZ_BSP_ISR;
    PieVectTable.EPWM4_TZ_INT = &EPWM4_TZ_BSP_ISR;
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1U;
    PieCtrlRegs.PIEIER2.bit.INTx3 = 1U;
    PieCtrlRegs.PIEIER2.bit.INTx4 = 1U;
    IER |= M_INT2;

    //以下是蜂鸣器ePWM配置
    EPwm5Regs.TBCTL.all = 0U;
    //向上计数,启用影子寄存器
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    //四分频,也就是100Mhz / (4+0) = 25Mhz
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    //初始频率配置为2KHz
    EPwm5Regs.TBPRD = (Uint16)(BEEP_PWM_CLOCK_HZ / BEEP_DEFAULT_FREQ_HZ - 1UL);
    EPwm5Regs.CMPCTL.all = 0U;
    //CTR=0时影子寄存器装配到里面
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    //初始占空比就定为0.5
    EPwm5Regs.CMPA.bit.CMPA = EPwm5Regs.TBPRD / 2U;
    EPwm5Regs.AQCTLA.all = 0U;
    //基波>载波高电平
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    //软件置位立即生效
    EPwm5Regs.AQSFRC.bit.RLDCSF = EPWM_AQ_FORCE_IMMEDIATE;
    //初始强制低
    EPwm5Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;

    EDIS;
}

//设置蜂鸣器频率
void BEEP_SetFreq(Uint16 freqHz)
{
    Uint32 periodTick;
    //限幅 400Hz~10kHz
    if      (freqHz < BEEP_MIN_FREQ_HZ) { freqHz = BEEP_MIN_FREQ_HZ; }
    else if (freqHz > BEEP_MAX_FREQ_HZ) { freqHz = BEEP_MAX_FREQ_HZ; }
    //频率转换为周期计数器值
    periodTick = BEEP_PWM_CLOCK_HZ / (Uint32)freqHz;
    if(periodTick > 0UL)       { periodTick--; }
    if(periodTick > 65535UL)   { periodTick = 65535UL; }
    //装配
    EALLOW;
    EPwm5Regs.TBPRD = (Uint16)periodTick;
    EPwm5Regs.CMPA.bit.CMPA = (Uint16)(periodTick / 2UL);
    EDIS;
}

//释放时间基同步，所有 ePWM 开始跑。功率输出仍被软件强制低
//函数里只有一句代码,主要是和ePWM_Config第一句相呼应,等所有的BSP都配置好了就开始启动ePWM
void EPWM_Start(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    EDIS;
}

//四个模块 A、B 全部软件强制低，功率管全关。
void EPWM_Disable(void)
{
    EALLOW;
    EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
    EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
    EPwm3Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    EPwm3Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_LOW;
    EDIS;
}

//先确认没有硬件故障，再解除软件强制
Uint16 EPWM_Enable(void)
{
    //判断错误,清除故障
    if(EPWM_TripZoneClear() == 0U)
    {
        return 0U;
    }
    EALLOW;
    EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
    EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
    EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    EPwm3Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
    EPwm3Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    EPwm4Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
    EPwm4Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    EDIS;
    return 1U;
}

//软件强制触发一次 TZ，让所有功率输出立刻关断。
void EPWM_TripZoneForce(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1U;
    EPwm2Regs.TZFRC.bit.OST = 1U;
    EPwm3Regs.TZFRC.bit.OST = 1U;
    EPwm4Regs.TZFRC.bit.OST = 1U;
    EDIS;
}

Uint16 EPWM_TripZoneClear(void)
{
    //GPIO62/63/64 分别是 TZ1/TZ2/TZ3 的物理引脚
    //只要任一路还是低，就不允许清故障
    if((GpioDataRegs.GPBDAT.bit.GPIO62 == 0U) ||
       (GpioDataRegs.GPBDAT.bit.GPIO63 == 0U) ||
       (GpioDataRegs.GPCDAT.bit.GPIO64 == 0U))
    {
        //通知系统出现过流故障
        gSysProblem.recovFault |= RECOV_TZ_FAULT;
        return 0U;
    }

    EALLOW;
    //清除单次触发障锁存标志位 
    EPwm1Regs.TZCLR.bit.OST = 1U;
    EPwm2Regs.TZCLR.bit.OST = 1U;
    EPwm3Regs.TZCLR.bit.OST = 1U;
    EPwm4Regs.TZCLR.bit.OST = 1U;

    //清除单次触发源标志位
    EPwm1Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm1Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm1Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm2Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm2Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm2Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST3 = 1U;    
    EPwm4Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm4Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm4Regs.TZOSTCLR.bit.OST3 = 1U;   

    //清除中断标志
    EPwm1Regs.TZCLR.bit.INT = 1U;
    EPwm2Regs.TZCLR.bit.INT = 1U; 
    EPwm3Regs.TZCLR.bit.INT = 1U;
    EPwm4Regs.TZCLR.bit.INT = 1U;
    EDIS;

    //清除错误标志位
    gSysProblem.recovFault &=~ RECOV_TZ_FAULT;
    return 1U;
}
//记录故障标志，清中断标志
static void EPWM_RecordTrip(volatile struct EPWM_REGS *pwm)
{
    gSysProblem.recovFault |= RECOV_TZ_FAULT;
    pwm->TZCLR.bit.INT = 1U;
}

//EPWM1和EPWM2绑定了TZ3,他代表电感电流过流
__interrupt void EPWM1_TZ_BSP_ISR(void)
{
    EPWM_RecordTrip(&EPwm1Regs);
    //清除PIE,说明中断处理完毕
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}
//EPWM3绑定了TZ1,他代表PV过流故障
__interrupt void EPWM3_TZ_BSP_ISR(void)
{
    EPWM_RecordTrip(&EPwm3Regs);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}
//未使用
__interrupt void EPWM4_TZ_BSP_ISR(void)
{
    EPWM_RecordTrip(&EPwm4Regs);
    EPWM_Disable();
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

/*单极性调制策略,输入的是占空比,暂时不涉及到高级的控制
死区固定,未来肯定会遇到过零点畸变的问题,到时候再改 */
void EPWM_SetInvertMode(float modulate)
{
    Uint16 compareValue;
    //限幅
    modulate = Invert_Clamp(modulate, -1.0f, 1.0f);
    //取绝对值,然后转换为CMP值
    compareValue = (Uint16)(((modulate >= 0.0f) ? modulate : -modulate) * (float)EPWM_PERIOD_TICK);
    //先一股脑给两个桥臂占空比,回头再精细控制
    //为什么CMPA可以直接等于|modulate|,你可以思考一下😋
    EPwm1Regs.CMPA.bit.CMPA = compareValue;
    EPwm2Regs.CMPA.bit.CMPA = compareValue;
    //如果占空比大于零,也就是想让桥臂1控制,桥臂2置高
    if(modulate > 0.0f)
    {  
        //EPWM2A强制高,意味着桥臂2保持高电平
        EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_HIGH;
        //剩下的自由调制
        EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
        EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
        EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    }
    //如果占空比小于零,也就是想让桥臂2控制,桥臂1置高
    else if(modulate < 0.0f)
    {
        //EPWM1强制高,意味着桥臂1保持高电平
        EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_HIGH;
        //剩下的自由调制
        EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
        EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_DISABLED;
        EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    }
    //两路都置高,差分电压为0
    else
    {
        EPwm1Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_HIGH;
        EPwm1Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
        EPwm2Regs.AQCSFRC.bit.CSFA = EPWM_AQ_FORCE_HIGH;
        EPwm2Regs.AQCSFRC.bit.CSFB = EPWM_AQ_FORCE_DISABLED;
    }
}

//分别输入两路BOOST的占空比
void EPWM_SetBoostDuty(float boost1Duty, float boost2Duty)
{
    Uint16 boost1Compare;
    Uint16 boost2Compare;
    //限幅
    boost1Duty = Invert_Clamp(boost1Duty, 0.0f, 0.98f);
    boost2Duty = Invert_Clamp(boost2Duty, 0.0f, 0.98f);
    //结合EPWM3的配置,实际上是交错式BOOST,而A路和正常逻辑相反
    boost1Compare = (Uint16)((1.0f - boost1Duty) * (float)EPWM_PERIOD_TICK);
    boost2Compare = (Uint16)(boost2Duty * (float)EPWM_PERIOD_TICK);
    //更新CMP
    EPwm3Regs.CMPA.bit.CMPA = boost1Compare;
    EPwm3Regs.CMPB.bit.CMPB = boost2Compare;
}
