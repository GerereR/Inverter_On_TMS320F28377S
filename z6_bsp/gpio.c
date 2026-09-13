#include "F28x_Project.h"
#include "bsp.h"

//四个KEY
#define GPIO_KEY_COUNT             4U
//消抖门限:4次state任务都是低电平
#define GPIO_KEY_DEBOUNCE_SAMPLES  4U 

static Uint16 GPIO_ReadKeyPressed(Uint16 keyNumber);
static void GPIO_KeyDebounceInit(void);

//每个按键的稳定状态,初始化的时候记录一次,自适应
static Uint16 GPIO_KeyStablePressed[GPIO_KEY_COUNT] = {0U, 0U, 0U, 0U};
//记录上一次四个按键的状态
static Uint16 GPIO_KeyLastPressed[GPIO_KEY_COUNT] = {0U, 0U, 0U, 0U};
//每个按键的连续性计数,这样可以做到 单个/多个 和 单次/连续 按键检测
static Uint16 GPIO_KeyDebounceCount[GPIO_KEY_COUNT] = {0U, 0U, 0U, 0U};
// 待处理事件累计寄存器
static Uint16 GPIO_KeyPendingEvents = 0U;
/* 按键事件位掩码表，索引对应 KEY_NUMBER_x。 */
static const Uint16 GPIO_KeyEventMasks[GPIO_KEY_COUNT] = {KEY_EVENT_1, KEY_EVENT_2, KEY_EVENT_3, KEY_EVENT_4};

//采用批量赋值的方法,
//各引脚定义,模块配置,掩码配置详见pin.md
#if 1
void GPIO_Config(void)
{
    EALLOW;
    //GPA
    GpioCtrlRegs.GPAPUD.all    |= 0x803C0DFFUL; 
    GpioCtrlRegs.GPAGMUX1.all  &= ~0x00000DFFUL;
    GpioCtrlRegs.GPAMUX1.all   &= ~0x00000DFFUL;
    GpioCtrlRegs.GPAMUX1.all   |= 0x000001FFUL;
    GpioCtrlRegs.GPAGMUX2.all  &= ~0x803C0000UL;
    GpioCtrlRegs.GPAMUX2.all   &= ~0x803C0000UL;
    GpioCtrlRegs.GPADIR.all    |= 0x803C0CFFUL;
    GpioDataRegs.GPASET.all     = 0x803C0CFFUL;

    //GPB
    GpioCtrlRegs.GPBPUD.all    &= ~0x00C00C00UL;  
    GpioCtrlRegs.GPBPUD.all    |= 0xE0200004UL;   
    GpioCtrlRegs.GPBQSEL1.all  |= 0x00000C00UL;
    GpioCtrlRegs.GPBQSEL2.all  |= 0x0000E0C0UL;
    GpioCtrlRegs.GPBGMUX1.all  &= ~0x00000C04UL;
    GpioCtrlRegs.GPBMUX1.all   &= ~0x00F00030UL;
    GpioCtrlRegs.GPBGMUX2.all  &= ~0xE0E00000UL;
    GpioCtrlRegs.GPBMUX2.all   &= ~0xFC00FC00UL;
    GpioCtrlRegs.GPBGMUX1.all  |= 0x00000C00UL;
    GpioCtrlRegs.GPBMUX1.all   |= 0x00A00000UL;
    GpioCtrlRegs.GPBGMUX2.all  |= 0x00C00000UL;
    GpioCtrlRegs.GPBMUX2.all   |= 0x0000A000UL;
    GpioCtrlRegs.GPBDIR.all    &= ~0xE0800000UL;
    GpioCtrlRegs.GPBDIR.all    |= 0x00600004UL;
    GpioDataRegs.GPBSET.all     = 0x00200004UL;

    //GPC
    GpioCtrlRegs.GPCPUD.all    &= ~0x003FF000UL;  
    GpioCtrlRegs.GPCPUD.all    |= 0x7F000009UL;   
    GpioCtrlRegs.GPCQSEL1.all  |= 0x0000F001UL;
    GpioCtrlRegs.GPCQSEL2.all  |= 0x0000002FUL;
    GpioCtrlRegs.GPCGMUX1.all  &= ~0x0000F009UL;
    GpioCtrlRegs.GPCMUX1.all   &= ~0xFF0000C3UL;
    GpioCtrlRegs.GPCGMUX2.all  &= ~0x00007F3FUL;
    GpioCtrlRegs.GPCMUX2.all   &= ~0x3FFFFFFFUL;
    GpioCtrlRegs.GPCDIR.all    &= ~0x003FF001UL;
    GpioCtrlRegs.GPCDIR.all    |= 0x7F000008UL;
    GpioDataRegs.GPCSET.all     = 0x7F000000UL;
    GpioDataRegs.GPCCLEAR.all   = 0x00000008UL;

    //X-bar
    InputXbarRegs.INPUT7SELECT  = 61U;
    InputXbarRegs.INPUT1SELECT  = 62U;
    InputXbarRegs.INPUT2SELECT  = 63U;
    InputXbarRegs.INPUT3SELECT  = 64U;

    //TZ
    EPwm1Regs.TZCLR.bit.OST     = 1U;
    EPwm1Regs.TZCLR.bit.INT     = 1U;
    EPwm3Regs.TZCLR.bit.OST     = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm3Regs.TZCLR.bit.INT     = 1U;
    EPwm4Regs.TZCLR.bit.OST     = 1U;
    EPwm4Regs.TZCLR.bit.INT     = 1U;

    //按键消抖模块初始化
    GPIO_KeyDebounceInit();

    EDIS
}
#endif

//采用寄存器操作的方法
#if 0
void GPIO_Config(void)
{
    EALLOW;

    /* ============================================================
     * 一、功率级 PWM 输出（驱动逆变器与 Boost）
     * 对应引脚表：
     *   GPIO0  EPWM? A  Dri_INV_B1_L
     *   GPIO1  EPWM? B  Dri_INV_B2_L
     *   GPIO2  EPWM? A  Dri_INV_C1_L
     *   GPIO3  EPWM? B  Dri_INV_C2_L
     *   GPIO4  EPWM? A  Dri_BST1_L
     *   GPIO5  EPWM? B  Dri_BST2_L
     *   GPIO6  EPWM? A  Dri_BST3_L
     *   GPIO7  EPWM? B  Dri_BST4_L
     * ============================================================ */
    GpioCtrlRegs.GPAPUD.all |= 0x000000FFUL;          // 禁用 GPIO0~7 内部上拉
    GpioCtrlRegs.GPAGMUX1.all &= ~0x0000FFFFUL;       // 清 GMUX0~7
    GpioCtrlRegs.GPAMUX1.all &= ~0x0000FFFFUL;        // 清 MUX0~7
    /* GPIO0~7 复用为 EPWMxA/EPWMxB（功能1） */
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1U;              // Dri_INV_B1_L
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1U;              // Dri_INV_B2_L
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1U;              // Dri_INV_C1_L
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1U;              // Dri_INV_C2_L
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1U;              // Dri_BST1_L
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1U;              // Dri_BST2_L
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1U;              // Dri_BST3_L
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1U;              // Dri_BST4_L
    GpioCtrlRegs.GPADIR.all |= 0x000000FFUL;          // GPIO0~7 设为输出
    GpioDataRegs.GPASET.all = 0x000000FFUL;           // 初始输出高（低有效驱动，释放状态）

    /* ============================================================
     * 二、蜂鸣器输出
     *   GPIO8  EPWM? A  BEEP
     * ============================================================ */
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1U;               // 禁用内部上拉
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0U;             // 清 GMUX
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1U;              // 复用为 EPWM5A（功能1）

    /* ============================================================
     * 三、数字输入（按键、开关、状态检测）
     * 对应引脚表：
     *   GPIO76  GPIO IN  OVP_BUS_Fin
     *   GPIO77  GPIO IN  FAN_State_L
     *   GPIO78  GPIO IN  POWER_SW3
     *   GPIO79  GPIO IN  Key4_L
     *   GPIO80  GPIO IN  POWER_SW2
     *   GPIO81  GPIO IN  Key3_L
     *   GPIO82  GPIO IN  POWER_SW1
     *   GPIO83  GPIO IN  Key2_L
     *   GPIO85  GPIO IN  Key1_L
     * 说明：按下接地，故使能内部上拉（PUD=0），QSEL=3 消抖。
     * ============================================================ */
    GpioCtrlRegs.GPCPUD.bit.GPIO76 = 0U;              // OVP_BUS_Fin
    GpioCtrlRegs.GPCPUD.bit.GPIO77 = 0U;              // FAN_State_L
    GpioCtrlRegs.GPCPUD.bit.GPIO78 = 0U;              // POWER_SW3
    GpioCtrlRegs.GPCPUD.bit.GPIO79 = 0U;              // Key4_L
    GpioCtrlRegs.GPCPUD.bit.GPIO80 = 0U;              // POWER_SW2
    GpioCtrlRegs.GPCPUD.bit.GPIO81 = 0U;              // Key3_L
    GpioCtrlRegs.GPCPUD.bit.GPIO82 = 0U;              // POWER_SW1
    GpioCtrlRegs.GPCPUD.bit.GPIO83 = 0U;              // Key2_L
    GpioCtrlRegs.GPCPUD.bit.GPIO85 = 0U;              // Key1_L

    GpioCtrlRegs.GPCGMUX1.bit.GPIO76 = 0U;            // 通用 GPIO
    GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 0U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO77 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 0U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO78 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 0U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO79 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO80 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO81 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO82 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO83 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0U;

    GpioCtrlRegs.GPCDIR.bit.GPIO76 = 0U;              // 输入
    GpioCtrlRegs.GPCDIR.bit.GPIO77 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO78 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO79 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO80 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO81 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO82 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO83 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO85 = 0U;

    GpioCtrlRegs.GPCQSEL1.bit.GPIO76 = 3U;            // 仅同步到 SYSCLK（消抖）
    GpioCtrlRegs.GPCQSEL1.bit.GPIO77 = 3U;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO78 = 3U;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO79 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO80 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO81 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO82 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO83 = 3U;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO85 = 3U;

    /* ============================================================
     * 四、数字输出（LED、继电器、风扇、EEPROM 写保护等）
     * 信号名带 _L 表示低有效，初始置高 = 释放。
     * ============================================================ */

    /* --- GPIO53  FAN_PWM_L --- */
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 1U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO53 = 0U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1U;
    GpioDataRegs.GPBSET.bit.GPIO53 = 1U;              // FAN_PWM_L 释放（高）

    /* --- GPIO31  LED1 --- */
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0U;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1U;
    GpioDataRegs.GPASET.bit.GPIO31 = 1U;              // LED1 灭（高）

    /* --- GPIO34  LED2 --- */
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1U;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0U;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1U;
    GpioDataRegs.GPBSET.bit.GPIO34 = 1U;              // LED2 灭（高）

    /* --- GPIO67  I2CA_WC (EEPROM 写保护) --- */
    GpioCtrlRegs.GPCPUD.bit.GPIO67 = 1U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO67 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1U;
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1U;            // 置低：写使能（允许 EEPROM 写入）

    /* --- GPIO88~94 共 7 路低有效输出 ---
     * GPIO88  BST_OFF_L
     * GPIO89  INV_OFF_L
     * GPIO90  Grid_Relay1_L
     * GPIO91  ISO_Relay1_L
     * GPIO92  Grid_Relay2_L
     * GPIO93  ISO_Relay2_L
     * GPIO94  Grid_Relay3_L
     */
    GpioCtrlRegs.GPCPUD.bit.GPIO88 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO89 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO90 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO91 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO92 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO93 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO94 = 1U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO88 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO88 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO89 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO89 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO90 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO90 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO91 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO91 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO92 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO92 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO93 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO93 = 0U;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO94 = 0U;
    GpioCtrlRegs.GPCMUX2.bit.GPIO94 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO88 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO89 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO90 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO91 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO92 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO93 = 1U;
    GpioCtrlRegs.GPCDIR.bit.GPIO94 = 1U;
    /* 批量置高：GPIO88~94 全部释放（低有效） */
    GpioDataRegs.GPCSET.all = (1UL << 24U) | (1UL << 25U) | (1UL << 26U) |
                               (1UL << 27U) | (1UL << 28U) | (1UL << 29U) |
                               (1UL << 30U);

    /* --- GPIO10/11  继电器控制 ---
     * GPIO10  Grid_Relay4_L
     * GPIO11  Grid_Relay_OFF_L
     */
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1U;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0U;
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1U;
    GpioDataRegs.GPASET.bit.GPIO10 = 1U;              // Grid_Relay4_L 释放
    GpioDataRegs.GPASET.bit.GPIO11 = 1U;              // Grid_Relay_OFF_L 释放

    /* --- GPIO18~21  状态/控制输出 ---
     * GPIO18  SPS_SD_L
     * GPIO19  GFCI_Check_L
     * GPIO20  Chock_Temp_SW_L
     * GPIO21  DSP_State_L
     */
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1U;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1U;
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1U;
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0U;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 0U;
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0U;
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1U;
    GpioDataRegs.GPASET.all = (1UL << 18U) | (1UL << 19U) |
                               (1UL << 20U) | (1UL << 21U);   // 全部释放（高）

    /* ============================================================
     * 五、通信接口
     * ============================================================ */

    /* --- SCIB：GPIO54=SCITXD (DSPtoPC_TX)，GPIO55=SCIRXD (DSPtoPC_RX) --- */
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0U;              // 使能上拉
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0U;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 3U;            // 同步到 SYSCLK
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 1U;            // GMUX=1, MUX=2 → SCIB 功能
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 2U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 1U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 2U;
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1U;              // TX 输出
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0U;              // RX 输入

    /* --- I2CA：GPIO42=SDA (I2C_SDA)，GPIO43=SCL (I2C_SCL) --- */
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0U;              // 使能上拉
    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0U;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO42 = 3U;            // 同步到 SYSCLK
    GpioCtrlRegs.GPBQSEL1.bit.GPIO43 = 3U;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO42 = 1U;            // GMUX=1, MUX=2 → I2CA 功能
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 2U;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO43 = 1U;
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 2U;

    /* ============================================================
     * 六、输入捕获（ECAP）
     *   GPIO61  XBAR-ECAP  ZCT_Grid_Fin
     * 通过 INPUT X-BAR 7 路由到 ECAP1
     * ============================================================ */
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1U;              // 禁用上拉
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3U;            // 同步到 SYSCLK
    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 0U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 0U;              // 输入
    InputXbarRegs.INPUT7SELECT = 61U;                 // GPIO61 → INPUT X-BAR 7

    /* ============================================================
     * 七、保护输入（Trip Zone）
     *   GPIO62  XBAR-TZ1  OCP_PV1_Fin
     *   GPIO63  XBAR-TZ2  OCP_PV2_Fin
     *   GPIO64  XBAR-TZ3  OCP_Grid_Fin
     * 通过 INPUT X-BAR 1/2/3 路由到 TZ1/2/3，硬件直接关断 PWM。
     * ============================================================ */
    GpioCtrlRegs.GPBPUD.bit.GPIO62 = 1U;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 1U;
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 1U;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3U;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3U;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO62 = 0U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0U;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0U;
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0U;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0U;
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO62 = 0U;
    GpioCtrlRegs.GPBDIR.bit.GPIO63 = 0U;
    GpioCtrlRegs.GPCDIR.bit.GPIO64 = 0U;
    InputXbarRegs.INPUT1SELECT = 62U;                 // GPIO62 → INPUT X-BAR 1 → TZ1
    InputXbarRegs.INPUT2SELECT = 63U;                 // GPIO63 → INPUT X-BAR 2 → TZ2
    InputXbarRegs.INPUT3SELECT = 64U;                 // GPIO64 → INPUT X-BAR 3 → TZ3

    /* ============================================================
     * 八、清除启动时可能产生的 TZ 误触发标志
     * ============================================================ */
    EPwm1Regs.TZCLR.bit.OST = 1U;
    EPwm1Regs.TZCLR.bit.INT = 1U;
    EPwm3Regs.TZCLR.bit.OST = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm3Regs.TZCLR.bit.INT = 1U;
    EPwm4Regs.TZCLR.bit.OST = 1U;
    EPwm4Regs.TZCLR.bit.INT = 1U;

    GPIO_KeyDebounceInit();

    EDIS;
}
#endif

//采用driver库的方法
#if 0
void GPIO_Config(void)
{
    EALLOW;

    /* ============================================================
     * 一、功率级 PWM 输出 + 蜂鸣器
     * ------------------------------------------------------------
     * GPIO0~7  → EPWMxA/EPWMxB（逆变器 + 双 Boost）
     * GPIO8    → EPWM5A（蜂鸣器 BEEP）
     * 复用索引：1（EPWM 功能）
     * ============================================================ */
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);   /* Dri_INV_B1_L  → EPWM1A */
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 1);   /* Dri_INV_B2_L  → EPWM1B */
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);   /* Dri_INV_C1_L  → EPWM2A */
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);   /* Dri_INV_C2_L  → EPWM2B */
    GPIO_SetupPinOptions(3, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 1);   /* Dri_BST1_L    → EPWM3A */
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 1);   /* Dri_BST2_L    → EPWM3B */
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 1);   /* Dri_BST3_L    → EPWM4A */
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 1);   /* Dri_BST4_L    → EPWM4B */
    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 1);   /* BEEP          → EPWM5A */
    GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);

    /* 上电初始电平：GPIO0~7 置高（低有效驱动，释放 = 关断功率管） */
    GpioDataRegs.GPASET.all = 0x000000FFUL;

    /* ============================================================
     * 二、数字输入（按键、开关、状态检测）
     * ------------------------------------------------------------
     * 复用索引：0（通用 GPIO）
     * 配置：输入 + 内部上拉 + 3 次采样量化（消抖）
     * ============================================================ */
    GPIO_SetupPinMux(76, GPIO_MUX_CPU1, 0);     /*GPIO76  → OVP_BUS_Fin*/
    GPIO_SetupPinOptions(76, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL3);

    GPIO_SetupPinMux(77, GPIO_MUX_CPU1, 0);     /*GPIO77  → FAN_State_L*/
    GPIO_SetupPinOptions(77, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL3);

    GPIO_SetupPinMux(78, GPIO_MUX_CPU1, 0);     /*GPIO78  → POWER_SW3*/
    GPIO_SetupPinOptions(78, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL3);

    GPIO_SetupPinMux(79, GPIO_MUX_CPU1, 0);     /*GPIO79  → Key4_L*/
    GPIO_SetupPinOptions(79, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL3);

    GPIO_SetupPinMux(80, GPIO_MUX_CPU1, 0);     /*GPIO80  → POWER_SW2*/
    GPIO_SetupPinOptions(80, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL3);

    GPIO_SetupPinMux(81, GPIO_MUX_CPU1, 0);     /*GPIO81  → Key3_L*/
    GPIO_SetupPinOptions(81, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL3);

    GPIO_SetupPinMux(82, GPIO_MUX_CPU1, 0);     /*GPIO82  → POWER_SW1*/
    GPIO_SetupPinOptions(82, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL3);

    GPIO_SetupPinMux(83, GPIO_MUX_CPU1, 0);     /*GPIO83  → Key2_L*/
    GPIO_SetupPinOptions(83, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL3);

    GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 0);     /*GPIO85  → Key1_L*/
    GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL3);


    /* ============================================================
     * 三、数字输出（LED、继电器、风扇、EEPROM 写保护等）
     * ------------------------------------------------------------
     * 信号名带 _L 表示低有效，初始置高 = 释放。
     * 复用索引：0（通用 GPIO）
     * ============================================================ */
    GPIO_SetupPinMux(53, GPIO_MUX_CPU1, 0);     /*GPIO53  → FAN_PWM_L*/
    GPIO_SetupPinOptions(53, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO53 = 1U;
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);     /*GPIO31  → LED1*/
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1U;  
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);     /*GPIO34  → LED2*/
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1U;

    /* --- GPIO67  I2CA_WC（EEPROM 写保护，低 = 写使能） --- */
    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1U;

    GPIO_SetupPinMux(88, GPIO_MUX_CPU1, 0);   /*GPIO88 → BST_OFF_L */
    GPIO_SetupPinOptions(88, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(89, GPIO_MUX_CPU1, 0);   /* GPIO89 → INV_OFF_L */
    GPIO_SetupPinOptions(89, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(90, GPIO_MUX_CPU1, 0);   /*GPIO90 → Grid_Relay1_L */
    GPIO_SetupPinOptions(90, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(91, GPIO_MUX_CPU1, 0);   /*GPIO91 → ISO_Relay1_L */
    GPIO_SetupPinOptions(91, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(92, GPIO_MUX_CPU1, 0);   /*GPIO92 → Grid_Relay2_L */
    GPIO_SetupPinOptions(92, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(93, GPIO_MUX_CPU1, 0);   /*GPIO93 → ISO_Relay2_L */
    GPIO_SetupPinOptions(93, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);   /*GPIO94 → Grid_Relay3_L */
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.all = 0x7F000000UL;/* 批量置高 GPIO88~94（低有效释放） */

    /* --- GPIO10/11  继电器控制 --- */
    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 0);   /* Grid_Relay4_L */
    GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 0);   /* Grid_Relay_OFF_L */
    GPIO_SetupPinOptions(11, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO10 = 1U;
    GpioDataRegs.GPASET.bit.GPIO11 = 1U;

    /* --- GPIO18~21  状态/控制输出 --- */
    GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 0);   /* SPS_SD_L */
    GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);   /* GFCI_Check_L */
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 0);   /* Chock_Temp_SW_L */
    GPIO_SetupPinOptions(20, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 0);   /* DSP_State_L */
    GPIO_SetupPinOptions(21, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.all = (1UL << 18U) | (1UL << 19U) |
                               (1UL << 20U) | (1UL << 21U);

    /* ============================================================
     * 四、通信接口
     * ------------------------------------------------------------
     * SCIB: GPIO54(TX) / GPIO55(RX)  → 复用索引 2
     * I2CA: GPIO42(SDA) / GPIO43(SCL) → 复用索引 6
     * ============================================================ */
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 2);   /* SCITXD → DSPtoPC_TX */
    GPIO_SetupPinOptions(54, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 2);   /* SCIRXD → DSPtoPC_RX */
    GPIO_SetupPinOptions(55, GPIO_INPUT, GPIO_ASYNC | GPIO_PULLUP);

    GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 6);   /* SDAA → I2C_SDA */
    GPIO_SetupPinOptions(42, GPIO_INPUT, GPIO_OPENDRAIN | GPIO_PULLUP | GPIO_QUAL3);
    GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 6);   /* SCLA → I2C_SCL */
    GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_OPENDRAIN | GPIO_PULLUP | GPIO_QUAL3);

    /* ============================================================
     * 五、输入捕获（ECAP）
     * ------------------------------------------------------------
     * GPIO61 → XBAR-ECAP → ZCT_Grid_Fin（电网过零检测）
     * 复用索引：0（通用 GPIO，通过 X-BAR 路由）
     * ============================================================ */
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_INPUT, GPIO_QUAL3);
    InputXbarRegs.INPUT7SELECT = 61U;          /* GPIO61 → X-BAR 7 → ECAP1 */

    /* ============================================================
     * 六、保护输入（Trip Zone）
     * ------------------------------------------------------------
     * GPIO62 → XBAR-TZ1 → OCP_PV1_Fin（PV1 过流）
     * GPIO63 → XBAR-TZ2 → OCP_PV2_Fin（PV2 过流）
     * GPIO64 → XBAR-TZ3 → OCP_Grid_Fin（电网过流）
     * 复用索引：0（通用 GPIO，通过 X-BAR 路由到 TZ）
     * ============================================================ */
    GPIO_SetupPinMux(62, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(62, GPIO_INPUT, GPIO_QUAL3);
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(63, GPIO_INPUT, GPIO_QUAL3);
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(64, GPIO_INPUT, GPIO_QUAL3);

    InputXbarRegs.INPUT1SELECT = 62U;          /* GPIO62 → X-BAR 1 → TZ1 */
    InputXbarRegs.INPUT2SELECT = 63U;          /* GPIO63 → X-BAR 2 → TZ2 */
    InputXbarRegs.INPUT3SELECT = 64U;          /* GPIO64 → X-BAR 3 → TZ3 */

    /* ============================================================
     * 七、清除启动时可能产生的 TZ 误触发标志
     * ============================================================ */
    EPwm1Regs.TZCLR.bit.OST = 1U;
    EPwm1Regs.TZCLR.bit.INT = 1U;
    EPwm3Regs.TZCLR.bit.OST = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST1 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST2 = 1U;
    EPwm3Regs.TZOSTCLR.bit.OST3 = 1U;
    EPwm3Regs.TZCLR.bit.INT = 1U;
    EPwm4Regs.TZCLR.bit.OST = 1U;
    EPwm4Regs.TZCLR.bit.INT = 1U;

    /* 按键消抖模块初始化 */
    GPIO_KeyDebounceInit();

    EDIS;
}
#endif

//很简单的读电平函数
static Uint16 GPIO_ReadKeyPressed(Uint16 keyNumber)
{
    Uint16 isPressed = 0U;
    switch(keyNumber)
    {
        case KEY_NUMBER_1: isPressed = (GpioDataRegs.GPCDAT.bit.GPIO85 == 0U) ? 1U : 0U; break;
        case KEY_NUMBER_2: isPressed = (GpioDataRegs.GPCDAT.bit.GPIO83 == 0U) ? 1U : 0U; break;
        case KEY_NUMBER_3: isPressed = (GpioDataRegs.GPCDAT.bit.GPIO81 == 0U) ? 1U : 0U; break;
        case KEY_NUMBER_4: isPressed = (GpioDataRegs.GPCDAT.bit.GPIO79 == 0U) ? 1U : 0U; break;
        default: isPressed = 0U; break;
    }
    return isPressed;
}

//把每个键的"稳定状态"和"上次值"都初始化成当前实际电平,防止报出一个假的的按下事件

static void GPIO_KeyDebounceInit(void)
{
    Uint16 keyIndex;
    Uint16 pressed;
    //初始化四个按键
    for(keyIndex = 0U; keyIndex < GPIO_KEY_COUNT; keyIndex++)
    {
        pressed = GPIO_ReadKeyPressed((Uint16)(keyIndex + 1U));
        //稳定状态 = 当前值
        GPIO_KeyStablePressed[keyIndex] = pressed;
        //上次值 = 当前值
        GPIO_KeyLastPressed[keyIndex] = pressed;
        //清零计数
        GPIO_KeyDebounceCount[keyIndex] = 0U;
    }
    GPIO_KeyPendingEvents = 0U;
}

//正式的按键检测
Uint16 GPIO_GetKeyEvents(void)
{
    Uint16 keyIndex;
    Uint16 pressed;
    //按键检测结果
    Uint16 eventMask = 0U;
    //依次读取四个按键状态
    for(keyIndex = 0U; keyIndex < GPIO_KEY_COUNT; keyIndex++)
    {
        //读取此时按键状态, 会呈持续返回0 or 1
        pressed = GPIO_ReadKeyPressed((Uint16)(keyIndex + 1U));
        //如果按键此时保持稳定状态
        if(pressed == GPIO_KeyStablePressed[keyIndex])
        {
            //清除计数
            GPIO_KeyDebounceCount[keyIndex] = 0U;
            //更新上一次按键数据
            GPIO_KeyLastPressed[keyIndex] = pressed;
        }
        //如果按键此时保持上一次状态,就值得注意了
        else if(pressed == GPIO_KeyLastPressed[keyIndex])
        {
            //保持时间不够的话,接着计数
            if(GPIO_KeyDebounceCount[keyIndex] < GPIO_KEY_DEBOUNCE_SAMPLES)
            {
                GPIO_KeyDebounceCount[keyIndex]++;
            }
            //如果20ms内连续检测到按键
            if(GPIO_KeyDebounceCount[keyIndex] >= GPIO_KEY_DEBOUNCE_SAMPLES)
            {
                GPIO_KeyStablePressed[keyIndex] = pressed;
                GPIO_KeyDebounceCount[keyIndex] = 0U;
                //记录已经按下且稳定的key
                if(pressed != 0U)
                {
                    eventMask |= GPIO_KeyEventMasks[keyIndex];
                }
            }
        }
        //这一个情况适用于突然检测到按键
        else
        {
            GPIO_KeyLastPressed[keyIndex] = pressed;
            GPIO_KeyDebounceCount[keyIndex] = 1U;
        }
    }
    //返回最终的按键结果
    GPIO_KeyPendingEvents |= eventMask;
    eventMask = GPIO_KeyPendingEvents;
    GPIO_KeyPendingEvents = 0U;
    return eventMask;
}
