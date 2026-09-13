# GPIO 配置掩码说明文档

---

## 一、GPIOA 端口（GPIO0~31）

### 本端口引脚用途一览

| GPIO | 功能 | 用途 |
| :--- | :--- | :--- |
| GPIO0 | EPWM A | Dri_INV_B1_L |
| GPIO1 | EPWM B | Dri_INV_B2_L |
| GPIO2 | EPWM A | Dri_INV_C1_L |
| GPIO3 | EPWM B | Dri_INV_C2_L |
| GPIO4 | EPWM A | Dri_BST1_L |
| GPIO5 | EPWM B | Dri_BST2_L |
| GPIO6 | EPWM A | Dri_BST3_L |
| GPIO7 | EPWM B | Dri_BST4_L |
| GPIO8 | EPWM5A | BEEP |
| GPIO10 | GPIO OUT | Grid_Relay4_L |
| GPIO11 | GPIO OUT | Grid_Relay_OFF_L |
| GPIO18 | GPIO OUT | SPS_SD_L |
| GPIO19 | GPIO OUT | GFCI_Check_L |
| GPIO20 | GPIO OUT | Chock_Temp_SW_L |
| GPIO21 | GPIO OUT | DSP_State_L |
| GPIO31 | GPIO OUT | LED1 |

### 位映射

> **GPIOA 寄存器位号 = GPIO 编号**
> （位0对应GPIO0，位31对应GPIO31）

### 掩码汇总表

| 寄存器 | 操作 | 掩码 | 说明 |
| :--- | :--- | :--- | :--- |
| **GPAPUD** | \|= | `0x803C0DFF`<br>`1000 0000 0011 1100 0000 1101 1111 1111` | 禁用上拉：位31/21~18/11~10/8~0 |
| **GPAGMUX1** | &= ~ | `0x00000DFF`<br>`0000 0000 0000 0000 0000 1101 1111 1111` | 清GMUX：位11~10/8~0 |
| **GPAMUX1** | &= ~ | `0x00000DFF`<br>`0000 0000 0000 0000 0000 1101 1111 1111` | 清MUX：位11~10/8~0 |
| **GPAMUX1** | \|= | `0x000001FF`<br>`0000 0000 0000 0000 0000 0001 1111 1111` | 设MUX=1：位8~0（GPIO0~8设为EPWM功能） |
| **GPAGMUX2** | &= ~ | `0x803C0000`<br>`1000 0000 0011 1100 0000 0000 0000 0000` | 清GMUX：位31/21~18 |
| **GPAMUX2** | &= ~ | `0x803C0000`<br>`1000 0000 0011 1100 0000 0000 0000 0000` | 清MUX：位31/21~18 |
| **GPADIR** | \|= | `0x803C0DFF`<br>`1000 0000 0011 1100 0000 1101 1111 1111` | 设输出：位31/21~18/11~10/8~0 |
| **GPASET** | = | `0x803C0DFF`<br>`1000 0000 0011 1100 0000 1101 1111 1111` | 初始置高：位31/21~18/11~10/8~0 |

---

## 二、GPIOB 端口（GPIO32~63）

### 本端口引脚用途一览

| GPIO | 功能 | 用途 |
| :--- | :--- | :--- |
| GPIO34 | GPIO OUT | LED2 |
| GPIO42 | I2CA SDA | I2C_SDA |
| GPIO43 | I2CA SCL | I2C_SCL |
| GPIO53 | GPIO OUT | FAN_PWM_L |
| GPIO54 | SCIB TX | DSPtoPC_TX |
| GPIO55 | SCIB RX | DSPtoPC_RX |
| GPIO61 | XBAR-ECAP | ZCT_Grid_Fin |
| GPIO62 | XBAR-TZ1 | OCP_PV1_Fin |
| GPIO63 | XBAR-TZ2 | OCP_PV2_Fin |

### 位映射

> **GPIOB 寄存器位号 = GPIO 编号 - 32**
> （GPIO32→位0，GPIO34→位2，GPIO42→位10，GPIO53→位21，GPIO63→位31）

### 掩码汇总表

| 寄存器 | 操作 | 掩码 | 说明 |
| :--- | :--- | :--- | :--- |
| **GPBPUD** | &= ~ | `0x00C00C00`<br>`0000 0000 1100 0000 0000 1100 0000 0000` | 使能上拉：位23/22/11/10（GPIO55/54/43/42） |
| **GPBPUD** | \|= | `0xE0200004`<br>`1110 0000 0010 0000 0000 0000 0000 0100` | 禁用上拉：位31/30/29/21/2（GPIO63/62/61/53/34） |
| **GPBQSEL1** | \|= | `0x00000C00`<br>`0000 0000 0000 0000 0000 1100 0000 0000` | QSEL=3：位11/10（GPIO43/42） |
| **GPBQSEL2** | \|= | `0x0000E0C0`<br>`0000 0000 0000 0000 1110 0000 1100 0000` | QSEL=3：位15/14/13/7/6（GPIO61/62/63/54/55） |
| **GPBGMUX1** | &= ~ | `0x00000C04`<br>`0000 0000 0000 0000 0000 1100 0000 0100` | 清GMUX：位11/10/2（GPIO43/42/34） |
| **GPBMUX1** | &= ~ | `0x00F00030`<br>`0000 0000 1111 0000 0000 0000 0011 0000` | 清MUX：位23~20/5~4（GPIO43/42/34） |
| **GPBGMUX2** | &= ~ | `0xE0E00000`<br>`1110 0000 1110 0000 0000 0000 0000 0000` | 清GMUX：位31~29/23~21（GPIO63~61/55~53） |
| **GPBMUX2** | &= ~ | `0xFC00FC00`<br>`1111 1100 0000 0000 1111 1100 0000 0000` | 清MUX：位31~26/15~10（GPIO63~61/55~53） |
| **GPBGMUX1** | \|= | `0x00000C00`<br>`0000 0000 0000 0000 0000 1100 0000 0000` | GMUX=1：位11/10（GPIO43/42） |
| **GPBMUX1** | \|= | `0x00A00000`<br>`0000 0000 1010 0000 0000 0000 0000 0000` | MUX=2：位23/21（GPIO43/42） |
| **GPBGMUX2** | \|= | `0x00C00000`<br>`0000 0000 1100 0000 0000 0000 0000 0000` | GMUX=1：位23/22（GPIO55/54） |
| **GPBMUX2** | \|= | `0x0000A000`<br>`0000 0000 0000 0000 1010 0000 0000 0000` | MUX=2：位15/13（GPIO55/54） |
| **GPBDIR** | &= ~ | `0xE0800000`<br>`1110 0000 1000 0000 0000 0000 0000 0000` | 设输入：位31/30/29/23（GPIO63/62/61/55） |
| **GPBDIR** | \|= | `0x00600004`<br>`0000 0000 0110 0000 0000 0000 0000 0100` | 设输出：位22/21/2（GPIO54/53/34） |
| **GPBSET** | = | `0x00200004`<br>`0000 0000 0010 0000 0000 0000 0000 0100` | 初始置高：位21/2（GPIO53/34） |

---

## 三、GPIOC 端口（GPIO64~95）

### 本端口引脚用途一览

| GPIO | 功能 | 用途 |
| :--- | :--- | :--- |
| GPIO64 | XBAR-TZ3 | OCP_Grid_Fin |
| GPIO67 | GPIO OUT | I2CA_WC（EEPROM写保护） |
| GPIO76 | GPIO IN | OVP_BUS_Fin |
| GPIO77 | GPIO IN | FAN_State_L |
| GPIO78 | GPIO IN | POWER_SW3 |
| GPIO79 | GPIO IN | Key4_L |
| GPIO80 | GPIO IN | POWER_SW2 |
| GPIO81 | GPIO IN | Key3_L |
| GPIO82 | GPIO IN | POWER_SW1 |
| GPIO83 | GPIO IN | Key2_L |
| GPIO85 | GPIO IN | Key1_L |
| GPIO88 | GPIO OUT | BST_OFF_L |
| GPIO89 | GPIO OUT | INV_OFF_L |
| GPIO90 | GPIO OUT | Grid_Relay1_L |
| GPIO91 | GPIO OUT | ISO_Relay1_L |
| GPIO92 | GPIO OUT | Grid_Relay2_L |
| GPIO93 | GPIO OUT | ISO_Relay2_L |
| GPIO94 | GPIO OUT | Grid_Relay3_L |

### 位映射

> **GPIOC 寄存器位号 = GPIO 编号 - 64**
> （GPIO64→位0，GPIO67→位3，GPIO76→位12，GPIO85→位21，GPIO88→位24，GPIO94→位30）

### 掩码汇总表

| 寄存器 | 操作 | 掩码 | 说明 |
| :--- | :--- | :--- | :--- |
| **GPCPUD** | &= ~ | `0x003FF000`<br>`0000 0000 0011 1111 1111 0000 0000 0000` | 使能上拉：位21~12（GPIO85~76） |
| **GPCPUD** | \|= | `0x7F000009`<br>`0111 1111 0000 0000 0000 0000 0000 1001` | 禁用上拉：位30~24/3/0（GPIO94~88/67/64） |
| **GPCQSEL1** | \|= | `0x0000F001`<br>`0000 0000 0000 0000 1111 0000 0000 0001` | QSEL=3：位15~12/0（GPIO79~76/64） |
| **GPCQSEL2** | \|= | `0x0000002F`<br>`0000 0000 0000 0000 0000 0000 0010 1111` | QSEL=3：位5/3~0（GPIO85/83~80） |
| **GPCGMUX1** | &= ~ | `0x0000F009`<br>`0000 0000 0000 0000 1111 0000 0000 1001` | 清GMUX：位15~12/3/0（GPIO79~76/67/64） |
| **GPCMUX1** | &= ~ | `0xFF0000C3`<br>`1111 1111 0000 0000 0000 0000 1100 0011` | 清MUX：位31~24/7~6/1~0（GPIO79~76/67/64） |
| **GPCGMUX2** | &= ~ | `0x00007F3F`<br>`0000 0000 0000 0000 0111 1111 0011 1111` | 清GMUX：位14~8/5~0（GPIO94~88/85~80） |
| **GPCMUX2** | &= ~ | `0x3FFFFFFF`<br>`0011 1111 1111 1111 1111 1111 1111 1111` | 清MUX：位29~16/11~0（GPIO94~88/85~80） |
| **GPCDIR** | &= ~ | `0x003FF001`<br>`0000 0000 0011 1111 1111 0000 0000 0001` | 设输入：位21~12/0（GPIO85~76/64） |
| **GPCDIR** | \|= | `0x7F000008`<br>`0111 1111 0000 0000 0000 0000 0000 1000` | 设输出：位30~24/3（GPIO94~88/67） |
| **GPCSET** | = | `0x7F000000`<br>`0111 1111 0000 0000 0000 0000 0000 0000` | 初始置高：位30~24（GPIO94~88） |
| **GPCCLEAR** | = | `0x00000008`<br>`0000 0000 0000 0000 0000 0000 0000 1000` | 初始置低：位3（GPIO67，EEPROM写使能） |

---

## 四、X-BAR 路由 + TZ 清除 + 收尾

### X-BAR 路由表

| 寄存器 | 值 | 路径 |
| :--- | :--- | :--- |
| INPUT1SELECT | 62 | GPIO62 → X-BAR 1 → TZ1（PV1过流） |
| INPUT2SELECT | 63 | GPIO63 → X-BAR 2 → TZ2（PV2过流） |
| INPUT3SELECT | 64 | GPIO64 → X-BAR 3 → TZ3（电网过流） |
| INPUT7SELECT | 61 | GPIO61 → X-BAR 7 → ECAP1（电网过零） |

### TZ 标志清除

| 寄存器 | 操作 | 说明 |
| :--- | :--- | :--- |
| EPwm1Regs.TZCLR.bit.OST | = 1 | 清 EPWM1 一次性故障锁存 |
| EPwm1Regs.TZCLR.bit.INT | = 1 | 清 EPWM1 跳闸中断标志 |
| EPwm3Regs.TZCLR.bit.OST | = 1 | 清 EPWM3 一次性故障锁存 |
| EPwm3Regs.TZOSTCLR.bit.OST1 | = 1 | 清 EPWM3 的 OST1 |
| EPwm3Regs.TZOSTCLR.bit.OST2 | = 1 | 清 EPWM3 的 OST2 |
| EPwm3Regs.TZOSTCLR.bit.OST3 | = 1 | 清 EPWM3 的 OST3 |
| EPwm3Regs.TZCLR.bit.INT | = 1 | 清 EPWM3 跳闸中断标志 |
| EPwm4Regs.TZCLR.bit.OST | = 1 | 清 EPWM4 一次性故障锁存 |
| EPwm4Regs.TZCLR.bit.INT | = 1 | 清 EPWM4 跳闸中断标志 |

### 收尾

- 调用 `GPIO_KeyDebounceInit()` 初始化按键消抖。
- `EDIS` 关闭寄存器保护。

---

## 五、配置规律速查表

| 引脚类型 | PUD | QSEL | GMUX/MUX | DIR | 初始电平 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| EPWM 输出 | 禁用(1) | 默认 | MUX=1 | 输出 | SET 置高 |
| GPIO 输入 | 使能(0) | 3 | 0/0 | 输入 | — |
| GPIO 输出 | 禁用(1) | 默认 | 0/0 | 输出 | SET 或 CLEAR |
| SCI/I2C | 使能(0) | 3 | GMUX=1,MUX=2 | 按方向 | — |
| ECAP 输入 | 禁用(1) | 3 | 0/0 | 输入 | — |
| TZ 保护输入 | 禁用(1) | 3 | 0/0 | 输入 | — |
