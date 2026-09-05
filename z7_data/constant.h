#ifndef CONSTANT_H_
#define CONSTANT_H_

#define MATH_PI_F           3.14159265358979323846f
#define MATH_HALF_PI_F      (0.5f * MATH_PI_F)
#define MATH_TWO_PI_F       (2.0f * MATH_PI_F)
#define MATH_THREE_HALF_PI_F (1.5f * MATH_PI_F)
#define MATH_INV_TWO_PI_F   (1.0f / MATH_TWO_PI_F)

/* Supported inverter model identifiers. */
#define MODEL_3KW                       1U
#define MODEL_4KW                       2U

/* Fixed grid operating and protection thresholds. */
#define GRID_NOM_VOLT_RMS_V           220.0f
#define GRID_NOM_FREQ_HZ               50.0f
#define GRID_OV_TRIP_RMS_V            242.0f
#define GRID_UV_TRIP_RMS_V            187.0f
#define GRID_OF_TRIP_HZ                50.5f
#define GRID_UF_TRIP_HZ                49.5f
#define GRID_RECONN_MAX_RMS_V         242.0f
#define GRID_RECONN_MIN_RMS_V         187.0f
#define GRID_RECONN_MAX_FREQ_HZ        50.5f
#define GRID_RECONN_MIN_FREQ_HZ        49.5f

/* Fast ADC-domain grid-presence check. A 205-sample window is about one
 * half-cycle at the 20 kHz control rate used by this project. */
#define GRID_FAST_PRESENT_PEAK_V      180.0f
#define GRID_FAST_WINDOW_SAMPLES        205U

/* Common PV and DC-bus thresholds recovered from the legacy design. */
#define PV_PRESENT_MIN_V               30.0f
#define PV_START_V                    150.0f
#define PV_OV_TRIP_V                  550.0f
#define PV_OV_RECOVER_V               545.0f
#define MPPT_VOLT_STEP_V                1.0f
#define MPPT_VOLT_FINE_STEP_V           0.25f
#define MPPT_FAST_POWER_DELTA_W         19.0f
#define MPPT_CURRENT_LIMIT_MARGIN_A      0.05f

/* PV input modes retained for the future topology detector. The current
 * hardware uses two independent MPPT channels. */
#define MPPT_INPUT_NONE                 0U
#define MPPT_INPUT_PV1_ONLY             1U
#define MPPT_INPUT_PV2_ONLY             2U
#define MPPT_INPUT_DUAL                 3U
#define DC_BUS_MIN_V                  370.0f
#define DC_BUS_MAX_V                  430.0f
#define DC_BUS_OV_TRIP_V              580.0f
#define DC_VOLT_MARGIN_5_V              5.0f
#define DC_VOLT_MARGIN_8_V              8.0f
#define DC_VOLT_MARGIN_10_V            10.0f
#define DC_VOLT_MARGIN_18_V            18.0f
#define DC_VOLT_MARGIN_20_V            20.0f
#define DC_VOLT_MARGIN_25_V            25.0f
#define DC_VOLT_MARGIN_50_V            50.0f

/* Grid-relay self-test sequence (ms). 原工程用 2ms 一拍（wWaitTime），此处
 * 按 1 拍 = 2ms 换算为毫秒，独立于调度周期。 */
#define RELAY_SEQ_STEP1_MS          100U    /* 原 50 拍    合 relay1 */
#define RELAY_SEQ_STEP2_MS          600U    /* 原 300 拍   合 relay2+relay3 */
#define RELAY_SEQ_STEP3_MS         1600U    /* 原 800 拍   断 relay3 */
#define RELAY_SEQ_STEP4_MS         1800U    /* 原 900 拍   合 relay4 */
#define RELAY_SEQ_STEP5_MS         3400U    /* 原 1700 拍  断 relay2 */
#define RELAY_SEQ_STEP6_MS         3600U    /* 原 1800 拍  合 relay3 */
#define RELAY_SEQ_STEP7_MS         5400U    /* 原 2700 拍  断 relay1 */
#define RELAY_SEQ_STEP8_MS         5600U    /* 原 2800 拍  合 relay2 */
#define RELAY_SEQ_STEP9_MIN_MS     7600U    /* 原 3800 拍  最终合 relay1 */
#define RELAY_SEQ_STEP9_MAX_MS     7800U    /* 原 3900 拍 */
#define RELAY_SEQ_TIMEOUT_MS      10400U    /* 原 >5200 拍 超时全断 */

/* Relay self-test voltage-difference criteria（方案A：电压差判据）. */
#define RELAY_DELTA_V_TRIP_V        60.0f   /* 原 c60V 压差阈值 */
#define RELAY_FAULT_FILTER_MS      250U     /* 原 125 拍 连续判据 */

/* Relay check windows (ms，由原 2ms 拍换算). */
#define RELAY_WIN_A_MIN_MS         1100U    /* 原 550 拍 */
#define RELAY_WIN_A_MAX_MS         1600U    /* 原 800 拍 */
#define RELAY_WIN_B_MIN_MS         2300U    /* 原 1150 拍 */
#define RELAY_WIN_B_MAX_MS         2800U    /* 原 1400 拍 */
#define RELAY_WIN_C_MIN_MS         4100U    /* 原 2050 拍 */
#define RELAY_WIN_C_MAX_MS         4600U    /* 原 2300 拍 */
#define RELAY_WIN_D_MIN_MS         6100U    /* 原 3050 拍 */
#define RELAY_WIN_D_MAX_MS         6600U    /* 原 3300 拍 */
#define RELAY_WIN_E_MIN_MS         8400U    /* 原 4200 拍 失效检测窗 */
#define RELAY_WIN_E_MAX_MS         8900U    /* 原 4450 拍 */
#define RELAY_SEQ_DONE_MS          8900U    /* 检测窗口结束后可判通过 */

/* Initial DC-bus outer-loop settings. The gains are bring-up values and
 * should be retuned after the power stage and bus capacitance are verified. */
#define BUS_CTRL_PERIOD_S                0.01f
#define BUS_VOLT_REF_V                 400.0f
#define BUS_PI_KP                        0.002f
#define BUS_PI_KI                        0.02f
#define BUS_CURRENT_AMP_MIN_NORM         0.0f
#define BUS_CURRENT_AMP_MAX_NORM         1.0f

/* Initial PV-voltage Boost-loop settings. The loop runs at the grid-peak
 * control event rate (approximately 100 Hz for a 50 Hz grid). */
#define BOOST_CTRL_PERIOD_S              0.01f
#define BOOST_PI_KP                      0.0005f
#define BOOST_PI_KI                      0.01f
#define BOOST_DUTY_MIN                   0.0f
#define BOOST_DUTY_MAX                   0.98f

/* 3 kW model limits.  The PV power limit applies to each input path. */
#define MODEL_3KW_RATED_POWER_W      3000.0f
#define MODEL_3KW_OVERLOAD_POWER_W   3200.0f
#define MODEL_3KW_HALF_LOAD_W        1500.0f
#define MODEL_3KW_PV_CUR_LIMIT_A       10.5f
#define MODEL_3KW_PV_POWER_LIMIT_W   2200.0f
#define MODEL_3KW_GRID_CUR_LIMIT_A     16.0f
#define MODEL_3KW_MPPT_MIN_V          160.0f
#define MODEL_3KW_DCI_TRIP_A            0.8f
#define MODEL_3KW_MPPT_SMALL_DELTA_W    3.0f
#define MODEL_3KW_MPPT_LARGE_DELTA_W    5.0f
#define MODEL_3KW_TEMP_DERATE_C         65.0f

/* 4 kW model limits.  The PV power limit applies to each input path. */
#define MODEL_4KW_RATED_POWER_W      4000.0f
#define MODEL_4KW_OVERLOAD_POWER_W   4300.0f
#define MODEL_4KW_HALF_LOAD_W        2000.0f
#define MODEL_4KW_PV_CUR_LIMIT_A       13.5f
#define MODEL_4KW_PV_POWER_LIMIT_W   2750.0f
#define MODEL_4KW_GRID_CUR_LIMIT_A     22.0f
#define MODEL_4KW_MPPT_MIN_V          165.0f
#define MODEL_4KW_DCI_TRIP_A            0.8f
#define MODEL_4KW_MPPT_SMALL_DELTA_W    3.0f
#define MODEL_4KW_MPPT_LARGE_DELTA_W    5.0f
#define MODEL_4KW_TEMP_DERATE_C         65.0f

/* Common protection and timing constants. */
#define DCI_DEADBAND_A                   0.02f
#define SOURCE_QUALIFY_DELAY_MS         500UL
#define GRID_RECONN_DELAY_MS          60000UL
#define GRID_START_DELAY_S              60.0f
#define OVERLOAD_TRIP_DELAY_S          600.0f
#define OVERLOAD_RECOVER_DELAY_S      1200.0f

/* 好像不需要你
#define GFCI_15MA_A                      0.015f
#define GFCI_20MA_A                      0.020f
#define GFCI_24MA_A                      0.024f
#define GFCI_30MA_A                      0.030f
#define GFCI_35MA_A                      0.035f
#define GFCI_48MA_A                      0.048f
#define GFCI_50MA_A                      0.050f
#define GFCI_70MA_A                      0.070f
#define GFCI_80MA_A                      0.080f
#define GFCI_85MA_A                      0.085f
#define GFCI_120MA_A                     0.120f
#define GFCI_250MA_A                     0.250f
#define GFCI_265MA_A                     0.265f
#define GFCI_280MA_A                     0.280f
#define GFCI_300MA_A                     0.300f*/


/* High-level machine states are fixed protocol values, not runtime data. */
typedef enum
{
    SYS_STATE_WAIT = 0U,
    SYS_STATE_CHECK,
    SYS_STATE_NORMAL,
    SYS_STATE_FAULT,
    SYS_STATE_PERMANENT
} SysState;

/* Ordered qualification stages executed while the machine is in CHECK. */
typedef enum
{
    SYS_CHECK_RESET = 0U,
    SYS_CHECK_SOURCE,
    SYS_CHECK_GRID,
    SYS_CHECK_BUS,
    SYS_CHECK_RELAY,
    SYS_CHECK_PREPARE
} SysCheckStage;

/* 12-bit ADC code-domain constants. */
#define ADC_FULL_SCALE                 4096.0f
#define ADC_BIPOLAR_ZERO               2048.0f

 /* Default measurement gains recovered from the legacy 5/6 kW design.
 * They are initialization values only; final hardware must be calibrated.*/
#define ADC_GRID_VOLTAGE_GAIN              (912.0f / ADC_FULL_SCALE)
#define ADC_INVERTER_VOLTAGE_GAIN          (912.0f / ADC_FULL_SCALE)
#define ADC_DC_BUS_VOLTAGE_GAIN            (604.8f / ADC_FULL_SCALE)
#define ADC_PV_VOLTAGE_GAIN                (604.8f / ADC_FULL_SCALE)
#define ADC_INDUCTOR_CURRENT_GAIN          (94.716f / ADC_FULL_SCALE)
#define ADC_PV_CURRENT_GAIN                (24.0f / ADC_FULL_SCALE)
#define ADC_GFCI_CURRENT_GAIN              (3000.0f / (ADC_FULL_SCALE * 2098.0f))
#define ADC_INVERTER_DC_CURRENT_GAIN       0.000547f
#define ADC_ISOLATION_VOLTAGE_GAIN         0.732f

/* Legacy 4.7 kOhm NTC divider and piecewise temperature-curve constants. */
#define ADC_TEMP_DIVIDER_RESISTANCE        4700.0f
#define ADC_DECI_C_TO_C                    0.1f

// 字库索引布局
static const char OLED_FontSupported[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz.:?!()-_~";
#define OLED_FONT_IDX_SPACE     0U
#define OLED_FONT_IDX_DIGIT     1U
#define OLED_FONT_IDX_ALPHA     11U
#define OLED_FONT_IDX_LOWER     37U
#define OLED_FONT_IDX_SYMBOL    63U
#define OLED_FONT_INDEX_QUESTION   65U   /* '?' = 符号区第 3 个,兜底显示用 */

static const unsigned char OLED_Font5x7[][5] =
{
    /* 空格 */
    {0x00U,0x00U,0x00U,0x00U,0x00U},
    /* 数字 0-9 */
    {0x3EU,0x51U,0x49U,0x45U,0x3EU}, {0x00U,0x42U,0x7FU,0x40U,0x00U},
    {0x72U,0x49U,0x49U,0x49U,0x46U}, {0x21U,0x41U,0x49U,0x4DU,0x33U},
    {0x18U,0x14U,0x12U,0x7FU,0x10U}, {0x27U,0x45U,0x45U,0x45U,0x39U},
    {0x3CU,0x4AU,0x49U,0x49U,0x31U}, {0x41U,0x21U,0x11U,0x09U,0x07U},
    {0x36U,0x49U,0x49U,0x49U,0x36U}, {0x46U,0x49U,0x49U,0x29U,0x1EU},
    /* 大写 A-Z */
    {0x7CU,0x12U,0x11U,0x12U,0x7CU}, {0x7FU,0x49U,0x49U,0x49U,0x36U},
    {0x3EU,0x41U,0x41U,0x41U,0x22U}, {0x7FU,0x41U,0x41U,0x41U,0x3EU},
    {0x7FU,0x49U,0x49U,0x49U,0x41U}, {0x7FU,0x09U,0x09U,0x09U,0x01U},
    {0x3EU,0x41U,0x41U,0x51U,0x73U}, {0x7FU,0x08U,0x08U,0x08U,0x7FU},
    {0x00U,0x41U,0x7FU,0x41U,0x00U}, {0x20U,0x40U,0x41U,0x3FU,0x01U},
    {0x7FU,0x08U,0x14U,0x22U,0x41U}, {0x7FU,0x40U,0x40U,0x40U,0x40U},
    {0x7FU,0x02U,0x1CU,0x02U,0x7FU}, {0x7FU,0x04U,0x08U,0x10U,0x7FU},
    {0x3EU,0x41U,0x41U,0x41U,0x3EU}, {0x7FU,0x09U,0x09U,0x09U,0x06U},
    {0x3EU,0x41U,0x51U,0x21U,0x5EU}, {0x7FU,0x09U,0x19U,0x29U,0x46U},
    {0x26U,0x49U,0x49U,0x49U,0x32U}, {0x03U,0x01U,0x7FU,0x01U,0x03U},
    {0x3FU,0x40U,0x40U,0x40U,0x3FU}, {0x1FU,0x20U,0x40U,0x20U,0x1FU},
    {0x3FU,0x40U,0x38U,0x40U,0x3FU}, {0x63U,0x14U,0x08U,0x14U,0x63U},
    {0x03U,0x04U,0x78U,0x04U,0x03U}, {0x61U,0x59U,0x49U,0x4DU,0x43U},
    /* 小写 a-z */
    {0x20U,0x54U,0x54U,0x78U,0x40U}, {0x7FU,0x28U,0x44U,0x44U,0x38U},
    {0x38U,0x44U,0x44U,0x44U,0x28U}, {0x38U,0x44U,0x44U,0x28U,0x7FU},
    {0x38U,0x54U,0x54U,0x54U,0x18U}, {0x00U,0x08U,0x7EU,0x09U,0x02U},
    {0x18U,0xA4U,0xA4U,0x9CU,0x78U}, {0x7FU,0x08U,0x04U,0x04U,0x78U},
    {0x00U,0x44U,0x7DU,0x40U,0x00U}, {0x20U,0x40U,0x44U,0x3DU,0x00U},
    {0x7FU,0x10U,0x28U,0x44U,0x00U}, {0x00U,0x41U,0x7FU,0x40U,0x00U},
    {0x7FU,0x04U,0x78U,0x04U,0x78U}, {0x7FU,0x08U,0x04U,0x04U,0x78U},
    {0x38U,0x44U,0x44U,0x44U,0x38U}, {0x7FU,0x14U,0x14U,0x14U,0x08U},
    {0x08U,0x14U,0x14U,0x18U,0x7FU}, {0x7FU,0x08U,0x04U,0x04U,0x08U},
    {0x48U,0x54U,0x54U,0x54U,0x24U}, {0x04U,0x04U,0x3FU,0x44U,0x24U},
    {0x3CU,0x40U,0x40U,0x20U,0x7CU}, {0x1CU,0x20U,0x40U,0x20U,0x1CU},
    {0x3CU,0x40U,0x30U,0x40U,0x3CU}, {0x44U,0x28U,0x10U,0x28U,0x44U},
    {0x0CU,0x50U,0x50U,0x50U,0x3CU}, {0x44U,0x64U,0x54U,0x4CU,0x44U},
    /* 符号 . : ? ! ( ) - _ ~ */
    {0x00U,0x00U,0x60U,0x60U,0x00U}, {0x00U,0x00U,0x14U,0x00U,0x00U},
    {0x02U,0x01U,0x59U,0x09U,0x06U}, {0x00U,0x00U,0x5FU,0x00U,0x00U},
    {0x00U,0x1CU,0x22U,0x41U,0x00U}, {0x00U,0x41U,0x22U,0x1CU,0x00U},
    {0x08U,0x08U,0x08U,0x08U,0x08U}, {0x40U,0x40U,0x40U,0x40U,0x40U},
    {0x08U,0x08U,0x2AU,0x1CU,0x08U}
};

#endif
