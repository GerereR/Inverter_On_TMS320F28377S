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
