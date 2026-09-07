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

/* 电网安规阈值已迁移到 gGridSafety 数据域（variable.h/.c）。
 * 10min窗口/DCI/GFCI/电网丢失/快速掉网等单文件常量已迁移到各自任务 .c。 */

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

/* 继电器自检时序/压差判据/检测窗口常量已迁移到 task_state.c。 */

/* 电流幅值限幅范围：母线环(dc)与电流环(ac)共享。 */
#define BUS_CURRENT_AMP_MIN_NORM         0.0f
#define BUS_CURRENT_AMP_MAX_NORM         1.0f

/* Boost 环常量已迁移到 task_dc_ctrl.c。 */

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
#define SOURCE_QUALIFY_DELAY_MS         500UL
#define GRID_RECONN_DELAY_MS          60000UL
#define GRID_START_DELAY_S              60.0f
#define OVERLOAD_TRIP_DELAY_S          600.0f
#define OVERLOAD_RECOVER_DELAY_S      1200.0f

/* GFCI 阈值/计数常量已迁移到 task_ac_monitor.c。 */


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

/* NTC 温度换算常量已迁移到 task_measure.c。 */

/* OLED 字库与索引常量已迁移到 task_ui.c。 */

#endif
