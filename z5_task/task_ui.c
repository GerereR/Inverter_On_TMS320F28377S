#include "F28x_Project.h"

#include "task.h"
#include "bsp.h"
#include "variable.h"

/* ============================================================================
 * task_ui.c —— OLED 显示任务(慢任务,由主循环调度器按旗标调用)
 *
 * 职责:
 *   1. 初始化 OLED(失败不阻塞,按 tick 重试)
 *   2. 周期性从共享数据区快照电压/电流/频率/状态
 *   3. 写入 OLED 帧缓冲,并限流地把脏页刷上屏
 *
 * 关键设计:
 *   - UI_OledReady=0 时本任务只做"重试初始化"一件事,保证主循环不被
 *     I2C 错误拖死;OLED 每次上电初始化有 ~22 次 I2C 事务,若总线异常
 *     会逐条超时,所以重试间隔拉到 10 个 tick(2 秒)。
 *   - 测量数据由同一主循环中的 Task_Measure 统一发布,UI任务执行期间不会
 *     被其他主循环任务抢占;ISR更新的故障字段为16位,可直接原子读取。
 *   - 每 tick 最多刷 2 页(约 6ms 忙等),把 UI 的 I2C 占用封顶,
 *     让 20kHz ADCA1 ISR 和调度器主循环有充足时间窗。
 * ==========================================================================*/

/* OLED 是否已初始化成功;0=未就绪(每 UI_INIT_RETRY_DIVIDER 个 tick 重试一次) */
static Uint16 UI_OledReady = 0U;
/* 初始化重试分频器:每 tick 自增,计满 10(即 200ms×10=2s)触发一次重试 */
static Uint16 UI_InitRetryDivider = 0U;
#define UI_INIT_RETRY_DIVIDER   10U  /* 重试周期:tick 数 */

/* 写状态行:"PLL Lx TZ Fy" 形式的定长文本。
 * x = 锁相环锁定状态,y = 跳变区故障状态(0/1)。
 * 手工拼接而不重复调用 OLED_WriteChar,是为了省掉每次字符函数调用的
 * 开销(慢任务里无所谓,但保持与 OLED 层风格一致的轻量实现)。        */
static void UI_WriteStatusLine(Uint16 page, Uint16 pllLocked, Uint16 tzFault)
{
    char text[OLED_LINE_CHARS + 1U];
    Uint16 position = 0U;
    /* 状态项表:标签 + 取值,两项分别是 PLL 锁定与 TZ 故障。 */
    const char *labels[] = { "PLL L", " TZ F" };
    Uint16 values[] = { (Uint16)(pllLocked != 0U), (Uint16)(tzFault != 0U) };
    Uint16 i;
    Uint16 j;

    text[0] = '\0';
    /* 逐项拼接 "标签+0/1",position 到 OLED_LINE_CHARS 即截断(防御)。 */
    for(i = 0U; i < 2U; i++)
    {
        const char *label = labels[i];
        while((*label != '\0') && (position < OLED_LINE_CHARS))
        {
            text[position++] = *label++;
        }
        if(position < OLED_LINE_CHARS)
        {
            text[position++] = (char)('0' + values[i]);
        }
        text[position] = '\0';
    }
    /* 剩余位置补空格,保证整行被"清干净"(覆盖上一帧残留内容)。 */
    for(j = position; j < OLED_LINE_CHARS; j++)
    {
        text[j] = ' ';
    }
    text[OLED_LINE_CHARS] = '\0';
    OLED_WriteLine(page, text);
}

/* UI 任务初始化(启动时由 main 调用一次,之后也可能被 Task_UI 重试调用):
 * 尝试初始化 OLED;成功则点亮自检画面并进入就绪态,失败则保持未就绪,
 * 由 Task_UI 的慢速重试机制兜底——OLED 不在线/接触不良不会卡死启动。 */
void Task_UI_Init(void)
{
    Uint16 status = OLED_Init();
    UI_OledReady = (status == I2C_STATUS_OK) ? 1U : 0U;
    UI_InitRetryDivider = 0U;

    if(UI_OledReady != 0U)
    {
        (void)OLED_BringupTest();   // 返回值忽略:自检失败不影响进入就绪态
    }
}

/* UI 任务主体:由主循环调度器按 UI 旗标调用,周期 200ms。
 * 执行流程:
 *   1. OLED 未就绪 → 只做慢速重试(2 秒一次),然后立即返回
 *   2. OLED 就绪   → 快照共享数据 → 写帧缓冲(4 行) → 限流刷脏页    */
void Task_UI(void)
{
    float gridVoltage;
    float inductorCurrent;
    float ecapFreq;
    Uint16 pllLocked;
    Uint16 tzFault;

    /* 未就绪分支:UI_InitRetryDivider 每 tick(200ms)自增,
     * 计满 10 个 tick(2s)重新尝试一次 OLED_Init。
     * 重试本身可能阻塞(每条 I2C 命令超时 3ms),但频率低、可接受。   */
    if(UI_OledReady == 0U)
    {
        UI_InitRetryDivider++;
        if(UI_InitRetryDivider >= UI_INIT_RETRY_DIVIDER)
        {
            UI_InitRetryDivider = 0U;
            Task_UI_Init();
        }
        return;
    }

    /* Task_Measure和Task_UI同属合作式主循环,测量快照读取期间不会被改写。
     * pllFault/tzFault由ISR写入,单个16位字段在C28x上可原子读取。 */
    gridVoltage = gMachineData.realAvg.gridVoltage;
    inductorCurrent = gMachineData.realAvg.inductorCurrent;
    ecapFreq = (float)gMachineData.ecapFreqCent * 0.01f;
    pllLocked = (gSysFault.pllFault == 0U) ? 1U : 0U;
    tzFault = gSysFault.tzFault;

    /* 只写帧缓冲(标脏页),不上屏——上屏在最后统一限流执行。
     * 4 条数据行分别占页 4/5/6/7。                                 */
    OLED_WriteFloat1Line(4U, "GRID", gridVoltage, "V");
    OLED_WriteFloat1Line(5U, "I", inductorCurrent, "A");
    OLED_WriteFloat1Line(6U, "ECAP", ecapFreq, "HZ");
    UI_WriteStatusLine(7U, pllLocked, tzFault);

    /* Bound UI work per tick: at most two 128-byte pages are pushed now. */
    /* 限流上屏:每 tick 最多刷 2 页。
     * 每页 = 3 条命令 + 129 字节数据帧 ≈ 2.9ms 忙等;
     * 轮转扫描保证4个脏页分两个tick全部上屏,不会饿死高页。          */
    (void)OLED_RefreshDirty(2U);
}
