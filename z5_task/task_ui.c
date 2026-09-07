#include "F28x_Project.h"

#include "task.h"
#include "bsp.h"
#include "variable.h"
#include "constant.h"
#include "task.h"
#include <string.h>

/* OLED 字库（原 constant.h 迁入，UI 任务私有数据）。 */
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

/* SSD1306配置和帧缓冲只属于UI任务，不暴露给其他任务。 */
#define OLED_I2C_ADDR_7BIT       0x3CU
#define OLED_WIDTH_COLUMNS       128U
#define OLED_HEIGHT_PAGES        8U
#define OLED_LINE_CHARS          21U
#define OLED_I2C_TIMEOUT_US      3000U
#define OLED_SEGMENT_REMAP       0xA1U
#define OLED_COM_SCAN            0xC8U
#define OLED_CONTROL_COMMAND     0x00U
#define OLED_CONTROL_DATA        0x40U
#define OLED_DIRTY_ALL_PAGES     0x00FFU

static unsigned char OLED_FrameBuffer[OLED_WIDTH_COLUMNS * OLED_HEIGHT_PAGES];
static Uint16 OLED_DirtyPages = OLED_DIRTY_ALL_PAGES;
static Uint16 OLED_RefreshStartPage = 0U;
static Uint16 OLED_CursorColumn = 0U;
static Uint16 OLED_CursorPage = 0U;

/* OLED驱动层私有函数声明。任务入口放在前面，具体实现集中放在文件后部。 */
static Uint16 OLED_WriteCommand(unsigned char command);
static void OLED_MarkPageDirty(Uint16 page);
static void OLED_Clear(void);
static Uint16 OLED_Init(void);
static void OLED_SetCursor(Uint16 column, Uint16 page);
static void OLED_WriteChar(char ch);
static void OLED_WriteLine(Uint16 page, const char *text);
static Uint16 OLED_RefreshPage(Uint16 page);
static Uint16 OLED_RefreshDirty(Uint16 maxPages);
static Uint16 OLED_RefreshAll(void);
static void OLED_AppendChar(char *text, Uint16 *position, char ch);
static void OLED_AppendString(char *text, Uint16 *position, const char *value);
static void OLED_WriteFloat1Line(Uint16 page, const char *label, float value, const char *unit);
static Uint16 OLED_BringupTest(void);

/* OLED 是否已初始化成功;0=未就绪(每 UI_INIT_RETRY_DIVIDER 个 tick 重试一次) */
static Uint16 UI_OledReady = 0U;
/* 初始化重试分频器:每 tick 自增,计满 10(即 200ms×10=2s)触发一次重试 */
static Uint16 UI_InitRetryDivider = 0U;
#define UI_INIT_RETRY_DIVIDER   10U  /* 重试周期:tick 数 */
#define UI_DISPLAY_SCREEN_COUNT 2U
#define UI_INIT_RETRY_LIMIT     2U  /* 1.5 s * 2 = 3 s */
#define UI_DISPLAY_PERIOD_TICKS 5U   /* Task_UI每200ms运行一次,每1s轮换页面 */

static Uint16 UI_DisplayScreen = 0U;

/* 写状态行:"PLL Lx TZ Fy" 形式的定长文本。
 * x = 锁相环锁定状态,y = 跳变区故障状态(0/1)。
 * 手工拼接而不重复调用 OLED_WriteChar,是为了省掉每次字符函数调用的
 * 开销(慢任务里无所谓,但保持与 OLED 层风格一致的轻量实现)。        */
#if 0
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

/* Legacy status layout retained only as a reference. */
#endif

/* Show the supervisory state and the two protection indicators together. */
static void UI_WriteControlStatusLine(Uint16 page,
                                      Uint16 systemState,
                                      Uint16 pllLocked,
                                      Uint16 tzFault)
{
    char text[OLED_LINE_CHARS + 1U];
    Uint16 position = 0U;
    Uint16 index;

    text[0] = '\0';
    OLED_AppendString(text, &position, "STATE ");
    OLED_AppendChar(text, &position, (char)('0' + (systemState % 10U)));
    OLED_AppendString(text, &position, " PLL ");
    OLED_AppendChar(text, &position, (char)('0' + (pllLocked != 0U)));
    OLED_AppendString(text, &position, " TZ ");
    OLED_AppendChar(text, &position, (char)('0' + (tzFault != 0U)));
    for(index = position; index < OLED_LINE_CHARS; index++)
    {
        text[index] = ' ';
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
    float gridVoltageRms;
    float inductorCurrentRms;
    float pv1Voltage;
    float pv1Current;
    float pv2Voltage;
    float pv2Current;
    float ecapFreq;
    float pllFreq;
    float inductorCurrentAmp;
    float boost1Duty;
    float boost2Duty;
    Uint16 pllLocked;
    Uint16 tzFault;

    /* 未就绪分支:UI_InitRetryDivider 每 tick(200ms)自增,
     * 计满 10 个 tick(2s)重新尝试一次 OLED_Init。
     * 重试本身可能阻塞(每条 I2C 命令超时 3ms),但频率低、可接受。   */
    if(UI_OledReady == 0U)
    {
        UI_InitRetryDivider++;
        if(UI_InitRetryDivider >= UI_INIT_RETRY_LIMIT)
        {
            UI_InitRetryDivider = 0U;
            Task_UI_Init();
        }
        return;
    }

    /* Task_Measure和Task_UI同属合作式主循环,测量快照读取期间不会被改写。
     * pllFault/tzFault由ISR写入,单个16位字段在C28x上可原子读取。 */
    gridVoltageRms = gMachineData.realRms.gridVoltage;
    inductorCurrentRms = gMachineData.realRms.inductorCurrent;
    pv1Voltage = gMachineData.realAvg.pv1Voltage;
    pv1Current = gMachineData.realAvg.pv1Current;
    pv2Voltage = gMachineData.realAvg.pv2Voltage;
    pv2Current = gMachineData.realAvg.pv2Current;
    ecapFreq = (float)gMachineData.ecapFreqCent * 0.01f;
    pllFreq = (float)gMachineData.pllFreqCent * 0.01f;
    inductorCurrentAmp = gBusCtrlData.currentAmpRef;
    boost1Duty = gBusCtrlData.boost1Duty;
    boost2Duty = gBusCtrlData.boost2Duty;
    pllLocked = (gSysFault.bit.pllFault == 0U) ? 1U : 0U;
    tzFault = gSysFault.bit.tzFault;

    /* 两组页面轮换显示, 每组都覆盖相同的页面, 不会残留上一组内容。 */
    if(UI_DisplayScreen == 0U)
    {
        OLED_WriteLine(0U, "MEASUREMENTS");
        OLED_WriteFloat1Line(1U, "GRID RMS", gridVoltageRms, "V");
        OLED_WriteFloat1Line(2U, "IND RMS", inductorCurrentRms, "A");
        OLED_WriteFloat1Line(3U, "PV1 VOLT", pv1Voltage, "V");
        OLED_WriteFloat1Line(4U, "PV1 CURR", pv1Current, "A");
        OLED_WriteFloat1Line(5U, "PV2 VOLT", pv2Voltage, "V");
        OLED_WriteFloat1Line(6U, "PV2 CURR", pv2Current, "A");
        OLED_WriteFloat1Line(7U, "DC BUS", gMachineData.realAvg.dcBusVoltage, "V");
    }
    else
    {
        OLED_WriteLine(0U, "CONTROL STATUS");
        OLED_WriteFloat1Line(1U, "INV VOLT", gMachineData.realAvg.inverterVoltage, "V");
        OLED_WriteFloat1Line(2U, "ECAP FREQ", ecapFreq, "HZ");
        OLED_WriteFloat1Line(3U, "PLL FREQ", pllFreq, "HZ");
        OLED_WriteFloat1Line(4U, "IREF AMP", inductorCurrentAmp, "PU");
        OLED_WriteFloat1Line(5U, "BOOST1 DUTY", boost1Duty, "PU");
        OLED_WriteFloat1Line(6U, "BOOST2 DUTY", boost2Duty, "PU");
        UI_WriteControlStatusLine(7U, (Uint16)gSysData.state, pllLocked, tzFault);
    }

    /* UI任务每200ms运行一次, 每5次切换一组显示页面。 */
    UI_DisplayScreen++;
    if(UI_DisplayScreen >= UI_DISPLAY_SCREEN_COUNT)
    {
        UI_DisplayScreen = 0U;
    }

    /* Bound UI work per tick: at most two 128-byte pages are pushed now. */
    /* 限流上屏:每 tick 最多刷 2 页。
     * 每页 = 3 条命令 + 129 字节数据帧 ≈ 2.9ms 忙等;
     * 轮转扫描保证4个脏页分两个tick全部上屏,不会饿死高页。          */
    (void)OLED_RefreshDirty(OLED_HEIGHT_PAGES);
}

/* OLED驱动层 =================================================================
 * 完成初始化命令、帧缓冲绘制、脏页管理和I2C上屏。
 * 它依赖通用I2C_MasterWrite，但不再占用i2c.c的底层驱动职责。         */

static Uint16 OLED_WriteCommand(unsigned char command)
{
    unsigned char tx[2];
    tx[0] = OLED_CONTROL_COMMAND;
    tx[1] = command;
    return I2C_MasterWrite(OLED_I2C_ADDR_7BIT, tx, 2U, OLED_I2C_TIMEOUT_US);
}

static void OLED_MarkPageDirty(Uint16 page)
{
    if(page < OLED_HEIGHT_PAGES)
    {
        OLED_DirtyPages |= (Uint16)(1U << page);
    }
}

/* SSD1306没有直接清屏命令，因此清空软件帧缓冲并标记全部页面。 */
static void OLED_Clear(void)
{
    memset(OLED_FrameBuffer, 0, sizeof(OLED_FrameBuffer));
    OLED_CursorColumn = 0U;
    OLED_CursorPage = 0U;
    OLED_DirtyPages = OLED_DIRTY_ALL_PAGES;
    OLED_RefreshStartPage = 0U;
}

static Uint16 OLED_Init(void)
{
    static const unsigned char initCommands[] =
    {
        0xAEU,       /* 关显示，配置期间禁止输出 */
        0xD5U, 0x80U,/* 显示时钟分频比=1，振荡频率=8 */
        0xA8U, 0x3FU,/* 64行复用率 */
        0xD3U, 0x00U,/* 显示起始行偏移=0 */
        0x40U,       /* 起始行地址=0 */
        0x8DU, 0x14U,/* 使能内部电荷泵 */
        0x20U, 0x02U,/* 页寻址模式 */
        OLED_SEGMENT_REMAP,
        OLED_COM_SCAN,
        0xDAU, 0x12U,/* 128x64面板COM引脚配置 */
        0x81U, 0xCFU,/* 对比度 */
        0xD9U, 0xF1U,/* 预充电周期 */
        0xDBU, 0x40U,/* VCOMH电平 */
        0xA4U,       /* 按GDRAM内容显示 */
        0xA6U,       /* 正常显示 */
        0xAFU        /* 开显示 */
    };
    Uint16 index;
    Uint16 status;

    for(index = 0U; index < (sizeof(initCommands) / sizeof(initCommands[0])); index++)
    {
        status = OLED_WriteCommand(initCommands[index]);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }

    OLED_Clear();
    return I2C_STATUS_OK;
}

static void OLED_SetCursor(Uint16 column, Uint16 page)
{
    if(column >= OLED_WIDTH_COLUMNS)
    {
        column = 0U;
    }
    if(page >= OLED_HEIGHT_PAGES)
    {
        page = 0U;
    }
    OLED_CursorColumn = column;
    OLED_CursorPage = page;
}

/* 写一个字符只修改帧缓冲，不直接进行I2C传输。 */
static void OLED_WriteChar(char ch)
{
    Uint16 fontIndex;
    Uint16 bufferIndex;
    Uint16 column;

    if(OLED_CursorColumn > (OLED_WIDTH_COLUMNS - 6U))
    {
        OLED_CursorColumn = 0U;
        OLED_CursorPage++;
        if(OLED_CursorPage >= OLED_HEIGHT_PAGES)
        {
            OLED_CursorPage = 0U;
        }
    }

    if(ch == ' ')
    {
        fontIndex = OLED_FONT_IDX_SPACE;
    }
    else if((ch >= '0') && (ch <= '9'))
    {
        fontIndex = (Uint16)(OLED_FONT_IDX_DIGIT + (Uint16)(ch - '0'));
    }
    else if((ch >= 'A') && (ch <= 'Z'))
    {
        fontIndex = (Uint16)(OLED_FONT_IDX_ALPHA + (Uint16)(ch - 'A'));
    }
    else if((ch >= 'a') && (ch <= 'z'))
    {
        fontIndex = (Uint16)(OLED_FONT_IDX_LOWER + (Uint16)(ch - 'a'));
    }
    else
    {
        switch(ch)
        {
            case '.': fontIndex = (Uint16)(OLED_FONT_IDX_SYMBOL + 0U); break;
            case ':': fontIndex = (Uint16)(OLED_FONT_IDX_SYMBOL + 1U); break;
            case '?': fontIndex = (Uint16)(OLED_FONT_IDX_SYMBOL + 2U); break;
            case '!': fontIndex = (Uint16)(OLED_FONT_IDX_SYMBOL + 3U); break;
            case '(': fontIndex = (Uint16)(OLED_FONT_IDX_SYMBOL + 4U); break;
            case ')': fontIndex = (Uint16)(OLED_FONT_IDX_SYMBOL + 5U); break;
            case '-': fontIndex = (Uint16)(OLED_FONT_IDX_SYMBOL + 6U); break;
            case '_': fontIndex = (Uint16)(OLED_FONT_IDX_SYMBOL + 7U); break;
            case '~': fontIndex = (Uint16)(OLED_FONT_IDX_SYMBOL + 8U); break;
            default:  fontIndex = OLED_FONT_INDEX_QUESTION; break;
        }
    }

    bufferIndex = (OLED_CursorPage * OLED_WIDTH_COLUMNS) + OLED_CursorColumn;
    for(column = 0U; column < 5U; column++)
    {
        OLED_FrameBuffer[bufferIndex + column] = OLED_Font5x7[fontIndex][column];
    }
    OLED_FrameBuffer[bufferIndex + 5U] = 0U;
    OLED_CursorColumn += 6U;
    OLED_MarkPageDirty(OLED_CursorPage);
}

static void OLED_WriteLine(Uint16 page, const char *text)
{
    Uint16 written = 0U;

    OLED_SetCursor(0U, page);
    if(text != 0)
    {
        while((*text != '\0') && (written < OLED_LINE_CHARS))
        {
            OLED_WriteChar(*text);
            text++;
            written++;
        }
    }
    while(written < OLED_LINE_CHARS)
    {
        OLED_WriteChar(' ');
        written++;
    }
}

/* 页寻址模式下，设定一次页和起始列后连续发送128字节。 */
static Uint16 OLED_RefreshPage(Uint16 page)
{
    static unsigned char tx[1U + OLED_WIDTH_COLUMNS];
    Uint16 column;
    Uint16 status;

    if(page >= OLED_HEIGHT_PAGES)
    {
        return I2C_STATUS_BAD_PARAMETER;
    }

    status = OLED_WriteCommand((unsigned char)(0xB0U | page));
    if(status != I2C_STATUS_OK) { return status; }

    status = OLED_WriteCommand(0x00U);
    if(status != I2C_STATUS_OK) { return status; }

    status = OLED_WriteCommand(0x10U);
    if(status != I2C_STATUS_OK) { return status; }

    tx[0] = OLED_CONTROL_DATA;
    for(column = 0U; column < OLED_WIDTH_COLUMNS; column++)
    {
        tx[column + 1U] = OLED_FrameBuffer[(page * OLED_WIDTH_COLUMNS) + column];
    }
    return I2C_MasterWrite(OLED_I2C_ADDR_7BIT,
                           tx,
                           (Uint16)(1U + OLED_WIDTH_COLUMNS),
                           OLED_I2C_TIMEOUT_US);
}

/* 轮转扫描脏页，每次最多刷新maxPages页。 */
static Uint16 OLED_RefreshDirty(Uint16 maxPages)
{
    Uint16 scanStart = OLED_RefreshStartPage;
    Uint16 pageOffset;
    Uint16 page;
    Uint16 refreshed = 0U;
    Uint16 status;

    for(pageOffset = 0U; pageOffset < OLED_HEIGHT_PAGES; pageOffset++)
    {
        page = scanStart + pageOffset;
        if(page >= OLED_HEIGHT_PAGES)
        {
            page -= OLED_HEIGHT_PAGES;
        }

        if((OLED_DirtyPages & (Uint16)(1U << page)) == 0U)
        {
            continue;
        }
        if(refreshed >= maxPages)
        {
            break;
        }

        status = OLED_RefreshPage(page);
        if(status != I2C_STATUS_OK)
        {
            OLED_RefreshStartPage = page;
            return status;
        }

        OLED_DirtyPages &= (Uint16)~(1U << page);
        refreshed++;
        OLED_RefreshStartPage = page + 1U;
        if(OLED_RefreshStartPage >= OLED_HEIGHT_PAGES)
        {
            OLED_RefreshStartPage = 0U;
        }
    }
    return I2C_STATUS_OK;
}

static Uint16 OLED_RefreshAll(void)
{
    return OLED_RefreshDirty(OLED_HEIGHT_PAGES);
}

static void OLED_AppendChar(char *text, Uint16 *position, char ch)
{
    if(*position < OLED_LINE_CHARS)
    {
        text[*position] = ch;
        (*position)++;
        text[*position] = '\0';
    }
}

static void OLED_AppendString(char *text, Uint16 *position, const char *value)
{
    if(value == 0)
    {
        return;
    }
    while((*value != '\0') && (*position < OLED_LINE_CHARS))
    {
        OLED_AppendChar(text, position, *value);
        value++;
    }
}

/* 手工格式化一位小数，避免引入printf浮点格式化开销。 */
static void OLED_WriteFloat1Line(Uint16 page, const char *label, float value, const char *unit)
{
    char text[OLED_LINE_CHARS + 1U];
    Uint16 position = 0U;
    long scaled;
    long integerPart;
    long divisor;
    Uint16 fractionPart;
    Uint16 started = 0U;

    text[0] = '\0';
    if(value > 9999.9f)
    {
        value = 9999.9f;
    }
    else if(value < -9999.9f)
    {
        value = -9999.9f;
    }

    OLED_AppendString(text, &position, label);
    OLED_AppendChar(text, &position, ' ');

    scaled = (long)((value >= 0.0f) ? ((value * 10.0f) + 0.5f) : ((value * 10.0f) - 0.5f));
    if(scaled < 0L)
    {
        OLED_AppendChar(text, &position, '-');
        scaled = -scaled;
    }
    integerPart = scaled / 10L;
    fractionPart = (Uint16)(scaled % 10L);

    for(divisor = 10000L; divisor > 0L; divisor /= 10L)
    {
        Uint16 digit = (Uint16)((integerPart / divisor) % 10L);
        if((digit != 0U) || (started != 0U) || (divisor == 1L))
        {
            OLED_AppendChar(text, &position, (char)('0' + digit));
            started = 1U;
        }
    }
    OLED_AppendChar(text, &position, '.');
    OLED_AppendChar(text, &position, (char)('0' + fractionPart));
    if(unit != 0)
    {
        OLED_AppendChar(text, &position, ' ');
        OLED_AppendString(text, &position, unit);
    }

    OLED_WriteLine(page, text);
}

static Uint16 OLED_BringupTest(void)
{
    OLED_Clear();
    OLED_WriteLine(0U, "INVERTER BASE");
    OLED_WriteLine(1U, "OLED SSD1306 OK");
    OLED_WriteLine(2U, "I2CA 7BIT 0X3C");
    OLED_WriteLine(4U, "GRID ---.- V");
    OLED_WriteLine(5U, "I ---.- A");
    OLED_WriteLine(6U, "UI BOOT");
    return OLED_RefreshAll();
}
