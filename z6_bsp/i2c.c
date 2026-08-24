#include "F28x_Project.h"
#include "bsp.h"
#include "constant.h" /* OLED_Font5x7 点阵字库(定义见 z7_data/constant.h) */
#include <string.h>   /* memset():OLED_Clear() 清帧缓冲用,补原型消除隐式声明警告 */


//I2C配置层========================================================================================
// 200 MHz SYSCLK -> 10 MHz I2C module clock; these dividers target 400 kHz SCL.
// First OLED bring-up should still be validated on a scope/analyzer; drop to
// 100 kHz if the wiring/pull-ups cannot sustain 400 kHz.

#define I2C_PRESCALER            19U  /* IPSC:200MHz / (19+1) = 10MHz 模块时钟 */
#define I2C_CLOCK_LOW            10U  /* ICCL:低电平 15 个模块周期 = 1.5us    */
#define I2C_CLOCK_HIGH           5U   /* ICCH:高电平 10 个模块周期 = 1.0us    */
#define I2C_WAIT_LOOPS_PER_US    20UL /* 空转延时估算系数:约 20 圈 ≈ 1us, 只用于超时兜底,不是精确定时 */
#define I2C_DEFAULT_TIMEOUT_US   2000U /* 单字节/单次等待的默认超时 2ms       */

//就是把现实的时间转换成CPU循环的圈数,相当于半个自适应算法了
static Uint32 I2C_WaitLoopsFromUs(Uint16 timeoutUs)
{
    Uint32 loops = (Uint32)timeoutUs * I2C_WAIT_LOOPS_PER_US;
    return (loops == 0UL) ? 1UL : loops;
}

//这几个寄存器是锁存型的,必须软件复位
static void I2C_ClearStatusFlags(void)
{
    I2caRegs.I2CSTR.bit.ARBL = 1U;  // 清除 "我刚才没抢到总线"这个状态
    I2caRegs.I2CSTR.bit.NACK = 1U;  // 清除 "从机没理我"这个状态

    I2caRegs.I2CSTR.bit.ARDY = 1U;  // 清除 "模块闲了,寄存器可以安全访问"这个状态
    I2caRegs.I2CSTR.bit.SCD  = 1U;  // 清除 "刚才一次传输正常结束了"这个状态
}

/* 读取错误状态并映射为 bsp.h 中定义的公共错误码。
 * 优先级:仲裁丢失 > NACK。SCD/超时由调用方的等待循环自行判断。     */
static Uint16 I2C_GetErrorStatus(void)
{
    if(I2caRegs.I2CSTR.bit.ARBL != 0U)
    {
        return I2C_STATUS_ARBITRATION_LOST;
    }
    if(I2caRegs.I2CSTR.bit.NACK != 0U)
    {
        return I2C_STATUS_NACK;
    }
    return I2C_STATUS_OK;
}

/* 出错统一收尾:主动发出 STOP 释放总线,再清状态标志,返回错误码。
 * 保证任何失败路径退出后总线都回到空闲、标志干净,不影响下次传输。  */
static Uint16 I2C_FinishWithError(Uint16 status)
{
    I2caRegs.I2CMDR.bit.STP = 1U;
    I2C_ClearStatusFlags();
    return status;
}

/* 轮询等待的公共检查(方案 1 重构):每个等待循环每次空转都调用一次,
 * 负责两件事——
 *   ① 检查 ARBL/NACK:出错立即走 FinishWithError 释放总线并返回错误码;
 *   ② 递减倒计时:减到 0 即超时,同样走 FinishWithError 返回超时码。
 * 返回 I2C_STATUS_OK 表示"还没出错、没超时,可以继续等"。
 * 注意:错误/超时路径内部已调用 FinishWithError,调用方检测到
 * 非 OK 返回值后只需直接 return 该值,不要重复收尾。               */
static Uint16 I2C_WaitPoll(Uint32 *waitLoops)
{
    Uint16 status = I2C_GetErrorStatus();
    if(status != I2C_STATUS_OK)
    {
        return I2C_FinishWithError(status);
    }
    if(--(*waitLoops) == 0UL)
    {
        return I2C_FinishWithError(I2C_STATUS_TIMEOUT);
    }
    return I2C_STATUS_OK;
}

/* I2CA 初始化:400kHz SCL,主机模式,纯轮询(模块中断全关)。
 * 由 System_Init() 调用,在主循环开始前完成。                        */
void I2C_Config(void)
{
    EALLOW;

    // 先复位模块并延时,保证修改时序寄存器期间模块处于确定状态。
    I2caRegs.I2CMDR.bit.IRS = 0U;       // IRS=0:复位 I2C 模块(总线操作停止)
    DELAY_US(1000);                     // 等待复位生效
    I2caRegs.I2CFFTX.all = 0x0000;      // 发送 FIFO 全复位(TXFFRST=0)
    I2caRegs.I2CFFRX.all = 0x0040;      // 接收 FIFO 复位,同时清 RXFFINT 标志
    I2caRegs.I2CPSC.bit.IPSC = I2C_PRESCALER;  // 预分频:200MHz → 10MHz 模块时钟
    I2caRegs.I2CCLKL = I2C_CLOCK_LOW;   // SCL 低电平时间(见文件头时序推导)
    I2caRegs.I2CCLKH = I2C_CLOCK_HIGH;  // SCL 高电平时间
    I2caRegs.I2COAR.bit.OAR = 0x0000;   // 本机地址:主机模式下不使用,清零即可
    I2caRegs.I2CSAR.bit.SAR = 0x0000;   // 从机地址:每次传输前由 I2C_MasterWrite 重写
    I2C_ClearStatusFlags();             // 清掉复位期间可能产生的残留标志

    // 关键设计:关闭全部模块级中断(I2CIER=0),走纯轮询。
    // 会顺带清除 ARDY/SCD 标志,可能"偷走"轮询传输正在等待的状态位。
    I2caRegs.I2CIER.all = 0x0000;       // 模块中断全关,纯轮询;不注册 ISR、不开 PIE/IER

    // 释放模块复位并开始工作。
    I2caRegs.I2CMDR.all = 0x0000;       // 先清整寄存器,保证从已知状态启动
    I2caRegs.I2CMDR.bit.IRS = 1U;       // 释放复位,模块进入空闲
    I2caRegs.I2CMDR.bit.FREE = 1U;      // 仿真挂起(断点)时 I2C 继续运行,不挂总线

    // 使能 FIFO:TXFFRST=1 释放发送 FIFO复位,I2CFFEN=1 使能 FIFO 模式;
    // RX 侧 RXFFIL=0(收到 1 字节即触发),因无中断,该阈值暂不生效。
    I2caRegs.I2CFFTX.all = 0x6040;
    I2caRegs.I2CFFRX.all = 0x2040;

    EDIS;
}

/* I2C 轮询式主机发送(整个 OLED 驱动都基于它):
 * 向 7 位地址的从机写入 length 字节数据,阻塞直到完成/超时/出错。
 * 返回 bsp.h 中定义的 I2C_STATUS_xxx 状态码。
 * 传输阶段划分:
 *   1) 参数校验
 *   2) 等待总线空闲(BB=0)
 *   3) 配置地址/长度并置位 STT+STP 启动一次完整传输
 *   4) 逐字节等待 XRDY 写 I2CDXR
 *   5) 等待移位器空(XSMT)且 STOP 后总线释放(BB=0)
 * 每段独立超时,出错路径统一走 I2C_FinishWithError 释放总线。      */
Uint16 I2C_MasterWrite//流式字节发送,就是很多个字节完成理论上一个字节完成的事情
(
    Uint16 slaveAddr7, 
    const unsigned char *data, 
    Uint16 length, 
    Uint16 timeoutUs
)
{
    Uint16 index;
    Uint16 status;
    Uint32 waitLoops;

    /* 参数校验:空指针、零长度、地址超过 7 位范围都直接拒绝。 */
    if((data == 0) || (length == 0U) || (slaveAddr7 > 0x7FU))
    {
        return I2C_STATUS_BAD_PARAMETER;
    }
    /* 未指定超时(0)则采用默认值,保证调用方可以不关心细节。 */
    if(timeoutUs == 0U)
    {
        timeoutUs = I2C_DEFAULT_TIMEOUT_US;//默认2ms
    }

    /* 等待总线空闲:BB=1 表示总线上有 START 条件未结束(可能被其他主机占用
     * 或上一次异常传输没有正确收尾),超时则放弃并上报 BUS_BUSY。      */
    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while(I2caRegs.I2CSTR.bit.BB != 0U)
    {
        if(waitLoops == 0UL)
        {
            return I2C_STATUS_BUS_BUSY;
        }
        waitLoops--;
    }

    /* 传输前清掉所有残留状态,并装载从机地址与数据长度。
     * 注意 SAR 是 7 位地址(硬件负责补 R/W 位)。                     */
    I2C_ClearStatusFlags();
    I2caRegs.I2CSAR.bit.SAR = slaveAddr7;
    I2caRegs.I2CCNT = length;


    I2caRegs.I2CMDR.all = 0x0000;
    I2caRegs.I2CMDR.bit.IRS = 1U;//IRS=1 模块使能 
    I2caRegs.I2CMDR.bit.TRX = 1U;//TRX=1 发送方向(主机写)
    I2caRegs.I2CMDR.bit.MST = 1U;//MST=1 主机模式
    I2caRegs.I2CMDR.bit.FREE = 1U;//FREE=1 仿真挂起时继续运行
    I2caRegs.I2CMDR.bit.STP = 1U;//STP=1 发完自动产生 STOP(一次性传输)
    I2caRegs.I2CMDR.bit.STT = 1U;//STT=1 立即产生 START(硬件自动等待总线空闲后起始)

    /* 逐字节发送:每次先等发送就绪(XRDY=1,表示 I2CDXR 可写入),
     * 等待期间每空转一圈由 I2C_WaitPoll 统一检查错误/超时。          */
    for(index = 0U; index < length; index++)
    {
        waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
        while(I2caRegs.I2CSTR.bit.XRDY == 0U)
        {
            status = I2C_WaitPoll(&waitLoops);
            if(status != I2C_STATUS_OK)
            {
                return status;   // 传输错误/超时已在 WaitPoll 内收尾
            }
        }
        I2caRegs.I2CDXR.bit.DATA = data[index];
    }

    /* 等待传输收尾:两个条件都满足才算真结束——
     *   XSMT=1:移位寄存器已空(最后一字节也送上了 SDA)
     *   BB=0  :STOP 已发出且总线回到空闲
     * 若只等 XSMT 就返回,下一次传输可能撞上还没释放的总线。          */
    waitLoops = I2C_WaitLoopsFromUs(timeoutUs);
    while((I2caRegs.I2CSTR.bit.XSMT == 0U) || (I2caRegs.I2CSTR.bit.BB != 0U))
    {
        status = I2C_WaitPoll(&waitLoops);
        if(status != I2C_STATUS_OK)
        {
            return status;   // 传输错误/超时已在 WaitPoll 内收尾
        }
    }

    /* 收尾后最后复查一次错误(NACK 可能出现在最后一个字节的应答位上)。 */
    status = I2C_GetErrorStatus();
    if(status != I2C_STATUS_OK)
    {
        return I2C_FinishWithError(status);
    }

    I2C_ClearStatusFlags();
    return I2C_STATUS_OK;
}


//OLED驱动层=======================================================================================

#define OLED_CONTROL_COMMAND       0x00U  /* 控制字节=命令:下一字节写入命令寄存器 */
#define OLED_CONTROL_DATA          0x40U  /* 控制字节=数据:后续字节写入 GDRAM     */
#define OLED_DIRTY_ALL_PAGES       0x00FFU /* 全部 8 页都脏(初始化时整屏刷)      */

//其实这个就是一个屏幕的所有数据
static unsigned char OLED_FrameBuffer[OLED_WIDTH_COLUMNS * OLED_HEIGHT_PAGES];

/* 脏页位图:bit n = 1 表示第 n 页缓冲被改过、需要刷到屏幕。
 * 写缓冲只标脏,真正刷屏由 OLED_RefreshDirty() 完成(UI 任务限流调用)。*/
static Uint16 OLED_DirtyPages = OLED_DIRTY_ALL_PAGES;//0b 1111 1111

/* 下一次脏页扫描的起点。轮转扫描可防止低页反复标脏时饿死高页。 */
static Uint16 OLED_RefreshStartPage = 0U;

/* 光标位置:OLED_WriteChar 按 6 列步进,用于字符流式输出。             */
static Uint16 OLED_CursorColumn = 0U;
static Uint16 OLED_CursorPage = 0U;

/* 发送一条单字节命令(1 字节控制 + 1 字节命令 = 2 字节 I2C 事务)。   */
static Uint16 OLED_WriteCommand(unsigned char command)
{
    unsigned char tx[2];
    tx[0] = OLED_CONTROL_COMMAND;//告诉 OLED "接下来我发的这个字节是命令"
    tx[1] = command;//具体的命令
    return I2C_MasterWrite(OLED_I2C_ADDR_7BIT, tx, 2U, OLED_I2C_TIMEOUT_US);
}

/* 把某一页标记为脏(边界内才有效),供刷新循环使用。                  */
static void OLED_MarkPageDirty(Uint16 page)
{
    if(page < OLED_HEIGHT_PAGES)
    {
        //进位是因为oled的页是从零开始,而OLED_DirtyPages是从1开始
        OLED_DirtyPages |= (Uint16)(1U << page);
    }
}

/* 清屏:帧缓冲全部写 0,光标归零,并标记整屏脏(等待下次刷新上屏)。    */
void OLED_Clear(void)//oled没有清屏命令...
{
    memset(OLED_FrameBuffer, 0, sizeof(OLED_FrameBuffer));
    OLED_CursorColumn = 0U;
    OLED_CursorPage = 0U;
    OLED_DirtyPages = OLED_DIRTY_ALL_PAGES;
    OLED_RefreshStartPage = 0U;
}


/* OLED 上电初始化:按 SSD1306 手册顺序发送初始化命令。
 * 任何一条命令失败立即返回错误码(由 Task_UI_Init 决定是否重试);
 * 全部成功后只清软件缓冲,由随后的自检画面完成第一次全屏刷新。      */
Uint16 OLED_Init(void)
{
    static const unsigned char initCommands[] =
    {
        0xAEU,       // 0xAE       关显示(配置期间禁止输出)
        0xD5U, 0x80U,// 0xD5,0x80  显示时钟分频比=1、振荡频率=8(默认值)
        0xA8U, 0x3FU,// 0xA8,0x3F  复用率=63(64 行,128x64 面板标配)
        0xD3U, 0x00U,// 0xD3,0x00  显示起始行偏移=0
        0x40U,       // 0x40       起始行地址=0
        0x8DU, 0x14U,// 0x8D,0x14  内部电荷泵使能(3.3V 单电源供电必需)
        0x20U, 0x02U,// 内存寻址模式:0x00水平/0x01垂直/0x02页寻址(见 bsp.h)
        OLED_SEGMENT_REMAP,         // 段重映射:0xA0正向 / 0xA1水平镜像(见 bsp.h)
        OLED_COM_SCAN,              // COM 扫描方向:0xC0自上而下 / 0xC8自下而上
        0xDAU, 0x12U,// 0xDA,0x12  COM 引脚配置=Alternative(128x64 面板标配)
        0x81U, 0xCFU,// 0x81,0xCF  对比度=0xCF(接近最大亮度)
        0xD9U, 0xF1U,// 0xD9,0xF1  预充电周期=Phase1:1 / Phase2:15
        0xDBU, 0x40U,// 0xDB,0x40  VCOMH 去压选择=0.77×VCC(默认档)
        0xA4U,       // 0xA4       从 GDRAM 内容正常显示(非全亮)
        0xA6U,       // 0xA6       正常显示(非反白)
        0xAFU        // 0xAF       开显示(最后一条,配置完成后点亮)
    };
    Uint16 index;
    Uint16 status;

    //一条条执行命令
    for(index = 0U; index < (sizeof(initCommands) / sizeof(initCommands[0])); index++)
    {
        status = OLED_WriteCommand(initCommands[index]);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
    }
    //清除软件缓存
    OLED_Clear();
    return I2C_STATUS_OK;
}

/* 设置光标。越界参数回绕到 0(防御性处理,调用方通常传入合法值)。   */
void OLED_SetCursor(Uint16 column, Uint16 page)
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

/* 在光标处写一个字符(只改帧缓冲,不直接刷屏):
 *   - 每字符占 6 列(5 列点阵 + 1 列间隔)
 *   - 剩余空间不足 6 列时自动换到下一页行首
 *   - 字库下标按 ASCII 区间直接计算(O(1),布局见 constant.h)
 *   - 写完后标记该页为脏,由 OLED_RefreshDirty 统一上屏              */
void OLED_WriteChar(char ch)
{
    Uint16 fontIndex;   /* 字库下标 */
    Uint16 bufferIndex; /* 帧缓冲起始下标 */
    Uint16 i;

    //页,列回绕
    if(OLED_CursorColumn > (OLED_WIDTH_COLUMNS - 6U))
    {
        OLED_CursorColumn = 0U;
        OLED_CursorPage++;
        if(OLED_CursorPage >= OLED_HEIGHT_PAGES)
        {
            OLED_CursorPage = 0U;   // 末页之后回绕到第 0 页(防越界)
        }
    }

    //字库下标:空格/数字/大写/小写按区间直接算,符号 switch,其余 '?' 兜底
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

    //定位到帧缓冲,写入 5 列字模 + 1 列间隔
    bufferIndex = (OLED_CursorPage * OLED_WIDTH_COLUMNS) + OLED_CursorColumn;
    for(i = 0U; i < 5U; i++)
    {
        OLED_FrameBuffer[bufferIndex + i] = OLED_Font5x7[fontIndex][i];
    }
    OLED_FrameBuffer[bufferIndex + 5U] = 0U;   // 字符间隔列清零
    OLED_CursorColumn += 6U;
    OLED_MarkPageDirty(OLED_CursorPage); // 标脏当前页(换页逻辑已更新)
}

/* 连续写字符串直到遇到 '\0'。无长度限制,超屏会按 WriteChar 规则回绕。*/
void OLED_WriteString(const char *text)
{
    if(text == 0)
    {
        return;
    }
    while(*text != '\0')
    {
        OLED_WriteChar(*text);
        text++;
    }
}

/* 整行写:光标先移到 (0, page),最多写 OLED_LINE_CHARS(21)个字符,
 * 不足部分补空格——把该页残留的旧内容也覆盖掉,保证整行干净。       */
void OLED_WriteLine(Uint16 page, const char *text)
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

/* 把某一页帧缓冲刷到屏幕。一次完整的页刷新 = 4 次 I2C 事务:
 *   ① 命令 0xB0|page —— 设置页地址(页寻址模式下必须)
 *   ② 命令 0x00     —— 列地址低 4 位 = 0
 *   ③ 命令 0x10     —— 列地址高 4 位 = 0(列地址整体归零,从头刷)
 *   ④ 129 字节数据帧(1 字节控制 0x40 + 128 字节整页点阵)
 * 所以无论只改了几个字符,都是以"整页"为粒度上屏。                  */

 //以上都是把待写数据放到buffer里面,这一步才正式开始让IIC模块写数据
static Uint16 OLED_RefreshPage(Uint16 page)
{
    static unsigned char tx[1U + OLED_WIDTH_COLUMNS];   // static:避免每次 129 字节栈开销
    Uint16 i;
    Uint16 status;

    //错页报警
    if(page >= OLED_HEIGHT_PAGES)
    {
        return I2C_STATUS_BAD_PARAMETER;
    }

    status = OLED_WriteCommand((unsigned char)(0xB0U | page));  // 设页地址
    if(status != I2C_STATUS_OK) { return status; }

    status = OLED_WriteCommand(0x00U);                          // 列地址低 4 位 = 0
    if(status != I2C_STATUS_OK) { return status; }
    
    status = OLED_WriteCommand(0x10U);                          // 列地址高 4 位 = 0
    if(status != I2C_STATUS_OK) { return status; }

    tx[0] = OLED_CONTROL_DATA;
    for(i = 0U; i < OLED_WIDTH_COLUMNS; i++)
    {
        tx[i + 1U] = OLED_FrameBuffer[(page * OLED_WIDTH_COLUMNS) + i];
    }
    return I2C_MasterWrite(OLED_I2C_ADDR_7BIT, tx, (Uint16)(1U + OLED_WIDTH_COLUMNS), OLED_I2C_TIMEOUT_US);
}

/* 增量刷新脏页:本次调用最多刷 maxPages 页(UI 任务用它限流)。
 *   - 从上次停止位置轮转遍历 8 页,避免低页持续标脏造成高页饥饿;
 *   - 任一页 I2C 失败立即中止并返回错误码(不继续刷,保留脏位待重试)。
 * 返回 I2C_STATUS_OK 只表示"本次尝试的页都成功了",不表示整屏干净。  */
Uint16 OLED_RefreshDirty(Uint16 maxPages)
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
            continue;                       // 该页无改动,跳过
        }
        if(refreshed >= maxPages)
        {
            break;                          // 达到本次限流上限,剩余脏页留给下次
        }
        status = OLED_RefreshPage(page);
        if(status != I2C_STATUS_OK)
        {
            OLED_RefreshStartPage = page;    // 下次仍从失败页开始重试
            return status;                  // 失败:脏位保留,便于下个 tick 重试
        }
        OLED_DirtyPages &= (Uint16)~(1U << page);   // 成功上屏后清除脏位
        refreshed++;
        OLED_RefreshStartPage = page + 1U;
        if(OLED_RefreshStartPage >= OLED_HEIGHT_PAGES)
        {
            OLED_RefreshStartPage = 0U;
        }
    }
    return I2C_STATUS_OK;
}

/* 全量刷新:一次刷完所有 8 页(初始化/自检画面用,会长时间阻塞调用者)。*/
Uint16 OLED_RefreshAll(void)
{
    return OLED_RefreshDirty(OLED_HEIGHT_PAGES);
}

/* 向行缓冲追加一个字符,并维护 '\0' 结尾;越界(>=21 字符)则丢弃。    */
static void OLED_AppendChar(char *text, Uint16 *position, char ch)
{
    if(*position < OLED_LINE_CHARS)
    {
        text[*position] = ch;
        (*position)++;
        text[*position] = '\0';
    }
}

/* 向行缓冲追加整个字符串(受 21 字符上限约束,超长自动截断)。        */
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

/* 在指定页显示 "label 值 unit",值为浮点、固定 1 位小数。
 * 不用 printf(浮点格式化代码量大),手工定点格式化:
 *   - 限幅到 ±9999.9,超出截断,防止整数部分溢出 5 位宽度
 *   - ×10 四舍五入后拆出整数部分与 1 位小数
 *   - 整数部分从 10000 开始逐位除出,跳过前导零(至少保留 1 位)
 * 最后整行写入帧缓冲(补空格覆盖旧内容),标脏但不直接刷屏。          */
void OLED_WriteFloat1Line(Uint16 page, const char *label, float value, const char *unit)
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

    /* ×10 并四舍五入,把"1 位小数"变成整数处理;负数先记符号再取绝对值。 */
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

/* 开机自检画面:Task_UI_Init 在 OLED_Init 成功后调用。
 * 作用:①确认整屏刷新链路(命令+数据)可用;②给用户一个可见的启动标志。*/
Uint16 OLED_BringupTest(void)
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
