#include "F28x_Project.h"

#include "task.h"
#include "bsp.h"
#include "variable.h"
#include "task.h"

/*
 * SCI communication uses a compact binary frame so that the protocol is
 * independent of the physical SCI pins and easy to port to another MCU.
 *
 * Frame format:
 *   0xAA 0x55 CMD SEQ LEN_L LEN_H PAYLOAD CRC_L CRC_H
 *
 * The CRC16 covers CMD, SEQ, LEN_L, LEN_H, and every payload byte.
 */
#define SCI_FRAME_SOF1             0xAAU//帧头
#define SCI_FRAME_SOF2             0x55U

#define SCI_FRAME_MAX_PAYLOAD      64U//最大响应Payload为64字节
#define SCI_TASK_MAX_RX_BYTES      512U//通讯任务处理字节上限

/* Commands sent by the host computer. */
//命令宏
#define SCI_CMD_READ_REAL          0x02U
#define SCI_CMD_READ_PLL_STATUS    0x03U
#define SCI_CMD_READ_FAULT_STATUS  0x04U
#define SCI_CMD_READ_RMS                0x05U
#define SCI_CMD_READ_CONTROL_STATUS     0x06U
#define SCI_CMD_SET_INDUCTOR_CUR_AMP       0x10U
#define SCI_CMD_CLEAR_FAULT        0x20U
#define SCI_CMD_READ_VERSION       0x30U

/* Response status values placed at payload[0]. */
//状态宏
#define SCI_STATUS_OK              0x00U
#define SCI_STATUS_BAD_LENGTH      0x01U
#define SCI_STATUS_BAD_COMMAND     0x02U
#define SCI_STATUS_BAD_PARAMETER   0x03U

//数据帧接收窗口
typedef enum
{
    SCI_PARSE_WAIT_SOF1 = 0U,
    SCI_PARSE_WAIT_SOF2,
    SCI_PARSE_COMMAND,
    SCI_PARSE_SEQUENCE,
    SCI_PARSE_LENGTH_LOW,
    SCI_PARSE_LENGTH_HIGH,
    SCI_PARSE_PAYLOAD,
    SCI_PARSE_CRC_LOW,
    SCI_PARSE_CRC_HIGH
} SCI_ParseState;

static SCI_ParseState SCI_ParseStateCurrent = SCI_PARSE_WAIT_SOF1;
static Uint16 SCI_ReceivedCommand = 0U;
static Uint16 SCI_ReceivedSequence = 0U;
static Uint16 SCI_ReceivedLength = 0U;
static Uint16 SCI_ReceivedPayload[SCI_FRAME_MAX_PAYLOAD];
static Uint16 SCI_ReceivedPayloadIndex = 0U;
static Uint16 SCI_ReceivedCrc = 0U;
static Uint16 SCI_CalculatedCrc = 0xFFFFU;

/* Protocol diagnostics are owned by this task and remain visible to CCS. */
static volatile Uint32 SCI_ProtocolCrcErrorCount = 0UL;
static volatile Uint32 SCI_ProtocolFormatErrorCount = 0UL;
static volatile Uint32 SCI_ProtocolFrameCount = 0UL;

/* Forward declarations keep the public task entry points near the top. */
static Uint16 SCI_Crc16Update(Uint16 crc, Uint16 data);//使用一个新字节更新 CRC16 校验值。
static void SCI_SendWordLE(Uint16 value);//以小端格式发送一个 16 位数据：
static void SCI_PutWordLE(Uint16 *payload, Uint16 *index, Uint16 value);//把一个 16 位数据按小端格式写入响应数据数组，并自动移动数组下标。
static void SCI_PutFloatLE(Uint16 *payload, Uint16 *index, float value);

//组装并发送完整协议响应帧，包括：
static void SCI_SendResponse(Uint16 command, Uint16 sequence, const Uint16 *payload, Uint16 payloadLength);
static void SCI_PutDwordLE(Uint16 *payload, Uint16 *index, Uint32 value);

static void SCI_HandleCommand(void);//处理一帧已经完成 CRC 校验的命令，根据命令号执行相应操作并发送响应。
static void SCI_ResetParser(void);//复位协议解析器
static void SCI_ParseByte(Uint16 receivedByte);//协议状态机的核心函数

void Task_Comm_Init(void)
{
    SCI_ResetParser();
    SCI_RxDataPending = 0U;
    SCI_ProtocolCrcErrorCount = 0UL;
    SCI_ProtocolFormatErrorCount = 0UL;
    SCI_ProtocolFrameCount = 0UL;
}

void Task_Comm(void)
{
    Uint16 receivedByte;
    Uint16 processedBytes = 0U;

    /* The RX ISR only announces data; protocol work runs at the slow task rate. */
    if(SCI_RxDataPending == 0U)//检查是否有数据
    {
        return;
    }

    /* Clear before draining so a new RX interrupt can leave the flag set. */
    SCI_RxDataPending = 0U;

    /* Bound the work per scheduler tick so communication cannot starve control. */
    //本次处理量没有超过限制值,且成功从软件环形缓冲区读出一个字节(多少不重要)
    while((processedBytes < SCI_TASK_MAX_RX_BYTES) && SCI_ReadByte(&receivedByte) != 0U)
    {
        SCI_ParseByte(receivedByte);//分析每一个八位数据(解析)
        processedBytes++;
    }
}

//计算 CRC, CRC-16/MODBUS 算法
static Uint16 SCI_Crc16Update(Uint16 crc, Uint16 data)
{
    Uint16 bitIndex;

    crc ^= data & 0x00FFU;
    for(bitIndex = 0U; bitIndex < 8U; bitIndex++)
    {
        if((crc & 0x0001U) != 0U)
        {
            crc = (crc >> 1U) ^ 0xA001U;
        }
        else
        {
            crc >>= 1U;
        }
    }
    return crc;
}

static void SCI_SendWordLE(Uint16 value)
{
    SCI_SendByte(value & 0x00FFU);
    SCI_SendByte((value >> 8U) & 0x00FFU);
}

//因为SCI是八位的,为得把16位数据拆开
static void SCI_PutWordLE(Uint16 *payload, Uint16 *index, Uint16 value)
{
    payload[*index] = value & 0x00FFU;
    (*index)++;
    payload[*index] = (value >> 8U) & 0x00FFU;
    (*index)++;
}

/* Serialize an IEEE-754 float as four little-endian payload bytes. */
static void SCI_PutFloatLE(Uint16 *payload, Uint16 *index, float value)
{
    union
    {
        float floatValue;
        Uint32 integerValue;
    } bits;

    bits.floatValue = value;
    payload[*index] = (Uint16)(bits.integerValue & 0x000000FFUL);
    (*index)++;
    payload[*index] = (Uint16)((bits.integerValue >> 8U) & 0x000000FFUL);
    (*index)++;
    payload[*index] = (Uint16)((bits.integerValue >> 16U) & 0x000000FFUL);
    (*index)++;
    payload[*index] = (Uint16)((bits.integerValue >> 24U) & 0x000000FFUL);
    (*index)++;
}

//把执行结果重新组装成带帧头、命令、序号、长度和 CRC16 的响应帧发送给上位机
static void SCI_PutDwordLE(Uint16 *payload, Uint16 *index, Uint32 value)
{
    payload[*index] = (Uint16)(value & 0x000000FFUL);
    (*index)++;
    payload[*index] = (Uint16)((value >> 8U) & 0x000000FFUL);
    (*index)++;
    payload[*index] = (Uint16)((value >> 16U) & 0x000000FFUL);
    (*index)++;
    payload[*index] = (Uint16)((value >> 24U) & 0x000000FFUL);
    (*index)++;
}

static void SCI_SendResponse(Uint16 command, Uint16 sequence, const Uint16 *payload, Uint16 payloadLength)
{
    Uint16 index;
    Uint16 crc = 0xFFFFU;
    Uint16 responseCommand = command | 0x0080U;

    if(payloadLength > SCI_FRAME_MAX_PAYLOAD)
    {
        payloadLength = 1U;
    }

    SCI_SendByte(SCI_FRAME_SOF1);
    SCI_SendByte(SCI_FRAME_SOF2);

    SCI_SendByte(responseCommand);
    crc = SCI_Crc16Update(crc, responseCommand);
    SCI_SendByte(sequence);
    crc = SCI_Crc16Update(crc, sequence);
    SCI_SendByte(payloadLength & 0x00FFU);
    crc = SCI_Crc16Update(crc, payloadLength & 0x00FFU);
    SCI_SendByte((payloadLength >> 8U) & 0x00FFU);
    crc = SCI_Crc16Update(crc, (payloadLength >> 8U) & 0x00FFU);

    for(index = 0U; index < payloadLength; index++)
    {
        SCI_SendByte(payload[index]);
        crc = SCI_Crc16Update(crc, payload[index]);
    }

    SCI_SendWordLE(crc);
}

//执行上位机的命令
static void SCI_HandleCommand(void)
{
    Uint16 responsePayload[SCI_FRAME_MAX_PAYLOAD];
    Uint16 responseLength = 1U;
    Uint16 requestedAmp;
    Uint16 responseIndex;

    responsePayload[0] = SCI_STATUS_OK;

    switch(SCI_ReceivedCommand)
    {
        case SCI_CMD_READ_REAL:
            /* Return calibrated average values, frequencies and reference. */
            responseIndex = 1U;
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.gridVoltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.inductorCurrent);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.gfciCurrent);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.dcBusVoltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.gridDcCurrent);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.inverterVoltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.pv1Current);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.pv2Current);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.pv1Voltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.pv2Voltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.pv1Isolation);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.pv2Isolation);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.inverterTemperature);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realAvg.boostTemperature);
            SCI_PutWordLE(responsePayload, &responseIndex, gMachineData.ecapFreqCent);
            SCI_PutWordLE(responsePayload, &responseIndex, gMachineData.pllFreqCent);
            SCI_PutWordLE(responsePayload, &responseIndex,
                          (Uint16)(gBusCtrlData.currentAmpRef * 4096.0f));
            responseLength = responseIndex;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        case SCI_CMD_READ_RMS:
            /* Return calibrated RMS values for all linear ADC channels. */
            responseIndex = 1U;
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.gridVoltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.inductorCurrent);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.gfciCurrent);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.dcBusVoltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.gridDcCurrent);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.inverterVoltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.pv1Current);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.pv2Current);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.pv1Voltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.pv2Voltage);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.pv1Isolation);
            SCI_PutFloatLE(responsePayload, &responseIndex, gMachineData.realRms.pv2Isolation);
            responseLength = responseIndex;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        case SCI_CMD_READ_PLL_STATUS:
            responseIndex = 1U;
            SCI_PutWordLE(responsePayload, &responseIndex, gMachineData.ecapFreqCent);
            SCI_PutWordLE(responsePayload, &responseIndex, gMachineData.pllFreqCent);
            responsePayload[responseIndex] =
                (gSysFault.bit.pllFault == 0U) ? 1U : 0U;
            responseIndex++;
            responseLength = responseIndex;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence, responsePayload, responseLength);
            break;

        case SCI_CMD_READ_CONTROL_STATUS:
            /* One snapshot for control commands, loop diagnostics and state. */
            responseIndex = 1U;
            SCI_PutFloatLE(responsePayload, &responseIndex, gBusCtrlData.stableVoltRef);
            SCI_PutFloatLE(responsePayload, &responseIndex, gBusCtrlData.currentAmpRef);
            SCI_PutFloatLE(responsePayload, &responseIndex, gBusCtrlData.boost1Duty);
            SCI_PutFloatLE(responsePayload, &responseIndex, gBusCtrlData.boost2Duty);
            SCI_PutWordLE(responsePayload, &responseIndex, (Uint16)gSysData.state);
            SCI_PutWordLE(responsePayload, &responseIndex, gMpptData.inputMode);
            SCI_PutWordLE(responsePayload, &responseIndex,
                          (gSysFault.bit.pllFault == 0U) ? 1U : 0U);
            SCI_PutWordLE(responsePayload, &responseIndex, gSysFault.bit.tzFault);
            SCI_PutDwordLE(responsePayload, &responseIndex, gSysFault.word.recoverable);
            SCI_PutDwordLE(responsePayload, &responseIndex, gSysFault.word.permanent);
            SCI_PutDwordLE(responsePayload, &responseIndex, gMachineData.measureSeq);
            responseLength = responseIndex;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        case SCI_CMD_READ_FAULT_STATUS:
            responseLength = 2U;
            responsePayload[1] = gSysFault.bit.tzFault;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence, responsePayload, responseLength);
            break;

        case SCI_CMD_SET_INDUCTOR_CUR_AMP:
            if(SCI_ReceivedLength != 2U)
            {
                responsePayload[0] = SCI_STATUS_BAD_LENGTH;
            }
            else
            {
                requestedAmp = SCI_ReceivedPayload[0] |
                               (SCI_ReceivedPayload[1] << 8U);
                if(requestedAmp > 4096U)
                {
                    responsePayload[0] = SCI_STATUS_BAD_PARAMETER;
                }
                else
                {
                    /* Q12: 4096 represents a normalized amplitude of 1.0.
                     * 手动电流上限：写 currentAmpMax，由 Task_Power 最终判断执行。 */
                    gPowerLimitData.currentAmpMax = (float)requestedAmp / 4096.0f;
                    responsePayload[1] = requestedAmp & 0x00FFU;
                    responsePayload[2] = (requestedAmp >> 8U) & 0x00FFU;
                    responseLength = 3U;
                }
            }
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        case SCI_CMD_CLEAR_FAULT:
            if(SCI_ReceivedLength != 0U)
            {
                responsePayload[0] = SCI_STATUS_BAD_LENGTH;
            }
            else
            {
                EPWM_TripZoneClear();
            }
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        case SCI_CMD_READ_VERSION:
            responseLength = 3U;
            responsePayload[1] = 2U; /* Protocol major version. */
            responsePayload[2] = 0U; /* Protocol minor version. */
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        default:
            responsePayload[0] = SCI_STATUS_BAD_COMMAND;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence, responsePayload, responseLength);
            break;
    }
}

static void SCI_ResetParser(void)
{
    SCI_ParseStateCurrent = SCI_PARSE_WAIT_SOF1;
    SCI_ReceivedCommand = 0U;
    SCI_ReceivedSequence = 0U;
    SCI_ReceivedLength = 0U;
    SCI_ReceivedPayloadIndex = 0U;
    SCI_ReceivedCrc = 0U;
    SCI_CalculatedCrc = 0xFFFFU;
}

//使用状态机依次识别 AA 55 命令 序号 长度 数据 CRC16,也就是数据帧接收
static void SCI_ParseByte(Uint16 receivedByte)
{
    receivedByte &= 0x00FFU;

    //其实这里是滑动的窗口
    switch(SCI_ParseStateCurrent)
    {

        //其实0xAA55就是1010 1010 0101 0101
        case SCI_PARSE_WAIT_SOF1://窗口滑动到帧头1
            if(receivedByte == SCI_FRAME_SOF1)
            {
                SCI_ParseStateCurrent = SCI_PARSE_WAIT_SOF2;//滑动窗口到帧头2
            }
            break;

        case SCI_PARSE_WAIT_SOF2://..
            if(receivedByte == SCI_FRAME_SOF2)
            {
                SCI_ParseStateCurrent = SCI_PARSE_COMMAND;//继续滑动...
                SCI_CalculatedCrc = 0xFFFFU;
            }
            else if(receivedByte != SCI_FRAME_SOF1)//如果已经是在帧头2了,但还是持续接收到帧头1,没关系,说明上位机多次尝试通讯,可以忍受
            {
                
                SCI_ResetParser();//如果已经是在帧头2了,接下来即收不到帧头1,也收不到帧头2,那就说明是偶发的
            }
            break;

        case SCI_PARSE_COMMAND:
            SCI_ReceivedCommand = receivedByte;//保存
            SCI_CalculatedCrc = SCI_Crc16Update(SCI_CalculatedCrc, receivedByte);//校验计算
            SCI_ParseStateCurrent = SCI_PARSE_SEQUENCE;//滑动
            break;

        case SCI_PARSE_SEQUENCE:
            SCI_ReceivedSequence = receivedByte;
            SCI_CalculatedCrc = SCI_Crc16Update(SCI_CalculatedCrc, receivedByte);
            SCI_ParseStateCurrent = SCI_PARSE_LENGTH_LOW;
            break;

        case SCI_PARSE_LENGTH_LOW:
            SCI_ReceivedLength = receivedByte;
            SCI_CalculatedCrc = SCI_Crc16Update(SCI_CalculatedCrc, receivedByte);
            SCI_ParseStateCurrent = SCI_PARSE_LENGTH_HIGH;
            break;

        case SCI_PARSE_LENGTH_HIGH:
            SCI_ReceivedLength |= receivedByte << 8U;
            SCI_CalculatedCrc = SCI_Crc16Update(SCI_CalculatedCrc, receivedByte);
            if(SCI_ReceivedLength > SCI_FRAME_MAX_PAYLOAD)
            {
                SCI_ProtocolFormatErrorCount++;
                SCI_ResetParser();
            }
            else if(SCI_ReceivedLength == 0U)//如果长度为零，就跳过 PAYLOAD，直接接收 CRC
            {
                SCI_ParseStateCurrent = SCI_PARSE_CRC_LOW;//滑动到CRC
            }
            else
            {
                SCI_ReceivedPayloadIndex = 0U;//知悉接下来会有多少数据了,准备接收
                SCI_ParseStateCurrent = SCI_PARSE_PAYLOAD;//滑动到接收窗口
            }
            break;

        case SCI_PARSE_PAYLOAD:
            SCI_ReceivedPayload[SCI_ReceivedPayloadIndex] = receivedByte;
            SCI_ReceivedPayloadIndex++;
            SCI_CalculatedCrc = SCI_Crc16Update(SCI_CalculatedCrc, receivedByte);
            if(SCI_ReceivedPayloadIndex >= SCI_ReceivedLength)
            {
                SCI_ParseStateCurrent = SCI_PARSE_CRC_LOW;//接收到了规定的量就进行CRC
            }
            break;

        case SCI_PARSE_CRC_LOW:
            SCI_ReceivedCrc = receivedByte;
            SCI_ParseStateCurrent = SCI_PARSE_CRC_HIGH;
            break;

        case SCI_PARSE_CRC_HIGH:
            SCI_ReceivedCrc |= receivedByte << 8U;
            if(SCI_ReceivedCrc == SCI_CalculatedCrc)
            {
                SCI_ProtocolFrameCount++;//正确次数加一
                SCI_HandleCommand();//给出回应
            }
            else
            {
                SCI_ProtocolCrcErrorCount++;//错误次数加一
            }
            SCI_ResetParser();//无论成功失败都准备下一次接收数据帧了
            break;

        default:
            SCI_ResetParser();
            break;
    }
}
