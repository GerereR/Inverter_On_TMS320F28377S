#include "F28x_Project.h"

#include "task.h"
#include "bsp.h"
#include "variable.h"

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

#define SCI_FRAME_MAX_PAYLOAD      32U//数据长度32字节
#define SCI_TASK_MAX_RX_BYTES      512U//通讯任务处理字节上限

/* Commands sent by the host computer. */
//命令宏
#define SCI_CMD_READ_MEASUREMENTS  0x01U
#define SCI_CMD_READ_PLL_STATUS    0x03U
#define SCI_CMD_READ_FAULT_STATUS  0x04U
#define SCI_CMD_SET_INDUCTOR_CURRENT_AMPL   0x10U  
#define SCI_CMD_CLEAR_FAULT        0x20U
#define SCI_CMD_READ_VERSION       0x30U

/* Response status values placed at payload[0]. */
//状态宏
#define SCI_STATUS_OK              0x00U
#define SCI_STATUS_BAD_LENGTH      0x01U
#define SCI_STATUS_BAD_COMMAND     0x02U
#define SCI_STATUS_BAD_PARAMETER   0x03U

//协议解析器
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

//初始化当前状态
static SCI_ParseState SCI_ParseStateCurrent = SCI_PARSE_WAIT_SOF1;
static Uint16 SCI_ReceivedCommand = 0U;
static Uint16 SCI_ReceivedSequence = 0U;
static Uint16 SCI_ReceivedLength = 0U;
static Uint16 SCI_ReceivedPayload[SCI_FRAME_MAX_PAYLOAD];
static Uint16 SCI_ReceivedPayloadIndex = 0U;
static Uint16 SCI_ReceivedCrc = 0U;
static Uint16 SCI_CalculatedCrc = 0xFFFFU;

volatile Uint32 SCI_ProtocolCrcErrorCount = 0UL;
volatile Uint32 SCI_ProtocolFormatErrorCount = 0UL;
volatile Uint32 SCI_ProtocolFrameCount = 0UL;

/* Forward declarations keep the public task entry points near the top. */
static Uint16 SCI_Crc16Update(Uint16 crc, Uint16 data);//使用一个新字节更新 CRC16 校验值。
static void SCI_SendWordLE(Uint16 value);//以小端格式发送一个 16 位数据：
static void SCI_PutWordLE(Uint16 *payload, Uint16 *index, Uint16 value);//把一个 16 位数据按小端格式写入响应数据数组，并自动移动数组下标。

//组装并发送完整协议响应帧，包括：
static void SCI_SendResponse(Uint16 command, Uint16 sequence, const Uint16 *payload, Uint16 payloadLength);

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
    if(SCI_RxDataPending == 0U)
    {
        return;
    }

    /* Clear before draining so a new RX interrupt can leave the flag set. */
    SCI_RxDataPending = 0U;

    /* Bound the work per scheduler tick so communication cannot starve control. */
    while((processedBytes < SCI_TASK_MAX_RX_BYTES) && SCI_ReadByte(&receivedByte) != 0U)
    {
        SCI_ParseByte(receivedByte);
        processedBytes++;
    }
}

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

static void SCI_PutWordLE(Uint16 *payload, Uint16 *index, Uint16 value)
{
    payload[*index] = value & 0x00FFU;
    (*index)++;
    payload[*index] = (value >> 8U) & 0x00FFU;
    (*index)++;
}

static void SCI_SendResponse(Uint16 command,
                             Uint16 sequence,
                             const Uint16 *payload,
                             Uint16 payloadLength)
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

static void SCI_HandleCommand(void)
{
    Uint16 responsePayload[SCI_FRAME_MAX_PAYLOAD];
    Uint16 responseLength = 1U;
    Uint16 requestedAmplitude;
    Uint16 responseIndex;

    responsePayload[0] = SCI_STATUS_OK;

    switch(SCI_ReceivedCommand)
    {
        case SCI_CMD_READ_MEASUREMENTS:
            /* All measurements are returned as raw ADC codes for now. */
            responseIndex = 1U;
            SCI_PutWordLE(responsePayload, &responseIndex, gMachineData.gridVoltage);
            SCI_PutWordLE(responsePayload, &responseIndex, gMachineData.inductorCurrent);
            SCI_PutWordLE(responsePayload, &responseIndex, gMachineData.pvVoltage);
            SCI_PutWordLE(responsePayload, &responseIndex, gMachineData.pvCurrent);
            SCI_PutWordLE(responsePayload, &responseIndex,
                          gMachineData.gridFrequencyCentihertz);
            responsePayload[responseIndex] = gMachineData.tripZoneFaulted;
            responseIndex++;
            responseLength = responseIndex;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        case SCI_CMD_READ_PLL_STATUS:
            responseIndex = 1U;
            SCI_PutWordLE(responsePayload, &responseIndex,
                          gMachineData.gridFrequencyCentihertz);
            SCI_PutWordLE(responsePayload, &responseIndex,
                          gMachineData.pllPhaseMilliradian);
            responsePayload[responseIndex] = gMachineData.pllLocked;
            responseIndex++;
            responseLength = responseIndex;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        case SCI_CMD_READ_FAULT_STATUS:
            responseLength = 2U;
            responsePayload[1] = gMachineData.tripZoneFaulted;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        case SCI_CMD_SET_INDUCTOR_CURRENT_AMPL:       
            if(SCI_ReceivedLength != 2U)
            {
                responsePayload[0] = SCI_STATUS_BAD_LENGTH;
            }
            else
            {
                requestedAmplitude = SCI_ReceivedPayload[0] |
                                     (SCI_ReceivedPayload[1] << 8U);
                if(requestedAmplitude > 4096U)
                {
                    responsePayload[0] = SCI_STATUS_BAD_PARAMETER;
                }
                else
                {
                    /* Q12: 4096 represents a normalized amplitude of 1.0. */
                    OpenLoopInductorCurrentAmplitude =
                        (float)requestedAmplitude / 4096.0f;
                    responsePayload[1] = requestedAmplitude & 0x00FFU;
                    responsePayload[2] = (requestedAmplitude >> 8U) & 0x00FFU;
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
            responsePayload[1] = 1U; /* Protocol major version. */
            responsePayload[2] = 0U; /* Protocol minor version. */
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
            break;

        default:
            responsePayload[0] = SCI_STATUS_BAD_COMMAND;
            SCI_SendResponse(SCI_ReceivedCommand, SCI_ReceivedSequence,
                             responsePayload, responseLength);
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

static void SCI_ParseByte(Uint16 receivedByte)
{
    receivedByte &= 0x00FFU;

    switch(SCI_ParseStateCurrent)
    {
        case SCI_PARSE_WAIT_SOF1:
            if(receivedByte == SCI_FRAME_SOF1)
            {
                SCI_ParseStateCurrent = SCI_PARSE_WAIT_SOF2;
            }
            break;

        case SCI_PARSE_WAIT_SOF2:
            if(receivedByte == SCI_FRAME_SOF2)
            {
                SCI_ParseStateCurrent = SCI_PARSE_COMMAND;
                SCI_CalculatedCrc = 0xFFFFU;
            }
            else if(receivedByte != SCI_FRAME_SOF1)
            {
                SCI_ResetParser();
            }
            break;

        case SCI_PARSE_COMMAND:
            SCI_ReceivedCommand = receivedByte;
            SCI_CalculatedCrc = SCI_Crc16Update(SCI_CalculatedCrc, receivedByte);
            SCI_ParseStateCurrent = SCI_PARSE_SEQUENCE;
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
            else if(SCI_ReceivedLength == 0U)
            {
                SCI_ParseStateCurrent = SCI_PARSE_CRC_LOW;
            }
            else
            {
                SCI_ReceivedPayloadIndex = 0U;
                SCI_ParseStateCurrent = SCI_PARSE_PAYLOAD;
            }
            break;

        case SCI_PARSE_PAYLOAD:
            SCI_ReceivedPayload[SCI_ReceivedPayloadIndex] = receivedByte;
            SCI_ReceivedPayloadIndex++;
            SCI_CalculatedCrc = SCI_Crc16Update(SCI_CalculatedCrc, receivedByte);
            if(SCI_ReceivedPayloadIndex >= SCI_ReceivedLength)
            {
                SCI_ParseStateCurrent = SCI_PARSE_CRC_LOW;
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
                SCI_ProtocolFrameCount++;
                SCI_HandleCommand();
            }
            else
            {
                SCI_ProtocolCrcErrorCount++;
            }
            SCI_ResetParser();
            break;

        default:
            SCI_ResetParser();
            break;
    }
}
