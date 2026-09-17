#include "F28x_Project.h"

#include "task.h"
#include "bsp.h"
#include "variable.h"

/* Frame: AA 55 CMD SEQ LEN_L LEN_H PAYLOAD CRC_L CRC_H.
 * CRC-16/MODBUS covers CMD through the final payload byte. */
#define SCI_FRAME_SOF1                  0xAAU
#define SCI_FRAME_SOF2                  0x55U
#define SCI_FRAME_MAX_PAYLOAD           64U
#define SCI_FRAME_OVERHEAD              8U
#define SCI_TASK_MAX_RX_BYTES           512U

/* Read-only protocol commands. MachineData is split to stay below 64 bytes. */
#define SCI_CMD_READ_MACHINE_AVG        0x02U
#define SCI_CMD_READ_SYS_PROBLEM        0x04U
#define SCI_CMD_READ_MACHINE_RMS        0x05U
#define SCI_CMD_READ_MACHINE_POWER      0x07U
#define SCI_CMD_READ_SYSTEM_STATE       0x08U
#define SCI_CMD_READ_VERSION            0x30U

#define SCI_STATUS_OK                   0x00U
#define SCI_STATUS_BAD_LENG             0x01U
#define SCI_STATUS_BAD_CMD              0x02U

typedef enum
{
    SCI_PARSE_WAIT_SOF1 = 0U,
    SCI_PARSE_WAIT_SOF2,
    SCI_PARSE_CMD,
    SCI_PARSE_SEQ,
    SCI_PARSE_LENG_LOW,
    SCI_PARSE_LENG_HIGH,
    SCI_PARSE_PAYLOAD,
    SCI_PARSE_CRC_LOW,
    SCI_PARSE_CRC_HIGH
} SCI_ParseState;

static SCI_ParseState SCI_ParseStateCurr = SCI_PARSE_WAIT_SOF1;
static Uint16 SCI_RecevCmd = 0U;
static Uint16 SCI_RecevSeq = 0U;
static Uint16 SCI_RecevLeng = 0U;
static Uint16 SCI_RecevPayloadIdx = 0U;
static Uint16 SCI_RecevCrc = 0U;
static Uint16 SCI_CalCrc = 0xFFFFU;
static Uint16 SCI_TxFrame[SCI_FRAME_MAX_PAYLOAD + SCI_FRAME_OVERHEAD];

static volatile Uint32 SCI_ProtocCrcErrorCnt = 0UL;
static volatile Uint32 SCI_ProtocFormatErrorCnt = 0UL;
static volatile Uint32 SCI_ProtocFrameCnt = 0UL;

static Uint16 SCI_Crc16Update(Uint16 crc, Uint16 data);
static void SCI_PutWordLE(Uint16 *payload, Uint16 *idx, Uint16 value);
static void SCI_PutDwordLE(Uint16 *payload, Uint16 *idx, Uint32 value);
static void SCI_PutFloatLE(Uint16 *payload, Uint16 *idx, float value);
static void SCI_SendRespon(Uint16 command, Uint16 sequence,
                             const Uint16 *payload, Uint16 payloadLeng);
static void SCI_HandleCmd(void);
static void SCI_ResetParser(void);
static void SCI_ParseByte(Uint16 recevByte);

void Task_Comm_Init(void)
{
    SCI_ResetParser();
    SCI_ProtocCrcErrorCnt = 0UL;
    SCI_ProtocFormatErrorCnt = 0UL;
    SCI_ProtocFrameCnt = 0UL;
}

void Task_Comm(void)
{
    Uint16 recevByte;
    Uint16 doneBytes = 0U;

    while((doneBytes < SCI_TASK_MAX_RX_BYTES) &&
          (SCI_ReadByte(&recevByte) != 0U))
    {
        SCI_ParseByte(recevByte);
        doneBytes++;
    }
}

static Uint16 SCI_Crc16Update(Uint16 crc, Uint16 data)
{
    Uint16 bitIdx;

    crc ^= data & 0x00FFU;
    for(bitIdx = 0U; bitIdx < 8U; bitIdx++)
    {
        crc = ((crc & 0x0001U) != 0U) ?
              (Uint16)((crc >> 1U) ^ 0xA001U) : (Uint16)(crc >> 1U);
    }
    return crc;
}

static void SCI_PutWordLE(Uint16 *payload, Uint16 *idx, Uint16 value)
{
    payload[(*idx)++] = value & 0x00FFU;
    payload[(*idx)++] = (value >> 8U) & 0x00FFU;
}

static void SCI_PutDwordLE(Uint16 *payload, Uint16 *idx, Uint32 value)
{
    payload[(*idx)++] = (Uint16)(value & 0x000000FFUL);
    payload[(*idx)++] = (Uint16)((value >> 8U) & 0x000000FFUL);
    payload[(*idx)++] = (Uint16)((value >> 16U) & 0x000000FFUL);
    payload[(*idx)++] = (Uint16)((value >> 24U) & 0x000000FFUL);
}

static void SCI_PutFloatLE(Uint16 *payload, Uint16 *idx, float value)
{
    union
    {
        float floatValue;
        Uint32 uintValue;
    } bits;

    bits.floatValue = value;
    SCI_PutDwordLE(payload, idx, bits.uintValue);
}

static void SCI_SendRespon(Uint16 cmd, Uint16 seq, const Uint16 *payload, Uint16 payloadLeng)
{
    Uint16 idx;
    Uint16 frameIdx = 0U;
    Uint16 crc = 0xFFFFU;
    Uint16 responCmd = cmd | 0x0080U;

    if(payloadLeng > SCI_FRAME_MAX_PAYLOAD)
    {
        return;
    }

    SCI_TxFrame[frameIdx++] = SCI_FRAME_SOF1;
    SCI_TxFrame[frameIdx++] = SCI_FRAME_SOF2;
    SCI_TxFrame[frameIdx++] = responCmd;
    crc = SCI_Crc16Update(crc, responCmd);
    SCI_TxFrame[frameIdx++] = seq;
    crc = SCI_Crc16Update(crc, seq);
    SCI_TxFrame[frameIdx++] = payloadLeng & 0x00FFU;
    crc = SCI_Crc16Update(crc, payloadLeng & 0x00FFU);
    SCI_TxFrame[frameIdx++] = (payloadLeng >> 8U) & 0x00FFU;
    crc = SCI_Crc16Update(crc, (payloadLeng >> 8U) & 0x00FFU);

    for(idx = 0U; idx < payloadLeng; idx++)
    {
        SCI_TxFrame[frameIdx++] = payload[idx];
        crc = SCI_Crc16Update(crc, payload[idx]);
    }

    SCI_TxFrame[frameIdx++] = crc & 0x00FFU;
    SCI_TxFrame[frameIdx++] = (crc >> 8U) & 0x00FFU;
    (void)SCI_TrySend(SCI_TxFrame, frameIdx);
}

static void SCI_HandleCmd(void)
{
    Uint16 respon[SCI_FRAME_MAX_PAYLOAD];
    Uint16 responLeng = 1U;
    Uint16 idx;

    respon[0] = SCI_STATUS_OK;
    if(SCI_RecevLeng != 0U)
    {
        respon[0] = SCI_STATUS_BAD_LENG;
        SCI_SendRespon(SCI_RecevCmd, SCI_RecevSeq,
                         respon, responLeng);
        return;
    }

    idx = 1U;
    switch(SCI_RecevCmd)
    {
        case SCI_CMD_READ_MACHINE_AVG:
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.gridVolt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.inductCurr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.gfciCurr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.dcBusVolt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.gridDcCurr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.invertVolt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.pv1Curr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.pv2Curr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.pv1Volt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.pv2Volt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.pv1Insul);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.pv2Insul);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.invertTemp);
            SCI_PutFloatLE(respon, &idx, gMachineData.realAvg.boostTemp);
            responLeng = idx;
            break;

        case SCI_CMD_READ_MACHINE_RMS:
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.gridVolt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.inductCurr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.gfciCurr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.dcBusVolt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.gridDcCurr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.invertVolt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.pv1Curr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.pv2Curr);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.pv1Volt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.pv2Volt);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.pv1Insul);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.pv2Insul);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.invertTemp);
            SCI_PutFloatLE(respon, &idx, gMachineData.realRms.boostTemp);
            responLeng = idx;
            break;

        case SCI_CMD_READ_MACHINE_POWER:
            SCI_PutFloatLE(respon, &idx, gMachineData.powerData.gridActivePower);
            SCI_PutFloatLE(respon, &idx, gMachineData.powerData.gridReactivePower);
            SCI_PutFloatLE(respon, &idx, gMachineData.powerData.gridApparPower);
            SCI_PutFloatLE(respon, &idx, gMachineData.powerData.gridPF);
            SCI_PutFloatLE(respon, &idx, gMachineData.powerData.pv1Power);
            SCI_PutFloatLE(respon, &idx, gMachineData.powerData.pv2Power);
            SCI_PutWordLE(respon, &idx, gMachineData.ecapFreqCent);
            SCI_PutWordLE(respon, &idx, gMachineData.pllFreqCent);
            SCI_PutDwordLE(respon, &idx, gMachineData.measuSeq);
            responLeng = idx;
            break;

        case SCI_CMD_READ_SYS_PROBLEM:
            SCI_PutDwordLE(respon, &idx, gSysProblem.warning);
            SCI_PutDwordLE(respon, &idx, gSysProblem.recovFault);
            SCI_PutDwordLE(respon, &idx, gSysProblem.permaFault);
            responLeng = idx;
            break;

        case SCI_CMD_READ_SYSTEM_STATE:
            SCI_PutWordLE(respon, &idx, (Uint16)gSysData.state);
            SCI_PutWordLE(respon, &idx, (Uint16)gSysData.checkStage);
            SCI_PutWordLE(respon, &idx, gSysData.startReq);
            SCI_PutWordLE(respon, &idx, gSysData.sourceReady);
            SCI_PutWordLE(respon, &idx, gSysData.gridReady);
            SCI_PutWordLE(respon, &idx, gSysData.busReady);
            SCI_PutWordLE(respon, &idx, gSysData.reloadFlag);
            SCI_PutWordLE(respon, &idx, gSysData.reloadCnt);
            responLeng = idx;
            break;

        case SCI_CMD_READ_VERSION:
            respon[idx++] = 3U;
            respon[idx++] = 0U;
            responLeng = idx;
            break;

        default:
            respon[0] = SCI_STATUS_BAD_CMD;
            break;
    }

    SCI_SendRespon(SCI_RecevCmd, SCI_RecevSeq, respon, responLeng);
}

static void SCI_ResetParser(void)
{
    SCI_ParseStateCurr = SCI_PARSE_WAIT_SOF1;
    SCI_RecevCmd = 0U;
    SCI_RecevSeq = 0U;
    SCI_RecevLeng = 0U;
    SCI_RecevPayloadIdx = 0U;
    SCI_RecevCrc = 0U;
    SCI_CalCrc = 0xFFFFU;
}

static void SCI_ParseByte(Uint16 recevByte)
{
    recevByte &= 0x00FFU;

    switch(SCI_ParseStateCurr)
    {
        case SCI_PARSE_WAIT_SOF1:
            if(recevByte == SCI_FRAME_SOF1)
            {
                SCI_ParseStateCurr = SCI_PARSE_WAIT_SOF2;
            }
            break;

        case SCI_PARSE_WAIT_SOF2:
            if(recevByte == SCI_FRAME_SOF2)
            {
                SCI_CalCrc = 0xFFFFU;
                SCI_ParseStateCurr = SCI_PARSE_CMD;
            }
            else if(recevByte != SCI_FRAME_SOF1)
            {
                SCI_ResetParser();
            }
            break;

        case SCI_PARSE_CMD:
            SCI_RecevCmd = recevByte;
            SCI_CalCrc = SCI_Crc16Update(SCI_CalCrc, recevByte);
            SCI_ParseStateCurr = SCI_PARSE_SEQ;
            break;

        case SCI_PARSE_SEQ:
            SCI_RecevSeq = recevByte;
            SCI_CalCrc = SCI_Crc16Update(SCI_CalCrc, recevByte);
            SCI_ParseStateCurr = SCI_PARSE_LENG_LOW;
            break;

        case SCI_PARSE_LENG_LOW:
            SCI_RecevLeng = recevByte;
            SCI_CalCrc = SCI_Crc16Update(SCI_CalCrc, recevByte);
            SCI_ParseStateCurr = SCI_PARSE_LENG_HIGH;
            break;

        case SCI_PARSE_LENG_HIGH:
            SCI_RecevLeng |= recevByte << 8U;
            SCI_CalCrc = SCI_Crc16Update(SCI_CalCrc, recevByte);
            if(SCI_RecevLeng > SCI_FRAME_MAX_PAYLOAD)
            {
                SCI_ProtocFormatErrorCnt++;
                SCI_ResetParser();
            }
            else if(SCI_RecevLeng == 0U)
            {
                SCI_ParseStateCurr = SCI_PARSE_CRC_LOW;
            }
            else
            {
                SCI_RecevPayloadIdx = 0U;
                SCI_ParseStateCurr = SCI_PARSE_PAYLOAD;
            }
            break;

        case SCI_PARSE_PAYLOAD:
            SCI_RecevPayloadIdx++;
            SCI_CalCrc = SCI_Crc16Update(SCI_CalCrc, recevByte);
            if(SCI_RecevPayloadIdx >= SCI_RecevLeng)
            {
                SCI_ParseStateCurr = SCI_PARSE_CRC_LOW;
            }
            break;

        case SCI_PARSE_CRC_LOW:
            SCI_RecevCrc = recevByte;
            SCI_ParseStateCurr = SCI_PARSE_CRC_HIGH;
            break;

        case SCI_PARSE_CRC_HIGH:
            SCI_RecevCrc |= recevByte << 8U;
            if(SCI_RecevCrc == SCI_CalCrc)
            {
                SCI_ProtocFrameCnt++;
                SCI_HandleCmd();
            }
            else
            {
                SCI_ProtocCrcErrorCnt++;
            }
            SCI_ResetParser();
            break;

        default:
            SCI_ProtocFormatErrorCnt++;
            SCI_ResetParser();
            break;
    }
}
