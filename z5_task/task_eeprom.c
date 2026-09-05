/* M24C64-RMN6TP persistence task.  EEPROM accesses stay in the cooperative
 * task context; ADC/PWM ISRs never wait for the I2C bus or EEPROM write cycle. */
#include "F28x_Project.h"
#include "task.h"
#include "bsp.h"
#include "variable.h"

#define EEPROM_I2C_ADDR_7BIT       0x50U    //芯片规格书定义的
#define EEPROM_CAPACITY_BYTES      8192UL   //0x2000 
#define EEPROM_PAGE_SIZE           32U      //0x20  共0x100页
#define EEPROM_WRITE_TIMEOUT_US    3000U    //3ms
#define EEPROM_READ_TIMEOUT_US     5000U    //5ms
#define EEPROM_ACK_TIMEOUT_US      10000U   //10ms
#define EEPROM_ACK_POLL_STEP_US    200U     //0.2ms

//其实从上到下就是数据的存储格式
#define EEPROM_CONFIG_ADDRESS      0U

#define EEPROM_CONFIG_MAGIC        0x4D243634UL /* "M$64" marker. */
#define EEPROM_CONFIG_VERSION      1U
#define EEPROM_CONFIG_FLOAT_COUNT  24U  //float数量
#define EEPROM_CONFIG_DATA_OFFSET  8U   //校准浮点数据的起始偏移地址：
#define EEPROM_CONFIG_CRC_OFFSET   (EEPROM_CONFIG_DATA_OFFSET + EEPROM_CONFIG_FLOAT_COUNT * 4U)
#define EEPROM_CONFIG_BYTES        (EEPROM_CONFIG_CRC_OFFSET + 2U)

static volatile Uint16 EEPROM_SavePending = 0U;
static volatile Uint16 EEPROM_LastStatus = I2C_STATUS_OK;

static Uint16 EEPROM_Crc16(const unsigned char *data, Uint16 length);

static void EEPROM_PutUint16(unsigned char *data, Uint16 *index, Uint16 value);
static void EEPROM_PutUint32(unsigned char *data, Uint16 *index, Uint32 value);

static Uint16 EEPROM_GetUint16(const unsigned char *data, Uint16 *index);
static Uint32 EEPROM_GetUint32(const unsigned char *data, Uint16 *index);

static void EEPROM_PutFloat(unsigned char *data, Uint16 *index, float value);
static float EEPROM_GetFloat(const unsigned char *data, Uint16 *index);

static Uint16 EEPROM_ReadBytes(Uint16 address, unsigned char *data, Uint16 length);
static Uint16 EEPROM_WriteBytes(Uint16 address, const unsigned char *data, Uint16 length);

static Uint16 EEPROM_WaitWriteCycle(void);
static Uint16 EEPROM_SaveCalibration(void);

static Uint16 EEPROM_LoadCalibration(void);



static Uint16 EEPROM_Crc16(const unsigned char *data, Uint16 length)
{
    Uint16 crc = 0xFFFFU;
    Uint16 index;
    Uint16 bit;
    Uint16 value;

    for(index = 0U; index < length; index++)
    {
        value = (Uint16)data[index];
        crc ^= (Uint16)(value << 8U);
        for(bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ? (Uint16)((crc << 1U) ^ 0x1021U)
                                   : (Uint16)(crc << 1U);
        }
    }
    return crc;
}

static void EEPROM_PutUint16(unsigned char *data, Uint16 *index, Uint16 value)
{
    data[(*index)++] = (unsigned char)(value & 0x00FFU);
    data[(*index)++] = (unsigned char)(value >> 8U);
}

static void EEPROM_PutUint32(unsigned char *data, Uint16 *index, Uint32 value)
{
    data[(*index)++] = (unsigned char)(value & 0x000000FFUL);
    data[(*index)++] = (unsigned char)((value >> 8U) & 0xFFUL);
    data[(*index)++] = (unsigned char)((value >> 16U) & 0xFFUL);
    data[(*index)++] = (unsigned char)((value >> 24U) & 0xFFUL);
}

static Uint16 EEPROM_GetUint16(const unsigned char *data, Uint16 *index)
{
    Uint16 value = (Uint16)data[(*index)++];
    value |= (Uint16)((Uint16)data[(*index)++] << 8U);
    return value;
}

static Uint32 EEPROM_GetUint32(const unsigned char *data, Uint16 *index)
{
    Uint32 value = (Uint32)data[(*index)++];
    value |= (Uint32)data[(*index)++] << 8U;
    value |= (Uint32)data[(*index)++] << 16U;
    value |= (Uint32)data[(*index)++] << 24U;
    return value;
}

static void EEPROM_PutFloat(unsigned char *data, Uint16 *index, float value)
{
    union
    {
        float real;
        Uint32 bits;
    } encoded;

    encoded.real = value;
    EEPROM_PutUint32(data, index, encoded.bits);
}

static float EEPROM_GetFloat(const unsigned char *data, Uint16 *index)
{
    union
    {
        float real;
        Uint32 bits;
    } encoded;

    encoded.bits = EEPROM_GetUint32(data, index);
    return encoded.real;
}

static Uint16 EEPROM_ReadBytes(Uint16 address, unsigned char *data, Uint16 length)
{
    unsigned char addressBytes[2];
    Uint16 chunk;
    Uint16 status;

    if(((Uint32)address + (Uint32)length) > EEPROM_CAPACITY_BYTES)
    {
        return I2C_STATUS_BAD_PARAMETER;
    }

    while(length != 0U)
    {
        /* Keep individual reads bounded so the I2C receive FIFO is drained. */
        chunk = (length > 32U) ? 32U : length;
        addressBytes[0] = (unsigned char)(address >> 8U);
        addressBytes[1] = (unsigned char)(address & 0x00FFU);
        status = I2C_MasterWriteRead(EEPROM_I2C_ADDR_7BIT,
                                     addressBytes, 2U,
                                     data, chunk,
                                     EEPROM_READ_TIMEOUT_US);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
        address = (Uint16)(address + chunk);
        data += chunk;
        length = (Uint16)(length - chunk);
    }
    return I2C_STATUS_OK;
}

static Uint16 EEPROM_WriteBytes(Uint16 address, const unsigned char *data, Uint16 length)
{
    unsigned char tx[2U + EEPROM_PAGE_SIZE];
    Uint16 pageOffset;
    Uint16 chunk;
    Uint16 index;
    Uint16 status;

    if(((Uint32)address + (Uint32)length) > EEPROM_CAPACITY_BYTES)
    {
        return I2C_STATUS_BAD_PARAMETER;
    }

    while(length != 0U)
    {
        pageOffset = (Uint16)(address % EEPROM_PAGE_SIZE);
        chunk = (Uint16)(EEPROM_PAGE_SIZE - pageOffset);
        if(chunk > length)
        {
            chunk = length;
        }

        tx[0] = (unsigned char)(address >> 8U);
        tx[1] = (unsigned char)(address & 0x00FFU);
        for(index = 0U; index < chunk; index++)
        {
            tx[2U + index] = data[index];
        }

        status = I2C_MasterWrite(EEPROM_I2C_ADDR_7BIT,
                                 tx, (Uint16)(chunk + 2U),
                                 EEPROM_WRITE_TIMEOUT_US);
        if(status != I2C_STATUS_OK)
        {
            return status;
        }
        status = EEPROM_WaitWriteCycle();
        if(status != I2C_STATUS_OK)
        {
            return status;
        }

        address = (Uint16)(address + chunk);
        data += chunk;
        length = (Uint16)(length - chunk);
    }
    return I2C_STATUS_OK;
}

static Uint16 EEPROM_WaitWriteCycle(void)
{
    Uint16 elapsed;
    Uint16 status;

    for(elapsed = 0U; elapsed < EEPROM_ACK_TIMEOUT_US; elapsed = (Uint16)(elapsed + EEPROM_ACK_POLL_STEP_US))
    {
        status = I2C_MasterProbe(EEPROM_I2C_ADDR_7BIT, EEPROM_ACK_POLL_STEP_US);
        if(status == I2C_STATUS_OK)
        {
            return I2C_STATUS_OK;
        }
        if((status != I2C_STATUS_NACK) && (status != I2C_STATUS_TIMEOUT))
        {
            return status;
        }
    }
    return I2C_STATUS_TIMEOUT;
}

static Uint16 EEPROM_SaveCalibration(void)
{
    unsigned char record[EEPROM_CONFIG_BYTES];
    Uint16 index = 0U;
    Uint16 crc;
    ADC_Calibrate cal;

    /* Copy once so a communication command cannot change half a record. */
    DINT;
    cal = gAdcCal;
    EINT;

    EEPROM_PutUint32(record, &index, EEPROM_CONFIG_MAGIC);
    EEPROM_PutUint16(record, &index, EEPROM_CONFIG_VERSION);
    EEPROM_PutUint16(record, &index, (Uint16)EEPROM_CONFIG_BYTES);
    EEPROM_PutFloat(record, &index, cal.gridVoltage.offset);
    EEPROM_PutFloat(record, &index, cal.gridVoltage.gain);
    EEPROM_PutFloat(record, &index, cal.inductorCurrent.offset);
    EEPROM_PutFloat(record, &index, cal.inductorCurrent.gain);
    EEPROM_PutFloat(record, &index, cal.gfciCurrent.offset);
    EEPROM_PutFloat(record, &index, cal.gfciCurrent.gain);
    EEPROM_PutFloat(record, &index, cal.dcBusVoltage.offset);
    EEPROM_PutFloat(record, &index, cal.dcBusVoltage.gain);
    EEPROM_PutFloat(record, &index, cal.gridDcCurrent.offset);
    EEPROM_PutFloat(record, &index, cal.gridDcCurrent.gain);
    EEPROM_PutFloat(record, &index, cal.inverterVoltage.offset);
    EEPROM_PutFloat(record, &index, cal.inverterVoltage.gain);
    EEPROM_PutFloat(record, &index, cal.pv1Current.offset);
    EEPROM_PutFloat(record, &index, cal.pv1Current.gain);
    EEPROM_PutFloat(record, &index, cal.pv2Current.offset);
    EEPROM_PutFloat(record, &index, cal.pv2Current.gain);
    EEPROM_PutFloat(record, &index, cal.pv1Voltage.offset);
    EEPROM_PutFloat(record, &index, cal.pv1Voltage.gain);
    EEPROM_PutFloat(record, &index, cal.pv2Voltage.offset);
    EEPROM_PutFloat(record, &index, cal.pv2Voltage.gain);
    EEPROM_PutFloat(record, &index, cal.pv1Isolation.offset);
    EEPROM_PutFloat(record, &index, cal.pv1Isolation.gain);
    EEPROM_PutFloat(record, &index, cal.pv2Isolation.offset);
    EEPROM_PutFloat(record, &index, cal.pv2Isolation.gain);

    crc = EEPROM_Crc16(record, EEPROM_CONFIG_CRC_OFFSET);
    EEPROM_PutUint16(record, &index, crc);
    return EEPROM_WriteBytes(EEPROM_CONFIG_ADDRESS, record, EEPROM_CONFIG_BYTES);
}

static Uint16 EEPROM_LoadCalibration(void)
{
    unsigned char record[EEPROM_CONFIG_BYTES];
    Uint16 index;
    Uint16 length;
    Uint16 status;
    Uint16 storedCrc;
    ADC_Calibrate cal;

    status = EEPROM_ReadBytes(EEPROM_CONFIG_ADDRESS, record, EEPROM_CONFIG_BYTES);
    if(status != I2C_STATUS_OK)
    {
        return status;
    }

    index = 0U;
    if(EEPROM_GetUint32(record, &index) != EEPROM_CONFIG_MAGIC)
    {
        return I2C_STATUS_NACK;
    }
    if(EEPROM_GetUint16(record, &index) != EEPROM_CONFIG_VERSION)
    {
        return I2C_STATUS_NACK;
    }
    length = EEPROM_GetUint16(record, &index);
    if(length != EEPROM_CONFIG_BYTES)
    {
        return I2C_STATUS_BAD_PARAMETER;
    }
    /* The CRC is located after all floats, not immediately after the header. */
    index = EEPROM_CONFIG_CRC_OFFSET;
    storedCrc = EEPROM_GetUint16(record, &index);
    if(storedCrc != EEPROM_Crc16(record, EEPROM_CONFIG_CRC_OFFSET))
    {
        return I2C_STATUS_NACK;
    }

    index = EEPROM_CONFIG_DATA_OFFSET;
    cal.gridVoltage.offset = EEPROM_GetFloat(record, &index);
    cal.gridVoltage.gain = EEPROM_GetFloat(record, &index);
    cal.inductorCurrent.offset = EEPROM_GetFloat(record, &index);
    cal.inductorCurrent.gain = EEPROM_GetFloat(record, &index);
    cal.gfciCurrent.offset = EEPROM_GetFloat(record, &index);
    cal.gfciCurrent.gain = EEPROM_GetFloat(record, &index);
    cal.dcBusVoltage.offset = EEPROM_GetFloat(record, &index);
    cal.dcBusVoltage.gain = EEPROM_GetFloat(record, &index);
    cal.gridDcCurrent.offset = EEPROM_GetFloat(record, &index);
    cal.gridDcCurrent.gain = EEPROM_GetFloat(record, &index);
    cal.inverterVoltage.offset = EEPROM_GetFloat(record, &index);
    cal.inverterVoltage.gain = EEPROM_GetFloat(record, &index);
    cal.pv1Current.offset = EEPROM_GetFloat(record, &index);
    cal.pv1Current.gain = EEPROM_GetFloat(record, &index);
    cal.pv2Current.offset = EEPROM_GetFloat(record, &index);
    cal.pv2Current.gain = EEPROM_GetFloat(record, &index);
    cal.pv1Voltage.offset = EEPROM_GetFloat(record, &index);
    cal.pv1Voltage.gain = EEPROM_GetFloat(record, &index);
    cal.pv2Voltage.offset = EEPROM_GetFloat(record, &index);
    cal.pv2Voltage.gain = EEPROM_GetFloat(record, &index);
    cal.pv1Isolation.offset = EEPROM_GetFloat(record, &index);
    cal.pv1Isolation.gain = EEPROM_GetFloat(record, &index);
    cal.pv2Isolation.offset = EEPROM_GetFloat(record, &index);
    cal.pv2Isolation.gain = EEPROM_GetFloat(record, &index);

    /* Initialization runs before main enables interrupts, so this assignment
     * must not execute EINT and accidentally start peripheral ISRs early. */
    gAdcCal = cal;
    return I2C_STATUS_OK;
}

void Task_Eeprom_Init(void)
{
    EEPROM_SavePending = 0U;
    EEPROM_LastStatus = EEPROM_LoadCalibration();
    /* Invalid/uninitialized EEPROM leaves the compiled-in defaults untouched. */
}

void Task_Eeprom(void)
{
    if(EEPROM_SavePending != 0U)
    {
        EEPROM_LastStatus = EEPROM_SaveCalibration();
        if(EEPROM_LastStatus == I2C_STATUS_OK)
        {
            EEPROM_SavePending = 0U;
        }
    }
}

void EEPROM_RequestSave(Uint16 saveGroup)
{
    if(saveGroup == EEPROM_SAVE_CALIBRATION)
    {
        EEPROM_SavePending = 1U;
    }
}
