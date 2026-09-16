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
#define EEPROM_CONFIG_FLOAT_CNT  24U  //float数量
#define EEPROM_CONFIG_DATA_Bias  8U   //校准浮点数据的起始偏移地址：
#define EEPROM_CONFIG_CRC_Bias   (EEPROM_CONFIG_DATA_Bias + EEPROM_CONFIG_FLOAT_CNT * 4U)
#define EEPROM_CONFIG_BYTES        (EEPROM_CONFIG_CRC_Bias + 2U)

static volatile Uint16 EEPROM_SavePend = 0U;
static volatile Uint16 EEPROM_LastStatus = I2C_STATUS_OK;

static Uint16 EEPROM_Crc16(const Uint16 *data, Uint16 length);

static void EEPROM_PutUint16(Uint16 *data, Uint16 *idx, Uint16 value);
static void EEPROM_PutUint32(Uint16 *data, Uint16 *idx, Uint32 value);

static Uint16 EEPROM_GetUint16(const Uint16 *data, Uint16 *idx);
static Uint32 EEPROM_GetUint32(const Uint16 *data, Uint16 *idx);

static void EEPROM_PutFloat(Uint16 *data, Uint16 *idx, float value);
static float EEPROM_GetFloat(const Uint16 *data, Uint16 *idx);

static Uint16 EEPROM_ReadBytes(Uint16 address, Uint16 *data, Uint16 length);
static Uint16 EEPROM_WriteBytes(Uint16 address, const Uint16 *data, Uint16 length);

static Uint16 EEPROM_WaitWriteCycle(void);
static Uint16 EEPROM_SaveCalibration(void);

static Uint16 EEPROM_LoadCalibration(void);



static Uint16 EEPROM_Crc16(const Uint16 *data, Uint16 length)
{
    Uint16 crc = 0xFFFFU;
    Uint16 idx;
    Uint16 bit;
    Uint16 value;

    for(idx = 0U; idx < length; idx++)
    {
        value = (Uint16)data[idx];
        crc ^= (Uint16)(value << 8U);
        for(bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ? (Uint16)((crc << 1U) ^ 0x1021U)
                                   : (Uint16)(crc << 1U);
        }
    }
    return crc;
}

static void EEPROM_PutUint16(Uint16 *data, Uint16 *idx, Uint16 value)
{
    data[(*idx)++] = (Uint16)(value & 0x00FFU);
    data[(*idx)++] = (Uint16)(value >> 8U);
}

static void EEPROM_PutUint32(Uint16 *data, Uint16 *idx, Uint32 value)
{
    data[(*idx)++] = (Uint16)(value & 0x000000FFUL);
    data[(*idx)++] = (Uint16)((value >> 8U) & 0xFFUL);
    data[(*idx)++] = (Uint16)((value >> 16U) & 0xFFUL);
    data[(*idx)++] = (Uint16)((value >> 24U) & 0xFFUL);
}

static Uint16 EEPROM_GetUint16(const Uint16 *data, Uint16 *idx)
{
    Uint16 value = (Uint16)data[(*idx)++];
    value |= (Uint16)((Uint16)data[(*idx)++] << 8U);
    return value;
}

static Uint32 EEPROM_GetUint32(const Uint16 *data, Uint16 *idx)
{
    Uint32 value = (Uint32)data[(*idx)++];
    value |= (Uint32)data[(*idx)++] << 8U;
    value |= (Uint32)data[(*idx)++] << 16U;
    value |= (Uint32)data[(*idx)++] << 24U;
    return value;
}

static void EEPROM_PutFloat(Uint16 *data, Uint16 *idx, float value)
{
    union
    {
        float real;
        Uint32 bits;
    } encoded;

    encoded.real = value;
    EEPROM_PutUint32(data, idx, encoded.bits);
}

static float EEPROM_GetFloat(const Uint16 *data, Uint16 *idx)
{
    union
    {
        float real;
        Uint32 bits;
    } encoded;

    encoded.bits = EEPROM_GetUint32(data, idx);
    return encoded.real;
}

static Uint16 EEPROM_ReadBytes(Uint16 address, Uint16 *data, Uint16 length)
{
    Uint16 addressBytes[2];
    Uint16 chunk;
    Uint16 status;

    if(((Uint32)address + (Uint32)length) > EEPROM_CAPACITY_BYTES)
    {
        return I2C_STATUS_BAD_PARAM;
    }

    while(length != 0U)
    {
        /* Keep individual reads bounded so the I2C receive FIFO is drained. */
        chunk = (length > 32U) ? 32U : length;
        addressBytes[0] = (Uint16)(address >> 8U);
        addressBytes[1] = (Uint16)(address & 0x00FFU);
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

static Uint16 EEPROM_WriteBytes(Uint16 address, const Uint16 *data, Uint16 length)
{
    Uint16 tx[2U + EEPROM_PAGE_SIZE];
    Uint16 pageBias;
    Uint16 chunk;
    Uint16 idx;
    Uint16 status;

    if(((Uint32)address + (Uint32)length) > EEPROM_CAPACITY_BYTES)
    {
        return I2C_STATUS_BAD_PARAM;
    }

    while(length != 0U)
    {
        pageBias = (Uint16)(address % EEPROM_PAGE_SIZE);
        chunk = (Uint16)(EEPROM_PAGE_SIZE - pageBias);
        if(chunk > length)
        {
            chunk = length;
        }

        tx[0] = (Uint16)(address >> 8U);
        tx[1] = (Uint16)(address & 0x00FFU);
        for(idx = 0U; idx < chunk; idx++)
        {
            tx[2U + idx] = data[idx];
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
    Uint16 delta;
    Uint16 status;

    for(delta = 0U; delta < EEPROM_ACK_TIMEOUT_US; delta = (Uint16)(delta + EEPROM_ACK_POLL_STEP_US))
    {
        status = I2C_MasterWrite(EEPROM_I2C_ADDR_7BIT, 0, 0U, EEPROM_ACK_POLL_STEP_US);
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
    Uint16 record[EEPROM_CONFIG_BYTES];
    Uint16 idx = 0U;
    Uint16 crc;
    Uint16 interruptState;
    ADC_Calibrate cal;

    /* Copy once so a communication command cannot change half a record. */
    interruptState = CPU_InterruptSaveDisable();
    cal = gAdcCal;
    CPU_InterruptRestore(interruptState);

    EEPROM_PutUint32(record, &idx, EEPROM_CONFIG_MAGIC);
    EEPROM_PutUint16(record, &idx, EEPROM_CONFIG_VERSION);
    EEPROM_PutUint16(record, &idx, (Uint16)EEPROM_CONFIG_BYTES);
    EEPROM_PutFloat(record, &idx, cal.gridVolt.Bias);
    EEPROM_PutFloat(record, &idx, cal.gridVolt.gain);
    EEPROM_PutFloat(record, &idx, cal.inductCurr.Bias);
    EEPROM_PutFloat(record, &idx, cal.inductCurr.gain);
    EEPROM_PutFloat(record, &idx, cal.gfciCurr.Bias);
    EEPROM_PutFloat(record, &idx, cal.gfciCurr.gain);
    EEPROM_PutFloat(record, &idx, cal.dcBusVolt.Bias);
    EEPROM_PutFloat(record, &idx, cal.dcBusVolt.gain);
    EEPROM_PutFloat(record, &idx, cal.gridDcCurr.Bias);
    EEPROM_PutFloat(record, &idx, cal.gridDcCurr.gain);
    EEPROM_PutFloat(record, &idx, cal.invertVolt.Bias);
    EEPROM_PutFloat(record, &idx, cal.invertVolt.gain);
    EEPROM_PutFloat(record, &idx, cal.pv1Curr.Bias);
    EEPROM_PutFloat(record, &idx, cal.pv1Curr.gain);
    EEPROM_PutFloat(record, &idx, cal.pv2Curr.Bias);
    EEPROM_PutFloat(record, &idx, cal.pv2Curr.gain);
    EEPROM_PutFloat(record, &idx, cal.pv1Volt.Bias);
    EEPROM_PutFloat(record, &idx, cal.pv1Volt.gain);
    EEPROM_PutFloat(record, &idx, cal.pv2Volt.Bias);
    EEPROM_PutFloat(record, &idx, cal.pv2Volt.gain);
    EEPROM_PutFloat(record, &idx, cal.pv1Insul.Bias);
    EEPROM_PutFloat(record, &idx, cal.pv1Insul.gain);
    EEPROM_PutFloat(record, &idx, cal.pv2Insul.Bias);
    EEPROM_PutFloat(record, &idx, cal.pv2Insul.gain);

    crc = EEPROM_Crc16(record, EEPROM_CONFIG_CRC_Bias);
    EEPROM_PutUint16(record, &idx, crc);
    return EEPROM_WriteBytes(EEPROM_CONFIG_ADDRESS, record, EEPROM_CONFIG_BYTES);
}

static Uint16 EEPROM_LoadCalibration(void)
{
    Uint16 record[EEPROM_CONFIG_BYTES];
    Uint16 idx;
    Uint16 length;
    Uint16 status;
    Uint16 storedCrc;
    ADC_Calibrate cal;

    status = EEPROM_ReadBytes(EEPROM_CONFIG_ADDRESS, record, EEPROM_CONFIG_BYTES);
    if(status != I2C_STATUS_OK)
    {
        return status;
    }

    idx = 0U;
    if(EEPROM_GetUint32(record, &idx) != EEPROM_CONFIG_MAGIC)
    {
        return I2C_STATUS_NACK;
    }
    if(EEPROM_GetUint16(record, &idx) != EEPROM_CONFIG_VERSION)
    {
        return I2C_STATUS_NACK;
    }
    length = EEPROM_GetUint16(record, &idx);
    if(length != EEPROM_CONFIG_BYTES)
    {
        return I2C_STATUS_BAD_PARAM;
    }
    /* The CRC is located after all floats, not immediately after the header. */
    idx = EEPROM_CONFIG_CRC_Bias;
    storedCrc = EEPROM_GetUint16(record, &idx);
    if(storedCrc != EEPROM_Crc16(record, EEPROM_CONFIG_CRC_Bias))
    {
        return I2C_STATUS_NACK;
    }

    idx = EEPROM_CONFIG_DATA_Bias;
    cal.gridVolt.Bias = EEPROM_GetFloat(record, &idx);
    cal.gridVolt.gain = EEPROM_GetFloat(record, &idx);
    cal.inductCurr.Bias = EEPROM_GetFloat(record, &idx);
    cal.inductCurr.gain = EEPROM_GetFloat(record, &idx);
    cal.gfciCurr.Bias = EEPROM_GetFloat(record, &idx);
    cal.gfciCurr.gain = EEPROM_GetFloat(record, &idx);
    cal.dcBusVolt.Bias = EEPROM_GetFloat(record, &idx);
    cal.dcBusVolt.gain = EEPROM_GetFloat(record, &idx);
    cal.gridDcCurr.Bias = EEPROM_GetFloat(record, &idx);
    cal.gridDcCurr.gain = EEPROM_GetFloat(record, &idx);
    cal.invertVolt.Bias = EEPROM_GetFloat(record, &idx);
    cal.invertVolt.gain = EEPROM_GetFloat(record, &idx);
    cal.pv1Curr.Bias = EEPROM_GetFloat(record, &idx);
    cal.pv1Curr.gain = EEPROM_GetFloat(record, &idx);
    cal.pv2Curr.Bias = EEPROM_GetFloat(record, &idx);
    cal.pv2Curr.gain = EEPROM_GetFloat(record, &idx);
    cal.pv1Volt.Bias = EEPROM_GetFloat(record, &idx);
    cal.pv1Volt.gain = EEPROM_GetFloat(record, &idx);
    cal.pv2Volt.Bias = EEPROM_GetFloat(record, &idx);
    cal.pv2Volt.gain = EEPROM_GetFloat(record, &idx);
    cal.pv1Insul.Bias = EEPROM_GetFloat(record, &idx);
    cal.pv1Insul.gain = EEPROM_GetFloat(record, &idx);
    cal.pv2Insul.Bias = EEPROM_GetFloat(record, &idx);
    cal.pv2Insul.gain = EEPROM_GetFloat(record, &idx);

    /* Initialization runs before main enables interrupts, so this assignment
     * must not execute EINT and accidentally start peripheral ISRs early. */
    gAdcCal = cal;
    return I2C_STATUS_OK;
}

void Task_Eeprom_Init(void)
{
    EEPROM_SavePend = 0U;
    EEPROM_LastStatus = EEPROM_LoadCalibration();
    /* Invalid/uninitialized EEPROM leaves the compiled-in defaults untouched. */
}

void Task_Eeprom(void)
{
    if(EEPROM_SavePend != 0U)
    {
        EEPROM_LastStatus = EEPROM_SaveCalibration();
        if(EEPROM_LastStatus == I2C_STATUS_OK)
        {
            EEPROM_SavePend = 0U;
        }
    }
}

void EEPROM_RequestSave(Uint16 saveGroup)
{
    if(saveGroup == EEPROM_SAVE_CALIBRATION)
    {
        EEPROM_SavePend = 1U;
    }
}
