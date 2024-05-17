/**
 * @file common_test.c
 *
 * @brief Source file containing definitions of APIs used in all tests.
 */
#include "common_test.h"

void unityCharPut(uint8_t c)
{
    DebugP_log("%c", c);
    if (c == '\n')
    {
        DebugP_log("\r");
    }
}

void app_critSecStart(void)
{
    /* Empty - No RTOS */
}

void app_critSecStop(void)
{
    /* Empty - No RTOS */
}

int32_t app_ioWrite(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, const uint8_t *txBuf)
{
    I2C_Handle i2cHandle;
    I2C_Transaction i2cTransaction;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t writeBuf[3U] = {0U};

    // Parameter check
    if ((pmicHandle == NULL) || (txBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && ((bufLen == 0U) || (bufLen > 2U)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // writeBuf[0U]: Target device internal register address
        // writeBuf[1U] and onwards: txBuf
        writeBuf[0U] = regAddr;
        memcpy(&(writeBuf[1U]), txBuf, bufLen);

        // Initialize I2C handle and I2C transaction struct
        i2cHandle = *((I2C_Handle *)pmicHandle->commHandle);
        I2C_Transaction_init(&i2cTransaction);

        /*** Configure I2C transaction for a write ***/
        i2cTransaction.targetAddress = pmicHandle->i2cAddr;
        i2cTransaction.writeBuf = writeBuf;
        i2cTransaction.writeCount = bufLen + 1U;
        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0U;

        // Initiate write
        status = I2C_transfer(i2cHandle, &i2cTransaction);

        // Convert platform-specific success/error code to driver success/error code
        if (status != I2C_STS_SUCCESS)
        {
            status = PMIC_ST_ERR_I2C_COMM_FAIL;
        }
        else
        {
            status = PMIC_ST_SUCCESS;
        }
    }

    return status;
}

int32_t app_ioRead(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *rxBuf)
{
    I2C_Handle i2cHandle;
    I2C_Transaction i2cTransaction;
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter check
    if ((pmicHandle == NULL) || (rxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (bufLen == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Initialize I2C handle and I2C transaction struct
        i2cHandle = *((I2C_Handle *)pmicHandle->commHandle);
        I2C_Transaction_init(&i2cTransaction);

        /*** Configure I2C transaction for a read ***/
        i2cTransaction.targetAddress = pmicHandle->i2cAddr;
        i2cTransaction.writeBuf = &regAddr;
        i2cTransaction.writeCount = 1U;
        i2cTransaction.readBuf = rxBuf;
        i2cTransaction.readCount = bufLen;

        // Initiate read
        status = I2C_transfer(i2cHandle, &i2cTransaction);

        // Convert platform-specific success/error code to driver success/error code
        if (status != I2C_STS_SUCCESS)
        {
            status = PMIC_ST_ERR_I2C_COMM_FAIL;
        }
        else
        {
            status = PMIC_ST_SUCCESS;
        }
    }

    return status;
}
