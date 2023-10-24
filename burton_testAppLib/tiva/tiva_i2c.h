#ifndef TIVA_I2C_H_
#define TIVA_I2C_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* I2C handle struct definition */
typedef struct i2cHandle_s
{
    uint32_t sysPeriphI2C;
    uint32_t sysPeriphGPIO;
    uint32_t gpioPortBase;
    uint32_t i2cBase;
    uint32_t sdaPin;
    uint32_t sclPin;
    uint32_t sdaPinToI2C;
    uint32_t sclPinToI2C;
    uint8_t  slaveAddr;
} i2cHandle_t;

/**
 * \brief Given an I2C handle, this function initializes an I2C module on the Tiva.
 *
 * \param i2cHandle [IN] Handle that is used to initialize an I2C module on the Tiva
 */
void initializeI2C(i2cHandle_t *i2cHandle);

/**
 * \brief Function to initialize the handle to I2C1 on the Tiva.
 *
 * \param i2cHandle [OUT] Handle to initialize
 */
void initializeI2C1Handle(i2cHandle_t *i2cHandle);

/**
 * \brief Function to initialize the handle to I2C2 on the Tiva.
 *
 * \param i2cHandle [OUT] Handle to initialize
 */
void initializeI2C2Handle(i2cHandle_t *i2cHandle);

/**
 * \brief Function to write consecutive bytes to a target I2C device.
 *
 * \param i2cHandle [IN]        Handle to an I2C module on the Tiva
 * \param regAddr   [IN]        Target internal register address of the I2C device
 * \param bufLen    [IN]        Number of bytes to send consecutively over I2C
 * \param pTxBuf    [IN]        Pointer to the buffer containing the bytes to send over I2C
 *
 * \return          int32_t     PMIC_ST_SUCCESS in case of success or appropriate error code.
 *                              For valid values \ref Pmic_ErrorCodes.
 */
int32_t I2CBurstWrite(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *pRxBuf);

/**
 * \brief Function to write a single byte to a target I2C device.
 *
 * \param i2cHandle [IN]        Handle to an I2C module on the Tiva
 * \param regAddr   [IN]        Target internal register address of the I2C device
 * \param pTxBuf    [IN]        Pointer to the buffer containing the byte to send over I2C
 *
 * \return          int32_t     PMIC_ST_SUCCESS in case of success or appropriate error code.
 *                              For valid values \ref Pmic_ErrorCodes.
 */
int32_t I2CSingleWrite(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t *pRxBuf);

/**
 * \brief Function to read consecutive bytes from a target I2C device.
 *
 * \param i2cHandle [IN]        Handle to an I2C module on the Tiva
 * \param regAddr   [IN]        Target internal register address of the I2C device
 * \param bufLen    [IN]        Number of bytes to read consecutively over I2C
 * \param pRxBuf    [OUT]       Pointer to the buffer that stores bytes received over I2C
 *
 * \return          int32_t     PMIC_ST_SUCCESS in case of success or appropriate error code.
 *                              For valid values \ref Pmic_ErrorCodes.
 */
int32_t I2CBurstRead(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *pRxBuf);

/**
 * \brief Function to read a single byte from a target I2C device.
 *
 * \param i2cHandle [IN]        Handle to an I2C module on the Tiva
 * \param regAddr   [IN]        Target internal register address of the I2C device
 * \param pRxBuf    [OUT]       Pointer to the buffer that stores the byte received over I2C
 *
 * \return          int32_t     PMIC_ST_SUCCESS in case of success or appropriate error code.
 *                              For valid values \ref Pmic_ErrorCodes.
 */
int32_t I2CSingleRead(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t *pRxBuf);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_I2C_H_ */
