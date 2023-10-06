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

int32_t initializeI2C(i2cHandle_t *i2cHandle);
int32_t initializeI2C1Handle(i2cHandle_t *i2cHandle);
int32_t initializeI2C2Handle(i2cHandle_t *i2cHandle);
int32_t I2CBurstRead(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *pRxBuf);
int32_t I2CBurstWrite(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *pRxBuf);
int32_t I2CSingleRead(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t *pRxBuf);
int32_t I2CSingleWrite(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t *pRxBuf);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_I2C_H_ */
