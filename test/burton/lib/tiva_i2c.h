/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
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
    uint8_t sdaPin;
    uint8_t sclPin;
    uint32_t gpioToSDA;
    uint32_t gpioToSCL;
    uint8_t slaveAddr;
    bool bFast;
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
 * \brief Function to write data (bytes) to a target I2C device.
 *
 * \param i2cHandle [IN]        Handle to an I2C module on the Tiva
 * \param regAddr   [IN]        Target internal register address of the I2C device
 * \param bufLen    [IN]        Number of bytes to write consecutively over I2C
 * \param pTxBuf    [OUT]       Pointer to the buffer that contains bytes to write over I2C.
 *                              Index zero is the first byte to be sent and index bufLen-1
 *                              is the last byte to be sent
 *
 * \return          int32_t     PMIC_ST_SUCCESS when all bytes in \p pTxBuf has been transmitted,
 *                              error code otherwise. For valid values refer to \ref Pmic_ErrorCodes
 */
int32_t I2CWrite(const i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, const uint8_t *pTxBuf);

/**
 * \brief Function to read data (bytes) from a target I2C device.
 *
 * \param i2cHandle [IN]        Handle to an I2C module on the Tiva
 * \param regAddr   [IN]        Target internal register address of the I2C device
 * \param bufLen    [IN]        Number of bytes to read consecutively over I2C
 * \param pRxBuf    [OUT]       Pointer to the buffer that stores bytes received over I2C. 
 *                              Index zero is the first byte received and index bufLen-1
 *                              is the last byte received 
 *
 * \return          int32_t     PMIC_ST_SUCCESS when \p bufLen bytes are received over I2C and
 *                              stored in \p pRxBuf error code otherwise. For valid values refer
 *                              to \ref Pmic_ErrorCodes.
 */
int32_t I2CRead(const i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *pRxBuf);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_I2C_H_ */
