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
#include "tiva_priv.h"
#include "tiva_i2c.h"

void initializeI2C(i2cHandle_t *i2cHandle)
{
    // Enable the I2C module
    SysCtlPeripheralEnable(i2cHandle->sysPeriphI2C);

    // Enable the I2C SDL and SDA pins' GPIO port
    SysCtlPeripheralEnable(i2cHandle->sysPeriphGPIO);

    // Ensure I2C GPIO port is ready to be configured
    while (!SysCtlPeripheralReady(i2cHandle->sysPeriphGPIO))
    {
    }

    // Configure I2C GPIO pins for I2C operation
    GPIOPinTypeI2C(i2cHandle->gpioPortBase, i2cHandle->sdaPin); // Configure SDA pin
    GPIOPinTypeI2CSCL(i2cHandle->gpioPortBase, i2cHandle->sclPin);

    // Configure the I2C GPIO pins to I2C functionality
    GPIOPinConfigure(i2cHandle->gpioToSDA);
    GPIOPinConfigure(i2cHandle->gpioToSCL);

    // Initialize the I2C module for use as a master
    // running at a clock rate of 100 KHz or 400 KHz
    I2CMasterInitExpClk(i2cHandle->i2cBase, SysCtlClockGet(), i2cHandle->bFast);
}

void initializeI2C1Handle(i2cHandle_t *i2cHandle)
{
    i2cHandle->sysPeriphI2C = SYSCTL_PERIPH_I2C0;
    i2cHandle->sysPeriphGPIO = SYSCTL_PERIPH_GPIOB;
    i2cHandle->gpioPortBase = GPIO_PORTB_BASE;
    i2cHandle->i2cBase = I2C0_BASE;
    i2cHandle->sdaPin = GPIO_PIN_3;
    i2cHandle->sclPin = GPIO_PIN_2;
    i2cHandle->gpioToSDA = GPIO_PB3_I2C0SDA;
    i2cHandle->gpioToSCL = GPIO_PB2_I2C0SCL;
    i2cHandle->slaveAddr = BURTON_I2C_USER_PAGE_ADDRESS;
    i2cHandle->bFast = false;
}

void initializeI2C2Handle(i2cHandle_t *i2cHandle)
{
    i2cHandle->sysPeriphI2C = SYSCTL_PERIPH_I2C1;
    i2cHandle->sysPeriphGPIO = SYSCTL_PERIPH_GPIOA;
    i2cHandle->gpioPortBase = GPIO_PORTA_BASE;
    i2cHandle->i2cBase = I2C1_BASE;
    i2cHandle->sdaPin = GPIO_PIN_7;
    i2cHandle->sclPin = GPIO_PIN_6;
    i2cHandle->gpioToSDA = GPIO_PA7_I2C1SDA;
    i2cHandle->gpioToSCL = GPIO_PA6_I2C1SCL;
    i2cHandle->slaveAddr = BURTON_I2C_WDG_PAGE_ADDRESS;
    i2cHandle->bFast = false;
}

static inline int32_t I2CStartWrite(const i2cHandle_t *i2cHandle, uint8_t regAddr)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Set target device I2C address and indicate that we want to write
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, false);

    /*****************************************************************************/
    /**************** Transmitting internal register address frame ***************/
    /*****************************************************************************/

    // Put the target internal register address into the data register
    I2CMasterDataPut(i2cHandle->i2cBase, regAddr);

    // Send the start condition, I2C address, write bit, and internal register addr
    I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait while the master is busy sending data to target I2C device
    while (I2CMasterBusy(i2cHandle->i2cBase))
    {
    }

    // Check to see if there is an error
    status = I2CMasterErr(i2cHandle->i2cBase);

    return status;
}

static inline int32_t I2CSingleWrite(const i2cHandle_t *i2cHandle, const uint8_t *pTxBuf)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Put data into the data register
    I2CMasterDataPut(i2cHandle->i2cBase, *pTxBuf);

    // Send the start bit, I2C address, write bit, and data across the bus
    I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_SEND_FINISH);

    // Wait while the master is busy writing data to I2C device
    while (I2CMasterBusy(i2cHandle->i2cBase))
    {
    }

    // Check to see if there is an error
    status = I2CMasterErr(i2cHandle->i2cBase);

    return status;
}

static inline int32_t I2CBurstWrite(const i2cHandle_t *i2cHandle, uint8_t bufLen, const uint8_t *pTxBuf)
{
    uint8_t i = 0;
    int32_t status = PMIC_ST_SUCCESS;

    for (i = 0; i < bufLen; i++)
    {
        // Put data into the data register
        I2CMasterDataPut(i2cHandle->i2cBase, pTxBuf[i]);

        // If on last iteration, Generate stop condition at end of transmission
        if ((bufLen - i) == 1U)
        {
            I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_SEND_FINISH);
        }
        // Else continue sending data
        else
        {
            I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_SEND_CONT);
        }

        // Wait while the master is busy writing data to I2C device
        while (I2CMasterBusy(i2cHandle->i2cBase))
        {
        }

        // Check to see if there is an error
        status = I2CMasterErr(i2cHandle->i2cBase);

        // If error, send stop bit 
        if (status != PMIC_ST_SUCCESS)
        {
            I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_SEND_STOP);
            break;
        }
    }

    return status;
}

int32_t I2CWrite(const i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, const uint8_t *pTxBuf)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter check
    if ((i2cHandle == NULL) || (pTxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = I2CStartWrite(i2cHandle, regAddr);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = (bufLen == 1U) ? I2CSingleWrite(i2cHandle, pTxBuf) : I2CBurstWrite(i2cHandle, bufLen, pTxBuf);
    }

    // The return code of the API I2CMasterErr() is positive when there is an I2C-related error
    if (status > 0)
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}

static inline int32_t I2CStartRead(const i2cHandle_t *i2cHandle, uint8_t regAddr)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Set target device I2C address and indicate that we want to write
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, false);

    /*****************************************************************************/
    /**************** Transmitting internal register address frame ***************/
    /*****************************************************************************/

    // Put the target internal register address into the data register
    I2CMasterDataPut(i2cHandle->i2cBase, regAddr);

    // Send the start condition, I2C address, write bit, and internal register addr
    I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait while the master is busy sending data to target I2C device
    while (I2CMasterBusy(i2cHandle->i2cBase))
    {
    }

    // Check to see if there is an error
    status = I2CMasterErr(i2cHandle->i2cBase);

    return status;
}

static inline int32_t I2CBurstRead(const i2cHandle_t *i2cHandle, uint8_t bufLen, uint8_t *pRxBuf)
{
    uint8_t i = 0;
    int32_t status = PMIC_ST_SUCCESS;

    for (i = 0; i < bufLen; i++)
    {
        // If beginning, set device I2C address, send the start condition, I2C address, and read bit
        if (i == 0U)
        {
            I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, true);
            I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_START);
        }
        // Else if on last iteration, send NACK to stop after next received byte
        else if ((bufLen - i) == 1U)
        {
            I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        }
        // Else send an ACK to indicate that we want to continue receiving
        else
        {
            I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        }

        // Wait while the master is busy receiving data from target I2C device
        while (I2CMasterBusy(i2cHandle->i2cBase))
        {
        }

        // Check if there is an error
        status = I2CMasterErr(i2cHandle->i2cBase);

        // If there is no error, read from data register 
        if (status == PMIC_ST_SUCCESS)
        {
            pRxBuf[i] = I2CMasterDataGet(i2cHandle->i2cBase);
        }
        // Else if there is an error, stop reading
        else
        {
            break;
        }
    }

    return status;
}

static inline int32_t I2CSingleRead(const i2cHandle_t *i2cHandle, uint8_t *pRxBuf)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Set target device I2C address and indicate that we want to read
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, true);

    // Send the start condition, I2C address, and read bit across the bus
    I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Wait while the master is busy reading data from target I2C device
    while (I2CMasterBusy(i2cHandle->i2cBase))
    {
    }

    // Check if there is an error
    status = I2CMasterErr(i2cHandle->i2cBase);

    // If there is no error, read from data register 
    if (status == PMIC_ST_SUCCESS)
    {
        *pRxBuf = I2CMasterDataGet(i2cHandle->i2cBase);
    }

    return status;
}

int32_t I2CRead(const i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *pRxBuf)
{
    // Variable declaration/initialization
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter check
    if ((i2cHandle == NULL) || (pRxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = I2CStartRead(i2cHandle, regAddr);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = (bufLen == 1U) ? I2CSingleRead(i2cHandle, pRxBuf) : I2CBurstRead(i2cHandle, bufLen, pRxBuf);
    }

    // The return code of the API I2CMasterErr() is positive when there is an I2C-related error
    if (status > 0)
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}
