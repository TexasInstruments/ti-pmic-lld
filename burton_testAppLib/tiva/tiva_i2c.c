#include "tiva_priv.h"
#include "tiva_i2c.h"

/**
 * \brief Given an I2C handle, this function initializes an I2C module on the Tiva.
 *
 * \param i2cHandle [IN] Handle that is used to initialize an I2C module on the Tiva
 */
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
    GPIOPinConfigure(i2cHandle->sdaPinToI2C);
    GPIOPinConfigure(i2cHandle->sclPinToI2C);

    // Initialize the I2C module for use as a master running at a clock rate of 100 KHz
    I2CMasterInitExpClk(i2cHandle->i2cBase, SysCtlClockGet(), false);
}

/**
 * \brief Function to initialize the handle to I2C1 on the Tiva.
 *
 * \param i2cHandle [OUT] Handle to initialize
 */
void initializeI2C1Handle(i2cHandle_t *i2cHandle)
{
    i2cHandle->sysPeriphI2C = SYSCTL_PERIPH_I2C0;
    i2cHandle->sysPeriphGPIO = SYSCTL_PERIPH_GPIOB;
    i2cHandle->gpioPortBase = GPIO_PORTB_BASE;
    i2cHandle->i2cBase = I2C0_BASE;
    i2cHandle->sdaPin = GPIO_PIN_3;
    i2cHandle->sclPin = GPIO_PIN_2;
    i2cHandle->sdaPinToI2C = GPIO_PB3_I2C0SDA;
    i2cHandle->sclPinToI2C = GPIO_PB2_I2C0SCL;
    i2cHandle->slaveAddr = BURTON_I2C_USER_PAGE_ADDRESS;
}

/**
 * \brief Function to initialize the handle to I2C2 on the Tiva.
 *
 * \param i2cHandle [OUT] Handle to initialize
 */
void initializeI2C2Handle(i2cHandle_t *i2cHandle)
{
    i2cHandle->sysPeriphI2C = SYSCTL_PERIPH_I2C1;
    i2cHandle->sysPeriphGPIO = SYSCTL_PERIPH_GPIOA;
    i2cHandle->gpioPortBase = GPIO_PORTA_BASE;
    i2cHandle->i2cBase = I2C1_BASE;
    i2cHandle->sdaPin = GPIO_PIN_7;
    i2cHandle->sclPin = GPIO_PIN_6;
    i2cHandle->sdaPinToI2C = GPIO_PA7_I2C1SDA;
    i2cHandle->sclPinToI2C = GPIO_PA6_I2C1SCL;
    i2cHandle->slaveAddr = BURTON_I2C_WDG_PAGE_ADDRESS;
}

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
int32_t I2CBurstWrite(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *pTxBuf)
{
    // Variable declaration/initialization
    uint8_t i = 0;

    // Parameter check
    if ((i2cHandle == NULL) || (pTxBuf == NULL))
        return PMIC_ST_ERR_NULL_PARAM;
    if (bufLen < 2)
        return PMIC_ST_ERR_INV_PARAM;

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
    if (I2CMasterErr(i2cHandle->i2cBase) != I2C_MASTER_ERR_NONE)
        return PMIC_ST_ERR_I2C_COMM_FAIL;

    /*****************************************************************************/
    /************ Transmitting data to I2C device's internal register ************/
    /*****************************************************************************/

    for (i = 0; i < bufLen; i++)
    {
        // Put data into the data register
        I2CMasterDataPut(i2cHandle->i2cBase, pTxBuf[i]);

        // If on last iteration, Generate stop condition at end of transmission
        if ((bufLen - i) == 1)
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
        if (I2CMasterErr(i2cHandle->i2cBase) != I2C_MASTER_ERR_NONE)
            return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return PMIC_ST_SUCCESS;
}

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
int32_t I2CSingleWrite(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t *pTxBuf)
{
    // Parameter check
    if ((i2cHandle == NULL) || (pTxBuf == NULL))
        return PMIC_ST_ERR_NULL_PARAM;

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
    if (I2CMasterErr(i2cHandle->i2cBase) != I2C_MASTER_ERR_NONE)
        return PMIC_ST_ERR_I2C_COMM_FAIL;

    /*****************************************************************************/
    /************ Transmitting data to I2C device's internal register ************/
    /*****************************************************************************/

    // Put data into the data register
    I2CMasterDataPut(i2cHandle->i2cBase, *pTxBuf);

    // Send the start bit, I2C address, write bit, and data across the bus
    I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_SEND_FINISH);

    // Wait while the master is busy writing data to I2C device
    while (I2CMasterBusy(i2cHandle->i2cBase))
    {
    }

    // Check to see if there is an error
    if (I2CMasterErr(i2cHandle->i2cBase) != I2C_MASTER_ERR_NONE)
        return PMIC_ST_ERR_I2C_COMM_FAIL;

    return PMIC_ST_SUCCESS;
}

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
int32_t I2CBurstRead(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *pRxBuf)
{
    // Variable declaration/initialization
    uint8_t i = 0;

    // Parameter check
    if ((i2cHandle == NULL) || (pRxBuf == NULL))
        return PMIC_ST_ERR_NULL_PARAM;
    if (bufLen < 2)
        return PMIC_ST_ERR_INV_PARAM;

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
    if (I2CMasterErr(i2cHandle->i2cBase) != I2C_MASTER_ERR_NONE)
        return PMIC_ST_ERR_I2C_COMM_FAIL;

    /*****************************************************************************/
    /************ Receiving data from I2C device's internal register *************/
    /*****************************************************************************/

    for (i = 0; i < bufLen; i++)
    {
        // If beginning, send the start condition, I2C address, and read bit
        if (i == 0)
        {
            I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, true);
            I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_START);
        }
        // Else if on last iteration, send NACK to stop after next received byte
        else if ((bufLen - i) == 1)
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

        // Check to see if there is an error
        if (I2CMasterErr(i2cHandle->i2cBase) != I2C_MASTER_ERR_NONE)
            return PMIC_ST_ERR_I2C_COMM_FAIL;

        // Read from data register if there is no error
        pRxBuf[i] = I2CMasterDataGet(i2cHandle->i2cBase);
    }

    return PMIC_ST_SUCCESS;
}

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
int32_t I2CSingleRead(i2cHandle_t *i2cHandle, uint8_t regAddr, uint8_t *pRxBuf)
{
    // Parameter check
    if ((i2cHandle == NULL) || (pRxBuf == NULL))
        return PMIC_ST_ERR_NULL_PARAM;

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
    if (I2CMasterErr(i2cHandle->i2cBase) != I2C_MASTER_ERR_NONE)
        return PMIC_ST_ERR_I2C_COMM_FAIL;

    /*****************************************************************************/
    /************ Receiving data from I2C device's internal register *************/
    /*****************************************************************************/

    // Set target device I2C address and indicate that we want to read
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, true);

    // Send the start condition, I2C address, and read bit across the bus
    I2CMasterControl(i2cHandle->i2cBase, I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Wait while the master is busy reading data from target I2C device
    while (I2CMasterBusy(i2cHandle->i2cBase))
    {
    }

    // Check to see if there is an error
    if (I2CMasterErr(i2cHandle->i2cBase) != I2C_MASTER_ERR_NONE)
        return PMIC_ST_ERR_I2C_COMM_FAIL;

    // Read from data register if there is no error
    *pRxBuf = I2CMasterDataGet(i2cHandle->i2cBase);

    return PMIC_ST_SUCCESS;
}
