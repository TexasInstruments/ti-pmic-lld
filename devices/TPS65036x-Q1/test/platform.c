/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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
/**
 * @file platform.c
 *
 * @brief Source file containing definitions to platform-specific APIs used in
 * testing PMIC LLD.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "platform.h"

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @brief Platform I2C information structure definition.
 */
typedef struct I2cHandle_s
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
} I2cHandle_t;

/**
 * @brief Platform UART information structure definition.
 */
typedef struct uartHandle_s
{
    uint32_t sysctlPeriphUART;
    uint32_t sysctlPeriphGPIO;
    uint32_t gpioPortBase;
    uint32_t uartBase;
    uint8_t gpioTxPin;
    uint8_t gpioRxPin;
    uint32_t TxPinToUART;
    uint32_t RxPinToUART;
    uint32_t clkSrc;
    uint32_t clkSrcFreq;
    uint32_t baudRate;
} uartHandle_t;

/**
 * @brief Platform Timer information structure definition.
 */
typedef struct timerHandle_s
{
    uint32_t sysctlPeriphTimer;
    uint32_t timerBase;
} timerHandle_t;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static void vcpInitModule(const uartHandle_t *vcpHandle);
static void vcpDeinitModule(const uartHandle_t *vcpHandle);
static void vcpInitHandle(uartHandle_t *vcpHandle);
static void vcpDeinitHandle(uartHandle_t *vcpHandle);
static void timerInitModule(const timerHandle_t *timerHandle);
static void timerDeinitModule(const timerHandle_t *timerHandle);
static void timerInitHandle(timerHandle_t *timerHandle);
static void timerDeinitHandle(timerHandle_t *timerHandle);
static void I2CInitModule(const I2cHandle_t *i2cHandle);
static void I2CDeinitModule(const I2cHandle_t *i2cHandle);
static void I2CInitHandle(I2cHandle_t *i2cHandle);
static void I2CDeinitHandle(I2cHandle_t *i2cHandle);
static inline void waitForUserResponse(bool wait);
static void UARTStrPut(const uartHandle_t *UARTHandle, const char *str);
static void vcpClearConsole(const uartHandle_t *uartHandle);
static int32_t I2CStartWrite(const I2cHandle_t *i2cHandle, uint8_t regAddr);
static int32_t I2CSingleWrite(const I2cHandle_t *i2cHandle, const uint8_t *pTxBuf);
static int32_t I2CBurstWrite(const I2cHandle_t *i2cHandle, uint8_t bufLen, const uint8_t *pTxBuf);
static int32_t I2CStartRead(const I2cHandle_t *i2cHandle, uint8_t regAddr);
static int32_t I2CBurstRead(const I2cHandle_t *i2cHandle, uint8_t bufLen, uint8_t *pRxBuf);
static int32_t I2CSingleRead(const I2cHandle_t *i2cHandle, uint8_t *pRxBuf);

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

/**
 * @brief I2C handle used to communicate to PMIC.
 */
static I2cHandle_t commHandle = {0U};

/**
 * @brief UART handle used to transmit and receive console data.
 */
static uartHandle_t consoleHandle = {0U};

/**
 * @brief Timer handle used to facilitate activities involving time.
 */
static timerHandle_t tHandle = {0U};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void platform_init(void)
{
    // Initialize console/terminal communication
    vcpInitHandle(&consoleHandle);
    vcpInitModule(&consoleHandle);

    // Clear the console/terminal of any prior data
    vcpClearConsole(&consoleHandle);

    // Initialize PMIC I2C communication
    I2CInitHandle(&commHandle);
    I2CInitModule(&commHandle);

    // Initialize timer
    timerInitHandle(&tHandle);
    timerInitModule(&tHandle);
}

void platform_deinit(void)
{
    // De-initialize console/terminal communication
    vcpDeinitModule(&consoleHandle);
    vcpDeinitHandle(&consoleHandle);

    // De-initialize PMIC I2C communication
    I2CDeinitModule(&commHandle);
    I2CDeinitHandle(&commHandle);

    // De-initialize timer
    timerDeinitModule(&tHandle);
    timerDeinitHandle(&tHandle);
}

void platform_setupTests(void)
{
    waitForUserResponse((bool)true);
    UNITY_BEGIN();
}

void platform_tearDownTests(void)
{
    UNITY_END();
}

void platform_printChar(char c)
{
    UARTCharPut(consoleHandle.uartBase, c);
    if (c == '\n')
    {
        UARTCharPut(consoleHandle.uartBase, '\r');
    }
}

void platform_printString(const char *str)
{
    UARTStrPut(&consoleHandle, str);
}

void platform_timerWaitMs(uint16_t ms)
{
    uint32_t cycles = 0U;

    /*** Delay for specified number of seconds ***/

    // Disable timer before configuration
    TimerDisable(tHandle.timerBase, TIMER_BOTH);

    // Configure timer to be in full-width Periodic mode counting down
    TimerConfigure(tHandle.timerBase, 0x0U);
    TimerConfigure(tHandle.timerBase, TIMER_CFG_PERIODIC);

    // Configure timer to generate a 1 second period
    TimerLoadSet(tHandle.timerBase, TIMER_A, SysCtlClockGet());

    // Clear any pending timer interrupt
    TimerIntClear(tHandle.timerBase, TIMER_TIMA_TIMEOUT);

    // Enable timer and start counting
    TimerEnable(tHandle.timerBase, TIMER_A);

    while (ms >= 1000U)
    {
        // Wait for 1 second
        while (TimerIntStatus(tHandle.timerBase, false) != TIMER_TIMA_TIMEOUT)
        {
        }

        // Clear timeout IRQ and decrement milliseconds
        TimerIntClear(tHandle.timerBase, TIMER_TIMA_TIMEOUT);
        ms -= 1000U;
    }

    // Disable timer after target duration has been met
    TimerDisable(tHandle.timerBase, TIMER_BOTH);

    /*** Delay for specified fraction of a second ***/

    // Configure timer to be in full-width One-Shot mode counting down
    TimerConfigure(tHandle.timerBase, 0x0U);
    TimerConfigure(tHandle.timerBase, TIMER_CFG_ONE_SHOT);

    cycles = (uint32_t)((SysCtlClockGet() / 1000U) * ms);
    TimerLoadSet(tHandle.timerBase, TIMER_A, cycles);

    // Clear any pending timer interrupt
    TimerIntClear(tHandle.timerBase, TIMER_TIMA_TIMEOUT);

    // Enable timer and start counting
    TimerEnable(tHandle.timerBase, TIMER_A);

    // Wait the fraction of a second
    while (TimerIntStatus(tHandle.timerBase, false) != TIMER_TIMA_TIMEOUT)
    {
    }

    // Clear flag
    TimerIntClear(tHandle.timerBase, TIMER_TIMA_TIMEOUT);

    // Disable timer after target duration has been met
    TimerDisable(tHandle.timerBase, TIMER_BOTH);
}

void platform_critSecStart(void)
{
    /* Empty - No RTOS */
}

void platform_critSecStop(void)
{
    /* Empty - No RTOS */
}

void platform_irqResponse(void)
{
    /* Empty - No response */
}

void *platform_getCommHandle(void)
{
    return (void*)(&commHandle);
}

int32_t platform_txByte(
    const struct Pmic_CoreHandle_s *pmicCorehandle, uint8_t regAddr, uint8_t bufLen, const uint8_t *pTxBuf)
{
    int32_t status = PMIC_ST_SUCCESS;

    if ((pmicCorehandle == NULL) || (pmicCorehandle->commHandle == NULL) || (pTxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (bufLen == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = I2CStartWrite((I2cHandle_t*)(pmicCorehandle->commHandle), regAddr);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = (bufLen == 1U) ? I2CSingleWrite((I2cHandle_t*)(pmicCorehandle->commHandle), pTxBuf) :
                                  I2CBurstWrite((I2cHandle_t*)(pmicCorehandle->commHandle), bufLen, pTxBuf);
    }

    // The return code of the API I2CMasterErr() is positive when there is an I2C-related error
    if (status > 0U)
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}

int32_t platform_rxByte(
    const struct Pmic_CoreHandle_s *pmicCorehandle, uint8_t regAddr, uint8_t bufLen, uint8_t *pRxBuf)
{
    // Variable declaration/initialization
    int32_t status = PMIC_ST_SUCCESS;

    if ((pmicCorehandle == NULL) || (pmicCorehandle->commHandle == NULL) || (pRxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (bufLen == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = I2CStartRead((I2cHandle_t*)(pmicCorehandle->commHandle), regAddr);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = (bufLen == 1U) ? I2CSingleRead((I2cHandle_t*)(pmicCorehandle->commHandle), pRxBuf) :
                                  I2CBurstRead((I2cHandle_t*)(pmicCorehandle->commHandle), bufLen, pRxBuf);
    }

    // The return code of the API I2CMasterErr() is positive when there is an I2C-related error
    if (status > 0U)
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}

void vcpInitModule(const uartHandle_t *vcpHandle)
{
    // Enable the UART module for PC <--> MCU communication
    SysCtlPeripheralEnable(vcpHandle->sysctlPeriphUART);

    // Enable UART TX and RX pins' GPIO port
    SysCtlPeripheralEnable(vcpHandle->sysctlPeriphGPIO);

    // Ensure VCP GPIO port is ready to be configured
    while (!SysCtlPeripheralReady(vcpHandle->sysctlPeriphGPIO))
    {
    }

    // Configure VCP GPIO TX and RX pins for UART operation
    GPIOPinTypeUART(vcpHandle->gpioPortBase, vcpHandle->gpioTxPin);
    GPIOPinTypeUART(vcpHandle->gpioPortBase, vcpHandle->gpioRxPin);

    // Configure VCP GPIO TX and RX pins to UART functionality
    GPIOPinConfigure(vcpHandle->TxPinToUART);
    GPIOPinConfigure(vcpHandle->RxPinToUART);

    // Ensure VCP UART is ready to be configured
    while (!SysCtlPeripheralReady(vcpHandle->sysctlPeriphUART))
    {
    }

    // Disable the UART before configuration
    UARTDisable(vcpHandle->uartBase);

    // Configure the UART to the popular configuration:
    // 9600 baud, 8 bit data, one stop bit, no parity
    // w/ input clock PIOSC (16,000,000 Hz)
    UARTConfigSetExpClk(vcpHandle->uartBase,
                        vcpHandle->clkSrcFreq,
                        vcpHandle->baudRate,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTClockSourceSet(vcpHandle->uartBase, vcpHandle->clkSrc);

    // Enable the UART after configuration
    UARTEnable(vcpHandle->uartBase);
}

static void vcpDeinitModule(const uartHandle_t *vcpHandle)
{
    UARTDisable(vcpHandle->uartBase);
    SysCtlPeripheralDisable(vcpHandle->sysctlPeriphUART);
}

static void vcpInitHandle(uartHandle_t *vcpHandle)
{
    vcpHandle->sysctlPeriphUART = SYSCTL_PERIPH_UART0;
    vcpHandle->sysctlPeriphGPIO = SYSCTL_PERIPH_GPIOA;
    vcpHandle->gpioPortBase = GPIO_PORTA_BASE;
    vcpHandle->uartBase = UART0_BASE;
    vcpHandle->gpioTxPin = GPIO_PIN_1;
    vcpHandle->gpioRxPin = GPIO_PIN_0;
    vcpHandle->TxPinToUART = GPIO_PA1_U0TX;
    vcpHandle->RxPinToUART = GPIO_PA0_U0RX;
    vcpHandle->clkSrc = UART_CLOCK_PIOSC;
    vcpHandle->clkSrcFreq = 16000000U;
    vcpHandle->baudRate = 115200U;
}

static void vcpDeinitHandle(uartHandle_t *vcpHandle)
{
    vcpHandle->sysctlPeriphUART = 0U;
    vcpHandle->sysctlPeriphGPIO = 0U;
    vcpHandle->gpioPortBase = 0U;
    vcpHandle->uartBase = 0U;
    vcpHandle->gpioTxPin = 0U;
    vcpHandle->gpioRxPin = 0U;
    vcpHandle->TxPinToUART = 0U;
    vcpHandle->RxPinToUART = 0U;
    vcpHandle->clkSrc = 0U;
    vcpHandle->clkSrcFreq = 0U;
    vcpHandle->baudRate = 0U;
}

static void timerInitHandle(timerHandle_t *timerHandle)
{
    timerHandle->sysctlPeriphTimer = SYSCTL_PERIPH_TIMER0;
    timerHandle->timerBase = TIMER0_BASE;
}

static void timerDeinitHandle(timerHandle_t *timerHandle)
{
    timerHandle->sysctlPeriphTimer = 0U;
    timerHandle->timerBase = 0U;
}

static void timerInitModule(const timerHandle_t *timerHandle)
{
    // Enable the target timer peripheral
    SysCtlPeripheralEnable(timerHandle->sysctlPeriphTimer);

    // Ensure timer is ready to be configured
    while (!SysCtlPeripheralReady(timerHandle->sysctlPeriphTimer))
    {
    }

    // Ensure timer is disabled so that other Timer APIs could safely configure the timer module
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);
}

static void timerDeinitModule(const timerHandle_t *timerHandle)
{
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);
    SysCtlPeripheralDisable(timerHandle->sysctlPeriphTimer);
}

static void I2CInitModule(const I2cHandle_t *i2cHandle)
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

static void I2CDeinitModule(const I2cHandle_t *i2cHandle)
{
    SysCtlPeripheralDisable(i2cHandle->sysPeriphI2C);
}

static void I2CInitHandle(I2cHandle_t *i2cHandle)
{
    i2cHandle->sysPeriphI2C = SYSCTL_PERIPH_I2C0;
    i2cHandle->sysPeriphGPIO = SYSCTL_PERIPH_GPIOB;
    i2cHandle->gpioPortBase = GPIO_PORTB_BASE;
    i2cHandle->i2cBase = I2C0_BASE;
    i2cHandle->sdaPin = GPIO_PIN_3;
    i2cHandle->sclPin = GPIO_PIN_2;
    i2cHandle->gpioToSDA = GPIO_PB3_I2C0SDA;
    i2cHandle->gpioToSCL = GPIO_PB2_I2C0SCL;
    i2cHandle->slaveAddr = PLATFORM_TARGET_I2C_ADDR;
    i2cHandle->bFast = (bool)false;
}

static void I2CDeinitHandle(I2cHandle_t *i2cHandle)
{
    i2cHandle->sysPeriphI2C = 0U;
    i2cHandle->sysPeriphGPIO = 0U;
    i2cHandle->gpioPortBase = 0U;
    i2cHandle->i2cBase = 0U;
    i2cHandle->sdaPin = 0U;
    i2cHandle->sclPin = 0U;
    i2cHandle->gpioToSDA = 0U;
    i2cHandle->gpioToSCL = 0U;
    i2cHandle->slaveAddr = 0U;
    i2cHandle->bFast = (bool)false;
}

static inline void waitForUserResponse(bool wait)
{
    // Block CPU until user transmits a character to the MCU
    if (wait)
    {
        (void)UARTCharGet(consoleHandle.uartBase);
    }
}

static void UARTStrPut(const uartHandle_t *UARTHandle, const char *str)
{
    if ((str != NULL) && (UARTHandle != NULL))
    {
        while (*str != '\0')
        {
            UARTCharPut(UARTHandle->uartBase, *str);
            str++;
        }
    }
}

static void vcpClearConsole(const uartHandle_t *uartHandle)
{
    if (uartHandle != NULL)
    {
        UARTStrPut(uartHandle, "\033[2J");
        UARTStrPut(uartHandle, "\033[1;1H");
    }
}

static inline int32_t I2CStartWrite(const I2cHandle_t *i2cHandle, uint8_t regAddr)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Set target device I2C address and indicate that we want to write
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, (bool)false);

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

static inline int32_t I2CSingleWrite(const I2cHandle_t *i2cHandle, const uint8_t *pTxBuf)
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

static inline int32_t I2CBurstWrite(const I2cHandle_t *i2cHandle, uint8_t bufLen, const uint8_t *pTxBuf)
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

static inline int32_t I2CStartRead(const I2cHandle_t *i2cHandle, uint8_t regAddr)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Set target device I2C address and indicate that we want to write
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, (bool)false);

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

static inline int32_t I2CBurstRead(const I2cHandle_t *i2cHandle, uint8_t bufLen, uint8_t *pRxBuf)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    for (i = 0U; i < bufLen; i++)
    {
        // If beginning, set device I2C address, send the start condition, I2C address, and read bit
        if (i == 0U)
        {
            I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, (bool)true);
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

static inline int32_t I2CSingleRead(const I2cHandle_t *i2cHandle, uint8_t *pRxBuf)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Set target device I2C address and indicate that we want to read
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, i2cHandle->slaveAddr, (bool)true);

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
