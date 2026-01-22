/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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


/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "platform.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @brief Length of global SPI RX/TX buffers.
 */
#define SPI_BUF_LEN (UINT8_MAX)

/**
 * @brief SPI read/write operation values.
 */
#define SPI_READ_OPER  (1U)
#define SPI_WRITE_OPER (0U)

/**
 * @brief DMA channel identifiers for SPI read and write.
 */
#define DMA_SPI_RX_CH (14U)
#define DMA_SPI_TX_CH (15U)

/**
 * @brief Number of ticks to occur per second.
 */
#define SYSTICKS_PER_SECOND (100U)

/* ========================================================================== */
/*                            Structures and Enums                            */
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
    uint8_t mainAddr;
    uint8_t secondaryAddr;
    bool bFast;
} I2cHandle_t;

typedef struct SpiHandle_s
{
    uint32_t sysPeriphSSI;
    uint32_t sysPeriphGPIO;
    uint32_t gpioPortBase;
    uint32_t ssiBase;
    uint32_t sclkPin;
    uint32_t csPin;
    uint32_t misoPin;
    uint32_t mosiPin;
    uint32_t gpioToSCLK;
    uint32_t gpioToCS;
    uint32_t gpioToMISO;
    uint32_t gpioToMOSI;
    uint32_t txMode;
    uint32_t operMode;
    uint32_t txFreq;
    uint32_t frameSize;
    uint32_t intNum;
} SpiHandle_t;

/**
 * @brief Platform UART information structure definition.
 */
typedef struct UartHandle_s
{
    uint32_t sysctlPeriphUART;
    uint32_t sysctlPeriphGPIO;
    uint32_t gpioPortBase;
    uint32_t uartBase;
    uint8_t gpioTxPin;
    uint8_t gpioRxPin;
    uint32_t TxPinToUART;
    uint32_t RxPinToUART;
    uint32_t dataLen;
    uint32_t stopBit;
    uint32_t parity;
    uint32_t clkSrc;
    uint32_t clkSrcFreq;
    uint32_t baudRate;
} UartHandle_t;

/**
 * @brief Platform Timer information structure definition.
 */
typedef struct TimerHandle_s
{
    uint32_t sysctlPeriphTimer;
    uint32_t timerBase;
} TimerHandle_t;

/**
 * @brief Platform GPIO pin type structure definition.
 */
typedef enum GpioType_e
{
    INVALID,
    INPUT,
    OUTPUT
} GpioType_t;

/**
 * @brief Platform GPIO pin handle structure definition.
 */
typedef struct GpioHandle_s {
    uint32_t sysPeriphGPIO;
    uint32_t gpioPortBase;
    uint32_t gpioPin;
    GpioType_t type;
} GpioHandle_t;

/**
 * @brief DMA channel handle structure definition.
 */
typedef struct DmaChHandle_s {
    uint32_t channel;
    uint32_t assignment;
    bool altSelect;
    bool useBurst;
    bool highPriority;
    bool reqMask;
    uint32_t dataSize;
    uint32_t srcInc;
    uint32_t dstInc;
    uint32_t arbSize;
} DmaChHandle_t;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief UART initialization/de-initialization APIs.
 */
static void uartInitModule(const UartHandle_t *uartHandle);
static void uartDeinitModule(const UartHandle_t *uartHandle);
static void vcpInitHandle(UartHandle_t *vcpHandle);
static void uartDeinitHandle(UartHandle_t *uartHandle);

/**
 * @brief Timer initialization/de-initialization APIs.
 */
static void timerInitModule(const TimerHandle_t *timerHandle);
static void timerDeinitModule(const TimerHandle_t *timerHandle);
static void timerInitHandle(TimerHandle_t *timerHandle);
static void timerDeinitHandle(TimerHandle_t *timerHandle);

/**
 * @brief GPIO initialization/de-initialization APIs.
 */
static void gpioInitPin(GpioHandle_t *gpioHandle);
static void gpioDeinitPin(GpioHandle_t *gpioHandle);
static void gpioDeinitHandle(GpioHandle_t *gpioHandle);
static void nCSInitHandle(GpioHandle_t *nCSHandle);
static void nCSAssert(const GpioHandle_t *nCSHandle, bool driveLow);
static void wakeupBlackbird(void);

/**
 * @brief SPI initialization/de-initialization APIs.
 */
static void spiInitModule(const SpiHandle_t *spiHandle);
static void spiInitModuleForDMA(const SpiHandle_t *spiHandle);
static void spiDeinitModule(const SpiHandle_t *spiHandle);
static void spiInitHandle(SpiHandle_t *spiHandle);
static void spiDeinitHandle(SpiHandle_t *spiHandle);

/**
 * @brief I2C initialization/de-initialization APIs.
 */
static void i2cInitModule(const I2cHandle_t *i2cHandle);
static void i2cDeinitModule(const I2cHandle_t *i2cHandle);
static void i2cInitHandle(I2cHandle_t *i2cHandle);
static void i2cDeinitHandle(I2cHandle_t *i2cHandle);

/**
 * @brief DMA initialization/de-initialization APIs.
 */
static void dmaInitModule(void);
static void dmaDeinitModule(void);
static void dmaInitCh(const DmaChHandle_t *dmaChHandle);
static void dmaDeinitCh(const DmaChHandle_t *dmaChHandle);
static void dmaDeinitChHandle(DmaChHandle_t *dmaChHandle);
static void dmaInitSpiTxChHandle(DmaChHandle_t *dmaChHandle);
static void dmaInitSpiRxChHandle(DmaChHandle_t *dmaChHandle);

/**
 * @brief UART operation APIs.
 */
static void uartStrPut(const UartHandle_t *uartHandle, const char *str);
static void vcpClearConsole(const UartHandle_t *vcpHandle);

/**
 * @brief I2C operation APIs.
 */
static int32_t i2cStartWrite(const I2cHandle_t *i2cHandle, uint16_t regAddr);
static int32_t i2cSingleWrite(const I2cHandle_t *i2cHandle, const uint8_t *txBuf);
static int32_t i2cBurstWrite(const I2cHandle_t *i2cHandle, uint8_t bufLen, const uint8_t *txBuf);
static int32_t i2cStartRead(const I2cHandle_t *i2cHandle, uint16_t regAddr);
static int32_t i2cBurstRead(const I2cHandle_t *i2cHandle, bool useSecondaryAddr, uint8_t bufLen, uint8_t *rxBuf);
static int32_t i2cSingleRead(const I2cHandle_t *i2cHandle, bool useSecondaryAddr, uint8_t *rxBuf);
static inline uint8_t decipherAddr(const I2cHandle_t *i2cHandle, uint16_t regAddr);

/**
 * @brief SPI operation APIs.
 */
static void spiTransferByte(const SpiHandle_t *spiHandle, uint32_t txByte, uint32_t *rxByte);
static int32_t spiWrite(const SpiHandle_t *spiHandle, uint16_t regAddr, const uint8_t *buffer, uint8_t bufLen);
static int32_t spiRead(const SpiHandle_t *spiHandle, uint16_t regAddr, uint8_t *buffer, uint8_t bufLen);

/**
 * @brief Interrupt handlers.
 */
void ssi3IntHandler(void);
void uDMAErrorHandler(void);
void sysTickIntHandler(void);

/**
 * @brief Miscellaneous APIs.
 */
static inline void waitForUserResponse(bool wait);
static int32_t asyncAwait(const struct Pmic_Handle_s *handle);

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

/**
 * @brief I2C handle used to communicate to PMIC.
 */
static I2cHandle_t i2cCommHandle = {0U};

/**
 * @brief SPI handle used to communicate to PMIC.
 */
static SpiHandle_t spiCommHandle = {0U};

/**
 * @brief UART handle used to transmit and receive console data.
 */
static UartHandle_t consoleHandle = {0U};

/**
 * @brief Timer handle used to facilitate activities involving time.
 */
static TimerHandle_t tHandle = {0U};

/**
 * @brief SPI negative chip select pin handle.
 */
static GpioHandle_t nCSPinHandle = {0U};

/**
 * @brief SPI read and write channel handles.
 */
static DmaChHandle_t dmaSpiReceiveChHandle = {0U};
static DmaChHandle_t dmaSpiTransmitChHandle = {0U};

/**
 * @brief Global SPI RX/TX buffers.
 */
static volatile uint8_t gSpiRxBuf[SPI_BUF_LEN] = {0U};
static volatile uint8_t gSpiTxBuf[SPI_BUF_LEN] = {0U};

/**
 * @brief Flags for asynchronous communication.
 */
static volatile bool awaitRxTransfer = (bool)false;
static volatile bool awaitTxTransfer = (bool)false;
static volatile bool dmaError = (bool)false;
static volatile bool spiError = (bool)false;

/**
 * @brief Control structure for DMA.
 */
#if defined(ccs)
#pragma DATA_ALIGN(dmaCtrlTable, 1024U)
uint8_t dmaCtrlTable[1024U];
#elif defined(__GNUC__) || defined(__clang__)
uint8_t dmaCtrlTable[1024U] __attribute__((aligned(1024U)));
#else
uint8_t dmaCtrlTable[1024U];
#endif

/**
 * @brief Used in system statistics/analytics.
 */
static volatile uint32_t gSeconds = 0U;
static volatile uint32_t cpuUsage = 0U;

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void platform_init(void)
{
    // Allow floating-point instructions to be used within interrupt handlers
    FPULazyStackingEnable();

    // Enable peripherals to operate when CPU is in sleep mode.
    SysCtlPeripheralClockGating((bool)true);

    // Initialize system clock
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Initialize console/terminal communication
    vcpInitHandle(&consoleHandle);
    uartInitModule(&consoleHandle);

    // Clear the console/terminal of any prior data
    vcpClearConsole(&consoleHandle);

    // Initialize PMIC communication
#if defined(USE_SPI) && defined(USE_DMA)
    nCSInitHandle(&nCSPinHandle);
    gpioInitPin(&nCSPinHandle);
    dmaInitModule();
    dmaInitSpiTxChHandle(&dmaSpiTransmitChHandle);
    dmaInitCh(&dmaSpiTransmitChHandle);
    dmaInitSpiRxChHandle(&dmaSpiReceiveChHandle);
    dmaInitCh(&dmaSpiReceiveChHandle);
    spiInitHandle(&spiCommHandle);
    spiInitModuleForDMA(&spiCommHandle);
#elif defined(USE_SPI)
    nCSInitHandle(&nCSPinHandle);
    gpioInitPin(&nCSPinHandle);
    spiInitHandle(&spiCommHandle);
    spiInitModule(&spiCommHandle);
#else
    i2cInitHandle(&i2cCommHandle);
    i2cInitModule(&i2cCommHandle);
#endif

    // Initialize timer
    timerInitHandle(&tHandle);
    timerInitModule(&tHandle);

#if defined(WAIT_FOR_USER_RESPONSE)
    waitForUserResponse((bool)true);
#else
    waitForUserResponse((bool)false);
#endif

    // Drive the Blackbird wakeup pin high
#if defined(USE_BLACKBIRD)
    wakeupBlackbird();
#endif
}

void platform_deinit(void)
{
    // De-initialize console/terminal communication
    uartDeinitModule(&consoleHandle);
    uartDeinitHandle(&consoleHandle);

    // Initialize PMIC communication
#if defined(USE_SPI) && defined(USE_DMA)
    dmaDeinitModule();
    dmaDeinitCh(&dmaSpiTransmitChHandle);
    dmaDeinitCh(&dmaSpiReceiveChHandle);
    dmaDeinitChHandle(&dmaSpiTransmitChHandle);
    dmaDeinitChHandle(&dmaSpiReceiveChHandle);
    spiDeinitModule(&spiCommHandle);
    spiDeinitHandle(&spiCommHandle);
    gpioDeinitPin(&nCSPinHandle);
    gpioDeinitHandle(&nCSPinHandle);
#elif defined(USE_SPI)
    spiDeinitModule(&spiCommHandle);
    spiDeinitHandle(&spiCommHandle);
    gpioDeinitPin(&nCSPinHandle);
    gpioDeinitHandle(&nCSPinHandle);
#else
    i2cDeinitHandle(&i2cCommHandle);
    i2cDeinitModule(&i2cCommHandle);
#endif

    // De-initialize timer
    timerDeinitModule(&tHandle);
    timerDeinitHandle(&tHandle);
}

void platform_setupTests(void)
{
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
    uartStrPut(&consoleHandle, str);
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

void platform_critSecStart(uint8_t resource)
{
    (void)resource;
    /* Empty - No RTOS */
}

void platform_critSecStop(uint8_t resource)
{
    (void)resource;
    /* Empty - No RTOS */
}

void platform_irqResponse(void)
{
    /* Empty - No response */
}

void *platform_getCommHandle(void)
{
    #ifdef USE_SPI
        return (void*)(&spiCommHandle);
    #else
        return (void*)(&i2cCommHandle);
    #endif
}

int32_t platform_txByte(const struct Pmic_Handle_s *handle, uint16_t regAddr, const uint8_t *buffer, uint8_t bufLen)
{
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (handle->commHandle == NULL) || (buffer == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (bufLen == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

#ifdef USE_SPI
    status = spiWrite((SpiHandle_t*)(handle->commHandle), regAddr, buffer, bufLen);
#else
    if (status == PMIC_ST_SUCCESS)
    {
        status = i2cStartWrite((I2cHandle_t*)(handle->commHandle), regAddr);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = (bufLen == 1U) ? i2cSingleWrite((I2cHandle_t*)(handle->commHandle), buffer) :
                                  i2cBurstWrite((I2cHandle_t*)(handle->commHandle), bufLen, buffer);
    }
#endif

    // The return code of the API I2CMasterErr() is positive when there is an I2C-related error
    if (status > 0U)
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}

int32_t platform_rxByte(const struct Pmic_Handle_s *handle, uint16_t regAddr, uint8_t *buffer, uint8_t bufLen)
{
    // Variable declaration/initialization
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (handle->commHandle == NULL) || (buffer == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (bufLen == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

#ifdef USE_SPI
    status = spiRead((SpiHandle_t*)(handle->commHandle), regAddr, buffer, bufLen);
#else
    if (status == PMIC_ST_SUCCESS)
    {
        status = i2cStartRead((I2cHandle_t*)(handle->commHandle), regAddr);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        const bool useSecondaryAddr =
            decipherAddr((I2cHandle_t*)(handle->commHandle), regAddr) == PLATFORM_I2C_ADDR_SECONDARY ? (bool)true : (bool)false;
        status = (bufLen == 1U) ? i2cSingleRead((I2cHandle_t*)(handle->commHandle), useSecondaryAddr, buffer) :
                                  i2cBurstRead((I2cHandle_t*)(handle->commHandle), useSecondaryAddr, bufLen, buffer);
    }
#endif

    // The return code of the API I2CMasterErr() is positive when there is an I2C-related error
    if (status > 0U)
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}

int32_t platform_asyncTxStart(const struct Pmic_Handle_s *handle, uint16_t regAddr, const uint8_t *buffer, uint8_t bufLen)
{
    const SpiHandle_t *spiHandle = (SpiHandle_t *)handle->commHandle;

    // buffer shall not be NULL and shall contain at least 24 bits
    if ((buffer == NULL) || (bufLen <= 2U))
    {
        return PMIC_ST_ERR_INV_PARAM;
    }

    // Configure DMA RX and TX transfer
    uDMAChannelTransferSet(
        dmaSpiReceiveChHandle.channel | UDMA_PRI_SELECT, UDMA_MODE_BASIC, (void *)(spiHandle->ssiBase + SSI_O_DR), (void *)gSpiRxBuf, bufLen);
    uDMAChannelTransferSet(
        dmaSpiTransmitChHandle.channel | UDMA_PRI_SELECT, UDMA_MODE_BASIC, (void *)buffer, (void *)(spiHandle->ssiBase + SSI_O_DR), bufLen);

    // Assert nCS
    nCSAssert(&nCSPinHandle, (bool)true);

    // Enable DMA RX and TX transfer
    uDMAChannelEnable(dmaSpiReceiveChHandle.channel);
    uDMAChannelEnable(dmaSpiTransmitChHandle.channel);

    // The caller of this function is now awaiting RX and TX transfers to complete
    awaitRxTransfer = (bool)true;
    awaitTxTransfer = (bool)true;
    return PMIC_ST_SUCCESS;
}

int32_t platform_asyncRxStart(const struct Pmic_Handle_s *handle, uint16_t regAddr, uint8_t *buffer, uint8_t bufLen)
{
    const SpiHandle_t *spiHandle = (SpiHandle_t *)handle->commHandle;

    // buffer shall not be NULL and shall contain at least 24 bits
    if ((buffer == NULL) || (bufLen <= 2U))
    {
        return PMIC_ST_ERR_INV_PARAM;
    }

    // Copy register address and page+R/W+reserved to global SPI TX buffer
    (void)memcpy((void*)gSpiTxBuf, (void*)buffer, bufLen);

    // Configure DMA RX and TX transfer
    uDMAChannelTransferSet(
        dmaSpiReceiveChHandle.channel | UDMA_PRI_SELECT, UDMA_MODE_BASIC, (void *)(spiHandle->ssiBase + SSI_O_DR), (void *)buffer, bufLen);
    uDMAChannelTransferSet(
        dmaSpiTransmitChHandle.channel | UDMA_PRI_SELECT, UDMA_MODE_BASIC, (void *)gSpiTxBuf, (void *)(spiHandle->ssiBase + SSI_O_DR), bufLen);

    // Assert nCS
    nCSAssert(&nCSPinHandle, (bool)true);

    // Enable DMA RX and TX transfer
    uDMAChannelEnable(dmaSpiReceiveChHandle.channel);
    uDMAChannelEnable(dmaSpiTransmitChHandle.channel);

    // The caller of this function is now awaiting RX and TX transfers to complete
    awaitRxTransfer = (bool)true;
    awaitTxTransfer = (bool)true;
    return PMIC_ST_SUCCESS;
}

static int32_t asyncAwait(const struct Pmic_Handle_s *handle)
{
    if (handle == NULL)
    {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    SysCtlSleep();
    while (awaitRxTransfer || awaitTxTransfer) {}

    // De-assert nCS after RX and TX transfers are complete
    nCSAssert(&nCSPinHandle, (bool)false);

    if (dmaError || spiError)
    {
        return PMIC_ST_ERR_SPI_COMM_FAIL;
    }

    return PMIC_ST_SUCCESS;
}

int32_t platform_asyncRxAwait(const struct Pmic_Handle_s *handle)
{
    return asyncAwait(handle);
}

int32_t platform_asyncTxAwait(const struct Pmic_Handle_s *handle)
{
    return asyncAwait(handle);
}

void platform_startSysMonitor(void)
{
    SysTickPeriodSet(SysCtlClockGet() / SYSTICKS_PER_SECOND);
    SysTickIntEnable();
    SysTickEnable();
    CPUUsageInit(SysCtlClockGet(), SYSTICKS_PER_SECOND, 2U);
}

void platform_stopSysMonitor(void)
{
    SysTickIntDisable();
    SysTickDisable();
}

uint32_t platform_getSysMonitorTime(void)
{
    return gSeconds;
}

void platform_setSysMonitorTime(uint32_t val)
{
    gSeconds = val;
}

uint32_t platform_getCpuUsageInteger(void)
{
    return cpuUsage >> 16U;
}

uint32_t platform_getCpuUsageRaw(void)
{
    return cpuUsage;
}

static void uartInitModule(const UartHandle_t *uartHandle)
{
    // Enable the UART module for PC <--> MCU communication
    SysCtlPeripheralEnable(uartHandle->sysctlPeriphUART);

    // Enable UART TX and RX pins' GPIO port
    SysCtlPeripheralEnable(uartHandle->sysctlPeriphGPIO);

    // Enable the UART and its GPIO port to operate in sleep mode
    SysCtlPeripheralSleepEnable(uartHandle->sysctlPeriphUART);
    SysCtlPeripheralSleepEnable(uartHandle->sysctlPeriphGPIO);

    // Ensure UART GPIO port is ready to be configured
    while (!SysCtlPeripheralReady(uartHandle->sysctlPeriphGPIO))
    {
    }

    // Configure UART GPIO TX and RX pins for UART operation
    GPIOPinTypeUART(uartHandle->gpioPortBase, uartHandle->gpioTxPin);
    GPIOPinTypeUART(uartHandle->gpioPortBase, uartHandle->gpioRxPin);

    // Configure UART GPIO TX and RX pins to UART functionality
    GPIOPinConfigure(uartHandle->TxPinToUART);
    GPIOPinConfigure(uartHandle->RxPinToUART);

    // Ensure UART is ready to be configured
    while (!SysCtlPeripheralReady(uartHandle->sysctlPeriphUART))
    {
    }

    // Disable the UART before configuration
    UARTDisable(uartHandle->uartBase);

    // Configure the UART
    UARTConfigSetExpClk(uartHandle->uartBase,
                        uartHandle->clkSrcFreq,
                        uartHandle->baudRate,
                        (uartHandle->dataLen | uartHandle->stopBit | uartHandle->parity));
    UARTClockSourceSet(uartHandle->uartBase, uartHandle->clkSrc);

    // Enable the UART after configuration
    UARTEnable(uartHandle->uartBase);
}

static void uartDeinitModule(const UartHandle_t *vcpHandle)
{
    UARTDisable(vcpHandle->uartBase);
    SysCtlPeripheralSleepDisable(vcpHandle->sysctlPeriphUART);
    SysCtlPeripheralDisable(vcpHandle->sysctlPeriphUART);
}

static void vcpInitHandle(UartHandle_t *vcpHandle)
{
    vcpHandle->sysctlPeriphUART = SYSCTL_PERIPH_UART0;
    vcpHandle->sysctlPeriphGPIO = SYSCTL_PERIPH_GPIOA;
    vcpHandle->gpioPortBase = GPIO_PORTA_BASE;
    vcpHandle->uartBase = UART0_BASE;
    vcpHandle->gpioTxPin = GPIO_PIN_1;
    vcpHandle->gpioRxPin = GPIO_PIN_0;
    vcpHandle->TxPinToUART = GPIO_PA1_U0TX;
    vcpHandle->RxPinToUART = GPIO_PA0_U0RX;
    vcpHandle->dataLen = UART_CONFIG_WLEN_8;
    vcpHandle->stopBit = UART_CONFIG_STOP_ONE;
    vcpHandle->parity = UART_CONFIG_PAR_NONE;
    vcpHandle->clkSrc = UART_CLOCK_PIOSC;
    vcpHandle->clkSrcFreq = 16000000U;
    vcpHandle->baudRate = 115200U;
}

static void uartDeinitHandle(UartHandle_t *vcpHandle)
{
    vcpHandle->sysctlPeriphUART = 0U;
    vcpHandle->sysctlPeriphGPIO = 0U;
    vcpHandle->gpioPortBase = 0U;
    vcpHandle->uartBase = 0U;
    vcpHandle->gpioTxPin = 0U;
    vcpHandle->gpioRxPin = 0U;
    vcpHandle->TxPinToUART = 0U;
    vcpHandle->RxPinToUART = 0U;
    vcpHandle->dataLen = 0U;
    vcpHandle->stopBit = 0U;
    vcpHandle->parity = 0U;
    vcpHandle->clkSrc = 0U;
    vcpHandle->clkSrcFreq = 0U;
    vcpHandle->baudRate = 0U;
}

static void timerInitHandle(TimerHandle_t *timerHandle)
{
    timerHandle->sysctlPeriphTimer = SYSCTL_PERIPH_TIMER0;
    timerHandle->timerBase = TIMER0_BASE;
}

static void timerDeinitHandle(TimerHandle_t *timerHandle)
{
    timerHandle->sysctlPeriphTimer = 0U;
    timerHandle->timerBase = 0U;
}

static void timerInitModule(const TimerHandle_t *timerHandle)
{
    // Enable the target timer peripheral
    SysCtlPeripheralEnable(timerHandle->sysctlPeriphTimer);

    // Enable the timer to operate in sleep mode
    SysCtlPeripheralSleepEnable(timerHandle->sysctlPeriphTimer);

    // Ensure timer is ready to be configured
    while (!SysCtlPeripheralReady(timerHandle->sysctlPeriphTimer))
    {
    }

    // Ensure timer is disabled so that other Timer APIs could safely configure the timer module
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);
}

static void timerDeinitModule(const TimerHandle_t *timerHandle)
{
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);
    SysCtlPeripheralSleepDisable(timerHandle->sysctlPeriphTimer);
    SysCtlPeripheralDisable(timerHandle->sysctlPeriphTimer);
}

static void i2cInitModule(const I2cHandle_t *i2cHandle)
{
    // Enable the I2C module
    SysCtlPeripheralEnable(i2cHandle->sysPeriphI2C);

    // Enable the I2C SDL and SDA pins' GPIO port
    SysCtlPeripheralEnable(i2cHandle->sysPeriphGPIO);

    // Enable the I2C module and its GPIO port to operate in sleep mode
    SysCtlPeripheralSleepEnable(i2cHandle->sysPeriphI2C);
    SysCtlPeripheralSleepEnable(i2cHandle->sysPeriphGPIO);

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

static void i2cDeinitModule(const I2cHandle_t *i2cHandle)
{
    SysCtlPeripheralSleepDisable(i2cHandle->sysPeriphI2C);
    SysCtlPeripheralDisable(i2cHandle->sysPeriphI2C);
}

static void i2cInitHandle(I2cHandle_t *i2cHandle)
{
    i2cHandle->sysPeriphI2C = SYSCTL_PERIPH_I2C0;
    i2cHandle->sysPeriphGPIO = SYSCTL_PERIPH_GPIOB;
    i2cHandle->gpioPortBase = GPIO_PORTB_BASE;
    i2cHandle->i2cBase = I2C0_BASE;
    i2cHandle->sdaPin = GPIO_PIN_3;
    i2cHandle->sclPin = GPIO_PIN_2;
    i2cHandle->gpioToSDA = GPIO_PB3_I2C0SDA;
    i2cHandle->gpioToSCL = GPIO_PB2_I2C0SCL;
    i2cHandle->mainAddr = PLATFORM_I2C_ADDR_MAIN;
    i2cHandle->secondaryAddr = PLATFORM_I2C_ADDR_SECONDARY;
    i2cHandle->bFast = (bool)false;
}

static void i2cDeinitHandle(I2cHandle_t *i2cHandle)
{
    i2cHandle->sysPeriphI2C = 0U;
    i2cHandle->sysPeriphGPIO = 0U;
    i2cHandle->gpioPortBase = 0U;
    i2cHandle->i2cBase = 0U;
    i2cHandle->sdaPin = 0U;
    i2cHandle->sclPin = 0U;
    i2cHandle->gpioToSDA = 0U;
    i2cHandle->gpioToSCL = 0U;
    i2cHandle->mainAddr = 0U;
    i2cHandle->secondaryAddr = 0U;
    i2cHandle->bFast = (bool)false;
}

static void dmaInitModule(void)
{
    // Enable DMA at the system level
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    // Enable DMA to operate during sleep mode
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    // Wait for DMA to be ready for configuration
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA)) {}

    // Set the DMA control table
    uDMAControlBaseSet(dmaCtrlTable);

    // Clear DMA interrupts
    uDMAIntClear(uDMAIntStatus());

    // Clear DMA error interrupt
    uDMAErrorStatusClear();

    // Enable DMA error interrupt
    IntEnable(INT_UDMAERR);

    // Enable DMA
    uDMAEnable();
}

static void dmaDeinitModule(void)
{
    uDMADisable();
    IntDisable(INT_UDMAERR);
    SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_UDMA);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_UDMA);
}

static void dmaInitCh(const DmaChHandle_t *dmaChHandle)
{
    uint32_t enabledAttributes = 0U;
    uint32_t disabledAttributes = 0U;

    if (dmaChHandle->altSelect == (bool)true)
    {
        enabledAttributes |= UDMA_ATTR_ALTSELECT;
    }
    else
    {
        disabledAttributes |= UDMA_ATTR_ALTSELECT;
    }

    if (dmaChHandle->useBurst == (bool)true)
    {
        enabledAttributes |= UDMA_ATTR_USEBURST;
    }
    else
    {
        disabledAttributes |= UDMA_ATTR_USEBURST;
    }

    if (dmaChHandle->highPriority == (bool)true)
    {
        enabledAttributes |= UDMA_ATTR_HIGH_PRIORITY;
    }
    else
    {
        disabledAttributes |= UDMA_ATTR_HIGH_PRIORITY;
    }

    if (dmaChHandle->reqMask == (bool)true)
    {
        enabledAttributes |= UDMA_ATTR_REQMASK;
    }
    else
    {
        disabledAttributes |= UDMA_ATTR_REQMASK;
    }

    // Disable channel attributes
    if (disabledAttributes != 0U)
    {
        uDMAChannelAttributeDisable(dmaChHandle->channel, disabledAttributes);
    }

    // Enable channel attributes
    if (enabledAttributes != 0U)
    {
        uDMAChannelAttributeEnable(dmaChHandle->channel, enabledAttributes);
    }

    // Set channel assignment
    uDMAChannelAssign(dmaChHandle->assignment);

    // Set channel control structure
    uDMAChannelControlSet(dmaChHandle->channel | UDMA_PRI_SELECT, dmaChHandle->dataSize | dmaChHandle->srcInc | dmaChHandle->dstInc | dmaChHandle->arbSize);
}

static void dmaDeinitCh(const DmaChHandle_t *dmaChHandle)
{
    /* Empty */
}

static void dmaDeinitChHandle(DmaChHandle_t *dmaChHandle)
{
    dmaChHandle->channel = 0U;
    dmaChHandle->assignment = 0U;
    dmaChHandle->altSelect = (bool)false;
    dmaChHandle->useBurst = (bool)false;
    dmaChHandle->highPriority = (bool)false;
    dmaChHandle->reqMask = (bool)false;
    dmaChHandle->dataSize = 0U;
    dmaChHandle->srcInc = 0U;
    dmaChHandle->dstInc = 0U;
    dmaChHandle->arbSize = 0U;
}

static void dmaInitSpiTxChHandle(DmaChHandle_t *dmaChHandle)
{
    dmaChHandle->channel = DMA_SPI_TX_CH;
    dmaChHandle->assignment = UDMA_CH15_SSI3TX;
    dmaChHandle->altSelect = (bool)false;
    dmaChHandle->useBurst = (bool)true;
    dmaChHandle->highPriority = (bool)false;
    dmaChHandle->reqMask = (bool)false;
    dmaChHandle->dataSize = UDMA_SIZE_8;
    dmaChHandle->srcInc = UDMA_SRC_INC_8;
    dmaChHandle->dstInc = UDMA_DST_INC_NONE;
    dmaChHandle->arbSize = UDMA_ARB_4;
}

static void dmaInitSpiRxChHandle(DmaChHandle_t *dmaChHandle)
{
    dmaChHandle->channel = DMA_SPI_RX_CH;
    dmaChHandle->assignment = UDMA_CH14_SSI3RX;
    dmaChHandle->altSelect = (bool)false;
    dmaChHandle->useBurst = (bool)false;
    dmaChHandle->highPriority = (bool)false;
    dmaChHandle->reqMask = (bool)false;
    dmaChHandle->dataSize = UDMA_SIZE_8;
    dmaChHandle->srcInc = UDMA_SRC_INC_NONE;
    dmaChHandle->dstInc = UDMA_DST_INC_8;
    dmaChHandle->arbSize = UDMA_ARB_1;
}

static void gpioInitPin(GpioHandle_t *gpioHandle)
{
    // Enable GPIO peripheral
    SysCtlPeripheralEnable(gpioHandle->sysPeriphGPIO);

    // Enable GPIO to operate during sleep mode
    SysCtlPeripheralSleepEnable(gpioHandle->sysPeriphGPIO);

    // Configure GPIO to desired type (e.g., input, output)
    switch (gpioHandle->type)
    {
        case INPUT:
        {
            GPIOPinTypeGPIOInput(gpioHandle->gpioPortBase, gpioHandle->gpioPin);
            break;
        }
        case OUTPUT:
        {
            GPIOPinTypeGPIOOutput(gpioHandle->gpioPortBase, gpioHandle->gpioPin);
            break;
        }
        default:
        {
            return;
        }
    }
}

static void gpioDeinitPin(GpioHandle_t *gpioHandle)
{
    /* Empty */
}

static void gpioDeinitHandle(GpioHandle_t *gpioHandle)
{
    gpioHandle->sysPeriphGPIO = 0U;
    gpioHandle->gpioPortBase = 0U;
    gpioHandle->gpioPin = 0U;
    gpioHandle->type = INVALID;
}

static void nCSInitHandle(GpioHandle_t *nCSHandle)
{
    nCSHandle->sysPeriphGPIO = SYSCTL_PERIPH_GPIOE;
    nCSHandle->gpioPortBase = GPIO_PORTE_BASE;
    nCSHandle->gpioPin = GPIO_PIN_1;
    nCSHandle->type = OUTPUT;
}

static void nCSAssert(const GpioHandle_t *nCSHandle, bool driveLow)
{
    if (driveLow)
    {
        GPIOPinWrite(nCSHandle->gpioPortBase, nCSHandle->gpioPin, 0U);
    }
    else
    {
        GPIOPinWrite(nCSHandle->gpioPortBase, nCSHandle->gpioPin, nCSHandle->gpioPin);
    }
}

static void wakeupBlackbird(void)
{
    if (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
        while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

static void spiInitModule(const SpiHandle_t *spiHandle)
{
    // De-assert SPI nCS
    nCSAssert(&nCSPinHandle, (bool)false);

    // Enable SPI module
    SysCtlPeripheralEnable(spiHandle->sysPeriphSSI);

    // Enable the GPIO peripheral in which the SPI pins reside in
    SysCtlPeripheralEnable(spiHandle->sysPeriphGPIO);

    // Enable SPI module and its GPIO port to operate in sleep mode
    SysCtlPeripheralSleepEnable(spiHandle->sysPeriphSSI);
    SysCtlPeripheralSleepEnable(spiHandle->sysPeriphGPIO);

    // Ensure SPI GPIO port is ready to be configured
    while (!SysCtlPeripheralReady(spiHandle->sysPeriphGPIO)) {}

    // Configure SPI GPIO pins for SPI operation
    GPIOPinConfigure(spiHandle->gpioToSCLK);
    GPIOPinConfigure(spiHandle->gpioToCS);
    GPIOPinConfigure(spiHandle->gpioToMISO);
    GPIOPinConfigure(spiHandle->gpioToMOSI);

    // Configure SPI GPIO pins to SPI functionality
    GPIOPinTypeSSI(GPIO_PORTD_BASE, spiHandle->sclkPin | spiHandle->csPin | spiHandle->misoPin | spiHandle->mosiPin);

    // Ensure the SPI module is ready to be configured
    while (!SysCtlPeripheralReady(spiHandle->sysPeriphSSI)) {}

    // Set SPI module to the following configurations:
    // System clock is input clock, CPOL=0, CPHA=0, master, 1 MHz frequency, 8-bit data size
    SSIConfigSetExpClk(spiHandle->ssiBase, SysCtlClockGet(), spiHandle->txMode, spiHandle->operMode, spiHandle->txFreq, spiHandle->frameSize);

    // Clear interrupts before enabling SPI
    SSIIntClear(SSI3_BASE, SSI_RXTO | SSI_RXOR);

    // Enable SPI module
    SSIEnable(SSI3_BASE);
}

static void spiInitModuleForDMA(const SpiHandle_t *spiHandle)
{
    spiInitModule(spiHandle);
    IntEnable(spiHandle->intNum);
    SSIDMAEnable(spiHandle->ssiBase, SSI_DMA_RX | SSI_DMA_TX);
}

static void spiDeinitModule(const SpiHandle_t *spiHandle)
{
    // De-assert SPI nCS
    nCSAssert(&nCSPinHandle, (bool)false);

    // Disable SPI module and DMA
    SSIDisable(spiHandle->ssiBase);
    SSIDMADisable(spiHandle->ssiBase, SSI_DMA_RX | SSI_DMA_TX);
    SysCtlPeripheralSleepDisable(spiHandle->sysPeriphSSI);
    SysCtlPeripheralDisable(spiHandle->sysPeriphSSI);
}

static void spiInitHandle(SpiHandle_t *spiHandle)
{
    spiHandle->sysPeriphSSI = SYSCTL_PERIPH_SSI3;
    spiHandle->sysPeriphGPIO = SYSCTL_PERIPH_GPIOD;
    spiHandle->gpioPortBase = GPIO_PORTD_BASE;
    spiHandle->ssiBase = SSI3_BASE;
    spiHandle->sclkPin = GPIO_PIN_0;
    spiHandle->csPin = GPIO_PIN_1;
    spiHandle->misoPin = GPIO_PIN_2;
    spiHandle->mosiPin = GPIO_PIN_3;
    spiHandle->gpioToSCLK = GPIO_PD0_SSI3CLK;
    spiHandle->gpioToCS = GPIO_PD1_SSI3FSS;
    spiHandle->gpioToMISO = GPIO_PD2_SSI3RX;
    spiHandle->gpioToMOSI = GPIO_PD3_SSI3TX;
    spiHandle->txMode = SSI_FRF_MOTO_MODE_0;
    spiHandle->operMode = SSI_MODE_MASTER;
    spiHandle->txFreq = 1000000U;
    spiHandle->frameSize = 8U;
    spiHandle->intNum = INT_SSI3;
}

static void spiDeinitHandle(SpiHandle_t *spiHandle)
{
    spiHandle->sysPeriphSSI = 0U;
    spiHandle->sysPeriphGPIO = 0U;
    spiHandle->gpioPortBase = 0U;
    spiHandle->ssiBase = 0U;
    spiHandle->sclkPin = 0U;
    spiHandle->csPin = 0U;
    spiHandle->misoPin = 0U;
    spiHandle->mosiPin = 0U;
    spiHandle->gpioToSCLK = 0U;
    spiHandle->gpioToCS = 0U;
    spiHandle->gpioToMISO = 0U;
    spiHandle->gpioToMOSI = 0U;
    spiHandle->txMode = 0U;
    spiHandle->operMode = 0U;
    spiHandle->txFreq = 0U;
    spiHandle->frameSize = 0U;
    spiHandle->intNum = 0U;
}

static inline void waitForUserResponse(bool wait)
{
    // Block CPU until user transmits a character to the MCU
    if (wait)
    {
        (void)UARTCharGet(consoleHandle.uartBase);
    }
}

static void uartStrPut(const UartHandle_t *uartHandle, const char *str)
{
    if ((str != NULL) && (uartHandle != NULL))
    {
        while (*str != '\0')
        {
            UARTCharPut(uartHandle->uartBase, *str);
            str++;
        }
    }
}

static void vcpClearConsole(const UartHandle_t *vcpHandle)
{
    if (vcpHandle != NULL)
    {
        uartStrPut(vcpHandle, "\033[2J");
        uartStrPut(vcpHandle, "\033[1;1H");
    }
}

static inline uint8_t decipherAddr(const I2cHandle_t *i2cHandle, uint16_t regAddr)
{
    if ((regAddr >> 8U) == 0x4U)
    {
        return i2cHandle->secondaryAddr;
    }
    else
    {
        return i2cHandle->mainAddr;
    }
}

static int32_t i2cStartWrite(const I2cHandle_t *i2cHandle, uint16_t regAddr)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t slaveAddr = decipherAddr(i2cHandle, regAddr);

    // Set target device I2C address and indicate that we want to write
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, slaveAddr, (bool)false);

    /*****************************************************************************/
    /**************** Transmitting internal register address frame ***************/
    /*****************************************************************************/

    // Put the target internal register address into the data register
    I2CMasterDataPut(i2cHandle->i2cBase, (regAddr & 0xFFU));

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

static int32_t i2cSingleWrite(const I2cHandle_t *i2cHandle, const uint8_t *txBuf)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Put data into the data register
    I2CMasterDataPut(i2cHandle->i2cBase, *txBuf);

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

static int32_t i2cBurstWrite(const I2cHandle_t *i2cHandle, uint8_t bufLen, const uint8_t *txBuf)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    for (i = 0U; i < bufLen; i++)
    {
        // Put data into the data register
        I2CMasterDataPut(i2cHandle->i2cBase, txBuf[i]);

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

static int32_t i2cStartRead(const I2cHandle_t *i2cHandle, uint16_t regAddr)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t slaveAddr = decipherAddr(i2cHandle, regAddr);

    // Set target device I2C address and indicate that we want to write
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, slaveAddr, (bool)false);

    /*****************************************************************************/
    /**************** Transmitting internal register address frame ***************/
    /*****************************************************************************/

    // Put the target internal register address into the data register
    I2CMasterDataPut(i2cHandle->i2cBase, (regAddr & 0xFFU));

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

static int32_t i2cBurstRead(const I2cHandle_t *i2cHandle, bool useSecondaryAddr, uint8_t bufLen, uint8_t *rxBuf)
{
    uint8_t i = 0U, slaveAddr = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    slaveAddr = useSecondaryAddr ? i2cHandle->secondaryAddr : i2cHandle->mainAddr;

    for (i = 0U; i < bufLen; i++)
    {
        // If beginning, set device I2C address, send the start condition, I2C address, and read bit
        if (i == 0U)
        {
            I2CMasterSlaveAddrSet(i2cHandle->i2cBase, slaveAddr, (bool)true);
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
            rxBuf[i] = I2CMasterDataGet(i2cHandle->i2cBase);
        }
        // Else if there is an error, stop reading
        else
        {
            break;
        }
    }

    return status;
}

static int32_t i2cSingleRead(const I2cHandle_t *i2cHandle, bool useSecondaryAddr, uint8_t *rxBuf)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t slaveAddr = useSecondaryAddr ? i2cHandle->secondaryAddr : i2cHandle->mainAddr;

    // Set target device I2C address and indicate that we want to read
    I2CMasterSlaveAddrSet(i2cHandle->i2cBase, slaveAddr, (bool)true);

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
        *rxBuf = I2CMasterDataGet(i2cHandle->i2cBase);
    }

    return status;
}

static void spiTransferByte(const SpiHandle_t *spiHandle, uint32_t txByte, uint32_t *rxByte)
{
    SSIDataPut(spiHandle->ssiBase, txByte);
    while (SSIBusy(spiHandle->ssiBase)) {}
    SSIDataGet(spiHandle->ssiBase, rxByte);
}

static int32_t spiWrite(const SpiHandle_t *spiHandle, uint16_t regAddr, const uint8_t *buffer, uint8_t bufLen)
{
    uint32_t temp = 0U;

    // buffer shall not be NULL and shall contain at least 24 bits
    if ((buffer == NULL) || (bufLen <= 2U))
    {
        return PMIC_ST_ERR_INV_PARAM;
    }

    // Assert nCS
    nCSAssert(&nCSPinHandle, (bool)true);

    // For each item in buffer...
    for (uint8_t i = 0U; i < bufLen; i++)
    {
        // Transmit buffer[i]; discard received byte
        spiTransferByte(spiHandle, (uint32_t)buffer[i], &temp);
    }

    // De-assert nCS
    nCSAssert(&nCSPinHandle, (bool)false);

    // Check SPI timeout and overrun statuses
    if (SSIIntStatus(spiHandle->ssiBase, (bool)true) != 0U)
    {
        return PMIC_ST_ERR_SPI_COMM_FAIL;
    }
    else
    {
        return PMIC_ST_SUCCESS;
    }
}

static int32_t spiRead(const SpiHandle_t *spiHandle, uint16_t regAddr, uint8_t *buffer, uint8_t bufLen)
{
    uint32_t temp = 0U;

    // buffer shall not be NULL and shall contain at least 24 bits
    if ((buffer == NULL) || (bufLen <= 2U))
    {
        return PMIC_ST_ERR_INV_PARAM;
    }

    // Assert nCS
    nCSAssert(&nCSPinHandle, (bool)true);

    // for each byte in buffer...
    for (uint8_t i = 0U; i < bufLen; i++)
    {
        // Transmit byte and save received byte
        spiTransferByte(spiHandle, buffer[i], &temp);
        buffer[i] = (uint8_t)(temp & 0xFFU);
    }

    // De-assert nCS
    nCSAssert(&nCSPinHandle, (bool)false);

    // Check SPI timeout and overrun statuses
    if (SSIIntStatus(spiHandle->ssiBase, (bool)true) != 0U)
    {
        return PMIC_ST_ERR_SPI_COMM_FAIL;
    }
    else
    {
        return PMIC_ST_SUCCESS;
    }
}

void ssi3IntHandler(void)
{
    IntMasterDisable();

    const uint32_t dmaChStatus = uDMAIntStatus();
    const uint32_t spiStatus = SSIIntStatus(SSI3_BASE, (bool)true);

    // Indicate and clear SPI statuses
    // NOTE: SPI interrupts should be disabled when using DMA for transfers
    if ((spiStatus & (SSI_RXTO | SSI_RXOR)) != 0U)
    {
        spiError = (bool)true;
    }
    else
    {
        spiError = (bool)false;
    }
    SSIIntClear(SSI3_BASE, SSI_RXTO | SSI_RXOR);

    // RX transfer finished
    if (!spiError && (uDMAChannelModeGet(DMA_SPI_RX_CH | UDMA_PRI_SELECT) == UDMA_MODE_STOP))
    {
        dmaError = (bool)false;
        awaitRxTransfer = (bool)false;
    }

    // TX transfer finished
    if (!spiError && (uDMAChannelModeGet(DMA_SPI_TX_CH | UDMA_PRI_SELECT) == UDMA_MODE_STOP))
    {
        dmaError = (bool)false;
        awaitTxTransfer = (bool)false;
    }

    // Clear DMA channel interrupts
    uDMAIntClear(dmaChStatus);

    IntMasterEnable();
}

void uDMAErrorHandler(void)
{
    IntMasterDisable();

    dmaError = (bool)true;
    uDMAErrorStatusClear();

    IntMasterEnable();
}

void sysTickIntHandler(void)
{
    static uint32_t tickCnt = 0;

    tickCnt++;
    if((tickCnt % SYSTICKS_PER_SECOND) == 0U)
    {
        platform_printChar('.');
        gSeconds++;
    }

    // Compute the amount of cycles used by the CPU since the last call
    // and return the result in percent in fixed point 16.16 format
    cpuUsage = CPUUsageTick();
}

void platform_unlockRegisters(void)
{
    #ifndef BUILD_MOCK
    // Hardware: Device-specific unlock sequence for TPS6522x-Q1
    // NOTE: Register unlock not needed for mock testing; implement for hardware tests
    #endif
    // Mock: No-op (mock doesn't enforce register locking)
}
