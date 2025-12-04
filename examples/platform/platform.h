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
#ifndef PLATFORM_H
#define PLATFORM_H

/* Platform part number. */
#ifndef PART_TM4C123GH6PM
#define PART_TM4C123GH6PM
#endif

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

/**
 * @brief Standard include(s).
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

/**
 * @brief LLD include.
 */
#include "pmic.h"

/**
 * @brief Testing framework include(s).
 */
#include "unity.h"

/* Platform-specific include(s) */
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/udma.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_ssi.h"
#include "utils/cpu_usage.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 * @brief Generic "invalid value" define used in test source codebase.
 *
 * @{
 */
#define PLATFORM_INVALID_VALUE (0x00U)
/** @} */

/**
 * @brief I2C address definitions for the platform (unused for LP8774x).
 *
 * @{
 */
#define PLATFORM_I2C_ADDR_MAIN      (0x00U)
#define PLATFORM_I2C_ADDR_SECONDARY (0x01U)
/** @} */

/**
 * @brief Macros/defines relating to testing framework.
 *
 * {
 */
#define PLATFORM_RUN_TEST(test)    RUN_TEST(test)
#define PLATFORM_ASSERT(condition) TEST_ASSERT(condition)
/** @} */

/**
 * @anchor Platform_GpioLevel
 * @name Platform GPIO Level
 *
 * @brief GPIO pin signal level definitions.
 * @{
 */
#define PLATFORM_GPIO_LOW  ((bool)false)
#define PLATFORM_GPIO_HIGH ((bool)true)
/** @} */

/* ========================================================================= */
/*                           Function Declarations                           */
/* ========================================================================= */

/**
 * @brief Initialize platform and its peripherals for testing LLD.
 */
void platform_init(void);

/**
 * @brief De-initialize platform and its peripherals.
 */
void platform_deinit(void);

/**
 * @brief Setup/initialize the testing framework to begin running tests.
 */
void platform_setupTests(void);

/**
 * @brief Halt/de-initialize the testing framework from running tests.
 */
void platform_tearDownTests(void);

/**
 * @brief Platform-specific API to write a character to the terminal/console.
 *
 * @details This API exists because some testing frameworks require an API to
 * write a single character. This is not used in test source code and is
 * optional.
 *
 * @param c [IN] Character to transmit to terminal/console.
 */
void platform_printChar(char c);

/**
 * @brief Platform-specific API to transmit a string to the terminal/console.
 *
 * @param str [IN] String to be transmitted.
 */
void platform_printString(const char *str);

/**
 * @brief Platform-specific API to block the CPU for a specific amount of time
 * in milliseconds.
 *
 * @param ms [IN] Time to wait in milliseconds.
 */
void platform_timerWaitMs(uint16_t ms);

/**
 * @brief Start platform-specific critical section.
 */
void platform_critSecStart(void);

/**
 * @brief Stop platform-specific critical section.
 */
void platform_critSecStop(void);

/**
 * @brief Platform-specific response to PMIC IRQ.
 */
void platform_irqResponse(void);

/**
 * @brief Get platform-specific communication handle.
 *
 * @return Address of communication handle casted as pointer to void.
 */
void *platform_getCommHandle(void);

/**
 * @brief Set the signal level of the platform's GPIO pin that is connected to
 * PMIC's nERR pin.
 *
 * @param level [IN] Signal level to set (true for high, false for low). See
 * @ref Platform_GpioLevel for convenient enumerations.
 */
void platform_esmPinWrite(bool level);

/**
 * @brief Platform-specific API to write one or mulitple bytes to the PMIC.
 *
 * @note This API only supports transmitting at most 2 bytes.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param page [IN] Page that `regAddr` resides in.
 *
 * @param regAddr [IN] Target PMIC register address.
 *
 * @param buffer [IN] Data to write to PMIC.
 *
 * @param bufLen [IN] Number of bytes to transmit. That is to say, Length of
 * `txBuf`.
 *
 * @return Success code if `bufLen` bytes have been written to PMIC, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t platform_txByte(
    const struct Pmic_Handle_s *handle, uint8_t page, uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen);

/**
 * @brief Platform-specific API to read one or multiple bytes from PMIC.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param page [IN] Page that `regAddr` resides in.
 *
 * @param regAddr [IN] Target PMIC register address.
 *
 * @param buffer [OUT] For I2C, data obtained from PMIC. For SPI, data to be
 * written then overwritten.
 *
 * @param bufLen [IN] Number of bytes to read from PMIC. That is to say, length
 * of `rxBuf`.
 *
 * @return Success code if `bufLen` bytes have been obtained from the PMIC,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t platform_rxByte(
    const struct Pmic_Handle_s *handle, uint8_t page, uint8_t regAddr, uint8_t *buffer, uint8_t bufLen);

/**
 * @brief Start asynchronous read transfer.
 *
 * @details This API configures and triggers the DMA engine to execute a read
 * transfer. A single transfer can involve the transfer of more than one byte,
 * all handled asychronously by the DMA while the CPU can execute other
 * instructions or routines.
 *
 * @attention This API only supports SPI operation.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param page [IN] Page that `regAddr` resides in.
 *
 * @param regAddr [IN] Target PMIC register address.
 *
 * @param buffer [IN] For I2C, received data is stored at this location. For
 * SPI, this buffer is written first and then later overwritten via DMA.
 *
 * @param bufLen [IN] For I2C, number of bytes to be read from target address.
 * For SPI, number of bytes to transfer (write and read).
 *
 * @return PMIC_ST_SUCCESS if asynchronous transfer has been initiated, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t platform_asyncRxStart(
    const struct Pmic_Handle_s *handle, uint8_t page, uint8_t regAddr, uint8_t *buffer, uint8_t bufLen);

/**
 * @brief Start asynchronous write transfer.
 *
 * @details This API configures and triggers the DMA engine to execute a write
 * transfer. A single transfer can involve the transfer of more than one byte,
 * all handled asychronously by the DMA while the CPU can execute other
 * instructions or routines.
 *
 * @attention This API only supports SPI operation.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param page [IN] Page that `regAddr` resides in.
 *
 * @param regAddr [IN] Target PMIC register address.
 *
 * @param buffer [IN] Data to be written.
 *
 * @param bufLen [IN] Number of bytes to be transmitted to the target address.
 *
 * @return PMIC_ST_SUCCESS if asynchronous transfer has been initiated, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t platform_asyncTxStart(
    const struct Pmic_Handle_s *handle, uint8_t page, uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen);

/**
 * @brief Await read transfer to be complete. An optional callback will be
 * invoked towards the end of the API routine.
 *
 * @details This API puts the CPU to sleep and waits for the transfer initiated
 * by platform_asyncRxStart() to complete. The CPU will wake once the transfer is
 * complete.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if the asynchronous transfer has been awaited, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t platform_asyncRxAwait(const struct Pmic_Handle_s *handle);

/**
 * @brief Await write transfer to be complete.
 *
 * @details This API puts the CPU to sleep and waits for the transfer initiated
 * by platform_asyncTxStart() to complete. The CPU will wake once the transfer is
 * complete.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if the asynchronous transfer has been awaited, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t platform_asyncTxAwait(const struct Pmic_Handle_s *handle);

/**
 * @brief Initiates platform monitoring.
 *
 * @details This API starts the system tick so that CPU usage can be monitored.
 */
void platform_startSysMonitor(void);

/**
 * @brief Stops platform monitoring.
 *
 * @details This API stops the system tick.
 */
void platform_stopSysMonitor(void);

/**
 * @brief Get the duration (in seconds) in which the system has been monitored.
 *
 * @attention The application must initialize the system tick via
 * platform_startSysMonitor() API.
 *
 * @return Number of seconds in which the system has been monitored.
 */
uint32_t platform_getSysMonitorTime(void);

/**
 * @brief Sets the system monitor time value.
 *
 * @param val [IN] Desired value to set the system monitor time to be (seconds).
 */
void platform_setSysMonitorTime(uint32_t val);

/**
 * @brief Gets the integer part of CPU usage.
 *
 * @attention The application must initialize the system tick via
 * platform_startSysMonitor() API.
 *
 * @return Integer part of CPU usage.
 */
uint32_t platform_getCpuUsageInteger(void);

/**
 * @brief Gets the CPU usage from the system monitor in raw form.
 *
 * @return Raw CPU usage, where the most significant 16 bits is the integer part
 * and least significant 16 bits is the decimal part.
 */
uint32_t platform_getCpuUsageRaw(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PLATFORM_H */
