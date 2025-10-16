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
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

/**
 * @file platform.h
 *
 * @brief Platform-specific macros/defines and function declarations for testing
 * PMIC LLD.
 */

/**
 * @brief Platform part number.
 */
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
 * @brief Platform-specific include(s).
 */
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"

/**
 * @brief Testing framework include(s).
 */
#include "unity.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 * @brief PMIC-related information.
 */
#define PLATFORM_TARGET_I2C_ADDR    (0x60U)

/**
 * @brief Generic "invalid value" define used in test source codebase.
 */
#define PLATFORM_INVALID_VALUE      (0x00U)

/**
 * @brief Macros/defines relating to testing framework.
 */
#define PLATFORM_RUN_TEST(test)     RUN_TEST(test)
#define PLATFORM_ASSERT(condition)  TEST_ASSERT(condition)

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
 * @brief Platform-specific API to write one or mulitple bytes to the PMIC.
 *
 * @note This API only supports transmitting at most 2 bytes.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param instType [IN] Instance type. For valid values, refer to
 * @ref Pmic_InstType.
 *
 * @param regAddr [IN] Target PMIC register address.
 *
 * @param pTxBuf [IN] Data to write to PMIC.
 *
 * @param bufLen [IN] Number of bytes to transmit. That is to say, Length of
 * `pTxBuf`.
 *
 * @return Success code if `bufLen` bytes have been written to PMIC, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t platform_txByte(const struct Pmic_CoreHandle_s *pmicCorehandle,
                        uint8_t regAddr,
                        uint8_t bufLen,
                        const uint8_t *pTxBuf);

/**
 * @brief Platform-specific API to read one or multiple bytes from PMIC.
 *
 * @param pmicCorehandle [IN] PMIC interface handle.
 *
 * @param instType [IN] Instance type. For valid values, refer to
 * @ref Pmic_InstType.
 *
 * @param regAddr [IN] Target PMIC register address.
 *
 * @param pRxBuf [OUT] Data obtained from PMIC.
 *
 * @param bufLen [IN] Number of bytes to read from PMIC. That is to say, length
 * of `pRxBuf`.
 *
 * @return Success code if `bufLen` bytes have been obtained from the PMIC,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t platform_rxByte(const struct Pmic_CoreHandle_s *pmicCorehandle,
                        uint8_t regAddr,
                        uint8_t bufLen,
                        uint8_t *pRxBuf);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PLATFORM_H__ */
