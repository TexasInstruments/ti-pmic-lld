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
#ifndef PMIC_TEST_PLATFORM_H
#define PMIC_TEST_PLATFORM_H



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
#if defined(BUILD_MOCK)
/* Mock mode: platform_mock.h included at end after function declarations */
#elif defined(BUILD_HOST)
/* Host mode: No hardware-specific includes needed */
/* Serial communication handled by platform_serial.h */
#endif

/**
 * @brief Testing framework include(s).
 */
#include "unity.h"
#include "test_filter.h"
#include "test_timer.h"

#if !defined(BUILD_MOCK) && !defined(BUILD_HOST)
  #ifdef __cplusplus
  extern "C" {
  #endif
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 * @brief PMIC-related information.
 */
#ifndef PLATFORM_TARGET_I2C_ADDR
#define PLATFORM_TARGET_I2C_ADDR    (0x48U)
#endif

/**
 * @brief Generic "invalid value" define used in test source codebase.
 */
#define PLATFORM_INVALID_VALUE      (0x00U)

/**
 * @brief I2C address configuration for TPS6522x-Q1
 *
 * TPS6522x-Q1 supports dual I2C addresses (main + secondary)
 */
#define PLATFORM_I2C_ADDR_MAIN      (0x48U)
#define PLATFORM_I2C_ADDR_SECONDARY (0x12U)

#define PLATFORM_I2C_PORT_MAIN      (2U)   /* Tiva I2C0: PB2(SCL)/PB3(SDA) */
#define PLATFORM_I2C_PORT_SECONDARY (0U)   /* Tiva I2C1: PA6(SCL)/PA7(SDA) */

/* ========================================================================= */
/*                        Module Name Tracking                               */
/* ========================================================================= */

/**
 * @brief Current module name for test result prefixes
 */
extern const char* g_currentModuleName;

/**
 * @brief Set the current module name for test result prefixes
 * @param moduleName The module name to use as a prefix (e.g., "ESM", "WDG")
 *
 * @note Internal use only - called automatically by testTimer_startModule().
 *       Test code should use testTimer_startModule() instead of calling this directly.
 */
void platform_setModuleName(const char* moduleName);

/**
 * @brief Macros/defines relating to testing framework.
 *
 * PLATFORM_RUN_TEST wraps Unity's RUN_TEST with test filtering and (on hardware)
 * TEST_PROTECT to catch assertion failures. Test filtering allows selective
 * test execution via environment variables without recompilation.
 *
 * Filtering is controlled by:
 * - PMIC_TEST_MODULES: Comma-separated module list (e.g., "irq,power")
 * - PMIC_TEST_FILTER: Wildcard pattern (e.g., "*mask*", "test_pos_*")
 * - PMIC_TEST_GROUPS: "positive", "negative", or "all"
 *
 * Note: platform_mock.h may have already defined PLATFORM_RUN_TEST, so we
 * undefine it first to ensure our filtered version is used.
 */
#ifdef PLATFORM_RUN_TEST
#undef PLATFORM_RUN_TEST
#endif

#if defined(BUILD_MOCK) || defined(BUILD_HOST)
    /* Mock/Host build: Use standard RUN_TEST with filtering */
    #define PLATFORM_RUN_TEST(test) \
        do { \
            if (testFilter_shouldRunTestWithGroup(#test)) { \
                testTimer_startTest(#test); \
                if (g_currentModuleName) printf("[%s] ", g_currentModuleName); \
                RUN_TEST(test); \
                testTimer_endTest(); \
            } \
        } while(0)
#endif

#define PLATFORM_ASSERT(condition)  TEST_ASSERT(condition)

/* ========================================================================= */
/*                           Function Declarations                           */
/* ========================================================================= */

/* Function declarations available in all build modes */

/**
 * @brief Initialize platform and its peripherals for testing LLD.
 */
void platform_init(void);

/**
 * @brief De-initialize platform and its peripherals.
 */
void platform_deinit(void);

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
void platform_critSecStart(uint8_t resource);

/**
 * @brief Stop platform-specific critical section.
 */
void platform_critSecStop(uint8_t resource);

/**
 * @brief Platform-specific response to PMIC IRQ.
 */
void platform_irqResponse(void);

/**
 * @brief Get platform-specific communication handle.
 *
 * @return Address of communication handle casted as pointer to void.
 */
void *platform_getCommHandle0(void);
void *platform_getCommHandle1(void);

/**
 * @brief Platform-specific API to write one or mulitple bytes to the PMIC.
 *
 * @note This API only supports transmitting at most 2 bytes.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param page [IN] Page number for register access.
 *
 * @param regAddr [IN] Target PMIC register address.
 *
 * @param pTxBuf [IN] Data to write to PMIC.
 *
 * @param bufLen [IN] Number of bytes to transmit. That is to say, Length of
 * `pTxBuf`.
 *
 * @return Success code if `txBuf` bytes have been written to PMIC, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t platform_txByte(const Pmic_Handle_t *handle,
                        uint8_t page,
                        uint8_t regAddr,
                        const uint8_t *buffer,
                        uint8_t bufLen);

/**
 * @brief Platform-specific API to read one or multiple bytes from PMIC.
 *
 * @param pmicCorehandle [IN] PMIC interface handle.
 *
 * @param page [IN] Page number for register access.
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
int32_t platform_rxByte(const Pmic_Handle_t *handle,
                        uint8_t page,
                        uint8_t regAddr,
                        uint8_t *buffer,
                        uint8_t bufLen);

/**
 * @brief Unlock PMIC registers for testing.
 *
 * @details This function unlocks PMIC configuration registers to allow
 * register modifications during testing. In hardware mode, this sends the
 * unlock sequence to the PMIC. In mock mode, this is a no-op since the mock
 * doesn't enforce register locking.
 */
void platform_unlockRegisters(void);

#if !defined(BUILD_MOCK)
/* Not provided by platform_mock.c; setupTests/tearDownTests become macros
 * via platform_mock.h; printChar and runTestLoop have no mock implementation. */

#define PLATFORM_REBOOT_INITIAL_WAIT_MS (2U)    /* time for nRSTOUT to assert after reboot trigger */
#define PLATFORM_REBOOT_TIMEOUT_MS      (500U)   /* max poll window for nRSTOUT to deassert */

void platform_softReboot(void);

/**
 * @brief Drive the nERR_MCU signal (TIVA PA2 → PMIC GPIO6) high or low.
 *
 * @details No-op on non-host builds. Call before enabling/starting ESM to
 *          provide a valid level-mode signal and prevent an immediate ESM fault.
 *
 * @param high true = drive PA2 high; false = drive PA2 low
 */
void platform_setEsmPin(bool high);

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
 * @brief Execute test callback with platform-appropriate behavior
 *
 * @details Hardware: Interactive loop (wait before start, re-run capability, wait after)
 *          Mock: Single execution, no waits
 *          Host: Single execution, no waits
 *
 * @param testCallback Function that executes all test suites
 */
void platform_runTestLoop(void (*testCallback)(void));

#endif /* !BUILD_MOCK */

#if !defined(BUILD_MOCK) && !defined(BUILD_HOST)
  #ifdef __cplusplus
  }
  #endif /* __cplusplus */
#endif /* !BUILD_MOCK && !BUILD_HOST */

/**
 * @brief Include mock-specific overrides when BUILD_MOCK is defined
 *
 * platform_mock.h provides macro definitions that override function declarations
 * with no-ops or redirects. This prevents double Unity initialization and other
 * issues that occur when using the mock backend.
 */
#ifdef BUILD_MOCK
    /* Undefine macros that platform_mock.h will redefine */
    #ifdef PLATFORM_TARGET_I2C_ADDR
    #undef PLATFORM_TARGET_I2C_ADDR
    #endif
    #ifdef PLATFORM_RUN_TEST
    #undef PLATFORM_RUN_TEST
    #endif
    #include "platform_mock.h"
#endif

/* Async test helper functions (TPS6522x-specific) */
extern int32_t test_pmic_asyncRxStart(const Pmic_Handle_t *handle, uint8_t page,
                                      uint8_t regAddr, uint8_t *buffer, uint8_t bufLen);
extern int32_t test_pmic_asyncTxStart(const Pmic_Handle_t *handle, uint8_t page,
                                      uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen);
extern int32_t test_pmic_asyncRxAwait(const Pmic_Handle_t *handle);
extern int32_t test_pmic_asyncTxAwait(const Pmic_Handle_t *handle);

#endif /* PMIC_TEST_PLATFORM_H */
