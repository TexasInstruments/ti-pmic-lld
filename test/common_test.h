/**
 * @file common_test.h
 *
 * @brief Header file for commonly used APIs and macros/defines for PMIC LLD
 * testing on the AM263x platform.
 */
#ifndef __COMMON_TEST_H__
#define __COMMON_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */

/* Standard includes */
#include <stdio.h>
#include <string.h>

/* Platform includes */
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* PMIC LLD include */
#include "pmic.h"

/* Unity testing framework include */
#include "unity.h"

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief This API is used by the Unity testing framework to write a character
 * to the console.
 *
 * @param c [IN] Character to transmit to the terminal.
 */
void unityCharPut(uint8_t c);

/**
 * @brief Platform-specific critical section start API, used in RTOS applications.
 */
void app_critSecStart(void);

/**
 * @brief Platform-specific critical section stop API, used in RTOS applications.
 */
void app_critSecStop(void);

/**
 * @brief Platform-specific response to PMIC IRQ(s).
 */
void app_irqResponse(void);

/**
 * @brief Write 24 bits to the PMIC and receive 24 bits from the PMIC.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param txBuf [IN] 24-bit data to be written to PMIC.
 *
 * @param rxBuf [OUT] 24-bit data received from the PMIC.
 *
 * @param len [IN] Number of bytes in @p txBuf and @p rxBuf parameters.
 *
 * @return Success code if SPI transfer is completed, error code otherwise. For
 * valid error/success codes, refer to @ref Pmic_errorCodes.
 */
int32_t app_spiTransfer(const Pmic_CoreHandle_t *pmicHandle, uint32_t txBuf, uint32_t *rxBuf, uint8_t len);

/**
 * @brief Wait a certain period of time specified in milliseconds.
 *
 * @param milliseconds [IN] Duration in which the MCU is blocked.
 */
void app_wait(uint16_t milliseconds);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __COMMON_TEST_H__ */
