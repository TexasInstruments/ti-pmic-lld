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
 * @brief read a single byte or multiple bytes via I2C from a target device.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Target device internal register address.
 *
 * @param bufLen [IN] Length of `rxBuf`. Min value is 1, max value is UINT8_MAX.
 *
 * @param rxBuf [OUT] Data obtained from PMIC.
 *
 * @return Success code if read exchange has occured with no issue, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t app_ioRead(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *rxBuf);

/**
 * @brief write a single byte or multiple bytes via I2C to a target device.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Target device internal register address.
 *
 * @param bufLen [IN] Length of `txBuf`. Permitted range: [1, 2].
 *
 * @param txBuf [IN] Data to be written to PMIC.
 *
 * @return Success code if write exchange has occured with no issue, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t app_ioWrite(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, const uint8_t *txBuf);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __COMMON_TEST_H__ */
