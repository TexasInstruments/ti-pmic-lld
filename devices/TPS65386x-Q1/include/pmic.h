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
#ifndef PMIC_H
#define PMIC_H

/**
 * @file pmic.h
 * @brief PMIC Driver initialization API/Interface
 */

/**
 * @defgroup DRV_PMIC_MODULE PMIC Driver Entry Point
 * @brief Application entry point for initialization of PMIC driver.
 *
 * This module contains the necessary functions and macros for initialization
 * and de-initialization of the PMIC driver to allow use of the other modules
 * defined in this driver.
 */

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pmic_common.h"
#include "pmic_core.h"
#include "pmic_diag.h"
#include "pmic_esm.h"
#include "pmic_fsm.h"
#include "pmic_gpio.h"
#include "pmic_io.h"
#include "pmic_irq.h"
#include "pmic_power.h"
#include "pmic_timer.h"
#include "pmic_wdg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 * @anchor Pmic_EnableDisable
 * @name PMIC Enable/Disable features control
 *
 * @{
 */
#define PMIC_ENABLE     ((bool)true)
#define PMIC_DISABLE    ((bool)false)
/** @} */

/**
 * @anchor Pmic_CommMode
 * @name PMIC Communication Mode
 *
 * @{
 */
#define PMIC_INTF_I2C_SINGLE (0U)
#define PMIC_INTF_I2C_DUAL   (1U)
#define PMIC_INTF_SPI        (2U)
#define PMIC_INTF_MIN        (PMIC_INTF_I2C_SINGLE)
#define PMIC_INTF_MAX        (PMIC_INTF_SPI)
/** @} */

/**
 * @anchor Pmic_ValidParamCfg
 * @name  PMIC Config Structure Param Bits
 * @brief The `validParams` values to be used when checking configuration of
 * the `Pmic_HandleCfg_t` type.
 *
 * @{
 */
#define PMIC_COMM_MODE_VALID              (1U << 0U)
#define PMIC_I2C_ADDR0_VALID              (1U << 1U)
#define PMIC_I2C_ADDR1_VALID              (1U << 2U)
#define PMIC_I2C_ADDR2_VALID              (1U << 3U)
#define PMIC_COMM_HANDLE_0_VALID          (1U << 5U)
#define PMIC_IO_READ_VALID                (1U << 6U)
#define PMIC_IO_WRITE_VALID               (1U << 7U)
#define PMIC_CRITICAL_SECTION_START_VALID (1U << 8U)
#define PMIC_CRITICAL_SECTION_STOP_VALID  (1U << 9U)
#define PMIC_RETRY_CNT_VALID              (1U << 10U)
#define PMIC_RETRY_INTERVAL_MS_VALID      (1U << 11U)
#define PMIC_TIMER_WAIT_MS_VALID          (1U << 12U)
/** @} */

/**
 * @anchor Pmic_ValidParamCfgShift
 * @name PMIC Config Structure Param Bit Shift Values
 *
 * Application can use below shifted values to set the validParam struct member
 * defined in Pmic_HandleCfg_t structure
 *
 * @{
 */
/** @brief Helper macro to set all `validParams` necessary for configuring I2C
 * based driver. */
#define PMIC_ALL_I2C_VALID        (\
    PMIC_COMM_MODE_VALID              |\
    PMIC_I2C_ADDR0_VALID              |\
    PMIC_I2C_ADDR1_VALID              |\
    PMIC_I2C_ADDR2_VALID              |\
    PMIC_IO_READ_VALID                |\
    PMIC_IO_WRITE_VALID               |\
    PMIC_COMM_HANDLE_0_VALID          |\
    PMIC_CRITICAL_SECTION_START_VALID |\
    PMIC_CRITICAL_SECTION_STOP_VALID  |\
    PMIC_RETRY_CNT_VALID              |\
    PMIC_RETRY_INTERVAL_MS_VALID      |\
    PMIC_TIMER_WAIT_MS_VALID)
/** @brief Helper macro to set all `validParams` necessary for configuring SPI
 * based driver. */
#define PMIC_ALL_SPI_VALID        (\
    PMIC_COMM_MODE_VALID              |\
    PMIC_IO_READ_VALID                |\
    PMIC_IO_WRITE_VALID               |\
    PMIC_COMM_HANDLE_0_VALID          |\
    PMIC_CRITICAL_SECTION_START_VALID |\
    PMIC_CRITICAL_SECTION_STOP_VALID  |\
    PMIC_RETRY_CNT_VALID              |\
    PMIC_RETRY_INTERVAL_MS_VALID      |\
    PMIC_TIMER_WAIT_MS_VALID)
/** @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/**
 * @brief PMIC configuration structure.
 *
 * Contains various parameters which are needed to prepare PMIC driver handle
 * using Valid params like, PMIC interface mode, I2C addresses, various
 * application defined API function pointers for LLD and Critical sections.
 *
 * Application has to set the corresponding bit in validParams structure member
 * to update the driver with Pmic_HandleCfg_t structure fields.
 *
 * For Example, If the Application needs to configure the PMIC driver
 * `commMode` member of the structure, then application has to set
 * `PMIC_CFG_COMM_MODE_VALID` bit of `validParams` struct and then call
 * `Pmic_init()`.
 *
 * @param validParams Controls which parameters below shall be considered by
 * `Pmic_init()`, decided by the combination of @ref Pmic_ValidParamCfgShift.
 *
 * @param commMode Communications interface mode: Single I2C, Dual I2C or SPI.
 * For valid values, see @ref Pmic_CommMode. Valid only when
 * `PMIC_CFG_COMM_MODE_VALID` bit of `validParams` is set.
 *
 * @param i2cAddr0 Main Interface Slave Address for I2C. Valid only when
 * `PMIC_CFG_I2CADDR0_VALID` bit of `validParams` is set. Only necessary for
 * I2C interfaces.
 *
 * @param i2cAddr1 WDOG QA Interface Slave Address for I2C. Valid only when
 * `PMIC_CFG_I2CADDR1_VALID` bit of `validParams` is set. Only necessary for
 * I2C interfaces.
 *
 * @param i2cAddr2 NVM Slave Address for I2C. This provides only read
 * access to CRC status of Page-1 Application shall use this slave address to
 * read only CRC status. Application shall not do any write operations using
 * this slave address. Valid only when `PMIC_CFG_I2CADDR2_VALID` bit of
 * `validParams` is set. Only necessary for I2C interfaces.
 *
 * @param retryCnt Upon communication related errors (bus error or CRC error),
 * PMIC LLD attempts to retry the failed transaction up to `retryCnt` times before
 * returning an error code to the calling application.
 *
 * @param retryIntervalMs LLD waits this configured amount of time (milliseconds)
 * between retry attempts using the `timerWaitMs()` hook.
 *
 * @param ioRead Pointer to I2C/SPI Comm LLD Read Function. Valid
 * only when `PMIC_CFG_COMM_IO_RD_VALID` bit of `validParams` is set.
 *
 * @param ioWrite Pointer to I2C/SPI Comm LLD Write Function. Valid
 * only when `PMIC_CFG_COMM_IO_WR_VALID` bit of `validParams` is set.
 *
 * @param commHandle0 Pointer to Handle for I2C1/SPI Main Interface. Valid only
 * when `PMIC_CFG_COMM_HANDLE_VALID` bit of `validParams` is set.
 *
 * @param qaCommHandle Pointer to Handle for I2C2-QA Interface. Valid only
 * when `PMIC_CFG_QACOMM_HANDLE_VALID` bit of `validParams` is set.
 *
 * @param criticalSectionStart Pointer to Pmic Critical-Section Start Function.
 * Valid only when `PMIC_CFG_CRITSEC_START_VALID` bit of `validParams` is set.
 *
 * @param criticalSectionStop Pointer to Pmic Critical-Section Stop Function.
 * Valid only when `PMIC_CFG_CRITSECSTOP_VALID` bit of `validParams` is set.
 *
 * @param timerWaitMs Function pointer to platform-specific timer-based wait API.
 * Upon invocation, the user-implemented hook waits a specified period of time
 * (milliseconds) before returning control to the caller.
 */
typedef struct Pmic_HandleCfg_s {
    uint32_t validParams;
    uint8_t commMode;
    uint8_t i2cAddr0;
    uint8_t i2cAddr1;
    uint8_t i2cAddr2;
    uint32_t retryCnt;
    uint32_t retryIntervalMs;
    void *commHandle0;
    int32_t (*ioRead)(const struct Pmic_Handle_s *handle,
                      uint8_t page,
                      uint8_t regAddr,
                      uint8_t *buffer,
                      uint8_t bufLen);
    int32_t (*ioWrite)(const struct Pmic_Handle_s *handle,
                       uint8_t page,
                       uint8_t regAddr,
                       const uint8_t *buffer,
                       uint8_t bufLen);
    void (*criticalSectionStart)(uint8_t resource);
    void (*criticalSectionStop)(uint8_t resource);
    void (*timerWaitMs)(uint32_t ms);
} Pmic_HandleCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/**
 * @ingroup DRV_PMIC_MODULE
 * @brief Initialize a PMIC handle instance for PMIC LLD. Reads the PMIC device
 * for information and stores obtained data in the handle instance.
 *
 * Design: PMICDRV-568
 * Architecture: PMICDRV-500, PMICDRV-501, PMICDRV-502, PMICDRV-504, PMICDRV-506,
 *               PMICDRV-508, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-524,
 *               PMICDRV-525, PMICDRV-527, PMICDRV-528, PMICDRV-545, PMICDRV-547,
 *               PMICDRV-549, PMICDRV-551
 *
 * @note This function does not automatically log diagnostic information upon
 * encountering errors or warnings (if any). The caller must invoke `Pmic_logStatus()`
 * to log the status if such information is desired.
 *
 * @param handle  [OUT] PMIC Interface Handle
 * @param coreCfg [IN]  PMIC Configuration Data
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * valid values @ref Pmic_ErrorCodes.
 */
int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *coreCfg);

/**
 * @ingroup DRV_PMIC_MODULE
 * @brief De-initialize a PMIC handle instance.
 *
 * Design: PMICDRV-569
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-551
 *
 * @note This function does not automatically log diagnostic information upon
 * encountering errors or warnings (if any). The caller must invoke `Pmic_logStatus()`
 * to log the status if such information is desired.
 *
 * @param  handle  [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * valid values @ref Pmic_ErrorCodes.
 */
int32_t Pmic_deinit(Pmic_Handle_t *handle);

/**
 * @ingroup DRV_PMIC_MODULE
 * @brief Validate a PMIC handle instance for proper initialization and
 * construction. Utilized by all public LLD APIs that accept a handle as input
 * parameter to help prevent corrupt handle usage. Can be used in the application
 * layer to check the handle independently.
 *
 * Design: PMICDRV-570
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-520, PMICDRV-521,
 *               PMICDRV-522, PMICDRV-526, PMICDRV-545, PMICDRV-551
 *
 * @note This function does not automatically log diagnostic information upon
 * encountering errors or warnings (if any). The caller must invoke `Pmic_logStatus()`
 * to log the status if such information is desired.
 *
 * @param  handle  [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * valid values @ref Pmic_ErrorCodes.
 */
int32_t Pmic_checkHandle(const Pmic_Handle_t *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_H */
