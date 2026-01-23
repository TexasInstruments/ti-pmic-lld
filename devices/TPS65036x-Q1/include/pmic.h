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
/**
 * @defgroup DRV_PMIC_MODULE PMIC Driver
 *
 * @file pmic.h
 *
 * @brief Top level include file for PMIC LLD.
 */
#ifndef PMIC_H
#define PMIC_H

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pmic_common.h"
#include "pmic_core.h"
#include "pmic_esm.h"
#include "pmic_gpio.h"
#include "pmic_io.h"
#include "pmic_irq.h"
#include "pmic_power.h"
#include "pmic_wdg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_HandleCfgValidParams
 * @name PMIC Handle Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_HandleCfg_t`.
 * Set the `validParams` member of `Pmic_HandleCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_I2C_ADDR0_VALID              (1UL << 0U)
#define PMIC_COMM_HANDLE_0_VALID          (1UL << 1U)
#define PMIC_IO_READ_VALID                (1UL << 2U)
#define PMIC_IO_WRITE_VALID               (1UL << 3U)
#define PMIC_CRITICAL_SECTION_START_VALID (1UL << 4U)
#define PMIC_CRITICAL_SECTION_STOP_VALID  (1UL << 5U)
#define PMIC_IRQ_RESPONSE_CALLBACK_VALID  (1UL << 6U)
#define PMIC_RETRY_CNT_VALID              (1UL << 7U)
#define PMIC_RETRY_INTERVAL_MS_VALID      (1UL << 8U)
#define PMIC_TIMER_WAIT_MS_VALID          (1UL << 9U)
/** @} */

/**
 * @anchor Pmic_HandleCfgAllValid
 * @name PMIC Handle Configuration All Valid Parameters
 *
 * @brief Convenience macro for setting all valid parameters at once.
 *
 * @{
 */
#define PMIC_ALL_VALID (\
    PMIC_I2C_ADDR0_VALID |\
    PMIC_COMM_HANDLE_0_VALID |\
    PMIC_IO_READ_VALID |\
    PMIC_IO_WRITE_VALID |\
    PMIC_CRITICAL_SECTION_START_VALID |\
    PMIC_CRITICAL_SECTION_STOP_VALID |\
    PMIC_IRQ_RESPONSE_CALLBACK_VALID |\
    PMIC_RETRY_CNT_VALID |\
    PMIC_RETRY_INTERVAL_MS_VALID |\
    PMIC_TIMER_WAIT_MS_VALID)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_HandleCfg
 * @name PMIC LLD Configuration
 *
 * @brief Configuration struct holding end-user settings/parameters relating to
 * the PMIC handle.
 *
 * @attention The `validParams` field must be set to indicate which parameters are valid.
 * For required parameters (i2cAddr0, commHandle0, ioRead, ioWrite, criticalSectionStart,
 * criticalSectionStop), the corresponding valid bit must be set. The `irqResponseCallback`
 * parameter is optional and only processed if PMIC_CFG_IRQ_RESPONSE_CALLBACK_VALID is set.
 *
 * @note Once the user sets all struct members, the struct should be passed into
 * Pmic_init() so that the PMIC driver handle can be initialized with the user's
 * desired configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this structure.
 * Specifically, if a bit is set to 1 in this variable, the corresponding structure member
 * is valid and will be considered by the driver API that is using this data structure.
 * Otherwise, if a bit is set to 0, the corresponding structure member is invalid and will
 * not be considered by the driver API that is using this data structure. For possible
 * valid parameter values, refer to @ref Pmic_HandleCfgValidParams.
 *
 * @param i2cAddr0 TPS65036x PMIC I2C address. Valid only when PMIC_CFG_I2C_ADDR0_VALID is set.
 *
 * @param retryCnt Upon communication related errors (bus error or CRC error),
 * PMIC LLD attempts to retry the failed transaction up to `retryCnt` times before
 * returning an error code to the calling application.
 *
 * @param retryIntervalMs LLD waits this configured amount of time (milliseconds)
 * between retry attempts using the `timerWaitMs()` hook.
 *
 * @param commHandle0 Pointer to platform-specific transport layer communication handle.
 * Valid only when PMIC_CFG_COMM_HANDLE_0_VALID is set.
 *
 * @param ioRead Function pointer to platform-specific transport layer read API.
 * Valid only when PMIC_CFG_IO_READ_VALID is set.
 *
 * @param ioWrite Function pointer to platform-specific transport layer write API.
 * Valid only when PMIC_CFG_IO_WRITE_VALID is set.
 *
 * @param criticalSectionStart Function pointer to platform-specific critical section start API.
 * Valid only when PMIC_CFG_CRITICAL_SECTION_START_VALID is set.
 *
 * @param criticalSectionStop Function pointer to platform-specific critical section stop API.
 * Valid only when PMIC_CFG_CRITICAL_SECTION_STOP_VALID is set.
 *
 * @param irqResponseCallback Function pointer to application IRQ response. Valid only when
 * PMIC_CFG_IRQ_RESPONSE_CALLBACK_VALID is set and when servicing the PMIC WDG in Q&A mode.
 *
 * @param timerWaitMs Function pointer to platform-specific timer-based wait API.
 * Upon invocation, the user-implemented hook waits a specified period of time
 * (milliseconds) before returning control to the caller.
 */
typedef struct Pmic_HandleCfg_s
{
    uint32_t validParams;
    uint8_t i2cAddr0;
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
    void (*irqResponseCallback)(void);
    void (*timerWaitMs)(uint32_t ms);
} Pmic_HandleCfg_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
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
 * @param handle [OUT] PMIC interface handle.
 *
 * @param config [IN] PMIC handle configuration struct. End-user will input
 * their settings/parameters in this struct to initialize the PMIC handle.
 *
 * @return Success code if PMIC handle is initialized without issue, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config);

/**
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
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC handle is de-initialized, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_deinit(Pmic_Handle_t *handle);

/**
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
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if the PMIC handle is valid, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_checkHandle(const Pmic_Handle_t *handle);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* PMIC_H */
