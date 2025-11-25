/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_ErrorCodes
 * @name PMIC LLD Error Codes
 *
 * @brief Error codes returned by PMIC LLD APIs
 *
 * Error codes are defined in pmic_status.h
 *
 * @{
 */
#include "pmic_status.h"
/** @} */

#include "pmic_common.h"

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
#define PMIC_I2C_ADDR0_VALID              (1U << 0U)
#define PMIC_COMM_HANDLE_0_VALID          (1U << 1U)
#define PMIC_IO_READ_VALID                (1U << 2U)
#define PMIC_IO_WRITE_VALID               (1U << 3U)
#define PMIC_CRITICAL_SECTION_START_VALID (1U << 4U)
#define PMIC_CRITICAL_SECTION_STOP_VALID  (1U << 5U)
#define PMIC_IRQ_RESPONSE_CALLBACK_VALID  (1U << 6U)
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
    PMIC_IRQ_RESPONSE_CALLBACK_VALID)
/** @} */

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"
#include "pmic_core.h"
#include "pmic_power.h"
#include "pmic_irq.h"
#include "pmic_wdg.h"
#include "pmic_esm.h"
#include "pmic_gpio.h"
#include "pmic_io.h"

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
 */
typedef struct Pmic_HandleCfg_s
{
    uint32_t validParams;
    uint8_t i2cAddr0;
    void *commHandle0;
    int32_t (*ioRead)(const struct Pmic_CoreHandle_s *pmicHandle,
                      uint8_t regAddr,
                      uint8_t bufLen,
                      uint8_t *rxBuf);
    int32_t (*ioWrite)(const struct Pmic_CoreHandle_s *pmicHandle,
                       uint8_t regAddr,
                       uint8_t bufLen,
                       const uint8_t *txBuf);
    void (*criticalSectionStart)(void);
    void (*criticalSectionStop)(void);
    void (*irqResponseCallback)(void);
} Pmic_HandleCfg_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Initialize a PMIC handle instance for PMIC LLD. Reads the PMIC device
 * for information and stores obtained data in the handle instance.
 *
 * Design: PMICDRV-568
 * Architecture: PMICDRV-527, PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-547
 *               PMICDRV-549, PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-502, PMICDRV-506
 *               PMICDRV-524, PMICDRV-504, PMICDRV-522, PMICDRV-528, PMICDRV-521, PMICDRV-500
 *               PMICDRV-512, PMICDRV-501, PMICDRV-525
 *
 * @param pmicCfg [IN] PMIC handle configuration struct. End-user will input
 * their settings/parameters in this struct to initialize the PMIC handle.
 *
 * @param pmicHandle [OUT] PMIC interface handle.
 *
 * @return Success code if PMIC handle is initialized without issue, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config);

/**
 * @brief De-initialize a PMIC handle instance.
 *
 * Design: PMICDRV-569
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-551, PMICDRV-545
 *               PMICDRV-546, PMICDRV-506, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC handle is de-initialized, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_deinit(Pmic_Handle_t *pmicHandle);

/**
 * @brief Validate a PMIC handle instance for proper initialization and
 * construction. Utilized by all public LLD APIs that accept a handle as input
 * parameter to help prevent corrupt handle usage. Can be used in the application
 * layer to check the handle independently.
 *
 * Design: PMICDRV-570
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522
 *               PMICDRV-534, PMICDRV-521, PMICDRV-520
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if the PMIC handle is valid, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_checkHandle(const Pmic_Handle_t *pmicHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* PMIC_H */
