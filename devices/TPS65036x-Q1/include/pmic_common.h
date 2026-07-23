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
 * @file pmic_common.h
 *
 * @brief This file contains declarations/definitions of common macros/defines,
 * data structures, and APIs used throughout PMIC LLD.
 */
#ifndef PMIC_COMMON_H
#define PMIC_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @brief Used to indicate status type for the defines listed under Pmic_ErrorCodes.
 * Shall be located at the most significant 16 bits of a status code.
 */
#define PMIC_ST_TYPE_SUCCESS (0U)
#define PMIC_ST_TYPE_ERROR   (1U)
#define PMIC_ST_TYPE_WARNING (2U)
#define PMIC_ST_TYPE_MIN     (PMIC_ST_TYPE_SUCCESS)
#define PMIC_ST_TYPE_MAX     (PMIC_ST_TYPE_WARNING)
#define PMIC_ST_TYPE_SHIFT   (16U)
#define PMIC_ST_TYPE_MASK    (0xFFFFU << PMIC_ST_TYPE_SHIFT)

/**
 * @brief Used to indicate status ID for the defines listed under Pmic_ErrorCodes.
 * Shall be located at the least significant 16 bits of a status code.
 */
#define PMIC_ST_ID_SUCCESS          (0U)
#define PMIC_ST_ID_SUCCESS_MIN      (PMIC_ST_ID_SUCCESS)
#define PMIC_ST_ID_SUCCESS_MAX      (PMIC_ST_ID_SUCCESS)
#define PMIC_ST_ID_I2C_COMM_FAIL    (0U)
#define PMIC_ST_ID_INV_PARAM        (1U)
#define PMIC_ST_ID_NULL_PARAM       (2U)
#define PMIC_ST_ID_DATA_IO_CRC      (3U)
#define PMIC_ST_ID_NULL_FPTR        (4U)
#define PMIC_ST_ID_REG_LOCKED       (5U)
#define PMIC_ST_ID_INV_HANDLE       (6U)
#define PMIC_ST_ID_FAIL             (7U)
#define PMIC_ST_ID_NOT_SUPPORTED    (8U)
#define PMIC_ST_ID_INV_STATUS_TYPE  (9U)
#define PMIC_ST_ID_INV_STATUS_ID    (10U)
#define PMIC_ST_ID_CONFIG_REG_CRC   (11U)
#define PMIC_ST_ID_ERROR_MIN        (PMIC_ST_ID_I2C_COMM_FAIL)
#define PMIC_ST_ID_ERROR_MAX        (PMIC_ST_ID_CONFIG_REG_CRC)
#define PMIC_ST_ID_NO_IRQ_REMAINING (0U)
#define PMIC_ST_ID_NON_MASKABLE_INT (1U)
#define PMIC_ST_ID_WARNING_MIN      (PMIC_ST_ID_NO_IRQ_REMAINING)
#define PMIC_ST_ID_WARNING_MAX      (PMIC_ST_ID_NON_MASKABLE_INT)
#define PMIC_ST_ID_SHIFT            (0U)
#define PMIC_ST_ID_MASK             (0xFFFFU << PMIC_ST_ID_SHIFT)

/**
 * @brief Macro to generate a 32-bit PMIC status code, where the most significant
 * 16 bits indicate the status type and the least significant 16 bits indicate the
 * status ID.
 */
#define PMIC_STATUS(type, id) ((int32_t)(((uint32_t)(type) << PMIC_ST_TYPE_SHIFT) | ((uint32_t)(id) << PMIC_ST_ID_SHIFT)))

/**
 * @anchor Pmic_ErrorCodes
 * @name PMIC Error Codes
 *
 * @brief Error codes returned by PMIC APIs.
 *
 * @note Application code should check all `Pmic_*` functions which return a
 * status code to verify that `PMIC_ST_SUCCESS` was returned, all other status
 * codes indicate that the requested operation did not succeed.
 *
 * **Common "User Error" Status Codes**
 *
 * The following status codes indicate an error in the expected input to an API
 * call and typically indicate a change is required in application code:
 *
 * - **PMIC_ST_ERR_INV_HANDLE**: Indicates that the `Pmic_Handle_t` passed
 *   to the API call is not valid. Ensure that the `Pmic_HandleCfg_t` has been
 *   properly configured and that `Pmic_init()` has been called.
 *
 * - **PMIC_ST_ERR_NULL_PARAM**: Indicates that a pointer type parameter needed
 *   by the API call was NULL. This should not happen under normal
 *   circumstances and likely indicates an unexpected error in application
 *   code. If the application is intentionally passing a NULL parameter, ensure
 *   that the relevant `validParam` bit is not set for that parameter.
 *
 * - **PMIC_ST_ERR_NULL_FPTR**: Like `PMIC_ST_ERR_NULL_PARAM`, but for function
 *   pointers specifically. This will generally only occur when performing
 *   `Pmic_init()` if the critical section or communications API function
 *   pointers are not set up correctly or not provided.
 *
 * - **PMIC_ST_ERR_INV_PARAM**: Indicates that one of the parameters necessary
 *   for an API call had an invalid value, refer to the documentation for the
 *   relevant function to find the valid values for each parameter.
 *
 * **Common "Communications Error" Status Codes**
 *
 * The following status codes indicate an error in the communication layer
 * between the MCU and the PMIC, and may be addressed by a retry, assuming the
 * underlying communications layer is functional.
 *
 * - **PMIC_ST_ERR_I2C_COMM_FAIL**: Indicates I2C comms. failure. Retry a limited
 *   number of times in case of spurious failure.
 *
 * - **PMIC_ST_ERR_DATA_IO_CRC**: Indicates that the PMIC rejected the I/O
 *   request due to a CRC failure. This likely indicates an error within the
 *   PMIC driver, a misconfiguration of PMIC CRC parameters, or a spurious
 *   failure of the communications layer. Retry a limited number of times in
 *   case of spurious failure.
 *
 * **Device-Specific Status Codes**
 *
 * The following status codes are specific to the TPS65036x-Q1 device family:
 *
 * - **PMIC_ST_ERR_REG_LOCKED**: Indicates that an attempt was made to write to
 *   a register that is currently locked. Unlock the register before attempting
 *   to write. Refer to the TPS65036x-Q1 datasheet for register lock/unlock
 *   procedures.
 *
 * - **PMIC_ST_WARN_NON_MASKABLE_INT**: Warning that a non-maskable interrupt
 *   has occurred. This interrupt cannot be disabled and requires immediate
 *   attention.
 *
 * - **PMIC_ST_ERR_CONFIG_REG_CRC**: Indicates that the PMIC-computed configuration
 *   register CRC does not match the expected CRC written by software. This indicates
 *   a configuration register has been corrupted. Refer to `Pmic_configCrcCalculate()`
 *   for more information.
 *
 * **Other Status Codes**
 *
 * Other status codes are for more specific errors which may occur in one of
 * the given submodules of the PMIC driver. The user is referred to that module
 * for more detail.
 *
 * @{
 */
#define PMIC_ST_SUCCESS               PMIC_STATUS(PMIC_ST_TYPE_SUCCESS, PMIC_ST_ID_SUCCESS)
#define PMIC_ST_ERR_I2C_COMM_FAIL     PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_I2C_COMM_FAIL)
#define PMIC_ST_ERR_INV_PARAM         PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_PARAM)
#define PMIC_ST_ERR_NULL_PARAM        PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_NULL_PARAM)
#define PMIC_ST_ERR_DATA_IO_CRC       PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_DATA_IO_CRC)
#define PMIC_ST_ERR_NULL_FPTR         PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_NULL_FPTR)
#define PMIC_ST_ERR_REG_LOCKED        PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_REG_LOCKED)
#define PMIC_ST_ERR_INV_HANDLE        PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_HANDLE)
#define PMIC_ST_ERR_FAIL              PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_FAIL)
#define PMIC_ST_ERR_NOT_SUPPORTED     PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_NOT_SUPPORTED)
#define PMIC_ST_ERR_INV_STATUS_TYPE   PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_STATUS_TYPE)
#define PMIC_ST_ERR_INV_STATUS_ID     PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_STATUS_ID)
#define PMIC_ST_ERR_CONFIG_REG_CRC    PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_CONFIG_REG_CRC)
#define PMIC_ST_WARN_NO_IRQ_REMAINING PMIC_STATUS(PMIC_ST_TYPE_WARNING, PMIC_ST_ID_NO_IRQ_REMAINING)
#define PMIC_ST_WARN_NON_MASKABLE_INT PMIC_STATUS(PMIC_ST_TYPE_WARNING, PMIC_ST_ID_NON_MASKABLE_INT)
/** @} */

/**
 * @anchor Pmic_arraySizeMacro
 * @name PMIC Array Size Macro
 *
 * @brief Macro used to find the size of an array.
 *
 * @{
 */
#define COUNT(x)                    ((uint8_t)(sizeof(x) / sizeof(x[0])))
/** @} */

/**
 * @anchor Pmic_commonDefines
 * @name PMIC LLD Common Defines
 *
 * @brief Common defines used throughout PMIC LLD.
 *
 * @{
 */
#define PMIC_CFG_DEACTIVATED        ((uint8_t)0U)
#define PMIC_CFG_ACTIVATED          ((uint8_t)1U)
#define PMIC_DISABLE                ((bool)false)
#define PMIC_ENABLE                 ((bool)true)
/** @} */


/**
 * @anchor Pmic_invalidValue
 * @name PMIC Invalid Value Definition
 *
 * @brief Used by PMIC LLD to indicate an invalid value.
 *
 * @{
 */
#define PMIC_INVALID_VALUE          ((uint8_t)0x00U)
/** @} */

/**
 * @anchor Pmic_DiagnosticValidParams
 * @name PMIC Diagnostic Valid Parameters
 *
 * @brief Valid parameters of the `Pmic_Diagnostic_t` structure. Set the
 * `validParams` member of the structure equal to a combination of the
 * following definitions using the bitwise OR operator.
 *
 * @{
 */
#define PMIC_DIAGNOSTIC_CNT_VALID  (1UL << 0U)
#define PMIC_DIAGNOSTIC_FLAG_VALID (1UL << 1U)
#define PMIC_DIAGNOSTIC_VALID_ALL  (PMIC_DIAGNOSTIC_CNT_VALID | PMIC_DIAGNOSTIC_FLAG_VALID)
/** @} */

/**
 * @anchor Pmic_SharedResources
 * @name PMIC LLD Shared Resources
 *
 * @brief Types of shared resources in PMIC LLD. Passed as input to critical
 * section start/stop hooks in `Pmic_Handle_t` to identify the corresponding
 * mutex/semaphore to take/release.
 *
 * @{
 */
#define PMIC_COMMUNICATION (0U)
#define PMIC_DIAGNOSTIC    (1U)
/** @} */

/**
 * @anchor Pmic_DiagnosticThresholds
 * @name PMIC Diagnostic Thresholds
 *
 * @brief Compile-time thresholds for diagnostic counters.
 *
 * @details These thresholds determine when a diagnostic error/warning/retry
 * counter in `Pmic_Diagnostic_t` is considered to have reached a critical
 * level. When this occurs, the associated counter wraps to zero and the
 * corresponding overflow threshold flag is set to true.
 *
 * @{
 */
#ifndef PMIC_ERR_CNT_OVERFLOW_THR
#define PMIC_ERR_CNT_OVERFLOW_THR (UINT32_MAX)
#endif
#ifndef PMIC_WARN_CNT_OVERFLOW_THR
#define PMIC_WARN_CNT_OVERFLOW_THR (UINT32_MAX)
#endif
#ifndef PMIC_RETRY_CNT_OVERFLOW_THR
#define PMIC_RETRY_CNT_OVERFLOW_THR (UINT32_MAX)
#endif
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_CoreHandle
 * @name TPS65036x PMIC Handle
 *
 * @brief Handle to TPS65036x PMIC that is used as input to all driver APIs.
 *
 * @details The handle contains information related to the TPS65036x PMIC such
 * as revision (device, NVM, silicon) and whether CRC is enabled. It also has
 * function pointers that serve to abstract platform-specific information, like
 * the transport layer write/read APIs and critical section start/stop.
 *
 * @attention Once the PMIC handle is initialized via the Pmic_init() API,
 * end-user must ensure that the handle is unmodified throughout application
 * runtime.
 *
 * @param drvInitStat Driver initialization status. When the driver is initialized,
 * the value comes out to be decimal value 1347242307, hex value 0x504D4943. When
 * converting to ASCII, the value reads "PMIC".
 *
 * @param i2cAddr0 PMIC device I2C address.
 *
 * @param devRev PMIC device revision identifier.
 *
 * @param nvmCode 0x00 - 0xF0 are reserved for TI manufactured NVM variants. 0xF1 - 0xFF
 * are reserved for special use.
 *
 * @param nvmRev NVM revision of the IC.
 *
 * @param devSiRev PMIC silicon revision identifier. SILICON_REV[7:6] - Reserved.
 * SILICON_REV[5:3] - ALR. SILICON_REV[2:0] - Metal.
 *
 * @param retryCnt Upon communication related errors (bus error or CRC error),
 * PMIC LLD attempts to retry the failed transaction up to `retryCnt` times before
 * returning an error code to the calling application.
 *
 * @param retryIntervalMs LLD waits this configured amount of time (milliseconds)
 * between retry attempts using the `timerWaitMs()` hook.
 *
 * @param isA0 Indication of whether the PMIC device silicon revision is A0.
 *
 * @param crcEnable Indication of whether PMIC CRC is enabled. Used by LLD to determine
 * whether to calculate CRC during communication with PMIC.
 *
 * @param commHandle0 Pointer to platform-specific transport layer communication handle.
 *
 * @param ioRead Function pointer to platform-specific transport layer read API.
 *
 * @param ioWrite Function pointer to platform-specific transport layer write API.
 *
 * @param criticalSectionStart Function pointer to platform-specific critical section start API.
 *
 * @param criticalSectionStop Function pointer to platform-specific critical section stop API.
 *
 * @param irqResponseCallback Function pointer to application IRQ response. Valid only when
 * servicing the PMIC WDG in Q&A mode.
 *
 * @param timerWaitMs Function pointer to platform-specific timer-based wait API.
 * Upon invocation, the user-implemented hook waits a specified period of time
 * (milliseconds) before returning control to the caller.
 */
typedef struct Pmic_Handle_s
{
    uint32_t drvInitStat;
    uint8_t i2cAddr0;
    uint8_t devRev;
    uint8_t nvmCode;
    uint8_t nvmRev;
    uint8_t devSiRev;
    uint32_t retryCnt;
    uint32_t retryIntervalMs;
    bool isA0;
    bool crcEnable;
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
} Pmic_Handle_t;

/**
 * @anchor Pmic_Diagnostic
 * @name PMIC Diagnostic
 *
 * @brief Structure to hold diagnostic information for a PMIC LLD error/warning.
 *
 * @details This structure is used to report diagnostic statistics for PMIC LLD
 * errors/warnings that occur during operation. It includes the error/warning
 * code, the count of how many times the error has occurred, and a status flag
 * to indicate whether the counter reached a specified overflow threshold (refer
 * to @ref Pmic_DiagnosticThresholds for more information on diagnostic thresholds).
 *
 * @param validParams Bitmask indicating which parameters are valid. For valid
 * values, refer to @ref Pmic_DiagnosticValidParams.
 *
 * @param code Error code indicating the type of error. For valid values, refer
 * to @ref Pmic_ErrorCodes.
 *
 * @param cnt Count of occurrences for the specific error or warning.
 *
 * @param flag Indication of whether the counter has reached its corresponding
 * compile-time threshold. For threshold values of diagnostic counters, refer to
 * @ref Pmic_DiagnosticThresholds.
 */
typedef struct Pmic_Diagnostic_s {
    uint32_t validParams;
    int32_t code;

    uint32_t cnt;
    bool flag;
} Pmic_Diagnostic_t;

/*==========================================================================  */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Checks whether a parameter is valid.
 *
 * Design: PMICDRV-571
 * Architecture: PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-519, PMICDRV-521,
 *               PMICDRV-522
 *
 * @param validParamVal [IN] Set of valid parameters.
 *
 * @param bitMask [IN] validParam to check for.
 *
 * @return True if validParams is set, false if validParam is not set.
 */
static bool Pmic_validParamCheck(uint32_t validParams, uint32_t bitMask)
{
    return ((validParams & bitMask) != 0U);
}

/**
 * @brief Checks whether a parameter is valid and whether the status code is equal
 * to LLD success code.
 *
 * Design: PMICDRV-572
 * Architecture: PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-519, PMICDRV-521,
 *               PMICDRV-522
 *
 * @param validParams [IN] Valid parameter value.
 *
 * @param bitMask [IN] Valid parameter bit mask. used to check whether the valid parameter is set in 'validParams'.
 *
 * @param status [IN] API checks whether this parameter is equal to LLD success code.
 *
 * @return True if valid parameter is set and status is equal to LLD success code, false otherwise.
 */
static inline bool Pmic_validParamStatusCheck(uint32_t validParams, uint32_t bitMask, int32_t status)
{
    return ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(validParams, bitMask));
}

/**
 * @brief Start a critical section when usage of a shared resource such as an I2C or
 * SPI bus is required.
 *
 * Design: PMICDRV-573
 * Architecture: PMICDRV-502, PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-507,
 *               PMICDRV-509, PMICDRV-516, PMICDRV-517, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-549
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param resource Resource identifier for the critical section.
 */
static inline void Pmic_criticalSectionStart(const Pmic_Handle_t *handle, uint8_t resource)
{
    if ((handle != NULL) && (handle->criticalSectionStart != NULL))
    {
        handle->criticalSectionStart(resource);
    }
}

/**
 * @brief Stop a critical section after the usage of a shared resource such as an
 * I2C or SPI bus is complete.
 *
 * Design: PMICDRV-574
 * Architecture: PMICDRV-502, PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-507,
 *               PMICDRV-509, PMICDRV-516, PMICDRV-517, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-549
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param resource Resource identifier for the critical section.
 */
static inline void Pmic_criticalSectionStop(const Pmic_Handle_t *handle, uint8_t resource)
{
    if ((handle != NULL) && (handle->criticalSectionStop != NULL))
    {
        handle->criticalSectionStop(resource);
    }
}

/**
 * @brief Indicate via callback function that an INT event has been detected on the
 * PMIC.
 *
 * Design: PMICDRV-732
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-517, PMICDRV-521
 *               PMICDRV-522, PMICDRV-537
 *
 * @param handle [IN] PMIC interface handle.
 */
static inline void Pmic_irqResponseCallback(const Pmic_Handle_t *handle)
{
    if ((handle != NULL) && (handle->irqResponseCallback != NULL))
    {
        handle->irqResponseCallback();
    }
}

/**
 * @brief Invokes user-implemented hook to wait a specified amount of time in
 * milliseconds.
 *
 * Design: PMICDRV-809
 * Architecture: PMICDRV-501, PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-508,
 *               PMICDRV-516, PMICDRV-517, PMICDRV-521, PMICDRV-542, PMICDRV-549,
 *               PMICDRV-551
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param ms [IN] Amount of time to wait in milliseconds.
 */
static inline void Pmic_timerWaitMs(const Pmic_Handle_t *handle, uint32_t ms)
{
    if ((handle != NULL) && (handle->timerWaitMs != NULL))
    {
        handle->timerWaitMs(ms);
    }
}

/**
 * @brief Set a bit field of a register to a desired value.
 *
 * Design: PMICDRV-575
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-521, PMICDRV-522
 *               PMICDRV-549
 *
 * @param regData [OUT] Pointer to variable holding register value.
 *
 * @param shift [IN] Target bit field position.
 *
 * @param mask [IN] Target bit field mask.
 *
 * @param value [IN] Desired bit field value.
 */
static inline void Pmic_setBitField(
    uint8_t *regData, uint8_t shift, uint8_t mask, uint8_t value)
{
    *regData = (((*regData) & (~mask)) | ((value << shift) & mask));
}

/**
 * @brief Sets the bit field of an 8-bit unsigned integer to the desired boolean
 * value.
 *
 * Design: PMICDRV-577
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-521, PMICDRV-522
 *               PMICDRV-549
 *
 * @param regData [OUT] Pointer to variable holding register value.
 *
 * @param shift [IN] Target bit field position.
 *
 * @param mask [IN] Target bit field mask.
 *
 * @param value [IN] Desired bit field value. When parameter set to true,
 * bit field value will be set to 1. Otherwise, bit field value will be set to 0.
 */
static inline void Pmic_setBitField_b(
    uint8_t *regData, uint8_t shift, uint8_t mask, bool value)
{
    const uint8_t fieldVal = value ? 1U : 0U;

    *regData = (((*regData) & (~mask)) | ((fieldVal << shift) & mask));
}

/**
 * @brief Get desired bit field of an 8-bit unsigned integer.
 *
 * Design: PMICDRV-578
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-521, PMICDRV-522
 *               PMICDRV-549
 *
 * @param regData [IN] Register data/value.
 *
 * @param regFieldShift [IN] Target bit field position.
 *
 * @param regFieldMask [IN] Target bit field mask.
 *
 * @return Desired bit field value.
 */
static inline uint8_t Pmic_getBitField(uint8_t regData, uint8_t regFieldShift, uint8_t regFieldMask)
{
    return ((regData & regFieldMask) >> regFieldShift);
}

/**
 * @brief Gets the desired bit field of an 8-bit unsigned integer, casted as a
 * boolean.
 *
 * Design: PMICDRV-580
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-521, PMICDRV-522
 *               PMICDRV-549
 *
 * @param regData [IN] Register data/value.
 *
 * @param regFieldShift [IN] Target bit field position.
 *
 * @return Desired bit field value cast as a boolean.
 */
static inline bool Pmic_getBitField_b(uint8_t regData, uint8_t regFieldShift)
{
    const uint8_t bitVal = ((regData & (uint8_t)(1U << (regFieldShift & 0x07U))) >> regFieldShift);

    return (bitVal == 1U);
}

/**
 * @brief Used internally by PMIC LLD to log and report a status code for
 * system diagnostic purposes. Made public so that other modules in PMIC LLD
 * can utilize it.
 *
 * Design: PMICDRV-798
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525
 *
 * @details PMIC LLD maintains diagnostic information for error/warning reporting,
 * monitoring, and analysis. The driver invokes this API to update the diagnostic
 * information.
 *
 * @attention This function will not log diagnostic information for the given
 * status if the handle is invalid, critical section hooks are NULL, or if the
 * status code is not recognized (invalid status type or ID).
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param status [IN] Status code to log and report. For possible status codes,
 * refer to @ref Pmic_ErrorCodes.
 *
 * @return Status code that was passed as input to the function.
 */
int32_t Pmic_logStatus(const Pmic_Handle_t *handle, int32_t status);

/**
 * @brief Gets the diagnostic information of an error/warning.
 *
 * Design: PMICDRV-799
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525, PMICDRV-526,
 *               PMICDRV-528
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param diagnostic [OUT] Diagnostic information of the specified error/warning.
 * For more information on error diagnostics, refer to @ref Pmic_Diagnostic_t.
 *
 * @return PMIC_ST_SUCCESS if diagnostic information has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getDiagnostic(const Pmic_Handle_t *handle, Pmic_Diagnostic_t *diagnostic);

/**
 * @brief Gets diagnostic information of multiple errors/warnings.
 *
 * Design: PMICDRV-800
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525, PMICDRV-526,
 *               PMICDRV-528
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param diagnostic [OUT] Each element in this array holds diagnostic
 * information of a specific error/warning. For more information on error
 * diagnostics, refer to @ref Pmic_Diagnostic_t.
 *
 * @param numDiagnostics [IN] Number of errors to get diagnostic information for.
 *
 * @return PMIC_ST_SUCCESS if diagnostic information has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getDiagnostics(const Pmic_Handle_t *handle, Pmic_Diagnostic_t diagnostic[], uint8_t numDiagnostics);

/**
 * @brief Clears the diagnostic information of an error or warning.
 *
 * Design: PMICDRV-801
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525
 *
 * @details The `code` member of `Pmic_Diagnostic_t` is used to identify the error
 * or warning. The `validParams` member is used to identify the information of
 * the error or warning to clear. All other members of the structure are ignored.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param diagnostic [IN] `validParams` and `code` members of this structure
 * shall be used to clear desired diagnostic information. For more information
 * on error diagnostics, refer to @ref Pmic_Diagnostic_t.
 *
 * @return PMIC_ST_SUCCESS if diagnostic information has been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_clrDiagnostic(const Pmic_Handle_t *handle, const Pmic_Diagnostic_t *diagnostic);

/**
 * @brief Clears diagnostic information of multiple errors or warnings.
 *
 * Design: PMICDRV-802
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525
 *
 * @details For each element in the `diagnostic` array, The `code` member of the
 * structure is used to identify the error or warning. The `validParams` member is
 * used to identify the information of the error or warning to clear. All other
 * members of the structure are ignored.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param diagnostic [IN] For each element in this array, `validParams` and
 * `code` members shall be used to clear desired diagnostic information. For
 * more information on error/warning diagnostics, refer to @ref Pmic_Diagnostic_t.
 *
 * @param numDiagnostics [IN] Number of diagnostic information of errors to clear.
 *
 * @return PMIC_ST_SUCCESS if diagnostic information has been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_clrDiagnostics(const Pmic_Handle_t *handle, const Pmic_Diagnostic_t diagnostic[], uint8_t numDiagnostics);

/**
 * @brief Resets PMIC LLD system diagnostics by clearing all error, warning, and
 * retry counters along with associated flags. For clearing diagnostic information
 * per error/warning, refer to `Pmic_clrDiagnostic()` and `Pmic_clrDiagnostics()`.
 *
 * Design: PMICDRV-803
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if diagnostics have been reset, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_clrDiagnosticsAll(const Pmic_Handle_t *handle);

/**
 * @brief Get the global number of times that PMIC LLD has attempted to retry or
 * re-issue operations/transactions.
 *
 * Design: PMICDRV-804
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525, PMICDRV-526,
 *               PMICDRV-528
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param retryCnt [OUT] Current value of the retry counter.
 *
 * @return PMIC_ST_SUCCESS if the retry counter has been retrieved successfully,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getRetryCnt(const Pmic_Handle_t *handle, uint32_t *retryCnt);

/**
 * @brief Get status of whether PMIC LLD retry counter has reached its compile-time
 * overflow threshold.
 *
 * Design: PMICDRV-805
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525, PMICDRV-526,
 *               PMICDRV-528
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param reachedThreshold [OUT] Flag indicating whether the retry counter has reached
 * the overflow threshold.
 *
 * @return PMIC_ST_SUCCESS if the retry overflow threshold flag has been retrieved
 * successfully, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getRetryCntOverflow(const Pmic_Handle_t *handle, bool *reachedThreshold);

/**
 * @brief Clear PMIC LLD retry counter.
 *
 * Design: PMICDRV-806
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525
 *
 * @details The retry counter keeps track of the number of times that PMIC LLD
 * has attempted to retry/re-issue operations/transactions. This API can
 * be used to reset the counter to zero.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if the retry counter has been cleared successfully,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_clrRetryCnt(const Pmic_Handle_t *handle);

/**
 * @brief Used internally by PMIC LLD to increment the retry counter upon
 * re-issuing a failed operation or transaction.
 *
 * Design: PMICDRV-807
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if the retry counter has been incremented successfully,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 *
 */
int32_t Pmic_incrementRetryCnt(const Pmic_Handle_t *handle);

/**
 * @brief Clears the PMIC LLD retry counter overflow threshold flag.
 *
 * Design: PMICDRV-808
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-515, PMICDRV-516,
 *               PMICDRV-520, PMICDRV-521, PMICDRV-522, PMICDRV-525
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if the retry overflow threshold flag has been cleared
 * successfully, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_clrRetryCntOverflow(const Pmic_Handle_t *handle);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* PMIC_COMMON_H */
