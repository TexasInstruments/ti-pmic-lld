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
#ifndef PMIC_COMMON_H
#define PMIC_COMMON_H

/**
 * @file pmic_common.h
 * @brief APIs and macros/typedefs commonly used across PMIC LLD.
 */

/**
 * @defgroup DRV_PMIC_COMMON_MODULE PMIC Common Module
 * @brief APIs and macros/typedefs commonly used across PMIC LLD.
 */

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/*                          Macros and Defines                              */
/*==========================================================================*/

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
#define PMIC_ST_ID_SUCCESS             (0U)
#define PMIC_ST_ID_SUCCESS_MIN         (PMIC_ST_ID_SUCCESS)
#define PMIC_ST_ID_SUCCESS_MAX         (PMIC_ST_ID_SUCCESS)
#define PMIC_ST_ID_INV_HANDLE          (0U)
#define PMIC_ST_ID_NULL_PARAM          (1U)
#define PMIC_ST_ID_INV_PARAM           (2U)
#define PMIC_ST_ID_INV_DEVICE          (3U)
#define PMIC_ST_ID_NULL_FPTR           (4U)
#define PMIC_ST_ID_INV_SUBSYSTEM       (5U)
#define PMIC_ST_ID_INSUFFICIENT_CFG    (6U)
#define PMIC_ST_ID_I2C_COMM_FAIL       (7U)
#define PMIC_ST_ID_SPI_COMM_FAIL       (8U)
#define PMIC_ST_ID_DATA_IO_CRC         (9U)
#define PMIC_ST_ID_INTF_SETUP_FAILED   (10U)
#define PMIC_ST_ID_COMM_INTF_INIT_FAIL (11U)
#define PMIC_ST_ID_FAIL                (12U)
#define PMIC_ST_ID_NOT_SUPPORTED       (13U)
#define PMIC_ST_ID_INV_STATUS_TYPE     (14U)
#define PMIC_ST_ID_INV_STATUS_ID       (15U)
#define PMIC_ST_ID_CONFIG_REG_CRC      (16U)
#define PMIC_ST_ID_ERROR_MIN           (PMIC_ST_ID_INV_HANDLE)
#define PMIC_ST_ID_ERROR_MAX           (PMIC_ST_ID_CONFIG_REG_CRC)
#define PMIC_ST_ID_INV_DEVICE_ID       (0U)
#define PMIC_ST_ID_NO_IRQ_REMAINING    (1U)
#define PMIC_ST_ID_WARNING_MIN         (PMIC_ST_ID_INV_DEVICE_ID)
#define PMIC_ST_ID_WARNING_MAX         (PMIC_ST_ID_NO_IRQ_REMAINING)
#define PMIC_ST_ID_SHIFT               (0U)
#define PMIC_ST_ID_MASK                (0xFFFFU << PMIC_ST_ID_SHIFT)

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
 * - **PMIC_ST_ERR_INSUFFICIENT_CFG**: Indicates that required configuration
 *   parameters were not provided or were incomplete. Check that all necessary
 *   configuration structure members are properly initialized and that the
 *   appropriate `validParam` bits are set.
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
 * - **PMIC_ST_ERR_SPI_COMM_FAIL**: Indicates SPI comms. failure. Retry a limited
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
 * The following status codes are specific to the TPS65386x-Q1 device family:
 *
 * - **PMIC_ST_ERR_INV_DEVICE**: Indicates that the device ID read from the PMIC
 *   does not match expected values for this device family.
 *
 * - **PMIC_ST_ERR_INV_SUBSYSTEM**: Indicates that an invalid subsystem was
 *   specified in an API call.
 *
 * - **PMIC_ST_ERR_INTF_SETUP_FAILED**: Indicates that the communication interface
 *   setup failed during initialization.
 *
 * - **PMIC_ST_ERR_COMM_INTF_INIT_FAIL**: Indicates that communication interface
 *   initialization failed.
 *
 * - **PMIC_ST_WARN_INV_DEVICE_ID**: Warning that device ID does not match
 *   expected value but operation can continue.
 *
 * - **PMIC_ST_DEFAULT_DATA**: Indicates that default or placeholder data is
 *   being used.
 *
 * - **PMIC_ST_ERR_CONFIG_REG_CRC**: Indicates a CRC mismatch was detected
 *   during configuration register CRC validation. The CRC computed by the
 *   hardware over the configuration register range did not match the value
 *   stored in CFG_REG_CRC0/CFG_REG_CRC1.
 *
 * **Other Status Codes**
 *
 * Other status codes are for more specific errors which may occur in one of
 * the given submodules of the PMIC driver. The user is referred to that module
 * for more detail.
 *
 * @{
 */
#define PMIC_ST_SUCCESS                 PMIC_STATUS(PMIC_ST_TYPE_SUCCESS, PMIC_ST_ID_SUCCESS)
#define PMIC_ST_ERR_INV_HANDLE          PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_HANDLE)
#define PMIC_ST_ERR_NULL_PARAM          PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_NULL_PARAM)
#define PMIC_ST_ERR_INV_PARAM           PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_PARAM)
#define PMIC_ST_ERR_INV_DEVICE          PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_DEVICE)
#define PMIC_ST_ERR_NULL_FPTR           PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_NULL_FPTR)
#define PMIC_ST_ERR_INV_SUBSYSTEM       PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_SUBSYSTEM)
#define PMIC_ST_ERR_INSUFFICIENT_CFG    PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INSUFFICIENT_CFG)
#define PMIC_ST_ERR_I2C_COMM_FAIL       PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_I2C_COMM_FAIL)
#define PMIC_ST_ERR_SPI_COMM_FAIL       PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_SPI_COMM_FAIL)
#define PMIC_ST_ERR_DATA_IO_CRC         PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_DATA_IO_CRC)
#define PMIC_ST_ERR_INTF_SETUP_FAILED   PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INTF_SETUP_FAILED)
#define PMIC_ST_ERR_COMM_INTF_INIT_FAIL PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_COMM_INTF_INIT_FAIL)
#define PMIC_ST_ERR_FAIL                PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_FAIL)
#define PMIC_ST_ERR_NOT_SUPPORTED       PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_NOT_SUPPORTED)
#define PMIC_ST_ERR_INV_STATUS_TYPE     PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_STATUS_TYPE)
#define PMIC_ST_ERR_INV_STATUS_ID       PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_INV_STATUS_ID)
#define PMIC_ST_ERR_CONFIG_REG_CRC      PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_CONFIG_REG_CRC)
#define PMIC_ST_WARN_INV_DEVICE_ID      PMIC_STATUS(PMIC_ST_TYPE_WARNING, PMIC_ST_ID_INV_DEVICE_ID)
#define PMIC_ST_WARN_NO_IRQ_REMAINING   PMIC_STATUS(PMIC_ST_TYPE_WARNING, PMIC_ST_ID_NO_IRQ_REMAINING)
/** @} */

#define COUNT(x) ((uint8_t)(sizeof(x) / sizeof(x[0])))

// Used to clear statuses
#define PMIC_CLEAR_STAT  ((bool)true)
#define PMIC_RETAIN_STAT ((bool)false)

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
#define PMIC_COMMON_DIAGNOSTIC_CNT_VALID  (1UL << 0U)
#define PMIC_COMMON_DIAGNOSTIC_FLAG_VALID (1UL << 1U)
#define PMIC_COMMON_DIAGNOSTIC_VALID_ALL  (PMIC_COMMON_DIAGNOSTIC_CNT_VALID | PMIC_COMMON_DIAGNOSTIC_FLAG_VALID)
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
#define PMIC_ERR_CNT_OVERFLOW_THR ((uint32_t)UINT8_MAX)
#endif
#ifndef PMIC_WARN_CNT_OVERFLOW_THR
#define PMIC_WARN_CNT_OVERFLOW_THR ((uint32_t)UINT8_MAX)
#endif
#ifndef PMIC_RETRY_CNT_OVERFLOW_THR
#define PMIC_RETRY_CNT_OVERFLOW_THR ((uint32_t)UINT8_MAX)
#endif
/** @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @anchor Pmic_CoreHandle
 * @name PMIC Core Handle
 *
 * @brief Handle used by LLD to abstract platform and OS specific functionality.
 * Also contains PMIC device information.
 *
 * @attention This structure is a central resource used by almost all LLD APIs
 * and must be initialized via 'Pmic_init()' before it can be used by other LLD
 * APIs. End-users should not modify the contents of this structure after it has
 * been initialized, especially during the execution of driver APIs.
 *
 * @param drvInitStat Driver initialization status. Used by LLD as a measure to
 * prevent corrupted handle usage.
 *
 * @param devId PMIC device type.
 *
 * @param pmicDevRev PMIC device revision.
 *
 * @param devSiRev PMIC device silicon revision.
 *
 * @param commMode Communication mode of the PMIC. Some PMICs may only have one
 * communication mode while others could have multiple (e.g., single I2C, dual I2C,
 * SPI).
 *
 * @param i2cAddr0 Main PMIC device address.
 *
 * @param i2cAddr1 Address for interacting with PMIC WDG Q&A.
 *
 * @param i2cAddr2 Address for interacting with PMIC NVM space.
 *
 * @param retryCnt Upon communication related errors (bus error or CRC error),
 * PMIC LLD attempts to retry the failed transaction up to `retryCnt` times before
 * returning an error code to the calling application.
 *
 * @param retryIntervalMs LLD waits this configured amount of time (milliseconds)
 * between retry attempts using the `timerWaitMs()` hook.
 *
 * @param crcEnable Status of whether serial communication CRC is enabled. Set to true
 * if enabled, false otherwise.
 *
 * @param configCrcEnable Status of whether configuration CRC is enabled. Set to true
 * if enabled, false otherwise.
 *
 * @param commHandle0 Pointer to serial communication handle for the PMIC device.
 *
 * @param commHandle1 Pointer to serial communication handle for PMIC WDG.
 *
 * @param ioRead Function pointer to platform-specific serial communication
 * read API.
 *
 * @param ioWrite Function pointer to platform-specific serial communication
 * write API.
 *
 * @param criticalSectionStart Function pointer to OS-specific critical section start.
 *
 * @param criticalSectionStop Function pointer to OS-specific critical section stop.
 *
 * @param irqResponseCallback Function pointer to application-specific IRQ response
 * when an IRQ is detected during WDG servicing.
 *
 * @param timerWaitMs Function pointer to platform-specific timer-based wait API.
 * Upon invocation, the user-implemented hook waits a specified period of time
 * (milliseconds) before returning control to the caller.
 */
typedef struct Pmic_Handle_s {
    uint32_t drvInitStat;
    uint8_t devRev;
    uint8_t devSiRev;
    uint8_t nvmCode;
    uint8_t nvmRev;
    uint8_t commMode;
    uint8_t i2cAddr0;
    uint8_t i2cAddr1;
    uint8_t i2cAddr2;
    uint32_t retryCnt;
    uint32_t retryIntervalMs;
    bool crcEnable;
    bool configCrcEnable;
    void *commHandle0;
    bool asyncEnable;
    void *taskHandle;
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
    int32_t (*asyncRxStart)(const struct Pmic_Handle_s *handle,
                            uint8_t page,
                            uint8_t regAddr,
                            uint8_t *buffer,
                            uint8_t bufLen);
    int32_t (*asyncTxStart)(const struct Pmic_Handle_s *handle,
                            uint8_t page,
                            uint8_t regAddr,
                            const uint8_t *buffer,
                            uint8_t bufLen);
    int32_t (*asyncRxAwait)(const struct Pmic_Handle_s *handle);
    int32_t (*asyncTxAwait)(const struct Pmic_Handle_s *handle);
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

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @brief Checks whether a bit in `validParams` is set. Used by driver APIs to
 * decipher whether a parameter will be processed in their routines.
 *
 * Design: PMICDRV-571
 * Architecture: PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-519, PMICDRV-521,
 *               PMICDRV-522
 *
 * @param validParams [IN] Indication of parameters that are valid. Each bit in
 * the variable corresponds to a structure member. If a bit is 0 in `validParams`,
 * the corresponding parameter is invalid and will not be processed by the calling
 * function. Else, if a bit is 1, the corresponding parameter is valid and will be
 * processed by the calling function.
 *
 * @param bitMask [IN] validParams to check for.
 *
 * @return True if validParams is set, false if validParams is not set.
 */
static inline bool Pmic_validParamCheck(uint32_t validParams, uint32_t bitMask) {
    return ((validParams & bitMask) != 0U);
}

/**
 * @brief Checks whether a parameter is valid and whether the status code is
 * equal to LLD success code.
 *
 * Design: PMICDRV-572
 * Architecture: PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-519, PMICDRV-521,
 *               PMICDRV-522
 *
 * @param validParams [IN] Valid parameter value. Each bit represents whether a parameter
 * is valid.
 *
 * @param bitMask [IN] valid parameter to check for.
 *
 * @param status [IN] The API checks whether the value of this parameter is
 * equal to the PMIC LLD success code.
 *
 * @return True if the status code is equal to the LLD success code and the
 * parameter is valid.
 */
static inline bool Pmic_validParamStatusCheck(uint32_t validParams, uint32_t bitMask, int32_t status) {
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
 * @param handle Pointer to the PMIC core handle structure.
 *
 * @param resource Resource identifier for the critical section.
 *
 * @return void No return value.
 */
void Pmic_criticalSectionStart(const Pmic_Handle_t *handle, uint8_t resource);

/**
 * @brief Stop a critical section after the usage of a shared resource such as an
 * I2C or SPI bus is complete.
 *
 * Design: PMICDRV-574
 * Architecture: PMICDRV-502, PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-507,
 *               PMICDRV-509, PMICDRV-516, PMICDRV-517, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-549
 *
 * @param handle Pointer to the PMIC core handle structure.
 *
 * @param resource Resource identifier for the critical section.
 *
 * @return void No return value.
 */
void Pmic_criticalSectionStop(const Pmic_Handle_t *handle, uint8_t resource);

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
void Pmic_timerWaitMs(const Pmic_Handle_t *handle, uint32_t ms);

/**
 * @brief Sets the bit field of an 8-bit unsigned integer to the desired value.
 *
 * Design: PMICDRV-575
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-521, PMICDRV-522
 *               PMICDRV-549
 *
 * @param regData [OUT] The API modifies the desired bit field of the value held
 * at this address.
 *
 * @param shift [IN] Bit field position.
 *
 * @param mask [IN] Bit field mask.
 *
 * @param value [IN] Desired bit field value to set.
 */
static inline void Pmic_setBitField(uint8_t *regData, uint8_t shift, uint8_t mask, uint8_t value)
{
    *regData = ((*regData & ~mask) | ((value << shift) & mask));
}

/**
 * @brief Sets the bit field of an 8-bit unsigned integer to the desired boolean
 * value.
 *
 * Design: PMICDRV-577
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-521, PMICDRV-522
 *               PMICDRV-549
 *
 * @param regData [OUT] The API modifies the desired bit field of the value held
 * at this address.
 *
 * @param shift [IN] Bit field position.
 *
 * @param value [IN] Bit field value (either true or false).
 */
static inline void Pmic_setBitField_b(uint8_t *regData, uint8_t shift, bool value)
{
    Pmic_setBitField(regData, shift, (uint8_t)(1U << (shift & 0x07U)), value ? 1U : 0U);
}

/**
 * @brief Get desired bit field of an 8-bit unsigned integer.
 *
 * Design: PMICDRV-578
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-521, PMICDRV-522
 *               PMICDRV-549
 *
 * @param regData [IN] The API gets the desired bit field from this value.
 *
 * @param shift [IN] Bit field location.
 *
 * @param mask [IN] Bit field mask.
 *
 * @return Value of the desired bit field.
 */
static inline uint8_t Pmic_getBitField(uint8_t regData, uint8_t shift, uint8_t mask)
{
    return ((regData & mask) >> shift);
}

/**
 * @brief Gets the desired bit field of an 8-bit unsigned integer, casted as a
 * boolean.
 *
 * Design: PMICDRV-580
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-507, PMICDRV-516, PMICDRV-521, PMICDRV-522
 *               PMICDRV-549
 *
 * @param regData [IN] The API gets the desired bit field from this value.
 *
 * @param shift [IN] Bit field location.
 *
 * @return Value of the desired bit field.
 */
static inline bool Pmic_getBitField_b(uint8_t regData, uint8_t shift)
{
    return Pmic_getBitField(regData, shift, (uint8_t)(1U << (shift & 0x07U))) == 1U;
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
