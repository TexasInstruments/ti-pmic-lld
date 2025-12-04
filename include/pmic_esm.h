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
#ifndef PMIC_ESM_H
#define PMIC_ESM_H

/**
 * @file pmic_esm.h
 *
 * @brief PMIC error signal monitor (ESM) interface. Contains APIs, macros/defines,
 * and data structures used to configure, control, and interact with the PMIC ESM.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 * @anchor Pmic_EsmMode
 * @name PMIC ESM Mode
 *
 * @brief Enumerations of valid PMIC ESM modes.
 *
 * @{
 */
#define PMIC_ESM_LEVEL_MODE (0U)
#define PMIC_ESM_PWM_MODE   (1U)
#define PMIC_ESM_MODE_MIN   (PMIC_ESM_LEVEL_MODE)
#define PMIC_ESM_MODE_MAX   (PMIC_ESM_PWM_MODE)
/** @} */

/**
 * @anchor Pmic_EsmErrCntThr
 * @name PMIC ESM Error Count Threshold
 *
 * @brief Minimum and maximum PMIC ESM error counter thresholds.
 *
 * @{
 */
#define PMIC_ESM_ERR_CNT_THR_MIN (0x00U)
#define PMIC_ESM_ERR_CNT_THR_MAX (0x0FU)
/** @} */

/**
 * @anchor Pmic_EsmCfgValidParams
 * @name PMIC ESM Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_EsmCfg_t`. Set
 * the `validParams` member of `Pmic_EsmCfg_t` equal to a combination of these
 * defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_ESM_MODE_VALID                 (1U << 0U)
#define PMIC_ESM_ERR_CNT_THR_VALID          (1U << 1U)
#define PMIC_ESM_DELAY1_VALID               (1U << 2U)
#define PMIC_ESM_DELAY2_VALID               (1U << 3U)
#define PMIC_ESM_HMAX_VALID                 (1U << 4U)
#define PMIC_ESM_HMIN_VALID                 (1U << 5U)
#define PMIC_ESM_LMAX_VALID                 (1U << 6U)
#define PMIC_ESM_LMIN_VALID                 (1U << 7U)
#define PMIC_ESM_DISABLE_CAN_ON_FAULT_VALID (1U << 8U)
#define PMIC_ESM_LEVEL_MODE_VALID_ALL       (\
    PMIC_ESM_MODE_VALID | \
    PMIC_ESM_DELAY1_VALID | \
    PMIC_ESM_DELAY2_VALID | \
    PMIC_ESM_DISABLE_CAN_ON_FAULT_VALID)
#define PMIC_ESM_PWM_MODE_VALID_ALL         (\
    PMIC_ESM_MODE_VALID | \
    PMIC_ESM_ERR_CNT_THR_VALID | \
    PMIC_ESM_DELAY1_VALID | \
    PMIC_ESM_DELAY2_VALID | \
    PMIC_ESM_HMAX_VALID | \
    PMIC_ESM_HMIN_VALID | \
    PMIC_ESM_LMAX_VALID | \
    PMIC_ESM_LMIN_VALID | \
    PMIC_ESM_DISABLE_CAN_ON_FAULT_VALID)
/** @} */

/**
 * @anchor Pmic_EsmErrStatusValidParams
 * @name PMIC ESM Error Status Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_EsmErrStatus_t`.
 * Set the `validParams` member of `Pmic_EsmErrStatus_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_ESM_RST_INT_VALID        (1U << 0U)
#define PMIC_ESM_FAIL_INT_VALID       (1U << 1U)
#define PMIC_ESM_PIN_INT_VALID        (1U << 2U)
#define PMIC_ESM_ERR_STATUS_VALID_ALL (\
    PMIC_ESM_RST_INT_VALID | \
    PMIC_ESM_FAIL_INT_VALID | \
    PMIC_ESM_PIN_INT_VALID)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 * @anchor Pmic_EsmCfg
 * @name PMIC ESM configuration Structure
 *
 * @brief Structure used to set/get configurations of the PMIC error signal
 * monitor (ESM).
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_EsmCfgValidParams.
 *
 * @param mode ESM mode of operation. For valid values, refer to @ref Pmic_EsmMode.
 *
 * @param errCntThr ESM error count threshold. For valid values, refer to @ref Pmic_EsmErrCntThr.
 *
 * @param delay1 ESM delay 1. All possible values of its data type are valid.
 *
 * @param delay2 ESM delay 2. All possible values of its data type are valid.
 *
 * @param hmax ESM maximum high duration code. All possible values of its data
 * type are valid. See PMIC device data sheet to convert from code to time.
 *
 * @param hmin ESM minimum high duration code. All possible values of its data
 * type are valid. See PMIC device data sheet to convert from code to time.
 *
 * @param lmax ESM low maximum duration code. All possible values of its data
 * type are valid. See PMIC device data sheet to convert from code to time.
 *
 * @param lmin ESM low minimum duration code. All possible values of its data
 * type are valid. See PMIC device data sheet to convert from code to time.
 *
 * @param disableCanOnFault Control whether ESM disables CAN on ESM_MCU_FAIL_INT
 * fault. When set to true, CAN_DIS is set active on ESM_MCU_FAIL_INT fault.
 */
typedef struct Pmic_EsmCfg_s {
    uint32_t validParams;

    uint8_t mode;
    uint8_t errCntThr;

    uint8_t delay1;
    uint8_t delay2;

    /* Valid only for ESM in PWM mode */
    uint8_t hmax;
    uint8_t hmin;
    uint8_t lmax;
    uint8_t lmin;

    bool disableCanOnFault;
} Pmic_EsmCfg_t;

/**
 * @anchor Pmic_EsmErrStatus
 * @name PMIC ESM Error Status Structure
 *
 * @brief Structure used to get and clear error statuses of the PMIC ESM.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_EsmErrStatusValidParams.
 *
 * @param rstInt Indicates that delay-1 and delay-2 have elapsed, but the ESM error
 * is still present or ESM interrupts have not yet been cleared. The PMIC device FSM
 * will handle this error has a trigger for warm reset.
 *
 * @param failInt If delay-2 is non-zero, this status indicates that delay-1 has
 * elapsed and either the ESM error is still present or ESM interrupts have not
 * yet been cleared.
 *
 * @param pinInt Indicates whether the PMIC has detected an error on its ESM
 * input pin.
 *
 * @{
 */
typedef struct Pmic_EsmErrStatus_s {
   uint32_t validParams;

   bool rstInt;
   bool failInt;
   bool pinInt;
} Pmic_EsmErrStatus_t;
/** @} */

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Start/stop the PMIC ESM.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enable [IN] `PMIC_ENABLE` - start the ESM; `PMIC_DISABLE` - stop the
 * ESM.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM has been successfully started/stopped,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmSetStartState(const Pmic_Handle_t *handle, bool enable);

/**
 * @brief Get PMIC ESM start/stop state.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] `PMIC_ENABLE` - ESM is started;
 * `PMIC_DISABLE` - ESM is stopped.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM start/stop state has been successfully
 * obtained, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmGetStartState(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Enable/disable the PMIC ESM.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enable [IN] `PMIC_ENABLE` - Enable the ESM;
 * `PMIC_DISABLE` - Disable the ESM.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM has been successfully enabled/disabled,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmSetEnableState(const Pmic_Handle_t *handle, bool enable);

/**
 * @brief Get PMIC ESM enable state.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] `PMIC_ENABLE` - ESM is enabled;
 * `PMIC_DISABLE` - ESM is disabled.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM enable state has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Set PMIC ESM configurations.
 *
 * @note The MCU can configure the ESM as long as its related start bit
 * (ESM_MCU_START) is cleared to 0. As soon as the MCU sets the start bit, the
 * PMIC device sets a write-protection on the ESM configuration except the start
 * bit, ESM_MCU_START.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param esmCfg [IN] Desired ESM configurations to set. For more information on
 * ESM configurations, refer to @ref Pmic_EsmCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM configurations have been set, error code
 * otherwise. for valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmSetCfg(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg);

/**
 * @brief Get PMIC ESM configurations. This API supports getting the same
 * configurations that are settable by `Pmic_esmSetCfg()`.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param esmCfg [OUT] ESM configurations obtained from the PMIC. For more
 * information on ESM configurations, refer to @ref Pmic_EsmCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmGetCfg(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg);

/**
 * @brief Clear PMIC ESM statuses.
 *
 * @note To indicate the desired ESM error status(es) to clear, the
 * validParams struct member of `errStatus` parameter must be set. All other
 * struct members will be ignored/unused throughout API execution. For valid
 * values of validParams, refer to @ref Pmic_EsmErrStatusValidParams.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param errStatus [IN] The validParams struct member of this parameter indicates
 * which ESM error status(es) to clear. For more information on ESM error statuses,
 * refer to @ref Pmic_EsmErrStatus.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM status(es) have been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmClrErrStatus(const Pmic_Handle_t *handle, const Pmic_EsmErrStatus_t *errStatus);

/**
 * @brief Clear all PMIC ESM error statuses. This API clears all statuses that
 * are clearable by `Pmic_esmClrErrStatus()`.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM error statuses have been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmClrErrStatusAll(const Pmic_Handle_t *handle);

/**
 * @brief Get PMIC ESM error statuses.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param errStatus [OUT] PMIC ESM error statuses obtained from the PMIC. For
 * more information on ESM error statuses, refer to @ref Pmic_EsmErrStatus.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM error statuses have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmGetErrStatus(const Pmic_Handle_t *handle, Pmic_EsmErrStatus_t *errStatus);

/**
 * @brief Get PMIC ESM error counter.
 *
 * @details The ESM has an error-counter which increments by +2 after each bad
 * event, and decrements with -1 after each good event. The ESM detects an ESM
 * error when the error-counter value is more than its related threshold value.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param errCnt [OUT] Error counter value obtained from the PMIC.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM error counter has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmGetErrCnt(const Pmic_Handle_t *handle, uint8_t *errCnt);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_ESM_H */
