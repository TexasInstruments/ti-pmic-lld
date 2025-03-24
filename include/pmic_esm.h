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
#ifndef __PMIC_ESM_H__
#define __PMIC_ESM_H__

/**
 * @file pmic_esm.h
 * @brief PMIC Driver ESM API/Interface
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdbool.h>
#include <stdint.h>

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_EsmCfgValidParam
 * @name PMIC ESM Configuration Valid Params
 *
 * @brief Valid parameters of the ESM configuration structure (Pmic_EsmCfg_t).
 *
 * @{
 */
#define PMIC_CFG_ESM_ENABLE_VALID       (0U)
#define PMIC_CFG_ESM_MODE_VALID         (1U)
#define PMIC_CFG_ESM_ERR_THR_VALID      (2U)
#define PMIC_CFG_ESM_POLARITY_VALID     (3U)
#define PMIC_CFG_ESM_DEGLITCH_VALID     (4U)
#define PMIC_CFG_ESM_TIME_BASE_VALID    (5U)
#define PMIC_CFG_ESM_DELAY1_VALID       (6U)
#define PMIC_CFG_ESM_DELAY2_VALID       (7U)
#define PMIC_CFG_ESM_HMAX_VALID         (8U)
#define PMIC_CFG_ESM_HMIN_VALID         (9U)
#define PMIC_CFG_ESM_LMAX_VALID         (10U)
#define PMIC_CFG_ESM_LMIN_VALID         (11U)
/** @} */

/**
 * @anchor Pmic_EsmCfgValidParamShift
 * @name PMIC ESM Configuration Valid Param Shifts
 *
 * @brief Valid parameter shifts of the ESM configuration structure
 * (Pmic_EsmCfg_t). End-user can set validParams to be equal to a combination
 * of the defines listed below to indicate which parameters are valid.
 *
 * @{
 */
#define PMIC_CFG_ESM_ENABLE_VALID_SHIFT     (1U << PMIC_CFG_ESM_ENABLE_VALID)
#define PMIC_CFG_ESM_MODE_VALID_SHIFT       (1U << PMIC_CFG_ESM_MODE_VALID)
#define PMIC_CFG_ESM_ERR_THR_VALID_SHIFT    (1U << PMIC_CFG_ESM_ERR_THR_VALID)
#define PMIC_CFG_ESM_POLARITY_VALID_SHIFT   (1U << PMIC_CFG_ESM_POLARITY_VALID)
#define PMIC_CFG_ESM_DEGLITCH_VALID_SHIFT   (1U << PMIC_CFG_ESM_DEGLITCH_VALID)
#define PMIC_CFG_ESM_TIME_BASE_VALID_SHIFT  (1U << PMIC_CFG_ESM_TIME_BASE_VALID)
#define PMIC_CFG_ESM_DELAY1_VALID_SHIFT     (1U << PMIC_CFG_ESM_DELAY1_VALID)
#define PMIC_CFG_ESM_DELAY2_VALID_SHIFT     (1U << PMIC_CFG_ESM_DELAY2_VALID)
#define PMIC_CFG_ESM_HMAX_VALID_SHIFT       (1U << PMIC_CFG_ESM_HMAX_VALID)
#define PMIC_CFG_ESM_HMIN_VALID_SHIFT       (1U << PMIC_CFG_ESM_HMIN_VALID)
#define PMIC_CFG_ESM_LMAX_VALID_SHIFT       (1U << PMIC_CFG_ESM_LMAX_VALID)
#define PMIC_CFG_ESM_LMIN_VALID_SHIFT       (1U << PMIC_CFG_ESM_LMIN_VALID)
/** @} */

/**
 * @anchor Pmic_EsmStatusValidParam
 * @name PMIC ESM Status Valid Params
 *
 * @brief Valid parameters of the ESM status structure (Pmic_EsmStatus_t).
 *
 * @{
 */
#define PMIC_ESM_ERR_VALID          (0U)
#define PMIC_ESM_DELAY1_ERR_VALID   (1U)
#define PMIC_ESM_DELAY2_ERR_VALID   (2U)
#define PMIC_ESM_ERR_CNT_VALID      (3U)
/** @} */

/**
 * @anchor Pmic_EsmStatusValidParamShift
 * @name PMIC ESM Status Valid Param Shifts
 *
 * @brief Valid Parameter shifts of the ESM status structure (Pmic_EsmStatus_t).
 * End-user can set validParams to be equal to a combination of the defines
 * listed below to indicate which parameters are valid.
 *
 * @{
 */
#define PMIC_ESM_ERR_VALID_SHIFT        (1U << PMIC_ESM_ERR_VALID)
#define PMIC_ESM_DELAY1_ERR_VALID_SHIFT (1U << PMIC_ESM_DELAY1_ERR_VALID)
#define PMIC_ESM_DELAY2_ERR_VALID_SHIFT (1U << PMIC_ESM_DELAY2_ERR_VALID)
#define PMIC_ESM_ERR_CNT_VALID_SHIFT    (1U << PMIC_ESM_ERR_CNT_VALID)
/** @} */

/**
 * @anchor Pmic_EsmMode
 * @name PMIC ESM Mode
 *
 * @brief PMIC ESM modes of operation.
 *
 * @{
 */
#define PMIC_ESM_LEVEL_MODE (0U)
#define PMIC_ESM_PWM_MODE   (1U)
#define PMIC_ESM_MODE_MAX   (PMIC_ESM_PWM_MODE)
/** @} */

/**
 * @anchor PMIC_EsmErrThrMaxVal
 * @name PMIC ESM Error Threshold Maximum Value
 *
 * @brief Maximum value of the ESM error counter threshold.
 *
 * @{
 */
#define PMIC_ESM_ERR_THR_MAX (0xFU)
/** @} */

/**
 * @anchor PMIC_EsmPolarity
 * @name PMIC ESM Polarity
 *
 * @brief Polarity configuration for the ESM operating in Level Mode.
 *
 * @{
 */
#define PMIC_ESM_POLARITY_LOW_GOOD  (0U)
#define PMIC_ESM_POLARITY_HIGH_GOOD (1U)
#define PMIC_ESM_POLARITY_MAX       (PMIC_ESM_POLARITY_HIGH_GOOD)
/** @} */

/**
 * @anchor PMIC_EsmDeglitch
 * @name PMIC ESM Deglitch
 *
 * @brief ESM Deglitch configuration.
 *
 * @{
 */
#define PMIC_ESM_DEGLITCH_1_US  (0U)
#define PMIC_ESM_DEGLITCH_4_US  (1U)
#define PMIC_ESM_DEGLITCH_MAX   (PMIC_ESM_DEGLITCH_4_US)
/** @} */

/**
 * @anchor PMIC_EsmTimeBase
 * @name PMIC ESM Time Base
 *
 * @brief ESM time base configuration.
 *
 * @{
 */
#define PMIC_ESM_TIME_BASE_2_US     (0U)
#define PMIC_ESM_TIME_BASE_4_US     (1U)
#define PMIC_ESM_TIME_BASE_8_US     (2U)
#define PMIC_ESM_TIME_BASE_16_US    (3U)
#define PMIC_ESM_TIME_BASE_32_US    (4U)
#define PMIC_ESM_TIME_BASE_48_US    (5U)
#define PMIC_ESM_TIME_BASE_64_US    (6U)
#define PMIC_ESM_TIME_BASE_96_US    (7U)
#define PMIC_ESM_TIME_BASE_MAX      (PMIC_ESM_TIME_BASE_96_US)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 * @anchor Pmic_EsmCfg
 * @name PMIC ESM Configuration Struct
 *
 * @brief Struct used to set and get ESM configurations.
 *
 * @param validParams For valid values, see @ref Pmic_EsmCfgValidParamShift.
 *
 * @param enable Whether the ESM module should be enabled or not. For valid
 * values, refer to @ref Pmic_EnableDisable.
 *
 * @param mode ESM mode of operation. For valid values, refer to
 * @ref Pmic_EsmMode
 *
 * @param errThr ESM error counter threshold. For the maximum value, refer to
 * @ref PMIC_EsmErrThrMaxVal
 *
 * @param polarity ESM polarity in Level Mode. For valid values, refer to
 * @ref PMIC_EsmPolarity
 *
 * @param deglitch ESM input deglitch. For valid values, refer to
 * @ref PMIC_EsmDeglitch
 *
 * @param timeBase ESM time base. For valid values, refer to
 * @ref PMIC_EsmTimeBase
 *
 * @param delay1 ESM delay-1 time interval.
 *
 * @param delay2 ESM delay-2 time interval.
 *
 * @param hmax ESM maximum high-pulse time threshold.
 *
 * @param hmin ESM minimum high-pulse time threshold.
 *
 * @param lmax ESM maximum low-pulse time threshold.
 *
 * @param lmin ESM minimum low-pulse time threshold.
 */
typedef struct Pmic_EsmCfg_s {
    uint32_t validParams;

    bool enable;
    uint8_t mode;

    uint8_t errThr;
    uint8_t polarity;
    uint8_t deglitch;
    uint8_t timeBase;

    uint8_t delay1;
    uint8_t delay2;

    uint8_t hmax;
    uint8_t hmin;
    uint8_t lmax;
    uint8_t lmin;
} Pmic_EsmCfg_t;

/**
 * @anchor Pmic_EsmStatus
 * @name PMIC ESM Status Struct
 *
 * @brief Struct used to get and clear ESM statuses.
 *
 * @param validParams For valid values, see @ref Pmic_EsmStatusValidParamShift.
 *
 * @param esmErr ESM error flag.
 *
 * @param delay1Err ESM delay-1 error flag.
 *
 * @param delay2Err ESM delay-2 error flag.
 *
 * @param errCnt ESM error counter.
 */
typedef struct Pmic_EsmStatus_s {
    uint32_t validParams;

    bool esmErr;
    bool delay1Err;
    bool delay2Err;

    uint8_t errCnt;
} Pmic_EsmStatus_t;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Start/stop the PMIC ESM. This API is a superset of Pmic_esmStart() and
 * Pmic_esmStop().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param start [IN] True - start; false - stop.
 *
 * @return Success code if PMIC ESM start state is configured, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_esmSetStartState(Pmic_CoreHandle_t *handle, bool start);

/**
 * @brief Start PMIC ESM. This API is a subset of Pmic_esmSetStartState().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC ESM start is enabled, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_esmStart(Pmic_CoreHandle_t *handle);

/**
 * @brief Stop PMIC ESM. This API is a subset of Pmic_esmSetStartState().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC ESM start is disabled, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_esmStop(Pmic_CoreHandle_t *handle);

/**
 * @brief Get the state of the PMIC ESM's start bit.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param start [OUT] ESM start bit value.
 *
 * @return Success code if PMIC ESM start state has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_esmGetStartState(Pmic_CoreHandle_t *handle, bool *start);

/**
 * @brief Set PMIC ESM configurations.
 *
 * @details The following options are configurable via this API
 * 1. Enable (validParam: PMIC_CFG_ESM_ENABLE_VALID_SHIFT)
 * 2. Mode (validParam: PMIC_CFG_ESM_MODE_VALID_SHIFT)
 * 3. Error Threshold (validParam: PMIC_CFG_ESM_ERR_THR_VALID_SHIFT)
 * 4. Polarity (validParam: PMIC_CFG_ESM_POLARITY_VALID_SHIFT)
 * 5. Deglitch (validParam: PMIC_CFG_ESM_DEGLITCH_VALID_SHIFT)
 * 6. Time base (validParam: PMIC_CFG_ESM_TIME_BASE_VALID_SHIFT)
 * 7. Delay 1 (validParam: PMIC_CFG_ESM_DELAY1_VALID_SHIFT)
 * 8. Delay 2 (validParam: PMIC_CFG_ESM_DELAY2_VALID_SHIFT)
 * 9. HMAX (validParam: PMIC_CFG_ESM_HMAX_VALID_SHIFT)
 * 10. HMIN (validParam: PMIC_CFG_ESM_HMIN_VALID_SHIFT)
 * 11. LMAX (validParam: PMIC_CFG_ESM_LMAX_VALID_SHIFT)
 * 12. LMIN (validParam: PMIC_CFG_ESM_LMIN_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param esmCfg [IN] ESM configurations to set.
 *
 * @return Success code if PMIC ESM configurations are set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_esmSetCfg(Pmic_CoreHandle_t *handle, const Pmic_EsmCfg_t *esmCfg);

/**
 * @brief Get PMIC ESM configurations. This API supports obtaining the same
 * configurations that are settable by Pmic_esmSetCfg().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param esmCfg [OUT] ESM configurations obtained from the PMIC.
 *
 * @return Success code if PMIC ESM configurations are obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_esmGetCfg(Pmic_CoreHandle_t *handle, Pmic_EsmCfg_t *esmCfg);

/**
 * @brief Get PMIC ESM statuses.
 *
 * @details The following ESM statuses are obtainable via this API
 * 1. ESM Error (validParam: PMIC_ESM_ERR_VALID)
 * 2. Delay 1 Error (validParam: PMIC_DELAY1_ERR_VALID)
 * 3. Delay 2 Error (validParam: PMIC_DELAY2_ERR_VALID)
 * 4. ESM Error Count (validParam: PMIC_ERR_CNT_VALID)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param esmStat [OUT] ESM statuses obtained from the PMIC.
 *
 * @return Success code if PMIC ESM statuses are obtained, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_esmGetStatus(Pmic_CoreHandle_t *handle, Pmic_EsmStatus_t *esmStat);

/**
 * @brief Clear PMIC ESM statuses.
 *
 * @details The following ESM statuses can be cleared via this API
 * 1. ESM Error (validParam: PMIC_ESM_ERR_VALID)
 * 2. Delay 1 Error (validParam: PMIC_DELAY1_ERR_VALID)
 * 3. Delay 2 Error (validParam: PMIC_DELAY2_ERR_VALID)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param esmStat [IN] ESM statuses to be cleared. validParams indicates the
 * desired statuses to be cleared - all other struct members are ignored.
 *
 * @return Success code if PMIC ESM statuses are cleared, error code otherwise.
 * For valid success/error codes, refer @ref Pmic_ErrorCodes
 */
int32_t Pmic_esmClrStatus(Pmic_CoreHandle_t *handle, const Pmic_EsmStatus_t *esmStat);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_ESM_H__ */
