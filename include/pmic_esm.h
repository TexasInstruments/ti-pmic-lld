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
 * @file pmic_esm.h
 *
 * @brief PMIC LLD ESM module header file.
 *
 * @details This module contains declarations/definitions of macros, data
 * structures, and APIs used to interact with the PMIC error signal monitor
 * (ESM). Some components of the PMIC ESM module are as follows: setting and
 * getting ESM configurations, starting and stopping the ESM, getting ESM
 * status, clearing ESM status, and getting ESM error count.
 */
#ifndef __PMIC_ESM_H__
#define __PMIC_ESM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/**
 * @anchor Pmic_EsmCfgValidParams
 * @name TPS65036x PMIC ESM Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_EsmCfg_t struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_EsmCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_ESM_ENABLE_VALID               ((uint32_t)1U << 0U)
#define PMIC_ESM_MODE_VALID                 ((uint32_t)1U << 1U)
#define PMIC_ESM_ERR_CNT_THR_VALID          ((uint32_t)1U << 2U)
#define PMIC_ESM_DELAY1_VALID               ((uint32_t)1U << 3U)
#define PMIC_ESM_DELAY2_VALID               ((uint32_t)1U << 4U)
#define PMIC_ESM_HMAX_VALID                 ((uint32_t)1U << 5U)
#define PMIC_ESM_HMIN_VALID                 ((uint32_t)1U << 6U)
#define PMIC_ESM_LMAX_VALID                 ((uint32_t)1U << 7U)
#define PMIC_ESM_LMIN_VALID                 ((uint32_t)1U << 8U)
/** @} */

/**
 * @anchor Pmic_EsmStatValidParams
 * @name TPS65036x PMIC ESM Status Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_EsmStat_t struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_EsmCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_ESM_RST_INT_VALID              ((uint32_t)1U << 0U)
#define PMIC_ESM_FAIL_INT_VALID             ((uint32_t)1U << 1U)
#define PMIC_ESM_PIN_INT_VALID              ((uint32_t)1U << 2U)
#define PMIC_ESM_STATUS_ALL_VALID           (PMIC_ESM_RST_INT_VALID | \
                                            PMIC_ESM_FAIL_INT_VALID | \
                                            PMIC_ESM_PIN_INT_VALID)
/** @} */

/**
 * @anchor Pmic_esmModes
 * @name TPS65036x ESM Modes
 *
 * @brief Valid values of the ESM_MCU_MODE bit field.
 */
#define ESM_LEVEL_MODE                      ((uint8_t)0U)
#define ESM_PWM_MODE                        ((uint8_t)1U)
#define ESM_MODE_MAX                        (ESM_PWM_MODE)
/** @} */

/**
 * @anchor Pmic_EsmErrCntThrMaxValue
 * @name TPS65036x ESM Error Count Threshold Max Value
 *
 * @brief Maximum value of the ESM_MCU_ERR_CNT_TH bit field.
 */
#define ESM_ERR_CNT_THR_MAX                 ((uint8_t)0x0FU)
/** @} */

/**
 * @anchor Pmic_esmStartStopValues
 * @name TPS65036x ESM Start/Stop Values
 *
 * @brief Valid values of the ESM_MCU_START bit field.
 */
#define PMIC_ESM_START                      ((bool)true)
#define PMIC_ESM_STOP                       ((bool)false)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_EsmCfg
 * @name PMIC ESM Configuration Struct
 *
 * @brief Struct used to set and get ESM configurations.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. For valid values, refer to @ref Pmic_EsmCfgValidParams.
 *
 * @param enable ESM activation. When set to true, the ESM is activated.
 * otherwise, the ESM is deactivated.
 *
 * @param mode Mode of operation. For valid values, refer to @ref Pmic_esmModes.
 *
 * @param errCntThr Error counter threshold. For the maximum threshold value,
 * refer to @ref Pmic_EsmErrCntThrMaxValue.
 *
 * @param delay1 Delay-1 time interval configuration. The ESM starts the delay-1
 * timer upon detecting an ESM error.
 *
 * @param delay2 Delay-2 time interval configuration. The ESM starts the delay-2
 * timer if the ESM error is still present or any ESM interrupts are pending
 * after delay-1 interval has elapsed.
 *
 * @param hmax Maximum high-pulse time threshold for the ESM operating in PWM mode.
 *
 * @param hmin Minimum high-pulse time threshold for the ESM operating in PWM mode.
 *
 * @param lmax Maximum low-pulse time threshold for the ESM operating in PWM mode.
 *
 * @param lmin Minimum low-pulse time threshold for the ESM operating in PWM mode.
 *
 */
typedef struct Pmic_EsmCfg_s
{
    uint32_t validParams;

    bool enable;
    uint8_t mode;
    uint8_t errCntThr;

    uint8_t delay1;
    uint8_t delay2;

    uint8_t hmax;
    uint8_t hmin;
    uint8_t lmax;
    uint8_t lmin;
} Pmic_EsmCfg_t;

/**
 * @anchor Pmic_EsmStat
 * @name PMIC ESM Status Struct
 *
 * @brief Struct used to get and clear ESM error statuses.
 *
 * @param validParams  Each bit in this variable represents whether a struct
 * member is valid. For valid values, refer to @ref Pmic_EsmStatValidParams.
 *
 * @param rstInt Status indicating that MCU ESM reset has been detected.
 *
 * @param failInt Status indicating that MCU ESM fail has been detected.
 *
 * @param pinInt Status indicating that MCU ESM fault has been detected.
 */
typedef struct Pmic_EsmStat_s
{
    uint32_t validParams;

    bool rstInt;
    bool failInt;
    bool pinInt;
} Pmic_EsmStat_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Set PMIC ESM configurations.
 *
 * @details The following options are configurable via this API
 * 1. Enable (validParam: PMIC_ESM_ENABLE_VALID)
 * 2. Mode (validParam: PMIC_ESM_MODE_VALID)
 * 3. Error count threshold (validParam: PMIC_ESM_ERR_CNT_THR_VALID)
 * 4. Delay-1 (validParam: PMIC_ESM_DELAY1_VALID)
 * 5. Delay-2 (validParam: PMIC_ESM_DELAY2_VALID)
 * 6. Maximum high-pulse time threshold (validParam: PMIC_ESM_HMAX_VALID)
 * 7. Minimum high-pulse time threshold (validParam: PMIC_ESM_HMIN_VALID)
 * 8. Maximum low-pulse time threshold (validParam: PMIC_ESM_LMAX_VALID)
 * 9. Minimum low-pulse time threshold (validParam: PMIC_ESM_LMIN_VALID)
 * For more information on ESM configurations, refer to @ref Pmic_EsmCfg.
 *
 * @attention Some ESM configurations can only be set when the ESM is stopped
 * (ESM_START=0). To stop the ESM, see Pmic_esmStartStop() or Pmic_esmStop().
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param esmCfg [IN] ESM configurations to write to PMIC.
 *
 * @return Success code if PMIC ESM configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmSetCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg);

/**
 * @brief Get PMIC ESM configurations. This "Get" API supports obtaining
 * the same parameters that are settable through the "Set" API (`Pmic_esmSetCfg`).
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param esmCfg [OUT] ESM configurations obtained from PMIC.
 *
 * @return Success code if PMIC ESM configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmGetCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg);

/**
 * @brief Start/stop the PMIC ESM.
 *
 * @details This API configures the control bit to start/stop the PMIC ESM. Some
 * ESM configurations can only be writen when the ESM is stopped.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param start [IN] ESM start/stop. For valid values, refer to
 * @ref Pmic_esmStartStopValues
 *
 * @return Success code if PMIC ESM_MCU_START bit has been configured, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmStartStop(const Pmic_CoreHandle_t *pmicHandle, bool start);

/**
 * @brief Start the PMIC ESM.
 *
 * @note End-user should ensure their desired ESM configurations are set prior
 * to starting the ESM.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC ESM_MCU_START bit has been set to 1, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmStart(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Stop the PMIC ESM.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC ESM_MCU_START bit has been set to 0, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmStop(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Get the PMIC ESM start/stop status.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param start [OUT] ESM start/stop status. If set to true or `PMIC_ESM_START`,
 * ESM has started. Otherwise, if set to false or `PMIC_ESM_STOP`, ESM has
 * stopped.
 *
 * @return Success code if PMIC ESM start/stop status has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmGetStartStop(const Pmic_CoreHandle_t *pmicHandle, bool *start);

/**
 * @brief Get PMIC ESM status.
 *
 * @details The following ESM statuses are obtainable from this API
 * 1. ESM_MCU_RST_INT (validParam: PMIC_ESM_RST_INT_VALID)
 * 2. ESM_MCU_FAIL_INT (validParam: PMIC_ESM_FAIL_INT_VALID)
 * 3. ESM_MCU_PIN_INT (validParam: PMIC_ESM_PIN_INT_VALID)
 * For more information on the ESM statuses, refer to @ref Pmic_EsmStat.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param esmStat [OUT] ESM statuses obtained from PMIC.
 *
 * @return Success code if PMIC ESM statuses have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmGetStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmStat_t *esmStat);

/**
 * @brief Clear PMIC ESM statuses. This API supports clearing the same statuses
 * that are obtainable through the "Get" API (`Pmic_esmGetStat`).
 *
 * @attention If end-user calls this API without addressing the root cause of
 * the ESM error statuses, the statuses could continue to be set after API call.
 *
 * @note To indicate the desired ESM error status(es) to clear, the validParams
 * struct member of \p esmStat parameter must be set. All other struct members
 * will be ignored/unused throughout API execution. For valid values of validParams,
 * refer to @ref Pmic_EsmStatValidParams.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param esmStat [IN] ESM statuses to be cleared.
 *
 * @return Success code if PMIC ESM statues have been cleared, error code
 * otherwise. for valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmClrStat(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmStat_t *esmStat);

/**
 * @brief Get PMIC ESM error count.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param errCnt [OUT] ESM error count.
 *
 * @return Success code if PMIC ESM error count has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmGetErrCnt(const Pmic_CoreHandle_t *pmicHandle, uint8_t *errCnt);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_ESM_H__ */
