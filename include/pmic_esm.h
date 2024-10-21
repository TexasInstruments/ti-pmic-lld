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
 * structures, and APIs used to interact with PMIC ESM functionalities.
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
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_EsmCfgValidParams
 * @name PMIC ESM Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_EsmCfg_t
 * struct. For more information on the parameters, refer to @ref Pmic_EsmCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 * @{
 */
#define PMIC_ESM_ENABLE_VALID               ((uint8_t)1U << 0U)
#define PMIC_ESM_PWM_EN_VALID               ((uint8_t)1U << 1U)
#define PMIC_ESM_LOCK_STEP_RST_EN_VALID     ((uint8_t)1U << 2U)
#define PMIC_ESM_MCU_ERR_CNT_THR_VALID      ((uint8_t)1U << 3U)
#define PMIC_ESM_LMIN_VALID                 ((uint8_t)1U << 4U)
#define PMIC_ESM_LMAX_VALID                 ((uint8_t)1U << 5U)
#define PMIC_ESM_HMIN_VALID                 ((uint8_t)1U << 6U)
#define PMIC_ESM_HMAX_VALID                 ((uint8_t)1U << 7U)
/** @} */

/**
 * @anchor Pmic_EsmMcuErrCntThrMaxVal
 * @name PMIC ESM MCU Error Count Threshold Max value
 *
 * @brief Maximum value of the MCU_ERR_CNT_TH[2:0] bit field.
 *
 * @{
 */
#define PMIC_ESM_MCU_ERR_CNT_THR_MAX        ((uint8_t)0x7U)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_EsmCfg
 * @name PMIC ESM Configuration Struct
 *
 * @brief Used to set and get ESM configurations.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. for valid values, refer to @ref Pmic_EsmCfgValidParams.
 *
 * @param enable ESM enable/disable. This setting is only effective if WDG is in
 * Q&A mode.
 *
 * @param pwmEn ESM PWM enable. If set to true, ESM operates in PWM mode. If set
 * to false and WDG is configured to Trigger mode, ESM is disabled. If set to
 * false and WDG is configured to Q&A mode, ESM operates in TMS570 mode.
 *
 * @param lockStepRstEn Configuration for detected MCU lock-step error. If set
 * to false, the PMIC transitions from ACTIVE state to SAFE state when
 * ERROR_PIN_FAIL bit is set to 1. If set to true, the PMIC transitions from
 * ACTIVE to RESET state when the ERROR_PIN_FAIL bit is set to 1.
 *
 * @param mcuErrCntThr MCU error count threshold. When MCU_ERR_CNT is greater
 * than MCU_ERR_CNT_TH, the ERROR_PIN_FAIL flag is set to 1. This setting is
 * only effective when WDG is in Q&A mode. Setting a value of 0 sets the
 * threshold value to 7.
 *
 * @param lmin Minimum low-phase duration of the signal at the ERROR/WDI pin.
 *
 * @param lmax Maximum low-phase duration of the signal at the ERROR/WDI pin.
 *
 * @param hmin Minimum high-phase duration of the signal at the ERROR/WDI pin.
 *
 * @param hmax Maximum high-phase duration of the signal at the ERROR/WDI pin.
 */
typedef struct Pmic_EsmCfg_s
{
    uint32_t validParams;

    bool enable;
    bool pwmEn;
    bool lockStepRstEn;

    uint8_t mcuErrCntThr;
    uint8_t lmin;
    uint8_t lmax;
    uint8_t hmin;
    uint8_t hmax;
} Pmic_EsmCfg_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Set PMIC ESM configurations.
 *
 * @details The following options are configurable via this API
 * 1. ESM enable (validParam: PMIC_ESM_ENABLE_VALID)
 * 2. PWM enable (validParam: PMIC_ESM_PWM_EN_VALID)
 * 3. Lock Step Reset Enable (validParam: PMIC_ESM_LOCK_STEP_RST_EN_VALID)
 * 4. MCU Error Count Threshold (validParam: PMIC_ESM_MCU_ERR_CNT_THR_VALID)
 * 5. LMIN (validParam: PMIC_ESM_LMIN_VALID)
 * 6. LMAX (validParam: PMIC_ESM_LMAX_VALID)
 * 7. HMIN (validParam: PMIC_ESM_HMIN_VALID)
 * 8. HMAX (validParam: PMIC_ESM_HMAX_VALID)
 *
 * @attention In order to set ESM configurations, the PMIC must be in Diagnostic
 * mode and configuration registers must be unlocked. To unlock configuration
 * registers, see Pmic_setRegLock() or Pmic_unlockRegs().
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
 * @brief Get PMIC ESM configurations. This API supports obtaining the same
 * parameters that are settable through Pmic_esmSetCfg().
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param esmCfg [OUT] ESM configurations obtained from the PMIC.
 *
 * @return Success code if ESM configurations have been obtained from the PMIC,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmGetCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg);

/**
 * @brief Get PMIC ESM MCU error count.
 *
 * @details Bad events detected by the ESM increment the counter by 2; good
 * events detected by the ESM decrement the counter by 1.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param mcuErrCnt [OUT] MCU error count value obtained from the PMIC.
 *
 * @return Success code if the MCU error count has been obtained from the PMIC,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_esmGetMcuErrCnt(const Pmic_CoreHandle_t *pmicHandle, uint8_t *mcuErrCnt);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_ESM_H__ */
