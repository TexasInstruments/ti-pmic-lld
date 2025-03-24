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
#ifndef __PMIC_TIMER_H__
#define __PMIC_TIMER_H__

/**
 * @file pmic_timer.h
 * @brief PMIC Driver Low IQ Timer API/Interface
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
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * @anchor Pmic_TimerCfgValidParam
 * @name PMIC Timer Configuration Valid Params
 *
 * @brief Valid parameters of the timer configuration structure (Pmic_TimerCfg_t).
 *
 * @{
 */
#define PMIC_CFG_TMR_PRESCALE_VALID (0U)
#define PMIC_CFG_TMR_MODE_VALID     (1U)
/** @} */

/**
 * @anchor Pmic_TimerCfgValidParamShift
 * @name PMIC Timer Configuration Valid Param Shifts
 *
 * @brief Valid parameter shifts of the timer configuration structure
 * (Pmic_TimerCfg_t). End-user can set validParams to be equal to a combination
 * of the defines listed below to indicate which parameters are valid.
 *
 * @{
 */
#define PMIC_CFG_TMR_PRESCALE_VALID_SHIFT (1U << PMIC_CFG_TMR_PRESCALE_VALID)
#define PMIC_CFG_TMR_MODE_VALID_SHIFT     (1U << PMIC_CFG_TMR_MODE_VALID)
/** @} */

/**
 * @anchor Pmic_TimerPrescale
 * @name PMIC Timer Prescale
 *
 * @brief Possible values in which the timer prescale can be configured to.
 *
 * @note The "P" in the definition name denotes a decimal point.
 *
 * @{
 */
#define PMIC_TMR_PRESCALE_64P64_US      (0U)
#define PMIC_TMR_PRESCALE_16P384_MS     (1U)
#define PMIC_TMR_PRESCALE_131P072_MS    (2U)
#define PMIC_TMR_PRESCALE_1049_MS       (3U)
#define PMIC_TMR_PRESCALE_MAX           (PMIC_TMR_PRESCALE_1049_MS)
/** @} */

/**
 * @anchor Pmic_TimerMode
 * @name PMIC Timer Mode
 *
 * @brief Different modes of operation for the timer. The timer increments its
 * counter in each mode with the exception of PMIC_TMR_MODE_STOPPED.
 *
 * @details Operating modes include:
 * 1. STOPPED - Timer is stopped.
 * 2. OPER_SEQ - Timer counts in operating and sequencing states.
 * 3. STDBY - Timer counts only in STANDBY state.
 * 4. STBY_WU - Timer counts only in STANDBY state and has wake-up from STANDBY.
 * 5. OPER_SEQ_STBY - Timer counts in operating, sequencing, and STANDBY states.
 * 6. OPER_SEQ_STDBY_WU - Timer counts in operating, sequencing, and STANDBY states; has wake-up from STANDBY.
 *
 * @{
 */
#define PMIC_TMR_MODE_STOPPED               (0U)
#define PMIC_TMR_MODE_OPER_SEQ              (1U)
#define PMIC_TMR_MODE_STDBY                 (2U)
#define PMIC_TMR_MODE_STDBY_WU              (3U)
#define PMIC_TMR_MODE_OPER_SEQ_STDBY        (4U)
#define PMIC_TMR_MODE_OPER_SEQ_STDBY_WU     (5U)
#define PMIC_TMR_MODE_MAX                   (PMIC_TMR_MODE_OPER_SEQ_STDBY_WU)
/** @} */

/**
 * @anchor Pmic_TimerCntWakeupMaxVal
 * @name PMIC Timer Counter and Wakeup Maximum Values
 *
 * @brief Maximum values of the timer counter and wakeup value.
 *
 * @{
 */
#define PMIC_TMR_CNT_MAX                    ((uint32_t)0xFFFFFFU)
#define PMIC_TMR_WAKEUP_VAL_MAX             ((uint32_t)0xFFFFFFU)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_TimerCfg
 * @name PMIC Timer Configuration Struct
 *
 * @brief Struct used to set and get timer configurations.
 *
 * @param validParams For valid values, see @ref Pmic_TimerCfgValidParamShift
 *
 * @param prescale Timer prescale selection for the counter time base. For valid
 * values, see @ref Pmic_TimerPrescale
 *
 * @param mode Timer mode of operation. For vlaid values, see @ref Pmic_TimerMode.
 *
 * @{
 */
typedef struct Pmic_timerCfg_e {
    uint32_t validParams;

    uint8_t prescale;
    uint8_t mode;
} Pmic_timerCfg_t;
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Set PMIC timer configuration.
 *
 * @details Timer configurations that are able to be set via this API are
 * 1. Prescale (validParam: PMIC_CFG_TMR_PRESCALE_VALID_SHIFT)
 * 2. Mode (PMIC_CFG_TMR_MODE_VALID_SHIFT)
 *
 * @attention The timer prescale configuration must not be changed when the
 * timer is enabled.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param timerCfg [IN] PMIC timer configurations to be set.
 *
 * @return Success code if timer configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_timerSetCfg(Pmic_CoreHandle_t *handle, const Pmic_timerCfg_t *timerCfg);

/**
 * @brief Get PMIC timer configuration. This API supports getting the same
 * configurations that are supported by Pmic_timerSetCfg().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param timerCfg [OUT] Timer configurations obtained from the PMIC.
 *
 * @return Success code if timer configurations have been obtained from the
 * PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes
 */
int32_t Pmic_timerGetCfg(Pmic_CoreHandle_t *handle, Pmic_timerCfg_t *timerCfg);

/**
 * @brief Stop PMIC timer. This API is a subset of Pmic_timerSetCfg(). That is
 * to say, the Pmic_timerSetCfg() API can be used alternatively to stop the PMIC
 * timer.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if timer is stopped, error code otherwise. For valid
 * success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_timerStop(Pmic_CoreHandle_t *handle);

/**
 * @brief Clear PMIC timer counter.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC timer counter is cleared, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_timerClr(Pmic_CoreHandle_t *handle);

/**
 * @brief Set the value of the PMIC timer counter.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param tmrCnt [IN] Desired timer counter value. Must be less than or equal
 * to PMIC_TMR_CNT_MAX found in @ref Pmic_TimerCntWakeupMaxVal
 *
 * @return Success code if PMIC timer counter value has been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_timerSetCnt(Pmic_CoreHandle_t *handle, uint32_t tmrCnt);

/**
 * @brief Get the value of the PMIC timer counter.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param tmrCnt [OUT] Timer counter value obtained from the PMIC.
 *
 * @return Success code if PMIC timer counter value has been obtained from the
 * PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes
 */
int32_t Pmic_timerGetCnt(Pmic_CoreHandle_t *handle, uint32_t *tmrCnt);

/**
 * @brief Set PMIC wakeup value.
 *
 * @details The timer can be configured to generate a wakeup event based on a
 * configured time while the device is in STANDBY state - see Pmic_timerSetCfg()
 * for configuring the timer. As such, the wakeup value is only used during
 * certain timer modes.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wakeupVal [IN] Desired wakeup value.
 *
 * @return Success code if PMIC wakeup value has been set, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_timerSetWakeupVal(Pmic_CoreHandle_t *handle, uint32_t wakeupVal);

/**
 * @brief Get PMIC wakeup value.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wakeupVal [OUT] Wakeup value obtained from the PMIC.
 *
 * @return Success code if PMIC wakeup value has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_timerGetWakeupVal(Pmic_CoreHandle_t *handle, uint32_t *wakeupVal);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_TIMER_H__ */
