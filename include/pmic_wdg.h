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
 * @file pmic_wdg.h
 *
 * @brief PMIC LLD Watchdog module header file.
 *
 * @details This module contains declarations/definitions of macros, data
 * structures, and APIs used to interact with the PMIC watchdog. Some
 * components of the PMIC watchdog module are as follows: setting/getting
 * watchdog configurations, getting watchdog error status, getting watchdog
 * fail counter status, and sending Q&A answers.
 */
#ifndef __PMIC_WDG_H__
#define __PMIC_WDG_H__

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
 * @anchor Pmic_WdgCfgValidParams
 * @name TPS65036x Watchdog Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_WdgCfg_t
 * struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_WdgCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_WD_RST_EN_VALID                    ((uint32_t)(1U << 0U))
#define PMIC_WD_MODE_VALID                      ((uint32_t)(1U << 1U))
#define PMIC_WD_TRIG_SEL_VALID                  ((uint32_t)(1U << 2U))
#define PMIC_WD_FAIL_THR_VALID                  ((uint32_t)(1U << 3U))
#define PMIC_WD_RST_THR_VALID                   ((uint32_t)(1U << 4U))
#define PMIC_WD_LONG_WIN_DURATION_VALID         ((uint32_t)(1U << 5U))
#define PMIC_WD_WIN1_DURATION_VALID             ((uint32_t)(1U << 6U))
#define PMIC_WD_WIN2_DURATION_VALID             ((uint32_t)(1U << 7U))
#define PMIC_WD_QA_FDBK_VALID                   ((uint32_t)(1U << 8U))
#define PMIC_WD_QA_LFSR_VALID                   ((uint32_t)(1U << 9U))
#define PMIC_WD_QA_SEED_VALID                   ((uint32_t)(1U << 10U))
/** @} */

/**
 * @anchor Pmic_WdgErrStatValidParams
 * @name TPS65036x Error Status Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_WdgErrStat_t
 * struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_WdgErrStat.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_RST_INT_VALID                      ((uint32_t)(1U << 0U))
#define PMIC_FAIL_INT_VALID                     ((uint32_t)(1U << 1U))
#define PMIC_ANSW_ERR_VALID                     ((uint32_t)(1U << 2U))
#define PMIC_SEQ_ERR_VALID                      ((uint32_t)(1U << 3U))
#define PMIC_ANSW_EARLY_ERR_VALID               ((uint32_t)(1U << 4U))
#define PMIC_TIMEOUT_ERR_VALID                  ((uint32_t)(1U << 5U))
#define PMIC_LONGWIN_TIMEOUT_INT_VALID          ((uint32_t)(1U << 6U))
#define PMIC_WDG_ERR_STAT_VALID_ALL             (PMIC_RST_INT_VALID        | \
                                                PMIC_FAIL_INT_VALID        | \
                                                PMIC_ANSW_ERR_VALID        | \
                                                PMIC_SEQ_ERR_VALID         | \
                                                PMIC_ANSW_EARLY_ERR_VALID  | \
                                                PMIC_TIMEOUT_ERR_VALID     | \
                                                PMIC_LONGWIN_TIMEOUT_INT_VALID)
/** @} */

/**
 * @anchor Pmic_WdgFailCntStatValidParams
 * @name TPS65036x Watchdog Fail Count Status Valid Parameters
 *
 * @brief Indication of which parameters are valid within the
 * Pmic_WdgFailCntStat_t struct.
 *
 * @details For more information on the parameters, refer to
 * @ref Pmic_WdgFailCntStat.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_BAD_EVENT_VALID                    ((uint32_t)(1U << 0U))
#define PMIC_GOOD_EVENT_VALID                   ((uint32_t)(1U << 1U))
#define PMIC_FAIL_CNT_VALID                     ((uint32_t)(1U << 2U))
/** @} */

/**
 * @anchor Pmic_wdgWindowMaxValues
 * @name TPS65036x Watchdog Window Max Values
 *
 * @brief Maximum values of Window-1, Window-2, and Long Window.
 *
 * @{
 */
#define PMIC_WD_WIN1_DURATION_MAX               ((uint8_t)0x7FU)
#define PMIC_WD_WIN2_DURATION_MAX               ((uint8_t)0x7FU)
#define PMIC_WD_LONG_WIN_DURATION_MAX           ((uint8_t)0xFFU)
/** @} */

/**
 * @anchor Pmic_wdgThrMaxValues
 * @name TPS65036x Watchdog Threshold Max Values
 *
 * @brief Maximum values of the watchdog Fail and Reset thresholds.
 *
 * @{
 */
#define PMIC_WD_FAIL_THR_MAX                    ((uint8_t)7U)
#define PMIC_WD_RST_THR_MAX                     ((uint8_t)7U)
/** @} */

/**
 * @anchor Pmic_wdgQaCfgMaxValues
 * @name TPS65036x Watchdog Q&A Configuration Max Values
 *
 * @brief Maximum values of WD_QA_FDBK, WD_QA_LFSR, and WD_QUESTION_SEED.
 *
 * @{
 */
#define PMIC_WD_QA_FDBK_MAX                     ((uint8_t)3U)
#define PMIC_WD_QA_LFSR_MAX                     ((uint8_t)3U)
#define PMIC_WD_QA_SEED_MAX                     ((uint8_t)0xFU)
/** @} */

/**
 * @anchor Pmic_wdTriggerSel
 * @name TPS65036x WD_TRIGGER_SEL Values
 *
 * @brief Valid values of the PMIC WD_TRIGGER_SEL bit field.
 *
 * @{
 */
#define PMIC_SW_TRIGGER                         ((uint8_t)0U)
#define PMIC_HW_TRIGGER                         ((uint8_t)1U)
#define PMIC_TRIG_SEL_MAX                       (PMIC_HW_TRIGGER)
/** @} */

/**
 * @anchor Pmic_wdModeSelect
 * @name TPS65036x WD_MODE_SELECT Values
 *
 * @brief Valid values of the PMIC WD_MODE_SELECT bit field.
 *
 * @{
 */
#define PMIC_TRIGGER_MODE                       ((uint8_t)0U)
#define PMIC_QA_MODE                            ((uint8_t)1U)
#define PMIC_WD_MODE_MAX                        (PMIC_QA_MODE)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_WdgCfg
 * @name TPS65036x Watchdog Configuration struct
 *
 * @brief Struct used to write/read PMIC watchdog configurations.
 *
 * @attention In order to set watchdog configurations, the watchdog must be
 * enabled and in the Long Window. Please see `Pmic_wdgSetRetLongWin()` for
 * how to return to Long Window and `Pmic_wdgEnable()` for enabling watchdog.
 *
 * @param validParams Each bit in this variable represents whether a struct member
 * is valid. for valid values, refer to @ref Pmic_WdgCfgValidParams.
 *
 * @param rstEn Watchdog warm reset enable. When set to true, the watchdog
 * triggers a warm reset when WD_FAIL_CNT is greater than (WD_FAIL_TH + WD_RST_TH).
 *
 * @param mode Watchdog mode configuration. For valid values, refer to
 * @ref Pmic_wdModeSelect.
 *
 * @param trigSel Watchdog trigger source for when the watchdog is operating
 * in Trigger mode. For valid values, refer to @ref Pmic_wdTriggerSel.
 *
 * @param failThr Watchdog configuration for first threshold of the watchdog
 * fail counter. For max value, refer to @ref Pmic_wdgThrMaxValues.
 *
 * @param rstThr Watchdog configuration for second threshold of the watchdog
 * fail counter. For max value, refer to @ref Pmic_wdgThrMaxValues.
 *
 * @param longWinDuration Watchdog Long Window duration. For max value, refer
 * to @ref Pmic_wdgWindowMaxValues.
 *
 * @param win1 Watchdog Window-1 duration. For max value, refer to
 * @ref Pmic_wdgWindowMaxValues.
 *
 * @param win2 Watchdog Window-2 duration. for max value, refer to
 * @ref Pmic_wdgWindowMaxValues.
 *
 * @param qaFdbk Watchdog Q&A feedback configuration. Controls the sequence of
 * generated questions and their respective reference answers. For max value,
 * refer to @ref Pmic_wdgQaCfgMaxValues.
 *
 * @param qaLfsr Watchdog Q&A LFSR configuration used to generate the questions.
 * For max value, refer to @ref Pmic_wdgQaCfgMaxValues.
 *
 * @param qaSeed Watchdog Q&A seed used to generate a new starting question.
 * For max value, refer to @ref Pmic_wdgQaCfgMaxValues.
 */
typedef struct Pmic_WdgCfg_s
{
    uint32_t validParams;

    bool rstEn;
    uint8_t mode;
    uint8_t trigSel;

    uint8_t failThr;
    uint8_t rstThr;

    uint8_t longWinDuration;
    uint8_t win1Duration;
    uint8_t win2Duration;

    uint8_t qaFdbk;
    uint8_t qaLfsr;
    uint8_t qaSeed;
} Pmic_WdgCfg_t;

/**
 * @anchor Pmic_WdgErrStat
 * @name TPS65036x Watchdog Error Status Struct
 *
 * @brief Struct used to get the PMIC watchdog error statuses.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. For valid values, refer to @ref Pmic_WdgErrStatValidParams.
 *
 * @param rstInt Status/indication of whether the PMIC underwent WARM_RESET due
 * to WD_FAIL_CNT[3:0] > WD_RST_TH[2:0].
 *
 * @param failInt Status/indication of whether WD_FAIL_CNT[3:0] > WD_FAIL_TH[2:0].
 *
 * @param answErr Status/indication of whether the watchdog has detected an
 * incorrect answer-byte.
 *
 * @param seqErr Status/indication of whether the watchdog has detected an
 * incorrect sequence of answer-bytes.
 *
 * @param answEarlyErr Status/indication of whether watchdog has received the
 * final answer-byte in Window-1.
 *
 * @param timeoutErr Status/indication of whether the watchdog has detected a
 * timeout event during a watchdog sequence.
 *
 * @param longWinTimeoutInt Status/indication of whether the PMIC has undergone
 * warm reset due to elapse of Long Window.
 */
typedef struct Pmic_WdgErrStat_s
{
    uint32_t validParams;

    bool rstInt;
    bool failInt;
    bool answErr;
    bool seqErr;
    bool answEarlyErr;
    bool timeoutErr;
    bool longWinTimeoutInt;
} Pmic_WdgErrStat_t;

/**
 * @anchor Pmic_WdgFailCntStat
 * @name TPS65036x Watchdog Fail Count Status Struct
 *
 * @brief Struct used to get the PMIC watchdog fail count status.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. For valid values, refer to @ref Pmic_WdgFailCntStatValidParams.
 *
 * @param badEvent Indication of whether a bad event has been detected in the
 * current watchdog sequence.
 *
 * @param goodEvent Indication of whether the PMIC watchdog has detected a good
 * event.
 *
 * @param failCnt Current value of the watchdog fail counter.
 */
typedef struct Pmic_WdgFailCntStat_s
{
    uint32_t validParams;

    bool badEvent;
    bool goodEvent;

    uint8_t failCnt;
} Pmic_WdgFailCntStat_t;

/*==========================================================================  */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Enable or disable the PMIC watchdog. End-user could utilize this API
 * as an alternative to `Pmic_wdgEnable()` and `Pmic_wdgDisable()`.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param enable [IN] When set to `PMIC_ENABLE`, the watchdog is enabled.
 * When set to `PMIC_DISABLE`, the watchdog is disabled.
 *
 * @return Success code if the PMIC watchdog has been enabled/disabled, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgSetEnableState(const Pmic_CoreHandle_t *pmicHandle, bool enable);

/**
 * @brief Enable the PMIC watchdog. End-user should call this API before
 * setting watchdog configurations.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if the PMIC watchdog has been enabled, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgEnable(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Disable the PMIC watchdog.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if the PMIC watchdog has been disabled, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgDisable(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Get the enable status of the PMIC watchdog.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgEnabled [OUT] Watchdog enable status. Value is set to true if PMIC
 * watchdog is enabled, else the value is set to false.
 *
 * @return Success code if the PMIC watchdog enable status has been obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetEnable(const Pmic_CoreHandle_t *pmicHandle, bool *wdgEnabled);

/**
 * @brief Set PMIC watchdog configurations.
 *
 * @details The following options are configurable via this API
 * 1. Reset enable
 * 2. Mode of operation
 * 3. Trigger select
 * 4. Fail threshold
 * 5. Reset threshold
 * 6. Long Window duration
 * 7. Window-1 duration
 * 8. Window-2 duration
 * 9. Q&A Feedback
 * 10. Q&A LFSR
 * 11. Q&A question seed
 * For more information on watchdog configurations, refer to @ref Pmic_WdgCfg.
 *
 * @attention Watchdog must be in Long Window and enabled before configuration.
 * See `Pmic_wdgEnable()` and `Pmic_wdgSetRetLongWin()` for more information.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgCfg [IN] Watchdog configurations to write to PMIC.
 *
 * @return Success code if PMIC watchdog configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgSetCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgCfg_t *wdgCfg);

/**
 * @brief Get PMIC watchdog configurations.
 *
 * @details The following configurations are obtainable from this API
 * 1. Reset enable
 * 2. Mode of operation
 * 3. Trigger select
 * 4. Fail threshold
 * 5. Reset threshold
 * 6. Long Window duration
 * 7. Window-1 duration
 * 8. Window-2 duration
 * 9. Q&A Feedback
 * 10. Q&A LFSR
 * 11. Q&A question seed
 * For more information on watchdog configurations, refer to @ref Pmic_WdgCfg.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgCfg [OUT] Watchdog configurations obtained from PMIC.
 *
 * @return Success code if PMIC watchdog configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgCfg_t *wdgCfg);

/**
 * @brief Set the PMIC WD_PWRHOLD bit.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param pwrHold [IN] When this parameter is set to true, WD_PWRHOLD will be
 * set to 1 (making the watchdog stay in Long Window). Otherwise, WD_PWRHOLD
 * will be set to 0 (allowing the watchdog to exit Long Window).
 *
 * @return Success code the WD_PWRHOLD bit is set, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgSetPwrHold(const Pmic_CoreHandle_t *pmicHandle, bool pwrHold);

/**
 * @brief Get the status of the PMIC WD_PWRHOLD bit.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param pwrHoldStat [OUT] Status of WD_PWRHOLD. If value is true, WD_PWRHOLD
 * is 1, else WD_PWRHOLD is 0.
 *
 * @return Success code if the status of WD_PWRHOLD has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetPwrHold(const Pmic_CoreHandle_t *pmicHandle, bool *pwrHoldStat);

/**
 * @brief Set the PMIC WD_RETURN_LONGWIN bit.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param retLongWin [IN] When this parameter is set to true, WD_RETURN_LONGWIN will
 * be set to 1 (enabling the watchdog to return to Long Window after completion of
 * the current sequence). Otherwise, WD_RETURN_LONGWIN will be set to 0 (enabling
 * the watchdog to continue sequences after the current sequence).
 *
 * @return Success code if WD_RETURN_LONGWIN bit is set, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgSetRetLongWin(const Pmic_CoreHandle_t *pmicHandle, bool retLongWin);

/**
 * @brief Get the status of the PMIC WD_RETURN_LONGWIN bit.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param retLongWinStat [OUT] Status of WD_RETURN_LONGWIN. If value is true,
 * WD_RETURN_LONGWIN is 1, else WD_RETURN_LONGWIN is 0.
 *
 * @return success code if the status of WD_RETURN_LONGWIN has been obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetRetLongWin(const Pmic_CoreHandle_t *pmicHandle, bool *retLongWinStat);

/**
 * @brief Send a software trigger to the PMIC watchdog.
 *
 * @details When the watchdog is operating in Trigger mode with Software Trigger
 * as the trigger source, the API should be called once during Long Window to
 * exit Long Window. For every trigger sequence thereafter, the API should be
 * called once in Window-2.
 *
 * @attention For this API, watchdog must be enabled, configured to Trigger
 * mode, and have Software Trigger as the trigger source. To enable the
 * watchdog, see `Pmic_wdgEnable()`. To configure the mode and the trigger
 * source, see `Pmic_wdgSetCfg()`.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if software trigger has been sent, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgSendSwTrigger(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Send a Q&A answer byte to the PMIC watchdog.
 *
 * @details When the watchdog is operating in Q&A mode, the API should be called
 * four times in Long Window to exit Long Window. For every Q&A sequence thereafter,
 * the API should be called three times in Window-1 and one time in Window-2.
 *
 * @attention For this API, watchdog must be enabled and configured to Q&A mode.
 * To enable watchdog, see `Pmic_wdgEnable()`. To configure the mode, see
 * `Pmic_wdgSetCfg()`.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if Q&A answer byte has been sent to the PMIC, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgWriteAnswer(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Clear all PMIC watchdog error statuses.
 *
 * @details The following watchdog error statuses are cleared by this API
 * 1. WD_RST_INT
 * 2. WD_FAIL_INT
 * 3. WD_ANSW_ERR
 * 4. WD_SEQ_ERR
 * 5. WD_ANSW_EARLY
 * 6. WD_TIMEOUT
 * 7. WD_LONGWIN_TIMEOUT_INT
 * For more information on the watchdog error statuses, refer to
 * @ref Pmic_WdgErrStat.
 *
 * @attention If end-user calls this API without addressing the root cause of the
 * watchdog error statuses, the statuses could continue to be set after API call.
 * Additionally, it may be required to enable watchdog by calling `Pmic_wdgEnable()`
 * before clearing the error statuses.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if all watchdog error statuses have been cleared, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgClrErrStatAll(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Clear PMIC watchdog error statuses.
 *
 * @details The following watchdog error statuses can be cleared by this API
 * 1. WD_RST_INT
 * 2. WD_FAIL_INT
 * 3. WD_ANSW_ERR
 * 4. WD_SEQ_ERR
 * 5. WD_ANSW_EARLY
 * 6. WD_TIMEOUT
 * 7. WD_LONGWIN_TIMEOUT_INT
 * For more information on the watchdog error statuses, refer to
 * @ref Pmic_WdgErrStat.
 *
 * @attention If end-user calls this API without addressing the root cause of the
 * watchdog error statuses, the statuses could continue to be set after API call.
 * Additionally, it may be required to enable watchdog by calling `Pmic_wdgEnable()`
 * before clearing the error status(es).
 *
 * @note To indicate the desired watchdog error status(es) to clear, the
 * validParams struct member of \p wdgErrStat parameter must be set. All other
 * struct members will be ignored/unused throughout API execution. For valid
 * values of validParams, refer to @ref Pmic_WdgErrStatValidParams.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgErrStat [IN] The validParams struct member of this parameter indicates
 * which watchdog error status(es) to clear.
 *
 * @return Success code if watchdog error status(es) have been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes
 */
int32_t Pmic_wdgClrErrStat(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgErrStat_t *wdgErrStat);

/**
 * @brief Get PMIC watchdog error statuses.
 *
 * @details The following watchdog error statuses can be obtained from this API
 * 1. WD_RST_INT
 * 2. WD_FAIL_INT
 * 3. WD_ANSW_ERR
 * 4. WD_SEQ_ERR
 * 5. WD_ANSW_EARLY
 * 6. WD_TIMEOUT
 * 7. WD_LONGWIN_TIMEOUT_INT
 * For more information on the watchdog error statuses, refer to
 * @ref Pmic_WdgErrStat.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgErrStat [OUT] Struct containing watchdog error statuses of the PMIC.
 *
 * @return Success code if watchdog error status(es) have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetErrStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgErrStat_t *wdgErrStat);

/**
 * @brief Get PMIC watchdog fail counter statuses.
 *
 * @details The following watchdog fail counter statuses can be obtained from this API
 * 1. WD_BAD_EVENT
 * 2. WD_FIRST_OK
 * 3. WD_FAIL_CNT
 * For more information on the watchdog fail counter statuses, refer to
 * @ref Pmic_WdgFailCntStat.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgFailCntStat [OUT] Struct containing the watchdog fail counter
 * statuses of the PMIC.
 *
 * @return Success code if the watchdog fail counter status(es) have been
 * obtained, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetFailCntStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgFailCntStat_t *wdgFailCntStat);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_WDG_H__ */
