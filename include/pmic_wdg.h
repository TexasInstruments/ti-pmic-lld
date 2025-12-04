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
#ifndef PMIC_WDG_H
#define PMIC_WDG_H

/**
 * @file pmic_io.h
 *
 * @brief PMIC watchdog (WDG) interface. Contains APIs, macros/defines, and data
 * structures used to configure, control, and interact with the PMIC WDG.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_WdgThresholdCount
 * @name PMIC Watchdog Timer Threshold Configurations
 *
 * @{
 */
#define PMIC_WDG_THR_CNT_0   (0U)
#define PMIC_WDG_THR_CNT_1   (1U)
#define PMIC_WDG_THR_CNT_2   (2U)
#define PMIC_WDG_THR_CNT_3   (3U)
#define PMIC_WDG_THR_CNT_4   (4U)
#define PMIC_WDG_THR_CNT_5   (5U)
#define PMIC_WDG_THR_CNT_6   (6U)
#define PMIC_WDG_THR_CNT_7   (7U)
#define PMIC_WDG_THR_CNT_MIN (PMIC_WDG_THR_CNT_0)
#define PMIC_WDG_THR_CNT_MAX (PMIC_WDG_THR_CNT_7)
/** @} */

/**
 * @anchor Pmic_WdgQaFdbkVal
 * @name PMIC Watchdog Timer Q&A Feedback Values
 *
 * @{
 */
#define PMIC_WDG_QA_FDBK_VAL_0   (0U)
#define PMIC_WDG_QA_FDBK_VAL_1   (1U)
#define PMIC_WDG_QA_FDBK_VAL_2   (2U)
#define PMIC_WDG_QA_FDBK_VAL_3   (3U)
#define PMIC_WDG_QA_FDBK_VAL_MIN (PMIC_WDG_QA_FDBK_VAL_0)
#define PMIC_WDG_QA_FDBK_VAL_MAX (PMIC_WDG_QA_FDBK_VAL_3)
/** @} */

/**
 * @anchor Pmic_WdgQaLfsrVal
 * @name PMIC Watchdog Timer Q&A LFSR Values
 *
 * @{
 */
#define PMIC_WDG_QA_LFSR_VAL_0   (0U)
#define PMIC_WDG_QA_LFSR_VAL_1   (1U)
#define PMIC_WDG_QA_LFSR_VAL_2   (2U)
#define PMIC_WDG_QA_LFSR_VAL_3   (3U)
#define PMIC_WDG_QA_LFSR_VAL_MIN (PMIC_WDG_QA_LFSR_VAL_0)
#define PMIC_WDG_QA_LFSR_VAL_MAX (PMIC_WDG_QA_LFSR_VAL_3)
/** @} */

/**
 * @anchor Pmic_WdgQaQuestionSeedVal
 * @name PMIC Watchdog Timer Q&A Question Seed Values
 *
 * @{
 */
#define PMIC_WDG_QA_SEED_VAL_0   (0U)
#define PMIC_WDG_QA_SEED_VAL_1   (1U)
#define PMIC_WDG_QA_SEED_VAL_2   (2U)
#define PMIC_WDG_QA_SEED_VAL_3   (3U)
#define PMIC_WDG_QA_SEED_VAL_4   (4U)
#define PMIC_WDG_QA_SEED_VAL_5   (5U)
#define PMIC_WDG_QA_SEED_VAL_6   (6U)
#define PMIC_WDG_QA_SEED_VAL_7   (7U)
#define PMIC_WDG_QA_SEED_VAL_8   (8U)
#define PMIC_WDG_QA_SEED_VAL_9   (9U)
#define PMIC_WDG_QA_SEED_VAL_10  (10U)
#define PMIC_WDG_QA_SEED_VAL_11  (11U)
#define PMIC_WDG_QA_SEED_VAL_12  (12U)
#define PMIC_WDG_QA_SEED_VAL_13  (13U)
#define PMIC_WDG_QA_SEED_VAL_14  (14U)
#define PMIC_WDG_QA_SEED_VAL_15  (15U)
#define PMIC_WDG_QA_SEED_VAL_MIN (PMIC_WDG_QA_SEED_VAL_0)
#define PMIC_WDG_QA_SEED_VAL_MAX (PMIC_WDG_QA_SEED_VAL_15)
/** @} */

/**
 * @anchor Pmic_wdgWindowMaxValues
 * @name PMIC Watchdog Window Max Values
 *
 * @brief Maximum values of Window-1, Window-2, and Long Window.
 *
 * @{
 */
#define PMIC_WDG_WIN_CODE_MAX      (0x7FU)
#define PMIC_WDG_LONG_WIN_CODE_MAX (0xFFU)
/** @} */

/**
 * @anchor Pmic_WdgCfgValidParams
 * @name PMIC Watchdog Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_WdgCfg_t`.
 * Set the `validParams` member of `Pmic_WdgCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_WDG_RST_EN_VALID        (1U << 0U)
#define PMIC_WDG_FAIL_THR_VALID      (1U << 1U)
#define PMIC_WDG_RST_THR_VALID       (1U << 2U)
#define PMIC_WDG_LONG_WIN_CODE_VALID (1U << 3U)
#define PMIC_WDG_WIN1_CODE_VALID     (1U << 4U)
#define PMIC_WDG_WIN2_CODE_VALID     (1U << 5U)
#define PMIC_WDG_QA_FDBK_VALID       (1U << 6U)
#define PMIC_WDG_QA_LFSR_VALID       (1U << 7U)
#define PMIC_WDG_QA_SEED_VALID       (1U << 8U)
#define PMIC_WDG_CFG_VALID_ALL       (\
    PMIC_WDG_RST_EN_VALID | \
    PMIC_WDG_FAIL_THR_VALID | \
    PMIC_WDG_RST_THR_VALID | \
    PMIC_WDG_LONG_WIN_CODE_VALID | \
    PMIC_WDG_WIN1_CODE_VALID | \
    PMIC_WDG_WIN2_CODE_VALID | \
    PMIC_WDG_QA_FDBK_VALID | \
    PMIC_WDG_QA_LFSR_VALID | \
    PMIC_WDG_QA_SEED_VALID)
/** @} */

/**
 * @anchor Pmic_WdgErrStatusValidParams
 * @name PMIC Watchdog Error Status Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_WdgErrStatus_t`.
 * Set the `validParams` member of `Pmic_WdgErrStatus_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_WDG_RST_INT_VALID              (1U << 0U)
#define PMIC_WDG_FAIL_INT_VALID             (1U << 1U)
#define PMIC_WDG_ANSW_ERR_VALID             (1U << 2U)
#define PMIC_WDG_SEQ_ERR_VALID              (1U << 3U)
#define PMIC_WDG_ANSW_EARLY_ERR_VALID       (1U << 4U)
#define PMIC_WDG_TIMEOUT_ERR_VALID          (1U << 5U)
#define PMIC_WDG_LONG_WIN_TIMEOUT_INT_VALID (1U << 6U)
#define PMIC_WDG_ERR_STATUS_VALID_ALL       (\
    PMIC_WDG_RST_INT_VALID | \
    PMIC_WDG_FAIL_INT_VALID | \
    PMIC_WDG_ANSW_ERR_VALID | \
    PMIC_WDG_SEQ_ERR_VALID | \
    PMIC_WDG_ANSW_EARLY_ERR_VALID | \
    PMIC_WDG_TIMEOUT_ERR_VALID | \
    PMIC_WDG_LONG_WIN_TIMEOUT_INT_VALID)
/** @} */

/**
 * @anchor Pmic_WdgFailCntStatusValidParams
 * @name PMIC Watchdog Fail Count Status Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_WdgFailCntStatus_t`.
 * Set the `validParams` member of `Pmic_WdgFailCntStatus_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_WDG_BAD_EVENT_VALID           (1U << 0U)
#define PMIC_WDG_GOOD_EVENT_VALID          (1U << 1U)
#define PMIC_WDG_FAIL_CNT_VALID            (1U << 2U)
#define PMIC_WDG_FAIL_CNT_STATUS_VALID_ALL (\
    PMIC_WDG_BAD_EVENT_VALID | \
    PMIC_WDG_GOOD_EVENT_VALID | \
    PMIC_WDG_FAIL_CNT_VALID)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 * @anchor Pmic_WdgCfg
 * @name Watchdog Configuration Structure
 *
 * @brief Structure used to set/get PMIC watchdog configurations.
 *
 * @attention In order to set watchdog configurations, the watchdog must be
 * enabled and in the Long Window. Please see `Pmic_wdgSetReturnToLongWindow()` for
 * how to return to Long Window and `Pmic_wdgEnable()` for enabling watchdog.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_WdgCfgValidParams.
 *
 * @param rstEn Watchdog warm reset enable. When set to true, the watchdog
 * triggers a warm reset when WD_FAIL_CNT is greater than (WD_FAIL_TH + WD_RST_TH).
 *
 * @param failThr Watchdog configuration for first threshold of the watchdog
 * fail counter. For valid values, refer to @ref Pmic_WdgThresholdCount.
 *
 * @param rstThr Watchdog configuration for second threshold of the watchdog
 * fail counter. For valid values, refer to @ref Pmic_WdgThresholdCount.
 *
 * @param longWinCode Watchdog Long Window duration code. For max value, refer
 * to @ref Pmic_wdgWindowMaxValues. See PMIC device data sheet for more information
 * on converting from code to time duration and vice versa.
 *
 * @param win1Code Watchdog Window-1 duration code. For max value, refer to
 * @ref Pmic_wdgWindowMaxValues. See PMIC device data sheet for more information
 * on converting from code to time duration and vice versa.
 *
 * @param win2Code Watchdog Window-2 duration code. For max value, refer to
 * @ref Pmic_wdgWindowMaxValues. See PMIC device data sheet for more information
 * on converting from code to time duration and vice versa.
 *
 * @param qaFdbk Watchdog Q&A feedback configuration. Controls the sequence of
 * generated questions and their respective reference answers. For valid values,
 * refer to @ref Pmic_WdgQaFdbkVal.
 *
 * @param qaLfsr Watchdog Q&A LFSR configuration used to generate the questions.
 * For valid values, refer to @ref Pmic_WdgQaLfsrVal.
 *
 * @param qaSeed Watchdog Q&A seed used to generate a new starting question.
 * For valid values, refer to @ref Pmic_WdgQaQuestionSeedVal.
 */
typedef struct Pmic_WdgCfg_s {
    uint32_t validParams;

    bool rstEn;

    uint8_t failThr;
    uint8_t rstThr;

    uint8_t longWinCode;
    uint8_t win1Code;
    uint8_t win2Code;

    uint8_t qaFdbk;
    uint8_t qaLfsr;
    uint8_t qaSeed;
} Pmic_WdgCfg_t;

/**
 * @anchor Pmic_WdgErrStatus
 * @name Watchdog Error Status Structure
 *
 * @brief Structure used to get and clear PMIC watchdog error statuses.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_WdgErrStatusValidParams.
 *
 * @param rstInt Status/indication of whether the PMIC underwent WARM_RESET due
 * to the fail counter exeeding the reset + fail threshold. That is, WD_FAIL_CNT[3:0] >
 * (WD_RST_TH[2:0] + WD_FAIL_TH[2:0]).
 *
 * @param failInt Status/indication of whether the watchdog fail counter exceeded
 * the fail threshold (WD_FAIL_CNT[3:0] > WD_FAIL_TH[2:0]).
 *
 * @param answErr Status/indication of whether the watchdog has detected an
 * incorrect answer-byte.
 *
 * @param seqErr Status/indication of whether the watchdog has detected an
 * incorrect sequence of answer-bytes.
 *
 * @param answEarlyErr Status/indication of whether watchdog has received the
 * final answer-byte in Window-1 (Q&A mode only).
 *
 * @param timeoutErr Status/indication of whether the watchdog has detected a
 * timeout event during a watchdog sequence.
 *
 * @param longWinTimeoutInt Status/indication of whether the PMIC has undergone
 * warm reset due to elapse of Long Window.
 */
typedef struct Pmic_WdgErrStatus_s
{
    uint32_t validParams;

    bool rstInt;
    bool failInt;
    bool answErr;
    bool seqErr;
    bool answEarlyErr;
    bool timeoutErr;
    bool longWinTimeoutInt;
} Pmic_WdgErrStatus_t;

/**
 * @anchor Pmic_WdgFailCntStatus
 * @name Watchdog Fail Count Status Structure
 *
 * @brief Structure used to get the PMIC watchdog fail count status.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. For valid values, refer to @ref Pmic_WdgFailCntStatusValidParams.
 *
 * @param badEvent Indication of whether a bad event has been detected in the
 * current watchdog sequence. A bad event occurs when one of the events occur:
 * (1) MCU sends correct answer-bytes, but not in correct watchdog window, (2)
 * MCU sends incorrect answer-bytes, (3) MCU returns correct answer-bytes, but
 * in the incorrect sequence.
 *
 * @param goodEvent Indication of whether the PMIC watchdog has detected a good
 * event. A good event occurs when the MCU sends the correct answer-bytes
 * calculated for the current question in the correct watchdog window and in the
 * current sequence.
 *
 * @param failCnt Current value of the watchdog fail counter. A good event
 * decrements the fail counter by one before the start of the next Window-1.
 * A bad event increments the fail counter by one before the start of the next
 * Window-1.
 */
typedef struct Pmic_WdgFailCntStatus_s
{
    uint32_t validParams;

    bool badEvent;
    bool goodEvent;

    uint8_t failCnt;
} Pmic_WdgFailCntStatus_t;

/**
 * @anchor Pmic_WdgAnsInfo
 * @name Watchdog Answer Information Structure
 *
 * @brief This structure contains the information needed to calculate WDG answer
 * bytes.
 *
 * @param fdbk WDG Q&A feedback value.
 *
 * @param answCnt WDG Q&A answer count value.
 *
 * @param question WDG Q&A question.
 */
typedef struct Pmic_WdgAnsInfo_s {
    uint8_t fdbk;
    uint8_t ansCnt;
    uint8_t question;
} Pmic_WdgAnsInfo_t;

/* ========================================================================= */
/*                          Function Declarations                            */
/* ========================================================================= */

/**
 * @brief Enable or disable the PMIC watchdog.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param enable [IN] `PMIC_ENABLE` - enable watchdog. `PMIC_DISABLE` - disable
 * watchdog.
 *
 * @return Success code if the PMIC watchdog has been enabled/disabled, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgSetEnableState(const Pmic_Handle_t *handle, bool enable);

/**
 * @brief Get the enable status of the PMIC watchdog.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgEnabled [OUT] `PMIC_ENABLE` - watchdog is enabled.
 * `PMIC_DISABLE` - watchdog is disabled.
 *
 * @return Success code if the PMIC watchdog enable status has been obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Set PMIC watchdog configurations.
 *
 * @attention Watchdog must be in Long Window and enabled before configuration.
 * See `Pmic_wdgSetEnableState()` and `Pmic_wdgSetReturnToLongWindow()` for more
 * information.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgCfg [IN] Watchdog configurations to write to PMIC. For more information
 * on watchdog configurations, refer to @ref Pmic_WdgCfg.
 *
 * @return Success code if PMIC watchdog configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgSetCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg);

/**
 * @brief Get PMIC watchdog configurations. This API supports getting the same
 * configurations that are settable by `Pmic_wdgSetCfg()`.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgCfg [OUT] Watchdog configurations obtained from PMIC. For more information
 * on watchdog configurations, refer to @ref Pmic_WdgCfg.
 *
 * @return Success code if PMIC watchdog configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg);

/**
 * @brief Enable or disable watchdog power hold, which controls whether the
 * watchdog stays in Long Window.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param enable [IN] `PMIC_ENABLE` - enable watchdog power hold (watchdog will
 * stay in Long Window). `PMIC_DISABLE` - disable watchdog power hold (watchdog
 * can exit Long Window).
 *
 * @return Success code the WD_PWRHOLD bit is set, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgSetPowerHold(const Pmic_Handle_t *handle, bool enable);

/**
 * @brief Get the enable/disable status of watchdog power hold.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] `PMIC_ENABLE` - watchdog power hold is enabled (watchdog
 * will stay in Long Window). `PMIC_DISABLE` - watchdog power hold is disabled
 * (watchdog can exit Long Window).
 *
 * @return Success code if the status of WD_PWRHOLD has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetPowerHold(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Enable or disable watchdog return to Long Window.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param enable [IN] `PMIC_ENABLE` - enable watchdog return to Long Window (the
 * watchdog will return to Long Window after completion of the current sequence).
 * `PMIC_DISABLE` - disable watchdog return to Long Window (watchdog can continue
 * sequences after the current sequence).
 *
 * @return Success code if WD_RETURN_LONGWIN bit is set, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgSetReturnToLongWindow(const Pmic_Handle_t *handle, bool enable);

/**
 * @brief Get the enable/disable status of watchdog return to Long Window.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] `PMIC_ENABLE` - watchdog return to Long Window is enabled
 * (watchdog will return to Long Window after completion of the current sequence).
 * `PMIC_DISABLE` - watchdog return to Long Window is disabled (watchdog can continue
 * sequences after the current sequence).
 *
 * @return success code if the status of WD_RETURN_LONGWIN has been obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetReturnToLongWindow(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Clear PMIC watchdog error statuses.
 *
 * @note To indicate the desired watchdog error status(es) to clear, the
 * validParams struct member of `wdgErrStatus` parameter must be set. All other
 * struct members will be ignored/unused throughout API execution. For valid
 * values of validParams, refer to @ref Pmic_WdgErrStatusValidParams.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgErrStatus [IN] The validParams struct member of this parameter indicates
 * which watchdog error status(es) to clear. For more information on watchdog error
 * statuses, refer to @ref Pmic_WdgErrStatus.
 *
 * @return Success code if watchdog error status(es) have been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes
 */
int32_t Pmic_wdgClrErrStatus(const Pmic_Handle_t *handle, const Pmic_WdgErrStatus_t *wdgErrStatus);

/**
 * @brief Clear all PMIC watchdog error statuses. For the statuses that are
 * cleared by this API, see `Pmic_wdgClrErrStatus()`.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if all watchdog error statuses have been cleared, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgClrErrStatusAll(const Pmic_Handle_t *handle);

/**
 * @brief Get PMIC watchdog error statuses. This API supports getting the same
 * statuses that are clearable by `Pmic_wdgClrErrStatus()`.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgErrStatus [OUT] Struct containing watchdog error statuses of the PMIC.
 * For more information on watchdog error statuses, refer to @ref Pmic_WdgErrStatus.
 *
 * @return Success code if watchdog error status(es) have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetErrStatus(const Pmic_Handle_t *handle, Pmic_WdgErrStatus_t *wdgErrStatus);

/**
 * @brief Get PMIC watchdog fail counter statuses.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgFailCntStatus [OUT] Fail count statuses obtained from the PMIC. For
 * more information on watchdog fail counter statuses, refer to
 * @ref Pmic_WdgFailCntStatus.
 *
 * @return Success code if the watchdog fail counter status(es) have been
 * obtained, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetFailCntStatus(const Pmic_Handle_t *handle, Pmic_WdgFailCntStatus_t *wdgFailCntStatus);

/**
 * @brief Calculate and send a Q&A answer byte to the PMIC watchdog.
 *
 * @details The API should be called four times in Long Window to exit Long
 * Window. For every Q&A sequence thereafter, the API should be called three
 * times in Window-1 and one time in Window-2.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if Q&A answer byte has been sent to the PMIC, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgQaWriteAnswer(const Pmic_Handle_t *handle);

/**
 * @brief Read PMIC register that has the WDG Q&A feedback.
 *
 * @details To calculate and send the correct WDG answer byte, the MCU needs
 * the correct WDG feedback, answer count, and question values. This API is
 * used as part of a series of 5 APIs which can be used to write an answer
 * byte to the PMIC.
 * 1. Pmic_wdgGetFdbkRegData()
 * 2. Pmic_wdgExtractFdbk()
 * 3. Pmic_wdgGetAnsCntAndQuesRegData()
 * 4. Pmic_wdgExtractAnsCntAndQues()
 * 5. Pmic_wdgWriteAnswer()
 *
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps automatically.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regData [OUT] Register data that contains the WDG Q&A feedback.
 *
 * @return PMIC_ST_SUCCESS if PMIC register data has been obtained, error code
 * otherwise. For valid success/error codes, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetFdbkRegData(const Pmic_Handle_t *handle, uint8_t *regData);

/**
 * @brief Extract WDG feedback value from the input register data.
 *
 * @details To calculate and send the correct WDG answer byte, the MCU needs
 * the correct WDG feedback, answer count, and question values. This API is
 * used as part of a series of 5 APIs which can be used to write an answer
 * byte to the PMIC.
 * 1. Pmic_wdgGetFdbkRegData()
 * 2. Pmic_wdgExtractFdbk()
 * 3. Pmic_wdgGetAnsCntAndQuesRegData()
 * 4. Pmic_wdgExtractAnsCntAndQues()
 * 5. Pmic_wdgWriteAnswer()
 *
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps automatically.
 *
 * @param regData [IN] Register data that contains the WDG Q&A feedback.
 *
 * @param wdgAnsInfo [OUT] WDG answer information structure. The feedback value
 * will be stored in this structure.
 *
 * @return PMIC_ST_SUCCESS if the WDG feedback value has been extracted, error
 * code otherwise. For valid success/error codes, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgExtractFdbk(uint8_t regData, Pmic_WdgAnsInfo_t *wdgAnsInfo);

/**
 * @brief Read PMIC register that has the WDG Q&A answer count and question.
 *
 * @details To calculate and send the correct WDG answer byte, the MCU needs
 * the correct WDG feedback, answer count, and question values. This API is
 * used as part of a series of 5 APIs which can be used to write an answer
 * byte to the PMIC.
 * 1. Pmic_wdgGetFdbkRegData()
 * 2. Pmic_wdgExtractFdbk()
 * 3. Pmic_wdgGetAnsCntAndQuesRegData()
 * 4. Pmic_wdgExtractAnsCntAndQues()
 * 5. Pmic_wdgWriteAnswer()
 *
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps automatically.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regData [OUT] Register data that contains the WDG answer count and
 * question.
 *
 * @return PMIC_ST_SUCCESS if PMIC register data has been obtained, error
 * code otherwise. For valid success/error codes, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetAnsCntAndQuesRegData(const Pmic_Handle_t *handle, uint8_t *regData);

/**
 * @brief Extract WDG answer count and question from the input register data.
 *
 * @details To calculate and send the correct WDG answer byte, the MCU needs
 * the correct WDG feedback, answer count, and question values. This API is
 * used as part of a series of 5 APIs which can be used to write an answer
 * byte to the PMIC.
 * 1. Pmic_wdgGetFdbkRegData()
 * 2. Pmic_wdgExtractFdbk()
 * 3. Pmic_wdgGetAnsCntAndQuesRegData()
 * 4. Pmic_wdgExtractAnsCntAndQues()
 * 5. Pmic_wdgWriteAnswer()
 *
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps automatically.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regData [IN] Register data that contains the WDG answer count and
 * question.
 *
 * @param wdgAnsInfo [OUT] WDG answer information structure. The answer count
 * and question will be stored in this structure.
 *
 * @return PMIC_ST_SUCCESS if the WDG answer count and question has been
 * extracted, error code otherwise. For valid success/error codes, see
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgExtractAnsCntAndQues(const Pmic_Handle_t *handle, uint8_t regData, Pmic_WdgAnsInfo_t *wdgAnsInfo);

/**
 * @brief Calculate and write WDG answer byte to the PMIC.
 *
 * @details To calculate and send the correct WDG answer byte, the MCU needs
 * the correct WDG feedback, answer count, and question values. This API is
 * used as part of a series of 5 APIs which can be used to write an answer
 * byte to the PMIC.
 * 1. Pmic_wdgGetFdbkRegData()
 * 2. Pmic_wdgExtractFdbk()
 * 3. Pmic_wdgGetAnsCntAndQuesRegData()
 * 4. Pmic_wdgExtractAnsCntAndQues()
 * 5. Pmic_wdgWriteAnswer()
 *
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps automatically.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wdgAnsInfo [IN] WDG answer information structure that contains the
 * necessary data to calculate the correct WDG answer byte.
 *
 * @return PMIC_ST_SUCCESS if the WDG answer has been calculated and sent to the
 * PMIC, error code otherwise. For valid success/error codes, see
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgWriteAnswer(const Pmic_Handle_t *handle, const Pmic_WdgAnsInfo_t *wdgAnsInfo);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_WDG_H */
