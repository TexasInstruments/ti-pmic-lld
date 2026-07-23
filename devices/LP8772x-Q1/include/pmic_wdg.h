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
#ifndef PMIC_WDG_H
#define PMIC_WDG_H

/**
 * @file pmic_wdg.h
 * @brief PMIC Driver Watchdog API/Interface
 */

/**
 * @defgroup DRV_PMIC_WDG_MODULE PMIC Watchdog Module
 * @brief Watchdog control, configuration, and status information.
 */

/**
 * @defgroup DRV_PMIC_WDG_CONFIG_GROUP PMIC Watchdog Configuration
 * @ingroup DRV_PMIC_WDG_MODULE
 * @brief Types and functions for configuring the PMIC watchdog module.
*/

/**
 * @defgroup DRV_PMIC_WDG_ERROR_GROUP PMIC Watchdog Error Handling
 * @ingroup DRV_PMIC_WDG_MODULE
 * @brief Types and functions for handling errors related to PMIC watchdog.
 */

/**
 * @defgroup DRV_PMIC_WDG_APP_GROUP PMIC Watchdog Application Use
 * @ingroup DRV_PMIC_WDG_MODULE
 * @brief Types and functions intended to simplify use of WDG by application code.
 */

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/

#include <stdbool.h>
#include <stdint.h>

#include "pmic.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_WdgThresholdCount
 * @name PMIC watchdog timer Threshold Configurations
 *
 * @{
 */
#define PMIC_WDG_THRESHOLD_COUNT_0                  (0U)
#define PMIC_WDG_THRESHOLD_COUNT_1                  (1U)
#define PMIC_WDG_THRESHOLD_COUNT_2                  (2U)
#define PMIC_WDG_THRESHOLD_COUNT_3                  (3U)
#define PMIC_WDG_THRESHOLD_COUNT_4                  (4U)
#define PMIC_WDG_THRESHOLD_COUNT_5                  (5U)
#define PMIC_WDG_THRESHOLD_COUNT_6                  (6U)
#define PMIC_WDG_THRESHOLD_COUNT_7                  (7U)
#define PMIC_WDG_THRESHOLD_COUNT_MAX                (PMIC_WDG_THRESHOLD_COUNT_7)
/** @} */

/**
 * @anchor Pmic_WdgQaFdbkVal
 * @name PMIC watchdog timer Q&A Feedback Values
 *
 * @{
 */
#define PMIC_WDG_QA_FEEDBACK_VALUE_0                (0U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_1                (1U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_2                (2U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_3                (3U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_MAX              (3U)
/** @} */

/**
 * @anchor Pmic_WdgQaLfsrVal
 * @name PMIC watchdog timer Q&A LFSR Values
 *
 * @{
 */
#define PMIC_WDG_QA_LFSR_VALUE_0                    (0U)
#define PMIC_WDG_QA_LFSR_VALUE_1                    (1U)
#define PMIC_WDG_QA_LFSR_VALUE_2                    (2U)
#define PMIC_WDG_QA_LFSR_VALUE_3                    (3U)
#define PMIC_WDG_QA_LFSR_VALUE_MAX                  (3U)
/** @} */

/**
 * @anchor Pmic_WdgQaQuestionSeedVal
 * @name PMIC watchdog timer Q&A Question Seed Values
 *
 * @{
 */
#define PMIC_WDG_QA_QUES_SEED_VALUE_0               (0U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_1               (1U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_2               (2U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_3               (3U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_4               (4U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_5               (5U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_6               (6U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_7               (7U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_8               (8U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_9               (9U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_10              (10U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_11              (11U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_12              (12U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_13              (13U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_14              (14U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_15              (15U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_MAX             (15U)
/** @} */

/**
 * @anchor Pmic_WdgWinCodeMax
 * @name PMIC watchdog timer maximum window code
 */
#define PMIC_WDG_WIN_CODE_MAX (0x7FU)

/**
 * @anchor Pmic_WdgCfgValidParamBitPos
 * @name PMIC watchdog timer Config Structure Param Bit positions
 *
 * @{
 */
#define PMIC_CFG_WDG_LONG_WIN_CODE_VALID   (1UL << 0U)
#define PMIC_CFG_WDG_WIN1_CODE_VALID       (1UL << 1U)
#define PMIC_CFG_WDG_WIN2_CODE_VALID       (1UL << 2U)
#define PMIC_CFG_WDG_THRESHOLD_RESET_VALID (1UL << 3U)
#define PMIC_CFG_WDG_THRESHOLD_FAIL_VALID  (1UL << 4U)
#define PMIC_CFG_WDG_MODE_VALID            (1UL << 5U)
#define PMIC_CFG_WDG_QA_FDBK_VALID         (1UL << 6U)
#define PMIC_CFG_WDG_QA_LFSR_VALID         (1UL << 7U)
#define PMIC_CFG_WDG_QA_QUES_SEED_VALID    (1UL << 8U)
#define PMIC_CFG_WDG_TIME_BASE_VALID       (1UL << 9U)
#define PMIC_CFG_WDG_RST_EN_VALID          (1UL << 10U)
/** @} */

/**
 * @anchor Pmic_WdgCfgValidParamBitShiftVal
 * @name PMIC WatchDog Config Structure Params Bit shift values
 *
 * @brief Application can use below shifted values to set the `validParam`
 * structure member defined in @ref Pmic_WdgCfg_t structure.
 *
 * @{
 */
#define PMIC_CFG_WDG_CFG_ALL_VALID     (\
    PMIC_CFG_WDG_LONG_WIN_CODE_VALID   |\
    PMIC_CFG_WDG_WIN1_CODE_VALID       |\
    PMIC_CFG_WDG_WIN2_CODE_VALID       |\
    PMIC_CFG_WDG_THRESHOLD_RESET_VALID |\
    PMIC_CFG_WDG_THRESHOLD_FAIL_VALID  |\
    PMIC_CFG_WDG_MODE_VALID            |\
    PMIC_CFG_WDG_QA_FDBK_VALID         |\
    PMIC_CFG_WDG_QA_LFSR_VALID         |\
    PMIC_CFG_WDG_QA_QUES_SEED_VALID    |\
    PMIC_CFG_WDG_TIME_BASE_VALID)
/** @} */

/**
 * @anchor Pmic_WdgErrStatCfgValidParamBitPos
 * @name PMIC watchdog timer error status Structure Param Bit positions.
 *
 * @{
 */
#define PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID   (1UL << 0U)
#define PMIC_CFG_WD_TIMEOUT_ERR_VALID           (1UL << 1U)
#define PMIC_CFG_WD_ANSW_EARLY_ERR_VALID        (1UL << 3U)
#define PMIC_CFG_WD_SEQ_ERR_ERR_VALID           (1UL << 4U)
#define PMIC_CFG_WD_ANSW_ERR_ERR_VALID          (1UL << 5U)
#define PMIC_CFG_WD_FAIL_INT_ERR_VALID          (1UL << 6U)
#define PMIC_CFG_WD_RST_INT_ERR_VALID           (1UL << 7U)
/** @} */

/**
 * @anchor Pmic_WdgFailCntStatCfgValidParamBitPos
 * @name PMIC watchdog Fail count status Structure Param Bit positions.
 *
 * @{
 */
#define PMIC_CFG_WD_BAD_EVENT_STAT_VALID            (1UL << 0U)
#define PMIC_CFG_WD_GOOD_EVENT_STAT_VALID           (1UL << 1U)
#define PMIC_CFG_WD_FAIL_CNT_VAL_VALID              (1UL << 2U)
#define PMIC_CFG_WD_LONGWIN_ACTIVE_VALID            (1UL << 3U)
/** @} */

/**
 * @anchor Pmic_WdgErrStatValidParamBitShiftVal
 * @name PMIC WatchDog Error status Structure Params Bit shift values
 *
 * @brief Application can use these values to set the validParams structure
 * member defined in @ref Pmic_WdgErrStatus_t structure.
 *
 * @{
 */
#define PMIC_CFG_WD_ERRSTAT_ALL_VALID (\
    PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID | \
    PMIC_CFG_WD_TIMEOUT_ERR_VALID         | \
    PMIC_CFG_WD_TRIG_EARLY_ERR_VALID      | \
    PMIC_CFG_WD_ANSW_EARLY_ERR_VALID      | \
    PMIC_CFG_WD_SEQ_ERR_ERR_VALID         | \
    PMIC_CFG_WD_ANSW_ERR_ERR_VALID        | \
    PMIC_CFG_WD_FAIL_INT_ERR_VALID        | \
    PMIC_CFG_WD_RST_INT_ERR_VALID)
/** @} */

/**
 * @anchor Pmic_WdgFailCntStatValidParamBitShiftVal
 * @name PMIC WatchDog Fail Count Status Structure validParams Shift Values
 *
 * @brief Application can use these values to set the validParams structure
 * member defined in @ref Pmic_WdgFailCntStatus_t structure.
 *
 *  @{
 */
#define PMIC_CFG_WD_FAILCNT_ALL_VALID         (\
    PMIC_CFG_WD_BAD_EVENT_STAT_VALID  |\
    PMIC_CFG_WD_GOOD_EVENT_STAT_VALID |\
    PMIC_CFG_WD_FAIL_CNT_VAL_VALID    |\
    PMIC_CFG_WD_LONGWIN_ACTIVE_VALID)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 * @brief This structure is used in setting or getting the Watchdog
 * configurations of supported PMICs.
 *
 * @note `validParams` is an input parameter for all Set and Get APIs. Other
 * struct members are input params for Set APIs and output params for Get APIs.
 *
 * @param validParams Selection of structure parameters to be set from the
 * combination of the @ref Pmic_WdgCfgValidParamBitPos and the corresponding
 * member value will be updated.
 *
 * @param rstEn Enable/disable warm reset when WDG fail counter (WD_FAIL_CNT) is
 * greater than `thresholdReset` + `thresholdFail`.
 *
 * @param thresholdReset Value for Watchdog Threshold 1 (WD_TH1). See @ref
 * Pmic_WdgThresholdCount.
 *
 * @param thresholdFail Value for Watchdog Threshold 2 (WD_TH2). See @ref
 * Pmic_WdgThresholdCount.
 *
 * @param longWinCode Long Window duration code for WD_LONGWIN_CFG register.
 * See datasheet for calculation of code to time.
 *
 * @param win1Code Window-1 duration code for WD_WIN1_CFG register. See
 * datasheet for calculation of code to time. For the maximum value, see
 * @ref Pmic_WdgWinCodeMax.
 *
 * @param win2Code Window-2 duration code for WD_WIN2_CFG register. See
 * datasheet for calculation of code to time. For the maximum value, see
 * @ref Pmic_WdgWinCodeMax.
 *
 * @param qaFdbk Configure Q&A Feedback value. See @ref Pmic_WdgQaFdbkVal.
 *
 * @param qaLfsr Configure Q&A LFSR value. See @ref Pmic_WdgQaLfsrVal.
 *
 * @param qaQuesSeed Configure Q&A question seed value. See @ref
 * Pmic_WdgQaQuestionSeedVal.
 */
typedef struct Pmic_WdgCfg_s {
    uint32_t validParams;

    bool rstEn;

    uint8_t thresholdReset;
    uint8_t thresholdFail;

    uint8_t longWinCode;
    uint8_t win1Code;
    uint8_t win2Code;

    uint8_t qaFdbk;
    uint8_t qaLfsr;
    uint8_t qaQuesSeed;
} Pmic_WdgCfg_t;

/**
 * @brief This struct is used to get and clear the Watchdog error statuses of
 * supported PMICs.
 *
 * @note `validParams` is an input parameter for all Set and Get APIs. Other
 * struct members are input params for Set APIs and output params for Get APIs.
 *
 * @note For other parameters, when used with `*GetErrorStatus` functions these
 * are output parameters, when used with `*ClrErrorStatus` functions these are
 * input parameters. When clearing error status, both the `validParams` field
 * and the relevant error status struct field must be set.
 *
 * @param validParams Selection of structure parameters to be set from the
 * combination of the @ref Pmic_WdgErrStatCfgValidParamBitPos and the
 * corresponding member value will be updated.
 *
 * @param timeout Indicates a watchdog error event occurred within Window 1/2.
 * @param longWindowTimeout Indicates the long window timer elapsed.
 * @param answerEarlyError Indicates all 4 answer bytes were received in Window 1.
 *        **Applies to Q&A mode only.**
 * @param sequenceError Indicates the number of bytes in previous answer
 *        sequence was incorrect. **Applies to Q&A mode only.**
 * @param answerError Indicates answers were incorrect either in timing or order.
 *        **Applies to Q&A mode only.**
 * @param failInt Watchdog failure counter exceeded `thresholdReset`.
 * @param resetInt Watchdog failure counter exceeded `thresholdFail`.
 */
typedef struct Pmic_WdgErrStatus_s {
    uint32_t validParams;

    bool timeout;
    bool longWindowTimeout;
    bool answerEarlyError;
    bool sequenceError;
    bool answerError;
    bool failInt;
    bool resetInt;
} Pmic_WdgErrStatus_t;

/**
 * @brief This struct is used to get the Watchdog bad/good event and fail count
 * of supported PMICs.
 *
 * @note `validParams` is an input parameter for all Set and Get APIs. Other
 * struct members are input params for Set APIs and output params for Get APIs.
 *
 * @param validParams Selection of structure parameters to be set from the
 * combination of the @ref Pmic_WdgFailCntStatCfgValidParamBitPos and the
 * corresponding member value will be updated.
 *
 * @param badEvent To get status of Bad Event is detected or not
 * @param goodEvent To get status of Good Event is detected or not
 * @param wdFailCnt To get Watchdog Fail Count value.
 */
typedef struct Pmic_WdgFailCntStatus_s {
    uint16_t validParams;

    bool badEvent;
    bool goodEvent;
    uint8_t wdFailCnt;
} Pmic_WdgFailCntStatus_t;

/**
 * @brief This struct contains the information needed to calculate WDG answer
 * bytes.
 *
 * @param fdbk WDG Q&A feedback value.
 * @param ansCnt WDG Q&A answer count value.
 * @param question WDG Q&A question.
 */
typedef struct Pmic_WdgAnsInfo_s {
    uint8_t fdbk;
    uint8_t ansCnt;
    uint8_t question;
} Pmic_WdgAnsInfo_t;

/* ========================================================================== */
/*                            Function Declarations                           */
/* ========================================================================== */
/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Enable PMIC watchdog. This API is a subset of `Pmic_wdgSetEnableState()`.
 *
 * Design: PMICDRV-660
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgEnable(const Pmic_Handle_t *handle);

/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Disable PMIC watchdog. This API is a subset of `Pmic_wdgSetEnableState()`.
 *
 * Design: PMICDRV-661
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgDisable(const Pmic_Handle_t *handle);

/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Enable or disable the PMIC watchdog. This API is a superset of
 * `Pmic_wdgEnable()` and `Pmic_wdgDisable()`.
 *
 * Design: PMICDRV-662
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN]  PMIC interface handle
 * @param enable [IN]  Set to true (PMIC_ENABLE) to enable watchdog, false
 * (PMIC_DISABLE) to disable. See @ref Pmic_EnableDisable.
 *
 * @return Success code if Watchdog Enable state is set, error code otherwise.
 * For possible success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgSetEnableState(const Pmic_Handle_t *handle, bool enable);

/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Get the enable state of the PMIC watchdog.
 *
 * Design: PMICDRV-663
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @param handle    [IN]  PMIC interface handle
 * @param isEnabled [OUT] true (PMIC_ENABLE) if watchdog is enabled, otherwise
 * false (PMIC_DISABLE). See @ref Pmic_EnableDisable.
 *
 * @return Success code if Watchdog Enable state is obtained, error code
 * otherwise. For possible success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Set PMIC watchdog configurations.
 *
 * Design: PMICDRV-664
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @note User has to call `Pmic_wdgEnable()` before setting the configuration.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param wdgCfg [IN] Watchdog configuration
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgSetCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg);

/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Get PMIC watchdog configurations. This API supports getting the same
 * configurations that are settable by `Pmic_wdgSetCfg()`.
 *
 * Design: PMICDRV-665
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @note User has to call `Pmic_wdgEnable()` before getting the configuration,
 * otherwise the results are invalid.
 *
 * @param handle [IN]     PMIC Interface Handle
 * @param wdgCfg [IN/OUT] Watchdog configuration pointer
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg);

/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Enable/disable WDG Power Hold, which controls whether WDG stays in Long-
 * Window.
 *
 * Design: PMICDRV-668
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN] PMIC Interface Handle
 * @param enable [IN] If set to true (PMIC_ENABLE), both WDG and Long-Window
 * timer are paused and do not cause the device to reset. Otherwise, the WDG
 * operates as normally configured. See @ref Pmic_EnableDisable.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgSetPowerHold(const Pmic_Handle_t *handle, bool enable);

/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Get the WDG Power Hold enable/disable state, which indicates whether WDG
 * stays in Long-Window.
 *
 * Design: PMICDRV-669
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @param handle    [IN] PMIC Interface Handle
 * @param isEnabled [OUT] If set to true (PMIC_ENABLE), both WDG and
 * Long-Window timer are paused and do not cause the device to reset.
 * Otherwise, the WDG is operating as normally configured. See @ref
 * Pmic_EnableDisable.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetPowerHold(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Enable/disable WDG Return to Long-Window, which controls whether WDG
 * returns to Long-Window at the end of the current sequence.
 *
 * Design: PMICDRV-670
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN] PMIC Interface Handle
 * @param enable [IN] If set to true (PMIC_ENABLE) while the WDG is in normal
 * operating mode it will return to Long-Window mode, otherwise the WDG will
 * return to or remain in normal operation. See @ref Pmic_EnableDisable.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgSetReturnToLongWindow(const Pmic_Handle_t *handle, bool enable);

/**
 * @ingroup DRV_PMIC_WDG_CONFIG_GROUP
 * @brief Get WDG Return to Long-Window enable/disable state, which indicates
 * whether WDG will return to Long-Window at the end of the current sequence.
 *
 * Design: PMICDRV-671
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @param handle    [IN] PMIC Interface Handle
 * @param isEnabled [IN] Tracks the value of the WD_RETURN_LONGWIN bit, will be
 * false (PMIC_DISABLE) if the WDG is in normal operating mode. See @ref
 * Pmic_EnableDisable.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetReturnToLongWindow(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @ingroup DRV_PMIC_WDG_ERROR_GROUP
 * @brief Get PMIC watchdog error statuses.
 *
 * Design: PMICDRV-731
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-528, PMICDRV-538
 *
 * @note User has to call `Pmic_wdgEnable()` before getting the error status,
 * otherwise the results are invalid.
 *
 * @param handle  [IN]     PMIC Interface Handle
 * @param errors  [IN/OUT] Watchdog error status pointer
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetErrStatus(const Pmic_Handle_t *handle, Pmic_WdgErrStatus_t *errors);

/**
 * @ingroup DRV_PMIC_WDG_ERROR_GROUP
 * @brief API to clear PMIC watchdog error status.
 *
 * Design: PMICDRV-673
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * This function is used to clear the watchdog error status from the PMIC for
 * trigger mode or Q&A (question and answer) mode.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param errors [IN] Errors to clear, the relevant `validParams` must be set,
 * as well as the desired struct member(s) to clear.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgClrErrStatus(const Pmic_Handle_t *handle, const Pmic_WdgErrStatus_t *errors);

/**
 * @ingroup DRV_PMIC_WDG_ERROR_GROUP
 * @brief Clear all PMIC watchdog error statuses. Provided as a convenience,
 * however, it is recommended to process watchdog statuses via
 * `Pmic_wdgGetErrStatus()` and `Pmic_wdgClrErrStatus()` APIs.
 *
 * Design: PMICDRV-674
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgClrErrStatusAll(const Pmic_Handle_t *handle);

/**
 * @ingroup DRV_PMIC_WDG_ERROR_GROUP
 * @brief Get PMIC watchdog fail counter statuses.
 *
 * Design: PMICDRV-675
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @note User has to call `Pmic_wdgEnable()` before getting the fail count,
 * otherwise the results are invalid.
 *
 * @param handle    [IN]     PMIC Interface Handle
 * @param failCount [IN/OUT] Watchdog fail count pointer
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetFailCntStatus(const Pmic_Handle_t *handle, Pmic_WdgFailCntStatus_t *failCount);

/**
 * @ingroup DRV_PMIC_WDG_APP_GROUP
 * @brief Calculate and send a WDG Q&A answer byte to the PMIC.
 *
 * Design: PMICDRV-676
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @note In the event this API returns PMIC_ST_ERR_INV_WDG_ANSWER, the user
 * should adjust the Long window time interval, Window1 time interval, and
 * Window2 time interval.
 *
 * @note In the event this API returns PMIC_ST_ERR_INV_WDG_ANSWER, the user
 * should call `Pmic_wdgGetErrStatus()` in order to read the WDG error. Based
 * on the determined error, the following actions may help:
 * - If the WDG error is Long Window Timeout or Timeout, user has to increase
 *   the Long window or window1 time interval accordingly
 * - If the WDG error is Answer early, user has to reduce the Window1 time
 *   interval
 * - For other WDG errors, user has to take action accordingly Application has
 *   to ensure to do proper configuration of WDG window time intervals.
 *
 * @note If not configured properly in Q&A mode then WDG will trigger the warm
 * reset to the PMIC device. This may cause system reset if PMIC is connected
 * to SOC/MCU
 *
 * @param handle [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgQaWriteAnswer(const Pmic_Handle_t *handle);

/**
 * @brief Read PMIC register that has the WDG Q&A feedback.
 *
 * Design: PMICDRV-754
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
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
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps
 * automatically.
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
 * Design: PMICDRV-755
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
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
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps
 * automatically.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regData [IN] Register data that contains the WDG Q&A feedback.
 *
 * @param wdgAnsInfo [OUT] WDG answer information structure. The feedback value
 * will be stored in this structure.
 *
 * @return PMIC_ST_SUCCESS if the WDG feedback value has been extracted, error
 * code otherwise. For valid success/error codes, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgExtractFdbk(const Pmic_Handle_t *handle, uint8_t regData, Pmic_WdgAnsInfo_t *wdgAnsInfo);

/**
 * @brief Read PMIC register that has the WDG Q&A answer count and question.
 *
 * Design: PMICDRV-756
  * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522
 *               PMICDRV-528, PMICDRV-538
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
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps
 * automatically.
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
 * Design: PMICDRV-757
  * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522
 *               PMICDRV-528, PMICDRV-538
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
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps
 * automatically.
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
 * Design: PMICDRV-758
  * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522
 *               PMICDRV-523, PMICDRV-538
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
 * See also: `Pmic_wdgQaWriteAnswer()` which performs these steps
 * automatically.
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
