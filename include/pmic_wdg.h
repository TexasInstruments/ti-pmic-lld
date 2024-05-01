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
 *  \ingroup DRV_PMIC_MODULE
 *  \defgroup DRV_PMIC_API_MODULE PMIC Driver Watchdog API
 *
 * Watchdog control, configuration, and status information.
 *
 *  @{
 */

/**
 *  @file pmic_wdg.h
 *
 *  @brief PMIC Driver Watchdog API/Interface
 */

#ifndef __PMIC_WDG_H__
#define __PMIC_WDG_H__

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/
#include "pmic_core_priv.h"
#include "pmic_wdg_priv.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_WdgTriggerQAMode
 * @name PMIC watchdog timer Trigger/Q&A Mode
 *
 * @brief Sets the watchdog operating mode, **only configurable while the
 * watchdog is in Long-Window mode**.
 *
 * @{
 */
#define PMIC_WDG_TRIGGER_MODE                       (0U)
#define PMIC_WDG_SW_TRIGGER_MODE                    (1U)
#define PMIC_WDG_QA_MODE                            (2U)
#define PMIC_WDG_QA_SINGLE_MODE                     (3U)
#define PMIC_WDG_MODE_MAX                           (PMIC_WDG_QA_SINGLE_MODE)
/** @} */

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
/** @} */

/**
 * @anchor Pmic_WdgTimeBase
 * @name PMIC Watchdog Time Base Configuration
 *
 * @{
 */
#define PMIC_WDG_TIME_BASE_137_5_US                 (0U)
#define PMIC_WDG_TIME_BASE_275_US                   (1U)
#define PMIC_WDG_TIME_BASE_550_US                   (2U)
#define PMIC_WDG_TIME_BASE_1100_US                  (3U)
#define PMIC_WDG_TIME_BASE_MAX                      (PMIC_WDG_TIME_BASE_1100_US)
/** @} */

/**
 * @anchor Pmic_WdgCfgValidParamBitPos
 * @name PMIC watchdog timer Config Structure Param Bit positions
 *
 * @{
 */
/** @brief validParams value used to set/get Long Window duration */
#define PMIC_CFG_WDG_LONGWINDURATION_VALID          (0U)
/** @brief validParams value used to set/get Window-1 duration */
#define PMIC_CFG_WDG_WIN1DURATION_VALID             (1U)
/** @brief validParams value used to set/get Window-2 duration */
#define PMIC_CFG_WDG_WIN2DURATION_VALID             (2U)
/** @brief validParams value used to set/get threshold-1 value */
#define PMIC_CFG_WDG_THRESHOLD_1_VALID              (3U)
/** @brief validParams value used to set/get threshold-2 Value */
#define PMIC_CFG_WDG_THRESHOLD_2_VALID              (4U)
/** @brief validParams value used to set/get watchdog mode */
#define PMIC_CFG_WDG_MODE_VALID                     (6U)
/** @brief validParams value used to set/get Q&A feed back value */
#define PMIC_CFG_WDG_QA_FDBK_VALID                  (9U)
/** @brief validParams value used to set/get Q&A LFSR value */
#define PMIC_CFG_WDG_QA_LFSR_VALID                  (10U)
/** @brief validParams value used to set/get Q&A question seed value */
#define PMIC_CFG_WDG_QA_QUES_SEED_VALID             (11U)
/** @brief validParams value used to set/get the watchdog time base configuration */
#define PMIC_CFG_WDG_TIME_BASE_VALID                (12U)
/** @} */

/**
 * @name PMIC Watchdog Min Wait Count
 * @brief Minimum number of iterations to wait for a Good/Bad event
 */
#define PMIC_WDG_WAIT_CNT_MIN_VAL                   (30U)

/**
 * @anchor Pmic_WdgCfgValidParamBitShiftVal
 * @name PMIC WatchDog Config Structure Params Bit shift values
 *
 * Application can use below shifted values to set the `validParam` structure
 * member defined in @ref Pmic_WdgCfg_t structure.
 *
 * @{
 */
#define PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT    (1U << PMIC_CFG_WDG_LONGWINDURATION_VALID)
#define PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT       (1U << PMIC_CFG_WDG_WIN1DURATION_VALID)
#define PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT       (1U << PMIC_CFG_WDG_WIN2DURATION_VALID)
#define PMIC_CFG_WDG_THRESHOLD_1_VALID_SHIFT        (1U << PMIC_CFG_WDG_THRESHOLD_1_VALID)
#define PMIC_CFG_WDG_THRESHOLD_2_VALID_SHIFT        (1U << PMIC_CFG_WDG_THRESHOLD_2_VALID)
#define PMIC_CFG_WDG_MODE_VALID_SHIFT               (1U << PMIC_CFG_WDG_MODE_VALID)
#define PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT            (1U << PMIC_CFG_WDG_QA_FDBK_VALID)
#define PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT            (1U << PMIC_CFG_WDG_QA_LFSR_VALID)
#define PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT       (1U << PMIC_CFG_WDG_QA_QUES_SEED_VALID)
#define PMIC_CFG_WDG_TIME_BASE_VALID_SHIFT          (1U << PMIC_CFG_WDG_TIME_BASE_VALID)
/** @} */

/**
 * @anchor Pmic_WdgErrStatCfgValidParamBitPos
 * @name PMIC watchdog timer error status Structure Param Bit positions.
 *
 *  @{
 */
/** @brief validParams value used to get Long Window timeout error status */
#define PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID   (0U)
/** @brief validParams value used to get Window1/Window2 timeout error status */
#define PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID           (1U)
/** @brief validParams value used to get Watchdog trigger mode error status */
#define PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID        (2U)
/** @brief validParams value used to get Watchdog early answer error status */
#define PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID        (3U)
/** @brief validParams value used to get Watchdog Q&A sequence error status */
#define PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID           (4U)
/** @brief validParams value used to get Watchdog Q&A wrong Answer error status */
#define PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID          (5U)
/** @brief validParams value used to get Watchdog fail error status */
#define PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID          (6U)
/** @brief validParams value used to get Watchdog reset error status */
#define PMIC_CFG_WD_RST_INT_ERRSTAT_VALID           (7U)
/** @} */

/**
 * @anchor Pmic_WdgFailCntStatCfgValidParamBitPos
 * @name PMIC watchdog Fail count status Structure Param Bit positions.
 *
 *  @{
 */
/** @brief validParams value used to get status of Bad Event is detected or not */
#define PMIC_CFG_WD_BAD_EVENT_STAT_VALID            (0U)
/** @brief validParams value used to get status of Good Event is detected or not */
#define PMIC_CFG_WD_GOOD_EVENT_STAT_VALID           (1U)
/** @brief validParams value used to get To get Watchdog Fail Count value */
#define PMIC_CFG_WD_FAIL_CNT_VAL_VALID              (2U)
/** @} */

/**
 * @anchor Pmic_WdgErrStatValidParamBitShiftVal
 * @name PMIC WatchDog Error status Structure Params Bit shift values
 *
 * Application can use below shifted values to set the validParams structure
 * member defined in @ref Pmic_WdgErrStatus_t structure
 *
 *  @{
 */
#define PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID_SHIFT (1U << PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID)
#define PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID_SHIFT         (1U << PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID)
#define PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID_SHIFT      (1U << PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID)
#define PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID_SHIFT      (1U << PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID)
#define PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID_SHIFT         (1U << PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID)
#define PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID_SHIFT        (1U << PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID)
#define PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID_SHIFT        (1U << PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID)
#define PMIC_CFG_WD_RST_INT_ERRSTAT_VALID_SHIFT         (1U << PMIC_CFG_WD_RST_INT_ERRSTAT_VALID)
/** @} */

/**
 *  @anchor Pmic_WdgFailCntStatValidParamBitShiftVal
 *  @name PMIC WatchDog Fail count status Structure Params Bit shift values
 *
 *  Application can use below shifted values to set the validParams
 *  structure member defined in @ref Pmic_WdgErrStatus_t structure.
 *
 *  @{
 */
#define PMIC_CFG_WD_BAD_EVENT_STAT_VALID_SHIFT      (1U << PMIC_CFG_WD_BAD_EVENT_STAT_VALID)
#define PMIC_CFG_WD_GOOD_EVENT_STAT_VALID_SHIFT     (1U << PMIC_CFG_WD_GOOD_EVENT_STAT_VALID)
#define PMIC_CFG_WD_FAIL_CNT_VAL_VALID_SHIFT        (1U << PMIC_CFG_WD_FAIL_CNT_VAL_VALID)
/** @} */

/**
 * @name  Macro for PMIC Watchdog Q&A infinite sequence
 */
#define PMIC_WD_QA_INFINITE_SEQ                     (0xFFFFFFFFU)

/**
 * @anchor Pmic_WdgErrType
 * @name PMIC WDG Error TYPE
 *
 * @{
 */
#define PMIC_WDG_ERR_TIMEOUT                        (0U)
#define PMIC_WDG_ERR_TRIGGER_EARLY                  (1U)
#define PMIC_WDG_ERR_ANSWER_EARLY                   (2U)
#define PMIC_WDG_ERR_SEQ_ERR                        (3U)
#define PMIC_WDG_ERR_ANS_ERR                        (4U)
#define PMIC_WDG_ERR_LONG_WIN_TIMEOUT               (5U)
#define PMIC_WDG_ERR_THRES1                         (6U)
#define PMIC_WDG_ERR_THRES2                         (7U)
#define PMIC_WDG_ERR_ALL                            (8U)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 * @name  PMIC Watchdog Configuration Structure
 *
 * @brief This structure is used in setting or getting the Watchdog
 * configurations of supported PMICs (TPS6522x, TPS6594x, LP8764x).
 *
 * @note `validParams is an input parameter for all Set and Get APIs. Other
 * struct members are input params for Set APIs and output params for Get APIs.
 *
 * @param validParams Selection of structure parameters to be set from the
 * combination of the @ref Pmic_WdgCfgValidParamBitPos and the corresponding
 * member value will be updated.
 *
 * @param mode Value to set watchdog mode to. See @ref Pmic_WdgTriggerQAMode.
 *
 * @param timeBase Value to set the watchdog time base configuration to. See
 * @ref Pmic_WdgTimeBase
 *
 * @param threshold1 Value for Watchdog Threshold 1 (WD_TH1). See @ref
 * Pmic_WdgThresholdCount.
 *
 * @param threshold2 Value for Watchdog Threshold 2 (WD_TH2). See @ref
 * Pmic_WdgThresholdCount.
 *
 * @param longWinDuration_ms Long Window duration in milliseconds. To get
 * more effective results user has to program long window with multiples of
 * 3000.
 * - For PG1.0 Leo/Hera, the valid range is (100, 3000, 6000, 9000,....12000,
 *   ..., 765000).
 * - For PG2.0 and Burton, the valid range is (80, 125, 250, 375,....8000,
 *   12000, 16000, 20000 ..., 772000).
 *
 * @param win1Duration_us Window-1 duration in microseconds.
 * To get more effective results user has to program window1 with multiples of
 * 550. The valid range is (550, 1100, 1650, 2200, 2750, ..., 70400).
 *
 * @param win2Duration_us Window-2 duration in microseconds.
 * To get more effective results user has to program window1 with multiples of
 * 550. The valid range is (550, 1100, 1650, 2200, 2750, ..., 70400).
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

    uint8_t mode;
    uint8_t timeBase;
    uint8_t threshold1;
    uint8_t threshold2;

    uint32_t longWinDuration_ms;
    uint32_t win1Duration_us;
    uint32_t win2Duration_us;

    uint8_t qaFdbk;
    uint8_t qaLfsr;
    uint8_t qaQuesSeed;
} Pmic_WdgCfg_t;

/**
 * @name PMIC Watchdog Error Status Structure
 *
 * @brief This struct is used to get the Watchdog error statuses of supported
 * PMICs.
 *
 * @note `validParams is an input parameter for all Set and Get APIs. Other
 * struct members are input params for Set APIs and output params for Get APIs.
 *
 * @param validParams Selection of structure parameters to be set from the
 * combination of the @ref Pmic_WdgErrStatCfgValidParamBitPos and the
 * corresponding member value will be updated.
 *
 * @param timeout Indicates a watchdog error event occured within Window 1/2.
 * @param longWindowTimeout Indicates the long window timer elapsed.
 * @param triggerEarlyError Indicates the watchdog trigger was received in Window 1.
 *        **Applies to Trigger Mode only.**
 * @param answerEarlyError Indicates all 4 answer bytes were received in Window 1.
 *        **Applies to Q&A mode only.**
 * @param sequenceError Indicates the number of bytes in previous answer
 *        sequence was incorrect. **Applies to Q&A mode only.**
 * @param answerError Indicates answers were incorrect either in timing or order.
 *        **Applies to Q&A mode only.**
 * @param threshold1Error Watchdog failure counter exceeded `threshold1`.
 * @param threshold2Error Watchdog failure counter exceeded `threshold2`.
 */
typedef struct Pmic_WdgErrStatus_s {
    uint32_t validParams;

    bool timeout;
    bool longWindowTimeout;
    bool triggerEarlyError;
    bool answerEarlyError;
    bool sequenceError;
    bool answerError;
    bool threshold1Error;
    bool threshold2Error;
} Pmic_WdgErrStatus_t;

/**
 * @name PMIC Watchdog Fail Count Status Structure
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
 * @param wdBadEvent To get status of Bad Event is detected or not
 * @param wdGudEvent To get status of Good Event is detected or not
 * @param wdFailCnt To get Watchdog Fail Count value.
 */
typedef struct Pmic_WdgFailCntStat_s {
    uint16_t validParams;

    bool wdBadEvent;
    bool wdGudEvent;
    uint32_t wdFailCnt;
} Pmic_WdgFailCntStat_t;

/* ========================================================================== */
/*                            Function Declarations                           */
/* ========================================================================== */
/**
 * @brief API to Enable Watchdog.
 *
 * This function is used to enable the PMIC watchdog. User needs to
 * ensure that this function is called to enable watchdog timer before
 * configuring or starting watchdog trigger or Q&A mode.
 *
 * @param handle [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgEnable(Pmic_CoreHandle_t *handle);

/**
 * @brief API to Disable Watchdog.
 *
 * This function is used to disable the PMIC watchdog. User needs to ensure
 * that after using this function, complete watchdog functionality and
 * configuration will be deactivated.
 *
 * @param handle [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgDisable(Pmic_CoreHandle_t *handle);

/**
 * @brief This function is used to set the Watchdog Enable state.
 *
 * @param handle [IN]  PMIC interface handle
 * @param enable [IN]  Set to true to enable watchdog, false to disable
 *
 * @return Success code if Watchdog Enable state is set, error code otherwise.
 * For possible success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgSetEnableState(Pmic_CoreHandle_t *handle, bool enable);

/**
 * @brief This function is used to get the Watchdog Enable state (that is to
 * say, whether WD_EN bit is set to 1 or 0).
 *
 * @param handle  [IN]  PMIC interface handle
 * @param enabled [OUT] true if watchdog is enabled, otherwise false
 *
 * @return Success code if Watchdog Enable state is obtained, error code
 * otherwise. For possible success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetEnableState(Pmic_CoreHandle_t *handle, bool *isEnabled);

/**
 * @brief API to set PMIC watchdog configurations.
 *
 * This function is used to configure the watchdog parameters in the PMIC for
 * trigger mode or Q&A (question and answer) mode, when corresponding
 * validParam bit fields are set in @ref Pmic_WdgCfg_t structure. User has to
 * call Pmic_wdgEnable() before setting the configuration.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param wdgCfg [IN] Watchdog configuration
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgSetCfg(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *wdgCfg);

/**
 * @brief API to get PMIC watchdog configurations.
 *
 * This function is used to get configuration of the watchdog from the PMIC for
 * trigger mode or Q&A (question and answer) mode, when corresponding
 * validParam bit fields are set in Pmic_WdgCfg_t structure.
 *
 * User has to call Pmic_wdgEnable() before getting the configuration,
 * otherwise the results are invalid.
 *
 * @param handle [IN]     PMIC Interface Handle
 * @param wdgCfg [IN/OUT] Watchdog configuration pointer
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetCfg(Pmic_CoreHandle_t * handle, Pmic_WdgCfg_t *wdgCfg);

/**
 * @brief API to set the WD_PWRHOLD bit, which pauses the Long-Window timer.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param enable [IN] If set, both WDG and Long-Window timer are paused and do
 *               not cause the device to reset. Otherwise, the WDG operates as
 *               normally configured.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgSetPowerHold(Pmic_CoreHandle_t *handle, bool enable);

/**
 * @brief API to get the WD_PWRHOLD bit, which pauses the Long-Window timer.
 *
 * @param handle    [IN] PMIC Interface Handle
 * @param isEnabled [OUT] IF set, both WDG and Long-Window timer are paused and do
 *                  not cause the device to reset. Otherwise, the WDG is
 *                  operating as normally configured.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetPowerHold(Pmic_CoreHandle_t *handle, bool *isEnabled);

/**
 * @brief API to set the WD_RETURN_LONGWIN bit, which can be used while the WDG
 * is in normal operating mode to return it to Long-Window mode.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param enable [IN] If set while the WDG is in normal operating mode it will
 *               return to Long-Window mode, otherwise the WDG will return to
 *               or remain in normal operation.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgSetReturnToLongWindow(Pmic_CoreHandle_t *handle, bool enable);

/**
 * @brief API to get the value of the WD_RETURN_LONGWIN bit, which can be used
 * while the WDG is in normal operating mode to return it to Long-Window mode.
 *
 * @param handle    [IN] PMIC Interface Handle
 * @param isEnabled [IN] Tracks the value of the WD_RETURN_LONGWIN bit, will be
 *                  false if the WDG is in normal operating mode.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetReturnToLongWindow(Pmic_CoreHandle_t *handle, bool *isEnabled);

/**
 * @brief API to get PMIC watchdog error status.
 *
 * This function is used to get the watchdog error status from the PMIC for
 * trigger mode or Q&A (question and answer) mode, when corresponding
 * validParam bit fields are set in @ref Pmic_WdgErrStatus_t structure.
 *
 * @note User has to call Pmic_wdgEnable() before getting the error status,
 * otherwise the results are invalid.
 *
 * @param handle      [IN]     PMIC Interface Handle
 * @param errorStatus [IN/OUT] Watchdog error status pointer
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetErrorStatus(Pmic_CoreHandle_t *handle, Pmic_WdgErrStatus_t *errorStatus);

/**
 * @brief API to clear PMIC watchdog error status.
 *
 * This function is used to clear the watchdog error status from the PMIC for
 * trigger mode or Q&A (question and answer) mode,
 *
 * @note User has to clear the WDG Error status only when Error status
 * bit is set for the corresponding errorType.
 *
 * @param handle    [IN] PMIC Interface Handle
 * @param errorType [IN] Watchdog error type to clear the status. For Valid
 * values, see @ref Pmic_WdgErrType
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgClrErrStatus(Pmic_CoreHandle_t *handle, const uint8_t errorType);

/**
 * @brief API to get PMIC watchdog fail count status.
 *
 * This function is used to get the watchdog fail count status from the PMIC
 * for trigger mode or Q&A (question and answer) mode.
 *
 * @note User has to call `Pmic_wdgEnable()` before getting the fail count,
 * otherwise the results are invalid.
 *
 * @param handle     [IN]     PMIC Interface Handle
 * @param failCount [IN/OUT] Watchdog fail count pointer
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetFailCntStat(Pmic_CoreHandle_t *handle, Pmic_WdgFailCntStat_t *failCount);

/**
 * @brief API to start watchdog Q&A mode.
 *
 * This function is used to start watchdog sequence and continues until the
 * given `seqCount`. User has to ensure proper configuration of all
 * Watchdog Q&A parameters using `Pmic_wdgSetCfg()` API, before starting Q&A
 * sequence using this API.
 *
 * @note To perform Q&A sequences, user must adjust the Long
 * Window/Window1/Window2 time intervals depending on errors given by this API.
 * - If user gets PMIC_ST_ERR_INV_WDG_WINDOW, then user has to increase the
 *   Long window or Window1 time interval.
 * - If user gets PMIC_ST_ERR_WDG_EARLY_ANSWER, then user has to reduce the
 *   Window1 time interval.
 *
 * @note Application must ensure proper configuration of WDG window time
 * intervals. If not configured properly then WDG will trigger the warm reset
 * to the PMIC device. This may cause system reset if PMIC is connected to
 * SOC/MCU.
 *
 * @note Application has to ensure proper configuration of WDG parameters. If
 * not configured properly then API doesn't receive good or bad event from the
 * PMIC FSM; In this case, the API returns a timeout error.
 *
 * @note If API receives a bad event due to wrong answer then API detects and
 * returns an error.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param seqCount [IN] Number of Q&A sequences to execute. If
 *        PMIC_WD_QA_INFINITE_SEQ is used, then the API runs for infinite sequence.
 * @param maxCount [IN] Number of iterations to wait for a Good/Bad event. The
 *        value should be greater than or equal to PMIC_WDG_WAIT_CNT_MIN_VAL.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgStartQaSequence(Pmic_CoreHandle_t *handle, uint32_t seqCount, uint32_t maxCount);

/**
 * @brief API to write answers in Long Window/Window1/Window2 intervals for
 * watchdog Q&A Sequence.
 *
 * This function is used to write Answers in Long Window/Window1/Window2
 * Interval for the WDG Q&A Sequence.
 *
 * User must ensure all Watchdog Q&A parameters are configured properly
 * using Pmic_wdgSetCfg() API, before writing Answers using this API
 * for the Q&A Sequence.
 *
 * @note In the event this API returns PMIC_ST_ERR_INV_WDG_ANSWER, the user
 * should adjust the Long window time interval, Window1 time interval, and
 * Window2 time interval.
 *
 * @note In the event this API returns PMIC_ST_ERR_INV_WDG_ANSWER, the user
 * should call `Pmic_wdgGetErrorStatus()` in order to read the WDG error. Based
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
int32_t Pmic_wdgQaSequenceWriteAnswer(Pmic_CoreHandle_t *handle);

/**
 * @brief API to start watchdog Trigger mode.
 *
 * This function is used to start watchdog trigger mode.
 *
 * User must ensure correct configuration of all Watchdog trigger parameters
 * using `Pmic_wdgSetCfg()` API before starting watchdog trigger mode using
 * this API. User can call `Pmic_wdgSetCfg()` API to stop watchdog trigger
 * mode.
 *
 * @note To perform watchdog trigger mode, user must adjust Long
 * Window/Window1/Window2 time intervals as below. Depending on the time-period
 * of the trigger pulse provided by other device:
 *  1. Long window time interval must be greater than Trigger pulse time
 *     period.
 *  2. Window1 time interval must be less than T-off time of the Trigger pulse
 *     time period.
 *  3. Window2 time interval must be greater than T-on time of the Trigger
 *     pulse time period.
 *  4. (Window1 time interval + Window2 time interval) approximately equal to
 *     the Trigger pulse time period.
 *
 * @note Application must ensure proper configuration of WDG window time
 * intervals. If not configured properly in Trigger mode then WDG will trigger
 * the warm reset to the PMIC device. This may cause system reset if PMIC is
 * connected to SOC/MCU
 *
 * @param handle [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgStartTriggerSequence(Pmic_CoreHandle_t *handle);

/**
 * @brief This API is used to initiate Watchdog (WDG) sequences - either
 * Trigger or Q&A sequences - for TPS6522x and other PMICs that have the same
 * Watchdog IP block.
 *
 * @details The API clears any previous WDG errors and enables the WDG to
 * receive triggers or answers from the MCU depending on the value of the
 * `mode` parameter.
 *
 * This API is intended to be called before sending any MCU triggers or answers
 * to the PMIC and just after configuration of the WDG using the
 * `Pmic_wdgSetCfg()` API.
 *
 * Once this API is called, depending on the WDG mode of operation, a trigger
 * or four answer bytes must be sent shortly afterwards to exit Long Window.
 *
 * Once Long Window is exited, Watchdog sequences will begin.
 * - For every Trigger mode sequence, a trigger must be sent in Window-2.
 * - For every Q&A mode sequence, end-user must call
 *   Pmic_wdgQaSequenceWriteAnswer() three times in Window-1 and call the API
 *   again one time during Window-2.
 *
 * If end-user wants to end WDG sequences, the user should configure the
 * Watchdog to return to Long Window via `Pmic_wdgSetCfg()`. Once return to
 * Long Window is enabled, watchdog enters Long Window at the end of the next
 * Window.
 *
 * Once in Long Window mode, the user could set WD_PWRHOLD to stay in Long Window.
 *
 * @note There are three prerequisites that must be met for this API.
 * 1. **Implementation of the triggers are left to the end-user to help enable
 *    the driver to be platform-agnostic and allow flexibility for the
 *    end-user**. The triggers can be a PWM or a periodic GPIO pulse to
 *    TRIG_WDOG (configurable function of GPIO2 pin for TPS6522x). This
 *    prerequisite is only applicable when the desired WDG mode of operation is
 *    Trigger mode.
 * 2. **This API must be called during Long Window so that the API can
 *    configure the Watchdog appropriately for Trigger mode or Q&A mode**. The
 *    PMIC should already be in Long Window when power is first supplied to the
 *    PMIC. However, the end-user could return to Long Window by using
 *    `Pmic_wdgSetCfg()`.
 * 3. **PMIC Watchdog must be configured appropriately with respect to
 *    end-user's application before calling this API by using
 *    `Pmic_wdgSetCfg()`**. If the Watchdog is not configured appropriately,
 *    there may be errors that occur. For example:
 *     - If WDG is operating in Trigger mode and the Long Window duration is
 *       shorter than the trigger pulse duration, there will be a Long Window
 *       timeout error.
 *     - If Window-1 and/or Window-2 durations are too short or too long, there
 *       could be a timeout error, early trigger error (Trigger mode WDG), or
 *       early answer error (Q&A mode WDG). For list of configurable Watchdog
 *       options, refer to @ref Pmic_WdgCfg_t struct and its members.
 *
 * @param handle [IN] PMIC interface handle
 * @param mode   [IN] Desired WDG mode during the sequences. For valid values,
 *        see @ref Pmic_WdgTriggerQAMode.
 *
 * @return Success code if WDG trigger sequences have been initiated and
 * PMIC WDG is ready to receive triggers from MCU, error code otherwise. For
 * possible success/error codes, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgBeginSequences(Pmic_CoreHandle_t *handle, const uint8_t mode);

#endif /* __PMIC_WDG_H__ */

/** @} */
