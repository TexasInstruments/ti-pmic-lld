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
 *
 * @brief PMIC watchdog (WDG) interface. Contains APIs, macros/defines, and data
 * structures used to configure, control, and interact with the PMIC WDG.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * @anchor Pmic_WdgMode
 * @name PMIC WDG Mode
 *
 * @brief Range of values for WDG mode.
 *
 * @{
 */
#define PMIC_WDG_TRIGGER_MODE (0U)
#define PMIC_WDG_QA_MODE      (1U)
#define PMIC_WDG_MODE_MIN     ((uint8_t)PMIC_WDG_TRIGGER_MODE)
#define PMIC_WDG_MODE_MAX     ((uint8_t)PMIC_WDG_QA_MODE)
/** @} */

/**
 * @anchor Pmic_WdgWin1Code
 * @name PMIC WDG Window-1 Code
 *
 * @brief Range of values for WDG Window-1.
 *
 * @{
 */
#define PMIC_WDG_WIN1_CODE_MIN ((uint8_t)0x00U)
#define PMIC_WDG_WIN1_CODE_MAX ((uint8_t)0x7FU)
/** @} */

/**
 * @anchor Pmic_WdgWin2Code
 * @name PMIC WDG Window-2 Code
 *
 * @brief Range of values for WDG Window-2.
 *
 * @{
 */
#define PMIC_WDG_WIN2_CODE_MIN ((uint8_t)0x00U)
#define PMIC_WDG_WIN2_CODE_MAX ((uint8_t)0x7FU)
/** @} */

/**
 * @anchor Pmic_WdgLongWinCode
 * @name PMIC WDG Long Window Code
 *
 * @brief Range of values for WDG Long Window.
 *
 * @{
 */
#define PMIC_WDG_LONG_WIN_MIN ((uint8_t)0x00U)
#define PMIC_WDG_LONG_WIN_MAX ((uint8_t)0xFFU)
/** @} */

/**
 * @anchor Pmic_WdgQaFdbk
 * @name PMIC WDG Q&A Feedback
 *
 * @brief Range of values for WDG Q&A feedback.
 *
 * @{
 */
#define PMIC_WDG_QA_FDBK_MIN ((uint8_t)0x0U)
#define PMIC_WDG_QA_FDBK_MAX ((uint8_t)0x3U)
/** @} */

/**
 * @anchor Pmic_WdgQaLfsr
 * @name PMIC WDG Q&A LFSR
 *
 * @brief Range of values for WDG Q&A LFSR.
 *
 * @{
 */
#define PMIC_WDG_QA_LFSR_MIN ((uint8_t)0x0U)
#define PMIC_WDG_QA_LFSR_MAX ((uint8_t)0x3U)
/** @} */

/**
 * @anchor Pmic_WdgQaSeed
 * @name PMIC WDG Q&A Seed
 *
 * @brief Range of values for WDG Q&A seed.
 *
 * @{
 */
#define PMIC_WDG_QA_SEED_MIN ((uint8_t)0x0U)
#define PMIC_WDG_QA_SEED_MAX ((uint8_t)0xFU)
/** @} */

/**
 * @anchor Pmic_WdgFailThr
 * @name PMIC WDG Failure Threshold
 *
 * @brief Range of values for WDG failure threshold.
 *
 * @{
 */
#define PMIC_WDG_FAIL_THR_MIN ((uint8_t)0x0U)
#define PMIC_WDG_FAIL_THR_MAX ((uint8_t)0x7U)
/** @} */

/**
 * @anchor Pmic_WdgRstThr
 * @name PMIC WDG Reset Threshold
 *
 * @brief Range of values for WDG reset threshold.
 *
 * @{
 */
#define PMIC_WDG_RST_THR_MIN ((uint8_t)0x0U)
#define PMIC_WDG_RST_THR_MAX ((uint8_t)0x7U)
/** @} */

/**
 * @anchor Pmic_WdgCntSel
 * @name PMIC WDG Count Selection
 *
 * @brief Range of values for WDG count selection.
 *
 * @{
 */
#define PMIC_WDG_CNT_SEL_1_1 (0U)
#define PMIC_WDG_CNT_SEL_2_1 (1U)
#define PMIC_WDG_CNT_SEL_MIN ((uint8_t)PMIC_WDG_CNT_SEL_1_1)
#define PMIC_WDG_CNT_SEL_MAX ((uint8_t)PMIC_WDG_CNT_SEL_2_1)
/** @} */

/**
 * @anchor Pmic_WdgCfgValidParams
 * @name PMIC WDG Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_WdgCfg_t`.
 * Set the `validParams` member of `Pmic_WdgCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_WDG_RST_EN_VALID        (1UL << 0U)
#define PMIC_WDG_MODE_SEL_VALID      (1UL << 1U)
#define PMIC_WDG_WIN1_CODE_VALID     (1UL << 2U)
#define PMIC_WDG_WIN2_CODE_VALID     (1UL << 3U)
#define PMIC_WDG_LONG_WIN_CODE_VALID (1UL << 4U)
#define PMIC_WDG_QA_FDBK_VALID       (1UL << 5U)
#define PMIC_WDG_QA_LFSR_VALID       (1UL << 6U)
#define PMIC_WDG_QA_SEED_VALID       (1UL << 7U)
#define PMIC_WDG_FAIL_THR_VALID      (1UL << 8U)
#define PMIC_WDG_RST_THR_VALID       (1UL << 9U)
#define PMIC_WDG_CNT_SEL_VALID       (1UL << 10U)
#define PMIC_WDG_EN_DRV_SEL_VALID    (1UL << 11U)
/** @} */

/**
 * @anchor Pmic_WdgErrStatusValidParams
 * @name PMIC WDG Error Status Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_WdgErrStatus_t`.
 * Set the `validParams` member of `Pmic_WdgErrStatus_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_WDG_RST_INT_VALID              (1UL << 0U)
#define PMIC_WDG_FAIL_INT_VALID             (1UL << 1U)
#define PMIC_WDG_ANSW_ERR_VALID             (1UL << 2U)
#define PMIC_WDG_SEQ_ERR_VALID              (1UL << 3U)
#define PMIC_WDG_ANSW_EARLY_ERR_VALID       (1UL << 4U)
#define PMIC_WDG_TRIG_EARLY_ERR_VALID       (1UL << 5U)
#define PMIC_WDG_TIMEOUT_ERR_VALID          (1UL << 6U)
#define PMIC_WDG_LONG_WIN_TIMEOUT_ERR_VALID (1UL << 7U)
/** @} */

/**
 * @anchor Pmic_WdgFailCntStatusValidParams
 * @name PMIC WDG Fail Count Status Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_WdgFailCntStatus_t`.
 * Set the `validParams` member of `Pmic_WdgFailCntStatus_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_WDG_BAD_EVENT_VALID  (1UL << 0U)
#define PMIC_WDG_GOOD_EVENT_VALID (1UL << 1U)
#define PMIC_WDG_FAIL_CNT_VALID   (1UL << 2U)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_WdgCfg
 * @name PMIC WDG Configuration Structure
 *
 * @brief Structure used to set and get WDG configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_WdgCfgValidParams.
 *
 * @param rstEn If set to true, the PMIC will warm reset when the WDG fail counter
 * (WD_FAIL_CNT[3:0]) is greater than the failure threshold + reset threshold
 * (WD_FAIL_TH[2:0] + WD_RST_TH[2:0]). Otherwise, if set to false, the PMIC will
 * not warm reset when the mentioned conditions are met.
 *
 * @param mode WDG mode of operation. For valid values, refer to @ref Pmic_WdgMode.
 *
 * @param win1Code WDG Window-1 duration code. For valid values, refer to
 * @ref Pmic_WdgWin1Code. Refer to the data sheet for converting code to a time
 * duration.
 *
 * @param win2Code WDG Window-2 duration code. For valid values, refer to
 * @ref Pmic_WdgWin2Code. Refer to the data sheet for converting code to a time
 * duration.
 *
 * @param longWinCode WDG Long Window duration code. For valid values, refer
 * to @ref Pmic_WdgLongWinCode. Refer to the data sheet for converting code to
 * a time duration.
 *
 * @param qaFdbk WDG Q&A feedback. Affects how WDG Q&A answer bytes are generated.
 * For valid values, refer to @ref Pmic_WdgQaFdbk.
 *
 * @param qaLfsr WDG Q&A LFSR. Affects how WDG Q&A questions are generated. For
 * valid values, refer to @ref Pmic_WdgQaLfsr.
 *
 * @param qaSeed WDG Q&A seed. Determines the starting WDG Q&A question. For valid
 * values, refer to @ref Pmic_WdgQaSeed.
 *
 * @param failThr WDG Q&A failure threshold, also known as the first threshold of
 * the WDG fail counter. When the WDG fail counter exceeds this threshold, the
 * WD_FAIL_INT error status will assert and the PMIC will clear ENABLE_DRV (if
 * the PMIC has been enabled to do so). For valid values, refer to
 * @ref Pmic_WdgFailThr.
 *
 * @param rstThr WDG Q&A reset threshold, also known as the second threshold of
 * the WDG fail counter. When the WDG fail counter exceeds failure threshold +
 * reset threshold, the WD_RST_INT error status will assert and the PMIC will
 * undergo warm reset (if the PMIC has been enabled to do so). For valid values,
 * refer to @ref Pmic_WdgRstThr.
 *
 * @param cntSel Counting scheme of the WDG fail counter. For valid values, refer
 * to @ref Pmic_WdgCntSel.
 *
 * @param clrEnDrvOnFailInt If set to true, the PMIC will clear EN_DRV when the
 * WDG fail counter exceeds the failure threshold and the WD_FAIL_INT error status
 * asserts. Otherwise, if set to false, EN_DRV will not be cleared when the
 * mentioned conditions are met.
 *
 * @{
 */
typedef struct Pmic_WdgCfg_s {
    uint32_t validParams;

    bool rstEn;
    uint8_t mode;

    uint8_t win1Code;
    uint8_t win2Code;
    uint8_t longWinCode;

    uint8_t qaFdbk;
    uint8_t qaLfsr;
    uint8_t qaSeed;

    uint8_t failThr;
    uint8_t rstThr;

    uint8_t cntSel;

    bool clrEnDrvOnFailInt;
} Pmic_WdgCfg_t;
/** @} */

/**
 * @anchor Pmic_WdgErrStatus
 * @name PMIC WDG Error Status Structure
 *
 * @brief Used to get and clear WDG error statuses.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_WdgErrStatusValidParams.
 *
 * @param rstInt Indicates whether the device has went through warm reset due to
 * the WDG fail counter (WD_FAIL_CNT[3:0]) exceeding the failure threshold
 * (WD_FAIL_TH[2:0]) + reset threshold (WD_RST_TH[2:0]).
 *
 * @param failInt Indicates whether the device has cleared ENABLE_DRV due to the
 * WDG fail counter exceeding the failure threshold.
 *
 * @param answErr Indicates whether the WDG has detected an incorrect answer-byte.
 * Only applicable for WDG operating in Q&A mode.
 *
 * @param seqErr Indicates whether the WDG has detected an incorrect sequence of
 * answer-bytes. Only applicable for WDG operating in Q&A mode.
 *
 * @param answEarlyErr Indicates whether the WDG has received the final answer-byte
 * in Window-1. Only applicable for WDG operating in Q&A mode.
 *
 * @param trigEarlyErr Indicates whether the WDG has received the WDG trigger in
 * Window-1. Only applicable for WDG operating in Trigger mode.
 *
 * @param timeoutErr Indicates whether the WDG has detected a timeout event in the
 * started WDG sequence.
 *
 * @param longWinTimeoutErr Indicates whether the device went through warm reset
 * due to elapse of Long Window time interval.
 *
 * @{
 */
typedef struct Pmic_WdgErrStatus_s {
    uint32_t validParams;

    bool rstInt;
    bool failInt;
    bool answErr;
    bool seqErr;
    bool answEarlyErr;
    bool trigEarlyErr;
    bool timeoutErr;
    bool longWinTimeoutErr;
} Pmic_WdgErrStatus_t;
/** @} */

/**
 * @anchor Pmic_WdgFailCntStatus
 * @name PMIC WDG Fail Count Status Structure
 *
 * @brief Structure used to get the WDG fail count status.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_WdgFailCntStatusValidParams.
 *
 * @param badEvent Indicates whether the WDG has detected a bad event in the
 * current WDG sequence. The PMIC automatically clears this bit at the end of
 * the WDG sequence.
 *
 * @param goodEvent Indicates whether the WDG has detected a good event. The PMIC
 * clears this bit when the WDG goes to the Long Window.
 *
 * @param failCnt Value of the WDG fail counter. The PMIC clears this counter when
 * the WDG goes to Long Window.
 *
 * @{
 */
typedef struct Pmic_WdgFailCntStatus_s {
    uint32_t validParams;

    bool badEvent;
    bool goodEvent;
    uint8_t failCnt;
} Pmic_WdgFailCntStatus_t;
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Enable or disable the PMIC watchdog.
 *
 * Design: PMICDRV-662
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enable [IN] `PMIC_ENABLE` - enable the WDG; `PMIC_DISABLE` - disable
 * the WDG.
 *
 * @return PMIC_ST_SUCCESS if PMIC watchdog has been enabled or disabled, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgSetEnableState(const Pmic_Handle_t *handle, bool enable);

/**
 * @brief Get the enable state of the PMIC watchdog.
 *
 * Design: PMICDRV-663
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] Watchdog enable status. Value is set to true if PMIC
 * watchdog is enabled; else the value is set to false.
 *
 * @return PMIC_ST_SUCCESS if PMIC watchdog enable status has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Set PMIC watchdog configurations.
 *
 * Design: PMICDRV-664
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @attention Watchdog must be in Long Window and enabled before configuration.
 * See `Pmic_wdgSetEnableState()` and `Pmic_wdgSetReturnToLongWindow()` for more
 * information.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wdgCfg [IN] Desired watchdog configurations to set. For more information
 * on watchdog configurations, refer to @ref Pmic_WdgCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC watchdog configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgSetCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg);

/**
 * @brief Get PMIC watchdog configurations.
 *
 * Design: PMICDRV-665
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wdgCfg [OUT] Watchdog configurations obtained from the PMIC. For more
 * information on watchdog configurations, refer to @ref Pmic_WdgCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC watchdog configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg);

/**
 * @brief Set enable state of the PMIC watchdog Power Hold, which controls
 * whether WDG stays in Long Window.
 *
 * Design: PMICDRV-668
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enable [IN] `PMIC_ENABLE` - enable Power Hold;
 * `PMIC_DISABLE` - disable Power Hold.
 *
 * @return PMIC_ST_SUCCESS if watchdog Power Hold has been enabled/disabled,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgSetPowerHold(const Pmic_Handle_t *handle, bool enable);

/**
 * @brief Get enable state of the PMIC watchdog Power Hold, which controls
 * whether WDG stays in Long Window.
 *
 * Design: PMICDRV-669
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] `PMIC_ENABLE` - Power Hold is enabled;
 * `PMIC_DISABLE` - Power Hold is disabled.
 *
 * @return PMIC_ST_SUCCESS if watchdog Power Hold enable state has been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetPowerHold(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Set enable state of the PMIC watchdog Return to Long Window, which
 * controls whether the watchdog returns to Long Window at the end of the
 * current sequence.
 *
 * Design: PMICDRV-670
  * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522
 *               PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enable [IN] `PMIC_ENABLE` - Return to Long Window is enabled;
 * `PMIC_DISABLE` - Return to Long Window is disabled.
 *
 * @return PMIC_ST_SUCCESS if watchdog Return to Long Window has been enabled
 * or disabled, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgSetReturnToLongWindow(const Pmic_Handle_t *handle, bool enable);

/**
 * @brief Get enable state of the PMIC watchdog Return to Long Window, which
 * controls whether the watchdog returns to Long Window at the end of the
 * current sequence.
 *
 * Design: PMICDRV-671
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] `PMIC_ENABLE` - watchdog returns to Long Window at the
 * end of the current sequence; `PMIC_DISABLE` - watchdog does NOT return to Long
 * Window at the end of the current sequence.
 *
 * @return PMIC_ST_SUCCESS if enable state of the watchdog Return to Long Window
 * has been obtained, error code otherwise. For valid success/error codes, refer
 * to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetReturnToLongWindow(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Calculate and send a WDG Q&A answer byte to the PMIC.
 *
 * Design: PMICDRV-676
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @details When the watchdog is operating in Q&A mode, the API should be called
 * four times in Long Window to exit Long Window. For every Q&A sequence thereafter,
 * the API should be called three times in Window-1 and one time in Window-2.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if Q&A answer byte has been sent to the PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgQaWriteAnswer(const Pmic_Handle_t *handle);

/**
 * @brief Clear PMIC watchdog error statuses.
 *
 * Design: PMICDRV-673
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @note To indicate the desired watchdog error status(es) to clear, the
 * validParams struct member of `wdgErrStatus` parameter must be set. All other
 * struct members will be ignored/unused throughout API execution. For valid
 * values of validParams, refer to @ref Pmic_WdgErrStatusValidParams.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wdgErrStatus [IN] The validParams struct member of this parameter
 * indicates the watchdog error status(es) to clear. For more information on
 * watchdog error statuses, refer to @ref Pmic_WdgErrStatus.
 *
 * @return PMIC_ST_SUCCESS if watchdog error status(es) have been cleared, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgClrErrStatus(const Pmic_Handle_t *handle, const Pmic_WdgErrStatus_t *wdgErrStatus);

/**
 * @brief Clear all PMIC watchdog error statuses. Provided as a convenience,
 * however, it is recommended to process watchdog statuses via
 * `Pmic_wdgGetErrStatus()` and `Pmic_wdgClrErrStatus()` APIs.
 *
 * Design: PMICDRV-674
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if all watchdog error statuses have been cleared,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgClrErrStatusAll(const Pmic_Handle_t *handle);

/**
 * @brief Get PMIC watchdog error statuses.
 *
 * Design: PMICDRV-731
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-528, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wdgErrStatus [OUT] Error statuses obtained from the PMIC. For more
 * information on watchdog error statuses, refer to @ref Pmic_WdgErrStatus.
 *
 * @return PMIC_ST_SUCCESS if watchdog error status(es) have been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetErrStatus(const Pmic_Handle_t *handle, Pmic_WdgErrStatus_t *wdgErrStatus);

/**
 * @brief Get PMIC watchdog fail counter statuses.
 *
 * Design: PMICDRV-675
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-538
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wdgFailCntStatus [OUT] Watchdog fail count statuses obtained from the
 * PMIC. For more information on fail count statuses, refer to
 * @ref Pmic_WdgFailCntStatus.
 *
 * @return PMIC_ST_SUCCESS if the watchdog fail counter status(es) have been
 * obtained, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_wdgGetFailCntStatus(const Pmic_Handle_t *handle, Pmic_WdgFailCntStatus_t *wdgFailCntStatus);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_WDG_H */
