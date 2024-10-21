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
 * structures, and APIs used to interact with the PMIC Watchdog module.
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
 * @name PMIC Watchdog Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_WdgCfg_t
 * struct. For more information on the parameters, refer to @ref Pmic_WdgCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_WD_RST_EN_VALID            ((uint8_t)1U << 0U)
#define PMIC_WD_MODE_VALID              ((uint8_t)1U << 1U)
#define PMIC_WD_WIN1_DURATION_VALID     ((uint8_t)1U << 2U)
#define PMIC_WD_WIN2_DURATION_VALID     ((uint8_t)1U << 3U)
#define PMIC_WD_QA_FDBK_VALID           ((uint8_t)1U << 4U)
#define PMIC_WD_QA_LFSR_VALID           ((uint8_t)1U << 5U)
#define PMIC_WD_QA_SEED_VALID           ((uint8_t)1U << 6U)
/** @} */

/**
 * @anchor Pmic_WdgQaCfgMaxValues
 * @name PMIC Watchdog Q&A Configuration Max Values
 *
 * @brief Maximum values of WD_QA_FDBK, WD_QA_LFSR, and WD_QUESTION_SEED.
 *
 * @{
 */
#define PMIC_WD_QA_FDBK_MAX             ((uint8_t)3U)
#define PMIC_WD_QA_LFSR_MAX             ((uint8_t)3U)
#define PMIC_WD_QA_SEED_MAX             ((uint8_t)0xFU)
/** @} */

/**
 * @anchor Pmic_WdgWindowMaxValues
 * @name PMIC Watchdog Window Max Values
 *
 * @brief Maximum values of Window-1, Window-2, and Long Window.
 *
 * @{
 */
#define PMIC_WD_WIN1_DURATION_MAX       ((uint8_t)0x7FU)
#define PMIC_WD_WIN2_DURATION_MAX       ((uint8_t)0x1FU)
/** @} */

/**
 * @anchor Pmic_WdgModeValues
 * @name TPS65036x WD_MODE_SELECT Values
 *
 * @brief Valid values of the PMIC WD_MODE_SELECT bit field.
 *
 * @{
 */
#define PMIC_TRIGGER_MODE               ((uint8_t)0U)
#define PMIC_QA_MODE                    ((uint8_t)1U)
#define PMIC_WD_MODE_MAX                (PMIC_QA_MODE)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_WdgCfg
 * @name PMIC Watchdog Configuration struct
 *
 * @brief Struct used to write/read PMIC watchdog configurations.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. for valid values, refer to @ref Pmic_WdgCfgValidParams.
 *
 * @param rstEn Watchdog reset configuration. When enabled, the PMIC transitions
 * to the RESET state when WD_FAIL_CNT[2:0], the watchdog fail counter, equals
 * the value of 7.
 *
 * @param mode Watchdog mode configuration.
 *
 * @param win1Duration Watchdog window-1 duration.
 *
 * @param win2Duration Watchdog window-2 duration.
 *
 * @param qaFdbk qaFdbk Watchdog Q&A feedback configuration. Controls the
 * sequence of generated questions and their respective reference answers.
 *
 * @param qaLfsr Watchdog Q&A LFSR configuration used to generate the
 * questions.
 *
 * @param qaSeed Watchdog Q&A seed used to generate a new starting question.
 */
typedef struct Pmic_WdgCfg_s
{
    uint32_t validParams;

    bool rstEn;
    uint8_t mode;

    uint8_t win1Duration;
    uint8_t win2Duration;

    uint8_t qaFdbk;
    uint8_t qaLfsr;
    uint8_t qaSeed;
} Pmic_WdgCfg_t;

/**
 * @anchor Pmic_WdgErrStat
 * @name PMIC Watchdog Error Status Struct
 *
 * @brief Struct used to get/clear PMIC watchdog error statuses.
 *
 * @param answerErr Applicable only for WDG in Q&A mode. Status/indication of
 * whether WDG received an incorrect answer-byte in the current sequence. This
 * status is only cleared if the next answer-byte sequence is correct.
 *
 * @param answerEarly Applicable only for WDG in Q&A mode. Status/indication of
 * whether all four answer-bytes were received in Window-1. This status is
 * cleared after successful API call of Pmic_wdgGetErrStat().
 *
 * @param seqErr Applicable only for WDG in Q&A mode. Status/indication of
 * whether WDG received an incorrect answer-byte sequence (incorrect timing or
 * incorrect order). this status is cleared after successful API call of
 * Pmic_wdgGetErrStat().
 *
 * @param timeout If WDG is configured for Trigger mode, this status indicates
 * that no trigger event has been received on the ERROR/WDI pin during the
 * duration of window-1 or window-2. If WDG is configured for Q&A mode, this
 * status indicates that all 3 answer bytes have not been received during
 * Window-1 or the final answer byte has not been received in Window-2. This
 * status is cleared after successful API call of Pmic_wdgGetErrStat().
 *
 * @param wdgCfgChg Status/indication of whether there has been a change in the
 * configuration of WDG between Trigger mode and Q&A mode or a change in window
 * durations. This status is cleared after successful API call of
 * Pmic_wdgGetErrStat().
 *
 * @param failThr Status/indication of whether WDG fail counter has exceeded the
 * value of 4. This status is cleared after successful API call of
 * Pmic_wdgGetErrStat().
 */
typedef struct Pmic_WdgErrStat_s
{
    bool answerErr;
    bool answerEarly;
    bool seqErr;
    bool timeout;
    bool wdgCfgChg;
    bool failThr;
} Pmic_WdgErrStat_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Set PMIC watchdog configurations.
 *
 * @details The following options are configurable via this API
 * 1. Reset enable (validParam: PMIC_WD_RST_EN_VALID)
 * 2. Mode of operation (validParam: PMIC_WD_MODE_VALID)
 * 3. Window-1 duration (validParam: PMIC_WD_WIN1_DURATION_VALID)
 * 4. Window-2 duration (validParam: PMIC_WD_WIN2_DURATION_VALID)
 * 5. Q&A feedback (validParam: PMIC_WD_QA_FDBK_VALID)
 * 6. Q&A LFSR (validParam: PMIC_WD_QA_LFSR_VALID)
 * 7. Q&A question seed (validParam: PMIC_WD_QA_SEED_VALID)
 *
 * @attention In order to set watchdog configurations, the PMIC must be in
 * Diagnostic mode and configuration registers must be unlocked. To unlock
 * configuration registers, see Pmic_setRegLock() or Pmic_unlockRegs().
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
 * @brief Get PMIC watchdog configurations. This API supports obtaining the same
 * parameters that are settable through Pmic_wdgSetCfg().
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
 * @brief Send a Q&A answer byte to the PMIC watchdog.
 *
 * @details When the watchdog is operating in Q&A mode, The API should be called
 * three times in Window-1 and one time in Window-2 of each Q&A sequence.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if Q&A answer byte has been sent to the PMIC, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgQaSequenceWriteAnswer(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Get PMIC watchdog error statuses.
 *
 * @details The following watchdog error statuses are obtainable by this API
 * 1. ANSWER_ERR
 * 2. ANSWER_EARLY (status cleared upon API call)
 * 3. SEQ_ERR (status cleared upon API call)
 * 4. TIME_OUT (status cleared upon API call)
 * 5. WD_CFG_CHG (status cleared upon API call)
 * 6. FAIL_THR (status cleared upon API call)
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param wdgErrStat [OUT] Watchdog error statuses obtained from the PMIC.
 *
 * @return Success code if watchdog error statuses have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetErrStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgErrStat_t *wdgErrStat);

/**
 * @brief Get the watchdog fail counter value.
 *
 * @details the PMIC watchdog includes a fail counter, WD_FAIL_CNT[2:0], that
 * increments because of bad events or decrements because of good events. A good
 * event decrements WD_FAIL_CNT[2:0] by one while a bad event increments
 * WD_FAIL_CNT[2:0] by one.
 *
 * When WD_FAIL_CNT[2:0] reaches a value of 7, a new bad event does not change
 * the value. Similarly, when WD_FAIL_CNT[2:0] decrements to a value of 0, a new
 * good event does not change the value.
 *
 * WD_FAIL_CNT[2:0] initializes to a value of 5 after certains events: (1) PMIC
 * goes to RESET state, (2) PMIC goes from RESET to DIAGNOSTIC state, (3) PMIC
 * goes from DIAGNOSTIC state to ACTIVE state, (4) MCU changes the device
 * configuration from ESM-without-watchdog mode to trigger mode or Q&A mode.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param failCnt [OUT] Value of WD_FAIL_CNT[2:0].
 *
 * @return success code if the watchdog fail counter value has been obtained,
 * error code otherwise. for valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_wdgGetFailCnt(const Pmic_CoreHandle_t *pmicHandle, uint8_t *failCntVal);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_WDG_H__ */
