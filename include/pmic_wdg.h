/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \defgroup DRV_PMIC_WDG_MODULE PMIC WatchDog Driver API
 *      This Module explains about PMIC WatchDog driver parameters and
 *  APIs usage.
 *  PMIC WatchDog Driver module covers all WatchDog features APIs.
 *  Like, set/get watchdog configuration, Enable  or disable watchdog,
 *  Get watchdog error status, Get watchdog failcount, start watchdog
 *  QA sequence and start watchdog trigger mode.
 *
 *  Supported PMIC devices for Watchdog Module:
 *  1. TPS6594x (Leo PMIC Device)
 *  2. LP8764x  (Hera PMIC Device)
 *
 *  @{
 */

/**
 * \file   pmic_wdg.h
 *
 * \brief  PMIC Low Level Driver API/interface file for WatchDog APIs
 */

#ifndef PMIC_WDG_H_
#define PMIC_WDG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <pmic_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_WdgEnDisableMode
 *  \name PMIC watchdog timer En/Disable Modes
 *
 *  @{
 */
#define PMIC_WDG_DISABLE                     (bool)false
#define PMIC_WDG_ENABLE                      (bool)true
/* @} */

/**
 *  \anchor Pmic_WdgResetEnDisable
 *  \name PMIC watchdog timer warm reset En/Disable
 *
 *  @{
 */
#define PMIC_WDG_RESET_DISABLE               (0x0U)
#define PMIC_WDG_RESET_ENABLE                (0x1U)
/* @} */

/**
 *  \anchor Pmic_WdgReturnLongWinEnDisable
 *  \name PMIC watchdog timer Return Long Window En/Disable
 *
 *  @{
 */
#define PMIC_WDG_RETLONGWIN_DISABLE          (bool)false
#define PMIC_WDG_RETLONGWIN_ENABLE           (bool)true
/* @} */

/**
 *  \anchor Pmic_WdgPwrHoldEnDisable
 *  \name PMIC watchdog timer Power Hold En/Disable
 *
 *  @{
 */
#define PMIC_WDG_PWRHOLD_DISABLE             (0x0U)
#define PMIC_WDG_PWRHOLD_ENABLE              (0x1U)
/* @} */

/**
 *  \anchor Pmic_WdgTriggerQAMode
 *  \name PMIC watchdog timer Trigger/QA Mode
 *
 *  @{
 */
#define PMIC_WDG_TRIGGER_MODE                (0x0U)
#define PMIC_WDG_QA_MODE                     (0x1U)
/* @} */

/**
 *  \anchor Pmic_WdgResetThresholdCount
 *  \name PMIC watchdog timer Reset Threshold Configurations
 *
 *  @{
 */
#define PMIC_WDG_RESET_THRESHOLD_COUNT_0     (0x0U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_1     (0x1U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_2     (0x2U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_3     (0x3U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_4     (0x4U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_5     (0x5U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_6     (0x6U)
#define PMIC_WDG_RESET_THRESHOLD_COUNT_7     (0x7U)
/* @} */

/**
 *  \anchor Pmic_WdgFailThresholdCount
 *  \name PMIC watchdog timer Fail Threshold Configurations
 *
 *  @{
 */
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_0      (0x0U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_1      (0x1U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_2      (0x2U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_3      (0x3U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_4      (0x4U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_5      (0x5U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_6      (0x6U)
#define PMIC_WDG_FAIL_THRESHOLD_COUNT_7      (0x7U)
/* @} */

/**
 *  \anchor Pmic_WdgQaFdbkVal
 *  \name PMIC watchdog timer QA Feedback Values
 *
 *  @{
 */
#define PMIC_WDG_QA_FEEDBACK_VALUE_0         (0x0U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_1         (0x1U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_2         (0x2U)
#define PMIC_WDG_QA_FEEDBACK_VALUE_3         (0x3U)
/* @} */

/**
 *  \anchor Pmic_WdgQaLfsrVal
 *  \name PMIC watchdog timer QA LFSR Values
 *
 *  @{
 */
#define PMIC_WDG_QA_LFSR_VALUE_0             (0x0U)
#define PMIC_WDG_QA_LFSR_VALUE_1             (0x1U)
#define PMIC_WDG_QA_LFSR_VALUE_2             (0x2U)
#define PMIC_WDG_QA_LFSR_VALUE_3             (0x3U)
/* @} */

/**
 *  \anchor Pmic_WdgQaQuestionSeedVal
 *  \name PMIC watchdog timer QA Question Seed Values
 *
 *  @{
 */
#define PMIC_WDG_QA_QUES_SEED_VALUE_0        (0x0U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_1        (0x1U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_2        (0x2U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_3        (0x3U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_4        (0x4U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_5        (0x5U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_6        (0x6U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_7        (0x7U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_8        (0x8U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_9        (0x9U)
#define PMIC_WDG_QA_QUES_SEED_VALUE_10       (0xAU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_11       (0xBU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_12       (0xCU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_13       (0xDU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_14       (0xEU)
#define PMIC_WDG_QA_QUES_SEED_VALUE_15       (0xFU)
/* @} */

/**
 *  \anchor Pmic_WdgCfgValidParamBitPos
 *  \name PMIC watchdog timer Config Structure Param Bit positions
 *
 *  @{
 */
  /** \brief validParams value used to set/get Long Window duration */
#define PMIC_CFG_WDG_LONGWINDURATION_VALID   (0U)
/** \brief validParams value used to set/get Window-1 duration */
#define PMIC_CFG_WDG_WIN1DURATION_VALID      (1U)
/** \brief validParams value used to set/get Window-2 duration */
#define PMIC_CFG_WDG_WIN2DURATION_VALID      (2U)
/** \brief validParams value used to set/get Fail threshold value */
#define PMIC_CFG_WDG_FAILTHRESHOLD_VALID     (3U)
/** \brief validParams value used to set/get Reset threshold Value */
#define PMIC_CFG_WDG_RSTTHRESHOLD_VALID      (4U)
/** \brief validParams value used to set/get to enable or diable warm reset on
 *         fail */
#define PMIC_CFG_WDG_RSTENABLE_VALID         (5U)
/** \brief validParams value used to set/get watchdog mode */
#define PMIC_CFG_WDG_WDGMODE_VALID           (6U)
/** \brief validParams value used to set/get to Enable or disable  watchdog
 *         pwrHold */
#define PMIC_CFG_WDG_PWRHOLD_VALID           (7U)
/** \brief validParams value used to set/get to enable or disable return to long
 *         window */
#define PMIC_CFG_WDG_RETLONGWIN_VALID        (8U)
/** \brief validParams value used to set/get QA feed back value */
#define PMIC_CFG_WDG_QA_FDBK_VALID           (9U)
/** \brief validParams value used to set/get QA LFSR value */
#define PMIC_CFG_WDG_QA_LFSR_VALID           (10U)
/** \brief validParams value used to set/get QA question seed value */
#define PMIC_CFG_WDG_QA_QUES_SEED_VALID      (11U)
/* @} */

/*!
 * \brief  Minimum number of iterations to wait for a Good/Bad event.
 */
#define PMIC_WDG_WAIT_CNT_MIN_VAL                 (30U)

/**
 *  \anchor Pmic_WdgCfgValidParamBitShiftVal
 *  \name PMIC WatchDog Config Structure Params Bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  structure member defined in Pmic_WdgCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT       \
                     (1U << PMIC_CFG_WDG_LONGWINDURATION_VALID)
#define PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT          \
                     (1U << PMIC_CFG_WDG_WIN1DURATION_VALID)
#define PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT          \
                     (1U << PMIC_CFG_WDG_WIN2DURATION_VALID)
#define PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT         \
                     (1U << PMIC_CFG_WDG_FAILTHRESHOLD_VALID)
#define PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT          \
                     (1U << PMIC_CFG_WDG_RSTTHRESHOLD_VALID)
#define PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT             \
                     (1U << PMIC_CFG_WDG_RSTENABLE_VALID)
#define PMIC_CFG_WDG_WDGMODE_VALID_SHIFT               \
                     (1U << PMIC_CFG_WDG_WDGMODE_VALID)
#define PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT               \
                     (1U << PMIC_CFG_WDG_PWRHOLD_VALID)
#define PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT            \
                     (1U << PMIC_CFG_WDG_RETLONGWIN_VALID)
#define PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT               \
                     (1U << PMIC_CFG_WDG_QA_FDBK_VALID)
#define PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT               \
                     (1U << PMIC_CFG_WDG_QA_LFSR_VALID)
#define PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT          \
                     (1U << PMIC_CFG_WDG_QA_QUES_SEED_VALID)
/* @} */

/**
 *  \anchor Pmic_WdgErrStatCfgValidParamBitPos
 *  \name PMIC watchdog timer error status Structure Param Bit positions.
 *
 *  @{
 */
 /** \brief validParams value used to get Long Window timeout error status */
#define PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID     (0U)
/** \brief validParams value used to get Window1 and window2 timeout error
 *         status */
#define PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID             (1U)
/** \brief validParams value used to get Watchdog trigger mode error status */
#define PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID          (2U)
/** \brief validParams value used to get Watchdog early answer error status */
#define PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID          (3U)
/** \brief validParams value used to get Watchdog QA sequence error status */
#define PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID             (4U)
/** \brief validParams value used to get Watchdog QA wrong Answer error status
 */
#define PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID            (5U)
/** \brief validParams value used to get Watchdog fail error status */
#define PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID            (6U)
/** \brief validParams value used to get Watchdog reset error status */
#define PMIC_CFG_WD_RST_INT_ERRSTAT_VALID             (7U)
/* @} */

/**
 *  \anchor Pmic_WdgFailCntStatCfgValidParamBitPos
 *  \name PMIC watchdog Fail count status Structure Param Bit positions.
 *
 *  @{
 */
 /** \brief validParams value used to get status of Bad Event is detected or not
  */
#define PMIC_CFG_WD_BAD_EVENT_STAT_VALID     (0U)
/** \brief validParams value used to get status of Good Event is detected or not
 */
#define PMIC_CFG_WD_GOOD_EVENT_STAT_VALID    (1U)
/** \brief validParams value used to get To get Watchdog Fail Count value */
#define PMIC_CFG_WD_FAIL_CNT_VAL_VALID       (2U)
/* @} */

/**
 *  \anchor Pmic_WdgErrStatValidParamBitShiftVal
 *  \name PMIC WatchDog Error status Structure Params Bit shift values
 *
 *  Application can use below shifted values to set the validParams
 *  structure member defined in Pmic_WdgErrStatus_t structure
 *
 *  @{
 */
#define PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID_SHIFT    \
                            (1U << PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID)
#define PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID_SHIFT            \
                            (1U << PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID)
#define PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID_SHIFT         \
                            (1U << PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID)
#define PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID_SHIFT         \
                            (1U << PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID)
#define PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID_SHIFT            \
                            (1U << PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID)
#define PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID_SHIFT           \
                            (1U << PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID)
#define PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID_SHIFT           \
                            (1U << PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID)
#define PMIC_CFG_WD_RST_INT_ERRSTAT_VALID_SHIFT            \
                            (1U << PMIC_CFG_WD_RST_INT_ERRSTAT_VALID)
/* @} */

/**
 *  \anchor Pmic_WdgFailCntStatValidParamBitShiftVal
 *  \name PMIC WatchDog Fail count status Structure Params Bit shift values
 *
 *  Application can use below shifted values to set the validParams
 *  structure member defined in Pmic_WdgErrStatus_t structure
 *
 *  @{
 */
#define PMIC_CFG_WD_BAD_EVENT_STAT_VALID_SHIFT    \
                            (1U << PMIC_CFG_WD_BAD_EVENT_STAT_VALID)
#define PMIC_CFG_WD_GOOD_EVENT_STAT_VALID_SHIFT   \
                            (1U << PMIC_CFG_WD_GOOD_EVENT_STAT_VALID)
#define PMIC_CFG_WD_FAIL_CNT_VAL_VALID_SHIFT      \
                            (1U << PMIC_CFG_WD_FAIL_CNT_VAL_VALID)

/* @} */

/*!
 * \brief  Macro for PMIC Watchdog QA infinite sequence.
 */
#define PMIC_WD_QA_INFINITE_SEQ (0xFFFFFFFFU)

/**
 *  \anchor Pmic_WdgErrType
 *  \name PMIC WDG Error TYPE
 *
 *  @{
 */
#define PMIC_WDG_ERR_LONG_WIN_TIMEOUT        (0x0U)
#define PMIC_WDG_ERR_TIMEOUT                 (0x1U)
#define PMIC_WDG_ERR_TRIGGER_EARLY           (0x2U)
#define PMIC_WDG_ERR_ANSWER_EARLY            (0x3U)
#define PMIC_WDG_ERR_SEQ_ERR                 (0x4U)
#define PMIC_WDG_ERR_ANS_ERR                 (0x5U)
#define PMIC_WDG_ERR_FAIL_INT                (0x6U)
#define PMIC_WDG_ERR_RST_INT                 (0x7U)
#define PMIC_WDG_ERR_ALL                     (0x8U)
/* @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/*!
 * \brief    PMIC Watchdog configuration structure
 *           Note: validParams is input param for all Set and Get APIs. other
 *           params except validParams is input param for Set APIs and output
 *           param for Get APIs
 *
 * \param   validParams         Selection of structure parameters to be
 *                              set from the combination of the
 *                              \ref Pmic_WdgCfgValidParamBitPos
 *                              and the corresponding member value will be
 *                              updated.
 * \param   longWinDuration_ms  Long Window duration in milli seconds.
 *                              To get more effective results user has to
 *                              program long window with multiples of 3000.
 *                              For PG1.0, the valid range is (100, 3000, 6000,
 *                              9000,....12000, ..., 765000).
 *                              For PG2.0, the valid range is (80, 125, 250,
 *                              375,....8000, 12000, 16000, 20000 ..., 772000).
 * \param   win1Duration_us     Window-1 duration in Micro Seconds.
 *                              To get more effective results user has to
 *                              program window1 with multiples of 550.
 *                              The valid range is (550, 1100, 1650, 2200,
 *                                      2750, ..., 70400).
 * \param   win2Duration_us     Window-2 duration in Micro Seconds.
 *                              To get more effective results user has to
 *                              program window1 with multiples of 550.
 *                              The valid range is (550, 1100, 1650, 2200,
 *                                      2750, ..., 70400).
 * \param   failThreshold       Fail threshold value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgFailThresholdCount.
 * \param   rstThreshold        Reset threshold Value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgResetThresholdCount.
 * \param   wdgMode             Value to set watchdog mode.
 *                              For valid Values:
 *                                   \ref Pmic_WdgTriggerQAMode.
 * \param   pwrHold             Value to Enable or disable  watchdog pwrHold.
 *                              For valid Values:
 *                                   \ref Pmic_WdgPwrHoldEnDisable.
 * \param   rstEnable           To enable or diable warm reset on fail.
 *                              For valid Values:
 *                                   \ref Pmic_WdgResetEnDisable.
 * \param   retLongWin          To enable or disable return to long window
 *                              after completion of the curent sequence.
 *                              For valid Values:
 *                                   \ref Pmic_WdgReturnLongWinEnDisable.
 * \param   qaFdbk              Configure QA feed back value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgQaFdbkVal.
 * \param   qaLfsr              Configure QA LFSR value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgQaLfsrVal.
 * \param   qaQuesSeed          Configure QA question seed value.
 *                              For valid Values:
 *                                   \ref Pmic_WdgQaQuestionSeedVal.
 */
typedef struct Pmic_WdgCfg_s
{
    uint32_t validParams;

    uint32_t  longWinDuration_ms;
    uint32_t  win1Duration_us;
    uint32_t  win2Duration_us;

    uint8_t  failThreshold;
    uint8_t  rstThreshold;

    bool     wdgMode;
    bool     pwrHold;
    bool     rstEnable;
    bool     retLongWin;

    uint8_t  qaFdbk;
    uint8_t  qaLfsr;
    uint8_t  qaQuesSeed;

}Pmic_WdgCfg_t;

/*!
 * \brief    PMIC Watchdog error status structure
 *           Note: validParams is input param for all Get APIs. other params
 *           except validParams is output param for Get APIs
 *
 * \param   validParams         Selection of structure parameters to be
 *                              set from the combination of the
 *                              \ref Pmic_WdgErrStatCfgValidParamBitPos
 *                              and the corresponding member value will be
 *                              updated.
 * \param   wdLongWinTimeout    To get Long Window timeout error status.
 * \param   wdTimeout           To get Window1 and window2 timeout error status.
 * \param   wdTrigEarly         To get Watchdog trigger mode error status.
 * \param   wdAnswearly         To get Watchdog early answer error status.
 * \param   wdSeqErr            To get Watchdog QA sequence error status.
 * \param   wdAnswErr           To get Watchdog QA wrong Answer error status.
 * \param   wdFailInt           To get Watchdog fail error status.
 * \param   wdRstInt            To get Watchdog reset error status.
 */
typedef struct Pmic_WdgErrStatus_s
{
    uint32_t validParams;
    bool wdLongWinTimeout;
    bool wdTimeout;
    bool wdTrigEarly;
    bool wdAnswearly;
    bool wdSeqErr;
    bool wdAnswErr;
    bool wdFailInt;
    bool wdRstInt;
}Pmic_WdgErrStatus_t;

/*!
 * \brief    PMIC Watchdog Fail Count status structure
 *           Note: validParams is input param for all Get APIs. other params
 *           except validParams is output param for Get APIs
 *
 * \param   validParams         Selection of structure parameters to be
 *                              set from the combination of the
 *                              \ref Pmic_WdgFailCntStatCfgValidParamBitPos
 *                              and the corresponding member value will be
 *                              updated.
 * \param   wdBadEvent          To get status of Bad Event is detected or not
 * \param   wdGudEvent          To get status of Good Event is detected or not
 * \param   wdFailCnt           To get Watchdog Fail Count value.
 */
typedef struct Pmic_WdgFailCntStat_s
{
    uint32_t validParams;
    bool     wdBadEvent;
    bool     wdGudEvent;
    uint32_t wdFailCnt;
}Pmic_WdgFailCntStat_t;

/* ========================================================================== */
/*                            Function Declarations                           */
/* ========================================================================== */
/*!
 * \brief   API to Enable Watchdog timer.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to Enable the PMIC watchdog. User ensure
 *          that, this function needs to be called to enable watchdog timer
 *          before configuring or starting watchdog trigger or QA mode.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgEnable(Pmic_CoreHandle_t *pPmicCoreHandle);

/*!
 * \brief   API to Disable Watchdog timer.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to Disable the PMIC watchdog. User ensure
 *          that, after using this function, complete watchdog functionality
 *          and configuration will be deactivated.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgDisable(Pmic_CoreHandle_t *pPmicCoreHandle);

/*!
 * \brief   API to set PMIC watchdog configurations.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854), REQ_TAG(PDK-9115),
 *              REQ_TAG(PDK-9116)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to configure the watchdog parameters
 *          in the PMIC for trigger mode or Q&A(question and answer) mode,
 *          when corresponding validParam bit fields are set in
 *          Pmic_WdgCfg_t structure.
 *          User has to call Pmic_wdgEnable() before set the configuration.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   wdgCfg          [IN]    Watchdog configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgSetCfg(Pmic_CoreHandle_t   *pPmicCoreHandle,
                       const Pmic_WdgCfg_t  wdgCfg);

/*!
 * \brief   API to get PMIC watchdog configurations.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854), REQ_TAG(PDK-9115),
 *              REQ_TAG(PDK-9116)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to get configuration of the watchdog
 *          from the PMIC for trigger mode or Q&A(question and answer) mode,
 *          when corresponding validParam bit fields are set in
 *          Pmic_WdgCfg_t structure.
 *          User has to call Pmic_wdgEnable() before get the configuration.
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   pWdgCfg         [IN/OUT]   Watchdog configuration pointer
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                       Pmic_WdgCfg_t     *pWdgCfg);

/*!
 * \brief   API to Start watchdog QA mode.
 *
 * Requirement: REQ_TAG(PDK-5839)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to start watchdog sequence and continues
 *          till the given num_of_sequences. User has to ensure, configure
 *          all Watchdog QA parameters properly using Pmic_wdgSetCfg() API,
 *          before starting QA sequence using this API.
 *
 *          Note: To perform QA sequences, user has to adjust Long window
 *                time interval, Window1 time interval and Window2 time
 *                intervals depends on errors given by API. If user gets
 *                PMIC_ST_ERR_INV_WDG_WINDOW, then user has to increase the
 *                Long window or window1 time interval. If user gets
 *                PMIC_ST_ERR_WDG_EARLY_ANSWER, then user has to reduce
 *                the Window1 time interval.
 *                Application has to ensure to do proper configuration of WDG
 *                window time intervals. If not configured properly then WDG
 *                will trigger the warm reset to the PMIC device. This may cause
 *                system reset if PMIC is connected to SOC/MCU
 *                Application has to ensure to do proper configuration of WDG
 *                parameters. If not configured properly then API doesn't
 *                receive good or bad event from the PMIC FSM. Due to this API
 *                returns timeout error
 *                API receive bad event due to wrong answer then API detects and
 *                returns an error
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle
 * \param   num_of_sequences [IN]    number of QA sequences
 *                                   If PMIC_WD_QA_INFINITE_SEQ is used,
 *                                   then API runs for infinite sequence.
 * \param   maxCnt           [IN]    Number of iterations to wait for an
 *                                   Good/Bad event. The value should be greater
 *                                   than or equal to PMIC_WDG_WAIT_CNT_MIN_VAL.
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgStartQaSequence(Pmic_CoreHandle_t *pPmicCoreHandle,
                                uint32_t           num_of_sequences,
                                uint32_t           maxCnt);

/*!
 * \brief   API to get PMIC watchdog error status.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to get the watchdog error status from the
 *          PMIC for trigger mode or Q&A(question and answer) mode,
 *          when corresponding validParam bit fields are set in
 *          Pmic_WdgErrStatus_t structure.
 *          User has to call Pmic_wdgEnable() before getting the error status.
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   pErrStatus      [IN/OUT]   Watchdog error status pointer
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetErrorStatus(Pmic_CoreHandle_t   *pPmicCoreHandle,
                               Pmic_WdgErrStatus_t *pErrStatus);

/*!
 * \brief   API to get PMIC watchdog fail count status.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to get the watchdog fail count status from the
 *          PMIC for trigger mode or Q&A(question and answer) mode.
 *          User has to call Pmic_wdgEnable() before getting the fail count.
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   pFailCount      [IN/OUT]   Watchdog fail count pointer
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetFailCntStat(Pmic_CoreHandle_t      *pPmicCoreHandle,
                               Pmic_WdgFailCntStat_t  *pFailCount);

/*!
 * \brief   API to Start watchdog Trigger mode.
 *
 * Requirement: REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to start watchdog trigger mode.
 *          User has to ensure, configure all Watchdog trigger parameters
 *          properly using Pmic_wdgSetCfg() API, before starting watchdog
 *          trigger mode using this API. User can use Pmic_wdgSetCfg() API
 *          to stop watchdog trigger mode.
 *
 *          Note: To perform watchdog trigger mode, user has to
 *                adjust Long window time interval, Window1 time interval
 *                and Window2 time inervals as below, depends on the
 *                time-period of the trigger pulse provided by other
 *                device.
 *                1. Longwindow time interval must be greater than Trigger
 *                   pulse time period.
 *                2. Window1 time interval must be less than T-off time of
 *                   the Trigger pulse time period.
 *                3. Window2 time interval must be greater than T-on time
 *                   of the Trigger pulse time period.
 *                4. (Window1 time interval + Window2 time interval)
 *                   approximately equal to the Trigger pulse time period.
 *                Application has to ensure to do proper configuration of WDG
 *                window time intervals. If not configured properly in Trigger
 *                mode then WDG will trigger the warm reset to the PMIC device.
 *                This may cause system reset if PMIC is connected to SOC/MCU
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgStartTriggerSequence(Pmic_CoreHandle_t *pPmicCoreHandle);

/*!
 * \brief   API to clear PMIC watchdog error status.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to clear the watchdog error status from the
 *          PMIC for trigger mode or Q&A(question and answer) mode,
 *          Note: User has to clear the WDG Error status only when Error status
 *          bit is set for the corresponding wdgErrType
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   wdgErrType      [IN]       Watchdog error type to clear the status
 *                                     For Valid values:
 *                                     \ref Pmic_WdgErrType
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgClrErrStatus(Pmic_CoreHandle_t   *pPmicCoreHandle,
                             const uint8_t        wdgErrType);

/*!
 * \brief   API to Write Answers in Long Window/ Window1/ Window2 Interval for
 *          watchdog QA Sequence.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-9115), REQ_TAG(PDK-9116)
 * Design: did_pmic_wdg_cfg_readback
 * Architecture: aid_pmic_wdg_cfg
 *
 *          This function is used to write Answers in Long Window/ Window1/
 *          Window2 Interval for the WDG QA Sequence
 *          User has to ensure, configure all Watchdog QA parameters properly
 *          using Pmic_wdgSetCfg() API, before writing Answers using this API
 *          for the QA Sequence
 *
 *          Note: To perform QA sequences, user has to adjust Long window
 *                time interval, Window1 time interval and Window2 time
 *                intervals If the Pmic_wdgQaWriteAnswer API returns
 *                PMIC_ST_ERR_INV_WDG_ANSWER error
 *                If the Pmic_wdgQaWriteAnswer API returns
 *                PMIC_ST_ERR_INV_WDG_ANSWER error user has
 *                to call Pmic_wdgGetErrorStatus API to read the WDG error.
 *                If the WDG error is Long Window Timeout or Timeout, user has
 *                to increase the Long window or window1 time interval
 *                accordingly
 *                If the WDG error is Answer early, user has to reduce the
 *                Window1 time interval
 *                For other WDG errors, user has to take action accordingly
 *                Application has to ensure to do proper configuration of WDG
 *                window time intervals. If not configured properly in QA mode
 *                then WDG will trigger the warm reset to the PMIC device. This
 *                may cause system reset if PMIC is connected to SOC/MCU
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgQaSequenceWriteAnswer(Pmic_CoreHandle_t *pPmicCoreHandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_WDG_H_ */

/* @} */
