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
#ifndef __PMIC_FSM_H__
#define __PMIC_FSM_H__

/**
 * @file pmic_fsm.h
 * @brief PMIC Driver FSM API/Interface
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
 * @anchor Pmic_FsmCfgValidParams
 * @name PMIC FSM Configuration Valid Parameters
 *
 * @brief Valid parameters of the FSM configuration struct (Pmic_FsmCfg_t).
 *
 * @{
 */
#define PMIC_CFG_STBY_EN_VALID                      (0U)
#define PMIC_CFG_AUTO_BIST_EN_VALID                 (1U)
#define PMIC_CFG_NRST_ACTIVE_IN_STBY_SEQ_VALID      (2U)
#define PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID    (3U)
#define PMIC_CFG_PWD_THR_VALID                      (4U)
#define PMIC_CFG_NRST_EXT_VALID                     (5U)
#define PMIC_CFG_RST_MCU_TMO_VALID                  (6U)
#define PMIC_CFG_SAFE_TMO_VALID                     (7U)
#define PMIC_CFG_SAFE_LOCK_THR_VALID                (8U)
#define PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID          (9U)
/** @} */

/**
 * @anchor Pmic_FsmCfgValidParamShifts
 * @name PMIC FSM Configuration Valid Parameter Shifts
 *
 * @brief Valid Parameter shift values of the FSM configuration struct
 * (Pmic_FsmCfg_t).
 *
 * @{
 */
#define PMIC_CFG_STBY_EN_VALID_SHIFT                    (1U << PMIC_CFG_STBY_EN_VALID)
#define PMIC_CFG_AUTO_BIST_EN_VALID_SHIFT               (1U << PMIC_CFG_AUTO_BIST_EN_VALID)
#define PMIC_CFG_NRST_ACTIVE_IN_STBY_SEQ_VALID_SHIFT    (1U << PMIC_CFG_NRST_ACTIVE_IN_STBY_SEQ_VALID)
#define PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID_SHIFT  (1U << PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID)
#define PMIC_CFG_PWD_THR_VALID_SHIFT                    (1U << PMIC_CFG_PWD_THR_VALID)
#define PMIC_CFG_NRST_EXT_VALID_SHIFT                   (1U << PMIC_CFG_NRST_EXT_VALID)
#define PMIC_CFG_RST_MCU_TMO_VALID_SHIFT                (1U << PMIC_CFG_RST_MCU_TMO_VALID)
#define PMIC_CFG_SAFE_TMO_VALID_SHIFT                   (1U << PMIC_CFG_SAFE_TMO_VALID)
#define PMIC_CFG_SAFE_LOCK_THR_VALID_SHIFT              (1U << PMIC_CFG_SAFE_LOCK_THR_VALID)
#define PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID_SHIFT        (1U << PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID)
/** @} */

/**
 * @anchor Pmic_FsmWakeupCfgValidParams
 * @name PMIC FSM Wakeup Configuration Valid Parameters
 *
 * @brief Valid parameters of the FSM wakeup configuration struct
 * (Pmic_FsmWakeupCfg_t).
 *
 * @{
 */
#define PMIC_CFG_WAKE1_EVENT_VALID  (0U)
#define PMIC_CFG_WAKE2_EVENT_VALID  (1U)
#define PMIC_CFG_WAKE1_DGL_VALID    (2U)
#define PMIC_CFG_WAKE2_DGL_VALID    (3U)
/** @} */

/**
 * @anchor Pmic_FsmWakeupCfgValidParamShifts
 * @name PMIC FSM Wakeup Configuration Valid Parameter Shifts
 *
 * @brief Valid parameter shift values of the FSM wakeup configuration struct
 * (Pmic_FsmWakeupCfg_t).
 *
 * @{
 */
#define PMIC_CFG_WAKE1_EVENT_VALID_SHIFT    (1U << PMIC_CFG_WAKE1_EVENT_VALID)
#define PMIC_CFG_WAKE2_EVENT_VALID_SHIFT    (1U << PMIC_CFG_WAKE2_EVENT_VALID)
#define PMIC_CFG_WAKE1_DGL_VALID_SHIFT      (1U << PMIC_CFG_WAKE1_DGL_VALID)
#define PMIC_CFG_WAKE2_DGL_VALID_SHIFT      (1U << PMIC_CFG_WAKE2_DGL_VALID)
/** @} */

/**
 * @anchor Pmic_FsmPwrLatchCfgValidParams
 * @name PMIC FSM Power Latch Configuration Valid Parameters
 *
 * @brief Valid parameters of the FSM power latch configuration struct
 * (Pmic_FsmPwrLatchCfg_t).
 *
 * @{
 */
#define PMIC_CFG_PWD_DLY_VALID                      (0U)
#define PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID  (1U)
#define PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID          (2U)
#define PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID          (3U)
/** @} */

/**
 * @anchor Pmic_FsmPwrLatchCfgValidParamShifts
 * @name PMIC FSM Power Latch Configuration Valid Parameter Shifts
 *
 * @brief Valid parameter shift values of the FSM power latch configuration
 * struct (Pmic_FsmPwrLatchCfg_t).
 *
 * @{
 */
#define PMIC_CFG_PWD_DLY_VALID_SHIFT                        (1U << PMIC_CFG_PWD_DLY_VALID)
#define PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID_SHIFT    (1U << PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID)
#define PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT            (1U << PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID)
#define PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT            (1U << PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID)
/** @} */

/**
 * @anchor Pmic_FsmPwrLatchValidParams
 * @name PMIC FSM Power Latch Valid Parameters
 *
 * @brief Valid parameters of the FSM power latch struct.
 *
 * @{
 */
#define PMIC_CFG_STBY_ERR_WAKE_LATCH_VALID  (0U)
#define PMIC_CFG_STBY_TMR_WAKE_LATCH_VALID  (1U)
#define PMIC_CFG_M_PMIC_WAKE_LATCH_VALID    (2U)
#define PMIC_CFG_WAKE1_LATCH_VALID          (3U)
#define PMIC_CFG_WAKE2_LATCH_VALID          (4U)
/** @} */

/**
 * @anchor Pmic_FsmPwrLatchValidParamShifts
 * @name PMIC FSM Power Latch Valid Parameter Shifts
 *
 * @brief Valid parameter shift values of the FSM power latch struct.
 *
 * @{
 */
#define PMIC_CFG_STBY_ERR_WAKE_LATCH_VALID_SHIFT    (1U << PMIC_CFG_STBY_ERR_WAKE_LATCH_VALID)
#define PMIC_CFG_STBY_TMR_WAKE_LATCH_VALID_SHIFT    (1U << PMIC_CFG_STBY_TMR_WAKE_LATCH_VALID)
#define PMIC_CFG_M_PMIC_WAKE_LATCH_VALID_SHIFT      (1U << PMIC_CFG_M_PMIC_WAKE_LATCH_VALID)
#define PMIC_CFG_WAKE1_LATCH_VALID_SHIFT            (1U << PMIC_CFG_WAKE1_LATCH_VALID)
#define PMIC_CFG_WAKE2_LATCH_VALID_SHIFT            (1U << PMIC_CFG_WAKE2_LATCH_VALID)
/** @} */

/**
 * @anchor Pmic_PwdThrMaxVal
 * @name PMIC Power Down Threshold Maximum Value
 *
 * @brief Maximum value of the Power Down Threshold.
 *
 * @{
 */
#define PMIC_PWD_THR_MAX (0x1FU)
/** @} */

/**
 * @anchor Pmic_nRstExtMaxVal
 * @name PMIC nRST Extension Maximum Value
 *
 * @brief Maximum value of nRST extension time.
 *
 * @{
 */
#define PMIC_NRST_EXT_MAX (0xFU)
/** @} */

/**
 * @anchor Pmic_safeTmoMaxVal
 * @name PMIC Safe Timeout Maximum Value
 *
 * @brief Maximum value of Safe state timeout.
 *
 * @{
 */
#define PMIC_SAFE_TMO_MAX (7U)
/** @} */

/**
 * @anchor Pmic_SafeLockThrMaxVal
 * @name PMIC Safe Lock Threshold Maximum Value
 *
 * @brief Maximum value of the Safe Lock Threshold.
 *
 * @{
 */
#define PMIC_SAFE_LOCK_THR_MAX (0x1FU)
/** @} */

/**
 * @anchor Pmic_DevErrCntMax
 * @name PMIC Device Error Count Maximum Value
 *
 * @brief Maximum value of DEV_ERR_CNT.
 *
 * @{
 */
#define PMIC_DEV_ERR_CNT_MAX (0x1FU)
/** @} */

/**
 * @anchor Pmic_rstMcuTmo
 * @name PMIC RESET-MCU State Timeout
 *
 * @brief Valid values of the RESET-MCU state timeout configuration.
 *
 * @{
 */
#define PMIC_RST_MCU_TMO_512_MS         (0x0U)
#define PMIC_RST_MCU_TMO_1024_MS        (0x1U)
#define PMIC_RST_MCU_TMO_2048_MS        (0x2U)
#define PMIC_RST_MCU_TMO_INFINITE_MS    (0x3U)
#define PMIC_RST_MCU_TMO_MAX            (PMIC_RST_MCU_TMO_INFINITE_MS)
/** @} */

/**
 * @anchor Pmic_VbatStbyEntryThr
 * @name PMIC VBAT Standby Entry Threshold
 *
 * @brief Valid values of the vbatStbyEntryThr parameter.
 *
 * @{
 */
#define PMIC_VBAT_STBY_ENTRY_THR_2P4    (0U)
#define PMIC_VBAT_STBY_ENTRY_THR_3P1    (1U)
#define PMIC_VBAT_STBY_ENTRY_THR_3P6    (2U)
#define PMIC_VBAT_STBY_ENTRY_THR_4P1    (3U)
#define PMIC_VBAT_STBY_ENTRY_THR_4P6    (4U)
#define PMIC_VBAT_STBY_ENTRY_THR_5P1    (5U)
#define PMIC_VBAT_STBY_ENTRY_THR_5P6    (6U)
#define PMIC_VBAT_STBY_ENTRY_THR_6P1    (7U)
#define PMIC_VBAT_STBY_ENTRY_THR_MAX    (PMIC_VBAT_STBY_ENTRY_THR_6P1)
/** @} */

/**
 * @anchor Pmic_WakeEvent
 * @name PMIC Wake Event
 *
 * @brief Valid values of wake1Event and wake2Event.
 *
 * @{
 */
#define PMIC_WAKE_HIGH_LEVEL                    (0U)
#define PMIC_WAKE_RISING_EDGE_PLUS_HIGH_LEVEL   (1U)
#define PMIC_WAKE_EVENT_MAX                     (PMIC_WAKE_RISING_EDGE_PLUS_HIGH_LEVEL)
/** @} */

/**
 * @anchor Pmic_WakeDgl
 * @name PMIC Wake Deglitch
 *
 * @brief Valid values of wake1Dgl and wake2Dgl.
 *
 * @{
 */
#define PMIC_WAKE_DEGLITCH_TIME_2_MS    (0U)
#define PMIC_WAKE_DEGLITCH_TIME_16_MS   (1U)
#define PMIC_WAKE_DEGLITCH_TIME_MAX     (PMIC_WAKE_DEGLITCH_TIME_16_MS)
/** @} */

/**
 * @anchor Pmic_PwdDly
 * @name PMIC Power Down Delay
 *
 * @brief Valid values of Power Down configuration.
 *
 * @{
 */
#define PMIC_PWD_DLY_NONE       (0U)
#define PMIC_PWD_DLY_256_US     (1U)
#define PMIC_PWD_DLY_2P048_MS   (2U)
#define PMIC_PWD_DLY_4P096_MS   (3U)
#define PMIC_PWD_DLY_MAX        (PMIC_PWD_DLY_4P096_MS)
/** @} */

/**
 * @anchor Pmic_StateReq
 * @name PMIC State Requests
 *
 * @brief Valid values that can be passed as input to the Pmic_fsmSetDevState()
 * API.
 *
 * @{
 */
#define PMIC_NO_STATE_CHANGE_REQUEST                (0U)
#define PMIC_SAFE_TO_ACTIVE_REQUEST                 (1U)
#define PMIC_ACTIVE_TO_SAFE_REQUEST                 (2U)
#define PMIC_ACTIVE_OR_SAFE_TO_RESET_MCU_REQUEST    (3U)
#define PMIC_STANDBY_REQUEST                        (4U)
#define PMIC_OFF_REQUEST                            (5U)
#define PMIC_STATE_REQUEST_MAX                      (PMIC_OFF_REQUEST)
/** @} */

/**
 * @anchor Pmic_DeviceStates
 * @name PMIC Device States
 *
 * @brief Valid values that can be returned from the Pmic_fsmGetDevState() API.
 *
 * @{
 */
#define PMIC_OFF_STATE              (0x0U)
#define PMIC_INIT_STATE             (0x1U)
#define PMIC_PWRU_SEQ_STATE         (0x5U)
#define PMIC_RESET_MCU_STATE        (0x6U)
#define PMIC_AUTO_BIST              (0x7U)
#define PMIC_ACTIVE_STATE           (0x8U)
#define PMIC_SAFE_STATE             (0x9U)
#define PMIC_RUN_TIME_BIST          (0xAU)
#define PMIC_OTP_PROGRAMMING_STATE  (0xBU)
#define PMIC_PWRD_SEQ_STATE         (0xCU)
#define PMIC_STANDBY_STATE          (0xDU)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_FsmCfg
 * @name PMIC FSM Configuration
 *
 * @brief Struct used to set/get PMIC FSM configurations.
 *
 * @param validParams For valid values see @ref Pmic_FsmCfgValidParamShifts.
 *
 * @param stbyEn Enables low-power STANDBY state instead of OFF state.
 *
 * @param autoBistEn Enable BIST (ABIST, LBIST, RC BIST) to run when the device
 * is in the RESET-MCU state.
 *
 * @param nrstActiveInStbySeq This parameter set to true will enable nRST to be
 * active during STANDBY state and sequencing states. Otherwise, nRST will be
 * pulled low during STANDBY state and sequencing states.
 *
 * @param higherVbatStbyExitThr Enable higher STANDBY state exit threshold
 * and pseudo hysteresis voltage.
 *
 * @param pwdThr Device error-count threshold to power-down the device.
 * Particularly, When the DEV_ERR_CNT[4:0] bit is greater than this threshold,
 * the device goes to the OFF state. For the maximum value, see
 * @ref Pmic_PwdThrMaxVal
 *
 * @param nrstExt Reset extension time. See section 8.19.4.1.2 of the data sheet
 * for more information. For the maximum value, see @ref Pmic_nRstExtMaxVal
 *
 * @param rstMcuTmo RESET-MCU state timeout time. For valid values, see
 * @ref Pmic_rstMcuTmo
 *
 * @param safeTmo SAFE state timeout interval. See section 8.19.4.3.1 of the
 * data sheet for more information. For the maximum value, see
 * @ref Pmic_safeTmoMaxVal
 *
 * @param safeLockThr Safe-lock threshold. The device remains locked in the
 * SAFE state when the value of the device error counter DEV_ERR_CNT[4:0] is
 * greater than this threshold value. For the maximum value, see
 * @ref PMIC_SAFE_LOCK_THR_MAX
 *
 * @param vbatStbyEntryThr VBAT undervoltage threshold for entering the STANDBY
 * state from any of the operating states as the VBAT voltage drops. For valid
 * values, see @ref Pmic_VbatStbyEntryThr
 */
typedef struct Pmic_FsmCfg_s {
    uint32_t validParams;

    bool stbyEn;
    bool autoBistEn;
    bool nrstActiveInStbySeq;
    bool higherVbatStbyExitThr;

    uint8_t pwdThr;
    uint8_t nrstExt;
    uint8_t rstMcuTmo;
    uint8_t safeTmo;
    uint8_t safeLockThr;
    uint8_t vbatStbyEntryThr;
} Pmic_FsmCfg_t;

/**
 * @anchor Pmic_FsmWakeupCfg
 * @name PMIC FSM Wakeup Configuration
 *
 * @brief Struct used to set/get PMIC FSM Wakeup configurations.
 *
 * @param validParams For valid values, see @ref Pmic_FsmWakeupCfgValidParamShifts.
 *
 * @param wake1Event Event in which WAKE1 pin triggers wakeup from STANDBY. For
 * valid values, see @ref Pmic_WakeEvent
 *
 * @param wake2Event Event in which WAKE2 pin triggers wakeup from STANDBY. For
 * valid values, see @ref Pmic_WakeEvent
 *
 * @param wake1Dgl WAKE1 pin deglitch configuration. For valid values, see
 * @ref Pmic_WakeDgl
 *
 * @param wake2Dgl WAKE2 pin deglitch configuration. For valid values, see
 * @ref Pmic_WakeDgl
 */
typedef struct Pmic_FsmWakeupCfg_s {
    uint32_t validParams;

    uint8_t wake1Event;
    uint8_t wake2Event;
    uint8_t wake1Dgl;
    uint8_t wake2Dgl;
} Pmic_FsmWakeupCfg_t;

/**
 * @anchor Pmic_FsmWakeupStat
 * @name PMIC FSM Wakeup Status
 *
 * @brief Struct used to get FSM wakeup statuses.
 *
 * @param wake1Lvl WAKE1 pin level (deglitched). This parameter set to true
 * indicates the WAKE1 pin is high. Otherwise, the pin is low.
 *
 * @param wake2lvl WAKE2 pin level (deglitched). This parameter set to true
 * indicates the WAKE2 pin is high. Otherwise, the pin is low.
 *
 * @param wakeSrc Source of the wakeup event that caused the current power-up
 * cycle.
 */
typedef struct Pmic_FsmWakeupStat_s {
    bool wake1Lvl;
    bool wake2Lvl;
    uint8_t wakeSrc;
} Pmic_FsmWakeupStat_t;

/**
 * @anchor Pmic_FsmPwrLatchCfg
 * @name PMIC FSM Power Latch Configuration
 *
 * @brief Struct used to set/get power latch configurations.
 *
 * @param validParams For valid values, see @ref Pmic_FsmPwrLatchCfgValidParamShifts
 *
 * @param pwdDly Delay from the time the power latches are cleared until device
 * starts PWRD (power-down sequence) into STANDBY or OFF state. For valid values,
 * see @ref Pmic_PwdDly
 *
 * @param stbyErrWakeEventPwrlEn Enable STANDBY state error wakeup events to
 * engage power latch.
 *
 * @param wake1EventPwrlEn Enable WAKE1 pin wakeup events to engage power latch.
 *
 * @param wake2EventPwrlEn Enable WAKE2 pin wakeup events to engage power latch.
 */
typedef struct Pmic_FsmPwrLatchCfg_s {
    uint32_t validParams;

    uint8_t pwdDly;

    bool stbyErrWakeEventPwrlEn;
    bool wake1EventPwrlEn;
    bool wake2EventPwrlEn;
} Pmic_FsmPwrLatchCfg_t;

/**
 * @anchor Pmic_FsmPwrLatch
 * @name PMIC FSM Power Latch
 *
 * @brief Struct used to set/get power latches.
 *
 * @details When all power latches are disabled (i.e., cleared) and WAKEx pins
 * are low, the PMIC enters STANDBY or OFF state when the Power-down Delay time
 * (see \p pwdDly of Pmic_FsmPwrLatchCfg_t) expires.
 *
 * @param validParams For valid values, see @ref Pmic_FsmPwrLatchValidParamShifts
 *
 * @param stbyErrWakeLatch Error in STANDBY state wakeup power-latch, set to 1 on
 * wakeup event from STANDBY state due to a detected thermal, VMON or other error
 * while in STANDBY state on a rail configured for monitoring.
 *
 * @param stbyTmrWakeLatch Timer wakeup power-latch, set to 1 on wakeup event from
 * STANDBY state by the timer.
 *
 * @param mPmicWakeLatch Multi-PMIC wakeup power-latch, set to 1 on wakeup from
 * STANDBY in multi-device configuration.
 *
 * @param wake1Latch WAKE1 pin power-latch, set to 1 on wakeup event from WAKE1
 * pin while WAKE1 power latch is enabled (see \p wake1EventPwrlEn of
 * Pmic_FsmPwrLatchCfg_t).
 *
 * @param wake2Latch WAKE2 pin power-latch, set to 1 on wakeup event from WAKE2
 * pin while WAKE2 power latch is enabled (see \p wake2EventPwrlEn of
 * Pmic_FsmPwrLatchCfg_t).
 */
typedef struct Pmic_FsmPwrLatch_s {
    uint32_t validParams;

    bool stbyErrWakeLatch;
    bool stbyTmrWakeLatch;
    bool mPmicWakeLatch;
    bool wake1Latch;
    bool wake2Latch;
} Pmic_FsmPwrLatch_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Set desired PMIC device state.
 *
 * @details The state change requests in which this API can send are listed below
 * 1. No state change request (PMIC_NO_STATE_CHANGE_REQUEST)
 * 2. SAFE to ACTIVE state request (PMIC_SAFE_TO_ACTIVE_REQUEST)
 * 3. ACTIVE to SAFE request (PMIC_ACTIVE_TO_SAFE_REQUEST)
 * 4. ACTIVE or SAFE to RESET-MCU request (PMIC_ACTIVE_OR_SAFE_TO_RESET_MCU_REQUEST)
 * 5. STANDBY request (PMIC_STANDBY_REQUEST)
 * 6. OFF request (PMIC_OFF_REQUEST)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param state [IN] Device state change request. For valid values, see
 * @ref Pmic_StateReq
 *
 * @return Success code if state change request has been sent, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetDevState(Pmic_CoreHandle_t *handle, uint8_t state);

/**
 * @brief Get PMIC device state.
 *
 * @details The device states that can be returned from this API are listed
 * below.
 * 1. OFF state
 * 2. INIT state
 * 3. PWRU-SEQ state
 * 4. RESET-MCU state
 * 5. AUTO_BIST, an extension of RESET_MCU state
 * 6. ACTIVE state
 * 7. SAFE state
 * 8. Run time BIST, from ACTIVE or SAFE state
 * 9. OTP programming state
 * 10. PWRD-SEQ state
 * 11. STANDBY state
 * 12. OFF state
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param state [OUT] PMIC device state. For valid values returned, see
 * @ref Pmic_DeviceStates
 *
 * @return Success code if PMIC device state is returned, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetDevState(Pmic_CoreHandle_t *handle, uint8_t *state);

/**
 * @brief Set PMIC FSM configurations.
 *
 * @details The options that are configurable via this API are listed below.
 * 1. Standby enable (validParam: Pmic_CFG_STDBY_EN_VALID_SHIFT)
 * 2. AUTO_BIST enable (validParam: PMIC_CFG_AUTO_BIST_EN_VALID_SHIFT)
 * 3. Enable nRST to be active in STANDBY and sequencing states (validParam: PMIC_CFG_NRST_ACTIVE_IN_STBY_SEQ_VALID_SHIFT)
 * 4. Enable higher VBAT exit threshold for STANDBY state (validParam: PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID_SHIFT)
 * 5. Power-down threshold (validParam: PMIC_CFG_PWD_THR_VALID_SHIFT)
 * 6. nRST Extension (validParam: PMIC_CFG_NRST_EXT_VALID_SHIFT)
 * 7. RESET-MCU state timeout (validParam: PMIC_CFG_RST_MCU_TMO_VALID_SHIFT)
 * 8. SAFE state timeout (validParam: PMIC_CFG_SAFE_TMO_VALID_SHIFT)
 * 9. SAFE state lock threshold (validParam: PMIC_CFG_SAFE_LOCK_THR_VALID_SHIFT)
 * 10. VBAT STANDBY state entry threshold (validParam: PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param fsmCfg [IN] PMIC FSM configuration to set.
 *
 * @return Success code if PMIC FSM configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetCfg(Pmic_CoreHandle_t *handle, const Pmic_FsmCfg_t *fsmCfg);

/**
 * @brief Get PMIC FSM configurations. This API supports getting the same
 * configurations that are settable by Pmic_fsmSetCfg().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param fsmCfg [OUT] PMIC FSM configurations obtained from the PMIC.
 *
 * @return Success code if PMIC FSM configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetCfg(Pmic_CoreHandle_t *handle, Pmic_FsmCfg_t *fsmCfg);

/**
 * @brief Set PMIC device error count (DEV_ERR_CNT).
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param devErrCnt [IN] Device error count value to be set.
 *
 * @return Success code if the DEV_ERR_CNT has been set, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetDevErrCnt(Pmic_CoreHandle_t *handle, uint8_t devErrCnt);

/**
 * @brief Get PMIC device error count (DEV_ERR_CNT).
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param devErrCnt [OUT] Device error count value obtained from the PMIC.
 *
 * @return Success code if the DEV_ERR_CNT value has been obtained from the
 * PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetDevErrCnt(Pmic_CoreHandle_t *handle, uint8_t *devErrCnt);

/**
 * @brief Set PMIC Wakeup configurations.
 *
 * @details The options that are configurable through this API are listed below.
 * 1. Power-down delay (validParam: PMIC_CFG_PWD_DLY_VALID_SHIFT)
 * 2. Standby error wakeup event power latch enable (validParam: PMIC_CFG_STBY_ERR_WK_EVENT_PWRL_EN_VALID_SHIFT)
 * 3. WAKE1 event power latch enable (validParam: PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT)
 * 4. WAKE2 event power latch enable (validParam: PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wakeupCfg [IN] PMIC Wakeup configurations to be set.
 *
 * @return Success code if PMIC Wakeup configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetWakeupCfg(Pmic_CoreHandle_t *handle, const Pmic_FsmWakeupCfg_t *wakeupCfg);

/**
 * @brief Get PMIC Wakeup configurations. This API supports getting the same
 * configurations that are settable through Pmic_fsmSetWakeupCfg().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wakeupCfg [OUT] PMIC Wakeup configurations obtained from the PMIC.
 *
 * @return Success code if PMIC Wakeup configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetWakeupCfg(Pmic_CoreHandle_t *handle, Pmic_FsmWakeupCfg_t *wakeupCfg);

/**
 * @brief Get PMIC Wakeup statuses.
 *
 * @details The Wakeup statuses that can be obtained from this API are listed
 * below.
 * 1. WAKE1 pin level
 * 2. WAKE2 pin level
 * 3. WAKE source
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param wakeupStat [OUT] PMIC Wakeup statuses obtained from the PMIC.
 *
 * @return Success code if PMIC Wakeup statuses have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetWakeStatus(Pmic_CoreHandle_t *handle, Pmic_FsmWakeupStat_t *wakeupStat);

/**
 * @brief Set PMIC power latch configurations.
 *
 * @details The options that are configurable through this API are listed below.
 * 1. Power-down delay (validParam: PMIC_CFG_PWD_DLY_VALID_SHIFT)
 * 2. Standby error wakeup event power latch enable (validParam: PMIC_CFG_STBY_ERR_WK_EVENT_PWRL_EN_VALID_SHIFT)
 * 3. WAKE1 event power latch enable (validParam: PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT)
 * 4. WAKE2 event power latch enable (validParam: PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pwrLatchCfg [IN] PMIC power latch configurations to be set.
 *
 * @return Success code if PMIC power latch configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetPwrLatchCfg(Pmic_CoreHandle_t *handle, const Pmic_FsmPwrLatchCfg_t *pwrLatchCfg);

/**
 * @brief Get PMIC power latch configurations. This API supports getting the
 * same configurations that are settable through Pmic_fsmSetPwrLatchCfg().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pwrLatchCfg [OUT] PMIC power latch configurations obtained from the
 * PMIC.
 *
 * @return Success code if PMIC power latch configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetPwrLatchCfg(Pmic_CoreHandle_t *handle, Pmic_FsmPwrLatchCfg_t *pwrLatchCfg);

/**
 * @brief Set PMIC power latches.
 *
 * @details The power latches that can be set using this API are listed below.
 * 1. Standby error wakeup latch (validParam: PMIC_CFG_STBY_ERR_WK_LATCH_VALID_SHIFT)
 * 2. Standby timer wakeup latch (validParam: PMIC_CFG_STBY_TMR_WK_LATCH_VALID_SHIFT)
 * 3. Multiple PMIC wakeup latch (validParam: PMIC_CFG_M_PMIC_WK_LATCH_VALID_SHIFT)
 * 4. WAKE1 pin latch (validParam: PMIC_CFG_WAKE1_LATCH_VALID_SHIFT)
 * 5. WAKE2 pin latch (validParam: PMIC_CFG_WAKE2_LATCH_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pwrLatch [IN] Power latches to be set.
 *
 * @return Success code if PMIC power latches have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetPwrLatch(Pmic_CoreHandle_t *handle, const Pmic_FsmPwrLatch_t *pwrLatch);

/**
 * @brief Get PMIC power latches. This API supports getting the same power
 * latches that are settable through Pmic_fsmSetPwrLatch().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pwrLatch [OUT] Power latches obtained from the PMIC.
 *
 * @return Success code if PMIC power latches have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetPwrLatch(Pmic_CoreHandle_t *handle, Pmic_FsmPwrLatch_t *pwrLatch);

/**
 * @brief Get the duration of the last RESET-MCU event.
 *
 * @details See section 8.19.4.1.3 of the data sheet for more information on
 * deciphering the value that is returned from this API.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param duration [OUT] Duration of the last RESET-MCU event.
 *
 * @return Success code if the duration has been obtained from the PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetLastRstMcuStateDuration(Pmic_CoreHandle_t *handle, uint8_t *duration);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_FSM_H__ */
