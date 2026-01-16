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
#ifndef PMIC_FSM_H
#define PMIC_FSM_H

/**
 * @file pmic_fsm.h
 *
 * @brief PMIC finite state machine (FSM) interface. Contains APIs, macros/defines,
 * and data structures used to configure, control, and interact with the PMIC FSM.
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
 * @anchor Pmic_FsmGpioPinNum
 * @name PMIC FSM GPIO Pin Number
 *
 * @brief GPIO pins relating to GPIO trigger configuration.
 *
 * @{
 */
#define PMIC_FSM_GPIO_PIN1    (1U)
#define PMIC_FSM_GPIO_PIN2    (2U)
#define PMIC_FSM_GPIO_PIN3    (3U)
#define PMIC_FSM_GPIO_PIN4    (4U)
#define PMIC_FSM_GPIO_PIN5    (5U)
#define PMIC_FSM_GPIO_PIN6    (6U)
#define PMIC_FSM_GPIO_PIN_MIN ((uint8_t)PMIC_FSM_GPIO_PIN1)
#define PMIC_FSM_GPIO_PIN_MAX ((uint8_t)PMIC_FSM_GPIO_PIN6)
/** @} */

/**
 * @anchor Pmic_FsmTriggers
 * @name PMIC FSM Triggers
 *
 * @brief Possible triggers for SEVERE_ERR, MODERATE_ERR_TRIG, MCU_RAIL_TRIG,
 * SOC_RAIL_TRIG, and OTHER_RAIL_TRIG.
 *
 * @{
 */
#define PMIC_FSM_TRIGGER_IMMEDIATE_SHUTDOWN (0U)
#define PMIC_FSM_TRIGGER_ORDERLY_SHUTDOWN   (1U)
#define PMIC_FSM_TRIGGER_MCU_POWER_ERROR    (2U)
#define PMIC_FSM_TRIGGER_SOC_POWER_ERROR    (3U)
#define PMIC_FSM_TRIGGER_MIN                ((uint8_t)PMIC_FSM_TRIGGER_IMMEDIATE_SHUTDOWN)
#define PMIC_FSM_TRIGGER_MAX                ((uint8_t)PMIC_FSM_TRIGGER_SOC_POWER_ERROR)
/** @} */

/**
 * @anchor Pmic_FsmGpioMaskPol
 * @name PMIC FSM GPIO Mask Polarity
 *
 * @brief Possible mask polarities for GPIOs.
 *
 * @details Sets the signal level of a GPIO when it is masked. Setting a GPIO's mask
 * polarity to `PMIC_FSM_GPIO_MASK_POL_0` will set the GPIO signal value to 0. Setting
 * a GPIO's mask polarity to `PMIC_FSM_GPIO_MASK_POL_1` will set the GPIO signal level
 * to 1.
 *
 * @{
 */
#define PMIC_FSM_GPIO_MASK_POL_0   (0U)
#define PMIC_FSM_GPIO_MASK_POL_1   (1U)
#define PMIC_FSM_GPIO_MASK_POL_MIN ((uint8_t)PMIC_FSM_GPIO_MASK_POL_0)
#define PMIC_FSM_GPIO_MASK_POL_MAX ((uint8_t)PMIC_FSM_GPIO_MASK_POL_1)
/** @} */

/**
 * @anchor Pmic_FsmRecovCntThr
 * @name PMIC FSM Recovery Counter Threshold
 *
 * @brief Range of values for the recovery counter threshold.
 *
 * @{
 */
#define PMIC_FSM_RECOV_CNT_THR_MIN ((uint8_t)0x0U)
#define PMIC_FSM_RECOV_CNT_THR_MAX ((uint8_t)0xFU)
/** @} */

/**
 * @anchor Pmic_FsmStartupDest
 * @name PMIC FSM Startup Destination
 *
 * @brief Range of values for PMIC FSM startup destination. Refer to device
 * datasheet for a description of each destination.
 *
 * @{
 */
#define PMIC_FSM_STARTUP_DEST_STANDBY  (0U)
#define PMIC_FSM_STARTUP_DEST_MCU_ONLY (2U)
#define PMIC_FSM_STARTUP_DEST_ACTIVE   (3U)
#define PMIC_FSM_START_UP_DEST_MIN     ((uint8_t)PMIC_FSM_STARTUP_DEST_STANDBY)
#define PMIC_FSM_START_UP_DEST_MAX     ((uint8_t)PMIC_FSM_STARTUP_DEST_ACTIVE)
/** @} */

/**
 * @anchor Pmic_FsmTriggerCfgValidParam
 * @name PMIC FSM Trigger Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_FsmTriggerCfg_t`.
 * Set the `validParams` member of `Pmic_FsmTriggerCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_FSM_SEVERE_ERR_TRIG_VALID   (1U << 0U)
#define PMIC_FSM_OTHER_RAIL_TRIG_VALID   (1U << 1U)
#define PMIC_FSM_SOC_RAIL_TRIG_VALID     (1U << 2U)
#define PMIC_FSM_MCU_RAIL_TRIG_VALID     (1U << 3U)
#define PMIC_FSM_MODERATE_ERR_TRIG_VALID (1U << 4U)
/** @} */

/**
 * @anchor Pmic_FsmGpioTriggerCfgValidParams
 * @name PMIC FSM GPIO Trigger Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_FsmGpioTriggerCfg_t`.
 * Set the `validParams` member of `Pmic_FsmGpioTriggerCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_FSM_MASK_VALID     (1U << 0U)
#define PMIC_FSM_MASK_POL_VALID (1U << 1U)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_FsmTriggerCfg
 * @name PMIC FSM Trigger Configuration Structure
 *
 * @brief Structure used to set and get FSM trigger configurations.
 *
 * @note This structure does not support setting the trigger configuration for GPIOs;
 * For setting GPIO trigger configurations, refer to @ref Pmic_FsmGpioTriggerCfg.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_FsmTriggerCfgValidParam.
 *
 * @param severeErrTrig PMIC device behavior for Severe Error. For valid values,
 * refer to @ref Pmic_FsmTriggers.
 *
 * @param moderateErrTrig PMIC device behavior for Moderate Error. For valid
 * values, refer to @ref Pmic_FsmTriggers.
 *
 * @param mcuRailTrig PMIC device behavior for MCU rail group. For valid values,
 * refer to @ref Pmic_FsmTriggers.
 *
 * @param socRailTrig PMIC device behavior for SOC rail group. For valid values,
 * refer to @ref Pmic_FsmTriggers.
 *
 * @param otherRailTrig PMIC device behavior for OTHER rail group. For valid
 * values, refer to @ref Pmic_FsmTriggers.
 *
 * @{
 */
typedef struct Pmic_FsmTriggerCfg_s {
    uint32_t validParams;

    uint8_t severeErrTrig;
    uint8_t moderateErrTrig;
    uint8_t mcuRailTrig;
    uint8_t socRailTrig;
    uint8_t otherRailTrig;
} Pmic_FsmTriggerCfg_t;
/** @} */

/**
 * @anchor Pmic_FsmGpioTriggerCfg
 * @name PMIC FSM GPIO Trigger Configuration Structure
 *
 * @brief Structure used to set and get GPIO trigger configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_FsmGpioTriggerCfgValidParams.
 *
 * @param pinNum GPIO pin number.
 *
 * @param mask This parameter set to true will enable the FSM trigger mask for
 * the GPIO. Otherwise, if the parameter is set to false, the FSM trigger mask
 * will be disabled.
 *
 * @param maskPol FSM trigger masking polarity select for the GPIO. For valid
 * values, refer to @ref Pmic_FsmGpioMaskPol.
 *
 * @{
 */
typedef struct Pmic_FsmGpioTriggerCfg_s {
    uint32_t validParams;
    uint8_t pinNum;

    bool mask;
    uint8_t maskPol;
} Pmic_FsmGpioTriggerCfg_t;
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Set PMIC trigger configurations. A trigger is a signal that triggers an
 * event in the PMIC state machine.
 *
 * Design: PMICDRV-689
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-540
 *
 * @note This API does not set the GPIO trigger configurations. Refer to
 * @ref `Pmic_fsmSetGpioTriggerCfg()` API for setting such configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param triggerCfg [IN] Desired trigger configurations to set. For more
 * information on trigger configurations, refer to @ref Pmic_FsmTriggerCfg.
 *
 * @return PMIC_ST_SUCCESS if trigger configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmSetTriggerCfg(const Pmic_Handle_t *handle, const Pmic_FsmTriggerCfg_t *triggerCfg);

/**
 * @brief Get PMIC trigger configurations. A trigger is a signal that triggers an
 * event in the PMIC state machine.
 *
 * Design: PMICDRV-690
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-540
 *
 * @note This API does not get the GPIO trigger configurations. Refer to
 * @ref `Pmic_fsmGetGpioTriggerCfg()` API for getting such configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param triggerCfg [OUT] Trigger configurations obtained from the PMIC. For
 * more information on trigger configurations, refer to @ref Pmic_FsmTriggerCfg.
 *
 * @return PMIC_ST_SUCCESS if trigger configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmGetTriggerCfg(const Pmic_Handle_t *handle, Pmic_FsmTriggerCfg_t *triggerCfg);

/**
 * @brief Set PMIC GPIO trigger configurations. A GPIO can be used to trigger an
 * event in the PMIC state machine.
 *
 * Design: PMICDRV-691
  * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-540
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioTriggerCfg [IN] Desired GPIO trigger configurations to set. For
 * more information on GPIO trigger configurations, refer to
 * @ref Pmic_FsmGpioTriggerCfg.
 *
 * @return PMIC_ST_SUCCESS if GPIO trigger configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmSetGpioTriggerCfg(const Pmic_Handle_t *handle, const Pmic_FsmGpioTriggerCfg_t *gpioTriggerCfg);

/**
 * @brief Get PMIC GPIO trigger configurations. A GPIO can be used to trigger an
 * event in the PMIC state machine.
 *
 * Design: PMICDRV-692
  * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521
 *               PMICDRV-522, PMICDRV-528, PMICDRV-540
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioTriggerCfg [OUT] GPIO trigger configurations obtained from the
 * PMIC. For more information on GPIO trigger configurations, refer to
 * @ref Pmic_FsmGpioTriggerCfg.
 *
 * @return PMIC_ST_SUCCESS if GPIO trigger configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmGetGpioTriggerCfg(const Pmic_Handle_t *handle, Pmic_FsmGpioTriggerCfg_t *gpioTriggerCfg);

/**
 * @brief Get value of the PMIC recovery counter, which is a counter on the PMIC
 * that increments each time the PMIC goes through warm reset.
 *
 * Design: PMICDRV-693
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528,
 *               PMICDRV-540, PMICDRV-548
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param recovCnt [OUT] Recovery counter value obtained from the PMIC.
 *
 * @return PMIC_ST_SUCCESS if recovery counter value has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmGetRecovCnt(const Pmic_Handle_t *handle, uint8_t *recovCnt);

/**
 * @brief Clear PMIC recovery counter, which is a counter on the PMIC that
 * increments each time the PMIC goes through warm reset.
 *
 * Design: PMICDRV-694
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-540
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if recovery counter value has been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmClrRecovCnt(const Pmic_Handle_t *handle);

/**
 * @brief Set PMIC recovery counter threshold. When the recovery counter on the
 * PMIC reaches this threshold, the RECOV_CNT_INT interrupt on the PMIC asserts
 * and immediate power-down of all supply rails occur.
 *
 * Design: PMICDRV-695
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-540
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param recovCntThr [IN] Desired recovery counter threshold value to set.
 *
 * @return PMIC_ST_SUCCESS if recovery counter threshold has been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmSetRecovCntThr(const Pmic_Handle_t *handle, uint8_t recovCntThr);

/**
 * @brief Get PMIC recovery counter threshold. When the recovery counter on the
 * PMIC reaches this threshold, the RECOV_CNT_INT interrupt on the PMIC asserts
 * and immediate power-down of all supply rails occur.
 *
 * Design: PMICDRV-696
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-540
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param recovCntThr [OUT] Recovery counter threshold value obtained from the
 * PMIC.
 *
 * @return PMIC_ST_SUCCESS if the PMIC recovery counter threshold value has been
 * obtained, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmGetRecovCntThr(const Pmic_Handle_t *handle, uint8_t *recovCntThr);

/**
 * @brief Send SOFT_REBOOT request to the PMIC.
 *
 * Design: PMICDRV-697
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-540
 *
 * @details Upon sending the SOFT_REBOOT request, the PMIC will undergo ORDERLY
 * SHUTDOWN and automatically re-start to its configured startup destination.
 * To dynamically set the startup destination, see `Pmic_fsmSetStartupDest()`
 * API.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if SOFT_REBOOT request has been sent to the PMIC,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmSendSoftRebootReq(const Pmic_Handle_t *handle);

/**
 * @brief Set PMIC FSM startup destination.
 *
 * Design: PMICDRV-698
  * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-540
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param destination [IN] Desired startup destination to set. For more
 * information on possible destinations, refer to @ref Pmic_FsmStartupDest.
 *
 * @return PMIC_ST_SUCCESS if FSM startup destination has been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmSetStartupDest(const Pmic_Handle_t *handle, uint8_t destination);

/**
 * @brief Get PMIC FSM startup destination.
 *
 * Design: PMICDRV-699
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-540
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param destination [OUT] Startup destination obtained from the PMIC. For more
 * information on possible destinations, refer to @ref Pmic_FsmStartupDest.
 *
 * @return PMIC_ST_SUCCESS if FSM startup destination has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_fsmGetStartupDest(const Pmic_Handle_t *handle, uint8_t *destination);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_FSM_H */
