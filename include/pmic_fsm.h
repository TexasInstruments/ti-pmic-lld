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
 *  \defgroup DRV_PMIC_FSM_MODULE PMIC FSM Driver API
 *      This Module explains about PMIC FSM driver parameters and APIs usage.
 *      PMIC FSM Driver module covers all FSM features APIs. Like, set/get FSM
 *      states, enable FSM I2C Triggers, Mask and Unmask NSLEEP Signals and
 *      trigger Runtime BIST
 *
 *  Supported PMIC devices for FSM Module:
 *  1. TPS6594x (Leo PMIC Device)
 *  2. LP8764x  (Hera PMIC Device)
 *
 *  @{
 */

/**
 *  \file pmic_fsm.h
 *
 * \brief  PMIC Low Level Driver API/interface file for FSM API
 */

#ifndef PMIC_FSM_H_
#define PMIC_FSM_H_

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
 *  \anchor Pmic_Fsm_Off_Request_Type
 *  \name   PMIC FSM Off Request Type
 *
 *  @{
 */
#define PMIC_FSM_I2C_TRIGGER0_TYPE          (0U)
#define PMIC_FSM_ENABLE_PIN_TYPE            (1U)
#define PMIC_FSM_NPWRON_PIN_TYPE            (2U)
/*  @} */

/**
 *  \anchor Pmic_Fsm_Mission_State
 *  \name   To Select PMIC FSM  State
 *
 *  @{
 */
#define PMIC_FSM_STANBY_STATE               (0U)
#define PMIC_FSM_LP_STANBY_STATE            (1U)
#define PMIC_FSM_ACTIVE_STATE               (2U)
#define PMIC_FSM_MCU_ONLY_STATE             (3U)
#define PMIC_FSM_S2R_STATE                  (4U)
/* @} */

/**
 *  \anchor Pmic_Nsleep_Signals
 *  \name   PMIC Nsleep Signals
 *
 *  @{
 */
#define PMIC_NSLEEP1_SIGNAL                 (bool)false
#define PMIC_NSLEEP2_SIGNAL                 (bool)true
/* @} */

/**
 *  \anchor Pmic_Nsleep_Mask
 *  \name   PMIC Nsleep Masking/Unmasking
 *
 *  @{
 */
#define PMIC_NSLEEPX_MASK                    1U
#define PMIC_NSLEEPX_UNMASK                  0U
/* @} */

/**
 *  \anchor Pmic_Fsm_FastBist_Enable
 *  \name   PMIC Fast BIST Enable/Disable
 *
 *         This affects whether or not LBIST can be skipped to save boot up time
 *         from LP_STANDBY state.
 *         Application need to configure this before the device is turned off
 *         (using ENABLE pin or TRIGGER_I2C_0 bit) and enters LP_STANDBY state
 *
 *  @{
 */
/** \brief Only analog BIST is run at BOOT BIST  */
#define PMIC_FSM_FAST_BIST_ENABLE                    1U
/** \brief Logic and analog BIST is run at BOOT BIST. */
#define PMIC_FSM_FAST_BIST_DISABLE                   0U
/* @} */

/**
 *  \anchor Pmic_Nsleep_FSM_config
 *  \name   PMIC Nsleep FSM configuration
 *
 *  @{
 */
#define PMIC_NSLEEP1B_FSM_UNMASK              (0U)
#define PMIC_NSLEEP1B_FSM_MASK                (1U)
#define PMIC_NSLEEP2B_FSM_UNMASK              (0U)
#define PMIC_NSLEEP2B_FSM_MASK                (1U)
/*  @} */

/**
 *  \anchor Pmic_FSM_StartupDest_Select
 *  \name   Select PMIC FSM Startup Destination
 *
 *  @{
 */
 /** \brief Selects PMIC FSM Startup Destination as STANDBY state or
   *        LPSTANDBY state based on  LP_STANDBY_SEL */
#define PMIC_FSM_STARTUPDEST_STANDBY_LPSTANDBY     (0U)
/** \brief Selects PMIC FSM Startup Destination as MCU only state*/
#define PMIC_FSM_STARTUPDEST_MCUONLY               (2U)
/** \brief Selects PMIC FSM Startup Destination as Active state */
#define PMIC_FSM_STARTUPDEST_ACTIVE                (3U)
/*  @} */

/**
 *  \anchor Pmic_Fsm_LpStandby_Sel
 *  \name   PMIC LPSTANDBY State Selection
 *
 *  @{
 */
#define PMIC_FSM_SELECT_LPSTANDBY_STATE             1U
#define PMIC_FSM_SELECT_STANDBY_STATE               0U
/* @} */

/**
 *  \anchor Pmic_Fsm_IlimInt_FsmCtrl_En
 *  \name   To Enable/Disable PMIC Buck/LDO regulators ILIM interrupts affect
 *          FSM triggers
 *
 *  @{
 */
#define PMIC_FSM_ILIM_INT_FSMCTRL_ENABLE           1U
#define PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE          0U
/* @} */

/**
 *  \anchor Pmic_Pfsm_Delay_Type
 *  \name   PMIC PFSM Delay Type
 *
 *  @{
 */
#define PMIC_PFSM_DELAY1                    (0U)
#define PMIC_PFSM_DELAY2                    (1U)
#define PMIC_PFSM_DELAY3                    (2U)
#define PMIC_PFSM_DELAY4                    (3U)
/* @} */

/**
 *  \anchor Pmic_FsmValidParamCfg
 *  \name   PMIC FSM Configuration Structure Param Bits
 *
 *  PMIC FSM valid params configuration type for
 *  the structure member validParams of Pmic_FsmCfg_t
 *  structure
 *
 *  @{
 */
 /** \brief validParams value used to set/get to Enable/Disable Fast BIST  */
#define PMIC_FSM_CFG_FAST_BIST_EN_VALID           (0U)
/** \brief validParams value used to set/get Low Power Standby State Selection
 */
#define PMIC_FSM_CFG_LP_STANDBYSEL_VALID          (1U)
/** \brief validParams value used to set/get to Enable/Disable Buck/LDO
 *         regulators ILIM interrupts affect FSM triggers */
#define PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID    (2U)
/** \brief validParams value used to set/get to Select FSM Startup Destination
 */
#define PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID   (3U)
/*  @} */

/**
 *  \anchor Pmic_FsmValidParamBitShiftValues
 *  \name   PMIC FSM Configuration valid param bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_FsmCfg_t structure
 *
 *  @{
 */
#define PMIC_FSM_CFG_FAST_BIST_EN_VALID_SHIFT       \
                         (1U << PMIC_FSM_CFG_FAST_BIST_EN_VALID)
#define PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT       \
                         (1U << PMIC_FSM_CFG_LP_STANDBYSEL_VALID)
#define PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID_SHIFT       \
                         (1U << PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID)
#define PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT       \
                         (1U << PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID)
/*  @} */

/**
 *  \anchor Pmic_Nsleep_SignalLvl
 *  \name   PMIC Nsleep 1B/2B signal level
 *
 *  @{
 */
#define PMIC_NSLEEP_LOW                  (0U)
#define PMIC_NSLEEP_HIGH                 (1U)
/*  @} */

/**
 *  \anchor Pmic_Fsm_I2c_Trigger_Type
 *  \name   PMIC FSM I2C Trigger Type
 *
 *  @{
 */
#define PMIC_FSM_I2C_TRIGGER0                     (0x0U)
/** \brief Valid only for TPS6594x Leo PMIC PG2.0 and LP8764x Hera PMIC PG2.0 */
#define PMIC_FSM_I2C_TRIGGER1                     (0x1U)
/** \brief Valid only for TPS6594x Leo PMIC PG2.0 and LP8764x Hera PMIC PG2.0 */
#define PMIC_FSM_I2C_TRIGGER2                     (0x2U)
/** \brief Configuration of PMIC_FSM_I2C_TRIGGER3 is not supported for TPS6594x
 *         Leo and LP8764x Hera PMIC. */
#define PMIC_FSM_I2C_TRIGGER3                     (0x3U)
#define PMIC_FSM_I2C_TRIGGER4                     (0x4U)
#define PMIC_FSM_I2C_TRIGGER5                     (0x5U)
#define PMIC_FSM_I2C_TRIGGER6                     (0x6U)
#define PMIC_FSM_I2C_TRIGGER7                     (0x7U)
/*  @} */

/**
 *  \anchor Pmic_Fsm_I2c_Trigger_Val
 *  \name   PMIC FSM I2C Trigger Value
 *
 *  @{
 */
/** \brief Valid only for I2C4/ I2C5/ I2C6/ I2C7 */
#define PMIC_FSM_I2C_TRIGGER_VAL_0                (0x0U)
#define PMIC_FSM_I2C_TRIGGER_VAL_1                (0x1U)
/*  @} */

/**
 *  \anchor Pmic_Fsm_Retention_Mode
 *  \name   PMIC FSM Retention Mode
 *
 *  @{
 */
#define PMIC_FSM_DDR_RETENTION_MODE                 (0x0U)
/** \brief Valid only for J7200 SOC*/
#define PMIC_FSM_GPIO_RETENTION_MODE                (0x1U)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*!
 * \brief  PMIC FSM configuration structure.
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * \param   validParams         Selection of structure parameters to be set,
 *                              from the combination of
 *                              \ref Pmic_FsmValidParamCfg and the corresponding
 *                              member value must be updated
 *                              Valid values \ref Pmic_FsmValidParamCfg
 * \param   fastBistEn          Enable/Disable Fast BIST
 *                              Valid values \ref Pmic_Fsm_FastBist_Enable
 *                              Valid only when PMIC_FSM_CFG_FAST_BIST_EN_VALID
 *                              bit is set
 * \param   lpStandbySel        Low Power Standby State Selection
 *                              Valid values \ref Pmic_Fsm_LpStandby_Sel
 *                              Valid only when PMIC_FSM_CFG_LP_STANDBYSEL_VALID
 *                              bit is set
 * \param   ilimIntfsmCtrlEn    Enable/Disable Buck/LDO regulators ILIM
 *                              interrupts affect FSM triggers
 *                              Valid values \ref Pmic_Fsm_IlimInt_FsmCtrl_En
 *                              Valid only when
 *                              PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID
 *                              bit is set
 * \param   fsmStarupDestSel    Select FSM Startup Destination
 *                              Valid values \ref Pmic_FSM_StartupDest_Select
 *                              Valid only when
 *                              PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID
 *                              bit is set
 */
typedef struct Pmic_FsmCfg_s
{
    uint8_t                   validParams;
    bool                      fastBistEn;
    bool                      lpStandbySel;
    bool                      ilimIntfsmCtrlEn;
    uint8_t                   fsmStarupDestSel;
} Pmic_FsmCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief  API to initiate OFF Request FSM transition.
 *
 * Requirement: REQ_TAG(PDK-5851), REQ_TAG(PDK-9159), REQ_TAG(PDK-9329)
 * Design: did_pmic_lpstandby_cfg
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function initiate OFF Request FSM transition from any other
 *         mission state to the STANDBY state or the LP_STANDBY state
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   eventType         [IN]    Event Type used to initiate OFF Request
 *                                    Valid values:
 *                                    \ref Pmic_Fsm_Off_Request_Type
 * \param   fsmState          [IN]    FSM state.
 *                                    Only Valid for:
 *                                                  PMIC_FSM_STANBY_STATE and
 *                                                  PMIC_FSM_LP_STANBY_STATE
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmDeviceOffRequestCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                    uint8_t             eventType,
                                    uint8_t             fsmState);

/*!
 * \brief  API to initiate ON Request FSM transition.
 *
 * Requirement: REQ_TAG(PDK-5837)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function setup nSLEEP signal bits with STARTUP_DEST
 *         Which is common for all supported PMICs. This API needs to be called
 *         at PMIC init before clearing Enable and Start-Up interrupts.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
int32_t Pmic_fsmDeviceOnRequest(Pmic_CoreHandle_t *pPmicCoreHandle);

/*!
 * \brief  API to Set FSM mission States.
 *
 * Requirement: REQ_TAG(PDK-5837), REQ_TAG(PDK-5851)
 * Design: did_pmic_fsm_cfg_readback, did_pmic_lpstandby_cfg
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used for set/change the FSM mission states for PMIC
 *         using Nsleep1B and Nsleep2B signals in absence of GPIO pins
 *         Note: Application need to unmask Nsleep1B and Nsleep2B signals for
 *               FSM state transitions except for Standby/LpStandby State
 *               Application need to mask Nsleep1B and Nsleep2B signals for
 *               Standby/LpStandby State transition
 *               Application has to ensure the wakeup pins or RTC Timer/Alarm
 *               Interrupts are configured properly before triggering the PMIC
 *               device to Standby/LP Standby state.If not configured properly
 *               then PMIC device can't resume from sleep state
 *               Application has to ensure to connect/access the peripherals
 *               connected to only MCU Power lines when PMIC switch from Active
 *               to MCU state. If Application connects/access the peripherals
 *               connected to SOC Power lines when PMIC switch from Active to
 *               MCU state, then Application behaviour is unexpected.
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   pmicState        [IN]  PMIC FSM mission state
 *                                 Valid values: \ref Pmic_Fsm_Mission_State
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetMissionState(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                const uint8_t       pmicState);

/*!
 * \brief  API to MASK/UNMASK NSLEEP1B or 2B Signal.
 *
 * Requirement: REQ_TAG(PDK-5837)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used for Masking/Unmasking for NSLEEP2B or NSLEEP1B
 *         signal.
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   nsleepType       [IN]  NSLEEP signal
 *                                 Valid values: \ref Pmic_Nsleep_Signals
 * \param   maskEnable       [IN]  Parameter to select masking/unmasking
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetNsleepSignalMask(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                    const bool          nsleepType,
                                    const bool          maskEnable);

/*!
 * \brief  API to read the status of the NSLEEP1B/2B Signal is masked or not
 *
 * Requirement: REQ_TAG(PDK-9151)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used to read the status of the NSLEEP1B/2B Signal is
 *         masked or not
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   nsleepType       [IN]  NSLEEP signal
 *                                 Valid values: \ref Pmic_Nsleep_Signals
 * \param   pNsleepStat      [OUT] Pointer to store Nsleep Signal is masked or
 *                                 not
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetNsleepSignalMaskStat(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                        const bool          nsleepType,
                                        bool               *pNsleepStat);

/*!
 * \brief  API to initiate Runtime BIST.
 *
 * Requirement: REQ_TAG(PDK-5849)
 * Design: did_pmic_runtime_bist_cfg
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function initiates a request to exercise runtime BIST on the
 *         device
 *         Valid only for TPS6594x Leo PMIC PG2.0 and LP8764x Hera PMIC PG2.0
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmRequestRuntimeBist(Pmic_CoreHandle_t  *pPmicCoreHandle);

/*!
 * \brief   API to set PMIC FSM configuration.
 *
 * Requirement: REQ_TAG(PDK-9144), REQ_TAG(PDK-9134), REQ_TAG(PDK-9128)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to set the required FSM configuration when
 *          corresponding bit field is set.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   fsmCfg          [IN]    Set required FSM configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetConfiguration(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                 const Pmic_FsmCfg_t  fsmCfg);

/*!
 * \brief   API to get PMIC FSM configuration.
 *
 * Requirement: REQ_TAG(PDK-9144), REQ_TAG(PDK-9134), REQ_TAG(PDK-9128)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to get the FSM configuration when
 *          corresponding validParam bit fields are set in Pmic_FsmCfg_t
 *          structure
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   pFsmCfg         [IN/OUT]   Pointer to store FSM configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_FsmCfg_t     *pFsmCfg);

/*!
 * \brief  API to configure PFSM Delay
 *
 * Requirement: REQ_TAG(PDK-9136)
 * Design: did_pmic_pfsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used to configure PFSM Delay. PFSM Delay will affect
 *         the total power up sequence time before the system is released from
 *         reset
 *         Consider If the PFSM_Delay value is 'x' then Delay will calculated as
 *          Delay = x *(50ns * 2^PFSM_DELAY_STEP)
 *         Note: In this API, the default delay Type is assumed as
 *         PMIC_PFSM_DELAY1
 *         While adding support for New PMIC, developer need to update the API
 *         functionality for New PMIC device accordingly.
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   pFsmDelayType    [IN]  PFSM Delay Type
 *                                 Valid values: \ref Pmic_Pfsm_Delay_Type
 * \param   pfsmDelay        [IN]  Delay for PFSM
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetPfsmDelay(Pmic_CoreHandle_t  *pPmicCoreHandle,
                             const uint8_t       pFsmDelayType,
                             const uint8_t       pfsmDelay);

/*!
 * \brief  API to read PFSM Delay
 *
 * Requirement: REQ_TAG(PDK-9136)
 * Design: did_pmic_pfsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used to read PFSM Delay
 *         Note: In this API, the default delay Type is assumed as
 *         PMIC_PFSM_DELAY1
 *         While adding support for New PMIC, developer need to update the API
 *         functionality for New PMIC device accordingly.
 *
 * \param   pPmicCoreHandle  [IN]   PMIC Interface Handle
 * \param   pFsmDelayType    [IN]   PFSM Delay Type
 *                                  Valid values: \ref Pmic_Pfsm_Delay_Type
 * \param   pPfsmDelay       [OUT]  Pointer to store the Delay for PFSM
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetPfsmDelay(Pmic_CoreHandle_t  *pPmicCoreHandle,
                             uint8_t             pFsmDelayType,
                             uint8_t            *pPfsmDelay);

/*!
 * \brief   API to set PMIC Nsleep1B/2B Signal value.
 *
 * Requirement: REQ_TAG(PDK-9146)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to configure the Nsleep1B/2B signal level
 *
 * \param   pPmicCoreHandle  [IN]   PMIC Interface Handle.
 * \param   nsleepType       [IN]   NSLEEP signal
 *                                  Valid values: \ref Pmic_Nsleep_Signals
 * \param   nsleepVal        [IN]   PMIC Nsleep signal level High/Low to be
 *                                  configured.
 *                                  Valid values \ref Pmic_Nsleep_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetNsleepSignalVal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const bool         nsleepType,
                                   const uint8_t      nsleepVal);

/*!
 * \brief   API to get PMIC Nsleep1B/2B Signal value.
 *
 * Requirement: REQ_TAG(PDK-9146)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to read the signal level of the Nsleep1B/2B
 *          signal
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   nsleepType      [IN]    NSLEEP signal
 *                                  Valid values: \ref Pmic_Nsleep_Signals
 * \param   pNsleepVal      [OUT]   Pointer to store PMIC Nsleep signal level
 *                                  High/Low.
 *                                  Valid values \ref Pmic_Nsleep_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetNsleepSignalVal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const bool         nsleepType,
                                   uint8_t           *pNsleepVal);

/*!
 * \brief   API to recover from SOC Power Error using Nsleep1B and Nsleep2B
 *          signal
 *
 * Requirement: REQ_TAG(PDK-9123)
 * Design: did_pmic_fsm_recover_soc_pwr_err
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to recover from SOC Power Error without
 *          rebooting the system
 *          Note: Application need to call this API from MCU domain when SOC
 *          Power Error on Primary PMIC
 *          Step-1 - PMIC LLD has to configure NSLEEP2 & NSLEEP1 signals to ‘10’
 *          Step-2 - Application has to wait for 9us
 *          Step-3 - PMIC LLD has to configure NSLEEP2 & NSLEEP1 signals to ‘11’
 *
 *          Note: Valid only for TPS6594x Leo PMIC PG2.0 and LP8764x Hera PMIC
 *          PG2.0
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   nsleepVal       [IN]    PMIC Nsleep signal level High/Low to be
 *                                  configured.
 *                                  Valid values \ref Pmic_Nsleep_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmRecoverSocPwrErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t      nsleepVal);

/*!
 * \brief   API to initiate FSM I2C trigger for given FSM I2C trigger type
 *
 * Requirement: REQ_TAG(PDK-9330)
 * Design: did_pmic_fsm_i2c_trigger
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to to initiate FSM I2C trigger for given FSM
 *          I2C trigger type
 *          Note: In this API, the default i2cTriggerType is assumed as
 *                PMIC_FSM_I2C_TRIGGER0. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   i2cTriggerType  [IN]    FSM I2C Trigger Type
 *                                  Valid values: \ref Pmic_Fsm_I2c_Trigger_Type
 * \param   i2cTriggerVal   [IN]    FSM I2C Trigger Value
 *                                  Valid values: \ref Pmic_Fsm_I2c_Trigger_Val
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmEnableI2cTrigger(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                 const uint8_t       i2cTriggerType,
                                 const uint8_t       i2cTriggerVal);

/*!
 * \brief   API to get FSM I2C trigger Value for given FSM I2C trigger type
 *
 * Requirement: REQ_TAG(PDK-9330)
 * Design: did_pmic_fsm_i2c_trigger
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to read the FSM I2C trigger Value of the
 *          FSM Trigger Type
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   i2cTriggerType  [IN]    FSM I2C Trigger Type
 *                                  Valid values: \ref Pmic_Fsm_I2c_Trigger_Type
 * \param   pI2cTriggerVal  [OUT]   Pointer to store FSM I2C Trigger Value
 *                                  Valid values \ref Pmic_Fsm_I2c_Trigger_Val
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetI2cTriggerVal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t      i2cTriggerType,
                                 uint8_t           *pI2cTriggerVal);

/*!
 * \brief  API to initiate DDR/GPIO Retention Mode
 *
 * Requirement: REQ_TAG(PDK-9563), REQ_TAG(PDK-9564)
 * Design: did_pmic_ddr_gpio_retention_cfg
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function initiates a request to exercise DDR/GPIO Retention Mode
 *         on the device based on the Retention Mode
 *         Note: PMIC_FSM_GPIO_RETENTION_MODE is valid only for J7200 SOC
 *               Application has to ensure to connect/access the peripherals
 *               connected to only MCU Power lines except EN_GPIORET_LDSW,
 *               VDD_WK_0V8 Power lines when PMIC switch from Active to MCU
 *               state in GPIO Rentention mode with FSM i2c6 trigger value as
 *               '0'. If Application connects/access the peripherals connected
 *               to EN_GPIORET_LDSW, VDD_WK_0V8 Power lines when PMIC switch
 *               from Active to MCU state in GPIO Rentention mode with FSM i2c6
 *               trigger value as '0', then Application behaviour is unexpected.
 *               Application has to ensure to connect/access the peripherals
 *               connected to only MCU Power lines except VDD1_LPDDR4_1V8,
 *               VDD_DDR_1V1, GPIO_EN_VDDR_IO Power lines when PMIC switch from
 *               Active to MCU state in DDR Rentention mode with FSM i2c7
 *               trigger value as '0'. If Application connects/access the
 *               peripherals connected to VDD1_LPDDR4_1V8, VDD_DDR_1V1,
 *               GPIO_EN_VDDR_IO Power lines when PMIC switch from Active to MCU
 *               state in DDR Rentention mode with FSM i2c7 trigger value as
 *               '0', then Application behaviour is unexpected.
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   retentionMode    [IN]  Retention Mode
 *                                   Valid values: \ref Pmic_Fsm_Retention_Mode
 *                                     PMIC_FSM_GPIO_RETENTION_MODE is valid
 *                                     only for J7200 SOC
 * \param   i2cTriggerVal    [IN]   FSM I2C Trigger Value
 *                                   Valid values: \ref Pmic_Fsm_I2c_Trigger_Val
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmRequestDdrGpioRetentionMode(
                                        Pmic_CoreHandle_t  *pPmicCoreHandle,
                                        const uint8_t       retentionMode,
                                        const uint8_t       i2cTriggerVal);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_FSM_H_ */

/* @} */
