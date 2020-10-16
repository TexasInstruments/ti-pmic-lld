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
 *  PMIC FSM Driver module covers all FSM features APIs. Like, set/get FSM
 *  states, enable FSM I2C Triggers, Mask and Unmask NSLEEP Signals and 
 *  trigger Runtime BIST
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
#define PMIC_NSLEEPX_MASK                   (1U)
#define PMIC_NSLEEPX_UNMASK                 (0U)
/* @} */

/**
 *  \anchor Pmic_Fsm_Lpm_Cntrl
 *  \name   PMIC LPM Control
 *
 *  @{
 */
#define PMIC_FSM_LPM_ENABLE                   (1U)
#define PMIC_FSM_LPM_DISABLE                  (0U)
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
/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief  API to initiate OFF Request FSM transition.
 *
 * Requirement: REQ_TAG(PDK-5851)
 * Design: did_pmic_lpstandby_cfg
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
 * Design: did_pmic_fsm_cfg
 *
 *         This function setup nSLEEP signal bits with STARTUP_DEST
 *         Which is common for all supported PMICs. This API needs to be called
 *         at PMIC init before clearing Enable and Start-Up interrupts.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 */
int32_t Pmic_fsmDeviceOnRequest(Pmic_CoreHandle_t *pPmicCoreHandle);

/*!
 * \brief  API to Set FSM mission States.
 *
 * Requirement: REQ_TAG(PDK-5837), REQ_TAG(PDK-5851)
 * Design: did_pmic_fsm_cfg, did_pmic_lpstandby_cfg
 *
 *         This function is used for set/change the FSM mission states for PMIC
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   pmicState        [IN]  PMIC FSM MISSION STATE
 *                                 Valid values: \ref Pmic_Fsm_Mission_State
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetMissionState(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                uint8_t             pmicState);

/*!
 * \brief  API to MASK/UNMASK NSLEEP Signal.
 *
 * Requirement: REQ_TAG(PDK-5837)
 * Design: did_pmic_fsm_cfg
 *
 *         This function is used for Masking/Unmasking for NSLEEP2 or NSLEEP1
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
                                    bool                nsleepType,
                                    bool                maskEnable);

/*!
 * \brief  API to initiate Runtime BIST.
 *
 * Requirement: REQ_TAG(PDK-5849)
 * Design: did_pmic_runtime_bist_cfg
 *
 *         This function initiates a request to exercise runtime BIST on the
 *         device
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmRequestRuntimeBist(Pmic_CoreHandle_t  *pPmicCoreHandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_FSM_H_ */

/* @} */
