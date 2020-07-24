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
 *  states, enable FSM I2C Triggers.
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
 *  \anchor Pmic_Fsm_Event_Type
 *  \name   PMIC FSM Event Type
 *
 *  @{
 */
#define PMIC_FSM_I2C_TRIGGER0_EVENT_TYPE    (0U)
/*  @} */
/**
 *  \anchor Pmic_Fsm_I2c_Trigger_Type
 *  \name   PMIC FSM I2C Trigger Type
 *
 *  @{
 */
#define PMIC_FSM_I2C_TRIGGER0_TYPE          (0U)
/*  @} */

/**
 *  \anchor Pmic_Fsm_LpStandby_State
 *  \name   To Select PMIC LP_STANDBY/ STANBY State
 *
 *  @{
 */
#define PMIC_FSM_STANBY_STATE               (0U)
#define PMIC_FSM_LP_STANBY_STATE            (1U)
/*  @} */

/*!
 * \brief  Used to create PFSM trigger pulse for I2C0
 */
#define PMIC_FSM_I2C_TRIGGER_VAL            (1U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief  API to initiate OFF Request FSM transition
 *         This function initiate OFF Request FSM transition from any other
 *         mission state to the STANDBY state or the LP_STANDBY state
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   eventType         [IN]    Event Type used to initiate OFF Request
 *                                    Valid values \ref Pmic_Fsm_Event_Type
 * \param   fsmState          [IN]    FSM state
 *                                    Valid values \ref Pmic_Fsm_LpStandby_State
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmDeviceOffRequestCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                    uint8_t             eventType,
                                    bool                fsmState);

/*!
 * \brief  API to initiate OFF Request FSM transition
 *         This function initiate OFF Request FSM transition from any other
 *         mission state to the STANDBY state or the LP_STANDBY state
 *
 * \param   pPmicCoreHandle   [IN]   PMIC Interface Handle.
 * \param   i2cTriggerType    [IN]   FSM I2C Trigger type use to initiate OFF
 *                                   Request
 *                                   Valid values \ref Pmic_Fsm_I2c_Trigger_Type
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmEnableI2cTrigger(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                 bool                i2cTriggerType);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_FSM_H_ */

/* @} */
