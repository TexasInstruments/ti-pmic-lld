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
 *  \defgroup DRV_PMIC_COMMON_MODULE PMIC Common Driver API
 *
 *  PMIC common Driver contains Miscellaneous APIs supported by all supported
 *  PMIC Devices.
 *  Like, PMIC recovery count APIs and PMIC nSLEEP Setup APIs.
 *
 *  @{
 */

/**
 *  \file pmic_core.h
 *
 *  \brief PMIC Driver Common API/interface file.
 */

#ifndef PMIC_CORE_H_
#define PMIC_CORE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
/*!
 * \brief  PMIC driver Core Handle INIT status Magic Number.
 *         Used to validate Handle to avoid corrupted PmicHandle usage.
 *         on Success: (DRV_INIT_SUCCESS | Pmic_InstType_t)
 */
#define DRV_INIT_SUCCESS                      (0xABCD0000U)

/**
 *  \anchor Pmic_RecoveryCntCfgType
 *  \name PMIC Recovery Counter Configuration Type
 *
 *  @{
 */
#define PMIC_CFG_RECOV_CNT_THR_VAL_VALID      (0U)
#define PMIC_CFG_RECOV_CNT_CLR_CNT_VALID      (1U)
/* @} */

/**
 *  \anchor Pmic_RecoveryCntCfgTypeStructPrmBitShiftVal
 *  \name   PMIC Recovery Count Configuration Structure Param Bit shift values
 *
 *  Application can use below shifted values to set the validParams
 *  struct members defined in Pmic_RecovCntCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT  \
                       (1U << PMIC_CFG_RECOV_CNT_THR_VAL_VALID)
#define PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT  \
                       (1U << PMIC_CFG_RECOV_CNT_CLR_CNT_VALID)
/* @} */

/**
 *  \anchor Pmic_ScratchPad_Sel
 *  \name   PMIC Scratchpad register selection
 *
 *  @{
 */
#define PMIC_SCRATCH_PAD_REG_1             (0x1U)
#define PMIC_SCRATCH_PAD_REG_2             (0x2U)
#define PMIC_SCRATCH_PAD_REG_3             (0x3U)
#define PMIC_SCRATCH_PAD_REG_4             (0x4U)
/*  @} */

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
/*!
 * \brief  PMIC Recovery Counter Configuration
 *
 * \param   validParams   Selection of structure parameters to be set,
 *                        from the combination of \ref Pmic_RecoveryCntCfgType
 *                        and the corresponding member value must be updated.
 *                        Valid values \ref Pmic_RecoveryCntCfgType
 *  \param  thrVal        Recovery Counter Threshold Value.
 *                         Valid only when PMIC_CFG_RECOV_CNT_THR_VAL_VALID
 *                         bit is set.
 *  \param  clrCnt        Clear Recovery Counter Value and value should be 1U.
 *                         Valid only when PMIC_CFG_RECOV_CNT_CLR_VAL_VALID
 *                         bit is set.
 */
typedef struct Pmic_RecovCntCfg_s
{
    uint8_t    validParams;
    uint8_t    thrVal;
    bool       clrCnt;
} Pmic_RecovCntCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief  API to Set Recovery Counter Configuration.
 *         This function configures PMIC Recovery Counter register, controlling
 *         recovery count Threshold and Clear, when corresponding validParam
 *         bit field is set in the Pmic_RecovCntCfg_t structure.
 *         For more information \ref Pmic_RecovCntCfg_t
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   recovCntCfg       [IN]    Set configuration value for
 *                                    Recovery counter
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_SetRecoveryCntCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                               Pmic_RecovCntCfg_t  recovCntCfg);

/*!
 * \brief  API to Get Recovery Counter Configuration.
 *         This function gets PMIC Recovery Counter configuration values. Like,
 *         recovery count Threshold and Clear, when corresponding validParam
 *         bit field is set in the Pmic_RecovCntCfg_t structure.
 *         For more information \ref Pmic_RecovCntCfg_t
 *
 * \param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * \param   pRecovCntCfg          [OUT]   Pointer to store recovery counter
 *                                        configuration value
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getRecoveryCntCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                               Pmic_RecovCntCfg_t *pRecovCntCfg);

/*!
 * \brief  API to Read Recovery Count Value.
 *         This function reads out the recovery count value.
 *
 * \param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * \param   pRecovCntVal          [OUT]   Pointer to store recovery count
 *                                        value
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getRecoveryCnt(Pmic_CoreHandle_t *pPmicCoreHandle,
                            uint8_t           *pRecovCntVal);

/*!
 * \brief  API to setup nSLEEP signals.
 *         This function setup nSLEEP signal bits with STARTUP_DEST
 * Which is common for all supported PMICs. This API needs to be called
 * at PMIC init before clearing Enable and Start-Up interrupts.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 */
int32_t Pmic_nSleepSignalsSetup(Pmic_CoreHandle_t *pPmicCoreHandle);

/*!
 * \brief   API to set/write value in/to scratchpad register.
 *          This function is used write data to scratchpad register of PMIC
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   scratchPadRegNum   [IN]    ScratchPad register number
 *                                     \ref Pmic_ScratchPad_Sel
 * \param   data               [IN]    Data/Value to be written to scratchpad.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_setScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                uint8_t            scratchPadRegNum,
                                uint8_t            data);

/*!
 * \brief   API to get/read data from scratchpad register.
 *          This function is used read data from scratchpad register of PMIC
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   scratchPadRegNum   [IN]    ScratchPad register number
 *                                     \ref Pmic_ScratchPad_Sel
 * \param   pData               [OUT]   Parameter to hold the Data/Value read
 *                                     from scratchpad.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                uint8_t            scratchPadRegNum,
                                uint8_t            *pData);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_H_ */

/* @} */
