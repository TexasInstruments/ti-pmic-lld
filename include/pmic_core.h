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
 *  \defgroup DRV_PMIC_CORE_MODULE PMIC Common Driver API
 *            This is PMIC driver common parameters and API
 *
 *  @{
 */

/**
 *  \file pmic_core.h
 *
 *  \brief PMIC Low Level Driver API/interface file for common API
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
 * \brief: Indicator of valid driver instance :DRVDEF
 *         and validation code to avoid corrupted PmicHandle data
 *         on Success: (DRV_INIT_SUCCESS | Pmic_InstType_t)
 */
#define DRV_INIT_SUCCESS                    (0xABCD0000U)

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
 *  \name   PMIC Recovery Counter Configuration Type Structure Param Bit
 *          shift values
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

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
 * \brief: PMIC Recovery Counter Configuration
 *
 * \param   validParams   Selection of structure parameters to be set,
 *                        from the combination of \ref Pmic_RecoveryCntCfgType
 *                        and the corresponding member value must be updated
 *                        Valid values \ref Pmic_RecoveryCntCfgType
 *  \param  thrVal        Recovery Counter Threshold Value
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
 * \brief: Set Recovery Counter Configuration
 *         This function configures PMIC Recovery Counter register controlling
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
 * \brief: Get Recovery Counter Configuration
 *         This function get PMIC Recovery Counter configurations values of
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
 * \brief: Read Recovery Count Value
 *         This function reads the recovery count value
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
 * \brief: Function call to setup nSLEEP signals
 *         This function setup nSLEEP signal bits with STARTUP_DEST
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 */
int32_t Pmic_nSleepSignalsSetup(Pmic_CoreHandle_t *pPmicCoreHandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_H_ */

/* @} */
