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
 *  \defgroup DRV_PMIC_CORE_MODULE PMIC Driver Common Data types
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
#define PMIC_RECOV_CNT_CFLAG_THR            (0x01U)
#define PMIC_RECOV_CNT_CFLAG_CLR            (0x02U)
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
 * \brief: PMIC Recovery Counter Configuration
 *
 *  \param  cfgType       Recovery Counter Configuration Type as Counter
 *                        Threshold or CCounter Clear
 *  \param  thrVal        Recovery Counter Threshold Value
 *  \param  clrVal        Recovery Counter Clear Value
 */
typedef struct Pmic_RecovCntCfg_s
{
    uint8_t    cfgType;
    uint8_t    thrVal;
    uint8_t    clrVal;
} Pmic_RecovCntCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief: Set Recovery Counter Configuration
 *         This function configures PMIC Recovery Counter register controlling
 *         recovery count Threshold and Clear
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pRecovCnt         [IN]    Pointer to set configuration value for
 *                                    Recovery counter
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_SetRecoveryCntCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                               Pmic_RecovCntCfg_t *pRecovCnt);

/*!
 * \brief: Read Recovery Count Configuration
 *         This function reads the recovery count register value
 *
 * \param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * \param   pRecovCntCfgVal       [OUT]   Pointer to store recovery counter
 *                                        configuration value
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getRecoveryCntCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               uint8_t           *pRecovCntCfgVal);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_H_ */

/* @} */
