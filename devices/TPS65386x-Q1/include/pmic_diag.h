/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
#ifndef PMIC_DIAG_H
#define PMIC_DIAG_H

/**
 * @file pmic_diag.h
 * @brief PMIC Driver Diagnostic API/Interface
 */

/**
 * @defgroup DRV_PMIC_DIAG_MODULE PMIC Diagnostic Module
 * @brief APIs used to configure and interact with the PMIC Diagnostic features (AMUX/DMUX).
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
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_DiagOutCfgCtrlValidParam
 * @name PMIC Diagnostic Output Control Configuration Valid Params
 *
 * @{
 */
#define PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID (1UL << 0U)
#define PMIC_DIAG_OUT_CTRL_DMUX_EN_VALID (1UL << 1U)
#define PMIC_DIAG_OUT_CTRL_VALID         (1UL << 2U)
/** @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @brief PMIC diagnostic output control configuration structure
 *
 * @param validParams Selection of structure parameters to be set, from
 * @ref Pmic_DiagOutCfgCtrlValidParam
 * @param diagOutCtrl_AMUXEn AMUX enable (0=disable, 1=enable).
 * Valid when PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID is set
 * @param diagOutCtrl_DMUXEn DMUX enable (0=disable, 1=enable).
 * Valid when PMIC_DIAG_OUT_CTRL_DMUX_EN_VALID is set
 * @param diagOutCtrl Diagnostic output control state (0=disabled, 1=AMUX, 2=DMUX).
 * Valid when PMIC_DIAG_OUT_CTRL_VALID is set
 */
typedef struct Pmic_DiagOutCfgCtrl_s {
    uint32_t validParams;
    uint8_t diagOutCtrl_AMUXEn;
    uint8_t diagOutCtrl_DMUXEn;
    uint8_t diagOutCtrl;
} Pmic_DiagOutCfgCtrl_t;

/*==========================================================================*/
/*                           Function Declarations                          */
/*==========================================================================*/

/**
 * @brief Set diagnostic output control configuration (AMUX/DMUX enable).
 *
 * Design: PMICDRV-789
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [IN] Diagnostic output control configuration.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_diagSetOutCtrlCfg(const Pmic_Handle_t *handle, const Pmic_DiagOutCfgCtrl_t *config);

/**
 * @brief Get diagnostic output control configuration.
 *
 * Design: PMICDRV-790
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [OUT] Diagnostic output control configuration.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_diagGetOutCtrlCfg(const Pmic_Handle_t *handle, Pmic_DiagOutCfgCtrl_t *config);

/**
 * @brief Set AMUX channel configuration.
 *
 * Design: PMICDRV-791
 *
 * @param handle [IN] PMIC interface handle.
 * @param channel [IN] AMUX channel to select (0-31).
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_diagSetAmuxCfg(const Pmic_Handle_t *handle, uint8_t channel);

/**
 * @brief Get AMUX channel configuration.
 *
 * Design: PMICDRV-792
 *
 * @param handle [IN] PMIC interface handle.
 * @param channel [OUT] Current AMUX channel (0-31).
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_diagGetAmuxCfg(const Pmic_Handle_t *handle, uint8_t *channel);

/**
 * @brief Set DMUX group configuration.
 *
 * Design: PMICDRV-793
 *
 * @param handle [IN] PMIC interface handle.
 * @param group [IN] DMUX group to select (0-31).
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_diagSetDmuxCfg(const Pmic_Handle_t *handle, uint8_t group);

/**
 * @brief Get DMUX group configuration.
 *
 * Design: PMICDRV-794
 *
 * @param handle [IN] PMIC interface handle.
 * @param group [OUT] Current DMUX group (0-31).
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_diagGetDmuxCfg(const Pmic_Handle_t *handle, uint8_t *group);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_DIAG_H */
