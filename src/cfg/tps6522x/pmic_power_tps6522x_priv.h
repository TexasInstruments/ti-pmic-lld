/******************************************************************************
 * Copyright (c) 2023 Texas Instruments Incorporated - http://www.ti.com
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
 * \file   pmic_power_tps6522x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         TPS6522x BURTON PMIC driver specific PMIC power configuration
 *
 */

#ifndef PMIC_POWER_TPS6522X_PRIV_H_
#define PMIC_POWER_TPS6522X_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_power_priv.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*!
 *  \anchor     Pmic_Tps6522xBurton_powerRegAddr
 *  \brief      Burton power register addresses for internal use
 */
#define PMIC_TPS6522X_INVALID_REGADDR         (0x00U)
#define PMIC_TPS6522X_VCCA_VMON_CTRL_REGADDR  (0x2BU)
#define PMIC_TPS6522X_VCCA_PG_WINDOW_REGADDR  (0x2CU)
#define PMIC_TPS6522X_VMON1_PG_WINDOW_REGADDR (0x2DU)
#define PMIC_TPS6522X_VMON1_PG_LEVEL_REGADDR  (0x2EU)
#define PMIC_TPS6522X_VMON2_PG_WINDOW_REGADDR (0x2FU)
#define PMIC_TPS6522X_VMON2_PG_LEVEL_REGADDR  (0x30U)
#define PMIC_TPS6522X_RAIL_SEL_3_REGADDR      (0x43U)
#define PMIC_TPS6522X_STAT_BUCK_REGADDR       (0x6DU)
#define PMIC_TPS6522X_STAT_LDO_VMON_REGADDR   (0x70U)

/* ========================================================================== */
/*                          Structures and Enums                              */
/* ========================================================================== */

/* ========================================================================== */
/*                         Function Declarations                              */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_POWER_TPS6522X_PRIV_H_ */
