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
 * \file   pmic_core_tps6522x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         TPS6522x Burton PMIC driver specific PMIC Core configuration
 *
 */

#ifndef PMIC_CORE_TPS6522X_PRIV_H_
#define PMIC_CORE_TPS6522X_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_core_priv.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/*!
 * \brief   PMIC TPS6522x Spread Spectrum 1 Register Mask Values
 */
#define PMIC_TPS6522X_SPREAD_SPECTRUM_1_SS_DEPTH_MASK ((uint8_t)(0x01U << PMIC_SPREAD_SPECTRUM_1_SS_DEPTH_SHIFT))

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_CORE_TPS6522X_PRIV_H_ */
