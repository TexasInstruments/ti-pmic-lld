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
#ifndef __TEST_COMMON_H__
#define __TEST_COMMON_H__

/**
 * @file test_common.h
 *
 * @brief APIs commonly used across PMIC LLD testing.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                           Function Declarations                           */
/* ========================================================================= */

/**
 * @brief Clear all PMIC interrupt flags.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if all PMIC interrupt flags have been cleared, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes
 */
int32_t testCommon_clrAllPmicIrq(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Unlock all PMIC user space registers.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if all PMIC user space registers have been unlocked,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes
 */
int32_t testCommon_unlockPmicRegs(const Pmic_CoreHandle_t *pmicHandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __TEST_COMMON_H__ */
