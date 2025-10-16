/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "pmic.h"
#include "pmic_common.h"

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
bool Pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos) {
    return (((validParamVal >> bitPos) & 0x01U) != 0U);
}

bool Pmic_validParamStatusCheck(uint32_t validParamVal, uint8_t bitPos, int32_t status) {
    return ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(validParamVal, bitPos));
}

void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *handle) {
    if (handle->pFnPmicCritSecStart != (void *)0U) {
        handle->pFnPmicCritSecStart();
    }
}

void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *handle) {
    if (handle->pFnPmicCritSecStop != (void *)0U) {
        handle->pFnPmicCritSecStop();
    }
}

void Pmic_pseudoIrqTrigger(const Pmic_CoreHandle_t *handle) {
    if (handle->pFnPmicPseudoIrq != (void *)0U) {
        handle->pFnPmicPseudoIrq();
    }
}
