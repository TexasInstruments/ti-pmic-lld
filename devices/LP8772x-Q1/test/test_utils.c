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


/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_utils.h"

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

#define TEST_COMMON_MIN_INT_REG        ((uint8_t)0x50U)
#define TEST_COMMON_MAX_INT_REG        ((uint8_t)0x58U)
#define TEST_COMMON_WDG_ERR_STATUS_REG ((uint8_t)0x62U)
#define TEST_COMMON_REGISTER_LOCK_REG  ((uint8_t)0x09U)
#define TEST_COMMON_REGISTER_UNLOCK_KEY    ((uint8_t)0x9BU)

// BIT3 of SILICON_REV[7:0] identifies whether the PMIC is PG1 (A0) or PG2 (B1)
#define DEVICE_PG_IDENTIFIER_MASK (1UL << 3U)

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void testUtils_printSiRev(const Pmic_Handle_t *pmicHandle)
{
    char msg[50U] = {0};

    // LP8772x doesn't have isA0 field, use devSiRev only
    if ((pmicHandle->devSiRev & DEVICE_PG_IDENTIFIER_MASK) != 0U)
    {
        (void)sprintf(msg, "PMIC device is PG2 (B1)\r\n\r\n");
        platform_printString(msg);
    }
    else
    {
        (void)sprintf(msg, "PMIC device is PG1 (A0 or B0)\r\n\r\n");
        platform_printString(msg);
    }
}
