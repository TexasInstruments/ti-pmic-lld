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
/**
 * @file test_common.c
 *
 * @brief Source file containing definitions to APIs commonly used across PMIC
 * LLD testing.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_common.h"

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

#define TEST_COMMON_MIN_INT_REG        ((uint8_t)0x50U)
#define TEST_COMMON_MAX_INT_REG        ((uint8_t)0x58U)
#define TEST_COMMON_WDG_ERR_STATUS_REG ((uint8_t)0x62U)
#define TEST_COMMON_REGISTER_LOCK_REG  ((uint8_t)0x09U)
#define TEST_COMMON_REGISTER_UNLOCK_KEY    ((uint8_t)0x9BU)

// BIT3 of SILICON_REV[7:0] identifies whether the PMIC is PG1 (A0) or PG2 (B1)
#define DEVICE_PG_IDENTIFER_MASK (1U << 3U)

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

int32_t testCommon_clrAllPmicIrq(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);
    uint8_t txBuf = 0xFFU;

    for (uint8_t regAddr = TEST_COMMON_MIN_INT_REG; regAddr <= TEST_COMMON_MAX_INT_REG; regAddr++)
    {
        if (status != PMIC_ST_SUCCESS)
        {
            break;
        }

        status = platform_txByte(pmicHandle, regAddr, 1U, &txBuf);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_txByte(pmicHandle, TEST_COMMON_WDG_ERR_STATUS_REG, 1U, &txBuf);
    }

    return status;
}

int32_t testCommon_unlockPmicRegs(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);
    uint8_t txBuf = TEST_COMMON_REGISTER_UNLOCK_KEY;

    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_txByte(pmicHandle, TEST_COMMON_REGISTER_LOCK_REG, 1U, &txBuf);
    }

    return status;
}

void testCommon_printSiRev(const Pmic_CoreHandle_t *pmicHandle)
{
    char msg[50U] = {0};

    if ((pmicHandle->siliconRev & DEVICE_PG_IDENTIFER_MASK) != 0U)
    {
        (void)sprintf(msg, "PMIC device is B1\r\n\r\n");
        platform_printString(msg);
    }
    else if (pmicHandle->isA0)
    {
        (void)sprintf(msg, "PMIC device is A0\r\n\r\n");
        platform_printString(msg);
    }
    else
    {
        (void)sprintf(msg, "PMIC device is B0\r\n\r\n");
        platform_printString(msg);
    }
}

int32_t testCommon_disableConfigCrc(const Pmic_CoreHandle_t *pmicHandle)
{
    const uint8_t bufLen = 0U;
    uint8_t configCrcConfigRegAddr = (pmicHandle->isA0) ? 0x61U : 0x64U;
    return platform_txByte(pmicHandle, configCrcConfigRegAddr, bufLen, 0x0U);
}
