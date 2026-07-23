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

#include "platform.h"
#include "common_test.h"
#include "test_constants.h"

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t g_pmicHandle;

/* Mock callback tracking variables */
static uint32_t g_critSecStartCallCount = 0;
static uint32_t g_critSecStopCallCount = 0;
static uint8_t g_lastCritSecResource = TEST_INVALID_RSRC_SENTINEL;
static uint32_t g_timerWaitCallCount = 0;
static uint32_t g_lastTimerWaitMs = 0;

/* ========================================================================== */
/*                          Mock Callback Functions                           */
/* ========================================================================== */

static void mockCritSecStart(uint8_t resource)
{
    g_critSecStartCallCount++;
    g_lastCritSecResource = resource;
}

static void mockCritSecStop(uint8_t resource)
{
    g_critSecStopCallCount++;
}

static void mockTimerWait(uint32_t ms)
{
    g_timerWaitCallCount++;
    g_lastTimerWaitMs = ms;
}

static void resetMockCounters(void)
{
    g_critSecStartCallCount = 0;
    g_critSecStopCallCount = 0;
    g_lastCritSecResource = TEST_INVALID_RSRC_SENTINEL;
    g_timerWaitCallCount = 0;
    g_lastTimerWaitMs = 0;
}

/* ========================================================================== */
/*                     Critical Section Test Functions                        */
/* ========================================================================== */

void test_neg_common_criticalSectionStart_nullHandle(void)
{
    resetMockCounters();

    Pmic_criticalSectionStart(NULL, PMIC_COMMUNICATION);
    PLATFORM_ASSERT(g_critSecStartCallCount == 0);

    Pmic_criticalSectionStop(NULL, PMIC_COMMUNICATION);
    PLATFORM_ASSERT(g_critSecStopCallCount == 0);
}

void test_neg_common_criticalSectionStart_nullCallback(void)
{
    Pmic_Handle_t handle = {0};
    resetMockCounters();

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = NULL;

    Pmic_criticalSectionStart(&handle, PMIC_COMMUNICATION);
    PLATFORM_ASSERT(g_critSecStartCallCount == 0);

    Pmic_criticalSectionStop(&handle, PMIC_COMMUNICATION);
    PLATFORM_ASSERT(g_critSecStopCallCount == 0);
}

void test_pos_common_criticalSection_communication(void)
{
    Pmic_Handle_t handle = {0};
    resetMockCounters();

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_criticalSectionStart(&handle, PMIC_COMMUNICATION);
    PLATFORM_ASSERT(g_critSecStartCallCount == 1);
    PLATFORM_ASSERT(g_lastCritSecResource == PMIC_COMMUNICATION);

    Pmic_criticalSectionStop(&handle, PMIC_COMMUNICATION);
    PLATFORM_ASSERT(g_critSecStopCallCount == 1);
}

void test_pos_common_criticalSection_diagnostic(void)
{
    Pmic_Handle_t handle = {0};
    resetMockCounters();

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_criticalSectionStart(&handle, PMIC_DIAGNOSTIC);
    PLATFORM_ASSERT(g_critSecStartCallCount == 1);
    PLATFORM_ASSERT(g_lastCritSecResource == PMIC_DIAGNOSTIC);

    Pmic_criticalSectionStop(&handle, PMIC_DIAGNOSTIC);
    PLATFORM_ASSERT(g_critSecStopCallCount == 1);
}

/* ========================================================================== */
/*                       Timer Test Functions                                 */
/* ========================================================================== */

void test_neg_common_timerWaitMs_nullHandle(void)
{
    resetMockCounters();

    Pmic_timerWaitMs(NULL, 100U);
    PLATFORM_ASSERT(g_timerWaitCallCount == 0);
}

void test_neg_common_timerWaitMs_nullCallback(void)
{
    Pmic_Handle_t handle = {0};
    resetMockCounters();

    handle.timerWaitMs = NULL;

    Pmic_timerWaitMs(&handle, 100U);
    PLATFORM_ASSERT(g_timerWaitCallCount == 0);
}

void test_pos_common_timerWaitMs_validCall(void)
{
    Pmic_Handle_t handle = {0};
    resetMockCounters();

    handle.timerWaitMs = mockTimerWait;

    Pmic_timerWaitMs(&handle, 250U);
    PLATFORM_ASSERT(g_timerWaitCallCount == 1);
    PLATFORM_ASSERT(g_lastTimerWaitMs == 250U);

    Pmic_timerWaitMs(&handle, 0U);
    PLATFORM_ASSERT(g_timerWaitCallCount == 2);
    PLATFORM_ASSERT(g_lastTimerWaitMs == 0U);
}

/* ========================================================================== */
/*                     Pmic_logStatus Test Functions                          */
/* ========================================================================== */

void test_pos_common_logStatus_success(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;
    resetMockCounters();

    int32_t status = Pmic_logStatus(&handle, PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(g_critSecStartCallCount == 0);
}

void test_neg_common_logStatus_nullHandle(void)
{
    int32_t status = Pmic_logStatus(NULL, PMIC_ST_ERR_NULL_PARAM);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_common_logStatus_nullCritSec(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = NULL;

    int32_t status = Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_pos_common_logStatus_validError(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;
    resetMockCounters();

    Pmic_clrDiagnosticsAll(&handle);

    status = Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
    PLATFORM_ASSERT(g_critSecStartCallCount > 0);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 1U);
}

void test_pos_common_logStatus_validWarning(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    status = Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_WARN_NO_IRQ_REMAINING;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 1U);
}

void test_neg_common_logStatus_invalidStatusType(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    int32_t invalidStatus = PMIC_STATUS(TEST_INVALID_PARAM_99, 0U);
    int32_t status = Pmic_logStatus(&handle, invalidStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_neg_common_logStatus_invalidStatusId(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    int32_t invalidStatus = PMIC_STATUS(PMIC_ST_TYPE_ERROR, TEST_INVALID_PARAM_99);
    int32_t status = Pmic_logStatus(&handle, invalidStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

void test_pos_common_logStatus_allErrorCodes(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    int32_t errorCodes[] = {
        PMIC_ST_ERR_INV_HANDLE,
        PMIC_ST_ERR_NULL_PARAM,
        PMIC_ST_ERR_INV_PARAM,
        PMIC_ST_ERR_NULL_FPTR,
        PMIC_ST_ERR_DATA_IO_CRC,
        PMIC_ST_ERR_I2C_COMM_FAIL,
        PMIC_ST_ERR_NOT_SUPPORTED,
        PMIC_ST_ERR_INV_STATUS_TYPE,
        PMIC_ST_ERR_INV_STATUS_ID
    };

    for (uint8_t i = 0; i < (sizeof(errorCodes) / sizeof(errorCodes[0])); i++)
    {
        status = Pmic_logStatus(&handle, errorCodes[i]);
        PLATFORM_ASSERT(status == errorCodes[i]);

        diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
        diag.code = errorCodes[i];
        status = Pmic_getDiagnostic(&handle, &diag);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(diag.cnt >= 1U);
    }
}

/* ========================================================================== */
/*                     Diagnostic Get Test Functions                          */
/* ========================================================================== */

void test_neg_common_getDiagnostic_nullHandle(void)
{
    Pmic_Diagnostic_t diag = {0};
    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_getDiagnostic(NULL, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_getDiagnostic_nullDiagnostic(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_getDiagnostic(&handle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_common_getDiagnostic_invalidValidParams(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = 0U;
    diag.code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_common_getDiagnostic_invalidStatusCode(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_STATUS(TEST_INVALID_PARAM_99, 0U);

    int32_t status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_pos_common_getDiagnostic_errorCnt(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 3U);
}

void test_pos_common_getDiagnostic_errorFlag(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.flag == false);
}

void test_pos_common_getDiagnostic_warningCnt(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);
    Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_WARN_NO_IRQ_REMAINING;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 2U);
}

void test_pos_common_getDiagnostic_warningFlag(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diag.code = PMIC_ST_WARN_NO_IRQ_REMAINING;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.flag == false);
}

void test_neg_common_getDiagnostic_successType(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_SUCCESS;

    int32_t status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_neg_common_getDiagnostic_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_getDiagnostic_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_getDiagnostics_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_getDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_common_getDiagnostics_multiple(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[3] = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_ERR_INV_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    diags[1].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[1].code = PMIC_ST_ERR_INV_PARAM;
    diags[2].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[2].code = PMIC_ST_WARN_NO_IRQ_REMAINING;

    status = Pmic_getDiagnostics(&handle, diags, 3U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[0].cnt == 1U);
    PLATFORM_ASSERT(diags[1].cnt == 1U);
    PLATFORM_ASSERT(diags[2].cnt == 1U);
}

void test_neg_common_getDiagnostics_zeroCount(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_getDiagnostics(&handle, diags, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_common_getDiagnostics_exceedsMax(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[50] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_getDiagnostics(&handle, diags, 50U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                     Diagnostic Clear Test Functions                        */
/* ========================================================================== */

void test_neg_common_clrDiagnostic_nullHandle(void)
{
    Pmic_Diagnostic_t diag = {0};
    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_clrDiagnostic(NULL, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_common_clrDiagnostic_errorCnt(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 0U);
}

void test_pos_common_clrDiagnostic_warningCnt(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_WARN_NO_IRQ_REMAINING;
    status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_WARN_NO_IRQ_REMAINING;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 0U);
}

void test_pos_common_clrDiagnostics_multiple(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_ERR_INV_PARAM);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    diags[1].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[1].code = PMIC_ST_ERR_INV_PARAM;

    status = Pmic_clrDiagnostics(&handle, diags, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_getDiagnostic(&handle, &diags[0]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[0].cnt == 0U);

    diags[1].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[1].code = PMIC_ST_ERR_INV_PARAM;
    status = Pmic_getDiagnostic(&handle, &diags[1]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[1].cnt == 0U);
}

void test_pos_common_clrDiagnosticsAll_clearAll(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);
    Pmic_incrementRetryCnt(&handle);

    status = Pmic_clrDiagnosticsAll(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 0U);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_WARN_NO_IRQ_REMAINING;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 0U);

    uint32_t retryCnt = TEST_INVALID_PARAM_99;
    status = Pmic_getRetryCnt(&handle, &retryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(retryCnt == 0U);
}

void test_pos_common_overflow_errorCnt(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i <= PMIC_ERR_CNT_OVERFLOW_THR; i++)
    {
        Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    }

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID | PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 0U);
    PLATFORM_ASSERT(diag.flag == true);
}

void test_pos_common_overflow_warningCnt(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i <= PMIC_WARN_CNT_OVERFLOW_THR; i++)
    {
        Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);
    }

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID | PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diag.code = PMIC_ST_WARN_NO_IRQ_REMAINING;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 0U);
    PLATFORM_ASSERT(diag.flag == true);
}

/* ========================================================================== */
/*                     Retry Counter Test Functions                           */
/* ========================================================================== */

void test_neg_common_getRetryCnt_nullHandle(void)
{
    uint32_t retryCnt = 0;
    int32_t status = Pmic_getRetryCnt(NULL, &retryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_getRetryCnt_nullOutput(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_getRetryCnt(&handle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_common_getRetryCnt_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};
    uint32_t retryCnt = 0U;

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_getRetryCnt(&handle, &retryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_getRetryCnt_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};
    uint32_t retryCnt = 0U;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    int32_t status = Pmic_getRetryCnt(&handle, &retryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_common_getRetryCnt_initialZero(void)
{
    Pmic_Handle_t handle = {0};
    uint32_t retryCnt = TEST_INVALID_PARAM_99;
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    status = Pmic_getRetryCnt(&handle, &retryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(retryCnt == 0U);
}

void test_neg_common_incrementRetryCnt_nullHandle(void)
{
    int32_t status = Pmic_incrementRetryCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_common_incrementRetryCnt_once(void)
{
    Pmic_Handle_t handle = {0};
    uint32_t retryCnt = 0;
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    status = Pmic_incrementRetryCnt(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getRetryCnt(&handle, &retryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(retryCnt == 1U);
}

void test_pos_common_incrementRetryCnt_multiple(void)
{
    Pmic_Handle_t handle = {0};
    uint32_t retryCnt = 0;
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i < 10U; i++)
    {
        status = Pmic_incrementRetryCnt(&handle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    status = Pmic_getRetryCnt(&handle, &retryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(retryCnt == 10U);
}

void test_neg_common_incrementRetryCnt_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_incrementRetryCnt(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_incrementRetryCnt_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    int32_t status = Pmic_incrementRetryCnt(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrRetryCnt_nullHandle(void)
{
    int32_t status = Pmic_clrRetryCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_common_clrRetryCnt_afterIncrement(void)
{
    Pmic_Handle_t handle = {0};
    uint32_t retryCnt = 0;
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_incrementRetryCnt(&handle);
    Pmic_incrementRetryCnt(&handle);

    status = Pmic_clrRetryCnt(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getRetryCnt(&handle, &retryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(retryCnt == 0U);
}

void test_neg_common_clrRetryCnt_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_clrRetryCnt(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrRetryCnt_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    int32_t status = Pmic_clrRetryCnt(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_getRetryCntOverflow_nullHandle(void)
{
    bool reachedThreshold = false;
    int32_t status = Pmic_getRetryCntOverflow(NULL, &reachedThreshold);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrRetryCntOverflow_nullHandle(void)
{
    int32_t status = Pmic_clrRetryCntOverflow(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_getRetryCntOverflow_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};
    bool overflow = false;

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_getRetryCntOverflow(&handle, &overflow);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_getRetryCntOverflow_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};
    bool overflow = false;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    int32_t status = Pmic_getRetryCntOverflow(&handle, &overflow);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrRetryCntOverflow_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_clrRetryCntOverflow(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrRetryCntOverflow_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    int32_t status = Pmic_clrRetryCntOverflow(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_common_overflow_retryCnt(void)
{
    Pmic_Handle_t handle = {0};
    uint32_t retryCnt = 0;
    bool reachedThreshold = false;
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i <= PMIC_RETRY_CNT_OVERFLOW_THR; i++)
    {
        status = Pmic_incrementRetryCnt(&handle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    status = Pmic_getRetryCnt(&handle, &retryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(retryCnt == 0U);

    status = Pmic_getRetryCntOverflow(&handle, &reachedThreshold);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(reachedThreshold == true);

    status = Pmic_clrRetryCntOverflow(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getRetryCntOverflow(&handle, &reachedThreshold);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(reachedThreshold == false);
}

/* ========================================================================== */
/*                  Additional Edge Case Tests for 100% Coverage             */
/* ========================================================================== */

void test_neg_common_logStatus_invalidStatusNullHandle(void)
{
    int32_t invalidStatus = PMIC_STATUS(TEST_INVALID_PARAM_99, 0U);
    int32_t status = Pmic_logStatus(NULL, invalidStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_neg_common_logStatus_successTypeInvalidId(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t invalidStatus = PMIC_STATUS(PMIC_ST_TYPE_SUCCESS, TEST_INVALID_PARAM_99);
    int32_t status = Pmic_logStatus(&handle, invalidStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

void test_neg_common_getDiagnostics_nullHandle(void)
{
    Pmic_Diagnostic_t diags[2] = {0};
    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_getDiagnostics(NULL, diags, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_getDiagnostics_nullDiagnosticArray(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_getDiagnostics(&handle, NULL, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_common_getDiagnostics_zeroValidParamsInArray(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    diags[1].validParams = 0U;
    diags[1].code = PMIC_ST_ERR_INV_PARAM;

    int32_t status = Pmic_getDiagnostics(&handle, diags, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_common_getDiagnostics_invalidStatusCodeInArray(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    diags[1].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[1].code = PMIC_STATUS(TEST_INVALID_PARAM_99, 0U);

    int32_t status = Pmic_getDiagnostics(&handle, diags, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_neg_common_getDiagnostics_successTypeInArray(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_SUCCESS;

    int32_t status = Pmic_getDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_neg_common_getDiagnostics_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_getDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_common_getDiagnostics_cntOnlyError(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_logStatus(&handle, PMIC_ST_ERR_INV_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_ERR_INV_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_ERR_INV_PARAM);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_INV_PARAM;

    status = Pmic_getDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[0].cnt == 3U);
}

void test_neg_common_clrDiagnostic_nullDiagnostic(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_clrDiagnostic(&handle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_common_clrDiagnostic_invalidValidParams(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = 0U;
    diag.code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_common_clrDiagnostic_invalidStatusCode(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_STATUS(TEST_INVALID_PARAM_99, 0U);

    int32_t status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_neg_common_clrDiagnostic_successType(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_SUCCESS;

    int32_t status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_pos_common_clrDiagnostic_errorFlagOnly(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i <= PMIC_ERR_CNT_OVERFLOW_THR; i++)
    {
        Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    }

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID | PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.flag == false);
    PLATFORM_ASSERT(diag.cnt == 0U);
}

void test_pos_common_clrDiagnostic_warningFlagOnly(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i <= PMIC_WARN_CNT_OVERFLOW_THR; i++)
    {
        Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);
    }

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diag.code = PMIC_ST_WARN_NO_IRQ_REMAINING;
    status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID | PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_WARN_NO_IRQ_REMAINING;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.flag == false);
    PLATFORM_ASSERT(diag.cnt == 0U);
}

void test_neg_common_clrDiagnostics_nullHandle(void)
{
    Pmic_Diagnostic_t diags[1] = {0};
    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_clrDiagnostics(NULL, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrDiagnostics_nullDiagnosticArray(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_clrDiagnostics(&handle, NULL, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_common_clrDiagnostics_zeroCount(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_clrDiagnostics(&handle, diags, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_common_clrDiagnostics_exceedsMax(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[50] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_clrDiagnostics(&handle, diags, 50U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_common_clrDiagnostics_zeroValidParamsInArray(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    diags[1].validParams = 0U;
    diags[1].code = PMIC_ST_ERR_INV_PARAM;

    int32_t status = Pmic_clrDiagnostics(&handle, diags, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_common_clrDiagnostics_invalidStatusCodeInArray(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    diags[1].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[1].code = PMIC_STATUS(TEST_INVALID_PARAM_99, 0U);

    int32_t status = Pmic_clrDiagnostics(&handle, diags, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_neg_common_clrDiagnostics_successTypeInArray(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_SUCCESS;

    int32_t status = Pmic_clrDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_TYPE);
}

void test_neg_common_clrDiagnostics_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_clrDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrDiagnostics_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_clrDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_common_clrDiagnostics_cntOnlyError(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_logStatus(&handle, PMIC_ST_ERR_INV_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_ERR_INV_PARAM);
    Pmic_logStatus(&handle, PMIC_ST_ERR_INV_PARAM);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_INV_PARAM;

    status = Pmic_clrDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    status = Pmic_getDiagnostic(&handle, &diags[0]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[0].cnt == 0U);
}

void test_neg_common_getRetryCntOverflow_nullOutput(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_getRetryCntOverflow(&handle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_common_logStatus_warningTypeInvalidId(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t invalidStatus = PMIC_STATUS(PMIC_ST_TYPE_WARNING, TEST_INVALID_PARAM_99);
    int32_t status = Pmic_logStatus(&handle, invalidStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

void test_pos_common_getDiagnostics_errorFlagOnly(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i <= PMIC_ERR_CNT_OVERFLOW_THR; i++)
    {
        Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    }

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;

    status = Pmic_getDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[0].flag == true);
}

void test_pos_common_getDiagnostics_warningFlagOnly(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i <= PMIC_WARN_CNT_OVERFLOW_THR; i++)
    {
        Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);
    }

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diags[0].code = PMIC_ST_WARN_NO_IRQ_REMAINING;

    status = Pmic_getDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[0].flag == true);
}

void test_pos_common_clrDiagnostics_errorFlagOnly(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i <= PMIC_ERR_CNT_OVERFLOW_THR; i++)
    {
        Pmic_logStatus(&handle, PMIC_ST_ERR_NULL_PARAM);
    }

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;

    status = Pmic_clrDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID | PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    status = Pmic_getDiagnostic(&handle, &diags[0]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[0].flag == false);
}

void test_pos_common_clrDiagnostics_warningCntOnly(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);
    Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_WARN_NO_IRQ_REMAINING;

    status = Pmic_clrDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    status = Pmic_getDiagnostic(&handle, &diags[0]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[0].cnt == 0U);
}

void test_pos_common_clrDiagnostics_warningFlagOnly(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[1] = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    for (uint32_t i = 0; i <= PMIC_WARN_CNT_OVERFLOW_THR; i++)
    {
        Pmic_logStatus(&handle, PMIC_ST_WARN_NO_IRQ_REMAINING);
    }

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID;
    diags[0].code = PMIC_ST_WARN_NO_IRQ_REMAINING;

    status = Pmic_clrDiagnostics(&handle, diags, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_FLAG_VALID | PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    status = Pmic_getDiagnostic(&handle, &diags[0]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diags[0].flag == false);
}

void test_neg_common_clrDiagnosticsAll_nullHandle(void)
{
    int32_t status = Pmic_clrDiagnosticsAll(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrDiagnosticsAll_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    int32_t status = Pmic_clrDiagnosticsAll(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrDiagnosticsAll_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    int32_t status = Pmic_clrDiagnosticsAll(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrDiagnostic_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = NULL;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_common_clrDiagnostic_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = NULL;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_ST_ERR_NULL_PARAM;

    int32_t status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

/* ========================================================================== */
/*                      Boundary Condition Tests                              */
/* ========================================================================== */

void test_pos_common_logStatus_maxErrorId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    int32_t maxErrorStatus = PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_ERROR_MAX);
    status = Pmic_logStatus(&handle, maxErrorStatus);
    PLATFORM_ASSERT(status == maxErrorStatus);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = maxErrorStatus;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 1U);
}

void test_pos_common_logStatus_maxWarningId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    int32_t maxWarningStatus = PMIC_STATUS(PMIC_ST_TYPE_WARNING, PMIC_ST_ID_WARNING_MAX);
    status = Pmic_logStatus(&handle, maxWarningStatus);
    PLATFORM_ASSERT(status == maxWarningStatus);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = maxWarningStatus;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 1U);
}

void test_neg_common_logStatus_exceedsMaxErrorId(void)
{
    Pmic_Handle_t handle = {0};
    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    int32_t overMaxErrorStatus = PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_ERROR_MAX + 1U);
    int32_t status = Pmic_logStatus(&handle, overMaxErrorStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

void test_pos_common_getDiagnostic_maxErrorId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    int32_t maxErrorStatus = PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_ERROR_MAX);
    Pmic_logStatus(&handle, maxErrorStatus);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = maxErrorStatus;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 1U);
}

void test_neg_common_getDiagnostic_exceedsMaxErrorId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_ERROR_MAX + 1U);

    int32_t status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

void test_neg_common_getDiagnostic_exceedsMaxWarningId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_STATUS(PMIC_ST_TYPE_WARNING, PMIC_ST_ID_WARNING_MAX + 1U);

    int32_t status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

void test_pos_common_clrDiagnostic_maxErrorId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};
    int32_t status;

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    Pmic_clrDiagnosticsAll(&handle);

    int32_t maxErrorStatus = PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_ERROR_MAX);
    Pmic_logStatus(&handle, maxErrorStatus);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = maxErrorStatus;
    status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = maxErrorStatus;
    status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(diag.cnt == 0U);
}

void test_neg_common_clrDiagnostic_exceedsMaxErrorId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_ERROR_MAX + 1U);

    int32_t status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

void test_neg_common_clrDiagnostic_exceedsMaxWarningId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_STATUS(PMIC_ST_TYPE_WARNING, PMIC_ST_ID_WARNING_MAX + 1U);

    int32_t status = Pmic_clrDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

/* ========================================================================== */
// Out-of-range status ID boundary test functions
/* ========================================================================== */

/**
 * @brief Test Pmic_getDiagnostic with out-of-range ERROR status ID.
 *
 * Passes PMIC_ST_TYPE_ERROR with statusId == PMIC_ST_ID_ERROR_MAX + 1.
 * statusCodeCheck() catches this before the inner isValidErrorId check,
 * so the inner branch may be unreachable from external callers.
 * Note: may be unreachable — verify during coverage run
 */
void test_neg_common_getDiagnostic_outOfRangeErrorId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_ERROR_MAX + 1U);

    int32_t status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

/**
 * @brief Test Pmic_getDiagnostic with out-of-range WARNING status ID.
 *
 * Passes PMIC_ST_TYPE_WARNING with statusId == PMIC_ST_ID_WARNING_MAX + 1.
 * statusCodeCheck() catches this before the inner isValidWarningId check.
 * Note: may be unreachable — verify during coverage run
 */
void test_neg_common_getDiagnostic_outOfRangeWarningId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diag = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diag.validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diag.code = PMIC_STATUS(PMIC_ST_TYPE_WARNING, PMIC_ST_ID_WARNING_MAX + 1U);

    int32_t status = Pmic_getDiagnostic(&handle, &diag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

/**
 * @brief Test Pmic_getDiagnostics with an array element having out-of-range ERROR ID.
 *
 * Places a valid first element and a second element with an out-of-range error ID.
 * Note: may be unreachable — verify during coverage run
 */
void test_neg_common_getDiagnostics_outOfRangeErrorId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    diags[1].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[1].code = PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_ERROR_MAX + 1U);

    int32_t status = Pmic_getDiagnostics(&handle, diags, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

/**
 * @brief Test Pmic_clrDiagnostics with an array element having out-of-range ERROR ID.
 *
 * Note: may be unreachable — verify during coverage run
 */
void test_neg_common_clrDiagnostics_outOfRangeErrorId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    diags[1].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[1].code = PMIC_STATUS(PMIC_ST_TYPE_ERROR, PMIC_ST_ID_ERROR_MAX + 1U);

    int32_t status = Pmic_clrDiagnostics(&handle, diags, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

/**
 * @brief Test Pmic_clrDiagnostics with an array element having out-of-range WARNING ID.
 *
 * Note: may be unreachable — verify during coverage run
 */
void test_neg_common_clrDiagnostics_outOfRangeWarningId(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_Diagnostic_t diags[2] = {0};

    handle.criticalSectionStart = mockCritSecStart;
    handle.criticalSectionStop = mockCritSecStop;

    diags[0].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[0].code = PMIC_ST_ERR_NULL_PARAM;
    diags[1].validParams = PMIC_COMMON_DIAGNOSTIC_CNT_VALID;
    diags[1].code = PMIC_STATUS(PMIC_ST_TYPE_WARNING, PMIC_ST_ID_WARNING_MAX + 1U);

    int32_t status = Pmic_clrDiagnostics(&handle, diags, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_STATUS_ID);
}

/* ========================================================================== */
/*                        Test Suite Entry Point                              */
/* ========================================================================== */

void common_test(void *args)
{
    (void)args;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = PMIC_CFG_INIT_COMM_MODE_VALID |
                       PMIC_CFG_INIT_I2C_ADDR0_VALID |
                       PMIC_CFG_INIT_I2C_ADDR1_VALID |
                       PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                       PMIC_CFG_INIT_COMM_HANDLE_1_VALID |
                       PMIC_CFG_INIT_IO_READ_VALID |
                       PMIC_CFG_INIT_IO_WRITE_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_I2C_DUAL,
        .i2cAddr0 = PLATFORM_I2C_ADDR_MAIN,
        .i2cAddr1 = PLATFORM_I2C_ADDR_SECONDARY,
        .commHandle0 = platform_getCommHandle0(),
        .commHandle1 = platform_getCommHandle1(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    platform_init();
    testTimer_startModule("Common");

    printf("\r\n");
    printf("==================================================\r\n");
    printf("    TPS6522x-Q1 Common Module Tests\r\n");
    printf("==================================================\r\n\r\n");

    status = Pmic_init(&g_pmicHandle, &pmicCfg);
    if (status != PMIC_ST_SUCCESS)
    {
        printf("ERROR: PMIC initialization failed with status: %d\r\n", status);
        platform_deinit();
        return;
    }

    COMMON_TEST_RUN_ALL();

    testTimer_endModule();
    (void)Pmic_deinit(&g_pmicHandle);
    platform_deinit();

    printf("\r\n==================================================\r\n");
    printf("    Common Module Tests Complete\r\n");
    printf("==================================================\r\n\r\n");
}

/* ========================================================================== */
/*                   Pmic_validParamCheck Test Functions                      */
/* ========================================================================== */

void test_pos_common_validParamCheck_bitSet(void)
{
    const uint32_t validParams = (1UL << 3U);

    PLATFORM_ASSERT(Pmic_validParamCheck(validParams, (1UL << 3U)) == true);
}

void test_pos_common_validParamCheck_bitClear(void)
{
    const uint32_t validParams = (1UL << 3U);

    PLATFORM_ASSERT(Pmic_validParamCheck(validParams, (1UL << 5U)) == false);
}

void test_pos_common_validParamCheck_noParamsSet(void)
{
    PLATFORM_ASSERT(Pmic_validParamCheck(0U, (1UL << 0U)) == false);
}

void test_pos_common_validParamCheck_allParamsSet(void)
{
    PLATFORM_ASSERT(Pmic_validParamCheck(0xFFFFFFFFU, (1UL << 15U)) == true);
}

void test_pos_common_validParamCheck_multipleParams(void)
{
    const uint32_t validParams = (1UL << 0U) | (1UL << 2U) | (1UL << 7U);

    PLATFORM_ASSERT(Pmic_validParamCheck(validParams, (1UL << 2U)) == true);
    PLATFORM_ASSERT(Pmic_validParamCheck(validParams, (1UL << 4U)) == false);
}

/* ========================================================================== */
/*                Pmic_validParamStatusCheck Test Functions                   */
/* ========================================================================== */

void test_pos_common_validParamStatusCheck_successAndBitSet(void)
{
    const uint32_t validParams = (1UL << 2U);

    PLATFORM_ASSERT(Pmic_validParamStatusCheck(validParams, (1UL << 2U), PMIC_ST_SUCCESS) == true);
}

void test_pos_common_validParamStatusCheck_errorAndBitSet(void)
{
    const uint32_t validParams = (1UL << 2U);

    PLATFORM_ASSERT(Pmic_validParamStatusCheck(validParams, (1UL << 2U), PMIC_ST_ERR_NULL_PARAM) == false);
}

void test_pos_common_validParamStatusCheck_successAndBitClear(void)
{
    const uint32_t validParams = (1UL << 1U);

    PLATFORM_ASSERT(Pmic_validParamStatusCheck(validParams, (1UL << 2U), PMIC_ST_SUCCESS) == false);
}

void test_pos_common_validParamStatusCheck_errorAndBitClear(void)
{
    const uint32_t validParams = (1UL << 1U);

    PLATFORM_ASSERT(Pmic_validParamStatusCheck(validParams, (1UL << 2U), PMIC_ST_ERR_NULL_PARAM) == false);
}

/* ========================================================================== */
/*                    Pmic_getBitField Test Functions                         */
/* ========================================================================== */

void test_pos_common_getBitField_lowNibble(void)
{
    const uint8_t regData = 0xABU;

    PLATFORM_ASSERT(Pmic_getBitField(regData, 0U, 0x0FU) == 0x0BU);
}

void test_pos_common_getBitField_highNibble(void)
{
    const uint8_t regData = 0xABU;

    PLATFORM_ASSERT(Pmic_getBitField(regData, 4U, 0xF0U) == 0x0AU);
}

void test_pos_common_getBitField_singleBitSet(void)
{
    const uint8_t regData = (uint8_t)(1U << 3U);

    PLATFORM_ASSERT(Pmic_getBitField(regData, 3U, (uint8_t)(1U << 3U)) == 1U);
}

void test_pos_common_getBitField_singleBitClear(void)
{
    const uint8_t regData = (uint8_t)(~(uint8_t)(1U << 3U));

    PLATFORM_ASSERT(Pmic_getBitField(regData, 3U, (uint8_t)(1U << 3U)) == 0U);
}

void test_pos_common_getBitField_fullByteMask(void)
{
    PLATFORM_ASSERT(Pmic_getBitField(TEST_PATTERN_A5, 0U, 0xFFU) == TEST_PATTERN_A5);
}

void test_pos_common_getBitField_twoBitField(void)
{
    /* 0xA5 = 1010_0101; bits[5:4] = 10b = 2 */
    const uint8_t regData = 0xA5U;

    PLATFORM_ASSERT(Pmic_getBitField(regData, 4U, 0x30U) == 0x02U);
}

/* ========================================================================== */
/*                   Pmic_getBitField_b Test Functions                        */
/* ========================================================================== */

void test_pos_common_getBitField_b_bitSet(void)
{
    const uint8_t regData = (uint8_t)(1U << 5U);

    PLATFORM_ASSERT(Pmic_getBitField_b(regData, 5U) == true);
}

void test_pos_common_getBitField_b_bitClear(void)
{
    const uint8_t regData = (uint8_t)(~(uint8_t)(1U << 5U));

    PLATFORM_ASSERT(Pmic_getBitField_b(regData, 5U) == false);
}

void test_pos_common_getBitField_b_lsb(void)
{
    PLATFORM_ASSERT(Pmic_getBitField_b(0x01U, 0U) == true);
    PLATFORM_ASSERT(Pmic_getBitField_b(0x00U, 0U) == false);
}

void test_pos_common_getBitField_b_msb(void)
{
    PLATFORM_ASSERT(Pmic_getBitField_b(0x80U, 7U) == true);
    PLATFORM_ASSERT(Pmic_getBitField_b(0x00U, 7U) == false);
}

void test_pos_common_getBitField_b_alternatingPattern(void)
{
    /* 0xAA = 1010_1010: odd positions are 1, even are 0 */
    /* 0x55 = 0101_0101: even positions are 1, odd are 0 */
    for (uint8_t pos = 0U; pos < 8U; pos++)
    {
        if ((pos % 2U) == 0U)
        {
            PLATFORM_ASSERT(Pmic_getBitField_b(TEST_PATTERN_AA, pos) == false);
            PLATFORM_ASSERT(Pmic_getBitField_b(TEST_PATTERN_55, pos) == true);
        }
        else
        {
            PLATFORM_ASSERT(Pmic_getBitField_b(TEST_PATTERN_AA, pos) == true);
            PLATFORM_ASSERT(Pmic_getBitField_b(TEST_PATTERN_55, pos) == false);
        }
    }
}

/* ========================================================================== */
/*                    Pmic_setBitField Test Functions                         */
/* ========================================================================== */

void test_pos_common_setBitField_setLowNibble(void)
{
    uint8_t regData = 0xF0U;

    Pmic_setBitField(&regData, 0U, 0x0FU, 0x05U);
    PLATFORM_ASSERT(regData == 0xF5U);
}

void test_pos_common_setBitField_setHighNibble(void)
{
    uint8_t regData = 0x0FU;

    Pmic_setBitField(&regData, 4U, 0xF0U, 0x0AU);
    PLATFORM_ASSERT(regData == 0xAFU);
}

void test_pos_common_setBitField_clearBits(void)
{
    uint8_t regData = 0xFFU;

    Pmic_setBitField(&regData, 0U, 0x0FU, 0x00U);
    PLATFORM_ASSERT(regData == 0xF0U);
}

void test_pos_common_setBitField_preservesOtherBits(void)
{
    /* Set bits[2:1] = 0b11 in TEST_PATTERN_AA (1010_1010).
     * Bits outside the mask must be unchanged. */
    uint8_t regData = TEST_PATTERN_AA;

    Pmic_setBitField(&regData, 1U, 0x06U, 0x03U);
    PLATFORM_ASSERT((regData & (uint8_t)(~0x06U)) == (TEST_PATTERN_AA & (uint8_t)(~0x06U)));
    PLATFORM_ASSERT((regData & 0x06U) == 0x06U);
}

void test_pos_common_setBitField_fullByteMask(void)
{
    uint8_t regData = TEST_PATTERN_55;

    Pmic_setBitField(&regData, 0U, 0xFFU, TEST_PATTERN_A5);
    PLATFORM_ASSERT(regData == TEST_PATTERN_A5);
}

/* ========================================================================== */
/*       Pmic_setBitField_b Test Functions (4-param: explicit mask)           */
/* ========================================================================== */

void test_pos_common_setBitField_b_setTrue(void)
{
    uint8_t regData = 0x00U;

    Pmic_setBitField_b(&regData, 3U, true);
    PLATFORM_ASSERT(regData == (uint8_t)(1U << 3U));
}

void test_pos_common_setBitField_b_setFalse(void)
{
    uint8_t regData = 0xFFU;

    Pmic_setBitField_b(&regData, 3U, false);
    PLATFORM_ASSERT((regData & (uint8_t)(1U << 3U)) == 0U);
}

void test_pos_common_setBitField_b_preservesOtherBitsWhenSet(void)
{
    /* TEST_PATTERN_AA = 1010_1010; bit 2 is 0 — set it to 1 */
    uint8_t regData = TEST_PATTERN_AA;

    Pmic_setBitField_b(&regData, 2U, true);
    PLATFORM_ASSERT((regData & (uint8_t)(~(uint8_t)(1U << 2U))) ==
                    (TEST_PATTERN_AA & (uint8_t)(~(uint8_t)(1U << 2U))));
    PLATFORM_ASSERT((regData & (uint8_t)(1U << 2U)) != 0U);
}

void test_pos_common_setBitField_b_preservesOtherBitsWhenCleared(void)
{
    /* TEST_PATTERN_AA = 1010_1010; bit 1 is 1 — clear it to 0 */
    uint8_t regData = TEST_PATTERN_AA;

    Pmic_setBitField_b(&regData, 1U, false);
    PLATFORM_ASSERT((regData & (uint8_t)(~(uint8_t)(1U << 1U))) ==
                    (TEST_PATTERN_AA & (uint8_t)(~(uint8_t)(1U << 1U))));
    PLATFORM_ASSERT((regData & (uint8_t)(1U << 1U)) == 0U);
}

void test_pos_common_setBitField_b_lsb(void)
{
    uint8_t regData = 0x00U;

    Pmic_setBitField_b(&regData, 0U, true);
    PLATFORM_ASSERT(regData == 0x01U);

    Pmic_setBitField_b(&regData, 0U, false);
    PLATFORM_ASSERT(regData == 0x00U);
}

void test_pos_common_setBitField_b_msb(void)
{
    uint8_t regData = 0x00U;

    Pmic_setBitField_b(&regData, 7U, true);
    PLATFORM_ASSERT(regData == 0x80U);

    Pmic_setBitField_b(&regData, 7U, false);
    PLATFORM_ASSERT(regData == 0x00U);
}

