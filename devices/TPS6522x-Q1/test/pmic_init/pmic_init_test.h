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
#ifndef PMIC_INIT_TEST_H
#define PMIC_INIT_TEST_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void pmic_init_test(void *args);

void test_negative_Pmic_init_nullHandle(void);
void test_negative_Pmic_init_nullConfig(void);
void test_negative_Pmic_deinit_nullHandle(void);
void test_negative_Pmic_checkHandle_nullHandle(void);
void test_negative_Pmic_init_invalidCommMode(void);
void test_negative_Pmic_init_nullCommHandle(void);
void test_negative_Pmic_init_nullTaskHandle(void);
void test_negative_Pmic_init_nullIoRead(void);
void test_negative_Pmic_init_nullIoWrite(void);
void test_negative_Pmic_init_nullAsyncRxStart(void);
void test_negative_Pmic_init_nullAsyncTxStart(void);
void test_negative_Pmic_init_nullAsyncRxAwait(void);
void test_negative_Pmic_init_nullAsyncTxAwait(void);
void test_negative_Pmic_init_nullCritSecStart(void);
void test_negative_Pmic_init_nullCritSecStop(void);
void test_negative_Pmic_init_nullIrqCallback(void);

void test_positive_Pmic_init_validConfig(void);
void test_positive_Pmic_deinit_afterInit(void);
void test_positive_Pmic_checkHandle_validHandle(void);
void test_positive_Pmic_checkHandle_invalidHandle(void);
void test_positive_Pmic_init_reinit(void);
void test_positive_Pmic_init_with_crc_enabled(void);
void test_positive_Pmic_init_with_both_crc_flags(void);
void test_positive_Pmic_init_crc_disabled(void);
void test_positive_Pmic_init_verify_crc_state(void);
void test_positive_Pmic_init_complete_flow(void);
void test_positive_Pmic_init_i2c_single_mode(void);
void test_positive_Pmic_init_i2c_dual_mode(void);
void test_positive_Pmic_init_device_info_retrieval(void);
void test_positive_Pmic_deinit_success_path(void);
void test_positive_Pmic_checkHandle_all_validations(void);
void test_positive_Pmic_init_async_mode(void);
void test_positive_Pmic_init_with_i2c_addresses(void);
void test_positive_Pmic_init_with_task_handle(void);
void test_positive_init_withRetryCnt(void);
void test_positive_init_withRetryInterval(void);
void test_positive_init_withTimerWaitMs(void);
void test_negative_init_timerWaitNull(void);
void test_negative_init_timerWaitMsCallbackNull(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__PMIC_INIT_TEST_H__*/
