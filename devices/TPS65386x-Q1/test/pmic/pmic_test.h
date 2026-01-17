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
#ifndef PMIC_TEST_H
#define PMIC_TEST_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "../platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void pmic_test(void *args);

/* ========================================================================== */
/*                       Pmic_init Test Declarations                          */
/* ========================================================================== */

/* Negative Tests - NULL Parameters */
void test_neg_pmic_init_nullHandle(void);
void test_neg_pmic_init_nullCoreCfg(void);
void test_neg_pmic_init_nullCommHandle(void);
void test_neg_pmic_init_nullIoRead(void);
void test_neg_pmic_init_nullIoWrite(void);
void test_neg_pmic_init_nullCritSecStart(void);
void test_neg_pmic_init_nullCritSecStop(void);

/* Negative Tests - Invalid Parameters */
void test_neg_pmic_init_invalidDeviceType(void);
void test_neg_pmic_init_invalidCommMode(void);
void test_neg_pmic_init_insufficientCfg_missingIoRead(void);
void test_neg_pmic_init_insufficientCfg_missingIoWrite(void);
void test_neg_pmic_init_insufficientCfg_missingCritSec(void);
void test_neg_pmic_init_timerWaitNull(void);

/* Positive Tests - Basic Init */
void test_pos_pmic_init_spiMode(void);
void test_pos_pmic_init_validateHandle(void);
void test_pos_pmic_init_withAllCallbacks(void);
void test_pos_pmic_init_multipleInitDeinit(void);
void test_pos_pmic_init_handlePersistence(void);

/* Positive Tests - Configuration Validation */
void test_pos_pmic_init_validDeviceType(void);
void test_pos_pmic_init_validCommMode(void);
void test_pos_pmic_init_validateCommHandleStored(void);
void test_pos_pmic_init_validateCallbacksStored(void);

/* Positive Tests - Communication Validation */
void test_pos_pmic_init_communicationTest(void);
void test_pos_pmic_init_registerAccess(void);

/* Positive Tests - Initialization Lifecycle */
void test_pos_pmic_init_deinit_reinit(void);
void test_pos_pmic_init_cleanStateAfterDeinit(void);
void test_pos_pmic_init_verifySubsystemInfo(void);

/* Positive Tests - Configuration Combinations */
void test_pos_pmic_init_minimalConfig(void);
void test_pos_pmic_init_fullConfig(void);
void test_pos_pmic_init_verifyInitMagic(void);

/* Positive Tests - Advanced */
void test_pos_pmic_init_critSecFunctions(void);
void test_pos_pmic_init_verifyDeviceComm(void);

/* Positive Tests - CRC Configuration */
void test_pos_pmic_init_with_crc_enabled(void);
void test_pos_pmic_init_with_config_crc_enabled(void);
void test_pos_pmic_init_with_both_crc_enabled(void);
void test_pos_pmic_init_crc_error_recovery(void);

/* Positive Tests - Device Info Retrieval & Complete Flow */
void test_pos_pmic_init_complete_flow(void);
void test_pos_pmic_init_device_info_retrieval(void);
void test_pos_pmic_init_communication_validation(void);
void test_pos_pmic_init_spi_comprehensive(void);

/* Positive Tests - Optional Config Fields */
void test_pos_pmic_init_withRetryCnt(void);
void test_pos_pmic_init_withRetryInterval(void);
void test_pos_pmic_init_withTimerWaitMs(void);

/* ========================================================================== */
/*                      Pmic_deinit Test Declarations                         */
/* ========================================================================== */

/* Negative Tests */
void test_neg_pmic_deinit_nullHandle(void);

/* Positive Tests */
void test_pos_pmic_deinit_clearsHandle(void);
void test_pos_pmic_deinit_success_comprehensive(void);

/* ========================================================================== */
/*                    Pmic_checkHandle Test Declarations                      */
/* ========================================================================== */

/* Negative Tests */
void test_neg_pmic_checkHandle_nullHandle(void);
void test_neg_pmic_checkHandle_invalidInitStat(void);
void test_neg_pmic_checkHandle_invalidCommMode(void);
void test_neg_pmic_checkHandle_nullCritSec(void);
void test_neg_pmic_checkHandle_nullTimerWithRetry(void);

/* Positive Tests */
void test_pos_pmic_checkHandle_validHandle(void);
void test_pos_pmic_checkHandle_afterInit(void);
void test_pos_pmic_checkHandle_detectsUninit(void);
void test_pos_pmic_checkHandle_detectsMissingIoRead(void);
void test_pos_pmic_checkHandle_detectsMissingCommHandle(void);
void test_pos_pmic_checkHandle_comprehensive(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__PMIC_TEST_H__*/
