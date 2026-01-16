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

#include "../platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void pmic_init_test(void *args);

/* Negative Tests - NULL Parameters */
void test_negative_init_nullHandle(void);
void test_negative_init_nullCoreCfg(void);
void test_negative_init_nullCommHandle(void);
void test_negative_init_nullIoRead(void);
void test_negative_init_nullIoWrite(void);
void test_negative_init_nullCritSecStart(void);
void test_negative_init_nullCritSecStop(void);
void test_negative_deinit_nullHandle(void);
void test_negative_checkHandle_nullHandle(void);

/* Negative Tests - Invalid Parameters */
void test_negative_init_invalidDeviceType(void);
void test_negative_init_invalidCommMode(void);
void test_negative_init_insufficientCfg_missingIoRead(void);
void test_negative_init_insufficientCfg_missingIoWrite(void);
void test_negative_init_insufficientCfg_missingCritSec(void);
void test_negative_checkHandle_invalidInitStat(void);

/* Positive Tests - Basic Init/Deinit */
void test_positive_init_spiMode(void);
void test_positive_init_validateHandle(void);
void test_positive_deinit_clearsHandle(void);
void test_positive_checkHandle_validHandle(void);

/* Positive Tests - Initialization Variations */
void test_positive_init_withAllCallbacks(void);
void test_positive_init_multipleInitDeinit(void);
void test_positive_init_handlePersistence(void);

/* Positive Tests - Configuration Validation */
void test_positive_init_validDeviceType(void);
void test_positive_init_validCommMode(void);
void test_positive_init_validateCommHandleStored(void);
void test_positive_init_validateCallbacksStored(void);

/* Positive Tests - Communication Validation */
void test_positive_init_communicationTest(void);
void test_positive_init_registerAccess(void);

/* Positive Tests - Handle Validation */
void test_positive_checkHandle_afterInit(void);
void test_positive_checkHandle_detectsUninit(void);
void test_positive_checkHandle_detectsMissingIoRead(void);
void test_positive_checkHandle_detectsMissingCommHandle(void);

/* Positive Tests - Initialization Lifecycle */
void test_positive_init_deinit_reinit(void);
void test_positive_init_cleanStateAfterDeinit(void);
void test_positive_init_verifySubsystemInfo(void);

/* Positive Tests - Configuration Combinations */
void test_positive_init_minimalConfig(void);
void test_positive_init_fullConfig(void);
void test_positive_init_verifyInitMagic(void);

/* Positive Tests - Advanced */
void test_positive_init_critSecFunctions(void);
void test_positive_init_verifyDeviceComm(void);

/* Positive Tests - CRC Configuration */
void test_positive_init_with_crc_enabled(void);
void test_positive_init_with_config_crc_enabled(void);
void test_positive_init_with_both_crc_enabled(void);
void test_positive_init_crc_error_recovery(void);

/* Phase 3 Tests - Device Info Retrieval & Complete Flow */
void test_positive_init_complete_flow(void);
void test_positive_init_device_info_retrieval(void);
void test_positive_init_communication_validation(void);
void test_positive_init_spi_comprehensive(void);
void test_positive_deinit_success_comprehensive(void);
void test_positive_checkHandle_comprehensive(void);

/* Optional config field tests */
void test_positive_init_withRetryCnt(void);
void test_positive_init_withRetryInterval(void);
void test_positive_init_withTimerWaitMs(void);
void test_negative_init_timerWaitNull(void);

/* Coverage tests for pmic.c checkHandle validation paths */
void test_negative_checkHandle_invalidCommMode(void);
void test_negative_checkHandle_nullCritSec(void);
void test_negative_checkHandle_nullTimerWithRetry(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__PMIC_INIT_TEST_H__*/
