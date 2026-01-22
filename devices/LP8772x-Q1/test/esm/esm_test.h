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


#ifndef ESM_TEST_H
#define ESM_TEST_H

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Entry point for ESM module tests.
 *
 * @param args [IN] Test arguments (unused).
 */
void esm_test(void *args);

/* ========================================================================== */
/*                  esmClrStatus API Tests                                    */
/* ========================================================================== */
void test_pos_esm_esmClrStatus_allFields(void);
void test_pos_esm_esmClrStatus_failInt(void);
void test_pos_esm_esmClrStatus_pinInt(void);
void test_pos_esm_esmClrStatus_rstInt(void);
void test_neg_esm_esmClrStatus_invalidValidParams(void);
void test_neg_esm_esmClrStatus_nullEsmStat(void);
void test_neg_esm_esmClrStatus_nullHandle(void);
void test_neg_esm_esmClrStatus_zeroValidParams(void);
void test_neg_esm_esmClrStatus_ioWriteFailure(void);

/* ========================================================================== */
/*                  esmGetCfg API Tests                                       */
/* ========================================================================== */
void test_pos_esm_esmGetCfg_delay1(void);
void test_pos_esm_esmGetCfg_delay2(void);
void test_pos_esm_esmGetCfg_delay2Only(void);
void test_pos_esm_esmGetCfg_errCntThr(void);
void test_pos_esm_esmGetCfg_hmax(void);
void test_pos_esm_esmGetCfg_hmaxOnly(void);
void test_pos_esm_esmGetCfg_hmin(void);
void test_pos_esm_esmGetCfg_hminOnly(void);
void test_pos_esm_esmGetCfg_lmax(void);
void test_pos_esm_esmGetCfg_lmaxOnly(void);
void test_pos_esm_esmGetCfg_lmin(void);
void test_pos_esm_esmGetCfg_lminOnly(void);
void test_pos_esm_esmGetCfg_mode(void);
void test_neg_esm_esmGetCfg_invalidValidParams(void);
void test_neg_esm_esmGetCfg_nullEsmCfg(void);
void test_neg_esm_esmGetCfg_nullHandle(void);
void test_neg_esm_esmGetCfg_cascadeFailure_delayToHmax(void);
void test_neg_esm_esmGetCfg_cascadeFailure_modeToDelay(void);
void test_neg_esm_esmGetCfg_delay1ReadFailure(void);
void test_neg_esm_esmGetCfg_hmaxReadFailure(void);
void test_neg_esm_esmGetCfg_lmaxReadFailure(void);
void test_neg_esm_esmGetCfg_modeCfgReadFailure(void);

/* ========================================================================== */
/*                  esmGetEnableState API Tests                               */
/* ========================================================================== */
void test_pos_esm_esmGetEnableState_enableDisable(void);
void test_neg_esm_esmGetEnableState_nullHandle(void);
void test_neg_esm_esmGetEnableState_nullIsEnabled(void);
void test_neg_esm_esmGetEnableState_ioReadFailure(void);

/* ========================================================================== */
/*                  esmGetErrCnt API Tests                                    */
/* ========================================================================== */
void test_pos_esm_esmGetErrCnt_getCount(void);
void test_neg_esm_esmGetErrCnt_nullEsmErrCnt(void);
void test_neg_esm_esmGetErrCnt_nullHandle(void);
void test_neg_esm_esmGetErrCnt_ioReadFailure(void);

/* ========================================================================== */
/*                  esmGetStartState API Tests                                */
/* ========================================================================== */
void test_pos_esm_esmGetStartState_startStop(void);
void test_neg_esm_esmGetStartState_nullHandle(void);
void test_neg_esm_esmGetStartState_nullStarted(void);
void test_neg_esm_esmGetStartState_ioReadFailure(void);

/* ========================================================================== */
/*                  esmGetStatus API Tests                                    */
/* ========================================================================== */
void test_pos_esm_esmGetStatus_allFields(void);
void test_pos_esm_esmGetStatus_failInt(void);
void test_pos_esm_esmGetStatus_pinInt(void);
void test_pos_esm_esmGetStatus_rstInt(void);
void test_neg_esm_esmGetStatus_invalidValidParams(void);
void test_neg_esm_esmGetStatus_nullEsmStat(void);
void test_neg_esm_esmGetStatus_nullHandle(void);
void test_neg_esm_esmGetStatus_zeroValidParams(void);
void test_neg_esm_esmGetStatus_ioReadFailure(void);

/* ========================================================================== */
/*                  esmSetCfg API Tests                                       */
/* ========================================================================== */
void test_pos_esm_esmSetCfg_combinedConfiguration(void);
void test_pos_esm_esmSetCfg_configurationReadbackVerification(void);
void test_pos_esm_esmSetCfg_delay1(void);
void test_pos_esm_esmSetCfg_delay2(void);
void test_pos_esm_esmSetCfg_delay2Only(void);
void test_pos_esm_esmSetCfg_errCntThr(void);
void test_pos_esm_esmSetCfg_hmax(void);
void test_pos_esm_esmSetCfg_hmin(void);
void test_pos_esm_esmSetCfg_hminOnly(void);
void test_pos_esm_esmSetCfg_lmax(void);
void test_pos_esm_esmSetCfg_lmin(void);
void test_pos_esm_esmSetCfg_lminOnly(void);
void test_pos_esm_esmSetCfg_mode(void);
void test_pos_esm_esmSetCfg_pwmModeConfiguration(void);
void test_neg_esm_esmSetCfg_invalidErrCntThr(void);
void test_neg_esm_esmSetCfg_invalidMode(void);
void test_neg_esm_esmSetCfg_invalidValidParams(void);
void test_neg_esm_esmSetCfg_nullEsmCfg(void);
void test_neg_esm_esmSetCfg_nullHandle(void);
void test_neg_esm_esmSetCfg_cascadeFailure_delayToHmax(void);
void test_neg_esm_esmSetCfg_cascadeFailure_hmaxToLmax(void);
void test_neg_esm_esmSetCfg_delay1ReadFailure(void);
void test_neg_esm_esmSetCfg_hmaxReadFailure(void);
void test_neg_esm_esmSetCfg_lmaxReadFailure(void);
void test_neg_esm_esmSetCfg_modeCfgReadFailure(void);

/* ========================================================================== */
/*                  esmSetEnableState API Tests                               */
/* ========================================================================== */
void test_neg_esm_esmSetEnableState_nullHandle(void);
void test_neg_esm_esmSetEnableState_ioReadFailure(void);

/* ========================================================================== */
/*                  esmSetStartState API Tests                                */
/* ========================================================================== */
void test_neg_esm_esmSetStartState_nullHandle(void);
void test_neg_esm_esmSetStartState_ioReadFailure(void);

/* ========================================================================== */
/*                  esmStart API Tests                                        */
/* ========================================================================== */
void test_pos_esm_esmStart_start(void);
void test_neg_esm_esmStart_nullHandle(void);

/* ========================================================================== */
/*                  esmStop API Tests                                         */
/* ========================================================================== */
void test_pos_esm_esmStop_stop(void);
void test_neg_esm_esmStop_nullHandle(void);

/* ========================================================================== */
/*                  Integration Tests                                         */
/* ========================================================================== */
void test_pos_esm_integration_completeConfigurationSequence(void);
void test_pos_esm_integration_enableConfigureStartSequence(void);

#endif /* ESM_TEST_H */
