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
#ifndef FSM_TEST_H
#define FSM_TEST_H

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                       Test APIs: fsmSetTriggerCfg                        */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMSETTRIGGERCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetTriggerCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetTriggerCfg_nullCfg); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetTriggerCfg_invalidSevereErrTrig); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmotherRailTrig_exceeds_max); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmsocRailTrig_exceeds_max); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmmcuRailTrig_exceeds_max); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmmoderateErrTrig_exceeds_max); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmsetTriggerCfg_zero_valid_params)

#define FSM_TEST_POS_FSMSETGETTRIGGERCFG() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetTriggerCfg_allTriggers); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetTriggerCfg_severeErrorTrigger); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetTriggerCfg_combinedTriggers); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetTriggerCfg_otherRailTrig); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetTriggerCfg_socRailTrig); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetTriggerCfg_mcuRailTrig); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetTriggerCfg_moderateErrTrig)

/* Test: TC-FSM-0015 */
#define FSM_TEST_FSMSETTRIGGERCFG() \
    FSM_TEST_POS_FSMSETGETTRIGGERCFG(); \
    FSM_TEST_NEG_FSMSETTRIGGERCFG()

/* ======================================================================== */
/*                       Test APIs: fsmGetTriggerCfg                        */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMGETTRIGGERCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetTriggerCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetTriggerCfg_nullCfg); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmgetTriggerCfg_zero_valid_params)

/* Test: TC-FSM-0016 */
#define FSM_TEST_FSMGETTRIGGERCFG() \
    FSM_TEST_NEG_FSMGETTRIGGERCFG()

/* ======================================================================== */
/*                     Test APIs: fsmSetGpioTriggerCfg                      */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMSETGPIOTRIGGERCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetGpioTriggerCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetGpioTriggerCfg_nullCfg); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetGpioTriggerCfg_invalidPin); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmsetGpioTriggerCfg_zero_valid_params); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmsetGpioTrigger_invalidMaskPol)

#define FSM_TEST_POS_FSMSETGETGPIOTRIGGERCFG() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetGpioTriggerCfg_gpio1); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGpioTriggerCfg_allGpioPins); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGpioTriggerCfg_polarity)

/* Test: TC-FSM-0017 */
#define FSM_TEST_FSMSETGPIOTRIGGERCFG() \
    FSM_TEST_POS_FSMSETGETGPIOTRIGGERCFG(); \
    FSM_TEST_NEG_FSMSETGPIOTRIGGERCFG()

/* ======================================================================== */
/*                     Test APIs: fsmGetGpioTriggerCfg                      */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMGETGPIOTRIGGERCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetGpioTriggerCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetGpioTriggerCfg_nullCfg); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetGpioTriggerCfg_invalidPin); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmgetGpioTriggerCfg_zero_valid_params)

/* Test: TC-FSM-0018 */
#define FSM_TEST_FSMGETGPIOTRIGGERCFG() \
    FSM_TEST_NEG_FSMGETGPIOTRIGGERCFG()

/* ======================================================================== */
/*                        Test APIs: fsmGetRecovCnt                         */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMGETRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCnt_nullRecovCnt)

#define FSM_TEST_POS_FSMGETRECOVCNT() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetClrRecovCnt_readAndClear)

/* Test: TC-FSM-0028 */
#define FSM_TEST_FSMGETRECOVCNT() \
    FSM_TEST_POS_FSMGETRECOVCNT(); \
    FSM_TEST_NEG_FSMGETRECOVCNT()

/* ======================================================================== */
/*                        Test APIs: fsmClrRecovCnt                         */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMCLRRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmClrRecovCnt_nullHandle)

/* Test: TC-FSM-0026 */
#define FSM_TEST_FSMCLRRECOVCNT() \
    FSM_TEST_NEG_FSMCLRRECOVCNT()

/* ======================================================================== */
/*                       Test APIs: fsmSetRecovCntThr                       */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMSETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetRecovCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetRecovCntThr_invalidValue)

#define FSM_TEST_POS_FSMSETGETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetRecovCntThr_allValues)

/* Test: TC-FSM-0033 */
#define FSM_TEST_FSMSETRECOVCNTTHR() \
    FSM_TEST_POS_FSMSETGETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMSETRECOVCNTTHR()

/* ======================================================================== */
/*                       Test APIs: fsmGetRecovCntThr                       */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMGETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCntThr_nullRecovCntThr)

/* Test: TC-FSM-0029 */
#define FSM_TEST_FSMGETRECOVCNTTHR() \
    FSM_TEST_NEG_FSMGETRECOVCNTTHR()

/* ======================================================================== */
/*                     Test APIs: fsmSendSoftRebootReq                      */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMSENDSOFTREBOOTREQ() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSendSoftRebootReq_nullHandle)

#define FSM_TEST_POS_FSMSENDSOFTREBOOTREQ() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSendSoftRebootReq_validRequest)

/* Test: TC-FSM-0023 */
#define FSM_TEST_FSMSENDSOFTREBOOTREQ() \
    FSM_TEST_POS_FSMSENDSOFTREBOOTREQ(); \
    FSM_TEST_NEG_FSMSENDSOFTREBOOTREQ()

/* ======================================================================== */
/*                       Test APIs: fsmSetStartupDest                       */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMSETSTARTUPDEST() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetStartupDest_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetStartupDest_invalidDest); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetStartupDest_invalidState)

#define FSM_TEST_POS_FSMSETGETSTARTUPDEST() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetGetStartupDest_allDestinations); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetStartupDest_validRead)

/* Test: TC-FSM-0024 */
#define FSM_TEST_FSMSETSTARTUPDEST() \
    FSM_TEST_POS_FSMSETGETSTARTUPDEST(); \
    FSM_TEST_NEG_FSMSETSTARTUPDEST()

/* ======================================================================== */
/*                       Test APIs: fsmGetStartupDest                       */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMGETSTARTUPDEST() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetStartupDest_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetStartupDest_nullDestination)

/* Test: TC-FSM-0025 */
#define FSM_TEST_FSMGETSTARTUPDEST() \
    FSM_TEST_NEG_FSMGETSTARTUPDEST()

/* ========================================================================== */
/*                          Aggregate test runners                            */
/* ========================================================================== */
#define FSM_TEST_RUN_POSITIVE() \
    FSM_TEST_POS_FSMSETGETTRIGGERCFG(); \
    FSM_TEST_POS_FSMSETGETGPIOTRIGGERCFG(); \
    FSM_TEST_POS_FSMGETRECOVCNT(); \
    FSM_TEST_POS_FSMSETGETRECOVCNTTHR(); \
    FSM_TEST_POS_FSMSENDSOFTREBOOTREQ(); \
    FSM_TEST_POS_FSMSETGETSTARTUPDEST()

#define FSM_TEST_RUN_NEGATIVE() \
    FSM_TEST_NEG_FSMSETTRIGGERCFG(); \
    FSM_TEST_NEG_FSMGETTRIGGERCFG(); \
    FSM_TEST_NEG_FSMSETGPIOTRIGGERCFG(); \
    FSM_TEST_NEG_FSMGETGPIOTRIGGERCFG(); \
    FSM_TEST_NEG_FSMGETRECOVCNT(); \
    FSM_TEST_NEG_FSMCLRRECOVCNT(); \
    FSM_TEST_NEG_FSMSETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMGETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMSENDSOFTREBOOTREQ(); \
    FSM_TEST_NEG_FSMSETSTARTUPDEST(); \
    FSM_TEST_NEG_FSMGETSTARTUPDEST()

#define FSM_TEST_RUN_ALL() \
    FSM_TEST_FSMSETTRIGGERCFG(); \
    FSM_TEST_FSMGETTRIGGERCFG(); \
    FSM_TEST_FSMSETGPIOTRIGGERCFG(); \
    FSM_TEST_FSMGETGPIOTRIGGERCFG(); \
    FSM_TEST_FSMGETRECOVCNT(); \
    FSM_TEST_FSMCLRRECOVCNT(); \
    FSM_TEST_FSMSETRECOVCNTTHR(); \
    FSM_TEST_FSMGETRECOVCNTTHR(); \
    FSM_TEST_FSMSENDSOFTREBOOTREQ(); \
    FSM_TEST_FSMSETSTARTUPDEST(); \
    FSM_TEST_FSMGETSTARTUPDEST()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Entry point for FSM module tests.
 *
 * @param args [IN] Optional arguments (not currently used).
 */
void fsm_test(void *args);

/* Negative test functions */
void test_neg_fsm_fsmSetTriggerCfg_nullHandle(void);
void test_neg_fsm_fsmSetTriggerCfg_nullCfg(void);
void test_neg_fsm_fsmSetTriggerCfg_invalidSevereErrTrig(void);
void test_neg_fsm_fsmGetTriggerCfg_nullHandle(void);
void test_neg_fsm_fsmGetTriggerCfg_nullCfg(void);
void test_neg_fsm_fsmSetGpioTriggerCfg_nullHandle(void);
void test_neg_fsm_fsmSetGpioTriggerCfg_nullCfg(void);
void test_neg_fsm_fsmSetGpioTriggerCfg_invalidPin(void);
void test_neg_fsm_fsmGetGpioTriggerCfg_nullHandle(void);
void test_neg_fsm_fsmGetGpioTriggerCfg_nullCfg(void);
void test_neg_fsm_fsmGetGpioTriggerCfg_invalidPin(void);
void test_neg_fsm_fsmGetRecovCnt_nullHandle(void);
void test_neg_fsm_fsmGetRecovCnt_nullRecovCnt(void);
void test_neg_fsm_fsmClrRecovCnt_nullHandle(void);
void test_neg_fsm_fsmSetRecovCntThr_nullHandle(void);
void test_neg_fsm_fsmSetRecovCntThr_invalidValue(void);
void test_neg_fsm_fsmGetRecovCntThr_nullHandle(void);
void test_neg_fsm_fsmGetRecovCntThr_nullRecovCntThr(void);
void test_neg_fsm_fsmSendSoftRebootReq_nullHandle(void);
void test_neg_fsm_fsmSetStartupDest_nullHandle(void);
void test_neg_fsm_fsmSetStartupDest_invalidDest(void);
void test_neg_fsm_fsmGetStartupDest_nullHandle(void);
void test_neg_fsm_fsmGetStartupDest_nullDestination(void);
void test_neg_fsm_fsmotherRailTrig_exceeds_max(void);
void test_neg_fsm_fsmsocRailTrig_exceeds_max(void);
void test_neg_fsm_fsmmcuRailTrig_exceeds_max(void);
void test_neg_fsm_fsmmoderateErrTrig_exceeds_max(void);
void test_neg_fsm_fsmsetTriggerCfg_zero_valid_params(void);
void test_neg_fsm_fsmgetTriggerCfg_zero_valid_params(void);
void test_neg_fsm_fsmsetGpioTriggerCfg_zero_valid_params(void);
void test_neg_fsm_fsmgetGpioTriggerCfg_zero_valid_params(void);
void test_neg_fsm_fsmSetStartupDest_invalidState(void);
void test_neg_fsm_fsmsetGpioTrigger_invalidMaskPol(void);

/* Positive test functions */
void test_pos_fsm_fsmSetGetTriggerCfg_allTriggers(void);
void test_pos_fsm_fsmSetGetTriggerCfg_severeErrorTrigger(void);
void test_pos_fsm_fsmSetGetGpioTriggerCfg_gpio1(void);
void test_pos_fsm_fsmSetGpioTriggerCfg_allGpioPins(void);
void test_pos_fsm_fsmSetGetRecovCntThr_allValues(void);
void test_pos_fsm_fsmGetClrRecovCnt_readAndClear(void);
void test_pos_fsm_fsmSetGetStartupDest_allDestinations(void);
void test_pos_fsm_fsmSendSoftRebootReq_validRequest(void);
void test_pos_fsm_fsmSetGetTriggerCfg_combinedTriggers(void);
void test_pos_fsm_fsmSetGpioTriggerCfg_polarity(void);
void test_pos_fsm_fsmSetGetTriggerCfg_otherRailTrig(void);
void test_pos_fsm_fsmSetGetTriggerCfg_socRailTrig(void);
void test_pos_fsm_fsmSetGetTriggerCfg_mcuRailTrig(void);
void test_pos_fsm_fsmSetGetTriggerCfg_moderateErrTrig(void);
void test_pos_fsm_fsmGetStartupDest_validRead(void);

#endif /* FSM_TEST_H */
