/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file   pmic_ut_esm.c
 *
 *  \brief  PMIC Unit Test for testing PMIC ESM APIs
 *
 */

#include <pmic_ut_esm.h>

/* Pointer holds the pPmicCoreHandle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

static uint8_t pmic_device_info = 0U;

/*!
 * \brief   PMIC ESM Test Cases
 */
static Pmic_Ut_Tests_t pmic_esm_tests[] =
{
    /*! TestID
     *  TestDesc
     */
     {
         7769,
         "Pmic_esmStart: Test to Start and Stop ESM MCU"
     },
     {
         7770,
         "Pmic_esmStart: Test to Start and Stop ESM SOC"
     },
     {
         7771,
         "Pmic_esmStart: Parameter validation for handle"
     },
     {
         7772,
         "Pmic_esmEnable: Test to Enable and Disable ESM MCU"
     },
     {
         7773,
         "Pmic_esmEnable: Test to Enable and Disable ESM SOC"
     },
     {
         7774,
         "Pmic_esmEnable: Parameter validation for handle"
     },
     {
         7775,
         "Pmic_esmGetEnableState: Test to verify ESM MCU Enable readback"
     },
     {
         7776,
         "Pmic_esmGetEnableState: Test to verify ESM SOC Enable readback"
     },
     {
         7777,
         "Pmic_esmGetEnableState: Parameter validation for handle"
     },
     {
         7778,
         "Pmic_esmGetEnableState: Parameter validation for pEsmState"
     },
     {
         7779,
         "Pmic_esmSetConfiguration: Test to configure ESM MCU in Level Mode"
     },
     {
         7780,
         "Pmic_esmSetConfiguration: Test to configure ESM SOC in Level Mode"
     },
     {
         7781,
         "Pmic_esmSetConfiguration: Test to configure ESM MCU in PWM Mode"
     },
     {
         7782,
         "Pmic_esmSetConfiguration: Test to configure ESM SOC in PWM Mode"
     },
     {
         7783,
         "Pmic_esmSetConfiguration: Parameter validation for handle"
     },
     {
         7784,
         "Pmic_esmSetConfiguration: Parameter validation for esmDelay1"
     },
     {
         7785,
         "Pmic_esmSetConfiguration: Parameter validation for esmDelay2"
     },
     {
         7786,
         "Pmic_esmSetConfiguration: Parameter validation for esmHMAX"
     },
     {
         7787,
         "Pmic_esmSetConfiguration: Parameter validation for esmHMIN"
     },
     {
         7789,
         "Pmic_esmSetConfiguration: Parameter validation for esmLMAX"
     },
     {
         7790,
         "Pmic_esmSetConfiguration: Parameter validation for esmLMIN"
     },
     {
         7791,
         "Pmic_esmSetConfiguration: Parameter validation for esmErrCntThr"
     },
     {
         7792,
         "Pmic_esmGetConfiguration: Test to verify PMIC ESM MCU Get Configuration"
     },
     {
         7793,
         "Pmic_esmGetConfiguration: Test to verify PMIC ESM SOC Get Configuration"
     },
     {
         7794,
         "Pmic_esmGetConfiguration: Parameter validation for handle"
     },
     {
         7795,
         "Pmic_esmGetConfiguration: Parameter validation for pEsmCfg"
     },
     {
         7796,
         "Pmic_esmGetErrCnt: Test to read the current ESM MCU Error Count Value"
     },
     {
         7797,
         "Pmic_esmGetErrCnt: Test to read the current ESM SOC Error Count Value"
     },
     {
         7798,
         "Pmic_esmGetErrCnt: Parameter validation for handle"
     },
     {
         7799,
         "Pmic_esmGetErrCnt: Parameter validation for pEsmErrCnt"
     },
     {
         7836,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode RST Interrupt"
     },
     {
         7837,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode RST Interrupt"
     },
     {
         7838,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode FAIL Interrupt"
     },
     {
         7839,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode FAIL Interrupt"
     },
     {
         7840,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode PIN Interrupt"
     },
     {
         7841,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode PIN Interrupt"
     },
     {
         7842,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode RST Interrupt"
     },
     {
         7843,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode RST Interrupt"
     },
     {
         7844,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode FAIL Interrupt"
     },
     {
         7845,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode FAIL Interrupt"
     },
     {
         7846,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode PIN Interrupt"
     },
     {
         7847,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode PIN Interrupt"
     },
     {
         7848,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode PIN, Fail and RST Interrupts"
     },
     {
         7849,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode PIN, Fail and RST Interrupts"
     },
     {
         7850,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode PIN, Fail and RST Interrupts disabled"
     },
     {
         7851,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode PIN, Fail and RST Interrupts disabled"
     },
     {
         7852,
         "Pmic_esmSetInterrupt: Parameter validation for handle"
     },
     {
         7853,
         "Pmic_esmStart : Negative test to verify ESM SOC Start for HERA"
     },
     {
         7854,
         "Pmic_esmEnable : Negative test to verify ESM SOC Enable for HERA"
     },
     {
         7855,
         "Pmic_esmSetConfiguration : Negative test to verify PMIC ESM SOC Set configuration for ESM SOC Level Mode for HERA"
     },
     {
         7856,
         "Pmic_esmGetConfiguration : Negative test to verify PMIC ESM SOC Get configuration for ESM SOC Level Mode for HERA"
     },
     {
         7857,
         "Pmic_esmGetErrCnt : Negative test to verify Error count value for HERA"
     },
     {
         7858,
         "Pmic_esmSetInterrupt : Negative test to verify ESM interrupt enable for HERA"
     },
     {
         7859,
         "Pmic_esmGetEnableState : Negative test to verify ESM get state for HERA"
     },
};

/*!
 * \brief   Test to verify ESM MCU Start and Stop functionality
 */
static void test_pmic_esm_startEsm_esmMcuStart(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    bool esmState      = PMIC_ESM_START;

    test_pmic_print_unity_testcase_info(7769,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_ESM_STARTED, pmicStatus);

    esmState = PMIC_ESM_STOP;
    pmicStatus = Pmic_esmStart(pPmicCoreHandle,esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify ESM SOC Start and Stop functionality
 */
static void test_pmic_esm_startEsm_esmSocStart(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = PMIC_ESM_START;

    test_pmic_print_unity_testcase_info(7770,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_ESM_STARTED, pmicStatus);

    esmState = PMIC_ESM_STOP;
    pmicStatus = Pmic_esmStart(pPmicCoreHandle,esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_esm_startEsmPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType        = PMIC_ESM_MODE_SOC;
    bool esmState       = PMIC_ESM_START;

    test_pmic_print_unity_testcase_info(7771,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(NULL, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Test to verify ESM MCU Enable and Disable functionality
 */
static void test_pmic_esm_enableEsm_esmMcuEnable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    bool esmToggle     = PMIC_ESM_ENABLE;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7772,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_ENABLE, esmState);

    esmToggle = PMIC_ESM_DISABLE;
    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_DISABLE, esmState);
}

/*!
 * \brief   Test to verify ESM SOC Enable and Disable functionality
 */
static void test_pmic_esm_enableEsm_esmSocEnable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmToggle     = PMIC_ESM_ENABLE;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7773,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_ENABLE, esmState);

    esmToggle = PMIC_ESM_DISABLE;
    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_DISABLE, esmState);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_esm_enableEsmPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmToggle     = PMIC_ESM_ENABLE;

    test_pmic_print_unity_testcase_info(7774,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmEnable(NULL, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Test to verify ESM MCU Enable readback
 */
static void test_pmic_esm_getEnableState_esmMcu(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    bool esmToggle     = PMIC_ESM_ENABLE;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7775,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(true, esmState);

    esmToggle = PMIC_ESM_DISABLE;
    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(false, esmState);
}

/*!
 * \brief   Test to verify ESM SOC Enable readback
 */
static void test_pmic_esm_getEnableState_esmSoc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmToggle     = PMIC_ESM_ENABLE;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7776,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(true, esmState);

    esmToggle = PMIC_ESM_DISABLE;
    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(false, esmState);
}

/*!
 * \brief Parameter Validation for handle
 */
static void test_pmic_esm_getEnableStatePrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7777,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetEnableState(NULL, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief Parameter Validation for pEsmState
 */
static void test_pmic_esm_getEnableStatePrmValTest_esmState(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7778,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
}

/*!
 * \brief   Test to verify PMIC ESM MCU Set configuration for ESM MCU
 *          Level Mode
 */
static void test_pmic_esm_setConfiguration_esmMcuLevelMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7779,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
}

/*!
 * \brief   Test to verify PMIC ESM SOC Set configuration for ESM SOC
 *          Level Mode
 */
static void test_pmic_esm_setConfiguration_esmSocLevelMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7780,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
}

/*!
 * \brief   Test to verify PMIC ESM MCU Set configuration for ESM MCU
 *          PWM Mode and readback the ESM configuration
 */
static void test_pmic_esm_setConfiguration_esmMcuPwmMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT   |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7781,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmax_us, esmCfg_rd.esmHmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmin_us, esmCfg_rd.esmHmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmax_us, esmCfg_rd.esmLmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmin_us, esmCfg_rd.esmLmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
    TEST_ASSERT_EQUAL(esmCfg.esmErrCntThr, esmCfg_rd.esmErrCntThr);
}

/*!
 * \brief   Test to verify PMIC ESM SOC Set configuration for ESM SOC
 *          PWM Mode and read back the ESM configuration.
 */
static void test_pmic_esm_setConfiguration_esmSocPwmMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7782,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmax_us, esmCfg_rd.esmHmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmin_us, esmCfg_rd.esmHmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmax_us, esmCfg_rd.esmLmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmin_us, esmCfg_rd.esmLmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
    TEST_ASSERT_EQUAL(esmCfg.esmErrCntThr, esmCfg_rd.esmErrCntThr);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_esm_setConfigurationPrmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7783,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(NULL, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Parameter validation for esmDelay1
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmDelay1(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        522401,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7784,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
}

/*!
 * \brief   Parameter validation for esmDelay2
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmDelay2(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        2048U,
        522401U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7785,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
}

/*!
 * \brief   Parameter validation for HMAX
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmHmax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        2048U,
        0U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7786,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
}

/*!
 * \brief   Parameter validation for HMIN
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmHmin(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        2048U,
        30U,
        0U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7787,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
}

/*!
 * \brief   Parameter validation for LMAX
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmLmax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        2048U,
        30U,
        30U,
        0U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7789,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
}

/*!
 * \brief   Parameter validation for LMIN
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmLmin(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        0U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7790,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
}

/*!
 * \brief   Parameter validation for ErrCntThr
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmErrCntThr(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        16U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7791,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Test to verify PMIC ESM MCU Get configuration
 */
static void test_pmic_esm_getConfiguration_esmMcuPwmMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT   |
                               PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_EN_DRV_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7792,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmax_us, esmCfg_rd.esmHmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmin_us, esmCfg_rd.esmHmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmax_us, esmCfg_rd.esmLmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmin_us, esmCfg_rd.esmLmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
    TEST_ASSERT_EQUAL(esmCfg.esmEnDrv, esmCfg_rd.esmEnDrv);
    TEST_ASSERT_EQUAL(esmCfg.esmErrCntThr, esmCfg_rd.esmErrCntThr);
}

/*!
 * \brief   Test to verify PMIC ESM SOC Get configuration
 */
static void test_pmic_esm_getConfiguration_esmSocPwmMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT   |
                               PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_EN_DRV_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7793,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmax_us, esmCfg_rd.esmHmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmin_us, esmCfg_rd.esmHmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmax_us, esmCfg_rd.esmLmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmin_us, esmCfg_rd.esmLmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
    TEST_ASSERT_EQUAL(esmCfg.esmEnDrv, esmCfg_rd.esmEnDrv);
    TEST_ASSERT_EQUAL(esmCfg.esmErrCntThr, esmCfg_rd.esmErrCntThr);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_esm_getConfigurationPrmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT   |
                               PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(7794,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetConfiguration(NULL, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Parameter validation for pEsmCfg
 */
static void test_esm_getConfigurationPrmValTest_pEsmCfg(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;

    test_pmic_print_unity_testcase_info(7795,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
}

/*!
 *  \brief Test to read to current ESM MCU Error Count value
 */
static void test_esm_getErrCnt_esmMcu(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmErrCnt  = 0U;
    bool esmType       = PMIC_ESM_MODE_MCU;

    test_pmic_print_unity_testcase_info(7796,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to read to current ESM SOC Error Count value
 */
static void test_esm_getErrCnt_esmSoc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmErrCnt  = 0U;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7797,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_esm_getErrCntPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmErrCnt  = 0U;
    bool esmType        = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7798,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(NULL, esmType, &esmErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Parameter validation for pEsmErrCnt
 */
static void test_esm_getErrCntPrmValTest_pEsmErrCnt(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType        = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7799,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode RST Interrupt
 */
static void test_esm_setInterrupt_esmMcuRstIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        true
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7836,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_RST_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU PWM Mode RST Interrupt
 */
static void test_esm_setInterrupt_esmMcuRstIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    uint8_t esmErrCnt         = 0U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        true
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7837,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_RST_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode FAIL Interrupt
 */
static void test_esm_setInterrupt_esmMcuFailIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7838,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_FAIL_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU PWM Mode FAIL Interrupt
 */
static void test_esm_setInterrupt_esmMcuFailIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    uint8_t esmErrCnt         = 0U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7839,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_FAIL_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode PIN Interrupt
 */
static void test_esm_setInterrupt_esmMcuPinIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7840,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_PIN_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU PWM Mode PIN Interrupt
 */
static void test_esm_setInterrupt_esmMcuPinIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    uint8_t esmErrCnt         = 0U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7841,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_PIN_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM SOC Level Mode RST Interrupt
 */
static void test_esm_setInterrupt_esmSocRstIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        true
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7842,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_RST_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode RST Interrupt
 */
static void test_esm_setInterrupt_esmSocRstIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    uint8_t esmErrCnt         = 0U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        true
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7843,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_RST_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM SOC Level Mode FAIL Interrupt
 */
static void test_esm_setInterrupt_esmSocFailIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7844,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_FAIL_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode FAIL Interrupt
 */
static void test_esm_setInterrupt_esmSocFailIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    uint8_t esmErrCnt         = 0U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7845,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_FAIL_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM SOC Level Mode PIN Interrupt
 */
static void test_esm_setInterrupt_esmSocPinIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7846,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_PIN_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode PIN Interrupt
 */
static void test_esm_setInterrupt_esmSocPinIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    uint8_t esmErrCnt         = 0U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7847,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_PIN_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode PIN, FAIL and RST Interrupts
 */
static void test_esm_setInterrupt_esmSocAllIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        true,
        true
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7848,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_PIN_INT);
                break;
            }
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_FAIL_INT);
                break;
            }
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_RST_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode PIN, FAIL and RST Interrupts
 */
static void test_esm_setInterrupt_esmMcuAllIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        true,
        true
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7849,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_PIN_INT);
                break;
            }
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_FAIL_INT);
                break;
            }
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_RST_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode PIN, FAIL and RST Interrupts
 *         disabled
 */
static void test_esm_setInterrupt_esmMcuAllIntrDisabled_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t pin               = 7U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7850,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
        (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) == 0U))
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
        (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) == 0U))
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
        (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) == 0U))
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode PIN, FAIL and RST Interrupts
 *         disabled
 */
static void test_esm_setInterrupt_esmSocAllIntrDisable_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t pin               = 3U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        true,
        true
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7851,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
        (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) == 0U))
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
        (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) == 0U))
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
        (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) == 0U))
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief Parameter validation for handle
 */
static void test_esm_setInterruptPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    test_pmic_print_unity_testcase_info(7852,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmSetInterrupt(NULL, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}


/*!
 * \brief   Pmic_esmStart : Negative test to verify ESM SOC Start for HERA
 */
static void test_pmic_esm_startEsm_esmSocStart_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = PMIC_ESM_START;

    test_pmic_print_unity_testcase_info(7853,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

}

/*!
 * \brief   Pmic_esmEnable : Negative test to verify ESM SOC Enable for HERA
 */
static void test_pmic_esm_startEsm_esmSocEnable_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7854,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);
}

/*!
 * \brief   Pmic_esmSetConfiguration : Negative test to verify PMIC ESM SOC Set configuration for ESM SOC
 *          Level Mode for HERA
 */
static void test_pmic_esm_setConfiguration_esmSocLevelMode_hera(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;

    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7855,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);
}

/*!
 * \brief   Pmic_esmGetConfiguration : Negative test to verify PMIC ESM SOC Get configuration for ESM SOC
 *          Level Mode for HERA
 */
static void test_pmic_esm_getConfiguration_esmSocLevelMode_hera(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT,};


    test_pmic_print_unity_testcase_info(7856,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);
}


/*!
 * \brief   Pmic_esmGetErrCnt : Negative test to verify Error count value for HERA
 */
static void test_Pmic_esmGetErrCnt_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;;
    uint8_t esmErrCnt  = 0U;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7857,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);

    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);
}


/*!
 * \brief   Pmic_esmSetInterrupt : Negative test to verify ESM interrupt enable for HERA
 */
static void test_Pmic_esmSetInterrupt_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    test_pmic_print_unity_testcase_info(7858,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);

    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);
}

/*!
 * \brief   Pmic_esmGetEnableState : Negative test to verify ESM get state for HERA
 */
static void test_Pmic_esmGetEnableState_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7859,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);

    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);
}

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run esm unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    RUN_TEST(test_pmic_esm_startEsm_esmMcuStart);
    RUN_TEST(test_pmic_esm_startEsm_esmSocStart);
    RUN_TEST(test_pmic_esm_startEsmPrmValTest_handle);
    RUN_TEST(test_pmic_esm_enableEsm_esmMcuEnable);
    RUN_TEST(test_pmic_esm_enableEsm_esmSocEnable);
    RUN_TEST(test_pmic_esm_enableEsmPrmValTest_handle);
    RUN_TEST(test_pmic_esm_getEnableState_esmMcu);
    RUN_TEST(test_pmic_esm_getEnableState_esmSoc);
    RUN_TEST(test_pmic_esm_getEnableStatePrmValTest_handle);
    RUN_TEST(test_pmic_esm_getEnableStatePrmValTest_esmState);
    RUN_TEST(test_pmic_esm_setConfiguration_esmMcuLevelMode);
    RUN_TEST(test_pmic_esm_setConfiguration_esmSocLevelMode);
    RUN_TEST(test_pmic_esm_setConfiguration_esmMcuPwmMode);
    RUN_TEST(test_pmic_esm_setConfiguration_esmSocPwmMode);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_handle);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmDelay1);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmDelay2);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmHmax);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmHmin);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmLmax);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmLmin);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmErrCntThr);
    RUN_TEST(test_pmic_esm_getConfiguration_esmMcuPwmMode);
    RUN_TEST(test_pmic_esm_getConfiguration_esmSocPwmMode);
    RUN_TEST(test_esm_getConfigurationPrmValTest_handle);
    RUN_TEST(test_esm_getConfigurationPrmValTest_pEsmCfg);
    RUN_TEST(test_esm_getErrCnt_esmMcu);
    RUN_TEST(test_esm_getErrCnt_esmSoc);
    RUN_TEST(test_esm_getErrCntPrmValTest_handle);
    RUN_TEST(test_esm_getErrCntPrmValTest_pEsmErrCnt);
    RUN_TEST(test_esm_setInterrupt_esmMcuRstIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuRstIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuFailIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuFailIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuPinIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuPinIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmSocRstIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmSocRstIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmSocFailIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmSocFailIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmSocPinIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmSocPinIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmSocAllIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuAllIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuAllIntrDisabled_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmSocAllIntrDisable_pwmMode);
    RUN_TEST(test_esm_setInterruptPrmValTest_handle);
    RUN_TEST(test_pmic_esm_startEsm_esmSocStart_hera);
    RUN_TEST(test_pmic_esm_startEsm_esmSocEnable_hera);
    RUN_TEST(test_pmic_esm_setConfiguration_esmSocLevelMode_hera);
    RUN_TEST(test_pmic_esm_getConfiguration_esmSocLevelMode_hera);
    RUN_TEST(test_Pmic_esmGetErrCnt_hera);
    RUN_TEST(test_Pmic_esmSetInterrupt_hera);
    RUN_TEST(test_Pmic_esmGetEnableState_hera);

    UNITY_END();
}

/*!
 * \brief   ESM Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_esm_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_DUAL_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr           = LEO_PMICA_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = LEO_PMICA_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

/*!
 * \brief   ESM Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_esm_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr           = LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

/*!
 * \brief   ESM Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_esm_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SPI;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

/*!
 * \brief   ESM Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_esm_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_HERA_LP8764X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr           = HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

static int32_t setup_pmic_interrupt()
{
    int32_t status = PMIC_ST_SUCCESS;

#ifdef SOC_J721E

    status = test_pmic_leo_pmicA_esm_testApp();
   /* Deinit pmic handle */
    if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
    {
        test_pmic_appDeInit(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = test_pmic_leo_pmicB_esm_testApp();
       /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }
    }
#endif
    return status;
}


static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM Using I2C Interface)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM Using I2C Interface)"
    " \r\n 2: Pmic Hera device"
    " \r\n 3: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 4: quit"
    " \r\n"
    " \r\n Enter option: "
};

/*!
 * \brief   Function to register ESM Unity Test App wrapper to Unity framework
 */
static void test_pmic_esm_testapp_runner(void)
{
    /* @description : Test runner for ESM Test App
     *
     * @requirements: 5833, 5846
     *
     * @cores       : mcu1_0, mcu1_1
     */

    int8_t num = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestAppMenu);
        if(UART_scanFmt("%d", &num) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(num)
        {
           case 0U:
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt())
                {
                   /* ESM Unity Test App wrapper Function for LEO PMIC-A */
                   test_pmic_leo_pmicA_esm_testApp();
                   pmic_device_info = J721E_LEO_PMICA_DEVICE;
                   /* Run esm test cases for Leo PMIC-A */
                   test_pmic_run_testcases();
                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
               break;
           case 1U:
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt())
                {
                   /* ESM Unity Test App wrapper Function for LEO PMIC-B */
                   test_pmic_leo_pmicB_esm_testApp();
                   pmic_device_info = J721E_LEO_PMICB_DEVICE;
                   pmic_log(" \r\n ESM feature cannot be tested on PMIC-B\n");

                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
               break;
           case 2U:
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt())
                {
                   /* ESM Unity Test App wrapper Function for HERA PMIC */
                   test_pmic_hera_esm_testApp();
                   /* Run esm test cases for Hera PMIC */
                   test_pmic_run_testcases();
                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
               break;
           case 3U:
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt())
                {
                   /* ESM Unity Test App wrapper Function for LEO PMIC-A using
                    * SPI stub functions */
                   test_pmic_leo_pmicA_spiStub_esm_testApp();
                   pmic_device_info = J721E_LEO_PMICA_DEVICE;
                   /* Run esm test cases for Leo PMIC-A */
                   test_pmic_run_testcases();
                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
               break;
           case 4U:
               pmic_log(" \r\n Quit from application\n");
               return;
           default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}
#endif

/*!
 * \brief   TI RTOS specific ESM TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    pmic_log("ESM Unity Test Application(%s %s)\n", __TIME__, __DATE__);
#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_esm_testapp_runner();
#endif
}
