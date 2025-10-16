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
 * @file platform.c
 * @brief Source file containing definitions to PMIC Init tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "pmic_init_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all PMIC_INIT tests */
#define PMIC_INIT_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_commHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_ioRead); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_ioWrite); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_critSecStart); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_critSecStop); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_deinit_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_init); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_deinit)

/* Run all PMIC_INIT negative tests */
#define PMIC_INIT_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_commHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_ioRead); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_ioWrite); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_critSecStart); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_critSecStop); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_deinit_nullParam_pmicHandle)

/* Run all PMIC_INIT positive tests */
#define PMIC_INIT_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_Pmic_init); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_deinit)

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static void pmicInitTest_initPmicCfg(Pmic_CoreCfg_t *pmicCfg);
static void pmicInitTest_nullParamPmicInit(const char *param);

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_CoreHandle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void pmic_init_test(void *args)
{
    platform_init();

    platform_printString("\r\n");
    platform_printString("PMIC_INIT_TEST\r\n");
    platform_printString("--------------\r\n\r\n");

    platform_setupTests();
    PMIC_INIT_TEST_RUN_ALL();
    platform_tearDownTests();

    platform_deinit();
}

static void pmicInitTest_initPmicCfg(Pmic_CoreCfg_t *pmicCfg)
{
    pmicCfg->i2cAddr = 0x60U;
    pmicCfg->commHandle = platform_getCommHandle();
    pmicCfg->ioRead = &platform_rxByte;
    pmicCfg->ioWrite = &platform_txByte;
    pmicCfg->critSecStart = &platform_critSecStart;
    pmicCfg->critSecStop = &platform_critSecStop;
    pmicCfg->irqResponse = &platform_irqResponse;
}

void test_negative_Pmic_init_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_init()
    Pmic_CoreCfg_t pmicCfg = {0U};
    pmicInitTest_initPmicCfg(&pmicCfg);
    int32_t status = Pmic_init(&pmicCfg, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullParam_pmicCfg(void)
{
    // Pass NULL pmicCfg into Pmic_init()
    int32_t status = Pmic_init(NULL, &pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void pmicInitTest_nullParamPmicInit(const char *param)
{
    Pmic_CoreCfg_t pmicCfg = {0U};
    pmicInitTest_initPmicCfg(&pmicCfg);

    if (strcmp(param, "commHandle") == 0U) {
        pmicCfg.commHandle = NULL;
    } else if (strcmp(param, "ioRead") == 0U) {
        pmicCfg.ioRead = NULL;
    } else if (strcmp(param, "ioWrite") == 0U) {
        pmicCfg.ioWrite = NULL;
    } else if (strcmp(param, "critSecStart") == 0U) {
        pmicCfg.critSecStart = NULL;
    } else if (strcmp(param, "critSecStop") == 0U) {
        pmicCfg.critSecStop = NULL;
    } else {
        PLATFORM_ASSERT(0U);
    }

    int32_t status = Pmic_init(&pmicCfg, &pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullParam_pmicCfg_commHandle(void)
{
    // Pass NULL commHandle into Pmic_init()
    pmicInitTest_nullParamPmicInit("commHandle");
}

void test_negative_Pmic_init_nullParam_pmicCfg_ioRead(void)
{
    // Pass NULL ioRead into Pmic_init()
    pmicInitTest_nullParamPmicInit("ioRead");
}

void test_negative_Pmic_init_nullParam_pmicCfg_ioWrite(void)
{
    // Pass NULL ioWrite into Pmic_init()
    pmicInitTest_nullParamPmicInit("ioWrite");
}

void test_negative_Pmic_init_nullParam_pmicCfg_critSecStart(void)
{
    // Pass NULL critSecStart into Pmic_init()
    pmicInitTest_nullParamPmicInit("critSecStart");
}

void test_negative_Pmic_init_nullParam_pmicCfg_critSecStop(void)
{
    // Pass NULL critSecStop into Pmic_init()
    pmicInitTest_nullParamPmicInit("critSecStop");
}

void test_negative_Pmic_deinit_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_deinit()
    int32_t status = Pmic_deinit(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_positive_Pmic_init(void)
{
    // Initialize PMIC LLD
    Pmic_CoreCfg_t pmicCfg = {0U};
    pmicInitTest_initPmicCfg(&pmicCfg);
    int32_t status = Pmic_init(&pmicCfg, &pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_Pmic_deinit(void)
{
    // De-initialize PMIC LLD
    int32_t status = Pmic_deinit(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.drvInitStat == 0U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr == 0U);
    PLATFORM_ASSERT(pmicHandle.devRev == 0U);
    PLATFORM_ASSERT(pmicHandle.nvmId == 0U);
    PLATFORM_ASSERT(pmicHandle.nvmRev == 0U);
    PLATFORM_ASSERT(pmicHandle.siliconRev == 0U);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.commHandle == NULL);
    PLATFORM_ASSERT(pmicHandle.ioRead == NULL);
    PLATFORM_ASSERT(pmicHandle.ioWrite == NULL);
    PLATFORM_ASSERT(pmicHandle.critSecStart == NULL);
    PLATFORM_ASSERT(pmicHandle.critSecStop == NULL);
    PLATFORM_ASSERT(pmicHandle.irqResponse == NULL);
}

/**
 * @brief Some testing frameworks require an API to setup tests. Rename/rewrite
 * as necessary.
 */
void setUp(void)
{
}

/**
 * @brief Some testing frameworks require an API to teardown tests.
 * Rename/rewrite as necessary.
 */
void tearDown(void)
{
}
