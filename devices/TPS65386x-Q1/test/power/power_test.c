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

#include "power_test.h"
#ifdef BUILD_MOCK
#include "test_inject.h"
#include "pmic_mock_core.h"
#include "pmic_mock_types.h"
#endif
#include "test_constants.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */

/*                             Global Variables                               */
/* ========================================================================== */

#ifdef BUILD_MOCK
extern PmicMockDevice_t *platform_getMockDevice(void);
#endif

static Pmic_Handle_t pmicHandle;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/* Test helper functions for LDO configuration */
static void helper_setGetLdoCfg_mode(uint16_t ldo);
static void helper_setGetLdoCfg_lvl(uint16_t ldo);
static void helper_setGetLdoCfg_ilimLvl(uint16_t ldo);
static void helper_setGetLdoCfg_ilimDgl(uint16_t ldo);
static void helper_setGetLdoCfg_vmonThr(uint16_t ldo);
static void helper_setGetLdoCfg_vmonDgl(uint16_t ldo);
static void helper_setGetLdoCfg_rampTime(uint16_t ldo);
static void helper_setGetLdoCfg_disableDischarge(uint16_t ldo);
static void helper_setGetLdoCfg_includeOvUvStatInPGood(uint16_t ldo);

/* Test helper functions for PLDO configuration */
static void helper_setGetPldoCfg_mode(uint16_t pldo);
static void helper_setGetPldoCfg_trackingMode(uint16_t pldo);
static void helper_setGetPldoCfg_lvl(uint16_t pldo);
static void helper_setGetPldoCfg_ilimLvl(uint16_t pldo);
static void helper_setGetPldoCfg_ilimDgl(uint16_t pldo);
static void helper_setGetPldoCfg_vmonThr(uint16_t pldo);
static void helper_setGetPldoCfg_vmonDgl(uint16_t pldo);
static void helper_setGetPldoCfg_vtrackRange(uint16_t pldo);
static void helper_setGetPldoCfg_rampTime(uint16_t pldo);
static void helper_setGetPldoCfg_disableDischarge(uint16_t pldo);
static void helper_setGetPldoCfg_includeOvUvStatInPGood(uint16_t pldo);

/* Test helper functions for ExtVmon configuration */
static void helper_setGetExtVmonCfg_mode(uint16_t extVmon);
static void helper_setGetExtVmonCfg_vmonThr(uint16_t extVmon);
static void helper_setGetExtVmonCfg_vmonDgl(uint16_t extVmon);
static void helper_setGetExtVmonCfg_includeOvUvStatInPGood(uint16_t extVmon);

/* Negative test helper functions */
static void helper_negative_setLdoCfg_mode(uint16_t ldo);
static void helper_negative_setLdoCfg_lvl(uint16_t ldo);
static void helper_negative_setLdoCfg_ilimLvl(uint16_t ldo);
static void helper_negative_setLdoCfg_ilimDgl(uint16_t ldo);
static void helper_negative_setLdoCfg_vmonThr(uint16_t ldo);
static void helper_negative_setLdoCfg_vmonDgl(uint16_t ldo);
static void helper_negative_setLdoCfg_rampTime(uint16_t ldo);

static void helper_negative_setPldoCfg_mode(uint16_t pldo);
static void helper_negative_setPldoCfg_lvl(uint16_t pldo);
static void helper_negative_setPldoCfg_ilimLvl(uint16_t pldo);
static void helper_negative_setPldoCfg_ilimDgl(uint16_t pldo);
static void helper_negative_setPldoCfg_vmonThr(uint16_t pldo);
static void helper_negative_setPldoCfg_vmonDgl(uint16_t pldo);
static void helper_negative_setPldoCfg_vtrackRange(uint16_t pldo);
static void helper_negative_setPldoCfg_rampTime(uint16_t pldo);

static void helper_negative_setExtVmonCfg_mode(uint16_t extVmon);
static void helper_negative_setExtVmonCfg_vmonThr(uint16_t extVmon);
static void helper_negative_setExtVmonCfg_vmonDgl(uint16_t extVmon);

static void helper_getRsrcStatus(uint16_t pwrRsrc);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void power_test(void *args)
{
    (void)args;
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();

    testTimer_startModule("Power");

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = PMIC_CFG_INIT_COMM_MODE_VALID |
                       PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                       PMIC_CFG_INIT_IO_READ_VALID |
                       PMIC_CFG_INIT_IO_WRITE_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        platform_unlockRegisters();
        platform_setupTests();
        POWER_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    testTimer_endModule();

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

/* ========================================================================== */
/*                     NEGATIVE TESTS - NULL Parameters                       */
/* ========================================================================== */

void test_neg_power_pwrSetBuckBoostCfg_nullHandle(void)
{
    Pmic_PwrBuckBoostCfg_t bbCfg = {0};
    int32_t status = Pmic_pwrSetBuckBoostCfg(NULL, &bbCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetBuckBoostCfg_nullConfig(void)
{
    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetBuckBoostCfg_nullHandle(void)
{
    Pmic_PwrBuckBoostCfg_t bbCfg = {0};
    int32_t status = Pmic_pwrGetBuckBoostCfg(NULL, &bbCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetBuckBoostCfg_nullConfig(void)
{
    int32_t status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetLdoCfg_nullHandle(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.ldo = PMIC_PWR_LDO1};
    int32_t status = Pmic_pwrSetLdoCfg(NULL, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetLdoCfg_nullConfig(void)
{
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetLdoCfg_nullHandle(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.ldo = PMIC_PWR_LDO1};
    int32_t status = Pmic_pwrGetLdoCfg(NULL, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetLdoCfg_nullConfig(void)
{
    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetPldoCfg_nullHandle(void)
{
    Pmic_PwrPldoCfg_t pldoCfg = {.pldo = PMIC_PWR_PLDO1};
    int32_t status = Pmic_pwrSetPldoCfg(NULL, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetPldoCfg_nullConfig(void)
{
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetPldoCfg_nullHandle(void)
{
    Pmic_PwrPldoCfg_t pldoCfg = {.pldo = PMIC_PWR_PLDO1};
    int32_t status = Pmic_pwrGetPldoCfg(NULL, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetPldoCfg_nullConfig(void)
{
    int32_t status = Pmic_pwrGetPldoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetExtVmonCfg_nullHandle(void)
{
    Pmic_PwrExtVmonCfg_t vmonCfg = {.extVmon = PMIC_PWR_EXT_VMON1};
    int32_t status = Pmic_pwrSetExtVmonCfg(NULL, &vmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetExtVmonCfg_nullConfig(void)
{
    int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetExtVmonCfg_nullHandle(void)
{
    Pmic_PwrExtVmonCfg_t vmonCfg = {.extVmon = PMIC_PWR_EXT_VMON1};
    int32_t status = Pmic_pwrGetExtVmonCfg(NULL, &vmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetExtVmonCfg_nullConfig(void)
{
    int32_t status = Pmic_pwrGetExtVmonCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetRsrcStatus_nullHandle(void)
{
    Pmic_PwrRsrcStatus_t stat = {.pwrRsrc = PMIC_PWR_BUCK_BOOST};
    int32_t status = Pmic_pwrGetRsrcStatus(NULL, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetRsrcStatus_nullStatus(void)
{
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrClrRsrcStatus_nullHandle(void)
{
    Pmic_PwrRsrcStatus_t stat = {.pwrRsrc = PMIC_PWR_BUCK_BOOST};
    int32_t status = Pmic_pwrClrRsrcStatus(NULL, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrClrRsrcStatus_nullStatus(void)
{
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrClrRsrcStatusAll_nullHandle(void)
{
    int32_t status = Pmic_pwrClrRsrcStatusAll(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetPGoodInStby_nullHandle(void)
{
    int32_t status = Pmic_pwrSetPGoodInStby(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetPGoodInStby_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_pwrGetPGoodInStby(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetPGoodInStby_nullIsEnabled(void)
{
    int32_t status = Pmic_pwrGetPGoodInStby(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*              NEGATIVE TESTS - Buck/Boost Out of Bounds                     */
/* ========================================================================== */

void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_lvl(void)
{
    Pmic_PwrBuckBoostCfg_t bbCfg = {
        .validParams = PMIC_CFG_PWR_BB_LVL_VALID,
        .lvl = PMIC_PWR_BB_LVL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &bbCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_stbyLvl(void)
{
    Pmic_PwrBuckBoostCfg_t bbCfg = {
        .validParams = PMIC_CFG_PWR_BB_STBY_LVL_VALID,
        .stbyLvl = PMIC_PWR_BB_STBY_LVL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &bbCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_vmonThr(void)
{
    Pmic_PwrBuckBoostCfg_t bbCfg = {
        .validParams = PMIC_CFG_PWR_BB_VMON_THR_VALID,
        .vmonThr = PMIC_PWR_BB_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &bbCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_vmonDgl(void)
{
    Pmic_PwrBuckBoostCfg_t bbCfg = {
        .validParams = PMIC_CFG_PWR_BB_VMON_DGL_VALID,
        .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &bbCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_boostTmo(void)
{
    Pmic_PwrBuckBoostCfg_t bbCfg = {
        .validParams = PMIC_CFG_PWR_BB_BOOST_TMO_VALID,
        .boostTmo = PMIC_PWR_BOOST_TMO_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &bbCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                 NEGATIVE TESTS - LDO Out of Bounds Tests                   */
/* ========================================================================== */

static void helper_negative_setLdoCfg_mode(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_MODE_VALID,
        .ldo = ldo,
        .mode = PMIC_PWR_LDO_MODE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setLdoCfg_lvl(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_LVL_VALID,
        .ldo = ldo,
        .lvl = PMIC_PWR_LDO_LVL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setLdoCfg_ilimLvl(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_ILIM_LVL_VALID,
        .ldo = ldo,
        .ilimLvl = PMIC_PWR_LDO_ILIM_LVL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setLdoCfg_ilimDgl(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_ILIM_DGL_VALID,
        .ldo = ldo,
        .ilimDgl = PMIC_PWR_LDO_ILIM_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setLdoCfg_vmonThr(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_THR_VALID,
        .ldo = ldo,
        .vmonThr = PMIC_PWR_LDO_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setLdoCfg_vmonDgl(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_DGL_VALID,
        .ldo = ldo,
        .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setLdoCfg_rampTime(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_RAMP_TIME_VALID,
        .ldo = ldo,
        .rampTime = PMIC_PWR_RT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_invalidParam_ldo1_mode(void) { helper_negative_setLdoCfg_mode(PMIC_PWR_LDO1); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_lvl(void) { helper_negative_setLdoCfg_lvl(PMIC_PWR_LDO1); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_ilimLvl(void) { helper_negative_setLdoCfg_ilimLvl(PMIC_PWR_LDO1); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_ilimDgl(void) { helper_negative_setLdoCfg_ilimDgl(PMIC_PWR_LDO1); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_vmonThr(void) { helper_negative_setLdoCfg_vmonThr(PMIC_PWR_LDO1); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_vmonDgl(void) { helper_negative_setLdoCfg_vmonDgl(PMIC_PWR_LDO1); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_rampTime(void) { helper_negative_setLdoCfg_rampTime(PMIC_PWR_LDO1); }

void test_neg_power_pwrSetLdoCfg_invalidParam_ldo2_mode(void) { helper_negative_setLdoCfg_mode(PMIC_PWR_LDO2); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_lvl(void) { helper_negative_setLdoCfg_lvl(PMIC_PWR_LDO2); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_ilimLvl(void) { helper_negative_setLdoCfg_ilimLvl(PMIC_PWR_LDO2); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_ilimDgl(void) { helper_negative_setLdoCfg_ilimDgl(PMIC_PWR_LDO2); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_vmonThr(void) { helper_negative_setLdoCfg_vmonThr(PMIC_PWR_LDO2); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_vmonDgl(void) { helper_negative_setLdoCfg_vmonDgl(PMIC_PWR_LDO2); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_rampTime(void) { helper_negative_setLdoCfg_rampTime(PMIC_PWR_LDO2); }

void test_neg_power_pwrSetLdoCfg_invalidParam_ldo3_mode(void) { helper_negative_setLdoCfg_mode(PMIC_PWR_LDO3); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_lvl(void) { helper_negative_setLdoCfg_lvl(PMIC_PWR_LDO3); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_ilimLvl(void) { helper_negative_setLdoCfg_ilimLvl(PMIC_PWR_LDO3); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_ilimDgl(void) { helper_negative_setLdoCfg_ilimDgl(PMIC_PWR_LDO3); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_vmonThr(void) { helper_negative_setLdoCfg_vmonThr(PMIC_PWR_LDO3); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_vmonDgl(void) { helper_negative_setLdoCfg_vmonDgl(PMIC_PWR_LDO3); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_rampTime(void) { helper_negative_setLdoCfg_rampTime(PMIC_PWR_LDO3); }

void test_neg_power_pwrSetLdoCfg_invalidParam_ldo4_mode(void) { helper_negative_setLdoCfg_mode(PMIC_PWR_LDO4); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_lvl(void) { helper_negative_setLdoCfg_lvl(PMIC_PWR_LDO4); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_ilimLvl(void) { helper_negative_setLdoCfg_ilimLvl(PMIC_PWR_LDO4); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_ilimDgl(void) { helper_negative_setLdoCfg_ilimDgl(PMIC_PWR_LDO4); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_vmonThr(void) { helper_negative_setLdoCfg_vmonThr(PMIC_PWR_LDO4); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_vmonDgl(void) { helper_negative_setLdoCfg_vmonDgl(PMIC_PWR_LDO4); }
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_rampTime(void) { helper_negative_setLdoCfg_rampTime(PMIC_PWR_LDO4); }

/* ========================================================================== */
/*                NEGATIVE TESTS - PLDO Out of Bounds Tests                   */
/* ========================================================================== */

static void helper_negative_setPldoCfg_mode(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t pldoCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_MODE_VALID,
        .pldo = pldo,
        .mode = (pldo == PMIC_PWR_PLDO1) ? (PMIC_PWR_PLDO1_MODE_MAX + 1U) : (PMIC_PWR_PLDO2_MODE_MAX + 1U)
    };
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setPldoCfg_lvl(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t pldoCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_LVL_VALID,
        .pldo = pldo,
        .lvl = PMIC_PWR_PLDO_LVL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setPldoCfg_ilimLvl(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t pldoCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_ILIM_LVL_VALID,
        .pldo = pldo,
        .ilimLvl = PMIC_PWR_PLDO_ILIM_LVL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setPldoCfg_ilimDgl(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t pldoCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_ILIM_DGL_VALID,
        .pldo = pldo,
        .ilimDgl = PMIC_PWR_LDO_ILIM_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setPldoCfg_vmonThr(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t pldoCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_VMON_THR_VALID,
        .pldo = pldo,
        .vmonThr = PMIC_PWR_PLDO_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setPldoCfg_vmonDgl(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t pldoCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_VMON_DGL_VALID,
        .pldo = pldo,
        .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setPldoCfg_vtrackRange(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t pldoCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_VTRACK_RANGE_VALID,
        .pldo = pldo,
        .vtrackRange = PMIC_PWR_VTRACK_RANGE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setPldoCfg_rampTime(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t pldoCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_RT_VALID,
        .pldo = pldo,
        .rampTime = PMIC_PWR_RT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &pldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetPldoCfg_invalidParam_pldo1_mode(void) { helper_negative_setPldoCfg_mode(PMIC_PWR_PLDO1); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_lvl(void) { helper_negative_setPldoCfg_lvl(PMIC_PWR_PLDO1); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_ilimLvl(void) { helper_negative_setPldoCfg_ilimLvl(PMIC_PWR_PLDO1); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_ilimDgl(void) { helper_negative_setPldoCfg_ilimDgl(PMIC_PWR_PLDO1); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_vmonThr(void) { helper_negative_setPldoCfg_vmonThr(PMIC_PWR_PLDO1); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_vmonDgl(void) { helper_negative_setPldoCfg_vmonDgl(PMIC_PWR_PLDO1); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_vtrackRange(void) { helper_negative_setPldoCfg_vtrackRange(PMIC_PWR_PLDO1); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_rampTime(void) { helper_negative_setPldoCfg_rampTime(PMIC_PWR_PLDO1); }

void test_neg_power_pwrSetPldoCfg_invalidParam_pldo2_mode(void) { helper_negative_setPldoCfg_mode(PMIC_PWR_PLDO2); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_lvl(void) { helper_negative_setPldoCfg_lvl(PMIC_PWR_PLDO2); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_ilimLvl(void) { helper_negative_setPldoCfg_ilimLvl(PMIC_PWR_PLDO2); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_ilimDgl(void) { helper_negative_setPldoCfg_ilimDgl(PMIC_PWR_PLDO2); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_vmonThr(void) { helper_negative_setPldoCfg_vmonThr(PMIC_PWR_PLDO2); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_vmonDgl(void) { helper_negative_setPldoCfg_vmonDgl(PMIC_PWR_PLDO2); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_vtrackRange(void) { helper_negative_setPldoCfg_vtrackRange(PMIC_PWR_PLDO2); }
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_rampTime(void) { helper_negative_setPldoCfg_rampTime(PMIC_PWR_PLDO2); }

/* ========================================================================== */
/*             NEGATIVE TESTS - ExtVmon Out of Bounds Tests                   */
/* ========================================================================== */

static void helper_negative_setExtVmonCfg_mode(uint16_t extVmon)
{
    Pmic_PwrExtVmonCfg_t vmonCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_MODE_VALID,
        .extVmon = extVmon,
        .mode = PMIC_PWR_EXT_VMON_MODE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &vmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setExtVmonCfg_vmonThr(uint16_t extVmon)
{
    Pmic_PwrExtVmonCfg_t vmonCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_THR_VALID,
        .extVmon = extVmon,
        .vmonThr = PMIC_PWR_EXT_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &vmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void helper_negative_setExtVmonCfg_vmonDgl(uint16_t extVmon)
{
    Pmic_PwrExtVmonCfg_t vmonCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_DGL_VALID,
        .extVmon = extVmon,
        .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &vmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetExtVmonCfg_invalidParam_vmon1_mode(void)
{
    helper_negative_setExtVmonCfg_mode(PMIC_PWR_EXT_VMON1);
}
void test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon1_vmonThr(void)
{
    helper_negative_setExtVmonCfg_vmonThr(PMIC_PWR_EXT_VMON1);
}
void test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon1_vmonDgl(void)
{
    helper_negative_setExtVmonCfg_vmonDgl(PMIC_PWR_EXT_VMON1);
}

void test_neg_power_pwrSetExtVmonCfg_invalidParam_vmon2_mode(void)
{
    helper_negative_setExtVmonCfg_mode(PMIC_PWR_EXT_VMON2);
}
void test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon2_vmonThr(void)
{
    helper_negative_setExtVmonCfg_vmonThr(PMIC_PWR_EXT_VMON2);
}
void test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon2_vmonDgl(void)
{
    helper_negative_setExtVmonCfg_vmonDgl(PMIC_PWR_EXT_VMON2);
}

/* ========================================================================== */
/*                  POSITIVE TESTS - Buck/Boost Configuration                 */
/* ========================================================================== */

void test_pos_power_setGetBuckBoostCfg_lvl(void)
{
    Pmic_PwrBuckBoostCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_BB_LVL_VALID,
        .lvl = PMIC_PWR_BB_LVL_5V
    };
    Pmic_PwrBuckBoostCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_BB_LVL_VALID
    };

    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.lvl == setCfg.lvl);
}

void test_pos_power_setGetBuckBoostCfg_stbyLvl(void)
{
    Pmic_PwrBuckBoostCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_BB_STBY_LVL_VALID,
        .stbyLvl = PMIC_PWR_BB_STBY_LVL_4V
    };
    Pmic_PwrBuckBoostCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_BB_STBY_LVL_VALID
    };

    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.stbyLvl == setCfg.stbyLvl);
}

void test_pos_power_setGetBuckBoostCfg_vmonThr(void)
{
    Pmic_PwrBuckBoostCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_BB_VMON_THR_VALID,
        .vmonThr = PMIC_PWR_BB_VMON_THR_6_PCT
    };
    Pmic_PwrBuckBoostCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_BB_VMON_THR_VALID
    };

    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vmonThr == setCfg.vmonThr);
}

void test_pos_power_setGetBuckBoostCfg_vmonDgl(void)
{
    Pmic_PwrBuckBoostCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_BB_VMON_DGL_VALID,
        .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_16_US
    };
    Pmic_PwrBuckBoostCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_BB_VMON_DGL_VALID
    };

    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vmonDgl == setCfg.vmonDgl);
}

void test_pos_power_setGetBuckBoostCfg_boostTmo(void)
{
    Pmic_PwrBuckBoostCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_BB_BOOST_TMO_VALID,
        .boostTmo = PMIC_PWR_BOOST_TMO_8_SEC
    };
    Pmic_PwrBuckBoostCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_BB_BOOST_TMO_VALID
    };

    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.boostTmo == setCfg.boostTmo);
}

void test_pos_power_setGetBuckBoostCfg_ssEn(void)
{
    Pmic_PwrBuckBoostCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_BB_SS_EN_VALID,
        .ssEn = true
    };
    Pmic_PwrBuckBoostCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_BB_SS_EN_VALID
    };

    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssEn == setCfg.ssEn);
}

void test_pos_power_setGetBuckBoostCfg_includeOvUvStatInPGood(void)
{
    Pmic_PwrBuckBoostCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID,
        .includeOvUvStatInPGood = true
    };
    Pmic_PwrBuckBoostCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID
    };

    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.includeOvUvStatInPGood == setCfg.includeOvUvStatInPGood);
}

void test_pos_power_setGetBuckBoostCfg_allCfg(void)
{
    Pmic_PwrBuckBoostCfg_t setCfg = {
        .validParams = PMIC_PWR_CFG_BB_ALL,
        .lvl = PMIC_PWR_BB_LVL_6V,
        .stbyLvl = PMIC_PWR_BB_STBY_LVL_4V,
        .vmonThr = PMIC_PWR_BB_VMON_THR_7_PCT,
        .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_24_US,
        .boostTmo = PMIC_PWR_BOOST_TMO_12_SEC,
        .ssEn = false,
        .includeOvUvStatInPGood = true
    };
    Pmic_PwrBuckBoostCfg_t getCfg = {
        .validParams = PMIC_PWR_CFG_BB_ALL
    };

    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.lvl == setCfg.lvl);
    PLATFORM_ASSERT(getCfg.stbyLvl == setCfg.stbyLvl);
    PLATFORM_ASSERT(getCfg.vmonThr == setCfg.vmonThr);
    PLATFORM_ASSERT(getCfg.vmonDgl == setCfg.vmonDgl);
    PLATFORM_ASSERT(getCfg.boostTmo == setCfg.boostTmo);
    PLATFORM_ASSERT(getCfg.ssEn == setCfg.ssEn);
    PLATFORM_ASSERT(getCfg.includeOvUvStatInPGood == setCfg.includeOvUvStatInPGood);
}

/* ========================================================================== */
/*                     POSITIVE TESTS - LDO Configuration                     */
/* ========================================================================== */

static void helper_setGetLdoCfg_mode(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_LDO_MODE_VALID,
        .ldo = ldo,
        .mode = PMIC_PWR_LDO_EN_AS_LDO_IN_OPER
    };
    Pmic_PwrLdoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_LDO_MODE_VALID,
        .ldo = ldo
    };

    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == setCfg.mode);

    setCfg.mode = PMIC_PWR_LDO_DISABLED;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void helper_setGetLdoCfg_lvl(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t defaultCfg = {
        .validParams = PMIC_CFG_PWR_LDO_LVL_VALID,
        .ldo = ldo
    };
    Pmic_PwrLdoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_LDO_LVL_VALID,
        .ldo = ldo
    };
    Pmic_PwrLdoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_LDO_LVL_VALID,
        .ldo = ldo
    };

    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, &defaultCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    setCfg.lvl = (defaultCfg.lvl == PMIC_PWR_LDO_LVL_1P8V) ? PMIC_PWR_LDO_LVL_1P75V : PMIC_PWR_LDO_LVL_1P8V;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.lvl == setCfg.lvl);
}

static void helper_setGetLdoCfg_ilimLvl(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_LDO_ILIM_LVL_VALID,
        .ldo = ldo,
        .ilimLvl = PMIC_PWR_LDO_ILIM_LVL_OPTION_1
    };
    Pmic_PwrLdoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_LDO_ILIM_LVL_VALID,
        .ldo = ldo
    };

    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ilimLvl == setCfg.ilimLvl);
}

static void helper_setGetLdoCfg_ilimDgl(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_LDO_ILIM_DGL_VALID,
        .ldo = ldo,
        .ilimDgl = PMIC_PWR_LDO_ILIM_DEGLITCH_1_MS
    };
    Pmic_PwrLdoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_LDO_ILIM_DGL_VALID,
        .ldo = ldo
    };

    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ilimDgl == setCfg.ilimDgl);
}

static void helper_setGetLdoCfg_vmonThr(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_THR_VALID,
        .ldo = ldo,
        .vmonThr = PMIC_PWR_LDO_VMON_THR_5_PCT
    };
    Pmic_PwrLdoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_THR_VALID,
        .ldo = ldo
    };

    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vmonThr == setCfg.vmonThr);
}

static void helper_setGetLdoCfg_vmonDgl(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_DGL_VALID,
        .ldo = ldo,
        .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_24_US
    };
    Pmic_PwrLdoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_DGL_VALID,
        .ldo = ldo
    };

    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vmonDgl == setCfg.vmonDgl);
}

static void helper_setGetLdoCfg_rampTime(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_LDO_RAMP_TIME_VALID,
        .ldo = ldo,
        .rampTime = PMIC_PWR_RT_LONGER
    };
    Pmic_PwrLdoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_LDO_RAMP_TIME_VALID,
        .ldo = ldo
    };

    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.rampTime == setCfg.rampTime);
}

static void helper_setGetLdoCfg_disableDischarge(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_LDO_DISABLE_DISCHARGE_VALID,
        .ldo = ldo,
        .disableDischarge = true
    };
    Pmic_PwrLdoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_LDO_DISABLE_DISCHARGE_VALID,
        .ldo = ldo
    };

    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.disableDischarge == setCfg.disableDischarge);
}

static void helper_setGetLdoCfg_includeOvUvStatInPGood(uint16_t ldo)
{
    Pmic_PwrLdoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID,
        .ldo = ldo,
        .includeOvUvStatInPGood = true
    };
    Pmic_PwrLdoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID,
        .ldo = ldo
    };

    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.includeOvUvStatInPGood == setCfg.includeOvUvStatInPGood);
}

/* LDO1 positive tests */
void test_pos_power_setGetLdoCfg_ldo1_mode(void) { helper_setGetLdoCfg_mode(PMIC_PWR_LDO1); }
void test_pos_power_setGetLdoCfg_ldo1_lvl(void)
{
#ifdef BUILD_MOCK
    helper_setGetLdoCfg_lvl(PMIC_PWR_LDO1);
#else
    TEST_IGNORE_MESSAGE("LDO1 voltage change triggers fault on hardware");
#endif
}
void test_pos_power_setGetLdoCfg_ldo1_ilimLvl(void) { helper_setGetLdoCfg_ilimLvl(PMIC_PWR_LDO1); }
void test_pos_power_setGetLdoCfg_ldo1_ilimDgl(void) { helper_setGetLdoCfg_ilimDgl(PMIC_PWR_LDO1); }
void test_pos_power_setGetLdoCfg_ldo1_vmonThr(void) { helper_setGetLdoCfg_vmonThr(PMIC_PWR_LDO1); }
void test_pos_power_setGetLdoCfg_ldo1_vmonDgl(void) { helper_setGetLdoCfg_vmonDgl(PMIC_PWR_LDO1); }
void test_pos_power_setGetLdoCfg_ldo1_rampTime(void) { helper_setGetLdoCfg_rampTime(PMIC_PWR_LDO1); }
void test_pos_power_setGetLdoCfg_ldo1_disableDischarge(void) { helper_setGetLdoCfg_disableDischarge(PMIC_PWR_LDO1); }
void test_pos_power_setGetLdoCfg_ldo1_includeOvUvStatInPGood(void) { helper_setGetLdoCfg_includeOvUvStatInPGood(PMIC_PWR_LDO1); }

/* LDO2 positive tests */
void test_pos_power_setGetLdoCfg_ldo2_mode(void) { helper_setGetLdoCfg_mode(PMIC_PWR_LDO2); }
void test_pos_power_setGetLdoCfg_ldo2_lvl(void) { helper_setGetLdoCfg_lvl(PMIC_PWR_LDO2); }
void test_pos_power_setGetLdoCfg_ldo2_ilimLvl(void) { helper_setGetLdoCfg_ilimLvl(PMIC_PWR_LDO2); }
void test_pos_power_setGetLdoCfg_ldo2_ilimDgl(void) { helper_setGetLdoCfg_ilimDgl(PMIC_PWR_LDO2); }
void test_pos_power_setGetLdoCfg_ldo2_vmonThr(void) { helper_setGetLdoCfg_vmonThr(PMIC_PWR_LDO2); }
void test_pos_power_setGetLdoCfg_ldo2_vmonDgl(void) { helper_setGetLdoCfg_vmonDgl(PMIC_PWR_LDO2); }
void test_pos_power_setGetLdoCfg_ldo2_rampTime(void) { helper_setGetLdoCfg_rampTime(PMIC_PWR_LDO2); }
void test_pos_power_setGetLdoCfg_ldo2_disableDischarge(void) { helper_setGetLdoCfg_disableDischarge(PMIC_PWR_LDO2); }
void test_pos_power_setGetLdoCfg_ldo2_includeOvUvStatInPGood(void) { helper_setGetLdoCfg_includeOvUvStatInPGood(PMIC_PWR_LDO2); }

/* LDO3 positive tests */
void test_pos_power_setGetLdoCfg_ldo3_mode(void) { helper_setGetLdoCfg_mode(PMIC_PWR_LDO3); }
void test_pos_power_setGetLdoCfg_ldo3_lvl(void) { helper_setGetLdoCfg_lvl(PMIC_PWR_LDO3); }
void test_pos_power_setGetLdoCfg_ldo3_ilimLvl(void) { helper_setGetLdoCfg_ilimLvl(PMIC_PWR_LDO3); }
void test_pos_power_setGetLdoCfg_ldo3_ilimDgl(void) { helper_setGetLdoCfg_ilimDgl(PMIC_PWR_LDO3); }
void test_pos_power_setGetLdoCfg_ldo3_vmonThr(void) { helper_setGetLdoCfg_vmonThr(PMIC_PWR_LDO3); }
void test_pos_power_setGetLdoCfg_ldo3_vmonDgl(void) { helper_setGetLdoCfg_vmonDgl(PMIC_PWR_LDO3); }
void test_pos_power_setGetLdoCfg_ldo3_rampTime(void) { helper_setGetLdoCfg_rampTime(PMIC_PWR_LDO3); }
void test_pos_power_setGetLdoCfg_ldo3_disableDischarge(void) { helper_setGetLdoCfg_disableDischarge(PMIC_PWR_LDO3); }
void test_pos_power_setGetLdoCfg_ldo3_includeOvUvStatInPGood(void) { helper_setGetLdoCfg_includeOvUvStatInPGood(PMIC_PWR_LDO3); }

/* LDO4 positive tests */
void test_pos_power_setGetLdoCfg_ldo4_mode(void) { helper_setGetLdoCfg_mode(PMIC_PWR_LDO4); }
void test_pos_power_setGetLdoCfg_ldo4_lvl(void)
{
#ifdef BUILD_MOCK
    helper_setGetLdoCfg_lvl(PMIC_PWR_LDO4);
#else
    TEST_IGNORE_MESSAGE("LDO4 voltage change triggers fault on hardware");
#endif
}
void test_pos_power_setGetLdoCfg_ldo4_ilimLvl(void) { helper_setGetLdoCfg_ilimLvl(PMIC_PWR_LDO4); }
void test_pos_power_setGetLdoCfg_ldo4_ilimDgl(void) { helper_setGetLdoCfg_ilimDgl(PMIC_PWR_LDO4); }
void test_pos_power_setGetLdoCfg_ldo4_vmonThr(void) { helper_setGetLdoCfg_vmonThr(PMIC_PWR_LDO4); }
void test_pos_power_setGetLdoCfg_ldo4_vmonDgl(void) { helper_setGetLdoCfg_vmonDgl(PMIC_PWR_LDO4); }
void test_pos_power_setGetLdoCfg_ldo4_rampTime(void) { helper_setGetLdoCfg_rampTime(PMIC_PWR_LDO4); }
void test_pos_power_setGetLdoCfg_ldo4_disableDischarge(void) { helper_setGetLdoCfg_disableDischarge(PMIC_PWR_LDO4); }
void test_pos_power_setGetLdoCfg_ldo4_includeOvUvStatInPGood(void) { helper_setGetLdoCfg_includeOvUvStatInPGood(PMIC_PWR_LDO4); }

/* ========================================================================== */
/*                    POSITIVE TESTS - PLDO Configuration                     */
/* ========================================================================== */

static void helper_setGetPldoCfg_mode(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_MODE_VALID,
        .pldo = pldo,
        .mode = PMIC_PWR_PLDO_EN_LDO_OPER
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_MODE_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == setCfg.mode);
}

static void helper_setGetPldoCfg_trackingMode(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_TRACKING_MODE_VALID,
        .pldo = pldo,
        .trackingMode = true
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_TRACKING_MODE_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.trackingMode == setCfg.trackingMode);
}

static void helper_setGetPldoCfg_lvl(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_LVL_VALID,
        .pldo = pldo,
        .lvl = PMIC_PWR_LDO_LVL_3P3V
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_LVL_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.lvl == setCfg.lvl);
}

static void helper_setGetPldoCfg_ilimLvl(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_ILIM_LVL_VALID,
        .pldo = pldo,
        .ilimLvl = PMIC_PWR_PLDO_ILIM_LVL_OPTION_2
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_ILIM_LVL_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ilimLvl == setCfg.ilimLvl);
}

static void helper_setGetPldoCfg_ilimDgl(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_ILIM_DGL_VALID,
        .pldo = pldo,
        .ilimDgl = PMIC_PWR_LDO_ILIM_DEGLITCH_1_MS
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_ILIM_DGL_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ilimDgl == setCfg.ilimDgl);
}

static void helper_setGetPldoCfg_vmonThr(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_VMON_THR_VALID,
        .pldo = pldo,
        .vmonThr = PMIC_PWR_PLDO_VMON_THR_8_PCT
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_VMON_THR_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vmonThr == setCfg.vmonThr);
}

static void helper_setGetPldoCfg_vmonDgl(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_VMON_DGL_VALID,
        .pldo = pldo,
        .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_32_US
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_VMON_DGL_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vmonDgl == setCfg.vmonDgl);
}

static void helper_setGetPldoCfg_vtrackRange(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_VTRACK_RANGE_VALID,
        .pldo = pldo,
        .vtrackRange = PMIC_PWR_VTRACK_GTE_2P2V
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_VTRACK_RANGE_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vtrackRange == setCfg.vtrackRange);
}

static void helper_setGetPldoCfg_rampTime(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_RT_VALID,
        .pldo = pldo,
        .rampTime = PMIC_PWR_RT_SHORTER
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_RT_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.rampTime == setCfg.rampTime);
}

static void helper_setGetPldoCfg_disableDischarge(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_DISABLE_DISCHARGE_VALID,
        .pldo = pldo,
        .disableDischarge = true
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_DISABLE_DISCHARGE_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.disableDischarge == setCfg.disableDischarge);
}

static void helper_setGetPldoCfg_includeOvUvStatInPGood(uint16_t pldo)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID,
        .pldo = pldo,
        .includeOvUvStatInPGood = false
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID,
        .pldo = pldo
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.includeOvUvStatInPGood == setCfg.includeOvUvStatInPGood);
}

/* PLDO1 positive tests */
void test_pos_power_setGetPldoCfg_pldo1_mode(void) { helper_setGetPldoCfg_mode(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_trackingMode(void) { helper_setGetPldoCfg_trackingMode(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_lvl(void) { helper_setGetPldoCfg_lvl(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_ilimLvl(void) { helper_setGetPldoCfg_ilimLvl(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_ilimDgl(void) { helper_setGetPldoCfg_ilimDgl(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_vmonThr(void) { helper_setGetPldoCfg_vmonThr(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_vmonDgl(void) { helper_setGetPldoCfg_vmonDgl(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_vtrackRange(void) { helper_setGetPldoCfg_vtrackRange(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_rampTime(void) { helper_setGetPldoCfg_rampTime(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_disableDischarge(void) { helper_setGetPldoCfg_disableDischarge(PMIC_PWR_PLDO1); }
void test_pos_power_setGetPldoCfg_pldo1_includeOvUvStatInPGood(void) { helper_setGetPldoCfg_includeOvUvStatInPGood(PMIC_PWR_PLDO1); }

/* PLDO2 positive tests */
void test_pos_power_setGetPldoCfg_pldo2_mode(void) { helper_setGetPldoCfg_mode(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_trackingMode(void) { helper_setGetPldoCfg_trackingMode(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_lvl(void) { helper_setGetPldoCfg_lvl(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_ilimLvl(void) { helper_setGetPldoCfg_ilimLvl(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_ilimDgl(void) { helper_setGetPldoCfg_ilimDgl(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_vmonThr(void) { helper_setGetPldoCfg_vmonThr(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_vmonDgl(void) { helper_setGetPldoCfg_vmonDgl(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_vtrackRange(void) { helper_setGetPldoCfg_vtrackRange(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_rampTime(void) { helper_setGetPldoCfg_rampTime(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_disableDischarge(void) { helper_setGetPldoCfg_disableDischarge(PMIC_PWR_PLDO2); }
void test_pos_power_setGetPldoCfg_pldo2_includeOvUvStatInPGood(void) { helper_setGetPldoCfg_includeOvUvStatInPGood(PMIC_PWR_PLDO2); }

/* ========================================================================== */
/*                  POSITIVE TESTS - ExtVmon Configuration                    */
/* ========================================================================== */

static void helper_setGetExtVmonCfg_mode(uint16_t extVmon)
{
    Pmic_PwrExtVmonCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_MODE_VALID,
        .extVmon = extVmon,
        .mode = PMIC_PWR_EXT_VMON_EN_IN_OPER
    };
    Pmic_PwrExtVmonCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_MODE_VALID,
        .extVmon = extVmon
    };

    int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetExtVmonCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == setCfg.mode);
}

static void helper_setGetExtVmonCfg_vmonThr(uint16_t extVmon)
{
    Pmic_PwrExtVmonCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_THR_VALID,
        .extVmon = extVmon,
        .vmonThr = PMIC_PWR_EXT_VMON_THR_6_PCT_HYSTERESIS
    };
    Pmic_PwrExtVmonCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_THR_VALID,
        .extVmon = extVmon
    };

    int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetExtVmonCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vmonThr == setCfg.vmonThr);
}

static void helper_setGetExtVmonCfg_vmonDgl(uint16_t extVmon)
{
    Pmic_PwrExtVmonCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_DGL_VALID,
        .extVmon = extVmon,
        .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_16_US
    };
    Pmic_PwrExtVmonCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_DGL_VALID,
        .extVmon = extVmon
    };

    int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetExtVmonCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vmonDgl == setCfg.vmonDgl);
}

static void helper_setGetExtVmonCfg_includeOvUvStatInPGood(uint16_t extVmon)
{
    Pmic_PwrExtVmonCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID,
        .extVmon = extVmon,
        .includeOvUvStatInPGood = true
    };
    Pmic_PwrExtVmonCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID,
        .extVmon = extVmon
    };

    int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetExtVmonCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.includeOvUvStatInPGood == setCfg.includeOvUvStatInPGood);
}

/* ExtVmon1 positive tests */
void test_pos_power_setGetExtVmonCfg_vmon1_mode(void)
{
    helper_setGetExtVmonCfg_mode(PMIC_PWR_EXT_VMON1);
}
void test_pos_power_setGetExtVmonCfg_vmon1_vmonThr(void)
{
    helper_setGetExtVmonCfg_vmonThr(PMIC_PWR_EXT_VMON1);
}
void test_pos_power_setGetExtVmonCfg_vmon1_vmonDgl(void)
{
    helper_setGetExtVmonCfg_vmonDgl(PMIC_PWR_EXT_VMON1);
}
void test_pos_power_setGetExtVmonCfg_vmon1_includeOvUvStatInPGood(void)
{
    helper_setGetExtVmonCfg_includeOvUvStatInPGood(PMIC_PWR_EXT_VMON1);
}

/* ExtVmon2 positive tests */
void test_pos_power_setGetExtVmonCfg_vmon2_mode(void)
{
    helper_setGetExtVmonCfg_mode(PMIC_PWR_EXT_VMON2);
}
void test_pos_power_setGetExtVmonCfg_vmon2_vmonThr(void)
{
    helper_setGetExtVmonCfg_vmonThr(PMIC_PWR_EXT_VMON2);
}
void test_pos_power_setGetExtVmonCfg_vmon2_vmonDgl(void)
{
    helper_setGetExtVmonCfg_vmonDgl(PMIC_PWR_EXT_VMON2);
}
void test_pos_power_setGetExtVmonCfg_vmon2_includeOvUvStatInPGood(void)
{
    helper_setGetExtVmonCfg_includeOvUvStatInPGood(PMIC_PWR_EXT_VMON2);
}

/* ========================================================================== */
/*                  POSITIVE TESTS - Resource Status                          */
/* ========================================================================== */

static void helper_getRsrcStatus(uint16_t pwrRsrc)
{
    Pmic_PwrRsrcStatus_t stat = {
        .pwrRsrc = pwrRsrc
    };

    // Determine valid params based on resource type
    if (pwrRsrc == PMIC_PWR_BUCK_BOOST) {
        stat.validParams = PMIC_PWR_RSRC_STAT_BB_ALL;
    } else if ((pwrRsrc >= PMIC_PWR_LDO_MIN) && (pwrRsrc <= PMIC_PWR_LDO_MAX)) {
        stat.validParams = PMIC_PWR_RSRC_STAT_LDO_ALL;
    } else if ((pwrRsrc >= PMIC_PWR_PLDO_MIN) && (pwrRsrc <= PMIC_PWR_PLDO_MAX)) {
        stat.validParams = PMIC_PWR_RSRC_STAT_PLDO_ALL;
    } else if ((pwrRsrc >= PMIC_PWR_EXT_VMON_MIN) && (pwrRsrc <= PMIC_PWR_EXT_VMON_MAX)) {
        stat.validParams = PMIC_PWR_RSRC_STAT_EXT_VMON_ALL;
    }

    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buckBoost(void) { helper_getRsrcStatus(PMIC_PWR_BUCK_BOOST); }
void test_pos_power_getRsrcStatus_ldo1(void) { helper_getRsrcStatus(PMIC_PWR_LDO1); }
void test_pos_power_getRsrcStatus_ldo2(void) { helper_getRsrcStatus(PMIC_PWR_LDO2); }
void test_pos_power_getRsrcStatus_ldo3(void) { helper_getRsrcStatus(PMIC_PWR_LDO3); }
void test_pos_power_getRsrcStatus_ldo4(void) { helper_getRsrcStatus(PMIC_PWR_LDO4); }
void test_pos_power_getRsrcStatus_pldo1(void) { helper_getRsrcStatus(PMIC_PWR_PLDO1); }
void test_pos_power_getRsrcStatus_pldo2(void) { helper_getRsrcStatus(PMIC_PWR_PLDO2); }
void test_pos_power_getRsrcStatus_extVmon1(void)
{
    helper_getRsrcStatus(PMIC_PWR_EXT_VMON1);
}
void test_pos_power_getRsrcStatus_extVmon2(void)
{
    helper_getRsrcStatus(PMIC_PWR_EXT_VMON2);
}

void test_pos_power_clrRsrcStatus_buckBoost(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID | PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };

    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatusAll(void)
{
    int32_t status = Pmic_pwrClrRsrcStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                       POSITIVE TESTS - PGOOD                               */
/* ========================================================================== */

void test_pos_power_setGetPGoodInStby(void)
{
    bool setEnable = true;
    bool getEnable = false;

    int32_t status = Pmic_pwrSetPGoodInStby(&pmicHandle, setEnable);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPGoodInStby(&pmicHandle, &getEnable);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getEnable == setEnable);
}

/* ========================================================================== */
/*                  POSITIVE TESTS - Comprehensive                            */
/* ========================================================================== */

void test_pos_power_setGetLdoCfg_allLdos_allCfg(void)
{
    const uint16_t ldos[] = {PMIC_PWR_LDO2, PMIC_PWR_LDO3};
    const uint8_t numLdos = sizeof(ldos) / sizeof(ldos[0]);

    for (uint8_t i = 0U; i < numLdos; i++)
    {
        Pmic_PwrLdoCfg_t setCfg = {
            .validParams = PMIC_PWR_CFG_LDO_ALL,
            .ldo = ldos[i],
            .mode = PMIC_PWR_LDO_EN_AS_LDO_IN_OPER,
            .lvl = PMIC_PWR_LDO_LVL_1P75V,
            .ilimLvl = PMIC_PWR_LDO_ILIM_LVL_OPTION_1,
            .ilimDgl = PMIC_PWR_LDO_ILIM_DEGLITCH_1_MS,
            .vmonThr = PMIC_PWR_LDO_VMON_THR_5_PCT,
            .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_24_US,
            .rampTime = PMIC_PWR_RT_SHORTER,
            .disableDischarge = true,
            .includeOvUvStatInPGood = true
        };
        Pmic_PwrLdoCfg_t getCfg = {
            .validParams = PMIC_PWR_CFG_LDO_ALL,
            .ldo = ldos[i]
        };

        Pmic_PwrLdoCfg_t disableCfg = {
            .validParams = PMIC_CFG_PWR_LDO_MODE_VALID,
            .ldo = ldos[i],
            .mode = PMIC_PWR_LDO_DISABLED
        };
        int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &disableCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        platform_timerWaitMs(10U);

        status = Pmic_pwrSetLdoCfg(&pmicHandle, &setCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetLdoCfg(&pmicHandle, &getCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        PLATFORM_ASSERT(getCfg.mode == setCfg.mode);
        PLATFORM_ASSERT(getCfg.lvl == setCfg.lvl);
        PLATFORM_ASSERT(getCfg.ilimLvl == setCfg.ilimLvl);
        PLATFORM_ASSERT(getCfg.ilimDgl == setCfg.ilimDgl);
        PLATFORM_ASSERT(getCfg.vmonThr == setCfg.vmonThr);
        PLATFORM_ASSERT(getCfg.vmonDgl == setCfg.vmonDgl);
        PLATFORM_ASSERT(getCfg.rampTime == setCfg.rampTime);
        PLATFORM_ASSERT(getCfg.disableDischarge == setCfg.disableDischarge);
        PLATFORM_ASSERT(getCfg.includeOvUvStatInPGood == setCfg.includeOvUvStatInPGood);
    }
}

void test_pos_power_setGetPldoCfg_allPldos_allCfg(void)
{
    const uint16_t pldos[] = {PMIC_PWR_PLDO1, PMIC_PWR_PLDO2};
    const uint8_t numPldos = sizeof(pldos) / sizeof(pldos[0]);

    for (uint8_t i = 0U; i < numPldos; i++)
    {
        Pmic_PwrPldoCfg_t setCfg = {
            .validParams = PMIC_PWR_CFG_PLDO_ALL,
            .pldo = pldos[i],
            .mode = PMIC_PWR_PLDO_EN_LDO_OPER,
            .trackingMode = false,
            .lvl = PMIC_PWR_LDO_LVL_3P3V,
            .ilimLvl = PMIC_PWR_PLDO_ILIM_LVL_OPTION_1,
            .ilimDgl = PMIC_PWR_LDO_ILIM_DEGLITCH_1_MS,
            .vmonThr = PMIC_PWR_PLDO_VMON_THR_6_PCT,
            .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_24_US,
            .vtrackRange = PMIC_PWR_VTRACK_LT_2P2V,
            .rampTime = PMIC_PWR_RT_LONGER,
            .disableDischarge = true,
            .includeOvUvStatInPGood = false
        };
        Pmic_PwrPldoCfg_t getCfg = {
            .validParams = PMIC_PWR_CFG_PLDO_ALL,
            .pldo = pldos[i]
        };

        int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        PLATFORM_ASSERT(getCfg.mode == setCfg.mode);
        PLATFORM_ASSERT(getCfg.trackingMode == setCfg.trackingMode);
        PLATFORM_ASSERT(getCfg.lvl == setCfg.lvl);
        PLATFORM_ASSERT(getCfg.ilimLvl == setCfg.ilimLvl);
        PLATFORM_ASSERT(getCfg.ilimDgl == setCfg.ilimDgl);
        PLATFORM_ASSERT(getCfg.vmonThr == setCfg.vmonThr);
        PLATFORM_ASSERT(getCfg.vmonDgl == setCfg.vmonDgl);
        PLATFORM_ASSERT(getCfg.vtrackRange == setCfg.vtrackRange);
        PLATFORM_ASSERT(getCfg.rampTime == setCfg.rampTime);
        PLATFORM_ASSERT(getCfg.disableDischarge == setCfg.disableDischarge);
        PLATFORM_ASSERT(getCfg.includeOvUvStatInPGood == setCfg.includeOvUvStatInPGood);
    }
}

void test_pos_power_setGetExtVmonCfg_allVmons_allCfg(void)
{
#ifndef BUILD_HOST
    const uint16_t vmons[] = {PMIC_PWR_EXT_VMON1, PMIC_PWR_EXT_VMON2};
    const uint8_t numVmons = sizeof(vmons) / sizeof(vmons[0]);

    for (uint8_t i = 0U; i < numVmons; i++)
    {
        Pmic_PwrExtVmonCfg_t setCfg = {
            .validParams = PMIC_PWR_CFG_EXT_VMON_ALL,
            .extVmon = vmons[i],
            .mode = PMIC_PWR_EXT_VMON_EN_IN_OPER_AND_STBY,
            .vmonThr = PMIC_PWR_EXT_VMON_THR_8_PCT_HYSTERESIS,
            .vmonDgl = PMIC_PWR_RSRC_VMON_DGL_32_US,
            .includeOvUvStatInPGood = true
        };
        Pmic_PwrExtVmonCfg_t getCfg = {
            .validParams = PMIC_PWR_CFG_EXT_VMON_ALL,
            .extVmon = vmons[i]
        };

        int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &setCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetExtVmonCfg(&pmicHandle, &getCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        PLATFORM_ASSERT(getCfg.mode == setCfg.mode);
        PLATFORM_ASSERT(getCfg.vmonThr == setCfg.vmonThr);
        PLATFORM_ASSERT(getCfg.vmonDgl == setCfg.vmonDgl);
        PLATFORM_ASSERT(getCfg.includeOvUvStatInPGood == setCfg.includeOvUvStatInPGood);
    }
#else
    TEST_IGNORE_MESSAGE("EXT_VMON mode test enables VMON on unconnected pin -- triggers ACTIVE->SAFE within test before VMON2 readback");
#endif
}

/* ========================================================================== */
/*                  Zero validParams Tests                                    */
/* ========================================================================== */

void test_neg_power_pwrSetBuckBoostCfg_zeroValidParams(void)
{
    Pmic_PwrBuckBoostCfg_t cfg = {
        .validParams = 0U
    };
    int32_t status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetBuckBoostCfg_zeroValidParams(void)
{
    Pmic_PwrBuckBoostCfg_t cfg = {
        .validParams = 0U
    };
    int32_t status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                  Invalid Resource ID Tests                                 */
/* ========================================================================== */

void test_neg_power_pwrSetLdoCfg_invalidLdoId(void)
{
    Pmic_PwrLdoCfg_t cfg = {
        .ldo = TEST_INVALID_PARAM_255,
        .validParams = PMIC_CFG_PWR_LDO_MODE_VALID
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetLdoCfg_invalidLdoId(void)
{
    Pmic_PwrLdoCfg_t cfg = {
        .ldo = TEST_INVALID_PARAM_255,
        .validParams = PMIC_CFG_PWR_LDO_MODE_VALID
    };
    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetPldoCfg_invalidPldoId(void)
{
    Pmic_PwrPldoCfg_t cfg = {
        .pldo = TEST_INVALID_PARAM_255,
        .validParams = PMIC_CFG_PWR_PLDO_MODE_VALID
    };
    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetPldoCfg_invalidPldoId(void)
{
    Pmic_PwrPldoCfg_t cfg = {
        .pldo = TEST_INVALID_PARAM_255,
        .validParams = PMIC_CFG_PWR_PLDO_MODE_VALID
    };
    int32_t status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetExtVmonCfg_invalidExtVmonId(void)
{
    Pmic_PwrExtVmonCfg_t cfg = {
        .extVmon = TEST_INVALID_PARAM_255,
        .validParams = PMIC_CFG_PWR_EXT_VMON_MODE_VALID
    };
    int32_t status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetExtVmonCfg_invalidExtVmonId(void)
{
    Pmic_PwrExtVmonCfg_t cfg = {
        .extVmon = TEST_INVALID_PARAM_255,
        .validParams = PMIC_CFG_PWR_EXT_VMON_MODE_VALID
    };
    int32_t status = Pmic_pwrGetExtVmonCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*              BuckBoost Status GET Tests                                    */
/* ========================================================================== */

void test_pos_power_getRsrcStatus_buckBoost_bbLite(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buckBoost_bbIlimLvl(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buckBoost_bbMode(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_MODE_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buckBoost_ovErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buckBoost_uvErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buckBoost_tsdErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buckBoost_tsdWarn(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_WARN_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*              LDO Status GET Tests                                          */
/* ========================================================================== */

void test_neg_power_getRsrcStatus_ldo_unsupportedBbLite(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID,
        .pwrRsrc = PMIC_PWR_LDO1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_getRsrcStatus_ldo_unsupportedBbIlimLvl(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID,
        .pwrRsrc = PMIC_PWR_LDO1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_getRsrcStatus_ldo_unsupportedBbMode(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_MODE_VALID,
        .pwrRsrc = PMIC_PWR_LDO1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_pos_power_getRsrcStatus_ldo_uvErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_LDO1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_ldo_ovErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_LDO2
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_ldo_tsdErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_LDO3
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_ldo_tsdWarn(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_WARN_VALID,
        .pwrRsrc = PMIC_PWR_LDO4
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*              PLDO Status GET Tests                                         */
/* ========================================================================== */

void test_neg_power_getRsrcStatus_pldo_unsupportedBbLite(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_getRsrcStatus_pldo_unsupportedBbMode(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_MODE_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_pos_power_getRsrcStatus_pldo_uvErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_pldo_ovErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_PLDO2
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_pldo_tsdErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_pldo_tsdWarn(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_WARN_VALID,
        .pwrRsrc = PMIC_PWR_PLDO2
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*              ExtVmon Status GET Tests                                      */
/* ========================================================================== */

void test_neg_power_getRsrcStatus_extVmon_unsupportedIlimErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_getRsrcStatus_extVmon_unsupportedTsdErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_pos_power_getRsrcStatus_extVmon_uvErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_extVmon_ovErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON2
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_power_getRsrcStatus_extVmon_unsupportedTsdWarn(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_WARN_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_getRsrcStatus_extVmon_unsupportedBbMode(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_MODE_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_pos_power_getRsrcStatus_extVmon_noValidParams(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = 0U,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Neither UV_ERR nor OV_ERR requested -- EXT_VMON_STAT_REG read is skipped.
    // Failing any I/O here proves the skip if the call still succeeds.
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
/*              BuckBoost Status CLEAR Tests                                  */
/* ========================================================================== */

void test_pos_power_clrRsrcStatus_buckBoost_bbMode(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_MODE_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_buckBoost_ilimErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_buckBoost_tsdErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_buckBoost_tsdWarn(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_WARN_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*              LDO Status CLEAR Tests                                        */
/* ========================================================================== */

void test_pos_power_clrRsrcStatus_ldo1_uvErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_LDO1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_ldo2_ovErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_LDO2
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_ldo3_tsdErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_LDO3
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_ldo4_tsdWarn(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_WARN_VALID,
        .pwrRsrc = PMIC_PWR_LDO4
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_ldo_allStatus(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_LDO_ALL,
        .pwrRsrc = PMIC_PWR_LDO1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*              PLDO Status CLEAR Tests                                       */
/* ========================================================================== */

void test_pos_power_clrRsrcStatus_pldo1_uvErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_pldo2_ovErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_PLDO2
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_pldo_tsdErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_pldo_allStatus(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_PLDO_ALL,
        .pwrRsrc = PMIC_PWR_PLDO2
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*              ExtVmon Status CLEAR Tests                                    */
/* ========================================================================== */

void test_pos_power_clrRsrcStatus_extVmon1_uvErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_extVmon2_ovErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON2
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_power_clrRsrcStatus_extVmon_unsupportedTsdWarn(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_WARN_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_pos_power_clrRsrcStatus_extVmon_allStatus(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_EXT_VMON_ALL,
        .pwrRsrc = PMIC_PWR_EXT_VMON2
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*              Resource Status Invalid Type Tests                            */
/* ========================================================================== */

void test_neg_power_pwrGetRsrcStatus_invalidResourceType(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = 0xFFFFU  // Invalid resource type
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrClrRsrcStatus_invalidResourceType(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = 0xFFFFU  // Invalid resource type
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* Tests for malformed resource IDs - type bits don't match actual resource */
void test_neg_power_pwrGetRsrcStatus_malformedBbResource(void)
{
    // Resource with BuckBoost type bits (0x00) but invalid ID
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID,
        .pwrRsrc = 0x00FFU  // Type 0 (BuckBoost) but ID doesn't match PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetRsrcStatus_malformedLdoResource(void)
{
    // Resource with LDO type bits (0x01) but ID outside LDO range
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = 0x01FFU  // Type 1 (LDO) but ID not in valid LDO range
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetRsrcStatus_malformedPldoResource(void)
{
    // Resource with PLDO type bits (0x02) but ID outside PLDO range
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = 0x02FFU  // Type 2 (PLDO) but ID not in valid PLDO range
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetRsrcStatus_malformedExtVmonResource(void)
{
    // Resource with ExtVmon type bits (0x03) but ID outside ExtVmon range
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = 0x03FFU  // Type 3 (ExtVmon) but ID not in valid ExtVmon range
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrClrRsrcStatus_malformedBbResource(void)
{
    // Resource with BuckBoost type bits (0x00) but invalid ID
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_MODE_VALID,
        .pwrRsrc = 0x00FFU  // Type 0 (BuckBoost) but ID doesn't match PMIC_PWR_BUCK_BOOST
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrClrRsrcStatus_malformedLdoResource(void)
{
    // Resource with LDO type bits (0x01) but ID outside LDO range
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = 0x01FFU  // Type 1 (LDO) but ID not in valid LDO range
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrClrRsrcStatus_malformedPldoResource(void)
{
    // Resource with PLDO type bits (0x02) but ID outside PLDO range
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = 0x02FFU  // Type 2 (PLDO) but ID not in valid PLDO range
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrClrRsrcStatus_malformedExtVmonResource(void)
{
    // Resource with ExtVmon type bits (0x03) but ID outside ExtVmon range
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = 0x03FFU  // Type 3 (ExtVmon) but ID not in valid ExtVmon range
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*              Edge Case Tests                                               */
/* ========================================================================== */

void test_pos_power_pwrGetPldoCfg_redundantModeConversion(void)
{
    // Test that getting PLDO config twice in a row works correctly
    Pmic_PwrPldoCfg_t cfg1 = {
        .validParams = PMIC_CFG_PWR_PLDO_MODE_VALID,
        .pldo = PMIC_PWR_PLDO1
    };
    Pmic_PwrPldoCfg_t cfg2 = {
        .validParams = PMIC_CFG_PWR_PLDO_MODE_VALID,
        .pldo = PMIC_PWR_PLDO1
    };

    int32_t status1 = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg1);
    PLATFORM_ASSERT(status1 == PMIC_ST_SUCCESS);

    int32_t status2 = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg2);
    PLATFORM_ASSERT(status2 == PMIC_ST_SUCCESS);

    // Both reads should return the same mode
    PLATFORM_ASSERT(cfg2.mode == cfg1.mode);
}

/* ========================================================================== */
/*                      Coverage Tests for pmic_power.c                       */
/* ========================================================================== */

/**
 * @brief Test PLDO mode fallback to DISABLED when register value > MAX.
 */
#ifdef BUILD_MOCK
void test_pos_power_pwr_getPldoMode_disabledFallback(void)
{
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    // Inject invalid mode value > MAX (PLDO2 mode is 3 bits, so values 5-7 are invalid)
    // PLDO_EN_OUT_CTRL_REG (0x27) PLDO2 mode field is bits 4:2
    // Inject value with PLDO2 mode = 5 (101 in bits 4:2 = 0x14)
    testInject_setRegister(0x27U, 0x14U);

    cfg.validParams = PMIC_CFG_PWR_PLDO_MODE_VALID;
    cfg.pldo = PMIC_PWR_PLDO2;

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfg.mode == 0U);  // Should fallback to 0 (disabled)

    // Test with value 7 (111 in bits 4:2 = 0x1C)
    testInject_setRegister(0x27U, 0x1CU);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfg.mode == 0U);  // Should fallback to 0 (disabled)
}
#else
void test_pos_power_pwr_getPldoMode_disabledFallback(void)
{
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
}
#endif

/**
 * @brief Test clearing LDO status with BB-specific validParams.
 */
void test_neg_power_pwr_clrLdoStat_unsupportedBbParams(void)
{
    Pmic_PwrRsrcStatus_t status_cfg = {0};

    // Set BB-specific validParam for LDO resource (not supported)
    status_cfg.validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID;
    status_cfg.pwrRsrc = PMIC_PWR_LDO1;

    int32_t result = Pmic_pwrClrRsrcStatus(&pmicHandle, &status_cfg);
    PLATFORM_ASSERT(result == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test clearing PLDO status with BB-specific validParams.
 */
void test_neg_power_pwr_clrPldoStat_unsupportedBbParams(void)
{
    Pmic_PwrRsrcStatus_t status_cfg = {0};

    // Set BB-specific validParam for PLDO resource (not supported)
    status_cfg.validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID;
    status_cfg.pwrRsrc = PMIC_PWR_PLDO1;

    int32_t result = Pmic_pwrClrRsrcStatus(&pmicHandle, &status_cfg);
    PLATFORM_ASSERT(result == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwr_clrLdoStat_unsupportedBbIlimLvlParams(void)
{
    Pmic_PwrRsrcStatus_t status_cfg = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID,
        .pwrRsrc = PMIC_PWR_LDO1
    };
    int32_t result = Pmic_pwrClrRsrcStatus(&pmicHandle, &status_cfg);
    PLATFORM_ASSERT(result == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwr_clrLdoStat_unsupportedBbModeParams(void)
{
    Pmic_PwrRsrcStatus_t status_cfg = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_MODE_VALID,
        .pwrRsrc = PMIC_PWR_LDO1
    };
    int32_t result = Pmic_pwrClrRsrcStatus(&pmicHandle, &status_cfg);
    PLATFORM_ASSERT(result == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwr_clrPldoStat_unsupportedBbIlimLvlParams(void)
{
    Pmic_PwrRsrcStatus_t status_cfg = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t result = Pmic_pwrClrRsrcStatus(&pmicHandle, &status_cfg);
    PLATFORM_ASSERT(result == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwr_clrPldoStat_unsupportedBbModeParams(void)
{
    Pmic_PwrRsrcStatus_t status_cfg = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_MODE_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t result = Pmic_pwrClrRsrcStatus(&pmicHandle, &status_cfg);
    PLATFORM_ASSERT(result == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_pos_power_clrRsrcStatus_pldo1_ovErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_clrRsrcStatus_pldo1_tsdWarn(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_WARN_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test clearing ExtVmon status with ILIM_ERR validParam (unsupported).
 */
void test_neg_power_pwr_clrExtVmonStat_ilimErrUnsupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test clearing ExtVmon status with BB_LITE validParam (unsupported).
 */
void test_neg_power_pwr_clrExtVmonStat_bbLiteUnsupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test clearing ExtVmon status with BB_ILIM_LVL validParam (unsupported).
 */
void test_neg_power_pwr_clrExtVmonStat_bbIlimLvlUnsupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test clearing ExtVmon status with BB_MODE validParam (unsupported).
 */
void test_neg_power_pwr_clrExtVmonStat_bbModeUnsupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_MODE_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test clearing ExtVmon status with TSD_ERR validParam (unsupported).
 */
void test_neg_power_pwr_clrExtVmonStat_tsdErrUnsupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/* ========================================================================== */
// Coverage Gap: I/O failure paths in BB getCfg sub-functions
/* ========================================================================== */

/**
 * @brief Test PWR_getBbSsEnLvlPGoodCfg() read failure path.
 */
void test_neg_power_pwrGetBuckBoostCfg_ssEnLvlPGoodReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrBuckBoostCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_BB_SS_EN_VALID;

    // I/O #1: Pmic_ioRxByte_CS(BUCK_BST_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getBbVmonThrBstTmo() read failure path.
 */
void test_neg_power_pwrGetBuckBoostCfg_vmonThrBstTmoReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrBuckBoostCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_BB_VMON_THR_VALID;

    // I/O #1: Pmic_ioRxByte_CS(VMON_TH_CFG3_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getBbVmonDgl() read failure path.
 */
void test_neg_power_pwrGetBuckBoostCfg_vmonDglReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrBuckBoostCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_BB_VMON_DGL_VALID;

    // I/O #1: Pmic_ioRxByte_CS(VMON_DGL_CFG1_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetBuckBoostCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: I/O failure paths in LDO getCfg sub-functions
/* ========================================================================== */

/**
 * @brief Test PWR_getLdoMode() read failure path.
 */
void test_neg_power_pwrGetLdoCfg_modeReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.ldo = PMIC_PWR_LDO1;
    cfg.validParams = PMIC_CFG_PWR_LDO_MODE_VALID;

    // I/O #1: LDO mode register read - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getLdoVmonThr() read failure path.
 */
void test_neg_power_pwrGetLdoCfg_vmonThrReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.ldo = PMIC_PWR_LDO1;
    cfg.validParams = PMIC_CFG_PWR_LDO_VMON_THR_VALID;

    // I/O #1: LDO VMON threshold register read - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: I/O failure paths in PLDO getCfg sub-functions
/* ========================================================================== */

/**
 * @brief Test PWR_getPldoMode() read failure path.
 */
void test_neg_power_pwrGetPldoCfg_modeReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.validParams = PMIC_CFG_PWR_PLDO_MODE_VALID;

    // I/O #1: PLDO mode register read - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getPldoVmonThr() read failure path.
 */
void test_neg_power_pwrGetPldoCfg_vmonThrReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.validParams = PMIC_CFG_PWR_PLDO_VMON_THR_VALID;

    // I/O #1: PLDO VMON threshold register read - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: I/O failure paths in ExtVmon getCfg sub-functions
/* ========================================================================== */

/**
 * @brief Test PWR_getExtVmonModePGoodCfg() read failure path.
 */
void test_neg_power_pwrGetExtVmonCfg_modeReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrExtVmonCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.extVmon = PMIC_PWR_EXT_VMON1;
    cfg.validParams = PMIC_CFG_PWR_EXT_VMON_MODE_VALID;

    // I/O #1: Pmic_ioRxByte_CS(EXT_VMON_CFG_CTRL_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetExtVmonCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getExtVmonThr() read failure path.
 */
void test_neg_power_pwrGetExtVmonCfg_vmonThrReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrExtVmonCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.extVmon = PMIC_PWR_EXT_VMON1;
    cfg.validParams = PMIC_CFG_PWR_EXT_VMON_THR_VALID;

    // I/O #1: Pmic_ioRxByte_CS(VMON_TH_CFGx_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetExtVmonCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getExtVmonDgl() read failure path.
 */
void test_neg_power_pwrGetExtVmonCfg_vmonDglReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrExtVmonCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.extVmon = PMIC_PWR_EXT_VMON1;
    cfg.validParams = PMIC_CFG_PWR_EXT_VMON_DGL_VALID;

    // I/O #1: Pmic_ioRxByte_CS(VMON_DGL_CFG3_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetExtVmonCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: I/O failure paths in LDO getCfg sub-functions (cont.)
/* ========================================================================== */

/**
 * @brief Test PWR_getLdoRtLvlIlimLvl() read failure path.
 */
void test_neg_power_pwrGetLdoCfg_lvlIlimLvlReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.ldo = PMIC_PWR_LDO1;
    cfg.validParams = PMIC_CFG_PWR_LDO_LVL_VALID;

    // I/O #1: Pmic_ioRxByte_CS(LDOx_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getLdoPGoodCfg() read failure path.
 */
void test_neg_power_pwrGetLdoCfg_pgoodCfgReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.ldo = PMIC_PWR_LDO1;
    cfg.validParams = PMIC_CFG_PWR_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID;

    // I/O #1: Pmic_ioRxByte_CS(LDO_PGOOD_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getLdoVmonDgl() read failure path.
 */
void test_neg_power_pwrGetLdoCfg_vmonDglReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.ldo = PMIC_PWR_LDO1;
    cfg.validParams = PMIC_CFG_PWR_LDO_VMON_DGL_VALID;

    // I/O #1: Pmic_ioRxByte_CS(VMON_DGL_CFG2_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getLdoDischargeDisable() read failure path.
 */
void test_neg_power_pwrGetLdoCfg_dischargeDisableReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.ldo = PMIC_PWR_LDO1;
    cfg.validParams = PMIC_CFG_PWR_LDO_DISABLE_DISCHARGE_VALID;

    // I/O #1: Pmic_ioRxByte_CS(LDO_DSCG_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getLdoIlimDgl() read failure path.
 */
void test_neg_power_pwrGetLdoCfg_ilimDglReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.ldo = PMIC_PWR_LDO1;
    cfg.validParams = PMIC_CFG_PWR_LDO_ILIM_DGL_VALID;

    // I/O #1: Pmic_ioRxByte_CS(ILIM_DGL_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: I/O failure paths in PLDO getCfg sub-functions (cont.)
/* ========================================================================== */

/**
 * @brief Test PWR_getPldoTrackingModeLvlIlimLvl() read failure path.
 */
void test_neg_power_pwrGetPldoCfg_trackingLvlIlimReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.validParams = PMIC_CFG_PWR_PLDO_LVL_VALID;

    // I/O #1: Pmic_ioRxByte_CS(PLDOx_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getPldoRtVTrackRange() read failure path.
 */
void test_neg_power_pwrGetPldoCfg_rtVtrackRangeReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.validParams = PMIC_CFG_PWR_PLDO_RT_VALID;

    // I/O #1: Pmic_ioRxByte_CS(PLDO_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getPldoPGoodCfg() read failure path.
 */
void test_neg_power_pwrGetPldoCfg_pgoodCfgReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.validParams = PMIC_CFG_PWR_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID;

    // I/O #1: Pmic_ioRxByte_CS(LDO_PGOOD_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getPldoVmonDgl() read failure path.
 */
void test_neg_power_pwrGetPldoCfg_vmonDglReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.validParams = PMIC_CFG_PWR_PLDO_VMON_DGL_VALID;

    // I/O #1: Pmic_ioRxByte_CS(VMON_DGL_CFG3_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getPldoDischarge() read failure path.
 */
void test_neg_power_pwrGetPldoCfg_dischargeReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.validParams = PMIC_CFG_PWR_PLDO_DISABLE_DISCHARGE_VALID;

    // I/O #1: Pmic_ioRxByte_CS(LDO_DSCG_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getPldoIlimDgl() read failure path.
 */
void test_neg_power_pwrGetPldoCfg_ilimDglReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.validParams = PMIC_CFG_PWR_PLDO_ILIM_DGL_VALID;

    // I/O #1: Pmic_ioRxByte_CS(ILIM_DGL_CFG_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: GetRsrcStatus I/O failure and NOT_SUPPORTED paths
/* ========================================================================== */

/**
 * @brief Test PWR_getLdoStat() I/O failure path.
 */
void test_neg_power_pwrGetRsrcStatus_ldoIlimStatReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    stat.pwrRsrc = PMIC_PWR_LDO1;
    stat.validParams = PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID;

    // I/O #1: Pmic_ioRxByte_CS(ILIM_STAT_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getPldoStat() NOT_SUPPORTED: BB_LITE_VALID with PLDO.
 */
void test_neg_power_pwrGetRsrcStatus_pldoBbLiteNotSupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .pwrRsrc = PMIC_PWR_PLDO1,
        .validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test PWR_getPldoStat() NOT_SUPPORTED: BB_ILIM_LVL_VALID with PLDO.
 */
void test_neg_power_pwrGetRsrcStatus_pldoBbIlimLvlNotSupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .pwrRsrc = PMIC_PWR_PLDO1,
        .validParams = PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test PWR_getPldoStat() I/O failure path.
 */
void test_neg_power_pwrGetRsrcStatus_pldoIlimStatReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    stat.pwrRsrc = PMIC_PWR_PLDO1;
    stat.validParams = PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID;

    // I/O #1: Pmic_ioRxByte_CS(ILIM_STAT_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getBbStat() I/O failure path.
 */
void test_neg_power_pwrGetRsrcStatus_bbIlimStatReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    stat.pwrRsrc = PMIC_PWR_BUCK_BOOST;
    stat.validParams = PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID;

    // I/O #1: Pmic_ioRxByte_CS(ILIM_STAT_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getExtVmonStat() NOT_SUPPORTED: ILIM_ERR_VALID with EXT_VMON.
 */
void test_neg_power_pwrGetRsrcStatus_extVmonIlimErrNotSupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .pwrRsrc = PMIC_PWR_EXT_VMON1,
        .validParams = PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test PWR_getExtVmonStat() NOT_SUPPORTED: BB_LITE_VALID with EXT_VMON.
 */
void test_neg_power_pwrGetRsrcStatus_extVmonBbLiteNotSupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .pwrRsrc = PMIC_PWR_EXT_VMON1,
        .validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test PWR_getExtVmonStat() NOT_SUPPORTED: BB_ILIM_LVL_VALID with EXT_VMON.
 */
void test_neg_power_pwrGetRsrcStatus_extVmonBbIlimLvlNotSupported(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .pwrRsrc = PMIC_PWR_EXT_VMON1,
        .validParams = PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/**
 * @brief Test PWR_getExtVmonStat() I/O failure path.
 */
void test_neg_power_pwrGetRsrcStatus_extVmonStatReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    stat.pwrRsrc = PMIC_PWR_EXT_VMON1;
    stat.validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID;

    // I/O #1: Pmic_ioRxByte_CS(EXT_VMON_STAT_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: I/O failure paths in BB/LDO/PLDO/ExtVmon setCfg
/* ========================================================================== */

/**
 * @brief Test PWR_setBbSsEnLvlPGoodCfg() write failure path ().
 */
void test_neg_power_pwrSetBuckBoostCfg_ssEnIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrBuckBoostCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_BB_SS_EN_VALID;
    cfg.ssEn = true;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setBbVmonThrBstTmo() write failure path ().
 */
void test_neg_power_pwrSetBuckBoostCfg_vmonThrIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrBuckBoostCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_BB_VMON_THR_VALID;
    cfg.vmonThr = PMIC_PWR_BB_VMON_THR_5_PCT;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setBbVmonDgl() write failure path ().
 */
void test_neg_power_pwrSetBuckBoostCfg_vmonDglIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrBuckBoostCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_BB_VMON_DGL_VALID;
    cfg.vmonDgl = PMIC_PWR_RSRC_VMON_DGL_8_US;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetBuckBoostCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setLdoRtLvlIlimLvl() write failure path ().
 */
void test_neg_power_pwrSetLdoCfg_lvlIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_LDO_LVL_VALID;
    cfg.ldo = PMIC_PWR_LDO1;
    cfg.lvl = 0U;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setLdoMode() write failure path ().
 */
void test_neg_power_pwrSetLdoCfg_modeIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_LDO_MODE_VALID;
    cfg.ldo = PMIC_PWR_LDO1;
    cfg.mode = PMIC_PWR_LDO_EN_AS_LDO_IN_OPER;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setLdoPGoodCfg() write failure path ().
 */
void test_neg_power_pwrSetLdoCfg_pgoodIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID;
    cfg.ldo = PMIC_PWR_LDO1;
    cfg.includeOvUvStatInPGood = true;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setLdoVmonThr() write failure path ().
 */
void test_neg_power_pwrSetLdoCfg_vmonThrIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_LDO_VMON_THR_VALID;
    cfg.ldo = PMIC_PWR_LDO1;
    cfg.vmonThr = PMIC_PWR_LDO_VMON_THR_3P5_PCT;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setLdoVmonDgl() write failure path ().
 */
void test_neg_power_pwrSetLdoCfg_vmonDglIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_LDO_VMON_DGL_VALID;
    cfg.ldo = PMIC_PWR_LDO1;
    cfg.vmonDgl = PMIC_PWR_RSRC_VMON_DGL_8_US;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setLdoDischargeDisable() write failure path ().
 */
void test_neg_power_pwrSetLdoCfg_dischargeDisableIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_LDO_DISABLE_DISCHARGE_VALID;
    cfg.ldo = PMIC_PWR_LDO1;
    cfg.disableDischarge = true;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setLdoIlimDgl() write failure path ().
 */
void test_neg_power_pwrSetLdoCfg_ilimDglIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrLdoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_LDO_ILIM_DGL_VALID;
    cfg.ldo = PMIC_PWR_LDO1;
    cfg.ilimDgl = PMIC_PWR_LDO_ILIM_DEGLITCH_10_US;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setPldoTrackingModeLvlIlimLvl() write failure path ().
 */
void test_neg_power_pwrSetPldoCfg_lvlIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_PLDO_LVL_VALID;
    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.lvl = 0U;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setPldoRtVTrackRange() write failure path ().
 */
void test_neg_power_pwrSetPldoCfg_rtIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_PLDO_RT_VALID;
    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.rampTime = 0U;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setPldoPGoodCfg() write failure path ().
 */
void test_neg_power_pwrSetPldoCfg_pgoodIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID;
    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.includeOvUvStatInPGood = true;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setPldoMode() write failure path ().
 */
void test_neg_power_pwrSetPldoCfg_modeIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_PLDO_MODE_VALID;
    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.mode = PMIC_PWR_PLDO_EN_LDO_OPER;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setPldoDischarge() write failure path ().
 */
void test_neg_power_pwrSetPldoCfg_dischargeIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_PLDO_DISABLE_DISCHARGE_VALID;
    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.disableDischarge = true;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setPldoVmonDgl() write failure path ().
 */
void test_neg_power_pwrSetPldoCfg_vmonDglIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_PLDO_VMON_DGL_VALID;
    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.vmonDgl = PMIC_PWR_RSRC_VMON_DGL_8_US;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setPldoIlimDgl() write failure path ().
 */
void test_neg_power_pwrSetPldoCfg_ilimDglIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_PLDO_ILIM_DGL_VALID;
    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.ilimDgl = PMIC_PWR_LDO_ILIM_DEGLITCH_10_US;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setPldoVmonThr() write failure path ().
 */
void test_neg_power_pwrSetPldoCfg_vmonThrIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_PLDO_VMON_THR_VALID;
    cfg.pldo = PMIC_PWR_PLDO1;
    cfg.vmonThr = PMIC_PWR_PLDO_VMON_THR_8_PCT;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setExtVmonModePGoodCfg() write failure path ().
 */
void test_neg_power_pwrSetExtVmonCfg_modeIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrExtVmonCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_EXT_VMON_MODE_VALID;
    cfg.extVmon = PMIC_PWR_EXT_VMON1;
    cfg.mode = 0U;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setExtVmonThr() write failure path ().
 */
void test_neg_power_pwrSetExtVmonCfg_vmonThrIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrExtVmonCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_EXT_VMON_THR_VALID;
    cfg.extVmon = PMIC_PWR_EXT_VMON1;
    cfg.vmonThr = PMIC_PWR_EXT_VMON_THR_DIGITAL_MODE_HYSTERESIS;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setExtVmonDgl() write failure path ().
 */
void test_neg_power_pwrSetExtVmonCfg_vmonDglIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrExtVmonCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_PWR_EXT_VMON_DGL_VALID;
    cfg.extVmon = PMIC_PWR_EXT_VMON1;
    cfg.vmonDgl = PMIC_PWR_RSRC_VMON_DGL_8_US;

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetExtVmonCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: PWR_clrExtVmonStat EXT_VMON1 paths (L2810, L2815)
/* ========================================================================== */

/**
 * @brief Test PWR_clrExtVmonStat() EXT_VMON1 OV_ERR true-arm ().
 */
void test_pos_power_pwrClrRsrcStatus_extVmon1_ovErr(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test PWR_clrExtVmonStat() no-op path when validParams=0 ( false).
 */
void test_pos_power_pwrClrRsrcStatus_extVmon1_noOp(void)
{
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = 0U,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };
    int32_t status = Pmic_pwrClrRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_pwrSetPGoodInStby with ioRxByte failure.
 */
void test_neg_power_pwrSetPGoodInStby_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPGoodInStby(&pmicHandle, true);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: PWR_getExtVmonStat / PWR_getPldoStat I/O failures
/* ========================================================================== */

/**
 * @brief Test PWR_getExtVmonStat() I/O read failure path.
 */
void test_neg_power_pwrGetRsrcStatus_extVmonStat_ioReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_EXT_VMON1
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getPldoStat() I/O read failure path.
 */
void test_neg_power_pwrGetRsrcStatus_pldoStat_ioReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_UV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: PWR_getBbDcdcStat / PWR_getBbTsdStat / PWR_getLdoOvUvStat
// I/O failure paths (Pmic_ioRxByte_CS has no upstream success guarantee)
/* ========================================================================== */

/**
 * @brief Test PWR_getBbDcdcStat() I/O read failure path (DCDC_STAT_REG).
 */
void test_neg_power_pwrGetRsrcStatus_bbDcdcStat_ioReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_BB_LITE_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    // I/O: Pmic_ioRxByte_CS(DCDC_STAT_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getBbTsdStat() I/O read failure path (THERMAL_STAT2_REG).
 */
void test_neg_power_pwrGetRsrcStatus_bbTsdStat_ioReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_BUCK_BOOST
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    // I/O: Pmic_ioRxByte_CS(THERMAL_STAT2_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getLdoOvUvStat() I/O read failure path (VMON_LDO_STAT_REG).
 */
void test_neg_power_pwrGetRsrcStatus_ldoOvUvStat_ioReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_OV_ERR_VALID,
        .pwrRsrc = PMIC_PWR_LDO1
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    // I/O: Pmic_ioRxByte_CS(VMON_LDO_STAT_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: PWR_getLdoTsdStat / PWR_getPldoTsdStat I/O failure paths
// (Pmic_ioRxByte_CS has no upstream success guarantee)
/* ========================================================================== */

/**
 * @brief Test PWR_getLdoTsdStat() I/O read failure path (THERMAL_STAT1_REG).
 */
void test_neg_power_pwrGetRsrcStatus_ldoTsdStat_ioReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_ERR_VALID,
        .pwrRsrc = PMIC_PWR_LDO1
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    // I/O: Pmic_ioRxByte_CS(THERMAL_STAT1_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_getPldoTsdStat() I/O read failure path (THERMAL_STAT2_REG).
 */
void test_neg_power_pwrGetRsrcStatus_pldoTsdStat_ioReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrRsrcStatus_t stat = {
        .validParams = PMIC_PWR_RSRC_STAT_TSD_WARN_VALID,
        .pwrRsrc = PMIC_PWR_PLDO1
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    // I/O: Pmic_ioRxByte_CS(THERMAL_STAT2_REG) - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

void test_neg_power_pwrSetPldoCfg_validatePldoMode_noModeValidParam(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_PwrPldoCfg_t cfg = {
        .validParams = PMIC_CFG_PWR_PLDO_LVL_VALID,
        .pldo = PMIC_PWR_PLDO1,
        .lvl = PMIC_PWR_LDO_LVL_3P3V
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject failure on first I/O so the LVL register read fails,
    // confirming we reached the I/O path inside PWR_setPldoTrackingModeLvlIlimLvl.
    // PWR_validatePldoMode is not invoked because PMIC_CFG_PWR_PLDO_MODE_VALID
    // is absent from validParams, exercising the false branch of its guard. */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetPldoCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_validatePldoMode() boundary: mode == modeMax for PLDO2
 *        (mode > modeMax comparison's false side at the exact upper bound,
 *        a combination not exercised by any existing in-range-mode test).
 */
void test_pos_power_pwrSetPldoCfg_validatePldoMode_pldo2ModeAtMax(void)
{
    Pmic_PwrPldoCfg_t setCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_MODE_VALID,
        .pldo = PMIC_PWR_PLDO2,
        .mode = PMIC_PWR_PLDO2_MODE_MAX
    };
    Pmic_PwrPldoCfg_t getCfg = {
        .validParams = PMIC_CFG_PWR_PLDO_MODE_VALID,
        .pldo = PMIC_PWR_PLDO2
    };

    int32_t status = Pmic_pwrSetPldoCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetPldoCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == setCfg.mode);
}
