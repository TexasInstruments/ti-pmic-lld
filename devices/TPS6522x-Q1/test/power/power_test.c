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

#include "platform.h"
#include "power_test.h"

#ifdef BUILD_MOCK
#include "pmic_mock_types.h"
#include "pmic_mock_core.h"
#endif

#ifdef BUILD_MOCK
extern PmicMockDevice_t *platform_getMockDevice(void);
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0};

/* ========================================================================== */
/*                       Negative Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test Pmic_pwrSetBuckCfg with NULL handle.
 */
void test_neg_power_setBuckCfg_nullHandle(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = PMIC_CFG_PWR_BUCK_EN_VALID, .resource = PMIC_POWER_RESOURCE_BUCK1, .buckEn = true};
    int32_t status = Pmic_pwrSetBuckCfg(NULL, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with NULL buckCfg parameter.
 */
void test_neg_power_setBuckCfg_nullBuckCfg(void)
{
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid validParams.
 */
void test_neg_power_setBuckCfg_invalidValidParams(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_BUCK1};
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid resource.
 */
void test_neg_power_setBuckCfg_invalidResource(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = PMIC_CFG_PWR_BUCK_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO1, .buckEn = true};
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid slew rate.
 */
void test_neg_power_setBuckCfg_invalidSlewRate(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .slewRate = PMIC_POWER_BUCK_SLEW_RATE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid vset for BUCK1.
 */
void test_neg_power_setBuckCfg_invalidVsetBuck1(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .vset = PMIC_POWER_BUCK1_VSET_MIN - 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid vmon threshold.
 */
void test_neg_power_setBuckCfg_invalidVmonThr(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK2,
        .vmonThr = PMIC_POWER_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid group select.
 */
void test_neg_power_setBuckCfg_invalidGrpSel(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK3,
        .grpSel = PMIC_POWER_GRP_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetBuckCfg with NULL handle.
 */
void test_neg_power_getBuckCfg_nullHandle(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = PMIC_CFG_PWR_BUCK_EN_VALID, .resource = PMIC_POWER_RESOURCE_BUCK1};
    int32_t status = Pmic_pwrGetBuckCfg(NULL, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetBuckCfg with NULL buckCfg parameter.
 */
void test_neg_power_getBuckCfg_nullBuckCfg(void)
{
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetBuckCfg with invalid validParams.
 */
void test_neg_power_getBuckCfg_invalidValidParams(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_BUCK1};
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetBuckCfg with invalid resource.
 */
void test_neg_power_getBuckCfg_invalidResource(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = PMIC_CFG_PWR_BUCK_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO2};
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with NULL handle.
 */
void test_neg_power_setLdoCfg_nullHandle(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_CFG_PWR_LDO_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO1, .ldoEn = true};
    int32_t status = Pmic_pwrSetLdoCfg(NULL, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with NULL ldoCfg parameter.
 */
void test_neg_power_setLdoCfg_nullLdoCfg(void)
{
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid validParams.
 */
void test_neg_power_setLdoCfg_invalidValidParams(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_LDO1};
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid resource.
 */
void test_neg_power_setLdoCfg_invalidResource(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_CFG_PWR_LDO_EN_VALID, .resource = PMIC_POWER_RESOURCE_BUCK1, .ldoEn = true};
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid vset for LDO1.
 */
void test_neg_power_setLdoCfg_invalidVsetLdo1(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .vset = PMIC_POWER_LDO1_VSET_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid vmon threshold.
 */
void test_neg_power_setLdoCfg_invalidVmonThr(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2,
        .vmonThr = PMIC_POWER_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid group select.
 */
void test_neg_power_setLdoCfg_invalidGrpSel(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO3,
        .grpSel = PMIC_POWER_GRP_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetLdoCfg with NULL handle.
 */
void test_neg_power_getLdoCfg_nullHandle(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_CFG_PWR_LDO_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO1};
    int32_t status = Pmic_pwrGetLdoCfg(NULL, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetLdoCfg with NULL ldoCfg parameter.
 */
void test_neg_power_getLdoCfg_nullLdoCfg(void)
{
    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetLdoCfg with invalid validParams.
 */
void test_neg_power_getLdoCfg_invalidValidParams(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_LDO2};
    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetLdoCfg with invalid resource.
 */
void test_neg_power_getLdoCfg_invalidResource(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_CFG_PWR_LDO_EN_VALID, .resource = PMIC_POWER_RESOURCE_VMON1};
    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with NULL handle.
 */
void test_neg_power_setVccaVmonCfg_nullHandle(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID, .resource = PMIC_POWER_RESOURCE_VCCA_VMON, .vmonEn = true};
    int32_t status = Pmic_pwrSetVccaVmonCfg(NULL, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with NULL vccaVmonCfg parameter.
 */
void test_neg_power_setVccaVmonCfg_nullVccaVmonCfg(void)
{
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid validParams.
 */
void test_neg_power_setVccaVmonCfg_invalidValidParams(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_VMON1};
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid resource.
 */
void test_neg_power_setVccaVmonCfg_invalidResource(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID, .resource = PMIC_POWER_RESOURCE_BUCK1, .vmonEn = true};
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid pgSet for VCCA.
 */
void test_neg_power_setVccaVmonCfg_invalidPgSetVcca(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .pgSet = PMIC_POWER_VCCA_VMON_PG_SET_MAX + 1U
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid threshold for VCCA.
 */
void test_neg_power_setVccaVmonCfg_invalidThrVcca(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .vmonThr = PMIC_POWER_VCCA_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid group select.
 */
void test_neg_power_setVccaVmonCfg_invalidGrpSel(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2,
        .grpSel = PMIC_POWER_GRP_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg with NULL handle.
 */
void test_neg_power_getVccaVmonCfg_nullHandle(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID, .resource = PMIC_POWER_RESOURCE_VCCA_VMON};
    int32_t status = Pmic_pwrGetVccaVmonCfg(NULL, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg with NULL vccaVmonCfg parameter.
 */
void test_neg_power_getVccaVmonCfg_nullVccaVmonCfg(void)
{
    int32_t status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg with invalid validParams.
 */
void test_neg_power_getVccaVmonCfg_invalidValidParams(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_VMON1};
    int32_t status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg with invalid resource.
 */
void test_neg_power_getVccaVmonCfg_invalidResource(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO1};
    int32_t status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetGlobalVmonDegl with NULL handle.
 */
void test_neg_power_setGlobalVmonDegl_nullHandle(void)
{
    int32_t status = Pmic_pwrSetGlobalVmonDegl(NULL, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetGlobalVmonDegl with invalid value.
 */
void test_neg_power_setGlobalVmonDegl_invalidValue(void)
{
    int32_t status = Pmic_pwrSetGlobalVmonDegl(&pmicHandle, PMIC_POWER_VMON_DEGL_SEL_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with NULL handle.
 */
void test_neg_power_setThermalCfg_nullHandle(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = PMIC_CFG_PWR_TWARN_LEVEL_VALID, .twarnLvl = PMIC_POWER_TWARN_LEVEL_130C};
    int32_t status = Pmic_pwrSetThermalCfg(NULL, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with NULL thermalCfg parameter.
 */
void test_neg_power_setThermalCfg_nullThermalCfg(void)
{
    int32_t status = Pmic_pwrSetThermalCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with invalid validParams.
 */
void test_neg_power_setThermalCfg_invalidValidParams(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = 0U};
    int32_t status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with invalid twarnLvl.
 */
void test_neg_power_setThermalCfg_invalidTwarnLvl(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = PMIC_CFG_PWR_TWARN_LEVEL_VALID, .twarnLvl = PMIC_POWER_TWARN_LEVEL_MAX + 1U};
    int32_t status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with invalid tsdOrdLvl.
 */
void test_neg_power_setThermalCfg_invalidTsdOrdLvl(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = PMIC_CFG_PWR_TSD_ORD_LEVEL_VALID, .tsdOrdLvl = PMIC_POWER_TSD_ORD_LEVEL_MAX + 1U};
    int32_t status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetThermalCfg with NULL handle.
 */
void test_neg_power_getThermalCfg_nullHandle(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = PMIC_CFG_PWR_TWARN_LEVEL_VALID};
    int32_t status = Pmic_pwrGetThermalCfg(NULL, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetThermalCfg with NULL thermalCfg parameter.
 */
void test_neg_power_getThermalCfg_nullThermalCfg(void)
{
    int32_t status = Pmic_pwrGetThermalCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetThermalCfg with invalid validParams.
 */
void test_neg_power_getThermalCfg_invalidValidParams(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = 0U};
    int32_t status = Pmic_pwrGetThermalCfg(&pmicHandle, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetSpreadSpectrumCfg with NULL handle.
 */
void test_neg_power_setSpreadSpectrumCfg_nullHandle(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {.validParams = PMIC_CFG_PWR_SS_EN_VALID, .ssEn = true};
    int32_t status = Pmic_pwrSetSpreadSpectrumCfg(NULL, &ssCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetSpreadSpectrumCfg with NULL ssCfg parameter.
 */
void test_neg_power_setSpreadSpectrumCfg_nullSsCfg(void)
{
    int32_t status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetSpreadSpectrumCfg with invalid validParams.
 */
void test_neg_power_setSpreadSpectrumCfg_invalidValidParams(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {.validParams = 0U};
    int32_t status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, &ssCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetSpreadSpectrumCfg with NULL handle.
 */
void test_neg_power_getSpreadSpectrumCfg_nullHandle(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {.validParams = PMIC_CFG_PWR_SS_EN_VALID};
    int32_t status = Pmic_pwrGetSpreadSpectrumCfg(NULL, &ssCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetSpreadSpectrumCfg with NULL ssCfg parameter.
 */
void test_neg_power_getSpreadSpectrumCfg_nullSsCfg(void)
{
    int32_t status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetSpreadSpectrumCfg with invalid validParams.
 */
void test_neg_power_getSpreadSpectrumCfg_invalidValidParams(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {.validParams = 0U};
    int32_t status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, &ssCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetRsrcStatus with NULL handle.
 */
void test_neg_power_getRsrcStatus_nullHandle(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {.validParams = PMIC_PWR_BUCK1_UVOV_VALID};
    int32_t status = Pmic_pwrGetRsrcStatus(NULL, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetRsrcStatus with NULL rsrcStatus parameter.
 */
void test_neg_power_getRsrcStatus_nullRsrcStatus(void)
{
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetRsrcStatus with invalid validParams.
 */
void test_neg_power_getRsrcStatus_invalidValidParams(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {.validParams = 0U};
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                       Positive Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test BUCK1 enable and disable.
 */
void test_pos_power_buck1_enableDisable(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .buckEn = false
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };
    int32_t status;

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.buckEn == false);

    buckCfgSet.buckEn = true;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.buckEn == true);
}

/**
 * @brief Test BUCK2 voltage set and get.
 */
void test_pos_power_buck2_vset(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK2,
        .vset = 0x20U
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_CFG_PWR_BUCK_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK2
    };
    int32_t status;

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vset == 0x20U);
}

/**
 * @brief Test BUCK3 slew rate configuration.
 */
void test_pos_power_buck3_slewRate(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK3,
        .slewRate = PMIC_POWER_BUCK_SLEW_RATE_10_MV_PER_US
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_CFG_PWR_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK3
    };
    int32_t status;

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.slewRate == PMIC_POWER_BUCK_SLEW_RATE_10_MV_PER_US);
}

/**
 * @brief Test BUCK4 VMON threshold.
 */
void test_pos_power_buck4_vmonThr(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK4,
        .vmonThr = PMIC_POWER_VMON_THR_3_PCT_30_MV
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_CFG_PWR_BUCK_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK4
    };
    int32_t status;

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vmonThr == PMIC_POWER_VMON_THR_3_PCT_30_MV);
}

/**
 * @brief Test BUCK1 group select.
 */
void test_pos_power_buck1_grpSel(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .grpSel = PMIC_POWER_GRP_SEL_MCU
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_CFG_PWR_BUCK_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };
    int32_t status;

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.grpSel == PMIC_POWER_GRP_SEL_MCU);
}

/**
 * @brief Test LDO1 enable and disable.
 */
void test_pos_power_ldo1_enableDisable(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_CFG_PWR_LDO_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .ldoEn = false
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_CFG_PWR_LDO_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };
    int32_t status;

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.ldoEn == false);

    ldoCfgSet.ldoEn = true;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.ldoEn == true);
}

/**
 * @brief Test LDO2 voltage set and get.
 */
void test_pos_power_ldo2_vset(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_CFG_PWR_LDO_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2,
        .vset = 0x15U
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_CFG_PWR_LDO_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2
    };
    int32_t status;

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.vset == 0x15U);
}

/**
 * @brief Test LDO3 bypass enable.
 */
void test_pos_power_ldo3_bypassEn(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_CFG_PWR_LDO_BYP_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO3,
        .bypEn = true
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_CFG_PWR_LDO_BYP_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO3
    };
    int32_t status;

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.bypEn == true);
}

/**
 * @brief Test LDO1 VMON threshold.
 */
void test_pos_power_ldo1_vmonThr(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .vmonThr = PMIC_POWER_VMON_THR_4_PCT_40_MV
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };
    int32_t status;

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.vmonThr == PMIC_POWER_VMON_THR_4_PCT_40_MV);
}

/**
 * @brief Test LDO2 group select.
 */
void test_pos_power_ldo2_grpSel(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_CFG_PWR_LDO_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2,
        .grpSel = PMIC_POWER_GRP_SEL_SOC
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_CFG_PWR_LDO_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2
    };
    int32_t status;

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.grpSel == PMIC_POWER_GRP_SEL_SOC);
}

/**
 * @brief Test VCCA enable and disable.
 */
void test_pos_power_vcca_enableDisable(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .vmonEn = false
    };
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vccaVmonCfgGet.vmonEn == false);

    vccaVmonCfgSet.vmonEn = true;
    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vccaVmonCfgGet.vmonEn == true);
}

/**
 * @brief Test VCCA PG set configuration.
 */
void test_pos_power_vcca_pgSet(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .pgSet = PMIC_POWER_VCCA_VMON_PG_SET_3P3_V
    };
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vccaVmonCfgGet.pgSet == PMIC_POWER_VCCA_VMON_PG_SET_3P3_V);
}

/**
 * @brief Test VCCA threshold configuration.
 */
void test_pos_power_vcca_threshold(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .vmonThr = PMIC_POWER_VCCA_VMON_THR_3_PCT
    };
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vccaVmonCfgGet.vmonThr == PMIC_POWER_VCCA_VMON_THR_3_PCT);
}

/**
 * @brief Test VCCA group select.
 */
void test_pos_power_vcca_grpSel(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .grpSel = PMIC_POWER_GRP_SEL_OTHER
    };
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vccaVmonCfgGet.grpSel == PMIC_POWER_GRP_SEL_OTHER);
}

/**
 * @brief Test VMON2 group select configuration.
 */
void test_pos_power_vmon2_grpSel(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2,
        .grpSel = PMIC_POWER_GRP_SEL_MCU
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vmonCfgGet.grpSel == PMIC_POWER_GRP_SEL_MCU);
}

/**
 * @brief Test VMON1 enable and disable.
 */
void test_pos_power_vmon1_enableDisable(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1,
        .vmonEn = false
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vmonCfgGet.vmonEn == false);
}

/**
 * @brief Test VMON2 PG set configuration.
 */
void test_pos_power_vmon2_pgSet(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2,
        .pgSet = 0x30U
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vmonCfgGet.pgSet == 0x30U);
}

/**
 * @brief Test VMON1 PG set configuration.
 */
void test_pos_power_vmon1_pgSet(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1,
        .pgSet = 0x20U
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vmonCfgGet.pgSet == 0x20U);
}

/**
 * @brief Test VMON2 enable configuration.
 */
void test_pos_power_vmon2_enableDisable(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2,
        .vmonEn = true
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vmonCfgGet.vmonEn == true);
}

/**
 * @brief Test VMON1 invalid PG set configuration.
 */
void test_neg_power_setVccaVmonCfg_invalidPgSetVmon1(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1,
        .pgSet = 0x09U  // Below PMIC_POWER_VMON1_PG_SET_MIN (0x0A)
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test global VMON deglitch setting for all valid values.
 */
void test_pos_power_globalVmonDegl_allValues(void)
{
    int32_t status;

    /* Test all valid deglitch values */
    for (uint8_t val = PMIC_POWER_VMON_DEGL_SEL_MIN; val <= PMIC_POWER_VMON_DEGL_SEL_MAX; val++)
    {
        status = Pmic_pwrSetGlobalVmonDegl(&pmicHandle, val);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

/**
 * @brief Test thermal warning level set and get.
 */
void test_pos_power_thermal_twarnLvl(void)
{
    Pmic_PwrThermalCfg_t thermalCfgSet = {
        .validParams = PMIC_CFG_PWR_TWARN_LEVEL_VALID,
        .twarnLvl = PMIC_POWER_TWARN_LEVEL_130C
    };
    Pmic_PwrThermalCfg_t thermalCfgGet = {
        .validParams = PMIC_CFG_PWR_TWARN_LEVEL_VALID
    };
    int32_t status;

    status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetThermalCfg(&pmicHandle, &thermalCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(thermalCfgGet.twarnLvl == PMIC_POWER_TWARN_LEVEL_130C);
}

/**
 * @brief Test thermal orderly shutdown level set and get.
 */
void test_pos_power_thermal_tsdOrdLvl(void)
{
    Pmic_PwrThermalCfg_t thermalCfgSet = {
        .validParams = PMIC_CFG_PWR_TSD_ORD_LEVEL_VALID,
        .tsdOrdLvl = PMIC_POWER_TSD_ORD_LEVEL_140C
    };
    Pmic_PwrThermalCfg_t thermalCfgGet = {
        .validParams = PMIC_CFG_PWR_TSD_ORD_LEVEL_VALID
    };
    int32_t status;

    status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetThermalCfg(&pmicHandle, &thermalCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(thermalCfgGet.tsdOrdLvl == PMIC_POWER_TSD_ORD_LEVEL_140C);
}

/**
 * @brief Test spread spectrum enable and disable.
 */
void test_pos_power_spreadSpectrum_enableDisable(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfgSet = {
        .validParams = PMIC_CFG_PWR_SS_EN_VALID,
        .ssEn = false
    };
    Pmic_PwrSpreadSpectrumCfg_t ssCfgGet = {
        .validParams = PMIC_CFG_PWR_SS_EN_VALID
    };
    int32_t status;

    status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, &ssCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, &ssCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ssCfgGet.ssEn == false);

    ssCfgSet.ssEn = true;
    status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, &ssCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, &ssCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ssCfgGet.ssEn == true);
}

/**
 * @brief Test spread spectrum depth configuration.
 */
void test_pos_power_spreadSpectrum_depth(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfgSet = {
        .validParams = PMIC_CFG_PWR_SS_DEPTH_VALID,
        .ssDepth = PMIC_POWER_SS_DEPTH_4_PCT
    };
    Pmic_PwrSpreadSpectrumCfg_t ssCfgGet = {
        .validParams = PMIC_CFG_PWR_SS_DEPTH_VALID
    };
    int32_t status;

    status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, &ssCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, &ssCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ssCfgGet.ssDepth == PMIC_POWER_SS_DEPTH_4_PCT);
}

/**
 * @brief Test get all BUCK UVOV status.
 */
void test_pos_power_rsrcStatus_buckUVOV(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {
        .validParams = PMIC_PWR_BUCK1_UVOV_VALID | PMIC_PWR_BUCK2_UVOV_VALID |
                       PMIC_PWR_BUCK3_UVOV_VALID | PMIC_PWR_BUCK4_UVOV_VALID
    };
    int32_t status;

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test get all LDO UVOV status.
 */
void test_pos_power_rsrcStatus_ldoUVOV(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {
        .validParams = PMIC_PWR_LDO1_UVOV_VALID | PMIC_PWR_LDO2_UVOV_VALID |
                       PMIC_PWR_LDO3_UVOV_VALID
    };
    int32_t status;

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test get all VMON UVOV status.
 */
void test_pos_power_rsrcStatus_vmonUVOV(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {
        .validParams = PMIC_PWR_VMON1_UVOV_VALID | PMIC_PWR_VMON2_UVOV_VALID |
                       PMIC_PWR_VCCA_VMON_UVOV_VALID
    };
    int32_t status;

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test BUCK Set/Get with only pldnEn parameter.
 */
void test_pos_power_buck_pldnEn(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_PLDN_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .pldnEn = true
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_CFG_PWR_BUCK_PLDN_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };
    int32_t status;

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.pldnEn == true);
}

/**
 * @brief Test BUCK Set/Get with only vmonEn parameter.
 */
void test_pos_power_buck_vmonEn(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .vmonEn = true
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_CFG_PWR_BUCK_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };
    int32_t status;

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vmonEn == true);
}

/**
 * @brief Test BUCK Set/Get with only fpwmEn parameter.
 */
void test_pos_power_buck_fpwmEn(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_FPWM_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .fpwmEn = true
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_CFG_PWR_BUCK_FPWM_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };
    int32_t status;

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.fpwmEn == true);
}

/**
 * @brief Test BUCK combined configuration.
 */
void test_pos_power_buck_combinedConfig(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID | PMIC_CFG_PWR_BUCK_PLDN_EN_VALID |
                       PMIC_CFG_PWR_BUCK_VMON_EN_VALID | PMIC_CFG_PWR_BUCK_FPWM_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .buckEn = true,
        .pldnEn = true,
        .vmonEn = true,
        .fpwmEn = false
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID | PMIC_CFG_PWR_BUCK_PLDN_EN_VALID |
                       PMIC_CFG_PWR_BUCK_VMON_EN_VALID | PMIC_CFG_PWR_BUCK_FPWM_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };
    int32_t status;

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.buckEn == buckCfgSet.buckEn);
    PLATFORM_ASSERT(buckCfgGet.pldnEn == buckCfgSet.pldnEn);
}

/**
 * @brief Test LDO combined configuration.
 */
void test_pos_power_ldo_combinedConfig(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_CFG_PWR_LDO_EN_VALID | PMIC_CFG_PWR_LDO_VMON_EN_VALID |
                       PMIC_CFG_PWR_LDO_DISCHARGE_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2,
        .ldoEn = true,
        .vmonEn = true,
        .dischargeEn = true
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_CFG_PWR_LDO_EN_VALID | PMIC_CFG_PWR_LDO_VMON_EN_VALID |
                       PMIC_CFG_PWR_LDO_DISCHARGE_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2
    };
    int32_t status;

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.ldoEn == ldoCfgSet.ldoEn);
    PLATFORM_ASSERT(ldoCfgGet.vmonEn == ldoCfgSet.vmonEn);
}

/**
 * @brief Test VMON combined configuration.
 */
void test_pos_power_vmon_combinedConfig(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID | PMIC_CFG_PWR_VCCA_VMON_THR_VALID |
                       PMIC_CFG_PWR_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1,
        .vmonEn = true,
        .vmonThr = PMIC_POWER_VMON_THR_3_PCT_30_MV,
        .grpSel = PMIC_POWER_GRP_SEL_MCU
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID | PMIC_CFG_PWR_VCCA_VMON_THR_VALID |
                       PMIC_CFG_PWR_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vmonCfgGet.vmonEn == vmonCfgSet.vmonEn);
}

/**
 * @brief Test thermal combined configuration.
 */
void test_pos_power_thermal_combinedConfig(void)
{
    Pmic_PwrThermalCfg_t thermalCfgSet = {
        .validParams = PMIC_CFG_PWR_TWARN_LEVEL_VALID | PMIC_CFG_PWR_TSD_ORD_LEVEL_VALID,
        .twarnLvl = PMIC_POWER_TWARN_LEVEL_140C,
        .tsdOrdLvl = PMIC_POWER_TSD_ORD_LEVEL_145C
    };
    Pmic_PwrThermalCfg_t thermalCfgGet = {
        .validParams = PMIC_CFG_PWR_TWARN_LEVEL_VALID | PMIC_CFG_PWR_TSD_ORD_LEVEL_VALID
    };
    int32_t status;

    status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetThermalCfg(&pmicHandle, &thermalCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(thermalCfgGet.twarnLvl == thermalCfgSet.twarnLvl);
    PLATFORM_ASSERT(thermalCfgGet.tsdOrdLvl == thermalCfgSet.tsdOrdLvl);
}

/**
 * @brief Test spread spectrum combined configuration.
 */
void test_pos_power_spreadSpectrum_combinedConfig(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfgSet = {
        .validParams = PMIC_CFG_PWR_SS_EN_VALID | PMIC_CFG_PWR_SS_DEPTH_VALID,
        .ssEn = true,
        .ssDepth = PMIC_POWER_SS_DEPTH_7_PCT
    };
    Pmic_PwrSpreadSpectrumCfg_t ssCfgGet = {
        .validParams = PMIC_CFG_PWR_SS_EN_VALID | PMIC_CFG_PWR_SS_DEPTH_VALID
    };
    int32_t status;

    status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, &ssCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, &ssCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ssCfgGet.ssEn == ssCfgSet.ssEn);
    PLATFORM_ASSERT(ssCfgGet.ssDepth == ssCfgSet.ssDepth);
}

/* ========================================================================== */
/*                        Build Mock Property Tests                           */
/* ========================================================================== */

/**
 * @brief Property test: BUCK voltage bounds.
 */
void test_pos_power_property_buckVoltageBounds(void)
{
#ifdef BUILD_MOCK
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_CFG_PWR_BUCK_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .vset = PMIC_POWER_BUCK1_VSET_MIN
    };

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    buckCfgSet.vset = PMIC_POWER_BUCK1_VSET_MAX;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for property-based testing");
#endif
}

/**
 * @brief Property test: VMON threshold enumeration exhaustion.
 */
void test_pos_power_property_vmonThresholdEnumeration(void)
{
#ifdef BUILD_MOCK
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };
    int32_t status;

    for (uint8_t val = PMIC_POWER_VCCA_VMON_THR_MIN; val <= PMIC_POWER_VCCA_VMON_THR_MAX; val++)
    {
        vccaVmonCfgSet.vmonThr = val;
        status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for property-based testing");
#endif
}

/* ========================================================================== */
/*                         MC/DC Coverage Tests                               */
/* ========================================================================== */

void test_pos_power_ldoValidParams_twoCondition_TT(void)
{
    int32_t status;
    Pmic_PwrLdoCfg_t ldoCfg = {0};
    ldoCfg.resource = PMIC_POWER_RESOURCE_LDO1;
    /* Both conditions TRUE: validParams & PARAM_A && validParams & PARAM_B */
    ldoCfg.validParams = PMIC_CFG_PWR_LDO_VSET_VALID | PMIC_CFG_PWR_LDO_VMON_THR_VALID;
    ldoCfg.vset = 0x22U;  /* Valid LDO voltage code */
    ldoCfg.vmonThr = PMIC_POWER_VMON_THR_6_PCT_60_MV;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_ldoValidParams_twoCondition_FF(void)
{
    int32_t status;
    Pmic_PwrLdoCfg_t ldoCfg = {0};
    ldoCfg.resource = PMIC_POWER_RESOURCE_LDO1;
    /* Both conditions FALSE: neither param A nor param B set */
    ldoCfg.validParams = PMIC_CFG_PWR_LDO_EN_VALID;
    ldoCfg.ldoEn = true;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buck1Only(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_BUCK1_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buck2Only(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_BUCK2_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buck3Only(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_BUCK3_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_buck4Only(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_BUCK4_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_ldo1Only(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_LDO1_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_ldo2Only(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_LDO2_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_ldo3Only(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_LDO3_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_vmon1Only(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_VMON1_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_vmon2Only(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_VMON2_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_vccaOnly(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_VCCA_VMON_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_noBucks(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_LDO1_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getRsrcStatus_noLdoVmon(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_BUCK1_UVOV_VALID;
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_power_getRsrcStatus_buckReadError(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    /* Set both BUCK and LDO flags to trigger both register reads */
    rsrcStatus.validParams = PMIC_PWR_BUCK1_UVOV_VALID | PMIC_PWR_LDO1_UVOV_VALID;

    /* Inject communication failure for BUCK register read (line 1406) */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    /* Function should fail at BUCK read, and skip LDO/VMON read due to status check */
    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Negative Tests - Pmic_PwrGetBuckCfg / Pmic_PwrGetLdoCfg
/* ========================================================================== */

/**
 * @brief Test Pmic_pwrGetBuckCfg I/O failure inside PWR_getBuckCfg.
 */
void test_neg_power_pwrGetBuckCfg_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };

    // Inject 1 comm failure – hits the Pmic_ioRxByte_CS inside PWR_getBuckCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetLdoCfg I/O failure inside PWR_getLdoCfg.
 */
void test_neg_power_pwrGetLdoCfg_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };

    // Inject 1 comm failure – hits the Pmic_ioRxByte_CS inside PWR_getLdoCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetThermalCfg I/O failure on Pmic_ioRxByte_CS.
 */
void test_neg_power_pwrGetThermalCfg_ioRxByteCSFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrThermalCfg_t thermalCfg = {
        .validParams = PMIC_CFG_PWR_TWARN_LEVEL_VALID
    };

    // Inject 1 comm failure – hits Pmic_ioRxByte_CS(CONFIG_1_REG)
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetThermalCfg(&pmicHandle, &thermalCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetSpreadSpectrumCfg I/O failure on Pmic_ioRxByte_CS.
 */
void test_neg_power_pwrGetSpreadSpectrumCfg_ioRxByteCSFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {
        .validParams = PMIC_CFG_PWR_SS_EN_VALID
    };

    // Inject 1 comm failure – hits Pmic_ioRxByte_CS(SPREAD_SPECTRUM_1_REG)
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, &ssCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg I/O failure inside PWR_getVccaVmonCfg.
 */
void test_neg_power_pwrGetVccaVmonCfg_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };

    // Inject 1 comm failure – hits the Pmic_ioRxByte_CS inside PWR_getVccaVmonCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrSetGlobalVmonDegl I/O failure on Pmic_ioRxByte.
 */
void test_neg_power_pwrSetGlobalVmonDegl_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject 1 comm failure – hits Pmic_ioRxByte(VCCA_VMON_CTRL_REG)
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetGlobalVmonDegl(&pmicHandle, PMIC_POWER_VMON_DEGL_SEL_MIN);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrSetSpreadSpectrumCfg I/O failure on Pmic_ioRxByte.
 */
void test_neg_power_pwrSetSpreadSpectrumCfg_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {
        .validParams = PMIC_CFG_PWR_SS_EN_VALID,
        .ssEn = true
    };

    // Inject 1 comm failure – hits Pmic_ioRxByte(SPREAD_SPECTRUM_1_REG)
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, &ssCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetRsrcStatus I/O failure on STAT_LDO_VMON_REG read.
 */
void test_neg_power_pwrGetRsrcStatus_ldoVmonReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Need both BUCK and LDO flags set so both register reads are attempted
    Pmic_PwrRsrcStatus_t rsrcStatus = {0};
    rsrcStatus.validParams = PMIC_PWR_BUCK1_UVOV_VALID | PMIC_PWR_LDO1_UVOV_VALID;

    // Skip first I/O (STAT_BUCK_REG succeeds), fail second (STAT_LDO_VMON_REG)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Static helper coverage: PWR_setBuckCfg / PWR_getBuckCfg I/O failures
/* ========================================================================== */

/**
 * @brief Test Pmic_pwrSetBuckCfg when the initial ioRxByte inside the ctrl.
 */
void test_neg_power_pwrSetBuckCfg_ctrlReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .buckEn = true
    };

    // First I/O is ioRxByte(BUCK1_CTRL_REG) – inject failure immediately
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrSetBuckCfg when ioRxByte inside the slew-rate block.
 */
void test_neg_power_pwrSetBuckCfg_slewRateReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID | PMIC_CFG_PWR_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .buckEn = true,
        .slewRate = PMIC_POWER_BUCK_SLEW_RATE_MAX
    };

    // Skip ctrl read+write (#0, #1), fail slew-rate read (#2)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 2U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrSetBuckCfg when ioRxByte inside the VMON threshold.
 */
void test_neg_power_pwrSetBuckCfg_vmonThrReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .vmonThr = PMIC_POWER_VMON_THR_MAX
    };

    // First I/O is ioRxByte(pgWindowRegAddr) – inject failure immediately
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrSetBuckCfg when ioRxByte inside the group select block.
 */
void test_neg_power_pwrSetBuckCfg_grpSelReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .grpSel = PMIC_POWER_GRP_SEL_MAX
    };

    // First I/O is ioRxByte(grpSelRegAddr) – inject failure immediately
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetBuckCfg when the slew-rate ioRxByte_CS inside.
 */
void test_neg_power_pwrGetBuckCfg_slewRateReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID | PMIC_CFG_PWR_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };

    // Skip ctrl read (#0), fail slew-rate read (#1)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetBuckCfg when the vset ioRxByte_CS inside.
 */
void test_neg_power_pwrGetBuckCfg_vsetReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID | PMIC_CFG_PWR_BUCK_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };

    // Skip ctrl read (#0), fail vout read (#1)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetBuckCfg when the vmonThr ioRxByte_CS inside.
 */
void test_neg_power_pwrGetBuckCfg_vmonThrReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID | PMIC_CFG_PWR_BUCK_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };

    // Skip ctrl read (#0), fail pgWindow read (#1)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetBuckCfg when the grpSel ioRxByte_CS inside.
 */
void test_neg_power_pwrGetBuckCfg_grpSelReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID | PMIC_CFG_PWR_BUCK_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1
    };

    // Skip ctrl read (#0), fail grpSel read (#1)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Static helper coverage: PWR_setLdoCfg / PWR_getLdoCfg I/O failures
/* ========================================================================== */

/**
 * @brief Test Pmic_pwrSetLdoCfg when the initial ioRxByte inside the ctrl.
 */
void test_neg_power_pwrSetLdoCfg_ctrlReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .ldoEn = true
    };

    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrSetLdoCfg when ioRxByte inside the VOUT block of.
 */
void test_neg_power_pwrSetLdoCfg_voutReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .vset = PMIC_POWER_LDO1_VSET_MIN
    };

    // First I/O is ioRxByte(voutRegAddr)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrSetLdoCfg when ioRxByte inside the VMON threshold.
 */
void test_neg_power_pwrSetLdoCfg_vmonThrReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .vmonThr = PMIC_POWER_VMON_THR_MAX
    };

    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrSetLdoCfg when ioRxByte inside the group select block.
 */
void test_neg_power_pwrSetLdoCfg_grpSelReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .grpSel = PMIC_POWER_GRP_SEL_MAX
    };

    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetLdoCfg when the vout ioRxByte_CS inside.
 */
void test_neg_power_pwrGetLdoCfg_voutReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_EN_VALID | PMIC_CFG_PWR_LDO_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };

    // Skip ctrl read (#0), fail vout read (#1)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetLdoCfg when the vmonThr ioRxByte_CS inside.
 */
void test_neg_power_pwrGetLdoCfg_vmonThrReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_EN_VALID | PMIC_CFG_PWR_LDO_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };

    // Skip ctrl read (#0), fail pgWindow read (#1)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetLdoCfg when the grpSel ioRxByte_CS inside.
 */
void test_neg_power_pwrGetLdoCfg_grpSelReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_EN_VALID | PMIC_CFG_PWR_LDO_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };

    // Skip ctrl read (#0), fail grpSel read (#1)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Static helper coverage: PWR_setVccaVmonCfg / PWR_getVccaVmonCfg failures
/* ========================================================================== */

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg when ioRxByte inside the PG set/threshold.
 */
void test_neg_power_pwrSetVccaVmonCfg_pgWindowReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .vmonThr = PMIC_POWER_VCCA_VMON_THR_MAX
    };

    // First I/O is ioRxByte(VCCA_PG_WINDOW_REG)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg when ioRxByte inside the group select.
 */
void test_neg_power_pwrSetVccaVmonCfg_grpSelReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .grpSel = PMIC_POWER_GRP_SEL_MAX
    };

    // First I/O is ioRxByte(grpSelRegAddr)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg when the pgWindow ioRxByte_CS inside.
 */
void test_neg_power_pwrGetVccaVmonCfg_pgWindowReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID | PMIC_CFG_PWR_VCCA_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };

    // Skip vmonEn read (#0), fail pgWindow read (#1)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg when the grpSel ioRxByte_CS inside.
 */
void test_neg_power_pwrGetVccaVmonCfg_grpSelReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID | PMIC_CFG_PWR_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };

    // Skip vmonEn read (#0), fail grpSel read (#1)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Negative Tests - Pmic_PwrSetBuckCfg / Pmic_PwrSetVccaVmonCfg
/* ========================================================================== */

void test_neg_power_pwrSetBuckCfg_isBuckBelowMin(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_EN_VALID,
        .resource = 0U,
        .buckEn = true
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetVccaVmonCfg_isVmonAboveMax(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID,
        .resource = (uint16_t)(PMIC_POWER_RESOURCE_VMON_MAX + 1U),
        .vmonEn = true
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_pos_power_getLdoCfg_vmonEnOnlyValid(void)
{
    int32_t status;
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_getLdoCfg_dischargeEnOnlyValid(void)
{
    int32_t status;
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_DISCHARGE_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_power_setBuckCfg_vsetAboveMaxBuck1(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_CFG_PWR_BUCK_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .vset = (uint8_t)(PMIC_POWER_BUCK1_VSET_MAX + 1U)
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_pos_power_setLdoCfg_vmonEnOnly(void)
{
    int32_t status;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .vmonEn = true
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_CFG_PWR_LDO_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.vmonEn == true);
}

void test_pos_power_setLdoCfg_dischargeEnOnly(void)
{
    int32_t status;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_CFG_PWR_LDO_DISCHARGE_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .dischargeEn = true
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_CFG_PWR_LDO_DISCHARGE_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1
    };
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.dischargeEn == true);
}

void test_neg_power_setLdoCfg_invalidVsetLdo2(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2,
        .vset = (uint8_t)(PMIC_POWER_LDO2_3_VSET_MAX + 1U)
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetVccaVmonCfg_vmonEnReadFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .vmonEn = true
    };

    // Fail the very first I/O (ioRxByte for VCCA_VMON_CTRL_REG)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

void test_neg_power_setVccaVmonCfg_pgSetAboveMaxVmon2(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2,
        .pgSet = (uint8_t)(PMIC_POWER_VMON2_PG_SET_MAX + 1U)
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_pos_power_setVccaVmonCfg_vccaPgSetOnly(void)
{
    int32_t status;
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .pgSet = 0U  // Valid VCCA PG set code (0 == 5.0 V enum)
    };
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };
    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vccaVmonCfgGet.pgSet == vccaVmonCfgSet.pgSet);
}

/**
 * @brief Test PWR_setLdoCfg — vset above max for LDO2/3.
 */
void test_neg_power_setLdoCfg_vsetAboveMaxLdo2(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDev = platform_getMockDevice();
    PLATFORM_ASSERT(mockDev != NULL);

    /* validParams = VSET_VALID, resource = LDO2, vset one above max.
     * I/O sequence: ioRxByte(voutRegAddr) at skipCount=0 succeeds (no injection),
     * then the vset range check fires and returns PMIC_ST_ERR_INV_PARAM before
     * any ioTxByte is called.  No error injection needed — the param check alone
     * causes the failure. */
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_CFG_PWR_LDO_VSET_VALID,
        .resource    = PMIC_POWER_RESOURCE_LDO2,
        .vset        = (uint8_t)(PMIC_POWER_LDO2_3_VSET_MAX + 1U)
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setVccaVmonCfg — THR_VALID only, no PG_SET_VALID.
 */
void test_neg_power_setVccaVmonCfg_thrValidNoPgSet(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDev = platform_getMockDevice();
    PLATFORM_ASSERT(mockDev != NULL);

    /* skipCount=0: the very first I/O (ioRxByte on pgWindowRegAddr) fails.
     * With no EN_VALID the vmonEn block is skipped entirely, so the first I/O
     * in this call is the pgWindow read inside the PG/THR block. */
    int32_t status = PmicMock_InjectErrorAfterN(mockDev, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_THR_VALID,
        .resource    = PMIC_POWER_RESOURCE_VCCA_VMON,
        .vmonThr     = PMIC_POWER_VCCA_VMON_THR_MAX
    };
    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

void test_neg_power_pwrGetVccaVmonCfg_pgLevelReadFailVmon1(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1
    };

    // Fail the very first I/O (ioRxByte_CS for pgLevelRegAddr)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test PWR_setLdoVoutReg — in-range vset for LDO1.
 *
 * Gap: pmic_power.c PWR_setLdoVoutReg line 662
 * ((ldoCfg->vset < vsetMin) || (ldoCfg->vset > vsetMax)).
 * Existing negative tests (test_neg_power_setLdoCfg_invalidVsetLdo1,
 * ..._invalidVsetLdo2, ..._vsetAboveMaxLdo2) all drive vset above
 * PMIC_POWER_LDO1_VSET_MAX / PMIC_POWER_LDO2_3_VSET_MAX, and the only
 * existing in-range/success test (test_pos_power_ldo2_vset) exercises the
 * false side of the condition, but only for LDO2/3 — never for LDO1. This
 * test drives an in-range vset for LDO1 so the false (in-range) side of the
 * branch is proven reachable for the ldoId == LDO1 path as well, reaching
 * the Pmic_setBitField() write at line 668.
 */
void test_pos_power_setLdoCfg_vsetInRangeLdo1(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_CFG_PWR_LDO_VSET_VALID,
        .resource    = PMIC_POWER_RESOURCE_LDO1,
        .vset        = 0x20U  /* within [PMIC_POWER_LDO1_VSET_MIN, PMIC_POWER_LDO1_VSET_MAX] */
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_CFG_PWR_LDO_VSET_VALID,
        .resource    = PMIC_POWER_RESOURCE_LDO1
    };
    int32_t status;

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.vset == 0x20U);
}

/**
 * @brief Test PWR_setVccaVmonPgWindow — VMON1 with THR_VALID and
 * PG_SET_VALID both set.
 *
 * Gap: pmic_power.c PWR_setVccaVmonPgWindow line 1033
 * (isVcca && PG_SET_VALID) term of the pgWindow write-back condition.
 * Existing positive tests only ever set THR_VALID or PG_SET_VALID alone:
 *   - test_pos_power_vcca_pgSet / vccaPgSetOnly: isVcca=true, PG_SET_VALID
 *     only -> (isVcca && PG_SET_VALID) evaluates true via the isVcca&&PG_SET
 *     term itself.
 *   - test_pos_power_vcca_threshold: isVcca=true, THR_VALID only -> overall
 *     OR is true via THR_VALID, isVcca&&PG_SET_VALID short-circuits false
 *     because PG_SET_VALID is false.
 *   - test_pos_power_vmon1_pgSet / vmon2_pgSet: isVcca=false, PG_SET_VALID
 *     only -> isVcca&&PG_SET_VALID short-circuits false because isVcca is
 *     false; PG_SET_VALID operand is never actually evaluated.
 * None of the existing tests set THR_VALID and PG_SET_VALID together for a
 * VMON resource (isVcca=false). That combination is the only way to reach
 * line 1033 with PG_SET_VALID true AND isVcca false while the surrounding
 * OR is still true (via THR_VALID) — i.e. the only way to prove that with
 * isVcca=false, the (isVcca && PG_SET_VALID) sub-term still contributes
 * "false" to the OR even when PG_SET_VALID is true, without that fact being
 * masked by the whole condition being false. This test sets both
 * PMIC_CFG_PWR_VCCA_VMON_THR_VALID and PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID
 * for VMON1, forcing evaluation of both operands of the isVcca && PG_SET_VALID
 * term while isVcca is false, and verifies both fields round-trip correctly
 * (pgSet is actually written via the separate PWR_setVmonPgSetReg() path to
 * pgLevelRegAddr, while vmonThr is written via the pgWindowRegAddr
 * write-back gated by line 1031-1033).
 */
void test_pos_power_setVccaVmonCfg_vmon1_thrAndPgSet(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_THR_VALID | PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource    = PMIC_POWER_RESOURCE_VMON1,
        .vmonThr     = PMIC_POWER_VMON_THR_MAX,
        .pgSet       = PMIC_POWER_VMON1_PG_SET_MIN
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_CFG_PWR_VCCA_VMON_THR_VALID | PMIC_CFG_PWR_VCCA_VMON_PG_SET_VALID,
        .resource    = PMIC_POWER_RESOURCE_VMON1
    };
    int32_t status;

    status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vmonCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vmonCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(vmonCfgGet.vmonThr == PMIC_POWER_VMON_THR_MAX);
    PLATFORM_ASSERT(vmonCfgGet.pgSet == PMIC_POWER_VMON1_PG_SET_MIN);
}

/* ========================================================================== */
/*                         Entry Point Function                               */
/* ========================================================================== */

void power_test(void *args)
{
    (void)args;
    int32_t status;

    platform_init();
    testTimer_startModule("Power");
    platform_setupTests();

    /* Initialize PMIC handle */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_CFG_INIT_COMM_MODE_VALID |
                       PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                       PMIC_CFG_INIT_IO_READ_VALID |
                       PMIC_CFG_INIT_IO_WRITE_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .commHandle0 = platform_getCommHandle0(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    status = Pmic_init(&pmicHandle, &handleCfg);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to initialize PMIC handle\r\n");
        platform_tearDownTests();
        platform_deinit();
        return;
    }

    platform_printString("\r\n=== Power Module Tests ===\r\n");
    POWER_TEST_RUN_ALL();

    testTimer_endModule();
    Pmic_deinit(&pmicHandle);
    platform_tearDownTests();
    platform_deinit();
}
