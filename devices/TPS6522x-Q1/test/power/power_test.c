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


#include "../platform.h"
#include "power_test.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0};

/* ========================================================================== */
/*                       Negative Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test Pmic_pwrSetBuckCfg with NULL handle
 */
static void test_pwr_setBuckCfg_nullHandle(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = PMIC_POWER_BUCK_EN_VALID, .resource = PMIC_POWER_RESOURCE_BUCK1, .buckEn = true};
    int32_t status = Pmic_pwrSetBuckCfg(NULL, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with NULL buckCfg parameter
 */
static void test_pwr_setBuckCfg_nullBuckCfg(void)
{
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid validParams
 */
static void test_pwr_setBuckCfg_invalidValidParams(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_BUCK1};
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid resource
 */
static void test_pwr_setBuckCfg_invalidResource(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = PMIC_POWER_BUCK_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO1, .buckEn = true};
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid slew rate
 */
static void test_pwr_setBuckCfg_invalidSlewRate(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_POWER_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .slewRate = PMIC_POWER_BUCK_SLEW_RATE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid vset for BUCK1
 */
static void test_pwr_setBuckCfg_invalidVsetBuck1(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_POWER_BUCK_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .vset = PMIC_POWER_BUCK1_VSET_MIN - 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid vmon threshold
 */
static void test_pwr_setBuckCfg_invalidVmonThr(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_POWER_BUCK_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK2,
        .vmonThr = PMIC_POWER_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetBuckCfg with invalid group select
 */
static void test_pwr_setBuckCfg_invalidGrpSel(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_POWER_BUCK_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK3,
        .grpSel = PMIC_POWER_GRP_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetBuckCfg with NULL handle
 */
static void test_pwr_getBuckCfg_nullHandle(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = PMIC_POWER_BUCK_EN_VALID, .resource = PMIC_POWER_RESOURCE_BUCK1};
    int32_t status = Pmic_pwrGetBuckCfg(NULL, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetBuckCfg with NULL buckCfg parameter
 */
static void test_pwr_getBuckCfg_nullBuckCfg(void)
{
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetBuckCfg with invalid validParams
 */
static void test_pwr_getBuckCfg_invalidValidParams(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_BUCK1};
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetBuckCfg with invalid resource
 */
static void test_pwr_getBuckCfg_invalidResource(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = PMIC_POWER_BUCK_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO2};
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with NULL handle
 */
static void test_pwr_setLdoCfg_nullHandle(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_POWER_LDO_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO1, .ldoEn = true};
    int32_t status = Pmic_pwrSetLdoCfg(NULL, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with NULL ldoCfg parameter
 */
static void test_pwr_setLdoCfg_nullLdoCfg(void)
{
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid validParams
 */
static void test_pwr_setLdoCfg_invalidValidParams(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_LDO1};
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid resource
 */
static void test_pwr_setLdoCfg_invalidResource(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_POWER_LDO_EN_VALID, .resource = PMIC_POWER_RESOURCE_BUCK1, .ldoEn = true};
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid vset for LDO1
 */
static void test_pwr_setLdoCfg_invalidVsetLdo1(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_POWER_LDO_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .vset = PMIC_POWER_LDO1_VSET_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid vmon threshold
 */
static void test_pwr_setLdoCfg_invalidVmonThr(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_POWER_LDO_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2,
        .vmonThr = PMIC_POWER_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetLdoCfg with invalid group select
 */
static void test_pwr_setLdoCfg_invalidGrpSel(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_POWER_LDO_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO3,
        .grpSel = PMIC_POWER_GRP_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetLdoCfg with NULL handle
 */
static void test_pwr_getLdoCfg_nullHandle(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_POWER_LDO_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO1};
    int32_t status = Pmic_pwrGetLdoCfg(NULL, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetLdoCfg with NULL ldoCfg parameter
 */
static void test_pwr_getLdoCfg_nullLdoCfg(void)
{
    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetLdoCfg with invalid validParams
 */
static void test_pwr_getLdoCfg_invalidValidParams(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_LDO2};
    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetLdoCfg with invalid resource
 */
static void test_pwr_getLdoCfg_invalidResource(void)
{
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_POWER_LDO_EN_VALID, .resource = PMIC_POWER_RESOURCE_VMON1};
    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with NULL handle
 */
static void test_pwr_setVccaVmonCfg_nullHandle(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = PMIC_POWER_VCCA_VMON_EN_VALID, .resource = PMIC_POWER_RESOURCE_VCCA_VMON, .vmonEn = true};
    int32_t status = Pmic_pwrSetVccaVmonCfg(NULL, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with NULL vccaVmonCfg parameter
 */
static void test_pwr_setVccaVmonCfg_nullVccaVmonCfg(void)
{
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid validParams
 */
static void test_pwr_setVccaVmonCfg_invalidValidParams(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_VMON1};
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid resource
 */
static void test_pwr_setVccaVmonCfg_invalidResource(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = PMIC_POWER_VCCA_VMON_EN_VALID, .resource = PMIC_POWER_RESOURCE_BUCK1, .vmonEn = true};
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid pgSet for VCCA
 */
static void test_pwr_setVccaVmonCfg_invalidPgSetVcca(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_POWER_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .pgSet = PMIC_POWER_VCCA_VMON_PG_SET_MAX + 1U
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid threshold for VCCA
 */
static void test_pwr_setVccaVmonCfg_invalidThrVcca(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_POWER_VCCA_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .vmonThr = PMIC_POWER_VCCA_VMON_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetVccaVmonCfg with invalid group select
 */
static void test_pwr_setVccaVmonCfg_invalidGrpSel(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_POWER_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2,
        .grpSel = PMIC_POWER_GRP_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg with NULL handle
 */
static void test_pwr_getVccaVmonCfg_nullHandle(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = PMIC_POWER_VCCA_VMON_EN_VALID, .resource = PMIC_POWER_RESOURCE_VCCA_VMON};
    int32_t status = Pmic_pwrGetVccaVmonCfg(NULL, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg with NULL vccaVmonCfg parameter
 */
static void test_pwr_getVccaVmonCfg_nullVccaVmonCfg(void)
{
    int32_t status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg with invalid validParams
 */
static void test_pwr_getVccaVmonCfg_invalidValidParams(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = 0U, .resource = PMIC_POWER_RESOURCE_VMON1};
    int32_t status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetVccaVmonCfg with invalid resource
 */
static void test_pwr_getVccaVmonCfg_invalidResource(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {.validParams = PMIC_POWER_VCCA_VMON_EN_VALID, .resource = PMIC_POWER_RESOURCE_LDO1};
    int32_t status = Pmic_pwrGetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetGlobalVmonDegl with NULL handle
 */
static void test_pwr_setGlobalVmonDegl_nullHandle(void)
{
    int32_t status = Pmic_pwrSetGlobalVmonDegl(NULL, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetGlobalVmonDegl with invalid value
 */
static void test_pwr_setGlobalVmonDegl_invalidValue(void)
{
    int32_t status = Pmic_pwrSetGlobalVmonDegl(&pmicHandle, PMIC_POWER_VMON_DEGL_SEL_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with NULL handle
 */
static void test_pwr_setThermalCfg_nullHandle(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = PMIC_POWER_TWARN_LEVEL_VALID, .twarnLvl = PMIC_POWER_TWARN_LEVEL_130C};
    int32_t status = Pmic_pwrSetThermalCfg(NULL, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with NULL thermalCfg parameter
 */
static void test_pwr_setThermalCfg_nullThermalCfg(void)
{
    int32_t status = Pmic_pwrSetThermalCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with invalid validParams
 */
static void test_pwr_setThermalCfg_invalidValidParams(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = 0U};
    int32_t status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with invalid twarnLvl
 */
static void test_pwr_setThermalCfg_invalidTwarnLvl(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = PMIC_POWER_TWARN_LEVEL_VALID, .twarnLvl = PMIC_POWER_TWARN_LEVEL_MAX + 1U};
    int32_t status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetThermalCfg with invalid tsdOrdLvl
 */
static void test_pwr_setThermalCfg_invalidTsdOrdLvl(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = PMIC_POWER_TSD_ORD_LEVEL_VALID, .tsdOrdLvl = PMIC_POWER_TSD_ORD_LEVEL_MAX + 1U};
    int32_t status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetThermalCfg with NULL handle
 */
static void test_pwr_getThermalCfg_nullHandle(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = PMIC_POWER_TWARN_LEVEL_VALID};
    int32_t status = Pmic_pwrGetThermalCfg(NULL, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetThermalCfg with NULL thermalCfg parameter
 */
static void test_pwr_getThermalCfg_nullThermalCfg(void)
{
    int32_t status = Pmic_pwrGetThermalCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetThermalCfg with invalid validParams
 */
static void test_pwr_getThermalCfg_invalidValidParams(void)
{
    Pmic_PwrThermalCfg_t thermalCfg = {.validParams = 0U};
    int32_t status = Pmic_pwrGetThermalCfg(&pmicHandle, &thermalCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetSpreadSpectrumCfg with NULL handle
 */
static void test_pwr_setSpreadSpectrumCfg_nullHandle(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {.validParams = PMIC_POWER_SS_EN_VALID, .ssEn = true};
    int32_t status = Pmic_pwrSetSpreadSpectrumCfg(NULL, &ssCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetSpreadSpectrumCfg with NULL ssCfg parameter
 */
static void test_pwr_setSpreadSpectrumCfg_nullSsCfg(void)
{
    int32_t status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrSetSpreadSpectrumCfg with invalid validParams
 */
static void test_pwr_setSpreadSpectrumCfg_invalidValidParams(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {.validParams = 0U};
    int32_t status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, &ssCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetSpreadSpectrumCfg with NULL handle
 */
static void test_pwr_getSpreadSpectrumCfg_nullHandle(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {.validParams = PMIC_POWER_SS_EN_VALID};
    int32_t status = Pmic_pwrGetSpreadSpectrumCfg(NULL, &ssCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetSpreadSpectrumCfg with NULL ssCfg parameter
 */
static void test_pwr_getSpreadSpectrumCfg_nullSsCfg(void)
{
    int32_t status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetSpreadSpectrumCfg with invalid validParams
 */
static void test_pwr_getSpreadSpectrumCfg_invalidValidParams(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfg = {.validParams = 0U};
    int32_t status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, &ssCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetRsrcStatus with NULL handle
 */
static void test_pwr_getRsrcStatus_nullHandle(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {.validParams = PMIC_POWER_BUCK1_UVOV_VALID};
    int32_t status = Pmic_pwrGetRsrcStatus(NULL, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetRsrcStatus with NULL rsrcStatus parameter
 */
static void test_pwr_getRsrcStatus_nullRsrcStatus(void)
{
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_pwrGetRsrcStatus with invalid validParams
 */
static void test_pwr_getRsrcStatus_invalidValidParams(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {.validParams = 0U};
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                       Positive Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test BUCK1 enable and disable
 */
static void test_pwr_buck1_enableDisable(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_POWER_BUCK_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .buckEn = false
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_POWER_BUCK_EN_VALID,
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
 * @brief Test BUCK2 voltage set and get
 */
static void test_pwr_buck2_vset(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_POWER_BUCK_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK2,
        .vset = 0x20U
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_POWER_BUCK_VSET_VALID,
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
 * @brief Test BUCK3 slew rate configuration
 */
static void test_pwr_buck3_slewRate(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_POWER_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK3,
        .slewRate = PMIC_POWER_BUCK_SLEW_RATE_10_MV_PER_US
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_POWER_BUCK_SLEW_RATE_VALID,
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
 * @brief Test BUCK4 VMON threshold
 */
static void test_pwr_buck4_vmonThr(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_POWER_BUCK_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK4,
        .vmonThr = PMIC_POWER_VMON_THR_3_PCT_30_MV
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_POWER_BUCK_VMON_THR_VALID,
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
 * @brief Test BUCK1 group select
 */
static void test_pwr_buck1_grpSel(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_POWER_BUCK_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .grpSel = PMIC_POWER_GRP_SEL_MCU
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_POWER_BUCK_GRP_SEL_VALID,
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
 * @brief Test LDO1 enable and disable
 */
static void test_pwr_ldo1_enableDisable(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_POWER_LDO_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .ldoEn = false
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_POWER_LDO_EN_VALID,
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
 * @brief Test LDO2 voltage set and get
 */
static void test_pwr_ldo2_vset(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_POWER_LDO_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2,
        .vset = 0x15U
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_POWER_LDO_VSET_VALID,
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
 * @brief Test LDO3 bypass enable
 */
static void test_pwr_ldo3_bypassEn(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_POWER_LDO_BYP_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO3,
        .bypEn = true
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_POWER_LDO_BYP_EN_VALID,
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
 * @brief Test LDO1 VMON threshold
 */
static void test_pwr_ldo1_vmonThr(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_POWER_LDO_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO1,
        .vmonThr = PMIC_POWER_VMON_THR_4_PCT_40_MV
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_POWER_LDO_VMON_THR_VALID,
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
 * @brief Test LDO2 group select
 */
static void test_pwr_ldo2_grpSel(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_POWER_LDO_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2,
        .grpSel = PMIC_POWER_GRP_SEL_SOC
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_POWER_LDO_GRP_SEL_VALID,
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
 * @brief Test VCCA enable and disable
 */
static void test_pwr_vcca_enableDisable(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .vmonEn = false
    };
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgGet = {
        .validParams = PMIC_POWER_VCCA_VMON_EN_VALID,
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
 * @brief Test VCCA PG set configuration
 */
static void test_pwr_vcca_pgSet(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .pgSet = PMIC_POWER_VCCA_VMON_PG_SET_3P3_V
    };
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgGet = {
        .validParams = PMIC_POWER_VCCA_VMON_PG_SET_VALID,
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
 * @brief Test VCCA threshold configuration
 */
static void test_pwr_vcca_threshold(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .vmonThr = PMIC_POWER_VCCA_VMON_THR_3_PCT
    };
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgGet = {
        .validParams = PMIC_POWER_VCCA_VMON_THR_VALID,
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
 * @brief Test VCCA group select
 */
static void test_pwr_vcca_grpSel(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON,
        .grpSel = PMIC_POWER_GRP_SEL_OTHER
    };
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgGet = {
        .validParams = PMIC_POWER_VCCA_VMON_GRP_SEL_VALID,
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
 * @brief Test VMON1 enable and disable
 */
static void test_pwr_vmon1_enableDisable(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1,
        .vmonEn = false
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_POWER_VCCA_VMON_EN_VALID,
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
 * @brief Test VMON2 PG set configuration
 */
static void test_pwr_vmon2_pgSet(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2,
        .pgSet = 0x30U
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_POWER_VCCA_VMON_PG_SET_VALID,
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
 * @brief Test VMON1 PG set configuration
 */
static void test_pwr_vmon1_pgSet(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1,
        .pgSet = 0x20U
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_POWER_VCCA_VMON_PG_SET_VALID,
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
 * @brief Test VMON2 enable configuration
 */
static void test_pwr_vmon2_enableDisable(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON2,
        .vmonEn = true
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_POWER_VCCA_VMON_EN_VALID,
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
 * @brief Test VMON1 invalid PG set configuration
 */
static void test_pwr_setVccaVmonCfg_invalidPgSetVmon1(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfg = {
        .validParams = PMIC_POWER_VCCA_VMON_PG_SET_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1,
        .pgSet = 0x09U  // Below PMIC_POWER_VMON1_PG_SET_MIN (0x0A)
    };
    int32_t status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test global VMON deglitch setting for all valid values
 */
static void test_pwr_globalVmonDegl_allValues(void)
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
 * @brief Test thermal warning level set and get
 */
static void test_pwr_thermal_twarnLvl(void)
{
    Pmic_PwrThermalCfg_t thermalCfgSet = {
        .validParams = PMIC_POWER_TWARN_LEVEL_VALID,
        .twarnLvl = PMIC_POWER_TWARN_LEVEL_130C
    };
    Pmic_PwrThermalCfg_t thermalCfgGet = {
        .validParams = PMIC_POWER_TWARN_LEVEL_VALID
    };
    int32_t status;

    status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetThermalCfg(&pmicHandle, &thermalCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(thermalCfgGet.twarnLvl == PMIC_POWER_TWARN_LEVEL_130C);
}

/**
 * @brief Test thermal orderly shutdown level set and get
 */
static void test_pwr_thermal_tsdOrdLvl(void)
{
    Pmic_PwrThermalCfg_t thermalCfgSet = {
        .validParams = PMIC_POWER_TSD_ORD_LEVEL_VALID,
        .tsdOrdLvl = PMIC_POWER_TSD_ORD_LEVEL_140C
    };
    Pmic_PwrThermalCfg_t thermalCfgGet = {
        .validParams = PMIC_POWER_TSD_ORD_LEVEL_VALID
    };
    int32_t status;

    status = Pmic_pwrSetThermalCfg(&pmicHandle, &thermalCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetThermalCfg(&pmicHandle, &thermalCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(thermalCfgGet.tsdOrdLvl == PMIC_POWER_TSD_ORD_LEVEL_140C);
}

/**
 * @brief Test spread spectrum enable and disable
 */
static void test_pwr_spreadSpectrum_enableDisable(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfgSet = {
        .validParams = PMIC_POWER_SS_EN_VALID,
        .ssEn = false
    };
    Pmic_PwrSpreadSpectrumCfg_t ssCfgGet = {
        .validParams = PMIC_POWER_SS_EN_VALID
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
 * @brief Test spread spectrum depth configuration
 */
static void test_pwr_spreadSpectrum_depth(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfgSet = {
        .validParams = PMIC_POWER_SS_DEPTH_VALID,
        .ssDepth = PMIC_POWER_SS_DEPTH_4_PCT
    };
    Pmic_PwrSpreadSpectrumCfg_t ssCfgGet = {
        .validParams = PMIC_POWER_SS_DEPTH_VALID
    };
    int32_t status;

    status = Pmic_pwrSetSpreadSpectrumCfg(&pmicHandle, &ssCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetSpreadSpectrumCfg(&pmicHandle, &ssCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ssCfgGet.ssDepth == PMIC_POWER_SS_DEPTH_4_PCT);
}

/**
 * @brief Test get all BUCK UVOV status
 */
static void test_pwr_rsrcStatus_buckUVOV(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {
        .validParams = PMIC_POWER_BUCK1_UVOV_VALID | PMIC_POWER_BUCK2_UVOV_VALID |
                       PMIC_POWER_BUCK3_UVOV_VALID | PMIC_POWER_BUCK4_UVOV_VALID
    };
    int32_t status;

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test get all LDO UVOV status
 */
static void test_pwr_rsrcStatus_ldoUVOV(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {
        .validParams = PMIC_POWER_LDO1_UVOV_VALID | PMIC_POWER_LDO2_UVOV_VALID |
                       PMIC_POWER_LDO3_UVOV_VALID
    };
    int32_t status;

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test get all VMON UVOV status
 */
static void test_pwr_rsrcStatus_vmonUVOV(void)
{
    Pmic_PwrRsrcStatus_t rsrcStatus = {
        .validParams = PMIC_POWER_VMON1_UVOV_VALID | PMIC_POWER_VMON2_UVOV_VALID |
                       PMIC_POWER_VCCA_VMON_UVOV_VALID
    };
    int32_t status;

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &rsrcStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test BUCK combined configuration
 */
static void test_pwr_buck_combinedConfig(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_POWER_BUCK_EN_VALID | PMIC_POWER_BUCK_PLDN_EN_VALID |
                       PMIC_POWER_BUCK_VMON_EN_VALID | PMIC_POWER_BUCK_FPWM_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .buckEn = true,
        .pldnEn = true,
        .vmonEn = true,
        .fpwmEn = false
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_POWER_BUCK_EN_VALID | PMIC_POWER_BUCK_PLDN_EN_VALID |
                       PMIC_POWER_BUCK_VMON_EN_VALID | PMIC_POWER_BUCK_FPWM_EN_VALID,
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
 * @brief Test LDO combined configuration
 */
static void test_pwr_ldo_combinedConfig(void)
{
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_POWER_LDO_EN_VALID | PMIC_POWER_LDO_VMON_EN_VALID |
                       PMIC_POWER_LDO_DISCHARGE_EN_VALID,
        .resource = PMIC_POWER_RESOURCE_LDO2,
        .ldoEn = true,
        .vmonEn = true,
        .dischargeEn = true
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_POWER_LDO_EN_VALID | PMIC_POWER_LDO_VMON_EN_VALID |
                       PMIC_POWER_LDO_DISCHARGE_EN_VALID,
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
 * @brief Test VMON combined configuration
 */
static void test_pwr_vmon_combinedConfig(void)
{
    Pmic_PwrVccaVmonCfg_t vmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_EN_VALID | PMIC_POWER_VCCA_VMON_THR_VALID |
                       PMIC_POWER_VCCA_VMON_GRP_SEL_VALID,
        .resource = PMIC_POWER_RESOURCE_VMON1,
        .vmonEn = true,
        .vmonThr = PMIC_POWER_VMON_THR_3_PCT_30_MV,
        .grpSel = PMIC_POWER_GRP_SEL_MCU
    };
    Pmic_PwrVccaVmonCfg_t vmonCfgGet = {
        .validParams = PMIC_POWER_VCCA_VMON_EN_VALID | PMIC_POWER_VCCA_VMON_THR_VALID |
                       PMIC_POWER_VCCA_VMON_GRP_SEL_VALID,
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
 * @brief Test thermal combined configuration
 */
static void test_pwr_thermal_combinedConfig(void)
{
    Pmic_PwrThermalCfg_t thermalCfgSet = {
        .validParams = PMIC_POWER_TWARN_LEVEL_VALID | PMIC_POWER_TSD_ORD_LEVEL_VALID,
        .twarnLvl = PMIC_POWER_TWARN_LEVEL_140C,
        .tsdOrdLvl = PMIC_POWER_TSD_ORD_LEVEL_145C
    };
    Pmic_PwrThermalCfg_t thermalCfgGet = {
        .validParams = PMIC_POWER_TWARN_LEVEL_VALID | PMIC_POWER_TSD_ORD_LEVEL_VALID
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
 * @brief Test spread spectrum combined configuration
 */
static void test_pwr_spreadSpectrum_combinedConfig(void)
{
    Pmic_PwrSpreadSpectrumCfg_t ssCfgSet = {
        .validParams = PMIC_POWER_SS_EN_VALID | PMIC_POWER_SS_DEPTH_VALID,
        .ssEn = true,
        .ssDepth = PMIC_POWER_SS_DEPTH_7_PCT
    };
    Pmic_PwrSpreadSpectrumCfg_t ssCfgGet = {
        .validParams = PMIC_POWER_SS_EN_VALID | PMIC_POWER_SS_DEPTH_VALID
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

#ifdef BUILD_MOCK

/**
 * @brief Property test: BUCK voltage bounds
 */
static void test_pwr_mock_buckVoltageBounds(void)
{
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_POWER_BUCK_VSET_VALID,
        .resource = PMIC_POWER_RESOURCE_BUCK1,
        .vset = PMIC_POWER_BUCK1_VSET_MIN
    };

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    buckCfgSet.vset = PMIC_POWER_BUCK1_VSET_MAX;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Property test: VMON threshold enumeration exhaustion
 */
static void test_pwr_mock_vmonThresholdEnumeration(void)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgSet = {
        .validParams = PMIC_POWER_VCCA_VMON_THR_VALID,
        .resource = PMIC_POWER_RESOURCE_VCCA_VMON
    };
    int32_t status;

    for (uint8_t val = PMIC_POWER_VCCA_VMON_THR_MIN; val <= PMIC_POWER_VCCA_VMON_THR_MAX; val++)
    {
        vccaVmonCfgSet.vmonThr = val;
        status = Pmic_pwrSetVccaVmonCfg(&pmicHandle, &vccaVmonCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

#endif

/* ========================================================================== */
/*                         Test Execution Macros                              */
/* ========================================================================== */

#define PWR_TEST_RUN_NEGATIVE() \
    do { \
        RUN_TEST(test_pwr_setBuckCfg_nullHandle); \
        RUN_TEST(test_pwr_setBuckCfg_nullBuckCfg); \
        RUN_TEST(test_pwr_setBuckCfg_invalidValidParams); \
        RUN_TEST(test_pwr_setBuckCfg_invalidResource); \
        RUN_TEST(test_pwr_setBuckCfg_invalidSlewRate); \
        RUN_TEST(test_pwr_setBuckCfg_invalidVsetBuck1); \
        RUN_TEST(test_pwr_setBuckCfg_invalidVmonThr); \
        RUN_TEST(test_pwr_setBuckCfg_invalidGrpSel); \
        RUN_TEST(test_pwr_getBuckCfg_nullHandle); \
        RUN_TEST(test_pwr_getBuckCfg_nullBuckCfg); \
        RUN_TEST(test_pwr_getBuckCfg_invalidValidParams); \
        RUN_TEST(test_pwr_getBuckCfg_invalidResource); \
        RUN_TEST(test_pwr_setLdoCfg_nullHandle); \
        RUN_TEST(test_pwr_setLdoCfg_nullLdoCfg); \
        RUN_TEST(test_pwr_setLdoCfg_invalidValidParams); \
        RUN_TEST(test_pwr_setLdoCfg_invalidResource); \
        RUN_TEST(test_pwr_setLdoCfg_invalidVsetLdo1); \
        RUN_TEST(test_pwr_setLdoCfg_invalidVmonThr); \
        RUN_TEST(test_pwr_setLdoCfg_invalidGrpSel); \
        RUN_TEST(test_pwr_getLdoCfg_nullHandle); \
        RUN_TEST(test_pwr_getLdoCfg_nullLdoCfg); \
        RUN_TEST(test_pwr_getLdoCfg_invalidValidParams); \
        RUN_TEST(test_pwr_getLdoCfg_invalidResource); \
        RUN_TEST(test_pwr_setVccaVmonCfg_nullHandle); \
        RUN_TEST(test_pwr_setVccaVmonCfg_nullVccaVmonCfg); \
        RUN_TEST(test_pwr_setVccaVmonCfg_invalidValidParams); \
        RUN_TEST(test_pwr_setVccaVmonCfg_invalidResource); \
        RUN_TEST(test_pwr_setVccaVmonCfg_invalidPgSetVcca); \
        RUN_TEST(test_pwr_setVccaVmonCfg_invalidPgSetVmon1); \
        RUN_TEST(test_pwr_setVccaVmonCfg_invalidThrVcca); \
        RUN_TEST(test_pwr_setVccaVmonCfg_invalidGrpSel); \
        RUN_TEST(test_pwr_getVccaVmonCfg_nullHandle); \
        RUN_TEST(test_pwr_getVccaVmonCfg_nullVccaVmonCfg); \
        RUN_TEST(test_pwr_getVccaVmonCfg_invalidValidParams); \
        RUN_TEST(test_pwr_getVccaVmonCfg_invalidResource); \
        RUN_TEST(test_pwr_setGlobalVmonDegl_nullHandle); \
        RUN_TEST(test_pwr_setGlobalVmonDegl_invalidValue); \
        RUN_TEST(test_pwr_setThermalCfg_nullHandle); \
        RUN_TEST(test_pwr_setThermalCfg_nullThermalCfg); \
        RUN_TEST(test_pwr_setThermalCfg_invalidValidParams); \
        RUN_TEST(test_pwr_setThermalCfg_invalidTwarnLvl); \
        RUN_TEST(test_pwr_setThermalCfg_invalidTsdOrdLvl); \
        RUN_TEST(test_pwr_getThermalCfg_nullHandle); \
        RUN_TEST(test_pwr_getThermalCfg_nullThermalCfg); \
        RUN_TEST(test_pwr_getThermalCfg_invalidValidParams); \
        RUN_TEST(test_pwr_setSpreadSpectrumCfg_nullHandle); \
        RUN_TEST(test_pwr_setSpreadSpectrumCfg_nullSsCfg); \
        RUN_TEST(test_pwr_setSpreadSpectrumCfg_invalidValidParams); \
        RUN_TEST(test_pwr_getSpreadSpectrumCfg_nullHandle); \
        RUN_TEST(test_pwr_getSpreadSpectrumCfg_nullSsCfg); \
        RUN_TEST(test_pwr_getSpreadSpectrumCfg_invalidValidParams); \
        RUN_TEST(test_pwr_getRsrcStatus_nullHandle); \
        RUN_TEST(test_pwr_getRsrcStatus_nullRsrcStatus); \
        RUN_TEST(test_pwr_getRsrcStatus_invalidValidParams); \
    } while(0)

#define PWR_TEST_RUN_POSITIVE() \
    do { \
        RUN_TEST(test_pwr_buck1_enableDisable); \
        RUN_TEST(test_pwr_buck2_vset); \
        RUN_TEST(test_pwr_buck3_slewRate); \
        RUN_TEST(test_pwr_buck4_vmonThr); \
        RUN_TEST(test_pwr_buck1_grpSel); \
        RUN_TEST(test_pwr_ldo1_enableDisable); \
        RUN_TEST(test_pwr_ldo2_vset); \
        RUN_TEST(test_pwr_ldo3_bypassEn); \
        RUN_TEST(test_pwr_ldo1_vmonThr); \
        RUN_TEST(test_pwr_ldo2_grpSel); \
        RUN_TEST(test_pwr_vcca_enableDisable); \
        RUN_TEST(test_pwr_vcca_pgSet); \
        RUN_TEST(test_pwr_vcca_threshold); \
        RUN_TEST(test_pwr_vcca_grpSel); \
        RUN_TEST(test_pwr_vmon1_enableDisable); \
        RUN_TEST(test_pwr_vmon1_pgSet); \
        RUN_TEST(test_pwr_vmon2_pgSet); \
        RUN_TEST(test_pwr_vmon2_enableDisable); \
        RUN_TEST(test_pwr_globalVmonDegl_allValues); \
        RUN_TEST(test_pwr_thermal_twarnLvl); \
        RUN_TEST(test_pwr_thermal_tsdOrdLvl); \
        RUN_TEST(test_pwr_spreadSpectrum_enableDisable); \
        RUN_TEST(test_pwr_spreadSpectrum_depth); \
        RUN_TEST(test_pwr_rsrcStatus_buckUVOV); \
        RUN_TEST(test_pwr_rsrcStatus_ldoUVOV); \
        RUN_TEST(test_pwr_rsrcStatus_vmonUVOV); \
        RUN_TEST(test_pwr_buck_combinedConfig); \
        RUN_TEST(test_pwr_ldo_combinedConfig); \
        RUN_TEST(test_pwr_vmon_combinedConfig); \
        RUN_TEST(test_pwr_thermal_combinedConfig); \
        RUN_TEST(test_pwr_spreadSpectrum_combinedConfig); \
    } while(0)

#ifdef BUILD_MOCK
#define PWR_TEST_RUN_MOCK() \
    do { \
        RUN_TEST(test_pwr_mock_buckVoltageBounds); \
        RUN_TEST(test_pwr_mock_vmonThresholdEnumeration); \
    } while(0)
#else
#define PWR_TEST_RUN_MOCK()
#endif

#define PWR_TEST_RUN_ALL() \
    do { \
        PWR_TEST_RUN_NEGATIVE(); \
        PWR_TEST_RUN_POSITIVE(); \
        PWR_TEST_RUN_MOCK(); \
    } while(0)

/* ========================================================================== */
/*                         Entry Point Function                               */
/* ========================================================================== */

void power_test(void *args)
{
    (void)args;
    int32_t status;

    platform_init();
    platform_setupTests();

    /* Initialize PMIC handle */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = platform_rxByte,
        .ioWrite = platform_txByte,
        .criticalSectionStart = platform_critSecStart,
        .criticalSectionStop = platform_critSecStop
    };

    status = Pmic_init(&pmicHandle, &handleCfg);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to initialize PMIC handle\r\n");
        platform_tearDownTests();
        platform_deinit();
        return;
    }

    /* Unlock registers for power configuration */
    status = Pmic_setRegLockState(&pmicHandle, false);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to unlock registers\r\n");
        Pmic_deinit(&pmicHandle);
        platform_tearDownTests();
        platform_deinit();
        return;
    }

    platform_printString("\r\n=== Power Module Tests ===\r\n");
    PWR_TEST_RUN_ALL();

    Pmic_deinit(&pmicHandle);
    platform_tearDownTests();
    platform_deinit();
}
