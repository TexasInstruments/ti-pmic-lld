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
#ifndef POWER_TEST_H
#define POWER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                 Test APIs: pwrSetBuckCfg, pwrGetBuckCfg                  */
/* ======================================================================== */

#define POWER_TEST_NEG_SETGETBUCKCFG() \
    PLATFORM_RUN_TEST(test_neg_power_setBuckCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_setBuckCfg_nullBuckCfg); \
    PLATFORM_RUN_TEST(test_neg_power_setBuckCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_setBuckCfg_invalidResource); \
    PLATFORM_RUN_TEST(test_neg_power_setBuckCfg_invalidSlewRate); \
    PLATFORM_RUN_TEST(test_neg_power_setBuckCfg_invalidVsetBuck1); \
    PLATFORM_RUN_TEST(test_neg_power_setBuckCfg_invalidVmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_setBuckCfg_invalidGrpSel); \
    PLATFORM_RUN_TEST(test_neg_power_getBuckCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_getBuckCfg_nullBuckCfg); \
    PLATFORM_RUN_TEST(test_neg_power_getBuckCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_getBuckCfg_invalidResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_isBuckBelowMin); \
    PLATFORM_RUN_TEST(test_neg_power_setBuckCfg_vsetAboveMaxBuck1); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_ctrlReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_slewRateReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_vmonThrReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_grpSelReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_slewRateReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_vsetReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_vmonThrReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_grpSelReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_ioRxByteFail)

#define POWER_TEST_POS_SETGETBUCKCFG() \
    PLATFORM_RUN_TEST(test_pos_power_buck1_enableDisable); \
    PLATFORM_RUN_TEST(test_pos_power_buck2_vset); \
    PLATFORM_RUN_TEST(test_pos_power_buck3_slewRate); \
    PLATFORM_RUN_TEST(test_pos_power_buck4_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_buck1_grpSel); \
    PLATFORM_RUN_TEST(test_pos_power_buck_pldnEn); \
    PLATFORM_RUN_TEST(test_pos_power_buck_vmonEn); \
    PLATFORM_RUN_TEST(test_pos_power_buck_fpwmEn); \
    PLATFORM_RUN_TEST(test_pos_power_buck_combinedConfig)

/* Test: TC-POWER-0014 */
#define POWER_TEST_SETGETBUCKCFG() \
    POWER_TEST_NEG_SETGETBUCKCFG(); \
    POWER_TEST_POS_SETGETBUCKCFG()

/* ======================================================================== */
/*                  Test APIs: pwrSetLdoCfg, pwrGetLdoCfg                   */
/* ======================================================================== */

#define POWER_TEST_NEG_SETGETLDOCFG() \
    PLATFORM_RUN_TEST(test_neg_power_setLdoCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_setLdoCfg_nullLdoCfg); \
    PLATFORM_RUN_TEST(test_neg_power_setLdoCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_setLdoCfg_invalidResource); \
    PLATFORM_RUN_TEST(test_neg_power_setLdoCfg_invalidVsetLdo1); \
    PLATFORM_RUN_TEST(test_neg_power_setLdoCfg_invalidVmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_setLdoCfg_invalidGrpSel); \
    PLATFORM_RUN_TEST(test_neg_power_getLdoCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_getLdoCfg_nullLdoCfg); \
    PLATFORM_RUN_TEST(test_neg_power_getLdoCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_getLdoCfg_invalidResource); \
    PLATFORM_RUN_TEST(test_neg_power_setLdoCfg_invalidVsetLdo2); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_ctrlReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_voutReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_vmonThrReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_grpSelReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_voutReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_vmonThrReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_grpSelReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_setLdoCfg_vsetAboveMaxLdo2); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_ioRxByteFail)

#define POWER_TEST_POS_SETGETLDOCFG() \
    PLATFORM_RUN_TEST(test_pos_power_ldo1_enableDisable); \
    PLATFORM_RUN_TEST(test_pos_power_ldo2_vset); \
    PLATFORM_RUN_TEST(test_pos_power_ldo3_bypassEn); \
    PLATFORM_RUN_TEST(test_pos_power_ldo1_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_ldo2_grpSel); \
    PLATFORM_RUN_TEST(test_pos_power_ldo_combinedConfig); \
    PLATFORM_RUN_TEST(test_pos_power_ldoValidParams_twoCondition_TT); \
    PLATFORM_RUN_TEST(test_pos_power_ldoValidParams_twoCondition_FF); \
    PLATFORM_RUN_TEST(test_pos_power_getLdoCfg_vmonEnOnlyValid); \
    PLATFORM_RUN_TEST(test_pos_power_getLdoCfg_dischargeEnOnlyValid); \
    PLATFORM_RUN_TEST(test_pos_power_setLdoCfg_vmonEnOnly); \
    PLATFORM_RUN_TEST(test_pos_power_setLdoCfg_dischargeEnOnly); \
    PLATFORM_RUN_TEST(test_pos_power_setLdoCfg_vsetInRangeLdo1)

/* Test: TC-POWER-0015 */
#define POWER_TEST_SETGETLDOCFG() \
    POWER_TEST_NEG_SETGETLDOCFG(); \
    POWER_TEST_POS_SETGETLDOCFG()

/* ======================================================================== */
/*             Test APIs: pwrSetVccaVmonCfg, pwrGetVccaVmonCfg              */
/* ======================================================================== */

#define POWER_TEST_NEG_SETGETVCCAVMONCFG() \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_nullVccaVmonCfg); \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_invalidResource); \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_invalidPgSetVcca); \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_invalidPgSetVmon1); \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_invalidThrVcca); \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_invalidGrpSel); \
    PLATFORM_RUN_TEST(test_neg_power_getVccaVmonCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_getVccaVmonCfg_nullVccaVmonCfg); \
    PLATFORM_RUN_TEST(test_neg_power_getVccaVmonCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_getVccaVmonCfg_invalidResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetVccaVmonCfg_isVmonAboveMax); \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_pgSetAboveMaxVmon2); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetVccaVmonCfg_pgWindowReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetVccaVmonCfg_grpSelReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetVccaVmonCfg_pgWindowReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetVccaVmonCfg_grpSelReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetVccaVmonCfg_vmonEnReadFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetVccaVmonCfg_pgLevelReadFailVmon1); \
    PLATFORM_RUN_TEST(test_neg_power_setVccaVmonCfg_thrValidNoPgSet); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetVccaVmonCfg_ioRxByteFail)

#define POWER_TEST_POS_SETGETVCCAVMONCFG() \
    PLATFORM_RUN_TEST(test_pos_power_vcca_enableDisable); \
    PLATFORM_RUN_TEST(test_pos_power_vcca_pgSet); \
    PLATFORM_RUN_TEST(test_pos_power_vcca_threshold); \
    PLATFORM_RUN_TEST(test_pos_power_vcca_grpSel); \
    PLATFORM_RUN_TEST(test_pos_power_vmon1_enableDisable); \
    PLATFORM_RUN_TEST(test_pos_power_vmon1_pgSet); \
    PLATFORM_RUN_TEST(test_pos_power_vmon2_pgSet); \
    PLATFORM_RUN_TEST(test_pos_power_vmon2_enableDisable); \
    PLATFORM_RUN_TEST(test_pos_power_vmon2_grpSel); \
    PLATFORM_RUN_TEST(test_pos_power_vmon_combinedConfig); \
    PLATFORM_RUN_TEST(test_pos_power_setVccaVmonCfg_vccaPgSetOnly); \
    PLATFORM_RUN_TEST(test_pos_power_setVccaVmonCfg_vmon1_thrAndPgSet)

/* Test: TC-POWER-0016 */
#define POWER_TEST_SETGETVCCAVMONCFG() \
    POWER_TEST_NEG_SETGETVCCAVMONCFG(); \
    POWER_TEST_POS_SETGETVCCAVMONCFG()

/* ======================================================================== */
/*                     Test APIs: pwrSetGlobalVmonDegl                      */
/* ======================================================================== */

#define POWER_TEST_NEG_SETGLOBALVMONDEGL() \
    PLATFORM_RUN_TEST(test_neg_power_setGlobalVmonDegl_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_setGlobalVmonDegl_invalidValue); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetGlobalVmonDegl_ioRxByteFail)

#define POWER_TEST_POS_SETGLOBALVMONDEGL() \
    PLATFORM_RUN_TEST(test_pos_power_globalVmonDegl_allValues)

/* Test: TC-POWER-0017 */
#define POWER_TEST_SETGLOBALVMONDEGL() \
    POWER_TEST_NEG_SETGLOBALVMONDEGL(); \
    POWER_TEST_POS_SETGLOBALVMONDEGL()

/* ======================================================================== */
/*              Test APIs: pwrSetThermalCfg, pwrGetThermalCfg               */
/* ======================================================================== */

#define POWER_TEST_NEG_SETGETTHERMALCFG() \
    PLATFORM_RUN_TEST(test_neg_power_setThermalCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_setThermalCfg_nullThermalCfg); \
    PLATFORM_RUN_TEST(test_neg_power_setThermalCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_setThermalCfg_invalidTwarnLvl); \
    PLATFORM_RUN_TEST(test_neg_power_setThermalCfg_invalidTsdOrdLvl); \
    PLATFORM_RUN_TEST(test_neg_power_getThermalCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_getThermalCfg_nullThermalCfg); \
    PLATFORM_RUN_TEST(test_neg_power_getThermalCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetThermalCfg_ioRxByteCSFail)

#define POWER_TEST_POS_SETGETTHERMALCFG() \
    PLATFORM_RUN_TEST(test_pos_power_thermal_twarnLvl); \
    PLATFORM_RUN_TEST(test_pos_power_thermal_tsdOrdLvl); \
    PLATFORM_RUN_TEST(test_pos_power_thermal_combinedConfig)

/* Test: TC-POWER-0018 */
#define POWER_TEST_SETGETTHERMALCFG() \
    POWER_TEST_NEG_SETGETTHERMALCFG(); \
    POWER_TEST_POS_SETGETTHERMALCFG()

/* ======================================================================== */
/*       Test APIs: pwrSetSpreadSpectrumCfg, pwrGetSpreadSpectrumCfg        */
/* ======================================================================== */

#define POWER_TEST_NEG_SETGETSPREADSPECTRUMCFG() \
    PLATFORM_RUN_TEST(test_neg_power_setSpreadSpectrumCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_setSpreadSpectrumCfg_nullSsCfg); \
    PLATFORM_RUN_TEST(test_neg_power_setSpreadSpectrumCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_getSpreadSpectrumCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_getSpreadSpectrumCfg_nullSsCfg); \
    PLATFORM_RUN_TEST(test_neg_power_getSpreadSpectrumCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSpreadSpectrumCfg_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSpreadSpectrumCfg_ioRxByteFail)

#define POWER_TEST_POS_SETGETSPREADSPECTRUMCFG() \
    PLATFORM_RUN_TEST(test_pos_power_spreadSpectrum_enableDisable); \
    PLATFORM_RUN_TEST(test_pos_power_spreadSpectrum_depth); \
    PLATFORM_RUN_TEST(test_pos_power_spreadSpectrum_combinedConfig)

/* Test: TC-POWER-0019 */
#define POWER_TEST_SETGETSPREADSPECTRUMCFG() \
    POWER_TEST_NEG_SETGETSPREADSPECTRUMCFG(); \
    POWER_TEST_POS_SETGETSPREADSPECTRUMCFG()

/* ======================================================================== */
/*                     Test APIs: Pmic_pwrGetRsrcStatus                     */
/* ======================================================================== */

#define POWER_TEST_NEG_GETRSRCSTATUS() \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_nullRsrcStatus); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_buckReadError); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_ldoVmonReadFail)

#define POWER_TEST_POS_GETRSRCSTATUS() \
    PLATFORM_RUN_TEST(test_pos_power_rsrcStatus_buckUVOV); \
    PLATFORM_RUN_TEST(test_pos_power_rsrcStatus_ldoUVOV); \
    PLATFORM_RUN_TEST(test_pos_power_rsrcStatus_vmonUVOV); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buck1Only); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buck2Only); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buck3Only); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buck4Only); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo1Only); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo2Only); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo3Only); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_vmon1Only); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_vmon2Only); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_vccaOnly); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_noBucks); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_noLdoVmon)

/* Test: TC-POWER-0020 */
#define POWER_TEST_GETRSRCSTATUS() \
    POWER_TEST_NEG_GETRSRCSTATUS(); \
    POWER_TEST_POS_GETRSRCSTATUS()

/* ========================================================================== */
/*                     Property Tests (BUILD_MOCK)                            */
/* ========================================================================== */

#ifdef BUILD_MOCK
/* Test: TC-POWER-0043 */
#define POWER_TEST_POS_PROPERTY() \
    PLATFORM_RUN_TEST(test_pos_power_property_buckVoltageBounds); \
    PLATFORM_RUN_TEST(test_pos_power_property_vmonThresholdEnumeration)
#else
#define POWER_TEST_POS_PROPERTY()
#endif

/* ========================================================================== */
/*                          Aggregate Test Macros                             */
/* ========================================================================== */

#define POWER_TEST_RUN_POSITIVE() \
    POWER_TEST_POS_SETGETBUCKCFG(); \
    POWER_TEST_POS_SETGETLDOCFG(); \
    POWER_TEST_POS_SETGETVCCAVMONCFG(); \
    POWER_TEST_POS_SETGLOBALVMONDEGL(); \
    POWER_TEST_POS_SETGETTHERMALCFG(); \
    POWER_TEST_POS_SETGETSPREADSPECTRUMCFG(); \
    POWER_TEST_POS_GETRSRCSTATUS(); \
    POWER_TEST_POS_PROPERTY()

#define POWER_TEST_RUN_NEGATIVE() \
    POWER_TEST_NEG_SETGETBUCKCFG(); \
    POWER_TEST_NEG_SETGETLDOCFG(); \
    POWER_TEST_NEG_SETGETVCCAVMONCFG(); \
    POWER_TEST_NEG_SETGLOBALVMONDEGL(); \
    POWER_TEST_NEG_SETGETTHERMALCFG(); \
    POWER_TEST_NEG_SETGETSPREADSPECTRUMCFG(); \
    POWER_TEST_NEG_GETRSRCSTATUS()

#define POWER_TEST_RUN_ALL() \
    POWER_TEST_NEG_SETGETBUCKCFG(); \
    POWER_TEST_POS_SETGETBUCKCFG(); \
    POWER_TEST_NEG_SETGETLDOCFG(); \
    POWER_TEST_POS_SETGETLDOCFG(); \
    POWER_TEST_NEG_SETGETVCCAVMONCFG(); \
    POWER_TEST_POS_SETGETVCCAVMONCFG(); \
    POWER_TEST_NEG_SETGLOBALVMONDEGL(); \
    POWER_TEST_POS_SETGLOBALVMONDEGL(); \
    POWER_TEST_NEG_SETGETTHERMALCFG(); \
    POWER_TEST_POS_SETGETTHERMALCFG(); \
    POWER_TEST_NEG_SETGETSPREADSPECTRUMCFG(); \
    POWER_TEST_POS_SETGETSPREADSPECTRUMCFG(); \
    POWER_TEST_NEG_GETRSRCSTATUS(); \
    POWER_TEST_POS_GETRSRCSTATUS(); \
    POWER_TEST_POS_PROPERTY()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Execute all power module tests.
 *
 * @param args [IN] Unused parameter.
 */
void power_test(void *args);

/* Negative test functions */
void test_neg_power_setBuckCfg_nullHandle(void);
void test_neg_power_setBuckCfg_nullBuckCfg(void);
void test_neg_power_setBuckCfg_invalidValidParams(void);
void test_neg_power_setBuckCfg_invalidResource(void);
void test_neg_power_setBuckCfg_invalidSlewRate(void);
void test_neg_power_setBuckCfg_invalidVsetBuck1(void);
void test_neg_power_setBuckCfg_invalidVmonThr(void);
void test_neg_power_setBuckCfg_invalidGrpSel(void);
void test_neg_power_getBuckCfg_nullHandle(void);
void test_neg_power_getBuckCfg_nullBuckCfg(void);
void test_neg_power_getBuckCfg_invalidValidParams(void);
void test_neg_power_getBuckCfg_invalidResource(void);
void test_neg_power_setLdoCfg_nullHandle(void);
void test_neg_power_setLdoCfg_nullLdoCfg(void);
void test_neg_power_setLdoCfg_invalidValidParams(void);
void test_neg_power_setLdoCfg_invalidResource(void);
void test_neg_power_setLdoCfg_invalidVsetLdo1(void);
void test_neg_power_setLdoCfg_invalidVmonThr(void);
void test_neg_power_setLdoCfg_invalidGrpSel(void);
void test_neg_power_getLdoCfg_nullHandle(void);
void test_neg_power_getLdoCfg_nullLdoCfg(void);
void test_neg_power_getLdoCfg_invalidValidParams(void);
void test_neg_power_getLdoCfg_invalidResource(void);
void test_neg_power_setVccaVmonCfg_nullHandle(void);
void test_neg_power_setVccaVmonCfg_nullVccaVmonCfg(void);
void test_neg_power_setVccaVmonCfg_invalidValidParams(void);
void test_neg_power_setVccaVmonCfg_invalidResource(void);
void test_neg_power_setVccaVmonCfg_invalidPgSetVcca(void);
void test_neg_power_setVccaVmonCfg_invalidThrVcca(void);
void test_neg_power_setVccaVmonCfg_invalidGrpSel(void);
void test_neg_power_getVccaVmonCfg_nullHandle(void);
void test_neg_power_getVccaVmonCfg_nullVccaVmonCfg(void);
void test_neg_power_getVccaVmonCfg_invalidValidParams(void);
void test_neg_power_getVccaVmonCfg_invalidResource(void);
void test_neg_power_setGlobalVmonDegl_nullHandle(void);
void test_neg_power_setGlobalVmonDegl_invalidValue(void);
void test_neg_power_setThermalCfg_nullHandle(void);
void test_neg_power_setThermalCfg_nullThermalCfg(void);
void test_neg_power_setThermalCfg_invalidValidParams(void);
void test_neg_power_setThermalCfg_invalidTwarnLvl(void);
void test_neg_power_setThermalCfg_invalidTsdOrdLvl(void);
void test_neg_power_getThermalCfg_nullHandle(void);
void test_neg_power_getThermalCfg_nullThermalCfg(void);
void test_neg_power_getThermalCfg_invalidValidParams(void);
void test_neg_power_setSpreadSpectrumCfg_nullHandle(void);
void test_neg_power_setSpreadSpectrumCfg_nullSsCfg(void);
void test_neg_power_setSpreadSpectrumCfg_invalidValidParams(void);
void test_neg_power_getSpreadSpectrumCfg_nullHandle(void);
void test_neg_power_getSpreadSpectrumCfg_nullSsCfg(void);
void test_neg_power_getSpreadSpectrumCfg_invalidValidParams(void);
void test_neg_power_getRsrcStatus_nullHandle(void);
void test_neg_power_getRsrcStatus_nullRsrcStatus(void);
void test_neg_power_getRsrcStatus_invalidValidParams(void);
void test_neg_power_getRsrcStatus_buckReadError(void);
void test_neg_power_setVccaVmonCfg_invalidPgSetVmon1(void);

/* Positive test functions */
void test_pos_power_buck1_enableDisable(void);
void test_pos_power_buck2_vset(void);
void test_pos_power_buck3_slewRate(void);
void test_pos_power_buck4_vmonThr(void);
void test_pos_power_buck1_grpSel(void);
void test_pos_power_buck_pldnEn(void);
void test_pos_power_buck_vmonEn(void);
void test_pos_power_buck_fpwmEn(void);
void test_pos_power_ldo1_enableDisable(void);
void test_pos_power_ldo2_vset(void);
void test_pos_power_ldo3_bypassEn(void);
void test_pos_power_ldo1_vmonThr(void);
void test_pos_power_ldo2_grpSel(void);
void test_pos_power_vcca_enableDisable(void);
void test_pos_power_vcca_pgSet(void);
void test_pos_power_vcca_threshold(void);
void test_pos_power_vcca_grpSel(void);
void test_pos_power_vmon1_enableDisable(void);
void test_pos_power_vmon2_pgSet(void);
void test_pos_power_vmon1_pgSet(void);
void test_pos_power_vmon2_enableDisable(void);
void test_pos_power_vmon2_grpSel(void);
void test_pos_power_globalVmonDegl_allValues(void);
void test_pos_power_thermal_twarnLvl(void);
void test_pos_power_thermal_tsdOrdLvl(void);
void test_pos_power_spreadSpectrum_enableDisable(void);
void test_pos_power_spreadSpectrum_depth(void);
void test_pos_power_rsrcStatus_buckUVOV(void);
void test_pos_power_rsrcStatus_ldoUVOV(void);
void test_pos_power_rsrcStatus_vmonUVOV(void);
void test_pos_power_buck_combinedConfig(void);
void test_pos_power_ldo_combinedConfig(void);
void test_pos_power_vmon_combinedConfig(void);
void test_pos_power_thermal_combinedConfig(void);
void test_pos_power_spreadSpectrum_combinedConfig(void);
void test_pos_power_property_buckVoltageBounds(void);
void test_pos_power_property_vmonThresholdEnumeration(void);

/* MC/DC coverage test functions */
void test_pos_power_ldoValidParams_twoCondition_TT(void);
void test_pos_power_ldoValidParams_twoCondition_FF(void);

/* Dynamic analysis / error injection test functions (BUILD_MOCK) */
void test_neg_power_pwrGetBuckCfg_ioRxByteFail(void);
void test_neg_power_pwrGetLdoCfg_ioRxByteFail(void);
void test_neg_power_pwrGetThermalCfg_ioRxByteCSFail(void);
void test_neg_power_pwrGetSpreadSpectrumCfg_ioRxByteCSFail(void);
void test_neg_power_pwrGetVccaVmonCfg_ioRxByteFail(void);
void test_neg_power_pwrSetGlobalVmonDegl_ioRxByteFail(void);
void test_neg_power_pwrSetSpreadSpectrumCfg_ioRxByteFail(void);
void test_neg_power_pwrGetRsrcStatus_ldoVmonReadFail(void);

void test_neg_power_pwrSetBuckCfg_isBuckBelowMin(void);
void test_neg_power_pwrSetVccaVmonCfg_isVmonAboveMax(void);
void test_pos_power_getLdoCfg_vmonEnOnlyValid(void);
void test_pos_power_getLdoCfg_dischargeEnOnlyValid(void);
void test_neg_power_setBuckCfg_vsetAboveMaxBuck1(void);
void test_pos_power_setLdoCfg_vmonEnOnly(void);
void test_pos_power_setLdoCfg_dischargeEnOnly(void);
void test_neg_power_setLdoCfg_invalidVsetLdo2(void);
void test_neg_power_setVccaVmonCfg_pgSetAboveMaxVmon2(void);
void test_pos_power_setVccaVmonCfg_vccaPgSetOnly(void);
void test_neg_power_pwrSetVccaVmonCfg_vmonEnReadFail(void);
void test_neg_power_pwrGetVccaVmonCfg_pgLevelReadFailVmon1(void);
void test_neg_power_setLdoCfg_vsetAboveMaxLdo2(void);
void test_neg_power_setVccaVmonCfg_thrValidNoPgSet(void);
void test_pos_power_setLdoCfg_vsetInRangeLdo1(void);
void test_pos_power_setVccaVmonCfg_vmon1_thrAndPgSet(void);

/* Static helper coverage test functions (BUILD_MOCK) — internal I/O failures */
void test_neg_power_pwrSetBuckCfg_ctrlReadFail(void);
void test_neg_power_pwrSetBuckCfg_slewRateReadFail(void);
void test_neg_power_pwrSetBuckCfg_vmonThrReadFail(void);
void test_neg_power_pwrSetBuckCfg_grpSelReadFail(void);
void test_neg_power_pwrGetBuckCfg_slewRateReadFail(void);
void test_neg_power_pwrGetBuckCfg_vsetReadFail(void);
void test_neg_power_pwrGetBuckCfg_vmonThrReadFail(void);
void test_neg_power_pwrGetBuckCfg_grpSelReadFail(void);
void test_neg_power_pwrSetLdoCfg_ctrlReadFail(void);
void test_neg_power_pwrSetLdoCfg_voutReadFail(void);
void test_neg_power_pwrSetLdoCfg_vmonThrReadFail(void);
void test_neg_power_pwrSetLdoCfg_grpSelReadFail(void);
void test_neg_power_pwrGetLdoCfg_voutReadFail(void);
void test_neg_power_pwrGetLdoCfg_vmonThrReadFail(void);
void test_neg_power_pwrGetLdoCfg_grpSelReadFail(void);
void test_neg_power_pwrSetVccaVmonCfg_pgWindowReadFail(void);
void test_neg_power_pwrSetVccaVmonCfg_grpSelReadFail(void);
void test_neg_power_pwrGetVccaVmonCfg_pgWindowReadFail(void);
void test_neg_power_pwrGetVccaVmonCfg_grpSelReadFail(void);
void test_pos_power_getRsrcStatus_buck1Only(void);
void test_pos_power_getRsrcStatus_buck2Only(void);
void test_pos_power_getRsrcStatus_buck3Only(void);
void test_pos_power_getRsrcStatus_buck4Only(void);
void test_pos_power_getRsrcStatus_ldo1Only(void);
void test_pos_power_getRsrcStatus_ldo2Only(void);
void test_pos_power_getRsrcStatus_ldo3Only(void);
void test_pos_power_getRsrcStatus_vmon1Only(void);
void test_pos_power_getRsrcStatus_vmon2Only(void);
void test_pos_power_getRsrcStatus_vccaOnly(void);
void test_pos_power_getRsrcStatus_noBucks(void);
void test_pos_power_getRsrcStatus_noLdoVmon(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_TEST_H */
