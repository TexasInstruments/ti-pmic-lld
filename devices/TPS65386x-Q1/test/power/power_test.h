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



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "../platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                      Test APIs: pwrSetBuckBoostCfg                       */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETBUCKBOOSTCFG() \
    PLATFORM_RUN_TEST(test_pos_power_setGetBuckBoostCfg_lvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetBuckBoostCfg_stbyLvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetBuckBoostCfg_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_setGetBuckBoostCfg_vmonDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetBuckBoostCfg_boostTmo); \
    PLATFORM_RUN_TEST(test_pos_power_setGetBuckBoostCfg_ssEn); \
    PLATFORM_RUN_TEST(test_pos_power_setGetBuckBoostCfg_includeOvUvStatInPGood); \
    PLATFORM_RUN_TEST(test_pos_power_setGetBuckBoostCfg_allCfg)

#define POWER_TEST_NEG_PWRSETBUCKBOOSTCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckBoostCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckBoostCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckBoostCfg_outOfBounds_lvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckBoostCfg_outOfBounds_stbyLvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckBoostCfg_outOfBounds_vmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckBoostCfg_outOfBounds_vmonDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckBoostCfg_outOfBounds_boostTmo); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckBoostCfg_zeroValidParams)

/* Test: TC-POWER-0001 */
#define POWER_TEST_PWRSETBUCKBOOSTCFG() \
    POWER_TEST_POS_PWRSETBUCKBOOSTCFG(); \
    POWER_TEST_NEG_PWRSETBUCKBOOSTCFG()

/* ======================================================================== */
/*                      Test APIs: pwrGetBuckBoostCfg                       */
/* ======================================================================== */
#define POWER_TEST_POS_PWRGETBUCKBOOSTCFG() \
    /* Positive tests for pwrGetBuckBoostCfg are combined with pwrSetBuckBoostCfg tests */

#define POWER_TEST_NEG_PWRGETBUCKBOOSTCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckBoostCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckBoostCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckBoostCfg_zeroValidParams)

/* Test: TC-POWER-0002 */
#define POWER_TEST_PWRGETBUCKBOOSTCFG() \
    POWER_TEST_POS_PWRGETBUCKBOOSTCFG(); \
    POWER_TEST_NEG_PWRGETBUCKBOOSTCFG()

/* ======================================================================== */
/*                         Test APIs: pwrSetLdoCfg                          */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETLDOCFG() \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo1_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo1_lvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo1_ilimLvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo1_ilimDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo1_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo1_vmonDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo1_rampTime); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo1_disableDischarge); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo1_includeOvUvStatInPGood); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo2_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo2_lvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo2_ilimLvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo2_ilimDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo2_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo2_vmonDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo2_rampTime); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo2_disableDischarge); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo2_includeOvUvStatInPGood); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo3_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo3_lvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo3_ilimLvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo3_ilimDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo3_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo3_vmonDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo3_rampTime); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo3_disableDischarge); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo3_includeOvUvStatInPGood); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo4_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo4_lvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo4_ilimLvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo4_ilimDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo4_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo4_vmonDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo4_rampTime); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo4_disableDischarge); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_ldo4_includeOvUvStatInPGood); \
    PLATFORM_RUN_TEST(test_pos_power_setGetLdoCfg_allLdos_allCfg)

#define POWER_TEST_NEG_PWRSETLDOCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_invalidParam_ldo1_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_lvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_ilimLvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_ilimDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_vmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_vmonDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_rampTime); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_invalidParam_ldo2_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_lvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_ilimLvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_ilimDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_vmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_vmonDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_rampTime); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_invalidParam_ldo3_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_lvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_ilimLvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_ilimDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_vmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_vmonDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_rampTime); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_invalidParam_ldo4_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_lvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_ilimLvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_ilimDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_vmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_vmonDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_rampTime); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_invalidLdoId)

/* Test: TC-POWER-0032 */
#define POWER_TEST_PWRSETLDOCFG() \
    POWER_TEST_POS_PWRSETLDOCFG(); \
    POWER_TEST_NEG_PWRSETLDOCFG()

/* ======================================================================== */
/*                         Test APIs: pwrGetLdoCfg                          */
/* ======================================================================== */
#define POWER_TEST_POS_PWRGETLDOCFG() \
    /* Positive tests for pwrGetLdoCfg are combined with pwrSetLdoCfg tests */

#define POWER_TEST_NEG_PWRGETLDOCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_invalidLdoId)

/* Test: TC-POWER-0033 */
#define POWER_TEST_PWRGETLDOCFG() \
    POWER_TEST_POS_PWRGETLDOCFG(); \
    POWER_TEST_NEG_PWRGETLDOCFG()

/* ======================================================================== */
/*                         Test APIs: pwrSetPldoCfg                         */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETPLDOCFG() \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_trackingMode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_lvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_ilimLvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_ilimDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_vmonDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_vtrackRange); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_rampTime); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_disableDischarge); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo1_includeOvUvStatInPGood); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_trackingMode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_lvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_ilimLvl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_ilimDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_vmonDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_vtrackRange); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_rampTime); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_disableDischarge); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_pldo2_includeOvUvStatInPGood); \
    PLATFORM_RUN_TEST(test_pos_power_setGetPldoCfg_allPldos_allCfg)

#define POWER_TEST_NEG_PWRSETPLDOCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_invalidParam_pldo1_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_lvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_ilimLvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_ilimDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_vmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_vmonDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_vtrackRange); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_rampTime); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_invalidParam_pldo2_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_lvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_ilimLvl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_ilimDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_vmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_vmonDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_vtrackRange); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_rampTime); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPldoCfg_invalidPldoId)

/* Test: TC-POWER-0005 */
#define POWER_TEST_PWRSETPLDOCFG() \
    POWER_TEST_POS_PWRSETPLDOCFG(); \
    POWER_TEST_NEG_PWRSETPLDOCFG()

/* ======================================================================== */
/*                         Test APIs: pwrGetPldoCfg                         */
/* ======================================================================== */
#define POWER_TEST_POS_PWRGETPLDOCFG() \
    PLATFORM_RUN_TEST(test_pos_power_pwrGetPldoCfg_redundantModeConversion)

#define POWER_TEST_NEG_PWRGETPLDOCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetPldoCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetPldoCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetPldoCfg_invalidPldoId)

/* Test: TC-POWER-0006 */
#define POWER_TEST_PWRGETPLDOCFG() \
    POWER_TEST_POS_PWRGETPLDOCFG(); \
    POWER_TEST_NEG_PWRGETPLDOCFG()

/* ======================================================================== */
/*                       Test APIs: pwrSetExtVmonCfg                        */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETEXTVMONCFG() \
    PLATFORM_RUN_TEST(test_pos_power_setGetExtVmonCfg_vmon1_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetExtVmonCfg_vmon1_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_setGetExtVmonCfg_vmon1_vmonDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetExtVmonCfg_vmon1_includeOvUvStatInPGood); \
    PLATFORM_RUN_TEST(test_pos_power_setGetExtVmonCfg_vmon2_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetExtVmonCfg_vmon2_vmonThr); \
    PLATFORM_RUN_TEST(test_pos_power_setGetExtVmonCfg_vmon2_vmonDgl); \
    PLATFORM_RUN_TEST(test_pos_power_setGetExtVmonCfg_vmon2_includeOvUvStatInPGood); \
    PLATFORM_RUN_TEST(test_pos_power_setGetExtVmonCfg_allVmons_allCfg)

#define POWER_TEST_NEG_PWRSETEXTVMONCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetExtVmonCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetExtVmonCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetExtVmonCfg_invalidParam_vmon1_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon1_vmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon1_vmonDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetExtVmonCfg_invalidParam_vmon2_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon2_vmonThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon2_vmonDgl); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetExtVmonCfg_invalidExtVmonId)

/* Test: TC-POWER-0007 */
#define POWER_TEST_PWRSETEXTVMONCFG() \
    POWER_TEST_POS_PWRSETEXTVMONCFG(); \
    POWER_TEST_NEG_PWRSETEXTVMONCFG()

/* ======================================================================== */
/*                       Test APIs: pwrGetExtVmonCfg                        */
/* ======================================================================== */
#define POWER_TEST_POS_PWRGETEXTVMONCFG() \
    /* Positive tests for pwrGetExtVmonCfg are combined with pwrSetExtVmonCfg tests */

#define POWER_TEST_NEG_PWRGETEXTVMONCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetExtVmonCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetExtVmonCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetExtVmonCfg_invalidExtVmonId)

/* Test: TC-POWER-0008 */
#define POWER_TEST_PWRGETEXTVMONCFG() \
    POWER_TEST_POS_PWRGETEXTVMONCFG(); \
    POWER_TEST_NEG_PWRGETEXTVMONCFG()

/* ======================================================================== */
/*                       Test APIs: pwrGetRsrcStatus                        */
/* ======================================================================== */
#define POWER_TEST_POS_PWRGETRSRCSTATUS() \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buckBoost); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo1); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo2); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo3); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo4); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_pldo1); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_pldo2); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_extVmon1); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_extVmon2); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buckBoost_bbLite); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buckBoost_bbIlimLvl); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buckBoost_bbMode); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buckBoost_ovErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buckBoost_uvErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buckBoost_tsdErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_buckBoost_tsdWarn); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo_uvErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo_ovErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo_tsdErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_ldo_tsdWarn); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_pldo_uvErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_pldo_ovErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_pldo_tsdErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_pldo_tsdWarn); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_extVmon_uvErr); \
    PLATFORM_RUN_TEST(test_pos_power_getRsrcStatus_extVmon_ovErr)

#define POWER_TEST_NEG_PWRGETRSRCSTATUS() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_nullStatus); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_invalidResourceType); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_malformedBbResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_malformedLdoResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_malformedPldoResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_malformedExtVmonResource); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_ldo_unsupportedBbLite); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_ldo_unsupportedBbIlimLvl); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_ldo_unsupportedBbMode); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_pldo_unsupportedBbLite); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_extVmon_unsupportedIlimErr); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_extVmon_unsupportedTsdErr); \
    PLATFORM_RUN_TEST(test_neg_power_getRsrcStatus_extVmon_unsupportedTsdWarn)

/* Test: TC-POWER-0038 */
#define POWER_TEST_PWRGETRSRCSTATUS() \
    POWER_TEST_POS_PWRGETRSRCSTATUS(); \
    POWER_TEST_NEG_PWRGETRSRCSTATUS()

/* ======================================================================== */
/*                       Test APIs: pwrClrRsrcStatus                        */
/* ======================================================================== */
#define POWER_TEST_POS_PWRCLRRSRCSTATUS() \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_buckBoost); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_buckBoost_bbMode); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_buckBoost_ilimErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_buckBoost_tsdErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_buckBoost_tsdWarn); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_ldo1_uvErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_ldo2_ovErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_ldo3_tsdErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_ldo4_tsdWarn); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_ldo_allStatus); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_pldo1_uvErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_pldo2_ovErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_pldo_tsdErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_pldo_allStatus); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_extVmon1_uvErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_extVmon2_ovErr); \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatus_extVmon_allStatus)

#define POWER_TEST_NEG_PWRCLRRSRCSTATUS() \
    PLATFORM_RUN_TEST(test_neg_power_pwrClrRsrcStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrClrRsrcStatus_nullStatus); \
    PLATFORM_RUN_TEST(test_neg_power_pwrClrRsrcStatus_invalidResourceType); \
    PLATFORM_RUN_TEST(test_neg_power_pwrClrRsrcStatus_malformedBbResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrClrRsrcStatus_malformedLdoResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrClrRsrcStatus_malformedPldoResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrClrRsrcStatus_malformedExtVmonResource); \
    PLATFORM_RUN_TEST(test_neg_power_clrRsrcStatus_extVmon_unsupportedTsdWarn)

/* Test: TC-POWER-0010 */
#define POWER_TEST_PWRCLRRSRCSTATUS() \
    POWER_TEST_POS_PWRCLRRSRCSTATUS(); \
    POWER_TEST_NEG_PWRCLRRSRCSTATUS()

/* ======================================================================== */
/*                      Test APIs: pwrClrRsrcStatusAll                      */
/* ======================================================================== */
#define POWER_TEST_POS_PWRCLRRSRCSTATUSALL() \
    PLATFORM_RUN_TEST(test_pos_power_clrRsrcStatusAll)

#define POWER_TEST_NEG_PWRCLRRSRCSTATUSALL() \
    PLATFORM_RUN_TEST(test_neg_power_pwrClrRsrcStatusAll_nullHandle)

/* Test: TC-POWER-0011 */
#define POWER_TEST_PWRCLRRSRCSTATUSALL() \
    POWER_TEST_POS_PWRCLRRSRCSTATUSALL(); \
    POWER_TEST_NEG_PWRCLRRSRCSTATUSALL()

/* ======================================================================== */
/*                       Test APIs: pwrSetPGoodInStby                       */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETPGOODINSTBY() \
    PLATFORM_RUN_TEST(test_pos_power_setGetPGoodInStby)

#define POWER_TEST_NEG_PWRSETPGOODINSTBY() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetPGoodInStby_nullHandle)

/* Test: TC-POWER-0012 */
#define POWER_TEST_PWRSETPGOODINSTBY() \
    POWER_TEST_POS_PWRSETPGOODINSTBY(); \
    POWER_TEST_NEG_PWRSETPGOODINSTBY()

/* ======================================================================== */
/*                       Test APIs: pwrGetPGoodInStby                       */
/* ======================================================================== */
#define POWER_TEST_POS_PWRGETPGOODINSTBY() \
    /* Positive tests for pwrGetPGoodInStby are combined with pwrSetPGoodInStby tests */

#define POWER_TEST_NEG_PWRGETPGOODINSTBY() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetPGoodInStby_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetPGoodInStby_nullIsEnabled)

/* Test: TC-POWER-0013 */
#define POWER_TEST_PWRGETPGOODINSTBY() \
    POWER_TEST_POS_PWRGETPGOODINSTBY(); \
    POWER_TEST_NEG_PWRGETPGOODINSTBY()

/* ========================================================================== */
/*                          Aggregate Test Runners                            */
/* ========================================================================== */
#define POWER_TEST_RUN_POSITIVE() \
    POWER_TEST_POS_PWRSETBUCKBOOSTCFG(); \
    POWER_TEST_POS_PWRGETBUCKBOOSTCFG(); \
    POWER_TEST_POS_PWRSETLDOCFG(); \
    POWER_TEST_POS_PWRGETLDOCFG(); \
    POWER_TEST_POS_PWRSETPLDOCFG(); \
    POWER_TEST_POS_PWRGETPLDOCFG(); \
    POWER_TEST_POS_PWRSETEXTVMONCFG(); \
    POWER_TEST_POS_PWRGETEXTVMONCFG(); \
    POWER_TEST_POS_PWRGETRSRCSTATUS(); \
    POWER_TEST_POS_PWRCLRRSRCSTATUS(); \
    POWER_TEST_POS_PWRCLRRSRCSTATUSALL(); \
    POWER_TEST_POS_PWRSETPGOODINSTBY(); \
    POWER_TEST_POS_PWRGETPGOODINSTBY()

#define POWER_TEST_RUN_NEGATIVE() \
    POWER_TEST_NEG_PWRSETBUCKBOOSTCFG(); \
    POWER_TEST_NEG_PWRGETBUCKBOOSTCFG(); \
    POWER_TEST_NEG_PWRSETLDOCFG(); \
    POWER_TEST_NEG_PWRGETLDOCFG(); \
    POWER_TEST_NEG_PWRSETPLDOCFG(); \
    POWER_TEST_NEG_PWRGETPLDOCFG(); \
    POWER_TEST_NEG_PWRSETEXTVMONCFG(); \
    POWER_TEST_NEG_PWRGETEXTVMONCFG(); \
    POWER_TEST_NEG_PWRGETRSRCSTATUS(); \
    POWER_TEST_NEG_PWRCLRRSRCSTATUS(); \
    POWER_TEST_NEG_PWRCLRRSRCSTATUSALL(); \
    POWER_TEST_NEG_PWRSETPGOODINSTBY(); \
    POWER_TEST_NEG_PWRGETPGOODINSTBY()

#define POWER_TEST_RUN_ALL() \
    POWER_TEST_RUN_POSITIVE(); \
    POWER_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Power test suite entry point
 * @param args Test arguments (unused)
 */
void power_test(void *args);

/* ========================================================================== */
/*               pwrSetBuckBoostCfg / pwrGetBuckBoostCfg API Tests            */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_setGetBuckBoostCfg_lvl(void);
void test_pos_power_setGetBuckBoostCfg_stbyLvl(void);
void test_pos_power_setGetBuckBoostCfg_vmonThr(void);
void test_pos_power_setGetBuckBoostCfg_vmonDgl(void);
void test_pos_power_setGetBuckBoostCfg_boostTmo(void);
void test_pos_power_setGetBuckBoostCfg_ssEn(void);
void test_pos_power_setGetBuckBoostCfg_includeOvUvStatInPGood(void);
void test_pos_power_setGetBuckBoostCfg_allCfg(void);

/* Negative tests */
void test_neg_power_pwrSetBuckBoostCfg_nullHandle(void);
void test_neg_power_pwrSetBuckBoostCfg_nullConfig(void);
void test_neg_power_pwrGetBuckBoostCfg_nullHandle(void);
void test_neg_power_pwrGetBuckBoostCfg_nullConfig(void);
void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_lvl(void);
void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_stbyLvl(void);
void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_vmonThr(void);
void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_vmonDgl(void);
void test_neg_power_pwrSetBuckBoostCfg_outOfBounds_boostTmo(void);
void test_neg_power_pwrSetBuckBoostCfg_zeroValidParams(void);
void test_neg_power_pwrGetBuckBoostCfg_zeroValidParams(void);

/* ========================================================================== */
/*                  pwrSetLdoCfg / pwrGetLdoCfg API Tests                     */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_setGetLdoCfg_ldo1_mode(void);
void test_pos_power_setGetLdoCfg_ldo1_lvl(void);
void test_pos_power_setGetLdoCfg_ldo1_ilimLvl(void);
void test_pos_power_setGetLdoCfg_ldo1_ilimDgl(void);
void test_pos_power_setGetLdoCfg_ldo1_vmonThr(void);
void test_pos_power_setGetLdoCfg_ldo1_vmonDgl(void);
void test_pos_power_setGetLdoCfg_ldo1_rampTime(void);
void test_pos_power_setGetLdoCfg_ldo1_disableDischarge(void);
void test_pos_power_setGetLdoCfg_ldo1_includeOvUvStatInPGood(void);
void test_pos_power_setGetLdoCfg_ldo2_mode(void);
void test_pos_power_setGetLdoCfg_ldo2_lvl(void);
void test_pos_power_setGetLdoCfg_ldo2_ilimLvl(void);
void test_pos_power_setGetLdoCfg_ldo2_ilimDgl(void);
void test_pos_power_setGetLdoCfg_ldo2_vmonThr(void);
void test_pos_power_setGetLdoCfg_ldo2_vmonDgl(void);
void test_pos_power_setGetLdoCfg_ldo2_rampTime(void);
void test_pos_power_setGetLdoCfg_ldo2_disableDischarge(void);
void test_pos_power_setGetLdoCfg_ldo2_includeOvUvStatInPGood(void);
void test_pos_power_setGetLdoCfg_ldo3_mode(void);
void test_pos_power_setGetLdoCfg_ldo3_lvl(void);
void test_pos_power_setGetLdoCfg_ldo3_ilimLvl(void);
void test_pos_power_setGetLdoCfg_ldo3_ilimDgl(void);
void test_pos_power_setGetLdoCfg_ldo3_vmonThr(void);
void test_pos_power_setGetLdoCfg_ldo3_vmonDgl(void);
void test_pos_power_setGetLdoCfg_ldo3_rampTime(void);
void test_pos_power_setGetLdoCfg_ldo3_disableDischarge(void);
void test_pos_power_setGetLdoCfg_ldo3_includeOvUvStatInPGood(void);
void test_pos_power_setGetLdoCfg_ldo4_mode(void);
void test_pos_power_setGetLdoCfg_ldo4_lvl(void);
void test_pos_power_setGetLdoCfg_ldo4_ilimLvl(void);
void test_pos_power_setGetLdoCfg_ldo4_ilimDgl(void);
void test_pos_power_setGetLdoCfg_ldo4_vmonThr(void);
void test_pos_power_setGetLdoCfg_ldo4_vmonDgl(void);
void test_pos_power_setGetLdoCfg_ldo4_rampTime(void);
void test_pos_power_setGetLdoCfg_ldo4_disableDischarge(void);
void test_pos_power_setGetLdoCfg_ldo4_includeOvUvStatInPGood(void);
void test_pos_power_setGetLdoCfg_allLdos_allCfg(void);

/* Negative tests */
void test_neg_power_pwrSetLdoCfg_nullHandle(void);
void test_neg_power_pwrSetLdoCfg_nullConfig(void);
void test_neg_power_pwrGetLdoCfg_nullHandle(void);
void test_neg_power_pwrGetLdoCfg_nullConfig(void);
void test_neg_power_pwrSetLdoCfg_invalidParam_ldo1_mode(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_lvl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_ilimLvl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_ilimDgl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_vmonThr(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_vmonDgl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo1_rampTime(void);
void test_neg_power_pwrSetLdoCfg_invalidParam_ldo2_mode(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_lvl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_ilimLvl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_ilimDgl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_vmonThr(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_vmonDgl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo2_rampTime(void);
void test_neg_power_pwrSetLdoCfg_invalidParam_ldo3_mode(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_lvl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_ilimLvl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_ilimDgl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_vmonThr(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_vmonDgl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo3_rampTime(void);
void test_neg_power_pwrSetLdoCfg_invalidParam_ldo4_mode(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_lvl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_ilimLvl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_ilimDgl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_vmonThr(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_vmonDgl(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ldo4_rampTime(void);
void test_neg_power_pwrSetLdoCfg_invalidLdoId(void);
void test_neg_power_pwrGetLdoCfg_invalidLdoId(void);

/* ========================================================================== */
/*                 pwrSetPldoCfg / pwrGetPldoCfg API Tests                    */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_setGetPldoCfg_pldo1_mode(void);
void test_pos_power_setGetPldoCfg_pldo1_trackingMode(void);
void test_pos_power_setGetPldoCfg_pldo1_lvl(void);
void test_pos_power_setGetPldoCfg_pldo1_ilimLvl(void);
void test_pos_power_setGetPldoCfg_pldo1_ilimDgl(void);
void test_pos_power_setGetPldoCfg_pldo1_vmonThr(void);
void test_pos_power_setGetPldoCfg_pldo1_vmonDgl(void);
void test_pos_power_setGetPldoCfg_pldo1_vtrackRange(void);
void test_pos_power_setGetPldoCfg_pldo1_rampTime(void);
void test_pos_power_setGetPldoCfg_pldo1_disableDischarge(void);
void test_pos_power_setGetPldoCfg_pldo1_includeOvUvStatInPGood(void);
void test_pos_power_setGetPldoCfg_pldo2_mode(void);
void test_pos_power_setGetPldoCfg_pldo2_trackingMode(void);
void test_pos_power_setGetPldoCfg_pldo2_lvl(void);
void test_pos_power_setGetPldoCfg_pldo2_ilimLvl(void);
void test_pos_power_setGetPldoCfg_pldo2_ilimDgl(void);
void test_pos_power_setGetPldoCfg_pldo2_vmonThr(void);
void test_pos_power_setGetPldoCfg_pldo2_vmonDgl(void);
void test_pos_power_setGetPldoCfg_pldo2_vtrackRange(void);
void test_pos_power_setGetPldoCfg_pldo2_rampTime(void);
void test_pos_power_setGetPldoCfg_pldo2_disableDischarge(void);
void test_pos_power_setGetPldoCfg_pldo2_includeOvUvStatInPGood(void);
void test_pos_power_setGetPldoCfg_allPldos_allCfg(void);
void test_pos_power_pwrGetPldoCfg_redundantModeConversion(void);

/* Negative tests */
void test_neg_power_pwrSetPldoCfg_nullHandle(void);
void test_neg_power_pwrSetPldoCfg_nullConfig(void);
void test_neg_power_pwrGetPldoCfg_nullHandle(void);
void test_neg_power_pwrGetPldoCfg_nullConfig(void);
void test_neg_power_pwrSetPldoCfg_invalidParam_pldo1_mode(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_lvl(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_ilimLvl(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_ilimDgl(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_vmonThr(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_vmonDgl(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_vtrackRange(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo1_rampTime(void);
void test_neg_power_pwrSetPldoCfg_invalidParam_pldo2_mode(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_lvl(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_ilimLvl(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_ilimDgl(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_vmonThr(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_vmonDgl(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_vtrackRange(void);
void test_neg_power_pwrSetPldoCfg_outOfBounds_pldo2_rampTime(void);
void test_neg_power_pwrSetPldoCfg_invalidPldoId(void);
void test_neg_power_pwrGetPldoCfg_invalidPldoId(void);

/* ========================================================================== */
/*             pwrSetExtVmonCfg / pwrGetExtVmonCfg API Tests                  */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_setGetExtVmonCfg_vmon1_mode(void);
void test_pos_power_setGetExtVmonCfg_vmon1_vmonThr(void);
void test_pos_power_setGetExtVmonCfg_vmon1_vmonDgl(void);
void test_pos_power_setGetExtVmonCfg_vmon1_includeOvUvStatInPGood(void);
void test_pos_power_setGetExtVmonCfg_vmon2_mode(void);
void test_pos_power_setGetExtVmonCfg_vmon2_vmonThr(void);
void test_pos_power_setGetExtVmonCfg_vmon2_vmonDgl(void);
void test_pos_power_setGetExtVmonCfg_vmon2_includeOvUvStatInPGood(void);
void test_pos_power_setGetExtVmonCfg_allVmons_allCfg(void);

/* Negative tests */
void test_neg_power_pwrSetExtVmonCfg_nullHandle(void);
void test_neg_power_pwrSetExtVmonCfg_nullConfig(void);
void test_neg_power_pwrGetExtVmonCfg_nullHandle(void);
void test_neg_power_pwrGetExtVmonCfg_nullConfig(void);
void test_neg_power_pwrSetExtVmonCfg_invalidParam_vmon1_mode(void);
void test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon1_vmonThr(void);
void test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon1_vmonDgl(void);
void test_neg_power_pwrSetExtVmonCfg_invalidParam_vmon2_mode(void);
void test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon2_vmonThr(void);
void test_neg_power_pwrSetExtVmonCfg_outOfBounds_vmon2_vmonDgl(void);
void test_neg_power_pwrSetExtVmonCfg_invalidExtVmonId(void);
void test_neg_power_pwrGetExtVmonCfg_invalidExtVmonId(void);

/* ========================================================================== */
/*                    pwrGetRsrcStatus API Tests                              */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_getRsrcStatus_buckBoost(void);
void test_pos_power_getRsrcStatus_ldo1(void);
void test_pos_power_getRsrcStatus_ldo2(void);
void test_pos_power_getRsrcStatus_ldo3(void);
void test_pos_power_getRsrcStatus_ldo4(void);
void test_pos_power_getRsrcStatus_pldo1(void);
void test_pos_power_getRsrcStatus_pldo2(void);
void test_pos_power_getRsrcStatus_extVmon1(void);
void test_pos_power_getRsrcStatus_extVmon2(void);
void test_pos_power_getRsrcStatus_buckBoost_bbLite(void);
void test_pos_power_getRsrcStatus_buckBoost_bbIlimLvl(void);
void test_pos_power_getRsrcStatus_buckBoost_bbMode(void);
void test_pos_power_getRsrcStatus_buckBoost_ovErr(void);
void test_pos_power_getRsrcStatus_buckBoost_uvErr(void);
void test_pos_power_getRsrcStatus_buckBoost_tsdErr(void);
void test_pos_power_getRsrcStatus_buckBoost_tsdWarn(void);
void test_pos_power_getRsrcStatus_ldo_uvErr(void);
void test_pos_power_getRsrcStatus_ldo_ovErr(void);
void test_pos_power_getRsrcStatus_ldo_tsdErr(void);
void test_pos_power_getRsrcStatus_ldo_tsdWarn(void);
void test_pos_power_getRsrcStatus_pldo_uvErr(void);
void test_pos_power_getRsrcStatus_pldo_ovErr(void);
void test_pos_power_getRsrcStatus_pldo_tsdErr(void);
void test_pos_power_getRsrcStatus_pldo_tsdWarn(void);
void test_pos_power_getRsrcStatus_extVmon_uvErr(void);
void test_pos_power_getRsrcStatus_extVmon_ovErr(void);

/* Negative tests */
void test_neg_power_pwrGetRsrcStatus_nullHandle(void);
void test_neg_power_pwrGetRsrcStatus_nullStatus(void);
void test_neg_power_pwrGetRsrcStatus_invalidResourceType(void);
void test_neg_power_pwrGetRsrcStatus_malformedBbResource(void);
void test_neg_power_pwrGetRsrcStatus_malformedLdoResource(void);
void test_neg_power_pwrGetRsrcStatus_malformedPldoResource(void);
void test_neg_power_pwrGetRsrcStatus_malformedExtVmonResource(void);
void test_neg_power_getRsrcStatus_ldo_unsupportedBbLite(void);
void test_neg_power_getRsrcStatus_ldo_unsupportedBbIlimLvl(void);
void test_neg_power_getRsrcStatus_ldo_unsupportedBbMode(void);
void test_neg_power_getRsrcStatus_pldo_unsupportedBbLite(void);
void test_neg_power_getRsrcStatus_extVmon_unsupportedIlimErr(void);
void test_neg_power_getRsrcStatus_extVmon_unsupportedTsdErr(void);
void test_neg_power_getRsrcStatus_extVmon_unsupportedTsdWarn(void);

/* ========================================================================== */
/*                     pwrClrRsrcStatus API Tests                             */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_clrRsrcStatus_buckBoost(void);
void test_pos_power_clrRsrcStatus_buckBoost_bbMode(void);
void test_pos_power_clrRsrcStatus_buckBoost_ilimErr(void);
void test_pos_power_clrRsrcStatus_buckBoost_tsdErr(void);
void test_pos_power_clrRsrcStatus_buckBoost_tsdWarn(void);
void test_pos_power_clrRsrcStatus_ldo1_uvErr(void);
void test_pos_power_clrRsrcStatus_ldo2_ovErr(void);
void test_pos_power_clrRsrcStatus_ldo3_tsdErr(void);
void test_pos_power_clrRsrcStatus_ldo4_tsdWarn(void);
void test_pos_power_clrRsrcStatus_ldo_allStatus(void);
void test_pos_power_clrRsrcStatus_pldo1_uvErr(void);
void test_pos_power_clrRsrcStatus_pldo2_ovErr(void);
void test_pos_power_clrRsrcStatus_pldo_tsdErr(void);
void test_pos_power_clrRsrcStatus_pldo_allStatus(void);
void test_pos_power_clrRsrcStatus_extVmon1_uvErr(void);
void test_pos_power_clrRsrcStatus_extVmon2_ovErr(void);
void test_pos_power_clrRsrcStatus_extVmon_allStatus(void);

/* Negative tests */
void test_neg_power_pwrClrRsrcStatus_nullHandle(void);
void test_neg_power_pwrClrRsrcStatus_nullStatus(void);
void test_neg_power_pwrClrRsrcStatus_invalidResourceType(void);
void test_neg_power_pwrClrRsrcStatus_malformedBbResource(void);
void test_neg_power_pwrClrRsrcStatus_malformedLdoResource(void);
void test_neg_power_pwrClrRsrcStatus_malformedPldoResource(void);
void test_neg_power_pwrClrRsrcStatus_malformedExtVmonResource(void);
void test_neg_power_clrRsrcStatus_extVmon_unsupportedTsdWarn(void);

/* ========================================================================== */
/*                   pwrClrRsrcStatusAll API Tests                            */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_clrRsrcStatusAll(void);

/* Negative tests */
void test_neg_power_pwrClrRsrcStatusAll_nullHandle(void);

/* ========================================================================== */
/*              pwrSetPGoodInStby / pwrGetPGoodInStby API Tests               */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_setGetPGoodInStby(void);

/* Negative tests */
void test_neg_power_pwrSetPGoodInStby_nullHandle(void);
void test_neg_power_pwrGetPGoodInStby_nullHandle(void);
void test_neg_power_pwrGetPGoodInStby_nullIsEnabled(void);

/* ========================================================================== */
/*                          Coverage Tests                                    */
/* ========================================================================== */

void test_pos_power_pwr_getPldoMode_disabledFallback(void);
void test_neg_power_pwr_clrLdoStat_unsupportedBbParams(void);
void test_neg_power_pwr_clrPldoStat_unsupportedBbParams(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__POWER_TEST_H__*/
