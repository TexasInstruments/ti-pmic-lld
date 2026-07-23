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
/*                             Include Files                                  */
/* ========================================================================== */

#include "irq_test.h"
#ifdef BUILD_MOCK
#include "test_inject.h"
#endif
#include "test_constants.h"
#include "regmap/irq.h"
#include "regmap/fsm.h"

#ifdef BUILD_MOCK
#include "platform_mock.h"
#include "pmic_mock_core.h"
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static Pmic_Handle_t g_pmicHandle;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                            Helper Functions                                */
/* ========================================================================== */

/**
 * @brief Initialize PMIC handle for IRQ tests
 */
static int32_t irqTest_initHandle(void)
{
    /* Dummy handle for mock - driver validates non-NULL but doesn't dereference */
    static uint32_t dummyCommHandle = TEST_DUMMY_HANDLE;

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = (void*)&dummyCommHandle,  /* Driver requires non-NULL, even for mock */
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };
    int32_t status;

    status = Pmic_init(&g_pmicHandle, &pmicCfg);

    return status;
}

/**
 * @brief Helper function to test IRQ mask set/get
 *
 * @param irqNum   IRQ number to test
 * @param maskVal  Mask value to set (true = masked, false = unmasked)
 */
static void irqTest_setGetMask(uint8_t irqNum, bool maskVal)
{
    Pmic_IrqCfg_t setCfg, getCfg;
    int32_t status;

    memset(&setCfg, 0, sizeof(setCfg));
    memset(&getCfg, 0, sizeof(getCfg));

    /* Set IRQ mask configuration */
    setCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    setCfg.irqNum = irqNum;
    setCfg.mask = maskVal;

    status = Pmic_irqSetCfg(&g_pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get IRQ mask configuration */
    getCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    getCfg.irqNum = irqNum;

    status = Pmic_irqGetCfg(&g_pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify mask value */
    PLATFORM_ASSERT(getCfg.mask == maskVal);
}

/* ========================================================================== */
/*                         POSITIVE TEST CASES                                */
/* ========================================================================== */

/* ========================================================================== */
/*                        irqSetCfg/irqGetCfg API Tests                       */
/* ========================================================================== */

/**
 * @brief Set IRQ to interrupt-only mode, then verify unmask and mask.
 *
 * Sets INT_CFG=0 (interrupt-only) before the unmask/mask sequence so that
 * even if the error condition is asserting during the unmask step, the device
 * only generates an interrupt rather than a state machine transition.  Without
 * this, a transition to ACTIVE/SAFE/RESET-MCU would re-assert CFG_REG_LOCK,
 * silently rejecting the subsequent mask write and causing readback to fail.
 */
static void irqTest_maskTest(uint8_t irqNum)
{
    Pmic_IrqCfg_t cfgSet;
    int32_t status;

    memset(&cfgSet, 0, sizeof(cfgSet));
    cfgSet.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    cfgSet.irqNum = irqNum;
    cfgSet.config = PMIC_IRQ_CONFIG0_INT_SET;

    status = Pmic_irqSetCfg(&g_pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    irqTest_setGetMask(irqNum, false);
    irqTest_setGetMask(irqNum, true);
}

/* DCDC (BB/Buck-Boost) IRQ Tests */
void test_pos_irq_irqSetCfg_bb_uvErr_mask(void)
{
    irqTest_maskTest(PMIC_BB_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_bb_ovErr_mask(void)
{
    irqTest_maskTest(PMIC_BB_OV_ERR_INT);
}

/* LDO1 IRQ Tests */
void test_pos_irq_irqSetCfg_ldo1_uvErr_mask(void)
{
    irqTest_maskTest(PMIC_LDO1_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_ldo1_ovErr_mask(void)
{
    irqTest_maskTest(PMIC_LDO1_OV_ERR_INT);
}

/* LDO2 IRQ Tests */
void test_pos_irq_irqSetCfg_ldo2_uvErr_mask(void)
{
    irqTest_maskTest(PMIC_LDO2_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_ldo2_ovErr_mask(void)
{
    irqTest_maskTest(PMIC_LDO2_OV_ERR_INT);
}

/* LDO3 IRQ Tests */
void test_pos_irq_irqSetCfg_ldo3_uvErr_mask(void)
{
    irqTest_maskTest(PMIC_LDO3_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_ldo3_ovErr_mask(void)
{
    irqTest_maskTest(PMIC_LDO3_OV_ERR_INT);
}

/* LDO4 IRQ Tests */
void test_pos_irq_irqSetCfg_ldo4_uvErr_mask(void)
{
    irqTest_maskTest(PMIC_LDO4_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_ldo4_ovErr_mask(void)
{
    irqTest_maskTest(PMIC_LDO4_OV_ERR_INT);
}

/* PLDO1 IRQ Tests */
void test_pos_irq_irqSetCfg_pldo1_uvErr_mask(void)
{
    irqTest_maskTest(PMIC_PLDO1_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_pldo1_ovErr_mask(void)
{
    irqTest_maskTest(PMIC_PLDO1_OV_ERR_INT);
}

/* PLDO2 IRQ Tests */
void test_pos_irq_irqSetCfg_pldo2_uvErr_mask(void)
{
    irqTest_maskTest(PMIC_PLDO2_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_pldo2_ovErr_mask(void)
{
    irqTest_maskTest(PMIC_PLDO2_OV_ERR_INT);
}

/* External VMON1 IRQ Tests */
void test_pos_irq_irqSetCfg_extVmon1_uvErr_mask(void)
{
    irqTest_maskTest(PMIC_EXT_VMON1_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_extVmon1_ovErr_mask(void)
{
    irqTest_maskTest(PMIC_EXT_VMON1_OV_ERR_INT);
}

/* External VMON2 IRQ Tests */
void test_pos_irq_irqSetCfg_extVmon2_uvErr_mask(void)
{
    irqTest_maskTest(PMIC_EXT_VMON2_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_extVmon2_ovErr_mask(void)
{
    irqTest_maskTest(PMIC_EXT_VMON2_OV_ERR_INT);
}

/* Watchdog IRQ Tests */
void test_pos_irq_irqSetCfg_wdTh1Err_mask(void)
{
    irqTest_maskTest(PMIC_WD_TH1_ERR_INT);
}

void test_pos_irq_irqSetCfg_wdTh2Err_mask(void)
{
    irqTest_maskTest(PMIC_WD_TH2_ERR_INT);
}

/* ESM IRQ Tests */
void test_pos_irq_irqSetCfg_esmDly1Err_mask(void)
{
    irqTest_maskTest(PMIC_ESM_DLY1_ERR_INT);
}

void test_pos_irq_irqSetCfg_esmDly2Err_mask(void)
{
    irqTest_maskTest(PMIC_ESM_DLY2_ERR_INT);
}

/* Readback Error IRQ Tests */
void test_pos_irq_irqSetCfg_nrstRdbkErr_mask(void)
{
    irqTest_maskTest(PMIC_NRST_RDBK_ERR_INT);
}

void test_pos_irq_irqSetCfg_safeOut1RdbkErr_mask(void)
{
    irqTest_maskTest(PMIC_SAFE_OUT1_RDBK_ERR_INT);
}

void test_pos_irq_irqSetCfg_enOutRdbkErr_mask(void)
{
    irqTest_maskTest(PMIC_EN_OUT_RDBK_ERR_INT);
}

void test_pos_irq_irqSetCfg_gpo1RdbkErr_mask(void)
{
    irqTest_maskTest(PMIC_GPO1_RDBK_ERR_INT);
}

void test_pos_irq_irqSetCfg_gpo2RdbkErr_mask(void)
{
    irqTest_maskTest(PMIC_GPO2_RDBK_ERR_INT);
}

void test_pos_irq_irqSetCfg_gpo3RdbkErr_mask(void)
{
    irqTest_maskTest(PMIC_GPO3_RDBK_ERR_INT);
}

void test_pos_irq_irqSetCfg_gpo4RdbkErr_mask(void)
{
    irqTest_maskTest(PMIC_GPO4_RDBK_ERR_INT);
}

/* Comparator 1 IRQ Tests */
void test_pos_irq_irqSetCfg_comp1pUvErr_mask(void)
{
    irqTest_maskTest(PMIC_COMP1P_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_comp1pOvErr_mask(void)
{
    irqTest_maskTest(PMIC_COMP1P_OV_ERR_INT);
}

void test_pos_irq_irqSetCfg_comp1nUvErr_mask(void)
{
    irqTest_maskTest(PMIC_COMP1N_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_comp1nOvErr_mask(void)
{
    irqTest_maskTest(PMIC_COMP1N_OV_ERR_INT);
}

/* Comparator 2 IRQ Tests */
void test_pos_irq_irqSetCfg_comp2pUvErr_mask(void)
{
    irqTest_maskTest(PMIC_COMP2P_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_comp2pOvErr_mask(void)
{
    irqTest_maskTest(PMIC_COMP2P_OV_ERR_INT);
}

void test_pos_irq_irqSetCfg_comp2nUvErr_mask(void)
{
    irqTest_maskTest(PMIC_COMP2N_UV_ERR_INT);
}

void test_pos_irq_irqSetCfg_comp2nOvErr_mask(void)
{
    irqTest_maskTest(PMIC_COMP2N_OV_ERR_INT);
}

/* Multiple IRQ Configuration Test */
void test_pos_irq_irqSetCfgs_multipleMasks(void)
{
    Pmic_IrqCfg_t setCfgs[5];
    Pmic_IrqCfg_t getCfgs[5];
    int32_t status;
    uint8_t i;

    memset(setCfgs, 0, sizeof(setCfgs));
    memset(getCfgs, 0, sizeof(getCfgs));

    /* Configure multiple IRQ masks */
    setCfgs[0].validParams = PMIC_IRQ_CFG_MASK_VALID;
    setCfgs[0].irqNum = PMIC_BB_UV_ERR_INT;
    setCfgs[0].mask = true;

    setCfgs[1].validParams = PMIC_IRQ_CFG_MASK_VALID;
    setCfgs[1].irqNum = PMIC_LDO1_UV_ERR_INT;
    setCfgs[1].mask = false;

    setCfgs[2].validParams = PMIC_IRQ_CFG_MASK_VALID;
    setCfgs[2].irqNum = PMIC_WD_TH1_ERR_INT;
    setCfgs[2].mask = true;

    setCfgs[3].validParams = PMIC_IRQ_CFG_MASK_VALID;
    setCfgs[3].irqNum = PMIC_ESM_DLY1_ERR_INT;
    setCfgs[3].mask = false;

    setCfgs[4].validParams = PMIC_IRQ_CFG_MASK_VALID;
    setCfgs[4].irqNum = PMIC_COMP1P_UV_ERR_INT;
    setCfgs[4].mask = true;

    /* Set multiple IRQ configurations */
    status = Pmic_irqSetCfgs(&g_pmicHandle, 5, setCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Prepare getCfgs with same IRQ numbers */
    for (i = 0; i < 5; i++)
    {
        getCfgs[i].validParams = PMIC_IRQ_CFG_MASK_VALID;
        getCfgs[i].irqNum = setCfgs[i].irqNum;
    }

    /* Get multiple IRQ configurations */
    status = Pmic_irqGetCfgs(&g_pmicHandle, 5, getCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify all mask values */
    for (i = 0; i < 5; i++)
    {
        PLATFORM_ASSERT(getCfgs[i].mask == setCfgs[i].mask);
    }
}

/* IRQ Status and Flag Tests */
void test_pos_irq_irqGetStatus_allIrqs(void)
{
    Pmic_IrqStatus_t irqStat;
    int32_t status;

    memset(&irqStat, 0, sizeof(irqStat));

    /* Get status of all IRQs */
    status = Pmic_irqGetStatus(&g_pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetNextFlag_singleFlag(void)
{
    Pmic_IrqStatus_t irqStat;
    uint8_t irqNum;
    int32_t status;

    memset(&irqStat, 0, sizeof(irqStat));

    /* Get all IRQ statuses first */
    status = Pmic_irqGetStatus(&g_pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get next flag (may return PMIC_ST_WARN_NO_IRQ_REMAINING if no flags set) */
    status = Pmic_irqGetNextFlag(&g_pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) ||
                    (status == PMIC_ST_WARN_NO_IRQ_REMAINING));
}

void test_pos_irq_irqGetFlag_andClrFlag(void)
{
    bool flag;
    int32_t status;
    uint8_t testIrq = PMIC_BB_UV_ERR_INT;

    /* Get flag status */
    status = Pmic_irqGetFlag(&g_pmicHandle, testIrq, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear the flag */
    status = Pmic_irqClrFlag(&g_pmicHandle, testIrq);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify flag is cleared */
    status = Pmic_irqGetFlag(&g_pmicHandle, testIrq, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(flag == false);
}

void test_pos_irq_irqClrAllFlags_basic(void)
{
    int32_t status;

    /* Clear all IRQ flags */
    status = Pmic_irqClrAllFlags(&g_pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                         NEGATIVE TEST CASES                                */
/* ========================================================================== */

/* Pmic_irqSetCfg Negative Tests */
void test_neg_irq_irqSetCfg_nullParam_handle(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_BB_UV_ERR_INT;
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(NULL, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqSetCfg_nullParam_irqCfg(void)
{
    int32_t status;

    status = Pmic_irqSetCfg(&g_pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqSetCfg_invalidParam_irqNum(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_IRQ_MAX + 1U;  /* Invalid IRQ number */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* Pmic_irqSetCfgs Negative Tests */
void test_neg_irq_irqSetCfgs_nullParam_handle(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_BB_UV_ERR_INT;

    status = Pmic_irqSetCfgs(NULL, 1, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqSetCfgs_nullParam_irqCfg(void)
{
    int32_t status;

    status = Pmic_irqSetCfgs(&g_pmicHandle, 1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqSetCfgs_invalidParam_numIrqs(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_BB_UV_ERR_INT;

    /* Test with numIrqs > PMIC_IRQ_NUM */
    status = Pmic_irqSetCfgs(&g_pmicHandle, PMIC_IRQ_NUM + 1U, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* Pmic_irqGetCfg Negative Tests */
void test_neg_irq_irqGetCfg_nullParam_handle(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_BB_UV_ERR_INT;

    status = Pmic_irqGetCfg(NULL, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetCfg_nullParam_irqCfg(void)
{
    int32_t status;

    status = Pmic_irqGetCfg(&g_pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetCfg_invalidParam_irqNum(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_IRQ_MAX + 1U;  /* Invalid IRQ number */

    status = Pmic_irqGetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* Pmic_irqGetCfgs Negative Tests */
void test_neg_irq_irqGetCfgs_nullParam_handle(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_BB_UV_ERR_INT;

    status = Pmic_irqGetCfgs(NULL, 1, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetCfgs_nullParam_irqCfgs(void)
{
    int32_t status;

    status = Pmic_irqGetCfgs(&g_pmicHandle, 1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetCfgs_invalidParam_numIrqs(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_BB_UV_ERR_INT;

    /* Test with numIrqs > PMIC_IRQ_NUM */
    status = Pmic_irqGetCfgs(&g_pmicHandle, PMIC_IRQ_NUM + 1U, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* Pmic_irqGetStatus Negative Tests */
void test_neg_irq_irqGetStatus_nullParam_handle(void)
{
    Pmic_IrqStatus_t irqStat;
    int32_t status;

    memset(&irqStat, 0, sizeof(irqStat));

    status = Pmic_irqGetStatus(NULL, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetStatus_nullParam_irqStat(void)
{
    int32_t status;

    status = Pmic_irqGetStatus(&g_pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* Pmic_irqGetNextFlag Negative Tests */
void test_neg_irq_irqGetNextFlag_nullParam_irqStat(void)
{
    uint8_t irqNum;
    int32_t status;

    status = Pmic_irqGetNextFlag(NULL, NULL, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetNextFlag_nullParam_irqNum(void)
{
    Pmic_IrqStatus_t irqStat;
    int32_t status;

    memset(&irqStat, 0, sizeof(irqStat));

    status = Pmic_irqGetNextFlag(NULL, &irqStat, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* Pmic_irqGetFlag Negative Tests */
void test_neg_irq_irqGetFlag_nullParam_handle(void)
{
    bool flag;
    int32_t status;

    status = Pmic_irqGetFlag(NULL, PMIC_BB_UV_ERR_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetFlag_nullParam_flag(void)
{
    int32_t status;

    status = Pmic_irqGetFlag(&g_pmicHandle, PMIC_BB_UV_ERR_INT, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetFlag_invalidParam_irqNum(void)
{
    bool flag;
    int32_t status;

    status = Pmic_irqGetFlag(&g_pmicHandle, PMIC_IRQ_MAX + 1U, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* Pmic_irqClrFlag Negative Tests */
void test_neg_irq_irqClrFlag_nullParam_handle(void)
{
    int32_t status;

    status = Pmic_irqClrFlag(NULL, PMIC_BB_UV_ERR_INT);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqClrFlag_invalidParam_irqNum(void)
{
    int32_t status;

    status = Pmic_irqClrFlag(&g_pmicHandle, PMIC_IRQ_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* Pmic_irqClrAllFlags Negative Tests */
void test_neg_irq_irqClrAllFlags_nullParam_handle(void)
{
    int32_t status;

    status = Pmic_irqClrAllFlags(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* Non-maskable IRQ Test */
void test_neg_irq_irqSetCfg_nonMaskableIrq(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_NORMAL_OFF_INT;  /* Non-maskable interrupt */
    irqCfg.mask = true;

    /* Attempting to mask a non-maskable IRQ should fail */
    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/* ========================================================================== */
/*                         NEW TESTS - Task 1B                                */
/* ========================================================================== */

/* 1. Non-configurable IRQ Tests (2 tests) */

void test_neg_irq_irqSetCfg_nonConfigurableIrq_ABIST_ERR(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfg.irqNum = PMIC_ABIST_ERR_INT;  /* Non-configurable interrupt */
    irqCfg.config = PMIC_IRQ_CONFIG0_INT_SET;

    /* Attempting to configure a non-configurable IRQ should fail */
    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqGetCfg_nonConfigurableIrq(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfg.irqNum = PMIC_NORMAL_OFF_INT;  /* Non-configurable interrupt */

    /* Attempting to get config of a non-configurable IRQ should fail */
    status = Pmic_irqGetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/* 2. Config validation tests (4 tests) */

void test_neg_irq_irqSetCfg_invalidConfig_cfgRegCrcErr(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfg.irqNum = PMIC_CFG_REG_CRC_ERR_INT;  /* CONFIG1_MAX validation */
    irqCfg.config = PMIC_IRQ_CONFIG1_MAX + 1U;  /* Invalid config value */

    /* Config value exceeds CONFIG1_MAX should fail */
    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqSetCfg_invalidConfig_comp1pUvErr(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfg.irqNum = PMIC_COMP1P_UV_ERR_INT;  /* CONFIG2_MAX validation */
    irqCfg.config = PMIC_IRQ_CONFIG2_MAX + 1U;  /* Invalid config value */

    /* Config value exceeds CONFIG2_MAX should fail */
    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqSetCfg_invalidConfig_comp2nOvErr(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfg.irqNum = PMIC_COMP2N_OV_ERR_INT;  /* CONFIG2_MAX validation */
    irqCfg.config = PMIC_IRQ_CONFIG2_MAX + 1U;  /* Invalid config value */

    /* Config value exceeds CONFIG2_MAX should fail */
    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqSetCfg_invalidConfig_otherIrq(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfg.irqNum = PMIC_BB_UV_ERR_INT;  /* CONFIG0_MAX validation */
    irqCfg.config = PMIC_IRQ_CONFIG0_MAX + 1U;  /* Invalid config value */

    /* Config value exceeds CONFIG0_MAX should fail */
    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* 3. Batch configuration tests (2 tests) */

void test_neg_irq_irqSetCfgs_invalidConfigInBatch(void)
{
    Pmic_IrqCfg_t irqCfgs[3];
    int32_t status;

    memset(irqCfgs, 0, sizeof(irqCfgs));

    /* First element: valid */
    irqCfgs[0].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[0].irqNum = PMIC_BB_UV_ERR_INT;
    irqCfgs[0].config = PMIC_IRQ_CONFIG0_INT_SET;

    /* Second element: invalid config value (triggers error) */
    irqCfgs[1].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[1].irqNum = PMIC_LDO1_UV_ERR_INT;
    irqCfgs[1].config = PMIC_IRQ_CONFIG0_MAX + 1U;  /* Invalid */

    /* Third element: valid (should not be processed) */
    irqCfgs[2].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[2].irqNum = PMIC_LDO2_UV_ERR_INT;
    irqCfgs[2].config = PMIC_IRQ_CONFIG0_INT_SET;

    /* Batch operation should fail on second element */
    status = Pmic_irqSetCfgs(&g_pmicHandle, 3, irqCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqGetCfgs_nonConfigurableInBatch(void)
{
    Pmic_IrqCfg_t irqCfgs[3];
    int32_t status;

    memset(irqCfgs, 0, sizeof(irqCfgs));

    /* First element: configurable IRQ */
    irqCfgs[0].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[0].irqNum = PMIC_BB_UV_ERR_INT;

    /* Second element: non-configurable IRQ (triggers error) */
    irqCfgs[1].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[1].irqNum = PMIC_ABIST_ERR_INT;  /* Non-configurable */

    /* Third element: configurable IRQ (should not be processed) */
    irqCfgs[2].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[2].irqNum = PMIC_LDO1_UV_ERR_INT;

    /* Batch operation should fail on second element */
    status = Pmic_irqGetCfgs(&g_pmicHandle, 3, irqCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/* 4. Special register tests (3 tests) */

void test_pos_irq_irqClrFlag_offStateStat1Register(void)
{
#ifdef BUILD_MOCK
    int32_t status;

    /* Set a flag in OFF_STATE_STAT1 register using test injection */
    status = testInject_setBits(OFF_STATE_STAT1_REG, 0x01U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear the flag - this exercises the special OFF_STATE_CLR path */
    /* The driver should write to OFF_STATE_CLR_REG, not directly to OFF_STATE_STAT1 */
    status = Pmic_irqClrFlag(&g_pmicHandle, PMIC_NORMAL_OFF_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

void test_pos_irq_irqClrFlag_offStateStat2Register(void)
{
#ifdef BUILD_MOCK
    int32_t status;

    /* Set a flag in OFF_STATE_STAT2 register using test injection */
    status = testInject_setBits(OFF_STATE_STAT2_REG, 0x01U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear the flag - this exercises the special OFF_STATE_CLR path */
    /* The driver should write to OFF_STATE_CLR_REG, not directly to OFF_STATE_STAT2 */
    status = Pmic_irqClrFlag(&g_pmicHandle, PMIC_CRC_ERR_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

void test_pos_irq_irqClrAllFlags_devErrStatPreservation(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint8_t regValue = 0U;

    /* Set DEV_ERR_CNT to a non-zero value and SAFE_ST_TMO_RST_ERR flag */
    /* First, set the entire register value */
    /* DEV_ERR_CNT=0x17 (bits 4:0), SAFE_ST_TMO_RST_ERR=1 (bit 6) */
    /* 0x57 = 0b01010111 */
    status = testInject_setBits(DEV_ERR_STAT_REG, 0x57U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear all flags */
    status = Pmic_irqClrAllFlags(&g_pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back DEV_ERR_STAT register to verify DEV_ERR_CNT is preserved */
    status = Pmic_ioRxByte(&g_pmicHandle, DEV_ERR_STAT_REG, &regValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify DEV_ERR_CNT field (bits 4:0 with mask 0x1F) is preserved */
    /* After clear, the SAFE_ST_TMO_RST_ERR bit should be cleared, but DEV_ERR_CNT preserved */
    PLATFORM_ASSERT((regValue & DEV_ERR_CNT_MASK) == 0x17U);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/* 5. Non-maskable IRQ tests (9 tests) */

void test_neg_irq_irqSetCfg_maskNonMaskable_OFF_INT_EVT_ERR_INT(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_OFF_INT_EVT_ERR_INT;  /* Non-maskable */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqSetCfg_maskNonMaskable_OFF_PROT_EVT_INT(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_OFF_PROT_EVT_INT;  /* Non-maskable */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqSetCfg_maskNonMaskable_FIRST_PWR_ON_INT(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_FIRST_PWR_ON_INT;  /* Non-maskable */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqSetCfg_maskNonMaskable_CLK_ERR_INT(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_CLK_ERR_INT;  /* Non-maskable */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqSetCfg_maskNonMaskable_INTERNAL_OV_INT(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_INTERNAL_OV_INT;  /* Non-maskable */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqSetCfg_maskNonMaskable_INIT_AN_TMO_INT(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_INIT_AN_TMO_INT;  /* Non-maskable */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqSetCfg_maskNonMaskable_WD_TMO_INT(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_WD_TMO_INT;  /* Non-maskable */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqSetCfg_maskNonMaskable_WD_TRIG_EARLY_INT(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_WD_TRIG_EARLY_INT;  /* Non-maskable */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqSetCfg_maskNonMaskable_ESM_ERR_INT(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_ESM_ERR_INT;  /* Non-maskable */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/* ========================================================================== */
/*                    Additional Coverage Tests                               */
/* ========================================================================== */

/**
 * @brief Test IRQ_getNextFlag with multiple interrupt flags set
 *
 * Covers IRQ_getNextFlag() internal helper (lines 590-624 in pmic_irq.c)
 * and IRQ_setIntrStat() (lines 567-578 in pmic_irq.c)
 */
void test_pos_irq_irqGetNextFlag_multipleFlagsSet(void)
{
#ifdef BUILD_MOCK
    Pmic_IrqStatus_t irqStat;
    uint8_t irqNum;
    int32_t status;
    int flagCount = 0;

    memset(&irqStat, 0, sizeof(irqStat));

    /* Inject multiple interrupt flags across different registers */
    /* BB_UV_ERR_INT is in DCDC_STAT_REG */
    status = testInject_setBits(DCDC_STAT_REG, TEST_MASK_LOW_NIBBLE);  /* Set multiple buck flags */
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* LDO1_UV_ERR_INT is in VMON_LDO_STAT_REG */
    status = testInject_setBits(VMON_LDO_STAT_REG, 0x03U);  /* Set multiple LDO flags */
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get all IRQ statuses - this will call IRQ_setIntrStat() for each set bit */
    status = Pmic_irqGetStatus(&g_pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Now call Pmic_irqGetNextFlag multiple times to iterate through flags */
    /* This will exercise IRQ_getNextFlag() internal helper */
    do {
        status = Pmic_irqGetNextFlag(&g_pmicHandle, &irqStat, &irqNum);
        if (status == PMIC_ST_SUCCESS) {
            flagCount++;
            PLATFORM_ASSERT(irqNum < PMIC_IRQ_MAX);
        }
    } while (status == PMIC_ST_SUCCESS && flagCount < 10);

    /* Verify we found some flags */
    PLATFORM_ASSERT(flagCount > 0);

    /* Final call should return NO_IRQ_REMAINING */
    status = Pmic_irqGetNextFlag(&g_pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test IRQ configuration with config values (not just mask)
 *
 * Covers IRQ_anyConfsForReg() path (lines 867-879 in pmic_irq.c)
 */
void test_pos_irq_irqSetCfg_configValue(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));

    /* Set config value for a configurable IRQ */
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfg.irqNum = PMIC_BB_UV_ERR_INT;
    irqCfg.config = PMIC_IRQ_CONFIG0_INT_SET;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back the config */
    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfg.irqNum = PMIC_BB_UV_ERR_INT;

    status = Pmic_irqGetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqCfg.config == PMIC_IRQ_CONFIG0_INT_SET);
}

/**
 * @brief Test batch IRQ configuration with multiple config values
 *
 * Covers IRQ_anyConfsForReg() batch processing path (lines 867-879 in pmic_irq.c)
 */
void test_pos_irq_irqSetCfgs_batchConfigValues(void)
{
    Pmic_IrqCfg_t irqCfgs[3];
    int32_t status;

    memset(irqCfgs, 0, sizeof(irqCfgs));

    /* First IRQ: BB_UV_ERR_INT with config */
    irqCfgs[0].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[0].irqNum = PMIC_BB_UV_ERR_INT;
    irqCfgs[0].config = PMIC_IRQ_CONFIG0_INT_SET;

    /* Second IRQ: BB_OV_ERR_INT with config (same register as BB_UV) */
    irqCfgs[1].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[1].irqNum = PMIC_BB_OV_ERR_INT;
    irqCfgs[1].config = PMIC_IRQ_CONFIG0_SET_GOTO_SAFE;

    /* Third IRQ: LDO1_UV_ERR_INT with config (different register) */
    irqCfgs[2].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[2].irqNum = PMIC_LDO1_UV_ERR_INT;
    irqCfgs[2].config = PMIC_IRQ_CONFIG1_INT_SET_GOTO_SAFE;

    /* Set all configs in batch - this exercises IRQ_anyConfsForReg */
    status = Pmic_irqSetCfgs(&g_pmicHandle, 3, irqCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify first config */
    memset(irqCfgs, 0, sizeof(irqCfgs));
    irqCfgs[0].validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfgs[0].irqNum = PMIC_BB_UV_ERR_INT;

    status = Pmic_irqGetCfg(&g_pmicHandle, &irqCfgs[0]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqCfgs[0].config == PMIC_IRQ_CONFIG0_INT_SET);
}

/**
 * @brief Test invalid IRQ number in IRQ_setMask
 *
 * Covers lines 628-630 in pmic_irq.c
 */
void test_neg_irq_irqSetCfg_invalidIrqNum_mask(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_IRQ_MAX + 1;  /* Invalid IRQ number */
    irqCfg.mask = true;

    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test invalid IRQ number via CONFIG validParam (calls setMask)
 *
 * Covers lines 629-630 in pmic_irq.c - IRQ_setMask path via setConfig
 * When CONFIG validParam is set along with MASK, setConfig calls setMask
 */
void test_neg_irq_irqSetCfg_invalidIrqNum_viaMask(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    // Set both CONFIG and MASK valid params to trigger both paths
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID | PMIC_IRQ_CFG_MASK_VALID;
    irqCfg.irqNum = PMIC_IRQ_MAX + 1;  /* Invalid IRQ number */
    irqCfg.config = PMIC_IRQ_CONFIG0_INT_SET;
    irqCfg.mask = true;

    // This will call IRQ_setConfig first which checks irqNum at line 664-665
    // Then it would call IRQ_setMask which checks irqNum at line 628-630
    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test invalid IRQ number in IRQ_setConfig
 *
 * Covers lines 665-666 in pmic_irq.c - IRQ_setConfig validation
 */
void test_neg_irq_irqSetCfg_invalidIrqNum_config(void)
{
    Pmic_IrqCfg_t irqCfg;
    int32_t status;

    memset(&irqCfg, 0, sizeof(irqCfg));
    irqCfg.validParams = PMIC_IRQ_CFG_CONFIG_VALID;
    irqCfg.irqNum = PMIC_IRQ_MAX + 1;  /* Invalid IRQ number */
    irqCfg.config = PMIC_IRQ_CONFIG0_INT_SET;

    // This will directly call IRQ_setConfig which checks irqNum at line 664-666
    status = Pmic_irqSetCfg(&g_pmicHandle, &irqCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_irqClrAllFlags with I/O failure mid-loop
 *
 * Covers line 1196 in pmic_irq.c - error handling break in register clear loop.
 * This test injects an I/O error that occurs during the loop iteration, causing
 * the break statement to execute.
 */
void test_pos_irq_irqClrAllFlags_ioFailureMidLoop(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    /* Inject 1 I/O error - will fail on the 2nd register write in the loop */
    status = PmicMock_InjectError(mockDevice,
                                   PMIC_MOCK_ERROR_COMM_FAILURE,
                                   1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    /* Call Pmic_irqClrAllFlags - will start loop, succeed once, then fail */
    /* This will hit the break at line 1196 when the second write fails */
    status = Pmic_irqClrAllFlags(&g_pmicHandle);

    /* Verify that operation failed due to injected error */
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    /* Test requires mock support for error injection */
    PLATFORM_ASSERT(true);
#endif
}

/* ========================================================================== */
/*                            Test Setup/Teardown                             */
/* ========================================================================== */

/**
 * @brief IRQ test suite entry point (wrapper for test runner)
 * @param args Test arguments (unused)
 *
 * Note: This module doesn't define setUp/tearDown at global scope to avoid
 * conflicts with other modules. Instead, it handles initialization internally.
 */
void irq_test(void *args)
{
    int32_t status;
    (void)args;  /* Unused parameter */

    /* Initialize once for all IRQ tests */
    platform_init();
    testTimer_startModule("IRQ");
    status = irqTest_initHandle();
    if (status != PMIC_ST_SUCCESS)
    {
        printf("ERROR: IRQ test initialization failed with status: %d\r\n", status);
        platform_deinit();
        return;
    }
    /* Run all IRQ tests */
    platform_unlockRegisters();
    platform_setupTests();
    IRQ_TEST_RUN_ALL();
    platform_tearDownTests();

    /* Cleanup */
    testTimer_endModule();
    Pmic_deinit(&g_pmicHandle);
    platform_deinit();

}
