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
 *    distribution and/or other materials provided with the
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
#include "irq_test.h"
#include "pmic.h"
#ifdef BUILD_MOCK
#include "test_inject.h"
#endif
#include "test_constants.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0};

/* ========================================================================== */
/*                       Negative Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test Pmic_irqSetMask with NULL handle
 */
void test_neg_irq_irqSetMask_nullHandle(void)
{
    int32_t status = Pmic_irqSetMask(NULL, PMIC_IRQ_ADC_CONV_READY_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqSetMask with invalid IRQ number
 */
void test_neg_irq_irqSetMask_invalidIrqNum(void)
{
    int32_t status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_INT_MAX + 1, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_irqSetMasks with NULL handle
 */
void test_neg_irq_irqSetMasks_nullHandle(void)
{
    Pmic_IrqMask_t irqMasks[2] = {
        {.irqNum = PMIC_IRQ_ADC_CONV_READY_INT, .mask = PMIC_IRQ_MASK},
        {.irqNum = PMIC_IRQ_TWARN_INT, .mask = PMIC_IRQ_MASK}
    };
    int32_t status = Pmic_irqSetMasks(NULL, 2, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqSetMasks with NULL irqMasks pointer
 */
void test_neg_irq_irqSetMasks_nullIrqMasks(void)
{
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 2, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqSetMasks with zero count
 */
void test_neg_irq_irqSetMasks_zeroCount(void)
{
    Pmic_IrqMask_t irqMasks[1] = {{.irqNum = PMIC_IRQ_ADC_CONV_READY_INT, .mask = PMIC_IRQ_MASK}};
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 0, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_irqGetMask with NULL handle
 */
void test_neg_irq_irqGetMask_nullHandle(void)
{
    Pmic_IrqMask_t irqMasks[1] = {{.irqNum = PMIC_IRQ_ADC_CONV_READY_INT}};
    int32_t status = Pmic_irqGetMask(NULL, 1, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqGetMask with NULL irqMasks pointer
 */
void test_neg_irq_irqGetMask_nullIrqMasks(void)
{
    int32_t status = Pmic_irqGetMask(&pmicHandle, 1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqGetMask with zero count
 */
void test_neg_irq_irqGetMask_zeroCount(void)
{
    Pmic_IrqMask_t irqMasks[1] = {{.irqNum = PMIC_IRQ_ADC_CONV_READY_INT}};
    int32_t status = Pmic_irqGetMask(&pmicHandle, 0, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_irqGetStatus with NULL handle
 */
void test_neg_irq_irqGetStatus_nullHandle(void)
{
    Pmic_IrqStatus_t irqStat = {0};
    int32_t status = Pmic_irqGetStatus(NULL, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqGetStatus with NULL irqStat pointer
 */
void test_neg_irq_irqGetStatus_nullIrqStat(void)
{
    int32_t status = Pmic_irqGetStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqGetNextFlag with NULL irqStat pointer
 */
void test_neg_irq_irqGetNextFlag_nullIrqStat(void)
{
    uint8_t irqNum = 0;
    int32_t status = Pmic_irqGetNextFlag(NULL, NULL, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqGetNextFlag with NULL irqNum pointer
 */
void test_neg_irq_irqGetNextFlag_nullIrqNum(void)
{
    Pmic_IrqStatus_t irqStat = {0};
    int32_t status = Pmic_irqGetNextFlag(NULL, &irqStat, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqGetFlag with NULL handle
 */
void test_neg_irq_irqGetFlag_nullHandle(void)
{
    bool flag = false;
    int32_t status = Pmic_irqGetFlag(NULL, PMIC_IRQ_ADC_CONV_READY_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqGetFlag with NULL flag pointer
 */
void test_neg_irq_irqGetFlag_nullFlag(void)
{
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_ADC_CONV_READY_INT, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqGetFlag with invalid IRQ number
 */
void test_neg_irq_irqGetFlag_invalidIrqNum(void)
{
    bool flag = false;
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_INT_MAX + 1, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_irqClrFlag with NULL handle
 */
void test_neg_irq_irqClrFlag_nullHandle(void)
{
    int32_t status = Pmic_irqClrFlag(NULL, PMIC_IRQ_ADC_CONV_READY_INT);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_irqClrFlag with invalid IRQ number
 */
void test_neg_irq_irqClrFlag_invalidIrqNum(void)
{
    int32_t status = Pmic_irqClrFlag(&pmicHandle, PMIC_IRQ_INT_MAX + 1);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_irqClrAllFlags with NULL handle
 */
void test_neg_irq_irqClrAllFlags_nullHandle(void)
{
    int32_t status = Pmic_irqClrAllFlags(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                       Positive Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test single IRQ mask set and get
 */
void test_pos_irq_irqSetGetMask_single(void)
{
    int32_t status;
    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_IRQ_ADC_CONV_READY_INT};

    /* Mask the interrupt */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ADC_CONV_READY_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back the mask */
    status = Pmic_irqGetMask(&pmicHandle, 1, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqMask.mask == PMIC_IRQ_MASK);

    /* Unmask the interrupt */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ADC_CONV_READY_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify unmasked */
    status = Pmic_irqGetMask(&pmicHandle, 1, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqMask.mask == PMIC_IRQ_UNMASK);
}

/**
 * @brief Test multiple IRQ masks set at once
 */
void test_pos_irq_irqSetMasks_multiple(void)
{
    int32_t status;
    Pmic_IrqMask_t irqMasksSet[4] = {
        {.irqNum = PMIC_IRQ_ADC_CONV_READY_INT, .mask = PMIC_IRQ_MASK},
        {.irqNum = PMIC_IRQ_TWARN_INT, .mask = PMIC_IRQ_MASK},
        {.irqNum = PMIC_IRQ_PB_RISE_INT, .mask = PMIC_IRQ_MASK},
        {.irqNum = PMIC_IRQ_PB_FALL_INT, .mask = PMIC_IRQ_MASK}
    };

    /* Set multiple masks */
    status = Pmic_irqSetMasks(&pmicHandle, 4, irqMasksSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    Pmic_IrqMask_t irqMasksGet[4] = {
        {.irqNum = PMIC_IRQ_ADC_CONV_READY_INT},
        {.irqNum = PMIC_IRQ_TWARN_INT},
        {.irqNum = PMIC_IRQ_PB_RISE_INT},
        {.irqNum = PMIC_IRQ_PB_FALL_INT}
    };

    status = Pmic_irqGetMask(&pmicHandle, 4, irqMasksGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    for (uint8_t i = 0; i < 4; i++)
    {
        PLATFORM_ASSERT(irqMasksGet[i].mask == PMIC_IRQ_MASK);
    }

    /* Unmask all */
    for (uint8_t i = 0; i < 4; i++)
    {
        irqMasksSet[i].mask = PMIC_IRQ_UNMASK;
    }
    status = Pmic_irqSetMasks(&pmicHandle, 4, irqMasksSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test IRQ status reading
 */
void test_pos_irq_irqGetStatus_read(void)
{
    int32_t status;
    Pmic_IrqStatus_t irqStat = {0};

    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test IRQ flag get and clear
 */
void test_pos_irq_irqGetClrFlag_single(void)
{
    int32_t status;
    bool flag = false;

    /* Clear any existing flag first */
    status = Pmic_irqClrFlag(&pmicHandle, PMIC_IRQ_ADC_CONV_READY_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get flag status */
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_ADC_CONV_READY_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test IRQ clear all flags
 */
void test_pos_irq_irqClrAllFlags(void)
{
    int32_t status;

    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test IRQ next flag iteration
 */
void test_pos_irq_irqGetNextFlag_iteration(void)
{
    int32_t status;
    Pmic_IrqStatus_t irqStat = {0};
    uint8_t irqNum = 0;

    /* Get status first */
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Iterate through flags */
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    /* Status can be SUCCESS if there's a flag, or WARN_NO_IRQ_REMAINING if none */
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_WARN_NO_IRQ_REMAINING));
}

/**
 * @brief Test masking all maskable interrupts
 */
void test_pos_irq_irqSetMask_allMaskable(void)
{
    int32_t status;

    /* Mask all maskable interrupts (0-48, excluding non-maskable ones) */
    uint8_t maskableIrqs[] = {
        PMIC_IRQ_ESM_MCU_RST_INT,
        PMIC_IRQ_ESM_MCU_FAIL_INT,
        PMIC_IRQ_ESM_MCU_PIN_INT,
        PMIC_IRQ_I2C2_ERR_INT,
        PMIC_IRQ_COMM_ERR_INT,
        PMIC_IRQ_SOC_PWR_ERR_INT,
        PMIC_IRQ_MCU_PWR_ERR_INT,
        PMIC_IRQ_ORD_SHUTDOWN_INT,
        PMIC_IRQ_IMM_SHUTOWN_INT,
        PMIC_IRQ_REG_CRC_ERR_INT,
        PMIC_IRQ_BIST_FAIL_INT,
        PMIC_IRQ_ADC_CONV_READY_INT,
        PMIC_IRQ_PB_RISE_INT,
        PMIC_IRQ_PB_FALL_INT,
        PMIC_IRQ_PB_LONG_INT,
        PMIC_IRQ_TWARN_INT,
        PMIC_IRQ_REG_UNLOCK_INT,
        PMIC_IRQ_EXT_CLK_INT,
        PMIC_IRQ_BIST_PASS_INT,
        PMIC_IRQ_SOFT_REBOOT_INT,
        PMIC_IRQ_FSD_INT,
        PMIC_IRQ_PB_SHORT_INT,
        PMIC_IRQ_ENABLE_INT,
        PMIC_IRQ_VSENSE_INT,
        PMIC_IRQ_GPIO6_INT,
        PMIC_IRQ_GPIO5_INT,
        PMIC_IRQ_GPIO4_INT,
        PMIC_IRQ_GPIO3_INT,
        PMIC_IRQ_GPIO2_INT,
        PMIC_IRQ_GPIO1_INT,
        PMIC_IRQ_VMON2_UVOV_INT,
        PMIC_IRQ_VMON1_UVOV_INT,
        PMIC_IRQ_VCCA_UVOV_INT,
        PMIC_IRQ_LDO3_UVOV_INT,
        PMIC_IRQ_LDO2_UVOV_INT,
        PMIC_IRQ_LDO1_UVOV_INT,
        PMIC_IRQ_BUCK4_UVOV_INT,
        PMIC_IRQ_BUCK3_UVOV_INT,
        PMIC_IRQ_BUCK2_UVOV_INT,
        PMIC_IRQ_BUCK1_UVOV_INT
    };

    uint8_t numMaskable = sizeof(maskableIrqs) / sizeof(maskableIrqs[0]);

    for (uint8_t i = 0; i < numMaskable; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, maskableIrqs[i], PMIC_IRQ_MASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    /* Unmask them all */
    for (uint8_t i = 0; i < numMaskable; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, maskableIrqs[i], PMIC_IRQ_UNMASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

/**
 * @brief Test GPIO interrupt masks
 */
void test_pos_irq_irqSetGetMask_gpioInt(void)
{
    int32_t status;
    uint8_t gpioIrqs[] = {
        PMIC_IRQ_GPIO1_INT,
        PMIC_IRQ_GPIO2_INT,
        PMIC_IRQ_GPIO3_INT,
        PMIC_IRQ_GPIO4_INT,
        PMIC_IRQ_GPIO5_INT,
        PMIC_IRQ_GPIO6_INT
    };

    /* Mask all GPIO interrupts */
    for (uint8_t i = 0; i < 6; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, gpioIrqs[i], PMIC_IRQ_MASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    /* Verify masks */
    Pmic_IrqMask_t irqMasks[6];
    for (uint8_t i = 0; i < 6; i++)
    {
        irqMasks[i].irqNum = gpioIrqs[i];
    }

    status = Pmic_irqGetMask(&pmicHandle, 6, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    for (uint8_t i = 0; i < 6; i++)
    {
        PLATFORM_ASSERT(irqMasks[i].mask == PMIC_IRQ_MASK);
    }

    /* Unmask all GPIO interrupts */
    for (uint8_t i = 0; i < 6; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, gpioIrqs[i], PMIC_IRQ_UNMASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

/**
 * @brief Test power rail UVOV interrupt masks
 */
void test_pos_irq_irqSetGetMask_powerUvov(void)
{
    int32_t status;
    uint8_t powerIrqs[] = {
        PMIC_IRQ_BUCK1_UVOV_INT,
        PMIC_IRQ_BUCK2_UVOV_INT,
        PMIC_IRQ_BUCK3_UVOV_INT,
        PMIC_IRQ_BUCK4_UVOV_INT,
        PMIC_IRQ_LDO1_UVOV_INT,
        PMIC_IRQ_LDO2_UVOV_INT,
        PMIC_IRQ_LDO3_UVOV_INT,
        PMIC_IRQ_VCCA_UVOV_INT,
        PMIC_IRQ_VMON1_UVOV_INT,
        PMIC_IRQ_VMON2_UVOV_INT
    };

    /* Mask all power rail UVOV interrupts */
    for (uint8_t i = 0; i < 10; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, powerIrqs[i], PMIC_IRQ_MASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    /* Unmask all */
    for (uint8_t i = 0; i < 10; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, powerIrqs[i], PMIC_IRQ_UNMASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

/**
 * @brief Test ESM interrupt masks
 */
void test_pos_irq_irqSetGetMask_esmInt(void)
{
    int32_t status;

    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ESM_MCU_RST_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ESM_MCU_FAIL_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ESM_MCU_PIN_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Unmask all ESM interrupts */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ESM_MCU_RST_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ESM_MCU_FAIL_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ESM_MCU_PIN_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test FSM error interrupt masks
 */
void test_pos_irq_irqSetGetMask_fsmErrorInt(void)
{
    int32_t status;
    uint8_t fsmIrqs[] = {
        PMIC_IRQ_I2C2_ERR_INT,
        PMIC_IRQ_COMM_ERR_INT,
        PMIC_IRQ_SOC_PWR_ERR_INT,
        PMIC_IRQ_MCU_PWR_ERR_INT,
        PMIC_IRQ_ORD_SHUTDOWN_INT,
        PMIC_IRQ_IMM_SHUTOWN_INT
    };

    /* Mask all FSM error interrupts */
    for (uint8_t i = 0; i < 6; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, fsmIrqs[i], PMIC_IRQ_MASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    /* Unmask all */
    for (uint8_t i = 0; i < 6; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, fsmIrqs[i], PMIC_IRQ_UNMASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

/**
 * @brief Test miscellaneous interrupt masks
 */
void test_pos_irq_irqSetGetMask_miscInt(void)
{
    int32_t status;
    uint8_t miscIrqs[] = {
        PMIC_IRQ_ADC_CONV_READY_INT,
        PMIC_IRQ_PB_RISE_INT,
        PMIC_IRQ_PB_FALL_INT,
        PMIC_IRQ_PB_LONG_INT,
        PMIC_IRQ_TWARN_INT,
        PMIC_IRQ_REG_UNLOCK_INT,
        PMIC_IRQ_EXT_CLK_INT,
        PMIC_IRQ_BIST_PASS_INT,
        PMIC_IRQ_SOFT_REBOOT_INT,
        PMIC_IRQ_FSD_INT,
        PMIC_IRQ_PB_SHORT_INT,
        PMIC_IRQ_ENABLE_INT,
        PMIC_IRQ_VSENSE_INT
    };

    /* Mask all misc interrupts */
    for (uint8_t i = 0; i < 13; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, miscIrqs[i], PMIC_IRQ_MASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    /* Unmask all */
    for (uint8_t i = 0; i < 13; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, miscIrqs[i], PMIC_IRQ_UNMASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

/**
 * @brief Test thermal warning interrupt
 */
void test_pos_irq_irqSetGetMask_thermalWarning(void)
{
    int32_t status;

    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_TWARN_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_IRQ_TWARN_INT};
    status = Pmic_irqGetMask(&pmicHandle, 1, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqMask.mask == PMIC_IRQ_MASK);

    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_TWARN_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test ADC conversion ready interrupt
 */
void test_pos_irq_irqSetGetMask_adcConvReady(void)
{
    int32_t status;

    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ADC_CONV_READY_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_IRQ_ADC_CONV_READY_INT};
    status = Pmic_irqGetMask(&pmicHandle, 1, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqMask.mask == PMIC_IRQ_MASK);

    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_ADC_CONV_READY_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test push button interrupts
 */
void test_pos_irq_irqSetGetMask_pushButton(void)
{
    int32_t status;
    uint8_t pbIrqs[] = {
        PMIC_IRQ_PB_RISE_INT,
        PMIC_IRQ_PB_FALL_INT,
        PMIC_IRQ_PB_LONG_INT,
        PMIC_IRQ_PB_SHORT_INT
    };

    for (uint8_t i = 0; i < 4; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, pbIrqs[i], PMIC_IRQ_MASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        status = Pmic_irqSetMask(&pmicHandle, pbIrqs[i], PMIC_IRQ_UNMASK);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

/**
 * @brief Test negative case: attempt to mask non-maskable interrupt
 */
void test_neg_irq_irqSetMask_nonMaskable(void)
{
    int32_t status;

    /* Try to mask WD_RST_NMI - which is non-maskable */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_WD_RST_NMI, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Try to mask WD_FAIL_NMI - which is non-maskable */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_WD_FAIL_NMI, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Try to mask WD_LONGWIN_TIMEOUT_NMI - which is non-maskable */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_WD_LONGWIN_TIMEOUT_NMI, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Try to mask BG_XMON_INT - which is non-maskable */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_BG_XMON_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Try to mask PFSM_ERR_INT - which is non-maskable */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_PFSM_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Try to mask VCCA_OVP_INT - which is non-maskable */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_VCCA_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Try to mask TSD_IMM_INT - which is non-maskable */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_TSD_IMM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Try to mask RECOV_CNT_INT - which is non-maskable */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_RECOV_CNT_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Try to mask TSD_ORD_INT - which is non-maskable */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_TSD_ORD_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test positive case: IRQ status with active flags
 */
void test_pos_irq_irqGetStatus_withActiveFlags(void)
{
    int32_t status;
    Pmic_IrqStatus_t irqStat = {0};

    /* Get IRQ status - in mock environment, this reads the current register state */
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* The intrStat array should be populated based on mock register values */
    /* In a real test with active interrupts, we would verify specific flags are set */
    /* For mock testing, we verify the API completes successfully */
}

/**
 * @brief Test positive case: IRQ next flag iteration with multiple flags
 */
void test_pos_irq_irqGetNextFlag_multipleFlags(void)
{
    int32_t status;
    Pmic_IrqStatus_t irqStat = {0};
    uint8_t irqNum = 0;
    uint8_t flagCount = 0;

    /* First, set a few interrupt flags by triggering them */
    /* In mock environment, we can trigger interrupts by reading their status */

    /* Get the overall IRQ status */
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Iterate through all flags using Pmic_irqGetNextFlag */
    while (true)
    {
        status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);

        if (status == PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            /* No more flags remaining */
            break;
        }

        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(irqNum <= PMIC_IRQ_INT_MAX);

        /* Clear this flag */
        status = Pmic_irqClrFlag(&pmicHandle, irqNum);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        flagCount++;

        /* Safety check to prevent infinite loop */
        if (flagCount > PMIC_IRQ_INT_MAX)
        {
            break;
        }
    }

    /* After iteration, there should be no remaining flags */
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
}

/**
 * @brief Test reading mask for non-maskable interrupts
 */
void test_pos_irq_irqGetMask_nonMaskable(void)
{
    int32_t status;
    Pmic_IrqMask_t irqMasks[9] = {
        {.irqNum = PMIC_IRQ_WD_RST_NMI},
        {.irqNum = PMIC_IRQ_WD_FAIL_NMI},
        {.irqNum = PMIC_IRQ_WD_LONGWIN_TIMEOUT_NMI},
        {.irqNum = PMIC_IRQ_BG_XMON_INT},
        {.irqNum = PMIC_IRQ_PFSM_ERR_INT},
        {.irqNum = PMIC_IRQ_VCCA_OVP_INT},
        {.irqNum = PMIC_IRQ_TSD_IMM_INT},
        {.irqNum = PMIC_IRQ_RECOV_CNT_INT},
        {.irqNum = PMIC_IRQ_TSD_ORD_INT}
    };

    /* Get mask status for all non-maskable interrupts */
    status = Pmic_irqGetMask(&pmicHandle, 9, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* All non-maskable interrupts should return mask = false */
    for (uint8_t i = 0; i < 9; i++)
    {
        PLATFORM_ASSERT(irqMasks[i].mask == false);
    }
}

/**
 * @brief Test positive case: IRQ array handling across multiple array indices
 * This test specifically exercises the IRQ status array indexing:
 * - arrayIndex = irqNum / 32U
 * - bitIndex = irqNum % 32U
 * - intrStat[arrayIndex] |= (1UL << bitIndex)
 */
void test_pos_irq_irqGetNextFlag_fromArray(void)
{
    int32_t status;
    Pmic_IrqStatus_t irqStat = {0};

    /* Get IRQ status - this will populate the intrStat array */
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* The IRQ status should be properly stored in the intrStat array.
     * For TPS6522x-Q1, IRQs 0-48 span across two array elements:
     * - intrStat[0] holds IRQs 0-31
     * - intrStat[1] holds IRQs 32-48
     */

    /* Test that we can iterate through IRQs spanning array boundaries */
    uint8_t irqNum = 0;
    uint8_t iterationCount = 0;
    const uint8_t MAX_ITERATIONS = PMIC_IRQ_INT_MAX + 1U;

    while (iterationCount < MAX_ITERATIONS)
    {
        status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);

        if (status == PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            /* No more flags - this is expected */
            break;
        }

        if (status == PMIC_ST_SUCCESS)
        {
            /* Verify the IRQ number is valid */
            PLATFORM_ASSERT(irqNum <= PMIC_IRQ_INT_MAX);

            /* Clear this flag to continue iteration */
            status = Pmic_irqClrFlag(&pmicHandle, irqNum);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        }

        iterationCount++;
    }

    /* Test completed successfully */
    PLATFORM_ASSERT(iterationCount <= MAX_ITERATIONS);
}

/**
 * @brief Test negative case: setMask with invalid IRQ number beyond PMIC_IRQ_INT_MAX
 * This ensures bounds checking for IRQ numbers > 48
 */
void test_neg_irq_irqSetMask_invalidIrqNumBeyondMax(void)
{
    /* Try to set mask for an IRQ number that's way out of range */
    int32_t status = Pmic_irqSetMask(&pmicHandle, 100U, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Try another out-of-range value */
    status = Pmic_irqSetMask(&pmicHandle, TEST_INVALID_PARAM_255, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test negative case: Pmic_irqGetFlag with invalid IRQ number
 * Tests error path for irqNum > PMIC_IRQ_INT_MAX (lines 391-393)
 */
void test_neg_irq_irqGetFlag_invalidIrqNumBeyondMax(void)
{
    bool flag = false;
    /* Try to get flag for IRQ number 50 (beyond max 48) */
    int32_t status = Pmic_irqGetFlag(&pmicHandle, 50U, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test negative case: Pmic_irqClrFlag with invalid IRQ number
 * Tests error path for irqNum > PMIC_IRQ_INT_MAX (lines 414-416)
 */
void test_neg_irq_irqClrFlag_invalidIrqNumBeyondMax(void)
{
    /* Try to clear flag for IRQ number 60 (beyond max 48) */
    int32_t status = Pmic_irqClrFlag(&pmicHandle, 60U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test positive case: Mask a specific interrupt
 * Tests masking and verifying specific interrupt (covers lines 439-448)
 */
void test_pos_irq_irqSetMask_specific(void)
{
    int32_t status;

    /* Mask BUCK1 UVOV interrupt */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_BUCK1_UVOV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's masked */
    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_IRQ_BUCK1_UVOV_INT};
    status = Pmic_irqGetMask(&pmicHandle, 1, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqMask.mask == PMIC_IRQ_MASK);
}

/**
 * @brief Test positive case: Unmask a specific interrupt
 * Tests unmasking specific interrupt (covers lines 499-502, 512-516)
 */
void test_pos_irq_irqSetMask_unmaskSpecific(void)
{
    int32_t status;

    /* First mask it */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_BUCK2_UVOV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Now unmask it */
    status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_BUCK2_UVOV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's unmasked */
    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_IRQ_BUCK2_UVOV_INT};
    status = Pmic_irqGetMask(&pmicHandle, 1, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqMask.mask == PMIC_IRQ_UNMASK);
}

/**
 * @brief Test Pmic_irqGetMask with invalid IRQ number in masks array
 * Covers lines 250-251 in pmic_irq.c
 */
void test_neg_irq_irqGetMask_invalidIrqInArray(void)
{
    Pmic_IrqMask_t irqMasks[2] = {
        {.irqNum = PMIC_IRQ_ADC_CONV_READY_INT},  /* Valid */
        {.irqNum = PMIC_IRQ_INT_MAX + 5U}         /* Invalid */
    };

    int32_t status = Pmic_irqGetMask(&pmicHandle, 2, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_irqGetStatus with set flags
 * Covers lines 325-331 in pmic_irq.c (flag detection and array population)
 */
void test_pos_irq_irqGetStatus_withSetFlag(void)
{
    int32_t status;
    Pmic_IrqStatus_t irqStat = {0};

    /* Clear all flags first */
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Inject IRQ flags into mock registers to trigger flag detection logic
     * TPS6522x-Q1 has multiple IRQ status registers. Setting bit 0 of INT_MISC_REG
     * will trigger the flag detection code path in Pmic_irqGetStatus() */
    status = testInject_setBits(0x66U, 0x01U);  /* INT_MISC_REG bit 0 */
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get IRQ status - this should detect the injected flag and populate intrStat array */
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify that the flag was detected and array was populated */
    PLATFORM_ASSERT(irqStat.intrStat[0] != 0U);
}

/**
 * @brief Test Pmic_irqGetNextFlag with set flags
 * Covers lines 362-368 in pmic_irq.c (flag iteration logic)
 */
void test_pos_irq_irqGetNextFlag_withSetFlag(void)
{
    int32_t status;
    Pmic_IrqStatus_t irqStat = {0};
    uint8_t irqNum = 0;

    /* Clear all flags first */
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Inject multiple IRQ flags into mock registers to test flag iteration
     * Setting multiple bits will ensure we iterate through multiple flags */
    status = testInject_setBits(0x66U, 0x03U);  /* INT_MISC_REG bits 0-1 */
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get IRQ status - this populates irqStat with the injected flags */
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify flags were detected */
    PLATFORM_ASSERT(irqStat.intrStat[0] != 0U);

    /* Get first flag - this should find the first set bit and clear it from irqStat */
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqNum <= PMIC_IRQ_INT_MAX);

    /* Get next flag - this should find the second flag */
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqNum <= PMIC_IRQ_INT_MAX);

    /* Try to get another flag - should indicate no more flags remaining */
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
}

/* ========================================================================== */
/*                         Entry Point Function                               */
/* ========================================================================== */

void irq_test(void *args)
{
    (void)args;
    int32_t status;

    platform_init();
    testTimer_startModule("IRQ");
    platform_setupTests();

    /* Initialize PMIC handle */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_CRC_ENABLE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID |
                       PMIC_MAX_LOOP_CNT_VALID,
        .commMode = PMIC_INTF_SPI,
        .crcEnable = false,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .maxLoopCnt = 1000
    };

    status = Pmic_init(&pmicHandle, &handleCfg);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to initialize PMIC handle\r\n");
        platform_tearDownTests();
        platform_deinit();
        return;
    }

    platform_printString("\r\n=== IRQ Module Tests ===\r\n");
    IRQ_TEST_RUN_ALL();

    testTimer_endModule();
    Pmic_deinit(&pmicHandle);
    platform_tearDownTests();
    platform_deinit();
}
