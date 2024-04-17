/****************************************************************************
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 ****************************************************************************/

/**
 *  @file  pmic_irq_test.c
 *
 *  @brief  This file contains all the testing related files APIs for the PMIC IRQ
 *          and basic register lock/unlock and other miscellaneous tests.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_irq_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                             */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_irq = NULL;

void GPIO_bankIsrFxn(void *args);
extern void Board_gpioInit(void);
extern void Board_gpioDeinit(void);
extern uint32_t Board_getGpioButtonIntrNum(void);
extern uint32_t Board_getGpioButtonSwitchNum(void);

/*!
 * @brief   Initialize the PMIC IRQ configuration for testing.
 * This function initializes the PMIC IRQ configuration for testing purposes.
 * It sets up various parameters required for IRQ configuration and initializes
 * the PMIC core handle.
 *
 * @param   void
 * @return  Returns the status of the initialization process.
 */

int32_t test_pmic_irq_config_init(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_BB_TPS65386X;
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

    status = test_pmic_appInit(&pPmicCoreHandle_irq, &pmicConfigData);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__, status);
    }
    return status;
}

/*!
 * @brief   Deinitialize the PMIC IRQ configuration after testing.
 * This function deinitializes the PMIC IRQ configuration after testing is completed.
 * It deinitializes the PMIC core handle and frees up any allocated memory.
 *
 * @param   void
 * @return  Returns the status of the deinitialization process.
 */
int32_t test_pmic_irq_config_deinit(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_deinit(pPmicCoreHandle_irq);
    free(pPmicCoreHandle_irq);
    SemaphoreP_destruct(&gpmicCoreObj);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__,  status);
    }
    return status;
}

void test_pmic_mask_all_gpo_intr()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    pmicStatus = Pmic_maskGpioIntr(pPmicCoreHandle_irq ,
                                   PMIC_IRQ_GPO_ALL_INT_MASK_NUM,
                                   TRUE,
                                   PMIC_IRQ_GPIO_INT_TYPE);

    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    else
    {
        DebugP_log("ALL GPO INTR are now UNMASKED!!!\r\n");
    }
}

void Board_gpioInit(void)
{
}

void Board_gpioDeinit(void)
{
}

uint32_t Board_getGpioButtonIntrNum(void)
{
    return (BOARD_BUTTON_GPIO_INTR_NUM);
}

uint32_t Board_getGpioButtonSwitchNum(void)
{
    return (BOARD_BUTTON_GPIO_SWITCH_NUM);
}

void test_pmic_gpo1_intr()
{
    int32_t         retVal;
    uint32_t        pinNum, intrNum, buttonNum;
    uint32_t        waitCount = 5;
    HwiP_Params     hwiPrms;

    Board_gpioInit();

    DebugP_log("GPIO Input Interrupt Test Started ...\r\n");
    DebugP_log("GPIO Interrupt Configured for Rising Edge (Button release will trigger interrupt) ...\r\n");

    pinNum          = CONFIG_GPIO0_PIN;
    intrNum         = Board_getGpioButtonIntrNum();
    buttonNum       = Board_getGpioButtonSwitchNum();

    /* Address translate */
    gGpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gGpioBaseAddr);

    /* Register pin interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = &GPIO_bankIsrFxn;
    hwiPrms.args     = (void *) pinNum;
    /* GPIO interrupt is a pulse type interrupt */
    hwiPrms.isPulse  = TRUE;
    retVal = HwiP_construct(&gGpioHwiObject, &hwiPrms);
    DebugP_assert(retVal == SystemP_SUCCESS );

    DebugP_log("Press and release SW%d button on EVM to trigger GPIO interrupt ...\r\n", buttonNum);
    while(gGpioIntrDone < waitCount)
    {
        /* Keep printing the current GPIO value */
        DebugP_log("Key is pressed %d times\r\n", gGpioIntrDone);
        ClockP_sleep(1);
    }
    DebugP_log("Key is pressed %d times\r\n", gGpioIntrDone);

    DebugP_log("GPIO Input Interrupt Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_gpioDeinit();
}

void GPIO_bankIsrFxn(void *args)
{
    uint32_t    pinNum = (uint32_t) args;
    uint32_t    bankNum =  GPIO_GET_BANK_INDEX(pinNum);
    uint32_t    intrStatus, pinMask = GPIO_GET_BANK_BIT_MASK(pinNum);

    /* Get and clear bank interrupt status */
    intrStatus = GPIO_getBankIntrStatus(gGpioBaseAddr, bankNum);
    GPIO_clearBankIntrStatus(gGpioBaseAddr, bankNum, intrStatus);

    /* Per pin interrupt handling */
    if(intrStatus & pinMask)
    {
        gGpioIntrDone++;
    }
}

/**
 * @brief Test function for IRQ.
 * This function serves as the main test case for the IRQ functionality.
 *
 * @param void
 * @return NULL
 */
void test_pmic_irq()
{
    DebugP_log("[IRQ TEST]\r\n");
    test_pmic_mask_all_gpo_intr();

    test_pmic_gpo1_intr();
}

/**
 * @brief Main test function for IRQ.
 * This function initializes the necessary drivers and configurations for
 * testing the IRQ functionality. It performs various tests.
 *
 * @param args Unused argument
 * @return NULL
 */
void *test_pmic_IRQ(void *args)
{
    Drivers_open();
    Board_driversOpen();

    mcspi_mux_pmic();

    /* Initialization */
    DebugP_log("Initializing...\r\n");
    test_pmic_irq_config_init();
    DebugP_log("Initialization Completed!\r\n\n");

    /* Lock config register initially */
    DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_irq, 0);

    /*API to unlock configuration register */
    DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_irq, 1);

    /* Lock counter register initially */
    DebugP_log("[INIT] TC and RC Lock Sequence:\r\n");
    test_pmic_CNT_LockUnlock(pPmicCoreHandle_irq, 0);

    /* Unlock counter register initially */
    DebugP_log("[INIT] TC and RC Unlock Sequence:\r\n");
    test_pmic_CNT_LockUnlock(pPmicCoreHandle_irq, 1);

    /* Low IQ Timer Test Cases */
    DebugP_log("[TEST] :\r\n");
    test_pmic_irq();

    /* De-initialization */
    test_pmic_irq_config_deinit();
    DebugP_log("\n[DE-INIT] De-initialization Completed\r\n");

    Board_driversClose();
    Drivers_close();

    return NULL;
}
