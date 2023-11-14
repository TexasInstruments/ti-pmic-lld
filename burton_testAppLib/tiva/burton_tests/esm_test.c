/**
 * \file esm_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Unity testing application that tests the PMIC driver ESM APIs
 * \version 1.0
 * \date 2023-10-11
 *
 * \copyright Copyright (c) 2023
 */

/* Standard includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Tiva test app library */
#include "pmic_drv/burton_testAppLib/tiva/tiva_testLib.h"

/* Test specific include */
#include "pmic_drv/burton_testAppLib/tiva/burton_tests/esm_test.h"

/* Unity testing library */
#include "unity/unity.h"

/* PMIC driver */
#include "pmic_drv/pmic.h"

Pmic_CoreHandle_t pmicCoreHandle;

/**
 * \brief   These variables are used in the Unity setup() and teardown() functions
 *          to save and restore NVM default ESM settings/configurations
 */
bool           nvmEsmEnableState = false;
bool           nvmEsmStartState = false;
Pmic_EsmCfg_t  nvmEsmCfg;
Pmic_GpioCfg_t nvmGpio6Cfg;
timerHandle_t  timerHandle;

static void disableVMON1LDO2(void);
static void saveNvmGpio6EsmConfig(void);

int main(void)
{
    /*** Variable declaration/initialization ***/
    // clang-format off
    uartHandle_t vcpHandle;
    i2cHandle_t I2C1Handle;
    Pmic_CoreCfg_t pmicConfigData = {
        .validParams =
            (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT | PMIC_CFG_COMM_MODE_VALID_SHIFT    | PMIC_CFG_SLAVEADDR_VALID_SHIFT |
            PMIC_CFG_QASLAVEADDR_VALID_SHIFT  | PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT | PMIC_CFG_COMM_HANDLE_VALID_SHIFT
            | PMIC_CFG_COMM_IO_RD_VALID_SHIFT   | PMIC_CFG_COMM_IO_WR_VALID_SHIFT   |
            PMIC_CFG_I2C1_SPEED_VALID_SHIFT),
        .instType           = PMIC_MAIN_INST,
        .pmicDeviceType     = PMIC_DEV_BURTON_TPS6522X,
        .commMode           = PMIC_INTF_SINGLE_I2C,
        .slaveAddr          = BURTON_I2C_USER_PAGE_ADDRESS,
        .qaSlaveAddr        = BURTON_I2C_WDG_PAGE_ADDRESS,
        .nvmSlaveAddr       = BURTON_I2C_NVM_PAGE_ADDRESS,
        .i2c1Speed          = PMIC_I2C_STANDARD_MODE,
        .pCommHandle        = &I2C1Handle,
        .pQACommHandle      = &I2C1Handle,
        .pFnPmicCommIoRead  = &pmicI2CRead,
        .pFnPmicCommIoWrite = &pmicI2CWrite};
    // clang-format on

    /*** System clock setup ***/
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 400 / 2 / 4 = 50 MHz clock rate

    /*** Virtual communication port setup ***/
    initializeVCPHandle(&vcpHandle);
    initializeVCP(&vcpHandle);

    /*** I2C setup ***/
    initializeI2C1Handle(&I2C1Handle);
    initializeI2C(&I2C1Handle);

    /*** Timer setup ***/
    initializeTimerHandle(&timerHandle);
    initializeTimer(&timerHandle);

    /*** PMIC setup ***/
    initializePmicCoreHandle(&pmicCoreHandle);
    Pmic_init(&pmicConfigData, &pmicCoreHandle);

    /*** Clear the console before printing anything ***/
    clearConsole(&vcpHandle);

    /*** Disable VMON1 and LDO2 OV/UV comparators, clear all IRQ, save GPIO 6 and ESM configuration ***/
    disableVMON1LDO2();
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);
    saveNvmGpio6EsmConfig();

    /*** Print welcome message ***/
    UARTStrPut(&vcpHandle, "Running all PMIC ESM tests...\r\n\r\n");

    /*** Begin unity testing ***/
    UNITY_BEGIN();

    RUN_TEST(test_ESM_enableDisable);
    RUN_TEST(test_ESM_startStop);
    RUN_TEST(test_ESM_setConfiguration_levelMode);
    RUN_TEST(test_ESM_setConfiguration_pwmMode);
    RUN_TEST(test_ESM_interrupt_enableDisable);
    RUN_TEST(test_ESM_levelMode_noErrors);
    RUN_TEST(test_ESM_levelMode_ESM_MCU_PIN_INT);
    RUN_TEST(test_ESM_levelMode_ESM_MCU_FAIL_INT);
    RUN_TEST(test_ESM_levelMode_ESM_MCU_RST_INT);
    RUN_TEST(test_ESM_pwmMode_noErrors);
    RUN_TEST(test_ESM_pwmMode_ESM_MCU_PIN_INT);
    RUN_TEST(test_ESM_pwmMode_ESM_MCU_FAIL_INT);
    RUN_TEST(test_ESM_pwmMode_ESM_MCU_RST_INT);

    /*** Finish unity testing ***/
    return UNITY_END();
}

/**
 * \brief   This function is used to reset an ESM configuration struct with all
 *          valid parameters set
 *
 * \param esmCfg [OUT] ESM configuration struct to be reset
 */
static void resetEsmCfg_withAllValidParams(Pmic_EsmCfg_t *esmCfg)
{
    // clang-format off
    esmCfg->validParams  = (PMIC_ESM_CFG_DELAY1_VALID_SHIFT      | PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                            PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                            PMIC_ESM_CFG_HMIN_VALID_SHIFT        | PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                            PMIC_ESM_CFG_LMIN_VALID_SHIFT        | PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                            PMIC_ESM_CFG_MODE_VALID_SHIFT);
    esmCfg->esmDelay1_us = 0;
    esmCfg->esmDelay2_us = 0;
    esmCfg->esmHmax_us   = 30;
    esmCfg->esmHmin_us   = 15;
    esmCfg->esmLmax_us   = 30;
    esmCfg->esmLmin_us   = 15;
    esmCfg->esmErrCntThr = 0;
    esmCfg->esmEnDrv     = PMIC_ESM_ERR_EN_DRV_CLEAR_DISABLE;
    esmCfg->esmMode      = PMIC_ESM_LEVEL_MODE;
    // clang-format on
}

/**
 * \brief This private helper function is used to save the NVM default configurations of
 *        GPIO 6 and ESM of Burton
 */
static void saveNvmGpio6EsmConfig(void)
{
    // Save GPIO 6 configuration
    resetGpioCfg_withAllValidParams(&nvmGpio6Cfg);
    (void)Pmic_gpioGetConfiguration(&pmicCoreHandle, PMIC_TPS6522X_GPIO6_PIN, &nvmGpio6Cfg);

    // Save NVM default ESM enable/disable setting
    (void)Pmic_esmGetEnableState(&pmicCoreHandle, PMIC_ESM_MODE_MCU, &nvmEsmEnableState);

    // Save NVM default ESM start/stop setting
    (void)Pmic_esmGetStatus(&pmicCoreHandle, PMIC_ESM_MODE_MCU, &nvmEsmStartState);

    // Save NVM default ESM configuration
    resetEsmCfg_withAllValidParams(&nvmEsmCfg);
    (void)Pmic_esmGetConfiguration(&pmicCoreHandle, PMIC_ESM_MODE_MCU, &nvmEsmCfg);
}

/**
 * \brief By TPS6522430-Q1 OTP default, LDO2 is monitored by VMON1. This private function is
 *        used to disable VMON1 and LDO2 and mask their interrupts to prevent their influence
 *        during tests
 */
static void disableVMON1LDO2(void)
{
    const int32_t done = 0xFFF;
    int32_t       status = PMIC_ST_SUCCESS;
    uint8_t       regData = 0;

    // Disable VMON 1 OV/UV comparator
    while (status != done)
    {
        status = pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, VCCA_VMON_CTRL_REG_ADDR, &regData, 1);
        if (status != PMIC_ST_SUCCESS)
        {
            continue;
        }

        regData &= ~(0b10);
        status = pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, VCCA_VMON_CTRL_REG_ADDR, &regData, 1);
        if (status == PMIC_ST_SUCCESS)
        {
            status = done;
        }
    }

    status = PMIC_ST_SUCCESS;

    // Disable LDO2 and its voltage monitor
    while (status != done)
    {
        status = pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, LDO2_CTRL_REG_ADDR, &regData, 1);
        if (status != PMIC_ST_SUCCESS)
        {
            continue;
        }

        regData &= ~(0b10001);
        status = pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, LDO2_CTRL_REG_ADDR, &regData, 1);
        if (status == PMIC_ST_SUCCESS)
        {
            status = done;
        }
    }

    // Mask VMON1 and LDO2 interrupts
    (void)Pmic_irqMaskIntr(&pmicCoreHandle, PMIC_TPS6522X_VMON1_UVOV_INT, PMIC_IRQ_MASK);
    (void)Pmic_irqMaskIntr(&pmicCoreHandle, PMIC_TPS6522X_LDO2_UVOV_INT, PMIC_IRQ_MASK);

    // Clear VMON1 and LDO2 interrupts
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_TPS6522X_VMON1_UVOV_INT);
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_TPS6522X_LDO2_UVOV_INT);
}

/**
 * \brief This private helper function is used to initialize all members of an ESM
 *        configuration struct for the Level Mode and PWM Mode configuration tests
 *
 * \param esmCfg        [OUT]       ESM configuration struct to initalize
 * \param esmMode       [IN]        ESM mode select. Valid values: \ref Pmic_EsmMode
 */
static void initEsmCfg_forConfigurationTest(Pmic_EsmCfg_t *esmCfg, bool esmMode)
{
    // clang-format off
    esmCfg->validParams  = (PMIC_ESM_CFG_DELAY1_VALID_SHIFT      | PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                            PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                            PMIC_ESM_CFG_HMIN_VALID_SHIFT        | PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                            PMIC_ESM_CFG_LMIN_VALID_SHIFT        | PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                            PMIC_ESM_CFG_MODE_VALID_SHIFT);
    esmCfg->esmDelay1_us = 20480;
    esmCfg->esmDelay2_us = 10240;
    esmCfg->esmHmax_us   = 1500;
    esmCfg->esmHmin_us   = 1200;
    esmCfg->esmLmax_us   = 1500;
    esmCfg->esmLmin_us   = 1200;
    esmCfg->esmErrCntThr = 0xF; // ESM error count threshold is 4 bit
    esmCfg->esmEnDrv     = PMIC_ESM_ERR_EN_DRV_CLEAR_DISABLE;
    esmCfg->esmMode      = esmMode;
    // clang-format on
}

/**
 * \brief This private helper function is used to initialize all members of an ESM
 *        configuration struct for the Level Mode interrupt tests
 *
 * \param esmCfg        [OUT]       ESM configuration struct to initalize
 */
static void initEsmCfg_forLevelModeErrTest(Pmic_EsmCfg_t *esmCfg)
{
    // clang-format off
    esmCfg->validParams  = (PMIC_ESM_CFG_DELAY1_VALID_SHIFT      | PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                            PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT | PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                            PMIC_ESM_CFG_MODE_VALID_SHIFT);
    esmCfg->esmDelay1_us = 522240;
    esmCfg->esmDelay2_us = 522240;
    esmCfg->esmErrCntThr = 3;
    esmCfg->esmEnDrv     = PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE;
    esmCfg->esmMode      = PMIC_ESM_LEVEL_MODE;
    // clang-format on
}

/**
 * \brief This private helper function is used to initialize all members of an ESM
 *        configuration struct for the PWM Mode interrupt tests
 *
 * \param esmCfg        [OUT]       ESM configuration struct to initalize
 */
static void initEsmCfg_forPwmModeErrTest(Pmic_EsmCfg_t *esmCfg)
{
    // clang-format off
    esmCfg->validParams     = (PMIC_ESM_CFG_DELAY1_VALID_SHIFT      | PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT        | PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT        | PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT);
    esmCfg->esmDelay1_us    = 522240;
    esmCfg->esmDelay2_us    = 522240;
    esmCfg->esmHmax_us      = 3840;
    esmCfg->esmHmin_us      = 15;
    esmCfg->esmLmax_us      = 3840;
    esmCfg->esmLmin_us      = 15;
    esmCfg->esmErrCntThr    = 3;
    esmCfg->esmEnDrv        = PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE;
    esmCfg->esmMode         = PMIC_ESM_PWM_MODE;
    // clang-format on
}

/**
 * \brief This private helper function is used in the Level Mode and PWM Mode configuration
 *        tests to check if two ESM configuration structs have equal members
 *
 * \param esmCfg1       [IN]    First ESM configuration struct
 * \param esmCfg2       [IN]    Second ESM configuration struct
 */
static inline void testForEqualEsmCfg(Pmic_EsmCfg_t esmCfg1, Pmic_EsmCfg_t esmCfg2)
{
    TEST_ASSERT_EQUAL_UINT32(esmCfg1.validParams, esmCfg2.validParams);
    TEST_ASSERT_EQUAL_UINT32(esmCfg1.esmDelay1_us, esmCfg2.esmDelay1_us);
    TEST_ASSERT_EQUAL_UINT32(esmCfg1.esmDelay2_us, esmCfg2.esmDelay2_us);
    TEST_ASSERT_EQUAL_UINT16(esmCfg1.esmHmax_us, esmCfg2.esmHmax_us);
    TEST_ASSERT_EQUAL_UINT16(esmCfg1.esmHmin_us, esmCfg2.esmHmin_us);
    TEST_ASSERT_EQUAL_UINT16(esmCfg1.esmLmax_us, esmCfg2.esmLmax_us);
    TEST_ASSERT_EQUAL_UINT16(esmCfg1.esmLmin_us, esmCfg2.esmLmin_us);
    TEST_ASSERT_EQUAL_UINT8(esmCfg1.esmErrCntThr, esmCfg2.esmErrCntThr);
    TEST_ASSERT_EQUAL(esmCfg1.esmEnDrv, esmCfg2.esmEnDrv);
    TEST_ASSERT_EQUAL(esmCfg1.esmMode, esmCfg2.esmMode);
}

/**
 * \brief This private helper function configures PMIC GPIO pin 6 to nERR MCU functionality
 */
static void setPmicGpioConfig_GPIO6_nERR_MCU(void)
{
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nERR_MCU_gpioCfg;

    // clang-format off
    resetGpioCfg_withSpecificValidParams(&nERR_MCU_gpioCfg, (PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                             PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                             PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                             PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT));
    nERR_MCU_gpioCfg.pinDir         = PMIC_GPIO_INPUT;
    nERR_MCU_gpioCfg.pinFunc        = PMIC_TPS6522X_GPIO_PINFUNC_GPIO6_NERR_MCU;
    nERR_MCU_gpioCfg.pullCtrl       = PMIC_GPIO_PULL_DOWN;
    nERR_MCU_gpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, PMIC_TPS6522X_GPIO6_PIN, nERR_MCU_gpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    // clang-format on
}

/**
 * \brief This private helper function is used to help setup ESM Level Mode interrupt tests. It sets
 *        the configuration on the PMIC nERR_MCU pin, configure the MCU pin that will interface the
 *        nERR_MCU pin, and unmask specified interrupts
 *
 * \param esmMaskCfg    [IN]    Used to mask or unmask an ESM interrupt.
 *                              Valid values: \ref esmIrqMaskingOptions
 */
static void setupEsmLevelModeErrTest(gpioPinHandle_t *mcuPinHandle, uint8_t esmIrqMaskCfg)
{
    Pmic_EsmCfg_t     esmCfg;
    Pmic_EsmIntrCfg_t esmIntrCfg;
    int32_t           status = PMIC_ST_SUCCESS;

    // Configure MCU pin that's interfacing the PMIC nERR_MCU pin to be output
    initializeEsmGpioOutputHandle(mcuPinHandle);
    initializeGpioPin(*mcuPinHandle);

    // Configure PMIC GPIO6 pin to nERR_MCU functionality
    setPmicGpioConfig_GPIO6_nERR_MCU();

    // Stop the ESM in case it is running, enable the ESM
    status = Pmic_esmStart(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    status = Pmic_esmEnable(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Configure ESM
    initEsmCfg_forLevelModeErrTest(&esmCfg);
    status = Pmic_esmSetConfiguration(&pmicCoreHandle, PMIC_ESM_MODE_MCU, esmCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Clear ESM interrupts
    if ((esmIrqMaskCfg & ESM_MCU_PIN_INT_UNMASKED) != 0)
    {
        status = Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_PIN_INT);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }
    if ((esmIrqMaskCfg & ESM_MCU_FAIL_INT_UNMASKED) != 0)
    {
        status = Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_FAIL_INT);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }
    if ((esmIrqMaskCfg & ESM_MCU_RST_INT_UNMASKED) != 0)
    {
        status = Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_RST_INT);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }

    // Enable ESM interrupts
    esmIntrCfg.esmRstIntr =
        (((esmIrqMaskCfg & ESM_MCU_RST_INT_UNMASKED) != 0) ? PMIC_ESM_INTERRUPT_ENABLE : PMIC_ESM_INTERRUPT_DISABLE);
    esmIntrCfg.esmFailIntr =
        (((esmIrqMaskCfg & ESM_MCU_FAIL_INT_UNMASKED) != 0) ? PMIC_ESM_INTERRUPT_ENABLE : PMIC_ESM_INTERRUPT_DISABLE);
    esmIntrCfg.esmPinIntr =
        (((esmIrqMaskCfg & ESM_MCU_PIN_INT_UNMASKED) != 0) ? PMIC_ESM_INTERRUPT_ENABLE : PMIC_ESM_INTERRUPT_DISABLE);
    status = Pmic_esmSetInterrupt(&pmicCoreHandle, PMIC_ESM_MODE_MCU, esmIntrCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief This private helper function is used to help setup ESM PWM interrupt tests. It sets
 *        the configuration on the PMIC nERR_MCU pin, configure the MCU pin that will interface
 *        the nERR_MCU pin, and unmask specified interrupts
 *
 * \param esmMaskCfg    [IN]    Used to mask or unmask an ESM interrupt.
 *                              Valid values: \ref esmIrqMaskingOptions
 */
static void setupEsmPwmModeErrTest(pwmHandle_t *pwmHandle, uint8_t esmIrqMaskCfg)
{
    Pmic_EsmCfg_t     esmCfg;
    Pmic_EsmIntrCfg_t esmIntrCfg;
    int32_t           status = PMIC_ST_SUCCESS;

    // Configure MCU pin that's interfacing the PMIC nERR_MCU pin to be a PWM output
    initializePwmHandle(pwmHandle);
    initializePwmPin(*pwmHandle);

    // Configure PMIC GPIO6 pin to nERR_MCU functionality
    setPmicGpioConfig_GPIO6_nERR_MCU();

    // Stop the ESM in case it is running, enable the ESM
    status = Pmic_esmStart(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    status = Pmic_esmEnable(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Configure ESM
    initEsmCfg_forPwmModeErrTest(&esmCfg);
    status = Pmic_esmSetConfiguration(&pmicCoreHandle, PMIC_ESM_MODE_MCU, esmCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Clear ESM interrupts
    if ((esmIrqMaskCfg & ESM_MCU_PIN_INT_UNMASKED) != 0)
    {
        status = Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_PIN_INT);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }
    if ((esmIrqMaskCfg & ESM_MCU_FAIL_INT_UNMASKED) != 0)
    {
        status = Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_FAIL_INT);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }
    if ((esmIrqMaskCfg & ESM_MCU_RST_INT_UNMASKED) != 0)
    {
        status = Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_RST_INT);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }

    // Enable ESM interrupts
    esmIntrCfg.esmRstIntr =
        (((esmIrqMaskCfg & ESM_MCU_RST_INT_UNMASKED) != 0) ? PMIC_ESM_INTERRUPT_ENABLE : PMIC_ESM_INTERRUPT_DISABLE);
    esmIntrCfg.esmFailIntr =
        (((esmIrqMaskCfg & ESM_MCU_FAIL_INT_UNMASKED) != 0) ? PMIC_ESM_INTERRUPT_ENABLE : PMIC_ESM_INTERRUPT_DISABLE);
    esmIntrCfg.esmPinIntr =
        (((esmIrqMaskCfg & ESM_MCU_PIN_INT_UNMASKED) != 0) ? PMIC_ESM_INTERRUPT_ENABLE : PMIC_ESM_INTERRUPT_DISABLE);
    status = Pmic_esmSetInterrupt(&pmicCoreHandle, PMIC_ESM_MODE_MCU, esmIntrCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief This private helper function is used to test whether the driver detects no ESM errors under
 *        correct ESM operation.
 *
 * \param esmMode [IN] Mode of operation to test the ESM
 */
static void esmNoErrTest(uint8_t esmMode)
{
    uint8_t          i = 0;
    int32_t          status = PMIC_ST_SUCCESS;
    uint8_t          actualIrqNum = 0;
    Pmic_IrqStatus_t errStat;
    gpioPinHandle_t  mcuPinHandle;
    pwmHandle_t      pwmHandle;

    // Set configuration on PMIC nERR_MCU pin, configure MCU pin as output GPIO or output PWM,
    // configure ESM, and unmask all interrupts
    switch (esmMode)
    {
        case PMIC_ESM_LEVEL_MODE:
            setupEsmLevelModeErrTest(&mcuPinHandle,
                                     (ESM_MCU_FAIL_INT_UNMASKED | ESM_MCU_PIN_INT_UNMASKED | ESM_MCU_RST_INT_UNMASKED));
            break;
        case PMIC_ESM_PWM_MODE:
            setupEsmPwmModeErrTest(&pwmHandle,
                                   (ESM_MCU_FAIL_INT_UNMASKED | ESM_MCU_PIN_INT_UNMASKED | ESM_MCU_RST_INT_UNMASKED));
            break;
        default:
            return;
    }

    // Ensure nERR_MCU is reading high signal level
    if (esmMode == PMIC_ESM_LEVEL_MODE)
    {
        GPIOPinWrite(mcuPinHandle.gpioPortBase, mcuPinHandle.gpioPin, 0xFF);
    }
    else
    {
        PWMOutputState(pwmHandle.pwmBase, pwmHandle.pwmOutBit, true);
    }

    // Start the ESM
    status = Pmic_esmStart(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_START);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Wait for 2 seconds (enough time for all ESM interrupt flags to be raised by the PMIC)
    delayTimeInMs(&timerHandle, 2000);

    // Get status of all interrupts
    status = Pmic_irqGetErrStatus(&pmicCoreHandle, &errStat, PMIC_IRQ_CLEAR_NONE);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // While there are new IRQs to read...
    for (i = 0; i < UINT8_MAX; i++)
    {
        // Get next IRQ number
        status = Pmic_getNextErrorStatus(&pmicCoreHandle, &errStat, &actualIrqNum);
        if (status == PMIC_ST_ERR_INV_INT)
        {
            break;
        }
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Check that the ESM interrupts are not raised
        TEST_ASSERT_NOT_EQUAL_UINT8(PMIC_TPS6522X_ESM_MCU_PIN_INT, actualIrqNum);
        TEST_ASSERT_NOT_EQUAL_UINT8(PMIC_TPS6522X_ESM_MCU_FAIL_INT, actualIrqNum);
        TEST_ASSERT_NOT_EQUAL_UINT8(PMIC_TPS6522X_ESM_MCU_RST_INT, actualIrqNum);
    }
    TEST_ASSERT_NOT_EQUAL_UINT8(UINT8_MAX, i);
}

/**
 * \brief This private helper function tests whether IRQs generated by the ESM can be detected.
 *
 * \param esmIrqNum [IN]    Target ESM interrupt to detect during the test
 * \param esmMode   [IN]    ESM mode of operation. Valid values: \ref Pmic_EsmMode
 */
static void esmErrTest(uint8_t esmIrqNum, uint8_t esmMode)
{
    uint8_t          i = 0;
    uint8_t          actualIrqNum = 0;
    uint16_t         milliseconds = 0;
    int32_t          status = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat;
    gpioPinHandle_t  mcuPinHandle;
    pwmHandle_t      pwmHandle;

    // Set configuration on PMIC nERR_MCU pin, configure MCU pin as output GPIO or output PWM,
    // configure ESM, and unmask all interrupts
    switch (esmMode)
    {
        case PMIC_ESM_LEVEL_MODE:
            setupEsmLevelModeErrTest(&mcuPinHandle,
                                     (ESM_MCU_FAIL_INT_UNMASKED | ESM_MCU_PIN_INT_UNMASKED | ESM_MCU_RST_INT_UNMASKED));
            break;
        case PMIC_ESM_PWM_MODE:
            setupEsmPwmModeErrTest(&pwmHandle,
                                   (ESM_MCU_FAIL_INT_UNMASKED | ESM_MCU_PIN_INT_UNMASKED | ESM_MCU_RST_INT_UNMASKED));
            break;
        default:
            return;
    }

    // Make the nERR_MCU pin read a low signal level
    if (esmMode == PMIC_ESM_LEVEL_MODE)
    {
        GPIOPinWrite(mcuPinHandle.gpioPortBase, mcuPinHandle.gpioPin, 0x00);
    }
    else
    {
        PWMOutputState(pwmHandle.pwmBase, pwmHandle.pwmOutBit, false);
    }

    // Start the ESM
    status = Pmic_esmStart(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_START);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Depending on the type of ESM IRQ, delay a certain amount of time
    // while nERR_MCU pin is reading low to trigger target interrupt
    switch (esmIrqNum)
    {
        case PMIC_TPS6522X_ESM_MCU_PIN_INT:
            milliseconds = 100;
            break;
        case PMIC_TPS6522X_ESM_MCU_FAIL_INT:
            milliseconds = 1000;
            break;
        case PMIC_TPS6522X_ESM_MCU_RST_INT:
            milliseconds = 1500;
            break;
        default:
            return;
    }

    // Wait while the nERR_MCU pin is reading low to trigger target interrupt
    delayTimeInMs(&timerHandle, milliseconds);

    // Stop the ESM
    status = Pmic_esmStart(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get status of all interrupts
    status = Pmic_irqGetErrStatus(&pmicCoreHandle, &errStat, PMIC_IRQ_CLEAR_NONE);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // While there are new IRQs to read...
    for (i = 0; i < UINT8_MAX; i++)
    {
        // Get next IRQ number
        status = Pmic_getNextErrorStatus(&pmicCoreHandle, &errStat, &actualIrqNum);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        if (actualIrqNum == esmIrqNum)
        {
            break;
        }
    }
    TEST_ASSERT_NOT_EQUAL_UINT8(UINT8_MAX, i);

    // Make sure that target IRQ is raised
    TEST_ASSERT_EQUAL_UINT8(esmIrqNum, actualIrqNum);
}

/**
 * \brief Test Pmic_esmEnable: Enable then disable the ESM
 */
void test_ESM_enableDisable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    actualEsmEnableState = false;

    // Enable ESM
    status = Pmic_esmEnable(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual ESM enable/disable state and compare expected vs. actual
    status = Pmic_esmGetEnableState(&pmicCoreHandle, PMIC_ESM_MODE_MCU, &actualEsmEnableState);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_ESM_ENABLE, actualEsmEnableState);

    // Disable ESM
    status = Pmic_esmEnable(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_DISABLE);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual ESM enable/disable state and compare expected vs. actual
    status = Pmic_esmGetEnableState(&pmicCoreHandle, PMIC_ESM_MODE_MCU, &actualEsmEnableState);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_ESM_DISABLE, actualEsmEnableState);
}

/**
 * \brief Test Pmic_esmStart: Start then stop the ESM
 */
void test_ESM_startStop(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    actualEsmStartState = false;

    // Start the ESM
    status = Pmic_esmStart(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_START);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual ESM start/stop state and compare expected vs. actual
    status = Pmic_esmGetStatus(&pmicCoreHandle, PMIC_ESM_MODE_MCU, &actualEsmStartState);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_ESM_START, actualEsmStartState);

    // Stop the ESM
    status = Pmic_esmStart(&pmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual ESM start/stop state and compare expected vs. actual
    status = Pmic_esmGetStatus(&pmicCoreHandle, PMIC_ESM_MODE_MCU, &actualEsmStartState);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_ESM_STOP, actualEsmStartState);
}

/**
 * \brief Test Pmic_esmSetConfiguration: Configure the ESM to operate in Level Mode
 */
void test_ESM_setConfiguration_levelMode(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expectedEsmCfg, actualEsmCfg;

    // Initialize ESM CFG structs
    resetEsmCfg_withAllValidParams(&expectedEsmCfg);
    resetEsmCfg_withAllValidParams(&actualEsmCfg);

    // Configure ESM
    initEsmCfg_forConfigurationTest(&expectedEsmCfg, PMIC_ESM_LEVEL_MODE);
    status = Pmic_esmSetConfiguration(&pmicCoreHandle, PMIC_ESM_MODE_MCU, expectedEsmCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual ESM configuration
    status = Pmic_esmGetConfiguration(&pmicCoreHandle, PMIC_ESM_MODE_MCU, &actualEsmCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual
    testForEqualEsmCfg(expectedEsmCfg, actualEsmCfg);
}

/**
 * \brief Test Pmic_esmSetConfiguration: Configure the ESM to operate in PWM Mode
 */
void test_ESM_setConfiguration_pwmMode(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expectedEsmCfg, actualEsmCfg;

    // Initialize ESM CFG structs
    resetEsmCfg_withAllValidParams(&expectedEsmCfg);
    resetEsmCfg_withAllValidParams(&actualEsmCfg);

    // Configure ESM
    initEsmCfg_forConfigurationTest(&expectedEsmCfg, PMIC_ESM_PWM_MODE);
    status = Pmic_esmSetConfiguration(&pmicCoreHandle, PMIC_ESM_MODE_MCU, expectedEsmCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual ESM configuration
    status = Pmic_esmGetConfiguration(&pmicCoreHandle, PMIC_ESM_MODE_MCU, &actualEsmCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual
    testForEqualEsmCfg(expectedEsmCfg, actualEsmCfg);
}

/**
 * \brief Test Pmic_esmSetInterrupt: Enable then disable (that is to say, unmask then mask)
 *                                    ESM interrupts
 */
void test_ESM_interrupt_enableDisable(void)
{
    int32_t           status = PMIC_ST_SUCCESS;
    Pmic_EsmIntrCfg_t esmIntrCfg;
    bool              actualEsmPinIntrMask = false, actualEsmFailIntrMask = false, actualEsmRstIntrMask = false;

    // Unmask ESM interrupts
    esmIntrCfg.esmFailIntr = PMIC_ESM_INTERRUPT_ENABLE;
    esmIntrCfg.esmPinIntr = PMIC_ESM_INTERRUPT_ENABLE;
    esmIntrCfg.esmRstIntr = PMIC_ESM_INTERRUPT_ENABLE;
    status = Pmic_esmSetInterrupt(&pmicCoreHandle, PMIC_ESM_MODE_MCU, esmIntrCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual ESM interrupt mask status
    status = Pmic_irqGetMaskIntrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_PIN_INT, &actualEsmPinIntrMask);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    status = Pmic_irqGetMaskIntrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_FAIL_INT, &actualEsmFailIntrMask);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    status = Pmic_irqGetMaskIntrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_RST_INT, &actualEsmRstIntrMask);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual
    TEST_ASSERT_EQUAL(PMIC_IRQ_UNMASK, actualEsmFailIntrMask);
    TEST_ASSERT_EQUAL(PMIC_IRQ_UNMASK, actualEsmPinIntrMask);
    TEST_ASSERT_EQUAL(PMIC_IRQ_UNMASK, actualEsmRstIntrMask);

    // Mask ESM interrupts
    esmIntrCfg.esmFailIntr = PMIC_ESM_INTERRUPT_DISABLE;
    esmIntrCfg.esmPinIntr = PMIC_ESM_INTERRUPT_DISABLE;
    esmIntrCfg.esmRstIntr = PMIC_ESM_INTERRUPT_DISABLE;
    status = Pmic_esmSetInterrupt(&pmicCoreHandle, PMIC_ESM_MODE_MCU, esmIntrCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual ESM interrupt mask status
    status = Pmic_irqGetMaskIntrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_FAIL_INT, &actualEsmFailIntrMask);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    status = Pmic_irqGetMaskIntrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_PIN_INT, &actualEsmPinIntrMask);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    status = Pmic_irqGetMaskIntrStatus(&pmicCoreHandle, PMIC_TPS6522X_ESM_MCU_RST_INT, &actualEsmRstIntrMask);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual
    TEST_ASSERT_EQUAL(PMIC_IRQ_MASK, actualEsmFailIntrMask);
    TEST_ASSERT_EQUAL(PMIC_IRQ_MASK, actualEsmPinIntrMask);
    TEST_ASSERT_EQUAL(PMIC_IRQ_MASK, actualEsmRstIntrMask);
}

/**
 * \brief Test ESM_levelMode: Test ESM in Level Mode for no errors during operation
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_levelMode_noErrors(void)
{
    uint8_t esmMode = PMIC_ESM_LEVEL_MODE;

    esmNoErrTest(esmMode);
}

/**
 * \brief Test ESM_levelMode: Test to verify ESM_MCU_PIN_INT error in Level Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_levelMode_ESM_MCU_PIN_INT(void)
{
    uint8_t esmMode = PMIC_ESM_LEVEL_MODE;
    uint8_t esmIrqNum = PMIC_TPS6522X_ESM_MCU_PIN_INT;

    esmErrTest(esmIrqNum, esmMode);
}

/**
 * \brief Test ESM_levelMode: Test to verify ESM_MCU_FAIL_INT error in Level Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_levelMode_ESM_MCU_FAIL_INT(void)
{
    uint8_t esmMode = PMIC_ESM_LEVEL_MODE;
    uint8_t esmIrqNum = PMIC_TPS6522X_ESM_MCU_FAIL_INT;

    esmErrTest(esmIrqNum, esmMode);
}

/**
 * \brief Test ESM_levelMode: Test to verify ESM_MCU_RST_INT error in Level Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_levelMode_ESM_MCU_RST_INT(void)
{
    uint8_t esmMode = PMIC_ESM_LEVEL_MODE;
    uint8_t esmIrqNum = PMIC_TPS6522X_ESM_MCU_RST_INT;

    esmErrTest(esmIrqNum, esmMode);
}

/**
 * \brief Test ESM_pwmMode: Test ESM in PWM Mode for no errors during operation
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_pwmMode_noErrors(void)
{
    uint8_t esmMode = PMIC_ESM_PWM_MODE;

    esmNoErrTest(esmMode);
}

/**
 * \brief Test ESM_pwmMode: Test to verify ESM_MCU_PIN_INT error in PWM Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_pwmMode_ESM_MCU_PIN_INT(void)
{
    uint8_t esmMode = PMIC_ESM_PWM_MODE;
    uint8_t esmIrqNum = PMIC_TPS6522X_ESM_MCU_PIN_INT;

    esmErrTest(esmIrqNum, esmMode);
}

/**
 * \brief Test ESM_pwmMode: Test to verify ESM_MCU_FAIL_INT error in PWM Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_pwmMode_ESM_MCU_FAIL_INT(void)
{
    uint8_t esmMode = PMIC_ESM_PWM_MODE;
    uint8_t esmIrqNum = PMIC_TPS6522X_ESM_MCU_FAIL_INT;

    esmErrTest(esmIrqNum, esmMode);
}

/**
 * \brief Test ESM_pwmMode: Test to verify ESM_MCU_RST_INT error in PWM Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_pwmMode_ESM_MCU_RST_INT(void)
{
    uint8_t esmMode = PMIC_ESM_PWM_MODE;
    uint8_t esmIrqNum = PMIC_TPS6522X_ESM_MCU_RST_INT;

    esmErrTest(esmIrqNum, esmMode);
}

/**
 * \brief This function is called by Unity when it starts a test
 */
void setUp(void)
{
    // Clear all interrupts
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);
}

/**
 * \brief This function is called by Unity when it finishes a test
 */
void tearDown(void)
{
    // Restore GPIO 6 configuration
    (void)Pmic_gpioSetConfiguration(&pmicCoreHandle, PMIC_TPS6522X_GPIO6_PIN, nvmGpio6Cfg);

    // Restore NVM default ESM enable/disable setting
    (void)Pmic_esmEnable(&pmicCoreHandle, PMIC_ESM_MODE_MCU, nvmEsmEnableState);

    // Restore NVM default ESM start/stop setting
    (void)Pmic_esmStart(&pmicCoreHandle, PMIC_ESM_MODE_MCU, nvmEsmStartState);

    // Restore NVM default ESM configuration
    (void)Pmic_esmSetConfiguration(&pmicCoreHandle, PMIC_ESM_MODE_MCU, nvmEsmCfg);
}
