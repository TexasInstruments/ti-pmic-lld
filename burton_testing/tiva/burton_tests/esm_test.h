/**
 * \file esm_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton ESM Unity tests
 * \version 1.0
 * \date 2023-10-12
 *
 * \copyright Copyright (c) 2023 Texas Instruments Incorporated
 *
 */
#ifndef ESM_TEST_H
#define ESM_TEST_H

#include "common_test.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \anchor esmIrqMaskingOptions
 *  \name   ESM interrupt unmasking options
 *
 *  @{
 */
#define ESM_MCU_RST_INT_UNMASKED         0x4U
#define ESM_MCU_RST_INT_UNMASKED_OPTION  0x4U
#define ESM_MCU_FAIL_INT_UNMASKED        0x2U
#define ESM_MCU_FAIL_INT_UNMASKED_OPTION 0x2U
#define ESM_MCU_PIN_INT_UNMASKED         0x1U
#define ESM_MCU_PIN_INT_UNMASKED_OPTION  0x1U
/*  @} */

/**
 * \brief Test Pmic_esmEnable: Enable then disable the ESM
 */
void test_ESM_enableDisable(void);

/**
 * \brief Test Pmic_esmStart: Start then stop the ESM
 */
void test_ESM_startStop(void);

/**
 * \brief Test Pmic_esmSetConfiguration: Configure the ESM to operate in Level Mode
 */
void test_ESM_setConfiguration_levelMode(void);

/**
 * \brief Test Pmic_esmSetConfiguration: Configure the ESM to operate in PWM Mode
 */
void test_ESM_setConfiguration_pwmMode(void);

/**
 * \brief Test Pmic_esmSetInterrupt: Enable then disable (that is to say, unmask then mask)
 *                                    ESM interrupts
 */
void test_ESM_interrupt_enableDisable(void);

/**
 * \brief Test ESM_levelMode: Test ESM in Level Mode for no errors during operation
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_levelMode_noErrors(void);

/**
 * \brief Test ESM_levelMode: Test to verify ESM_MCU_PIN_INT error in Level Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_levelMode_ESM_MCU_PIN_INT(void);

/**
 * \brief Test ESM_levelMode: Test to verify ESM_MCU_FAIL_INT error in Level Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_levelMode_ESM_MCU_FAIL_INT(void);

/**
 * \brief Test ESM_levelMode: Test to verify ESM_MCU_RST_INT error in Level Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_levelMode_ESM_MCU_RST_INT(void);

/**
 * \brief Test ESM_pwmMode: Test ESM in PWM Mode for no errors during operation
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_pwmMode_noErrors(void);

/**
 * \brief Test ESM_pwmMode: Test to verify ESM_MCU_PIN_INT error in PWM Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_pwmMode_ESM_MCU_PIN_INT(void);

/**
 * \brief Test ESM_pwmMode: Test to verify ESM_MCU_FAIL_INT error in PWM Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_pwmMode_ESM_MCU_FAIL_INT(void);

/**
 * \brief Test ESM_pwmMode: Test to verify ESM_MCU_RST_INT error in PWM Mode
 *
 * \note There must be a pin connection from the MCU to the PMIC's GPIO6 pin
 */
void test_ESM_pwmMode_ESM_MCU_RST_INT(void);

/**
 * \brief Test Pmic_esmGetErrCnt: Read error count of ESM
 */
void test_ESM_getErrorCount(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* ESM_TEST_H */
