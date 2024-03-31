/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
/**
 * \file esm_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton ESM Unity tests
 * \version 1.0
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
