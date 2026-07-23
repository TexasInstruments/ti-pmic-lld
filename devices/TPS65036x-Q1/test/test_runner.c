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


#include "unity.h"
#include "platform.h"
#include "debug.h"
#include "test_filter.h"
#include <stdio.h>
#include "regmap/core.h"
#include "regmap/irq.h"
#include "regmap/wdg.h"
#ifdef BUILD_MOCK
#include "pmic_mock_types.h"
#include "pmic_mock_core.h"
#endif



/**
 * @brief Unity setUp function - called before each test
 */
void setUp(void)
{
#ifndef BUILD_MOCK
    /* Cold-boot the PMIC before every test. COLD BOOT (0x55) runs the full
     * INIT → NVM reload → ABIST → STANDBY → ACTIVE sequence, restoring every
     * NVM-backed register to factory defaults. This eliminates all register
     * state left by the previous test with no per-register bookkeeping. */
    platform_softReboot();

    /* Build a minimal stub handle for platform_rxByte (read-only; no CRC needed). */
    Pmic_Handle_t h = {0};
    h.commHandle0 = platform_getCommHandle();
    uint8_t regVal = 0U;
    int32_t status;

    /* Freeze WDG in Long Window so config registers stay writable across the
     * test. After WARM_RESET the WDG starts in Long Window, but setting
     * WD_PWRHOLD=1 ensures it stays there even if setUp takes time. */
    status = platform_rxByte(&h, 0U, PMIC_WD_MODE_REG_REG, &regVal, 1U);
    if (status == PMIC_ST_SUCCESS) {
        regVal |= (uint8_t)PMIC_WD_PWRHOLD_MASK;
        platform_writeReg(PMIC_WD_MODE_REG_REG, regVal);
    }

    /* Clear the recovery counter. WARM_RESET increments RESET_CNT (not
     * RECOV_CNT), but clear it anyway to avoid cascade failures if a prior
     * test pushed the device into SAFE state. RECOV_CNT_CLR is self-clearing. */
    platform_writeReg(PMIC_RECOV_CNT_CONTROL_REG, (uint8_t)PMIC_RECOV_CNT_CLR_MASK);

    /* Clear all IRQ latch registers. These are write-1-to-clear sticky bits;
     * WARM_RESET does NOT clear them. Latched IRQs from one test will corrupt
     * the next test's state if not cleared. */
    static const uint8_t irqRegs[] = {
        PMIC_INT_BUCK_LDO_REG,
        PMIC_INT_BUCK1_2_REG,
        PMIC_INT_BUCK3_LDO_REG,
        PMIC_INT_MISC_REG,
        PMIC_INT_MODERATE_ERR_REG,
        PMIC_INT_SEVERE_ERR_REG,
        PMIC_INT_FSM_ERR_REG,
        PMIC_INT_COMM_ERR_REG,
        PMIC_INT_ESM_REG,
        PMIC_WD_ERR_STATUS_REG,
    };
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(irqRegs) / sizeof(irqRegs[0])); i++) {
        platform_writeReg(irqRegs[i], 0xFFU);
    }
#endif /* !BUILD_MOCK */

    /* Mock: unlock registers and clear injected errors */
    platform_unlockRegisters();

#ifdef BUILD_MOCK
    extern PmicMockDevice_t* platform_getMockDevice(void);
    PmicMockDevice_t* mock = platform_getMockDevice();
    if (mock != NULL) {
        extern void PmicMock_ClearErrors(PmicMockDevice_t *device);
        PmicMock_ClearErrors(mock);
    }
#endif
}

/**
 * @brief Unity tearDown function - called after each test
 */
void tearDown(void)
{
    /* Unlock registers (in case test locked them) */
    platform_unlockRegisters();

    /* Clear mock errors */
    #ifdef BUILD_MOCK
    extern PmicMockDevice_t* platform_getMockDevice(void);
    PmicMockDevice_t* mock = platform_getMockDevice();
    if (mock != NULL) {
        extern void PmicMock_ClearErrors(PmicMockDevice_t *device);
        PmicMock_ClearErrors(mock);
    }
    #endif
}

/* Declare test entry functions */
extern void common_test(void *args);
extern void core_test(void *args);
extern void esm_test(void *args);
extern void fsm_test(void *args);
extern void gpio_test(void *args);
extern void io_test(void *args);
extern void irq_test(void *args);
extern void pmic_test(void *args);
extern void power_test(void *args);
extern void wdg_test(void *args);

/**
 * @brief Run all test modules
 */
static void runAllTests(void)
{
    if (testFilter_shouldRunModule("common")) {
        printf("\n=== Running Common Tests ===\n");
        common_test(NULL);
    }

    if (testFilter_shouldRunModule("pmic")) {
        printf("\n=== Running PMIC Init Tests ===\n");
        pmic_test(NULL);
    }

    if (testFilter_shouldRunModule("core")) {
        printf("\n=== Running Core Tests ===\n");
        core_test(NULL);
    }

    if (testFilter_shouldRunModule("fsm")) {
        printf("\n=== Running FSM Tests ===\n");
        fsm_test(NULL);
    }

    if (testFilter_shouldRunModule("irq")) {
        printf("\n=== Running IRQ Tests ===\n");
        irq_test(NULL);
    }

    if (testFilter_shouldRunModule("wdg")) {
        printf("\n=== Running WDG Tests ===\n");
        wdg_test(NULL);
    }

    if (testFilter_shouldRunModule("power")) {
        printf("\n=== Running Power Tests ===\n");
        power_test(NULL);
    }

    if (testFilter_shouldRunModule("esm")) {
        printf("\n=== Running ESM Tests ===\n");
        esm_test(NULL);
    }

    if (testFilter_shouldRunModule("gpio")) {
        printf("\n=== Running GPIO Tests ===\n");
        gpio_test(NULL);
    }

    if (testFilter_shouldRunModule("io")) {
        printf("\n=== Running I/O Tests ===\n");
        io_test(NULL);
    }
}

/**
 * @brief Main test runner entry point
 */
int main(void)
{
    printf("======================================\n");
    printf("TPS65036x-Q1 PMIC Unity Test Suite\n");
#ifdef BUILD_MOCK
    printf("Backend: Mock (hardware-independent)\n");
#else
    printf("Backend: Hardware\n");
#endif
    printf("======================================\n\n");

    debug_init();
    testFilter_init();

    testTimer_init();
    testTimer_startSuite();

    UNITY_BEGIN();
    platform_runTestLoop(&runAllTests);
    int result = UNITY_END();

    testTimer_endSuite();
    return result;
}
