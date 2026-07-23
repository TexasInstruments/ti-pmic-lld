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
#include "unity_config.h"
#include "platform.h"
#ifdef BUILD_MOCK
#include "pmic_mock_types.h"
#include "pmic_mock_core.h"
#endif

/**
 * @brief Unity setUp function - called before each test
 *
 * Unlocks registers before every test to ensure clean state.
 * This prevents test failures caused by FSM tests that intentionally
 * lock registers as part of testing FSM state transitions.
 */
void setUp(void)
{
    /* Unlock registers to ensure clean state */
    extern void platform_unlockRegisters(void);
    platform_unlockRegisters();

    /* Clear mock errors if in mock mode */
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
    extern void platform_unlockRegisters(void);
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
extern void io_test(void *args);
#ifdef BUILD_MOCK
extern void irq_test(void *args);
extern void power_test(void *args);
#endif
extern void pmic_test(void *args);
extern void wdg_test(void *args);

/**
 * @brief Execute all test suites once
 * @note Called by platform_runTestLoop() - may be called multiple times on hardware
 */
static void runAllTests(void)
{
    printf("Starting tests\n");

    printf("\n=== Running Common Tests ===\n");
    common_test(NULL);

    printf("\n=== Running PMIC Init Tests ===\n");
    pmic_test(NULL);

    printf("\n=== Running Core Tests ===\n");
    core_test(NULL);

    printf("\n=== Running I/O Tests ===\n");
    io_test(NULL);

    printf("\n=== Running FSM Tests ===\n");
    fsm_test(NULL);

    printf("\n=== Running WDG Tests ===\n");
    wdg_test(NULL);

    printf("\n=== Running ESM Tests ===\n");
    esm_test(NULL);

    printf("All tests completed\n");
}

/**
 * @brief Main test runner entry point
 */
int main(void)
{
    // Initialize platform (works for both mock and hardware)
    platform_init();

    printf("======================================\n\n");
#ifdef BUILD_MOCK
    printf("Backend: Mock\n");
#else
    printf("Backend: Hardware\n");
#endif
    printf("======================================\n\n");

    /* Initialize Unity test framework */
    UNITY_BEGIN();

    /* Platform handles test loop (interactive on hardware, single-run on mock) */
    platform_runTestLoop(&runAllTests);

    /* Finalize Unity and get results */
    int result = UNITY_END();

    return result;
}
