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
    TEST_DEBUG(DEBUG_LEVEL_TRACE, "setUp: entry");

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

    TEST_DEBUG(DEBUG_LEVEL_TRACE, "setUp: completed");
}

/**
 * @brief Unity tearDown function - called after each test
 */
void tearDown(void)
{
    TEST_DEBUG(DEBUG_LEVEL_TRACE, "tearDown: entry");

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

    TEST_DEBUG(DEBUG_LEVEL_TRACE, "tearDown: completed");
}

/* Declare test entry functions */
extern void common_test(void *args);
extern void core_test(void *args);
extern void esm_test(void *args);
extern void fsm_test(void *args);
extern void io_test(void *args);
extern void irq_test(void *args);
extern void power_test(void *args);
extern void pmic_test(void *args);
extern void wdg_test(void *args);

/* ========================================================================= */
/*                         Test Module Registry                              */
/* ========================================================================= */

/**
 * @brief Test module function pointer type
 */
typedef void (*TestModuleFunc_t)(void *args);

/**
 * @brief Test module registry entry
 *
 * Defines a test module with its internal name (for filtering),
 * display name (for output), and entry function.
 */
typedef struct {
    const char *name;           /**< Module name for filtering (lowercase) */
    const char *displayName;    /**< Display name for output */
    TestModuleFunc_t func;      /**< Module entry function */
} TestModuleEntry_t;

/**
 * @brief Test module registry - all modules in execution order
 *
 * This table-driven approach makes it easy to add/remove test modules
 * and enables module-level filtering via PMIC_TEST_MODULES.
 */
static const TestModuleEntry_t g_testModules[] = {
    {"common", "Common",     common_test},
    {"pmic",   "PMIC Init",  pmic_test},
    {"core",   "Core",       core_test},
    {"io",     "I/O",        io_test},
    {"fsm",    "FSM",        fsm_test},
    {"wdg",    "WDG",        wdg_test},
    {"esm",    "ESM",        esm_test},
    {"irq",    "IRQ",        irq_test},
    {"power",  "POWER",      power_test},
};

/** Number of test modules */
#define NUM_TEST_MODULES (sizeof(g_testModules) / sizeof(g_testModules[0]))

/**
 * @brief Execute all test suites once (with filtering)
 * @note Called by platform_runTestLoop() - may be called multiple times on hardware
 */
static void runAllTests(void)
{
    printf("Starting tests\n");

    /* Print active filter configuration */
    testFilter_printConfig();

    /* Iterate through all registered test modules */
    for (uint32_t i = 0; i < NUM_TEST_MODULES; i++) {
        const TestModuleEntry_t *module = &g_testModules[i];

        /* Check if this module should run based on filters */
        if (testFilter_shouldRunModule(module->name)) {
            module->func(NULL);
        }
    }

    printf("All tests completed\n");
}

/**
 * @brief Main test runner entry point
 */
int main(void)
{
    /* Initialize debug system (reads PMIC_DEBUG_LEVEL and PMIC_DEBUG_MODULES env vars) */
    debug_init();

    /* Initialize test filter (reads PMIC_TEST_MODULES, PMIC_TEST_FILTER, PMIC_TEST_GROUPS) */
    testFilter_init();

    /* Initialize platform (works for both mock and hardware) */
    platform_init();

    printf("======================================\n\n");
#ifdef BUILD_MOCK
    printf("Backend: Mock\n");
#else
    printf("Backend: Hardware\n");
#endif
    printf("======================================\n\n");

    /* Initialize timing system */
    testTimer_init();
    testTimer_startSuite();

    /* Initialize Unity test framework */
    UNITY_BEGIN();

    /* Platform handles test loop (interactive on hardware, single-run on mock) */
    platform_runTestLoop(&runAllTests);

    /* Finalize Unity and get results */
    int result = UNITY_END();

    /* Print suite timing summary */
    testTimer_endSuite();

    return result;
}
