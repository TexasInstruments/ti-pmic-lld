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
#include "test_utils.h"
#include "debug.h"
#include "test_filter.h"
#ifdef BUILD_MOCK
#include "pmic_mock_types.h"
#include "pmic_mock_core.h"
#endif

/**
 * @brief Unity setUp function - called before each test
 */
void setUp(void)
{
    /* Unlock registers to ensure clean state */
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
extern void wdg_test(void *args);
extern void esm_test(void *args);
extern void fsm_test(void *args);
extern void gpio_test(void *args);
extern void io_test(void *args);
extern void irq_test(void *args);
extern void pmic_test(void *args);
extern void power_test(void *args);
extern void timer_test(void *args);

/* ========================================================================= */
/*                         Test Module Registry                              */
/* ========================================================================= */

typedef void (*TestModuleFunc_t)(void *args);

typedef struct {
    const char *name;
    const char *displayName;
    TestModuleFunc_t func;
} TestModuleEntry_t;

static const TestModuleEntry_t g_testModules[] = {
    {"common", "Common",    common_test},
    {"pmic",   "PMIC Init", pmic_test},
    {"core",   "Core",      core_test},
    {"io",     "I/O",       io_test},
    {"fsm",    "FSM",       fsm_test},
    {"wdg",    "WDG",       wdg_test},
    {"esm",    "ESM",       esm_test},
    {"irq",    "IRQ",       irq_test},
    {"power",  "POWER",     power_test},
    {"gpio",   "GPIO",      gpio_test},
    {"timer",  "TIMER",     timer_test},
};

#define NUM_TEST_MODULES (sizeof(g_testModules) / sizeof(g_testModules[0]))

static void runAllTests(void)
{
    testFilter_printConfig();
    for (uint32_t i = 0; i < NUM_TEST_MODULES; i++) {
        const TestModuleEntry_t *module = &g_testModules[i];
        if (testFilter_shouldRunModule(module->name)) {
            module->func(NULL);
        }
    }
}

/**
 * @brief Main test runner entry point
 */
int main(void)
{
    debug_init();
    testFilter_init();

    printf("======================================\n");
    printf("TPS65386x-Q1 PMIC Unity Test Suite\n");
#ifdef BUILD_MOCK
    printf("Backend: Mock (hardware-independent)\n");
#else
    printf("Backend: Hardware\n");
#endif
    printf("======================================\n\n");

    testTimer_init();
    testTimer_startSuite();

    UNITY_BEGIN();

    platform_runTestLoop(&runAllTests);

    int result = UNITY_END();

    testTimer_endSuite();

    return result;
}
