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
#include "regmap/esm.h"
#include "regmap/fsm.h"
#ifdef BUILD_MOCK
#include "pmic_mock_types.h"
#include "pmic_mock_core.h"
#include "test_inject.h"
#endif

/* Unity framework callbacks */
/**
 * @brief Unity setUp function - called before each test
 */
void setUp(void)
{
    static const uint8_t clrVal = 0xFFU;
    static const struct { uint8_t page; uint16_t addr; } irqRegs[] = {
        { PMIC_PAGE_MAIN, INT_BUCK_REG         },
        { PMIC_PAGE_MAIN, INT_LDO_VMON_REG     },
        { PMIC_PAGE_MAIN, INT_GPIO_REG         },
        { PMIC_PAGE_MAIN, INT_STARTUP_REG      },
        { PMIC_PAGE_MAIN, INT_MISC_REG         },
        { PMIC_PAGE_MAIN, INT_MODERATE_ERR_REG },
        { PMIC_PAGE_MAIN, INT_SEVERE_ERR_REG   },
        { PMIC_PAGE_MAIN, INT_FSM_ERR_REG      },
        { PMIC_PAGE_MAIN, INT_ESM_REG          },
        { PMIC_PAGE_WDG,  WD_ERR_STATUS_REG    },
    };
    char msg[64] = {0};
    uint8_t recov2 = 0U;

    Pmic_Handle_t h = {0};
    h.commHandle0 = platform_getCommHandle0();
    h.commHandle1 = platform_getCommHandle1();
    h.commMode    = PMIC_INTF_I2C_DUAL;
    h.i2cAddr0    = PLATFORM_I2C_ADDR_MAIN;
    h.i2cAddr1    = PLATFORM_I2C_ADDR_SECONDARY;

#ifndef BUILD_MOCK
    platform_softReboot();

    static const struct { uint8_t page; uint16_t addr; const char *name; } statusRegs[] = {
        { PMIC_PAGE_MAIN, INT_TOP_REG,           "INT_TOP      " },
        { PMIC_PAGE_MAIN, STAT_SEVERE_ERR_REG,   "STAT_SEVERE  " },
        { PMIC_PAGE_MAIN, STAT_MODERATE_ERR_REG, "STAT_MODERATE" },
        { PMIC_PAGE_MAIN, RECOV_CNT_REG_1_REG,   "RECOV_CNT    " },
        { PMIC_PAGE_MAIN, STARTUP_CTRL_REG,      "STARTUP_CTRL " },
    };
    uint8_t regVal = 0U;
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(statusRegs) / sizeof(statusRegs[0])); i++) {
        if (platform_rxByte(&h, statusRegs[i].page, statusRegs[i].addr, &regVal, 1U) == PMIC_ST_SUCCESS) {
            if (regVal != 0U) {
                printf("[PMIC STATE] %s = 0x%02X\n", statusRegs[i].name, regVal);
            }
        }
    }
#endif

    platform_unlockRegisters();

#ifndef BUILD_MOCK
    /* Freeze WDG in Long Window so config registers stay writable across tests */
    if (platform_rxByte(&h, PMIC_PAGE_WDG, (uint8_t)(WD_MODE_REG_REG & 0xFFU), &regVal, 1U) == PMIC_ST_SUCCESS) {
        regVal |= (uint8_t)WD_PWRHOLD_MASK;
        (void)platform_txByte(&h, PMIC_PAGE_WDG, (uint8_t)(WD_MODE_REG_REG & 0xFFU), &regVal, 1U);
    }

    /* Clear ESM_MCU_START so ESM config registers are writable */
    if (platform_rxByte(&h, PMIC_PAGE_MAIN, (uint8_t)(ESM_MCU_START_REG_REG & 0xFFU), &regVal, 1U) == PMIC_ST_SUCCESS) {
        regVal &= ~(uint8_t)ESM_MCU_START_MASK;
        (void)platform_txByte(&h, PMIC_PAGE_MAIN, (uint8_t)(ESM_MCU_START_REG_REG & 0xFFU), &regVal, 1U);
    }
#endif

    int32_t status = platform_rxByte(&h, PMIC_PAGE_MAIN, RECOV_CNT_REG_2_REG, &recov2, 1U);
    if (status != PMIC_ST_SUCCESS)
    {
        (void)sprintf(msg, "ERROR: Recovery counter read failed: %d\r\n", status);
        platform_printString(msg);
    }
    recov2 |= (uint8_t)RECOV_CNT_CLR_MASK;
    status = platform_txByte(&h, PMIC_PAGE_MAIN, RECOV_CNT_REG_2_REG, &recov2, 1U);
    if (status != PMIC_ST_SUCCESS)
    {
        (void)sprintf(msg, "ERROR: Recovery counter clear failed: %d\r\n", status);
        platform_printString(msg);
    }

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(irqRegs) / sizeof(irqRegs[0])); i++)
    {
        status = platform_txByte(&h, irqRegs[i].page, irqRegs[i].addr, &clrVal, 1U);
        if (status != PMIC_ST_SUCCESS)
        {
            (void)sprintf(msg, "ERROR: IRQ clear failed (reg 0x%04X): %d\r\n",
                          irqRegs[i].addr, status);
            platform_printString(msg);
        }
    }

#ifdef BUILD_MOCK
    extern PmicMockDevice_t* platform_getMockDevice(void);
    PmicMockDevice_t* mock = platform_getMockDevice();
    if (mock != NULL) {
        extern void PmicMock_ClearErrors(PmicMockDevice_t *device);
        PmicMock_ClearErrors(mock);
        testInject_init(mock);
    }
#endif
}

/**
 * @brief Unity tearDown function - called after each test
 */
void tearDown(void)
{
#ifdef BUILD_MOCK
    extern PmicMockDevice_t* platform_getMockDevice(void);
    PmicMockDevice_t* mock = platform_getMockDevice();
    if (mock != NULL) {
        extern void PmicMock_ClearErrors(PmicMockDevice_t *device);
        PmicMock_ClearErrors(mock);
    }
#endif
}

/* Declare test module entry functions */
extern void common_test(void *args);
extern void pmic_test(void *args);
extern void core_test(void *args);
extern void adc_test(void *args);
extern void power_test(void *args);
extern void gpio_test(void *args);
extern void wdg_test(void *args);
extern void esm_test(void *args);
extern void io_test(void *args);
extern void irq_test(void *args);
extern void fsm_test(void *args);

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
    {"adc",    "ADC",       adc_test},
    {"power",  "Power",     power_test},
    {"gpio",   "GPIO",      gpio_test},
    {"wdg",    "WDG",       wdg_test},
    {"esm",    "ESM",       esm_test},
    {"io",     "I/O",       io_test},
    {"irq",    "IRQ",       irq_test},
    {"fsm",    "FSM",       fsm_test},
};

#define NUM_TEST_MODULES (sizeof(g_testModules) / sizeof(g_testModules[0]))

static void runAllTests(void)
{
    testFilter_printConfig();
    for (uint32_t i = 0U; i < NUM_TEST_MODULES; i++) {
        const TestModuleEntry_t *module = &g_testModules[i];
        if (testFilter_shouldRunModule(module->name)) {
            platform_setModuleName(module->displayName);
            module->func(NULL);
        }
    }
}

/**
 * @brief Main entry point for test execution
 *
 * @return int Test result (0 = success, non-zero = failures)
 */
int main(void)
{
    printf("========================================\n");
    printf("TPS6522x-Q1 PMIC Unity Test Suite\n");
    printf("========================================\n");
#ifdef BUILD_MOCK
    printf("Backend: Mock (hardware-independent)\n");
#else
    printf("Backend: Hardware (TI TM4C123)\n");
#endif
    printf("========================================\n\n");

#ifndef BUILD_MOCK
    platform_init();
    {
        static const struct { uint8_t page; uint16_t addr; const char *name; } infoRegs[] = {
            { PMIC_PAGE_MAIN, DEV_REV_REG,             "DEV_REV"          },
            { PMIC_PAGE_MAIN, NVM_CODE_1_REG,          "NVM_CODE_1"       },
            { PMIC_PAGE_MAIN, NVM_CODE_2_REG,          "NVM_CODE_2"       },
            { PMIC_PAGE_MAIN, MANUFACTURING_VER_REG,   "MANUFACTURING_VER" },
            { PMIC_PAGE_MAIN, CUSTOMER_NVM_ID_REG_REG, "CUSTOMER_NVM_ID_REG" },
        };
        Pmic_Handle_t h = {0};
        h.commHandle0 = platform_getCommHandle0();
        h.commHandle1 = platform_getCommHandle1();
        h.commMode    = PMIC_INTF_I2C_DUAL;
        h.i2cAddr0    = PLATFORM_I2C_ADDR_MAIN;
        h.i2cAddr1    = PLATFORM_I2C_ADDR_SECONDARY;
        uint8_t regVal = 0U;
        int32_t status;
        for (uint8_t i = 0U; i < (uint8_t)(sizeof(infoRegs) / sizeof(infoRegs[0])); i++) {
            status = platform_rxByte(&h, infoRegs[i].page, infoRegs[i].addr, &regVal, 1U);
            if (status == PMIC_ST_SUCCESS) {
                printf("[DEVICE INFO] %-20s = 0x%02X\n", infoRegs[i].name, regVal);
            } else {
                printf("[DEVICE INFO] %-20s = ERROR (status=%d)\n", infoRegs[i].name, status);
            }
        }
    }
    printf("\n");
#endif

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
