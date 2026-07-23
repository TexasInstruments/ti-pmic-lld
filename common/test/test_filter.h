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

/**
 * @file test_filter.h
 * @brief Test filtering configuration and utilities
 *
 * Provides a flexible test filtering system to run specific test modules,
 * test name patterns, or test groups (positive/negative) via environment
 * variables. This enables rapid debugging and focused test execution without
 * recompilation.
 *
 * Environment Variables:
 * - PMIC_TEST_MODULES: Comma-separated list of modules (e.g., "irq,power")
 * - PMIC_TEST_FILTER: Wildcard pattern for test names (e.g., "*mask*")
 * - PMIC_TEST_GROUPS: "positive", "negative", or "all" (default: "all")
 *
 * Usage Examples:
 * @code
 *   export PMIC_TEST_MODULES="irq"
 *   export PMIC_TEST_FILTER="*mask*"
 *   export PMIC_TEST_GROUPS="negative"
 *   make test BUILD=host
 * @endcode
 */

#ifndef TEST_FILTER_H
#define TEST_FILTER_H

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/** Maximum number of module filters */
#define TEST_FILTER_MAX_MODULES 16

/** Maximum number of test name filters */
#define TEST_FILTER_MAX_PATTERNS 32

/** Maximum length of a single pattern */
#define TEST_FILTER_MAX_PATTERN_LEN 128

/** Maximum length of module name */
#define TEST_FILTER_MAX_MODULE_NAME_LEN 32

/**
 * @brief Module filter configuration
 *
 * Tracks which test modules should be executed based on the
 * PMIC_TEST_MODULES environment variable.
 */
typedef struct {
    /** Module names to enable (parsed from comma-separated list) */
    char modules[TEST_FILTER_MAX_MODULES][TEST_FILTER_MAX_MODULE_NAME_LEN];
    /** Number of specified modules */
    uint8_t numModules;
    /** Run all modules if true (no filter specified) */
    bool allModules;
} TestModuleFilter_t;

/**
 * @brief Test name pattern filter
 *
 * Supports wildcard matching (* and ?) for flexible test name filtering
 * via the PMIC_TEST_FILTER environment variable.
 */
typedef struct {
    /** Wildcard patterns (e.g., "test_pos_*", "*mask*") */
    char patterns[TEST_FILTER_MAX_PATTERNS][TEST_FILTER_MAX_PATTERN_LEN];
    /** Number of patterns */
    uint8_t numPatterns;
    /** Run all tests if true (no filter specified) */
    bool allTests;
} TestNameFilter_t;

/**
 * @brief Test group filter (positive/negative)
 *
 * Allows filtering tests by their pos/neg classification via the
 * PMIC_TEST_GROUPS environment variable.
 */
typedef enum {
    TEST_GROUP_ALL = 0,      /**< Run all test groups (default) */
    TEST_GROUP_POSITIVE = 1, /**< Run only positive tests (test_pos_*) */
    TEST_GROUP_NEGATIVE = 2  /**< Run only negative tests (test_neg_*) */
} TestGroupFilter_e;

/**
 * @brief Complete filter configuration
 *
 * Combines all three filtering levels: module, name pattern, and group.
 * Filters are AND-ed together (all must pass for a test to run).
 */
typedef struct {
    TestModuleFilter_t moduleFilter; /**< Module-level filtering */
    TestNameFilter_t nameFilter;     /**< Test name pattern filtering */
    TestGroupFilter_e groupFilter;   /**< Test group filtering (pos/neg) */
} TestFilterConfig_t;

/* ========================================================================= */
/*                         Global Variables                                  */
/* ========================================================================= */

/** Global filter configuration (initialized by testFilter_init()) */
extern TestFilterConfig_t g_testFilter;

/* ========================================================================= */
/*                           Function Declarations                           */
/* ========================================================================= */

/**
 * @brief Initialize test filter from environment variables
 *
 * Parses the following environment variables to configure test filtering:
 * - PMIC_TEST_MODULES: Comma-separated module list (e.g., "common,irq,power")
 * - PMIC_TEST_FILTER: Wildcard pattern (e.g., "*mask*", "test_pos_*")
 * - PMIC_TEST_GROUPS: "positive", "negative", or "all" (default)
 *
 * Should be called once at startup before running tests.
 */
void testFilter_init(void);

/**
 * @brief Check if a module should run
 *
 * @param moduleName Module name (e.g., "common", "irq", "power")
 * @return true if module should run, false otherwise
 */
bool testFilter_shouldRunModule(const char *moduleName);

/**
 * @brief Check if a test should run based on its name
 *
 * Performs wildcard pattern matching against configured patterns.
 * Supports * (any characters) and ? (single character).
 *
 * @param testName Full test name (e.g., "test_pos_irq_irqSetMask_enable")
 * @return true if test should run, false otherwise
 */
bool testFilter_shouldRunTest(const char *testName);

/**
 * @brief Check if a test group should run (positive/negative)
 *
 * Extracts test type from test name (test_pos_* or test_neg_*) and
 * matches against configured group filter.
 *
 * @param isPositive true for positive tests, false for negative
 * @return true if group should run, false otherwise
 */
bool testFilter_shouldRunGroup(bool isPositive);

/**
 * @brief Print active filter configuration
 *
 * Displays configured filters to help users understand which tests
 * will be executed. Shows "No filters active" if running all tests.
 */
void testFilter_printConfig(void);

/**
 * @brief Enhanced filter check combining name and group filtering
 *
 * This convenience function checks both test name pattern and group
 * (positive/negative) filters in a single call. Used by PLATFORM_RUN_TEST
 * macro for efficient filtering.
 *
 * @param testName Full test name (e.g., "test_pos_irq_irqSetMask_enable")
 * @return true if test should run, false otherwise
 */
bool testFilter_shouldRunTestWithGroup(const char *testName);

#endif /* TEST_FILTER_H */
