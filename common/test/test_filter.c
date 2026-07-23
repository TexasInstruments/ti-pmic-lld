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
 * @file test_filter.c
 * @brief Implementation of test filtering system
 */

#include "test_filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#ifdef _WIN32
#define str_icmp(a, b) _stricmp((a), (b))
#else
#include <strings.h>
#define str_icmp(a, b) strcasecmp((a), (b))
#endif

/* Strip all non-alphanumeric characters so "I/O" matches "io", "i-o", etc. */
static void normalizeModuleName(const char *src, char *dst, size_t dst_len)
{
    size_t j = 0;
    for (size_t i = 0; src[i] && j < dst_len - 1; i++) {
        if (isalnum((unsigned char)src[i])) {
            dst[j++] = src[i];
        }
    }
    dst[j] = '\0';
}

/* ========================================================================= */
/*                         Global Variables                                  */
/* ========================================================================= */

/** Global filter configuration */
TestFilterConfig_t g_testFilter;

/* ========================================================================= */
/*                         Internal Helper Functions                         */
/* ========================================================================= */

/**
 * @brief Match string against wildcard pattern
 *
 * Supports:
 * - '*' matches zero or more characters
 * - '?' matches exactly one character
 * - Case-sensitive matching
 *
 * @param pattern Pattern with wildcards
 * @param str String to match
 * @return true if matches, false otherwise
 */
static bool matchWildcard(const char *pattern, const char *str)
{
    if (!pattern || !str) {
        return false;
    }

    while (*pattern && *str) {
        if (*pattern == '*') {
            pattern++;
            /* Trailing * matches everything */
            if (!*pattern) {
                return true;
            }
            /* Try matching rest of pattern at each position in str */
            while (*str) {
                if (matchWildcard(pattern, str)) {
                    return true;
                }
                str++;
            }
            return false;
        } else if (*pattern == '?' || *pattern == *str) {
            pattern++;
            str++;
        } else {
            return false;
        }
    }

    while (*pattern == '*') {
        pattern++;
    }
    return !*pattern && !*str;
}

/**
 * @brief Parse comma-separated module list from environment variable
 *
 * @param envValue Environment variable value
 * @param filter Module filter structure to populate
 */
static void parseModuleFilter(const char *envValue, TestModuleFilter_t *filter)
{
    if (!envValue || !filter) {
        return;
    }

    filter->numModules = 0;
    filter->allModules = false;

    /* Handle special "ALL" keyword */
    if (strcmp(envValue, "ALL") == 0 || strcmp(envValue, "all") == 0) {
        filter->allModules = true;
        return;
    }

    /* Parse comma-separated list */
    char buffer[256];
    strncpy(buffer, envValue, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token = strtok(buffer, ",");
    while (token && filter->numModules < TEST_FILTER_MAX_MODULES) {
        /* Trim leading/trailing whitespace */
        while (*token && isspace((unsigned char)*token)) {
            token++;
        }
        size_t len = strlen(token);
        while (len > 0 && isspace((unsigned char)token[len - 1])) {
            len--;
        }
        token[len] = '\0';

        if (len > 0) {
            strncpy(filter->modules[filter->numModules], token,
                    TEST_FILTER_MAX_MODULE_NAME_LEN - 1);
            filter->modules[filter->numModules][TEST_FILTER_MAX_MODULE_NAME_LEN - 1] = '\0';
            filter->numModules++;
        }

        token = strtok(NULL, ",");
    }
}

/**
 * @brief Parse test name pattern filter from environment variable
 *
 * @param envValue Environment variable value
 * @param filter Name filter structure to populate
 */
static void parseNameFilter(const char *envValue, TestNameFilter_t *filter)
{
    if (!envValue || !filter) {
        return;
    }

    filter->numPatterns = 0;
    filter->allTests = false;

    /* Handle special "ALL" keyword */
    if (strcmp(envValue, "ALL") == 0 || strcmp(envValue, "all") == 0) {
        filter->allTests = true;
        return;
    }

    /* Parse comma-separated patterns */
    char buffer[512];
    strncpy(buffer, envValue, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token = strtok(buffer, ",");
    while (token && filter->numPatterns < TEST_FILTER_MAX_PATTERNS) {
        /* Trim leading/trailing whitespace */
        while (*token && isspace((unsigned char)*token)) {
            token++;
        }
        size_t len = strlen(token);
        while (len > 0 && isspace((unsigned char)token[len - 1])) {
            len--;
        }
        token[len] = '\0';

        if (len > 0) {
            strncpy(filter->patterns[filter->numPatterns], token,
                    TEST_FILTER_MAX_PATTERN_LEN - 1);
            filter->patterns[filter->numPatterns][TEST_FILTER_MAX_PATTERN_LEN - 1] = '\0';
            filter->numPatterns++;
        }

        token = strtok(NULL, ",");
    }
}

/**
 * @brief Parse test group filter from environment variable
 *
 * @param envValue Environment variable value
 * @return Parsed group filter value
 */
static TestGroupFilter_e parseGroupFilter(const char *envValue)
{
    if (!envValue) {
        return TEST_GROUP_ALL;
    }

    if (strcmp(envValue, "positive") == 0 || strcmp(envValue, "pos") == 0) {
        return TEST_GROUP_POSITIVE;
    } else if (strcmp(envValue, "negative") == 0 || strcmp(envValue, "neg") == 0) {
        return TEST_GROUP_NEGATIVE;
    } else if (strcmp(envValue, "all") == 0 || strcmp(envValue, "ALL") == 0) {
        return TEST_GROUP_ALL;
    }

    /* Unknown value - default to all */
    printf("WARNING: Unknown PMIC_TEST_GROUPS value '%s', using 'all'\n", envValue);
    return TEST_GROUP_ALL;
}

/**
 * @brief Determine if test is positive based on naming convention
 *
 * @param testName Full test name
 * @return true if test is positive (test_pos_*), false if negative (test_neg_*)
 */
static bool isPositiveTest(const char *testName)
{
    if (!testName) {
        return true;
    }

    /* Check for test_pos_ prefix */
    if (strncmp(testName, "test_pos_", 9) == 0) {
        return true;
    }

    /* Check for test_neg_ prefix */
    if (strncmp(testName, "test_neg_", 9) == 0) {
        return false;
    }

    /* Default to positive if no clear indication */
    return true;
}

/* ========================================================================= */
/*                         Public API Implementation                         */
/* ========================================================================= */

void testFilter_init(void)
{
    /* Initialize to defaults (run everything) */
    g_testFilter.moduleFilter.numModules = 0;
    g_testFilter.moduleFilter.allModules = true;
    g_testFilter.nameFilter.numPatterns = 0;
    g_testFilter.nameFilter.allTests = true;
    g_testFilter.groupFilter = TEST_GROUP_ALL;

    /* Parse PMIC_TEST_MODULES */
    const char *modulesEnv = getenv("PMIC_TEST_MODULES");
    if (modulesEnv && strlen(modulesEnv) > 0) {
        parseModuleFilter(modulesEnv, &g_testFilter.moduleFilter);
        /* If modules were parsed, disable allModules flag */
        if (g_testFilter.moduleFilter.numModules > 0) {
            g_testFilter.moduleFilter.allModules = false;
        }
    }

    /* Parse PMIC_TEST_FILTER */
    const char *filterEnv = getenv("PMIC_TEST_FILTER");
    if (filterEnv && strlen(filterEnv) > 0) {
        parseNameFilter(filterEnv, &g_testFilter.nameFilter);
        /* If patterns were parsed, disable allTests flag */
        if (g_testFilter.nameFilter.numPatterns > 0) {
            g_testFilter.nameFilter.allTests = false;
        }
    }

    /* Parse PMIC_TEST_GROUPS */
    const char *groupsEnv = getenv("PMIC_TEST_GROUPS");
    if (groupsEnv && strlen(groupsEnv) > 0) {
        g_testFilter.groupFilter = parseGroupFilter(groupsEnv);
    }
}

bool testFilter_shouldRunModule(const char *moduleName)
{
    if (!moduleName) {
        return false;
    }

    /* If no filter specified, run all modules */
    if (g_testFilter.moduleFilter.allModules) {
        return true;
    }

    /* Check if module is in the enabled list (normalize both sides to strip
     * punctuation so "io" matches "I/O", "i-o", etc.) */
    char normModule[TEST_FILTER_MAX_MODULE_NAME_LEN];
    char normFilter[TEST_FILTER_MAX_MODULE_NAME_LEN];
    normalizeModuleName(moduleName, normModule, sizeof(normModule));
    for (uint8_t i = 0; i < g_testFilter.moduleFilter.numModules; i++) {
        normalizeModuleName(g_testFilter.moduleFilter.modules[i], normFilter, sizeof(normFilter));
        if (str_icmp(normFilter, normModule) == 0) {
            return true;
        }
    }

    return false;
}

bool testFilter_shouldRunTest(const char *testName)
{
    if (!testName) {
        return false;
    }

    /* If no filter specified, run all tests */
    if (g_testFilter.nameFilter.allTests) {
        return true;
    }

    /* Check if test name matches any pattern */
    for (uint8_t i = 0; i < g_testFilter.nameFilter.numPatterns; i++) {
        if (matchWildcard(g_testFilter.nameFilter.patterns[i], testName)) {
            return true;
        }
    }

    return false;
}

bool testFilter_shouldRunGroup(bool isPositive)
{
    /* If no filter specified, run all groups */
    if (g_testFilter.groupFilter == TEST_GROUP_ALL) {
        return true;
    }

    /* Check if group matches filter */
    if (g_testFilter.groupFilter == TEST_GROUP_POSITIVE && isPositive) {
        return true;
    }

    if (g_testFilter.groupFilter == TEST_GROUP_NEGATIVE && !isPositive) {
        return true;
    }

    return false;
}

void testFilter_printConfig(void)
{
    bool hasFilters = false;

    printf("=== Test Filter Configuration ===\n");

    /* Module filter */
    if (!g_testFilter.moduleFilter.allModules) {
        printf("Modules: ");
        for (uint8_t i = 0; i < g_testFilter.moduleFilter.numModules; i++) {
            printf("%s", g_testFilter.moduleFilter.modules[i]);
            if (i < g_testFilter.moduleFilter.numModules - 1) {
                printf(", ");
            }
        }
        printf(" (%d modules)\n", g_testFilter.moduleFilter.numModules);
        hasFilters = true;
    }

    /* Name filter */
    if (!g_testFilter.nameFilter.allTests) {
        printf("Filter: ");
        for (uint8_t i = 0; i < g_testFilter.nameFilter.numPatterns; i++) {
            printf("%s", g_testFilter.nameFilter.patterns[i]);
            if (i < g_testFilter.nameFilter.numPatterns - 1) {
                printf(", ");
            }
        }
        printf(" (pattern matching)\n");
        hasFilters = true;
    }

    /* Group filter */
    if (g_testFilter.groupFilter != TEST_GROUP_ALL) {
        if (g_testFilter.groupFilter == TEST_GROUP_POSITIVE) {
            printf("Groups: positive tests only\n");
        } else if (g_testFilter.groupFilter == TEST_GROUP_NEGATIVE) {
            printf("Groups: negative tests only\n");
        }
        hasFilters = true;
    }

    if (!hasFilters) {
        printf("No filters active - running all tests\n");
    }

    printf("=================================\n\n");
}

/**
 * @brief Enhanced filter check that combines all filtering levels
 *
 * This is a convenience function used by the PLATFORM_RUN_TEST macro
 * to check both name and group filters in a single call.
 *
 * @param testName Full test name
 * @return true if test should run, false otherwise
 */
bool testFilter_shouldRunTestWithGroup(const char *testName)
{
    if (!testName) {
        return false;
    }

    /* Check name filter first (quick rejection) */
    if (!testFilter_shouldRunTest(testName)) {
        return false;
    }

    /* Check group filter */
    bool isPositive = isPositiveTest(testName);
    if (!testFilter_shouldRunGroup(isPositive)) {
        return false;
    }

    return true;
}
