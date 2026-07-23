/**
 * @file debug.c
 * @brief Runtime implementation of debug logging system
 *
 * Only compiled when PMIC_DEBUG is defined.
 * Provides runtime control of debug output via environment variables.
 */

#include "debug.h"

#ifdef PMIC_DEBUG

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/**
 * Global debug configuration
 */
static struct {
    DebugLevel_t level;           /**< Current debug level threshold */
    unsigned int module_mask;     /**< Bitmask of enabled modules */
    bool initialized;             /**< Whether debug_init() has been called */
} g_debug_config = {
    .level = DEBUG_LEVEL_NONE,
    .module_mask = 0,
    .initialized = false
};

/**
 * Helper: Convert string to uppercase (in-place)
 */
static void str_toupper(char* str) {
    if (!str) return;
    while (*str) {
        *str = (char)toupper((unsigned char)*str);
        str++;
    }
}

/**
 * Helper: Parse module name string to module enum
 * @param name Module name (case-insensitive)
 * @return DebugModule_t value, or (DebugModule_t)0 if unknown
 */
static DebugModule_t parse_module_name(const char* name) {
    if (!name) return (DebugModule_t)0;

    /* Make uppercase copy for comparison */
    char upper[32];
    strncpy(upper, name, sizeof(upper) - 1);
    upper[sizeof(upper) - 1] = '\0';
    str_toupper(upper);

    if (strcmp(upper, "ALL") == 0)      return DEBUG_MODULE_ALL;
    if (strcmp(upper, "PLATFORM") == 0) return DEBUG_MODULE_PLATFORM;
    if (strcmp(upper, "TEST") == 0)     return DEBUG_MODULE_TEST;
    if (strcmp(upper, "LLD") == 0)      return DEBUG_MODULE_LLD;
    if (strcmp(upper, "SERIAL") == 0)   return DEBUG_MODULE_SERIAL;

    return (DebugModule_t)0;
}

/**
 * Helper: Parse debug level from string
 * @param level_str Level as string ("0"-"5" or "NONE"/"ERROR"/etc)
 * @return DebugLevel_t value
 */
static DebugLevel_t parse_debug_level(const char* level_str) {
    if (!level_str) return DEBUG_LEVEL_NONE;

    /* Try parsing as number first */
    char* endptr;
    long level = strtol(level_str, &endptr, 10);
    if (*endptr == '\0' && level >= 0 && level <= 5) {
        return (DebugLevel_t)level;
    }

    /* Try parsing as name */
    char upper[32];
    strncpy(upper, level_str, sizeof(upper) - 1);
    upper[sizeof(upper) - 1] = '\0';
    str_toupper(upper);

    if (strcmp(upper, "NONE") == 0)    return DEBUG_LEVEL_NONE;
    if (strcmp(upper, "ERROR") == 0)   return DEBUG_LEVEL_ERROR;
    if (strcmp(upper, "WARNING") == 0) return DEBUG_LEVEL_WARNING;
    if (strcmp(upper, "INFO") == 0)    return DEBUG_LEVEL_INFO;
    if (strcmp(upper, "DEBUG") == 0)   return DEBUG_LEVEL_DEBUG;
    if (strcmp(upper, "TRACE") == 0)   return DEBUG_LEVEL_TRACE;

    return DEBUG_LEVEL_NONE;
}

/**
 * Initialize debug system from environment variables
 *
 * Environment variables:
 * - PMIC_DEBUG_LEVEL: Debug level (0-5 or NONE/ERROR/WARNING/INFO/DEBUG/TRACE)
 *   Default: 0 (NONE)
 *
 * - PMIC_DEBUG_MODULES: Comma-separated module list or "ALL"
 *   Examples: "PLATFORM", "PLATFORM,TEST", "ALL"
 *   Default: "" (no modules enabled)
 */
void debug_init(void) {
    if (g_debug_config.initialized) {
        return;  /* Already initialized */
    }

    /* Parse debug level */
    const char* level_env = getenv("PMIC_DEBUG_LEVEL");
    if (level_env) {
        g_debug_config.level = parse_debug_level(level_env);
    } else {
        g_debug_config.level = DEBUG_LEVEL_NONE;
    }

    /* Parse module mask */
    const char* modules_env = getenv("PMIC_DEBUG_MODULES");
    g_debug_config.module_mask = 0;

    if (modules_env && *modules_env) {
        /* Handle "ALL" specially */
        char upper_check[32];
        strncpy(upper_check, modules_env, sizeof(upper_check) - 1);
        upper_check[sizeof(upper_check) - 1] = '\0';
        str_toupper(upper_check);

        if (strcmp(upper_check, "ALL") == 0) {
            g_debug_config.module_mask = DEBUG_MODULE_ALL;
        } else {
            /* Parse comma-separated list */
            char modules_copy[256];
            strncpy(modules_copy, modules_env, sizeof(modules_copy) - 1);
            modules_copy[sizeof(modules_copy) - 1] = '\0';

            char* token = strtok(modules_copy, ",");
            while (token) {
                /* Skip leading/trailing whitespace */
                while (*token && isspace((unsigned char)*token)) token++;
                char* end = token + strlen(token) - 1;
                while (end > token && isspace((unsigned char)*end)) *end-- = '\0';

                if (*token) {
                    DebugModule_t module = parse_module_name(token);
                    g_debug_config.module_mask |= module;
                }

                token = strtok(NULL, ",");
            }
        }
    }

    g_debug_config.initialized = true;

    /* Print initialization info at INFO level */
    if (g_debug_config.level >= DEBUG_LEVEL_INFO && g_debug_config.module_mask) {
        printf("[DEBUG][SYSTEM][INFO] Debug system initialized: level=%s modules=0x%02X\n",
               debug_level_name(g_debug_config.level),
               g_debug_config.module_mask);
        fflush(stdout);
    }
}

/**
 * Check if a debug message should be logged
 */
bool debug_should_log(DebugModule_t module, DebugLevel_t level) {
    if (!g_debug_config.initialized) {
        debug_init();  /* Auto-initialize if not done yet */
    }

    /* Check level threshold */
    if (level > g_debug_config.level) {
        return false;
    }

    /* Check module mask */
    if ((g_debug_config.module_mask & module) == 0) {
        return false;
    }

    return true;
}

/**
 * Get string name for debug module
 */
const char* debug_module_name(DebugModule_t module) {
    switch (module) {
        case DEBUG_MODULE_PLATFORM: return "PLATFORM";
        case DEBUG_MODULE_TEST:     return "TEST";
        case DEBUG_MODULE_LLD:      return "LLD";
        case DEBUG_MODULE_SERIAL:   return "SERIAL";
        case DEBUG_MODULE_ALL:      return "ALL";
        default:                    return "UNKNOWN";
    }
}

/**
 * Get string name for debug level
 */
const char* debug_level_name(DebugLevel_t level) {
    switch (level) {
        case DEBUG_LEVEL_NONE:    return "NONE";
        case DEBUG_LEVEL_ERROR:   return "ERROR";
        case DEBUG_LEVEL_WARNING: return "WARNING";
        case DEBUG_LEVEL_INFO:    return "INFO";
        case DEBUG_LEVEL_DEBUG:   return "DEBUG";
        case DEBUG_LEVEL_TRACE:   return "TRACE";
        default:                  return "UNKNOWN";
    }
}

#endif  /* PMIC_DEBUG */
