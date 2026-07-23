/**
 * @file debug.h
 * @brief Zero-overhead debug logging system for PMIC test infrastructure
 *
 * ARCHITECTURE:
 * - Default builds: All debug macros compile to no-ops (zero cost)
 * - Debug builds: Define PMIC_DEBUG=1 to enable runtime-controlled logging
 * - Runtime control: PMIC_DEBUG_LEVEL and PMIC_DEBUG_MODULES env vars
 * - Module filtering: Enable debug for specific components
 * - Level filtering: Control verbosity (ERROR, WARNING, INFO, DEBUG, TRACE)
 *
 * USAGE:
 *
 * Default build (zero overhead):
 *   make test BUILD=host
 *   -> All DEBUG_* macros compile to ((void)0), no runtime cost
 *
 * Debug build with full output:
 *   export PMIC_DEBUG_LEVEL=5
 *   export PMIC_DEBUG_MODULES=ALL
 *   make test-debug BUILD=host
 *
 * Debug build with selective filtering:
 *   export PMIC_DEBUG_LEVEL=3               # INFO and above
 *   export PMIC_DEBUG_MODULES=PLATFORM      # Platform layer only
 *   make test-debug BUILD=host
 *
 *   export PMIC_DEBUG_MODULES=PLATFORM,TEST # Multiple modules
 *   make test-debug BUILD=host
 *
 * EXAMPLE OUTPUT:
 *   [DEBUG][PLATFORM][INFO] Host-controlled platform initialization
 *   [DEBUG][PLATFORM][DEBUG] Serial port: /dev/cu.usbmodem0E2393591
 *   [DEBUG][PLATFORM][TRACE] >>> platform_unlockRegisters
 *   [DEBUG][TEST][TRACE] setUp: entry
 *
 * ADDING DEBUG STATEMENTS:
 *
 * In platform.c:
 *   #include "debug.h"
 *   PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Initializing platform");
 *   PLATFORM_DEBUG(DEBUG_LEVEL_DEBUG, "Serial port: %s", port);
 *
 * In test_runner.c:
 *   #include "debug.h"
 *   TEST_DEBUG(DEBUG_LEVEL_TRACE, "setUp: entry");
 *
 * In PMIC LLD code:
 *   #include "debug.h"
 *   LLD_DEBUG(DEBUG_LEVEL_DEBUG, "Register write: addr=0x%02X val=0x%02X", addr, val);
 */

#ifndef PMIC_DEBUG_H
#define PMIC_DEBUG_H

#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Debug levels - hierarchical (higher levels include lower levels)
 */
typedef enum {
    DEBUG_LEVEL_NONE    = 0,  /**< No debug output */
    DEBUG_LEVEL_ERROR   = 1,  /**< Critical errors only */
    DEBUG_LEVEL_WARNING = 2,  /**< Warnings + errors */
    DEBUG_LEVEL_INFO    = 3,  /**< Informational messages + above */
    DEBUG_LEVEL_DEBUG   = 4,  /**< Detailed debug info + above */
    DEBUG_LEVEL_TRACE   = 5   /**< Function entry/exit + above (most verbose) */
} DebugLevel_t;

/**
 * Debug modules - bitmask for filtering by component
 */
typedef enum {
    DEBUG_MODULE_PLATFORM = 0x01,  /**< Platform layer (init/deinit/I2C) */
    DEBUG_MODULE_TEST     = 0x02,  /**< Test runner and fixtures */
    DEBUG_MODULE_LLD      = 0x04,  /**< PMIC LLD API calls */
    DEBUG_MODULE_SERIAL   = 0x08,  /**< Serial communication */
    DEBUG_MODULE_ALL      = 0xFF   /**< All modules */
} DebugModule_t;

#ifdef PMIC_DEBUG

/**
 * Initialize debug system from environment variables
 * Call once at program startup (before any debug output)
 *
 * Environment variables:
 * - PMIC_DEBUG_LEVEL: 0-5 (0=none, 5=trace)
 * - PMIC_DEBUG_MODULES: Comma-separated list or "ALL"
 *   Examples: "PLATFORM", "PLATFORM,TEST", "ALL"
 */
void debug_init(void);

/**
 * Check if a debug message should be logged
 * @param module Debug module (e.g., DEBUG_MODULE_PLATFORM)
 * @param level Debug level (e.g., DEBUG_LEVEL_INFO)
 * @return true if message should be logged, false otherwise
 */
bool debug_should_log(DebugModule_t module, DebugLevel_t level);

/**
 * Get string name for debug module
 * @param module Debug module
 * @return String name (e.g., "PLATFORM")
 */
const char* debug_module_name(DebugModule_t module);

/**
 * Get string name for debug level
 * @param level Debug level
 * @return String name (e.g., "INFO")
 */
const char* debug_level_name(DebugLevel_t level);

/**
 * Core debug logging macro
 * Format: [DEBUG][MODULE][LEVEL] message
 *
 * @param module DebugModule_t value
 * @param level DebugLevel_t value
 * @param fmt printf-style format string
 * @param ... format arguments
 */
#define DEBUG_LOG(module, level, fmt, ...) \
    do { \
        if (debug_should_log(module, level)) { \
            printf("[DEBUG][%s][%s] " fmt "\n", \
                   debug_module_name(module), \
                   debug_level_name(level), \
                   ##__VA_ARGS__); \
            fflush(stdout); \
        } \
    } while(0)

/**
 * Function entry trace macro
 * @param module DebugModule_t value
 * @param func Function name (string literal)
 */
#define DEBUG_ENTRY(module, func) \
    DEBUG_LOG(module, DEBUG_LEVEL_TRACE, ">>> %s", func)

/**
 * Function exit trace macro
 * @param module DebugModule_t value
 * @param func Function name (string literal)
 */
#define DEBUG_EXIT(module, func) \
    DEBUG_LOG(module, DEBUG_LEVEL_TRACE, "<<< %s", func)

#else  /* PMIC_DEBUG not defined */

/**
 * When PMIC_DEBUG is not defined, all debug macros compile to no-ops
 * This ensures zero runtime cost and zero code size increase in default builds
 */
#define DEBUG_LOG(module, level, fmt, ...)  ((void)0)
#define DEBUG_ENTRY(module, func)           ((void)0)
#define DEBUG_EXIT(module, func)            ((void)0)

/* Define stub functions to avoid "implicit function declaration" warnings
 * if debug.c is not compiled but code still has calls to these functions */
static inline void debug_init(void) { }
static inline bool debug_should_log(DebugModule_t module, DebugLevel_t level) {
    (void)module; (void)level; return false;
}
static inline const char* debug_module_name(DebugModule_t module) {
    (void)module; return "";
}
static inline const char* debug_level_name(DebugLevel_t level) {
    (void)level; return "";
}

#endif  /* PMIC_DEBUG */

/*
 * Convenience macros per module
 * Use these in your code for cleaner debug statements
 */

/** Platform layer debug logging */
#define PLATFORM_DEBUG(level, ...) \
    DEBUG_LOG(DEBUG_MODULE_PLATFORM, level, __VA_ARGS__)

/** Test runner debug logging */
#define TEST_DEBUG(level, ...) \
    DEBUG_LOG(DEBUG_MODULE_TEST, level, __VA_ARGS__)

/** PMIC LLD debug logging */
#define LLD_DEBUG(level, ...) \
    DEBUG_LOG(DEBUG_MODULE_LLD, level, __VA_ARGS__)

/** Serial communication debug logging */
#define SERIAL_DEBUG(level, ...) \
    DEBUG_LOG(DEBUG_MODULE_SERIAL, level, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif  /* PMIC_DEBUG_H */
