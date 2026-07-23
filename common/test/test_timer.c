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

#include "test_timer.h"
#include <stdio.h>
#include <sys/time.h>
#include <string.h>

/* Forward declaration of platform function */
extern void platform_setModuleName(const char* moduleName);

/* ========================================================================== */
/*                           Internal Data Types                             */
/* ========================================================================== */

/**
 * @brief Timing context for a single timing level (test/module/suite)
 */
typedef struct {
    struct timeval startTime;   /**< Start timestamp */
    struct timeval endTime;     /**< End timestamp (only valid after stop) */
    bool active;                /**< Whether timing is currently active */
    const char* name;           /**< Name of the item being timed */
    uint32_t testCount;         /**< Number of tests executed (module level only) */
} TimingContext_t;

/* ========================================================================== */
/*                           Internal State                                  */
/* ========================================================================== */

static TimingContext_t g_testContext;
static TimingContext_t g_moduleContext;
static TimingContext_t g_suiteContext;

/* ========================================================================== */
/*                           Internal Helper Functions                       */
/* ========================================================================== */

/**
 * @brief Calculate elapsed time in milliseconds between two timevals
 * @param start Start time
 * @param end End time
 * @return Elapsed time in milliseconds (with microsecond precision)
 */
static double calculateElapsedMs(const struct timeval* start, const struct timeval* end)
{
    double seconds = (double)(end->tv_sec - start->tv_sec);
    double microseconds = (double)(end->tv_usec - start->tv_usec);
    return (seconds * 1000.0) + (microseconds / 1000.0);
}

/**
 * @brief Start timing for a context
 * @param ctx Timing context to start
 * @param name Name of the item being timed
 */
static void startTiming(TimingContext_t* ctx, const char* name)
{
    ctx->name = name;
    ctx->active = (bool)true;
    gettimeofday(&ctx->startTime, NULL);
}

/**
 * @brief Stop timing for a context
 * @param ctx Timing context to stop
 */
static void stopTiming(TimingContext_t* ctx)
{
    gettimeofday(&ctx->endTime, NULL);
    ctx->active = (bool)false;
}

/**
 * @brief Get elapsed time for a context in milliseconds
 * @param ctx Timing context
 * @return Elapsed time in ms, or 0.0 if context is not active
 */
static double getElapsed(const TimingContext_t* ctx)
{
    if (!ctx->active) {
        return 0.0;
    }

    struct timeval now;
    gettimeofday(&now, NULL);
    return calculateElapsedMs(&ctx->startTime, &now);
}

/* ========================================================================== */
/*                           Public API Implementation                       */
/* ========================================================================== */

void testTimer_init(void)
{
    memset(&g_testContext, 0, sizeof(g_testContext));
    memset(&g_moduleContext, 0, sizeof(g_moduleContext));
    memset(&g_suiteContext, 0, sizeof(g_suiteContext));
}

void testTimer_startTest(const char* testName)
{
    startTiming(&g_testContext, testName);
}

void testTimer_endTest(void)
{
    if (!g_testContext.active) {
        return;
    }

    stopTiming(&g_testContext);
    double elapsedMs = calculateElapsedMs(&g_testContext.startTime, &g_testContext.endTime);

    /* Print timing in format: " (12.34 ms)" */
    printf(" (%.2f ms)", elapsedMs);

    /* Increment test count for module */
    if (g_moduleContext.active) {
        g_moduleContext.testCount++;
    }
}

void testTimer_startModule(const char* moduleName)
{
    platform_setModuleName(moduleName);  /* Set for PLATFORM_RUN_TEST macro */
    startTiming(&g_moduleContext, moduleName);
    g_moduleContext.testCount = 0;
}

void testTimer_endModule(void)
{
    if (!g_moduleContext.active) {
        return;
    }

    stopTiming(&g_moduleContext);
    double elapsedMs = calculateElapsedMs(&g_moduleContext.startTime, &g_moduleContext.endTime);

    /* Print module summary in format: "[MODULE] Module total: 1234.56 ms (N tests)" */
    printf("[%s] Module total: %.2f ms (%u tests)\n",
           g_moduleContext.name,
           elapsedMs,
           g_moduleContext.testCount);
}

void testTimer_startSuite(void)
{
    startTiming(&g_suiteContext, "Test Suite");
}

void testTimer_endSuite(void)
{
    if (!g_suiteContext.active) {
        return;
    }

    stopTiming(&g_suiteContext);
    double elapsedMs = calculateElapsedMs(&g_suiteContext.startTime, &g_suiteContext.endTime);

    /* Print suite summary in format: "=== TOTAL TEST SUITE: 12345.67 ms ===" */
    printf("\n=== TOTAL TEST SUITE: %.2f ms ===\n", elapsedMs);
}

double testTimer_getTestElapsed(void)
{
    return getElapsed(&g_testContext);
}

double testTimer_getModuleElapsed(void)
{
    return getElapsed(&g_moduleContext);
}
