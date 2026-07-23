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
#ifndef TEST_TIMER_H
#define TEST_TIMER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the test timing system
 *
 * Must be called before any timing operations.
 * Resets all timing state.
 */
void testTimer_init(void);

/**
 * @brief Start timing a test
 * @param testName Name of the test being timed
 */
void testTimer_startTest(const char* testName);

/**
 * @brief End timing a test and record elapsed time
 *
 * Prints: " (XX.XX ms)" after test result
 */
void testTimer_endTest(void);

/**
 * @brief Start timing a module
 * @param moduleName Name of the module being timed
 */
void testTimer_startModule(const char* moduleName);

/**
 * @brief End timing a module and print summary
 *
 * Prints: "[MODULE] Module total: XX.XX ms (N tests)"
 */
void testTimer_endModule(void);

/**
 * @brief Start timing the entire test suite
 */
void testTimer_startSuite(void);

/**
 * @brief End timing the test suite and print summary
 *
 * Prints: "=== TOTAL TEST SUITE: XXXX.XX ms ==="
 */
void testTimer_endSuite(void);

/**
 * @brief Get elapsed time for current test in milliseconds
 * @return Elapsed time in ms, or 0.0 if no test is active
 */
double testTimer_getTestElapsed(void);

/**
 * @brief Get elapsed time for current module in milliseconds
 * @return Elapsed time in ms, or 0.0 if no module is active
 */
double testTimer_getModuleElapsed(void);

#endif /* TEST_TIMER_H */
