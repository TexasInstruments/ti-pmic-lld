/******************************************************************************
 * unity_config.h - Unity Test Framework Configuration for Hardware Testing
 *
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
 *
 * Unity's default setjmp/longjmp mechanism works perfectly on ARM Cortex-M
 * bare-metal systems. This file exists only to provide TEST_UNPROTECT() for
 * use in our custom PLATFORM_RUN_TEST macro.
 *
 * Unity's built-in behavior:
 * - TEST_PROTECT() → setjmp(Unity.AbortFrame) → returns 0 on first call
 * - TEST_ABORT() → longjmp(Unity.AbortFrame, 1) → returns to TEST_PROTECT
 * - UNITY_FAIL_AND_BAIL → calls TEST_ABORT() after printing failure message
 *
 * This allows failed assertions to gracefully return control to the test
 * runner instead of hanging in abort() → _exit() → while(1) loop.
 *****************************************************************************/

#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

/**
 * @brief Configure Unity to use platform UART for output (legacy hardware builds only)
 *
 * On legacy hardware builds, UNITY_OUTPUT_CHAR wraps a real function (unity_output_char_impl)
 * instead of directly macro-expanding to platform_printChar.
 * This ensures proper calling convention and eliminates macro expansion
 * issues with string constant access.
 *
 * On mock/host builds, Unity uses its default putchar() implementation.
 */
#if !defined(BUILD_MOCK) && !defined(BUILD_HOST)
extern void unity_output_char_impl(int c);
extern void platform_printChar(char c);
#define UNITY_OUTPUT_CHAR(c) unity_output_char_impl(c)
#endif

/**
 * @brief Define TEST_UNPROTECT for cleanup after TEST_PROTECT
 *
 * Unity doesn't provide this macro, but we use it in PLATFORM_RUN_TEST
 * for symmetry with TEST_PROTECT(). It's a no-op since Unity's built-in
 * TEST_PROTECT mechanism already handles cleanup automatically.
 */
#define TEST_UNPROTECT() ((void)0)

#endif /* UNITY_CONFIG_H */
