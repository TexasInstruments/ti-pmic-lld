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
 * @file platform.c
 * @brief Host-controlled platform layer for PMIC testing
 *
 * This implementation runs tests on the host PC while communicating with
 * pmic-tiva-host firmware on TM4C123 via serial commands. The firmware acts
 * as an I2C/GPIO translation layer, providing unlimited memory and easy
 * debugging while still accessing real PMIC hardware.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "platform.h"
#include "regmap/fsm.h"
#include "debug.h"
#include <stdbool.h>

#ifdef BUILD_HOST
#include "platform_serial.h"
#include <unistd.h>  /* For usleep */
#include <stdlib.h>  /* For strtoul, getenv, exit */
#endif

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @brief Pseudo I2C handle for host mode (just stores slave address)
 */
typedef struct {
    uint8_t slaveAddr;
} I2cHandle_t;

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static I2cHandle_t commHandle0 = {0U};  /* Main I2C (port 2, addr 0x48) */
static I2cHandle_t commHandle1 = {0U};  /* Secondary I2C (port 0, addr 0x12) */

/**
 * @brief Current module name for test result prefixes
 */
const char* g_currentModuleName = NULL;

/**
 * @brief Platform initialization state
 */
static bool g_platform_initialized = false;

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

/**
 * @brief Set the current module name for test result prefixes
 * @param moduleName The module name to use as a prefix (e.g., "ESM", "WDG")
 */
void platform_setModuleName(const char* moduleName)
{
    g_currentModuleName = moduleName;
}

void platform_init(void)
{
#ifdef BUILD_HOST
    const char *port;
    char response[256];
    int32_t status;

    /* If already initialized, just return success */
    if (g_platform_initialized) {
        PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Already initialized, skipping");
        return;
    }

    PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Host-controlled platform initialization");

    /* Reset initialization state */
    g_platform_initialized = false;
    commHandle0.slaveAddr = 0x00;
    commHandle1.slaveAddr  = 0x00;

    /* Get serial port from environment or use default */
    port = getenv("PMIC_SERIAL_PORT");
    if (port == NULL) {
        port = SERIAL_DEFAULT_PORT;
    }

    PLATFORM_DEBUG(DEBUG_LEVEL_DEBUG, "Serial port: %s", port);

    printf("Initializing host-controlled PMIC testing...\n");
    printf("Serial port: %s\n", port);
    fflush(stdout);

    /* Initialize serial communication */
    status = serial_init(port, SERIAL_DEFAULT_BAUD);
    if (status != 0) {
        fprintf(stderr, "ERROR: Failed to open serial port %s\n", port);
        fprintf(stderr, "       %s\n", serial_get_last_error());
        fprintf(stderr, "\nTroubleshooting:\n");
        fprintf(stderr, "  1. Check that pmic-tiva-host firmware is flashed to TM4C123\n");
        fprintf(stderr, "  2. Verify USB cable is connected\n");
        fprintf(stderr, "  3. Check port name (set PMIC_SERIAL_PORT environment variable)\n");
        fprintf(stderr, "  4. On Linux: Add user to dialout group (sudo usermod -a -G dialout $USER)\n");
        exit(1);
    }

    /* Verify firmware connection - send 'id' command */
    printf("Verifying firmware connection...\n");

    /* Wait for startup banner to complete */
    usleep(500000);  /* 500ms */

    /* Flush any stale data (startup banner) */
    serial_flush();

    status = serial_send_command("id");
    if (status != 0) {
        fprintf(stderr, "ERROR: Failed to send command to firmware\n");
        fprintf(stderr, "       %s\n", serial_get_last_error());
        serial_close();
        exit(1);
    }

    /* Read response (should be "STATUS: OK" followed by "ID: PPC Host...") */
    memset(response, 0, sizeof(response));
    status = serial_read_response(response, sizeof(response));
    if (status < 0) {
        fprintf(stderr, "ERROR: No response from firmware\n");
        fprintf(stderr, "       %s\n", serial_get_last_error());
        fprintf(stderr, "\nFirmware is not responding. Check that:\n");
        fprintf(stderr, "  1. pmic-tiva-host firmware is flashed and running\n");
        fprintf(stderr, "  2. Board is powered on\n");
        fprintf(stderr, "  3. UART0 is functioning (TX=PA1, RX=PA0)\n");
        serial_close();
        exit(1);
    }

    /* Check for "STATUS: OK" in first line */
    if (strstr(response, "STATUS: OK") == NULL) {
        fprintf(stderr, "ERROR: Firmware returned error: %s\n", response);
        serial_close();
        exit(1);
    }

    /* Read ID line */
    memset(response, 0, sizeof(response));
    status = serial_read_response(response, sizeof(response));
    if (status < 0) {
        fprintf(stderr, "ERROR: Firmware ID read failed\n");
        fprintf(stderr, "       %s\n", serial_get_last_error());
        serial_close();
        exit(1);
    }

    /* Validate firmware ID contains "PPC Host" */
    if (strstr(response, "PPC Host") == NULL) {
        fprintf(stderr, "ERROR: Invalid firmware ID: %s\n", response);
        fprintf(stderr, "       Expected 'PPC Host' in firmware identification\n");
        serial_close();
        exit(1);
    }

    printf("Firmware connected: %s\n", response);
    printf("Host-controlled platform initialized successfully\n\n");

    /* Configure nRSTOUT monitor pin (PB5) as input with weak pull-up.
     * nRSTOUT is open-drain active-low; WPU is required for correct reads.
     * gpioc uses atoi() server-side so bitmask must be decimal (32 = GPIO_PIN_5).
     * pullupdown=10 = GPIO_PIN_TYPE_STD_WPU in TivaWare. */
    memset(response, 0, sizeof(response));
    status = serial_send_command("gpioc pb 32 i 10 1");
    if (status == 0) {
        serial_read_response(response, sizeof(response));
    }

    /* Configure PA2 as GPIO output for ESM_IN (PMIC GPIO6/nERR_MCU). Drive HIGH
     * initially to provide a valid "no fault" level-mode signal.
     * Port A clock is already enabled by firmware (UART0 uses PA0/PA1); PA2 is free.
     * bitmask 4 = 0x04 = PA2; pulltype 8 = GPIO_PIN_TYPE_STD; strength 1 = 2MA. */
    memset(response, 0, sizeof(response));
    status = serial_send_command("gpioc pa 4 o 8 1");
    if (status == 0) {
        serial_read_response(response, sizeof(response));
    }
    memset(response, 0, sizeof(response));
    status = serial_send_command("gpiow pa 4 4 0");
    if (status == 0) {
        serial_read_response(response, sizeof(response));
    }

    /* Only set address and state after all validation passes */
    commHandle0.slaveAddr = PLATFORM_TARGET_I2C_ADDR;
    commHandle1.slaveAddr  = PLATFORM_I2C_ADDR_SECONDARY;
    g_platform_initialized = true;

    PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Initialization complete - main: 0x%02X, sec: 0x%02X",
                   PLATFORM_TARGET_I2C_ADDR, PLATFORM_I2C_ADDR_SECONDARY);
#endif
}

void platform_deinit(void)
{
#ifdef BUILD_HOST
    PLATFORM_DEBUG(DEBUG_LEVEL_DEBUG, "platform_deinit called (initialized=%d)", g_platform_initialized);

    /* Skip deinit when running test suites - keep port open for next suite */
    /* Only truly deinit when explicitly needed (not during normal test runs) */
    if (g_platform_initialized) {
        PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Keeping port open for test suites");
        /* Keep the port open - just return without deinitialization */
        return;
    }

    PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Performing actual deinitialization");
#endif
    /* This code only runs if already deinitialized */
    g_platform_initialized = false;
    commHandle0.slaveAddr = 0x00;
    commHandle1.slaveAddr  = 0x00;
}

#ifdef BUILD_HOST
static void platform_waitNRSTOUT(void)
{
    char response[256];
    uint32_t elapsed_ms;
    bool ready = false;

    /* Brief initial wait for the PMIC to start asserting nRSTOUT low */
    usleep((useconds_t)PLATFORM_REBOOT_INITIAL_WAIT_MS * 1000U);
    elapsed_ms = PLATFORM_REBOOT_INITIAL_WAIT_MS;

    while (elapsed_ms < PLATFORM_REBOOT_TIMEOUT_MS) {
        if (serial_send_command("gpior pb 32") != 0) break;

        if (serial_read_response(response, sizeof(response)) < 0) break;
        if (strstr(response, "STATUS: OK") == NULL) break;

        if (serial_read_response(response, sizeof(response)) < 0) break;

        /* Last "0x"-prefixed token in the RESULTS line is the pin value */
        char *cursor = response;
        char *last_hex = NULL;
        while ((cursor = strstr(cursor, "0x")) != NULL) { last_hex = cursor; cursor += 2; }
        if (last_hex != NULL) {
            uint32_t val = (uint32_t)strtoul(last_hex, NULL, 0);
            if ((val & 0x20U) != 0U) { ready = true; break; }
        }

        usleep(10000U);
        elapsed_ms += 10U;
    }

    if (ready) {
        printf("[SOFT REBOOT] nRSTOUT HIGH after %ums\n", elapsed_ms);
    } else {
        printf("[SOFT REBOOT] WARNING: nRSTOUT timeout after %ums, continuing\n", elapsed_ms);
    }
}
#endif

void platform_softReboot(void)
{
#ifdef BUILD_HOST
    Pmic_Handle_t h = {0};
    h.i2cAddr0 = PLATFORM_TARGET_I2C_ADDR;
    h.i2cAddr1 = PLATFORM_I2C_ADDR_SECONDARY;
    uint8_t val = (uint8_t)SOFT_REBOOT_MASK;
    int32_t status = platform_txByte(&h, PMIC_PAGE_MAIN, (uint8_t)SOFT_REBOOT_REG_REG, &val, 1U);
    if (status != PMIC_ST_SUCCESS)
    {
        printf("[SOFT REBOOT] ERROR: I2C write failed (status=%d)\n", status);
        return;
    }
    printf("[SOFT REBOOT] triggered\n");
    platform_waitNRSTOUT();
#endif
}

void platform_setupTests(void)
{
    /* No setup needed for host mode */
}

void platform_tearDownTests(void)
{
    /* No teardown needed for host mode */
}

void platform_printChar(char c)
{
    if (c == '\n') {
        putchar('\r');  /* CR first */
        putchar('\n');  /* LF second = proper CRLF */
        fflush(stdout);
    } else if (c == '\r') {
        /* Skip standalone CR - we handle line endings with LF */
        return;
    } else {
        putchar(c);
    }
}

void platform_printString(const char *str)
{
    if (str != NULL) {
        while (*str != '\0') {
            platform_printChar(*str);
            str++;
        }
    }
}

void platform_timerWaitMs(uint16_t ms)
{
#ifdef BUILD_HOST
    /* Simple host-side delay (firmware doesn't need to wait) */
    usleep(ms * 1000);
#endif
}

void platform_critSecStart(uint8_t resource)
{
    (void)resource;
    /* Empty - No RTOS */
}

void platform_critSecStop(uint8_t resource)
{
    (void)resource;
    /* Empty - No RTOS */
}

void platform_irqResponse(void)
{
    /* Empty - No response */
}

void *platform_getCommHandle0(void)
{
    return (void*)(&commHandle0);
}

void *platform_getCommHandle1(void)
{
    return (void*)(&commHandle1);
}

int32_t platform_txByte(
    const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen)
{
#ifdef BUILD_HOST
    char cmd[SERIAL_MAX_CMD_LEN];
    char response[SERIAL_MAX_RESPONSE_LEN];
    int32_t status;
    int n;
    uint8_t i;

    /* Validate platform is initialized */
    if (!g_platform_initialized) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Parameter validation */
    if ((handle == NULL) || (buffer == NULL)) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (bufLen == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    /* Select I2C address and Tiva port based on page and mode */
    uint8_t slaveAddr = (page == PMIC_PAGE_WDG) ? handle->i2cAddr1 : handle->i2cAddr0;
    uint8_t tivaPort  = ((page == PMIC_PAGE_WDG) && (handle->commMode == PMIC_INTF_I2C_DUAL))
                        ? PLATFORM_I2C_PORT_SECONDARY : PLATFORM_I2C_PORT_MAIN;

    /* Build i2ce command: write only (read_len=0)
     * Format: i2ce <port> <speed> <addr> <read_len> <write_len> <data>...
     * NOTE: write_len must include the register byte, so bufLen + 1
     *       The data bytes start with register address, then buffer contents
     */
    n = snprintf(cmd, sizeof(cmd), "i2ce %u 400000 0x%02X 0 %d 0x%02X",
                 tivaPort, slaveAddr, bufLen + 1, regAddr);

    if (n < 0 || n >= (int)sizeof(cmd)) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    /* Append data bytes */
    for (i = 0; i < bufLen; i++) {
        n += snprintf(&cmd[n], sizeof(cmd) - (size_t)n, " 0x%02X", buffer[i]);
        if (n >= (int)sizeof(cmd)) {
            return PMIC_ST_ERR_INV_PARAM;
        }
    }

    /* Send command to firmware */
    if (getenv("PMIC_TRACE")) { printf("[TX] %s\n", cmd); fflush(stdout); }
    status = serial_send_command(cmd);
    if (status != 0) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Read response: "STATUS: OK\n" or "STATUS: ERROR_CMD - ...\n" */
    memset(response, 0, sizeof(response));
    status = serial_read_response(response, sizeof(response));
    if (status < 0) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Parse response */
    if (strstr(response, "STATUS: OK") != NULL) {
        return PMIC_ST_SUCCESS;
    } else if (strstr(response, "I2C") != NULL || strstr(response, "NACK") != NULL) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    } else {
        return PMIC_ST_ERR_INV_PARAM;
    }
#else
    /* Should not reach here - BUILD_HOST should be defined */
    (void)handle;
    (void)page;
    (void)regAddr;
    (void)buffer;
    (void)bufLen;
    return PMIC_ST_ERR_INV_PARAM;
#endif
}

int32_t platform_rxByte(
    const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr, uint8_t *buffer, uint8_t bufLen)
{
#ifdef BUILD_HOST
    char cmd[SERIAL_MAX_CMD_LEN];
    char response[SERIAL_MAX_RESPONSE_LEN];
    int32_t status;
    char *results;
    char *token;
    uint8_t idx;

    /* Validate platform is initialized */
    if (!g_platform_initialized) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Parameter validation */
    if ((handle == NULL) || (buffer == NULL)) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (bufLen == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    /* Select I2C address and Tiva port based on page and mode */
    uint8_t slaveAddr = (page == PMIC_PAGE_WDG) ? handle->i2cAddr1 : handle->i2cAddr0;
    uint8_t tivaPort  = ((page == PMIC_PAGE_WDG) && (handle->commMode == PMIC_INTF_I2C_DUAL))
                        ? PLATFORM_I2C_PORT_SECONDARY : PLATFORM_I2C_PORT_MAIN;

    /* Build i2ce command: write reg address, then read
     * Format: i2ce <port> <speed> <addr> <read_len> <write_len> <data>...
     *   This writes the register address, then reads bufLen bytes
     */
    snprintf(cmd, sizeof(cmd), "i2ce %u 400000 0x%02X %d 1 0x%02X",
             tivaPort, slaveAddr, bufLen, regAddr);

    /* Send command to firmware */
    if (getenv("PMIC_TRACE")) { printf("[RX] %s\n", cmd); fflush(stdout); }
    status = serial_send_command(cmd);
    if (status != 0) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Read first line: "STATUS: OK\n" or "STATUS: ERROR_CMD - ...\n" */
    memset(response, 0, sizeof(response));
    status = serial_read_response(response, sizeof(response));
    if (status < 0) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Check for errors */
    if (strstr(response, "STATUS: OK") == NULL) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Read second line: "RESULTS: 0xAA 0xBB 0xCC\n" */
    memset(response, 0, sizeof(response));
    status = serial_read_response(response, sizeof(response));
    if (status < 0) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Find "RESULTS:" prefix */
    results = strstr(response, "RESULTS:");
    if (results == NULL) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Parse data: "RESULTS: 0xAA 0xBB 0xCC" or "RESULTS: 170 187 204" */
    results += 9;  /* Skip "RESULTS: " */

    /* Use strtok to split by spaces */
    token = strtok(results, " \n\r");
    idx = 0;

    while (token != NULL && idx < bufLen) {
        /* Convert string to byte (base 0 = auto-detect hex with 0x prefix or decimal) */
        unsigned long val = strtoul(token, NULL, 0);
        buffer[idx] = (uint8_t)val;
        idx++;
        token = strtok(NULL, " \n\r");
    }

    /* Verify we got the expected number of bytes */
    if (idx != bufLen) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return PMIC_ST_SUCCESS;
#else
    /* Should not reach here - BUILD_HOST should be defined */
    (void)handle;
    (void)page;
    (void)regAddr;
    (void)buffer;
    (void)bufLen;
    return PMIC_ST_ERR_INV_PARAM;
#endif
}

void platform_unlockRegisters(void)
{
#ifdef BUILD_HOST
    /* Hardware: Unlock TPS6522x-Q1 configuration registers
     * Write 0x9B to register 0xA1 (REGISTER_LOCK_REG, per include/regmap/core.h)
     */
    const uint8_t REGISTER_LOCK_REG = 0xA1U;
    const uint8_t UNLOCK_KEY = 0x9BU;
    char cmd[SERIAL_MAX_CMD_LEN];
    char response[SERIAL_MAX_RESPONSE_LEN];
    int32_t status;

    PLATFORM_DEBUG(DEBUG_LEVEL_TRACE, ">>> platform_unlockRegisters");

    /* Validate platform is initialized */
    if (!g_platform_initialized) {
        PLATFORM_DEBUG(DEBUG_LEVEL_ERROR, "Platform not initialized");
        fprintf(stderr, "ERROR: Platform not initialized\n");
        return;
    }

    /* Validate I2C address is correct */
    if (commHandle0.slaveAddr != PLATFORM_TARGET_I2C_ADDR) {
        PLATFORM_DEBUG(DEBUG_LEVEL_ERROR, "Invalid I2C address: 0x%02X (expected 0x%02X)",
                       commHandle0.slaveAddr, PLATFORM_TARGET_I2C_ADDR);
        fprintf(stderr, "ERROR: Invalid I2C address 0x%02X (expected 0x%02X)\n",
                commHandle0.slaveAddr, PLATFORM_TARGET_I2C_ADDR);
        return;
    }

    /* Build i2ce command to write unlock key to register lock
     * Format: i2ce 0 400000 0x60 0 2 0x09 0x9B
     *   Write 0x9B to register 0x09, no read
     */
    snprintf(cmd, sizeof(cmd), "i2ce 2 400000 0x%02X 0 2 0x%02X 0x%02X",
             commHandle0.slaveAddr, REGISTER_LOCK_REG, UNLOCK_KEY);

    PLATFORM_DEBUG(DEBUG_LEVEL_DEBUG, "Sending unlock command: %s", cmd);

    /* Send command to firmware */
    status = serial_send_command(cmd);
    if (status != 0) {
        platform_printString("ERROR: Failed to send unlock command\r\n");
        return;
    }

    /* Read response */
    memset(response, 0, sizeof(response));
    status = serial_read_response(response, sizeof(response));

    PLATFORM_DEBUG(DEBUG_LEVEL_DEBUG, "Got response (status=%d): %s", status,
                   status >= 0 ? response : "(no response)");

    if (status < 0) {
        PLATFORM_DEBUG(DEBUG_LEVEL_ERROR, "No response from firmware");
        platform_printString("ERROR: No response from firmware during unlock\r\n");
        return;
    }

    /* Check response */
    if (strstr(response, "STATUS: OK") == NULL) {
        PLATFORM_DEBUG(DEBUG_LEVEL_WARNING, "Unlock failed: %s", response);
        fprintf(stderr, "ERROR: Failed to unlock PMIC registers: %s\n", response);
    } else {
        PLATFORM_DEBUG(DEBUG_LEVEL_TRACE, "<<< platform_unlockRegisters (success)");
    }
#endif
    /* Mock: No-op (mock doesn't enforce register locking) */
}

void platform_setEsmPin(bool high)
{
#ifdef BUILD_HOST
    char gpio_response[256];
    int32_t rc;
    const char *cmd = high ? "gpiow pa 4 4 0" : "gpiow pa 4 0 0";
    rc = serial_send_command(cmd);
    if (rc == 0)
    {
        (void)serial_read_response(gpio_response, sizeof(gpio_response));
    }
    (void)rc;
#else
    (void)high;
#endif
}

void platform_runTestLoop(void (*testCallback)(void))
{
#ifdef BUILD_HOST
    /* Host build: Just run tests once, no interaction */
    testCallback();
#else
    /* Should not reach here for host build */
    testCallback();
#endif
}

/**
 * @brief Unity output character implementation
 *
 * Real function implementation for UNITY_OUTPUT_CHAR to ensure proper
 * calling convention and ABI compatibility. The macro in unity_config.h
 * expands to call this function.
 *
 * @param c Character to output
 */
void unity_output_char_impl(int c)
{
    platform_printChar((char)c);
}
