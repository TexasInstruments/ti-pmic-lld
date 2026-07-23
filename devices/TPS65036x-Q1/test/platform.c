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

/**
 * @brief I2C handle used to communicate to PMIC (stores slave address)
 */
static I2cHandle_t commHandle = {0U};

/**
 * @brief Current module name for test result prefixes
 */
const char* g_currentModuleName = NULL;

/**
 * @brief Platform initialization state
 */
static bool g_platform_initialized = false;

/**
 * @brief Mirrors I2C_CRC_EN in INTERFACE_CONF (0x24); updated in platform_txByte
 * when that register is written, and read from hardware during platform_init.
 */
static bool g_pmic_crc_enabled = true;

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

#ifdef BUILD_HOST
void platform_writeReg(uint8_t regAddr, uint8_t data);
#endif

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

    /* Only set address and state after all validation passes */
    commHandle.slaveAddr = PLATFORM_TARGET_I2C_ADDR;
    g_platform_initialized = true;

    /* Read INTERFACE_CONF (0x24) to detect initial CRC state; reads need no CRC byte. */
    {
        uint8_t ifaceConf = 0U;
        Pmic_Handle_t tmpH = {0};
        tmpH.commHandle0 = (void*)&commHandle;
        if (platform_rxByte(&tmpH, 0U, 0x24U, &ifaceConf, 1U) == PMIC_ST_SUCCESS) {
            g_pmic_crc_enabled = (ifaceConf & 0x80U) != 0U;
        }
    }

    platform_unlockRegisters();

    /* Clear both counters at startup — previous runs may have left them elevated */
    platform_writeReg(0x07U, 0x03U);

    /* Flush any extra response lines the firmware may have sent for the above
     * register writes (e.g. "WRITTEN: N bytes" trailing a "STATUS: OK").
     * platform_txByte only consumes one line per command; leaving stale lines
     * in the buffer would corrupt the first Pmic_init I2C read. */
    serial_flush();


    PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Initialization complete - I2C addr: 0x%02X", PLATFORM_TARGET_I2C_ADDR);
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
    commHandle.slaveAddr = 0x00;
}

void platform_setupTests(void)
{
    /* No setup needed for host mode */
}

void platform_tearDownTests(void)
{
    /* No teardown needed for host mode */
}

#ifdef BUILD_HOST
static void platform_waitNRSTOUT(void)
{
    char response[256];
    uint32_t elapsed_ms;
    bool ready = false;

    usleep((useconds_t)PLATFORM_REBOOT_INITIAL_WAIT_MS * 1000U);
    elapsed_ms = PLATFORM_REBOOT_INITIAL_WAIT_MS;

    while (elapsed_ms < PLATFORM_REBOOT_TIMEOUT_MS) {
        if (serial_send_command("gpior pb 32") != 0) break;

        if (serial_read_response(response, sizeof(response)) < 0) break;
        if (strstr(response, "STATUS: OK") == NULL) break;

        if (serial_read_response(response, sizeof(response)) < 0) break;

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
    platform_unlockRegisters();
    platform_writeReg(0x05U, 0x55U); /* COLD_BOOT_REQUEST — full NVM reload */
    printf("[SOFT REBOOT] triggered\n");
    platform_waitNRSTOUT();
    g_pmic_crc_enabled = true; /* NVM restored after cold boot */
    platform_unlockRegisters();
    platform_writeReg(0x07U, 0x03U); /* clear RESET_CNT and RECOV_CNT */
#endif
    /* Mock: no-op */
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

void *platform_getCommHandle(void)
{
    return (void*)(&commHandle);
}

#ifdef BUILD_HOST
/* CRC-8 lookup table: polynomial 0x07, init 0xFF. Identical to pmic_io.c CRC8_TABLE. */
static const uint8_t PMIC_CRC8_TABLE[] = {
    0x00U, 0x07U, 0x0eU, 0x09U, 0x1cU, 0x1bU, 0x12U, 0x15U,
    0x38U, 0x3fU, 0x36U, 0x31U, 0x24U, 0x23U, 0x2aU, 0x2dU,
    0x70U, 0x77U, 0x7eU, 0x79U, 0x6cU, 0x6bU, 0x62U, 0x65U,
    0x48U, 0x4fU, 0x46U, 0x41U, 0x54U, 0x53U, 0x5aU, 0x5dU,
    0xe0U, 0xe7U, 0xeeU, 0xe9U, 0xfcU, 0xfbU, 0xf2U, 0xf5U,
    0xd8U, 0xdfU, 0xd6U, 0xd1U, 0xc4U, 0xc3U, 0xcaU, 0xcdU,
    0x90U, 0x97U, 0x9eU, 0x99U, 0x8cU, 0x8bU, 0x82U, 0x85U,
    0xa8U, 0xafU, 0xa6U, 0xa1U, 0xb4U, 0xb3U, 0xbaU, 0xbdU,
    0xc7U, 0xc0U, 0xc9U, 0xceU, 0xdbU, 0xdcU, 0xd5U, 0xd2U,
    0xffU, 0xf8U, 0xf1U, 0xf6U, 0xe3U, 0xe4U, 0xedU, 0xeaU,
    0xb7U, 0xb0U, 0xb9U, 0xbeU, 0xabU, 0xacU, 0xa5U, 0xa2U,
    0x8fU, 0x88U, 0x81U, 0x86U, 0x93U, 0x94U, 0x9dU, 0x9aU,
    0x27U, 0x20U, 0x29U, 0x2eU, 0x3bU, 0x3cU, 0x35U, 0x32U,
    0x1fU, 0x18U, 0x11U, 0x16U, 0x03U, 0x04U, 0x0dU, 0x0aU,
    0x57U, 0x50U, 0x59U, 0x5eU, 0x4bU, 0x4cU, 0x45U, 0x42U,
    0x6fU, 0x68U, 0x61U, 0x66U, 0x73U, 0x74U, 0x7dU, 0x7aU,
    0x89U, 0x8eU, 0x87U, 0x80U, 0x95U, 0x92U, 0x9bU, 0x9cU,
    0xb1U, 0xb6U, 0xbfU, 0xb8U, 0xadU, 0xaaU, 0xa3U, 0xa4U,
    0xf9U, 0xfeU, 0xf7U, 0xf0U, 0xe5U, 0xe2U, 0xebU, 0xecU,
    0xc1U, 0xc6U, 0xcfU, 0xc8U, 0xddU, 0xdaU, 0xd3U, 0xd4U,
    0x69U, 0x6eU, 0x67U, 0x60U, 0x75U, 0x72U, 0x7bU, 0x7cU,
    0x51U, 0x56U, 0x5fU, 0x58U, 0x4dU, 0x4aU, 0x43U, 0x44U,
    0x19U, 0x1eU, 0x17U, 0x10U, 0x05U, 0x02U, 0x0bU, 0x0cU,
    0x21U, 0x26U, 0x2fU, 0x28U, 0x3dU, 0x3aU, 0x33U, 0x34U,
    0x4eU, 0x49U, 0x40U, 0x47U, 0x52U, 0x55U, 0x5cU, 0x5bU,
    0x76U, 0x71U, 0x78U, 0x7fU, 0x6aU, 0x6dU, 0x64U, 0x63U,
    0x3eU, 0x39U, 0x30U, 0x37U, 0x22U, 0x25U, 0x2cU, 0x2bU,
    0x06U, 0x01U, 0x08U, 0x0fU, 0x1aU, 0x1dU, 0x14U, 0x13U,
    0xaeU, 0xa9U, 0xa0U, 0xa7U, 0xb2U, 0xb5U, 0xbcU, 0xbbU,
    0x96U, 0x91U, 0x98U, 0x9fU, 0x8aU, 0x8dU, 0x84U, 0x83U,
    0xdeU, 0xd9U, 0xd0U, 0xd7U, 0xc2U, 0xc5U, 0xccU, 0xcbU,
    0xe6U, 0xe1U, 0xe8U, 0xefU, 0xfaU, 0xfdU, 0xf4U, 0xf3U
};

/* CRC-aware write for platform-internal register writes (bypasses Pmic_ioTxByte). */
void platform_writeReg(uint8_t regAddr, uint8_t data)
{
    uint8_t buf[2U];
    Pmic_Handle_t h = {0};

    h.commHandle0 = (void*)&commHandle;

    if (g_pmic_crc_enabled) {
        uint8_t crc = 0xFFU;
        crc = PMIC_CRC8_TABLE[(uint8_t)((uint8_t)(PLATFORM_TARGET_I2C_ADDR << 1U) ^ crc)];
        crc = PMIC_CRC8_TABLE[regAddr ^ crc];
        crc = PMIC_CRC8_TABLE[data ^ crc];
        buf[0U] = data;
        buf[1U] = crc;
        (void)platform_txByte(&h, 0U, regAddr, buf, 2U);
    } else {
        buf[0U] = data;
        (void)platform_txByte(&h, 0U, regAddr, buf, 1U);
    }
}
#endif

int32_t platform_txByte(
    const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen)
{
#ifdef BUILD_HOST
    char cmd[SERIAL_MAX_CMD_LEN];
    char response[SERIAL_MAX_RESPONSE_LEN];
    int32_t status;
    int n;
    uint8_t i;

    (void)page;  /* LP8772x-Q1: Page mapping not used in current implementation */

    /* Validate platform is initialized */
    if (!g_platform_initialized) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Parameter validation */
    if ((handle == NULL) || (handle->commHandle0 == NULL) || (buffer == NULL)) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (bufLen == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    /* Get slave address from handle */
    I2cHandle_t *i2cHandle = (I2cHandle_t*)(handle->commHandle0);

    /* Build i2ce command: write only (read_len=0)
     * Format: i2ce <port> <speed> <addr> <read_len> <write_len> <data>...
     * Example: i2ce 0 400000 0x60 0 2 0x10 0xAA
     * NOTE: write_len must include the register byte, so bufLen + 1
     *       The data bytes start with register address, then buffer contents
     */
    n = snprintf(cmd, sizeof(cmd), "i2ce 2 400000 0x%02X 0 %d 0x%02X",
                 i2cHandle->slaveAddr, bufLen + 1, regAddr);

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

    /* Intercept INTERFACE_CONF writes to track I2C_CRC_EN; buffer[0] is always the data byte. */
    if (regAddr == 0x24U) {
        g_pmic_crc_enabled = (buffer[0] & 0x80U) != 0U;
    }

    /* Send command to firmware */
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

    (void)page;  /* LP8772x-Q1: Page mapping not used in current implementation */

    /* Validate platform is initialized */
    if (!g_platform_initialized) {
        return PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    /* Parameter validation */
    if ((handle == NULL) || (handle->commHandle0 == NULL) || (buffer == NULL)) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (bufLen == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    /* Get slave address from handle */
    I2cHandle_t *i2cHandle = (I2cHandle_t*)(handle->commHandle0);

    /* Build i2ce command: write reg address, then read
     * Format: i2ce <port> <speed> <addr> <read_len> <write_len> <data>...
     * Example: i2ce 0 400000 0x60 2 1 0x10
     *   This writes 0x10 (register address), then reads 2 bytes
     */
    snprintf(cmd, sizeof(cmd), "i2ce 2 400000 0x%02X %d 1 0x%02X",
             i2cHandle->slaveAddr, bufLen, regAddr);

    /* Send command to firmware */
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
    /* Hardware: Unlock LP8772x-Q1 configuration registers
     * Write 0x9B to register 0x09 (REGISTER_LOCK)
     */
    if (!g_platform_initialized) {
        return;
    }

    platform_writeReg(0x09U, 0x9BU);
    /* Disable CONFIG_CRC-16 monitor. PMIC_CONFIG_CRC_CONFIG_REG = 0x64 on B0/B1
     * silicon (no address offset). Writing 0x00 clears CONFIG_CRC_EN. */
    platform_writeReg(0x64U, 0x00U);
#endif
    /* Mock: No-op (mock doesn't enforce register locking) */
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
