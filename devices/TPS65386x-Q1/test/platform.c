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
 * @brief Host-controlled platform layer for PMIC testing (SPI interface)
 *
 * This implementation runs tests on the host PC while communicating with
 * pmic-tiva-host firmware on TM4C123 via serial commands. The firmware acts
 * as an SPI/GPIO translation layer, providing unlimited memory and easy
 * debugging while still accessing real PMIC hardware.
 *
 * TPS65386x-Q1 uses SPI interface, not I2C.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "platform.h"
#include "debug.h"
#include <stdbool.h>

#ifdef BUILD_HOST
#include "platform_serial_spi.h"
#include <unistd.h>  /* For usleep */
#include <stdlib.h>  /* For strtoul, getenv, exit */
#endif

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @brief Pseudo SPI handle for host mode
 */
typedef struct {
    uint8_t dummy;  /* SPI doesn't use slave address like I2C */
} SpiHandle_t;

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

/**
 * @brief SPI handle used to communicate to PMIC
 */
static SpiHandle_t commHandle = {0U};

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
        PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Already initialized, flushing serial buffer");
        serial_flush();
        return;
    }

    PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Host-controlled platform initialization (SPI)");

    /* Reset initialization state */
    g_platform_initialized = false;

    /* Get serial port from environment or use default */
    port = getenv("PMIC_SERIAL_PORT");
    if (port == NULL) {
        port = SERIAL_DEFAULT_PORT;
    }

    PLATFORM_DEBUG(DEBUG_LEVEL_DEBUG, "Serial port: %s", port);

    printf("Initializing host-controlled PMIC testing (SPI)...\n");
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

    /* Initialize SPI peripheral on firmware side */
    printf("Initializing SPI peripheral (SSI2)...\n");
    status = platform_serial_init();
    if (status != 0) {
        fprintf(stderr, "ERROR: Failed to initialize SPI peripheral\n");
        fprintf(stderr, "       %s\n", serial_get_last_error());
        serial_close();
        exit(1);
    }

    /* Assert WAKE1 (PB5) high — must be held high for normal device operation.
     * Port B is already enabled by the SPI init (SSI2 uses PB4/PB6/PB7).
     *
     * Command format: gpioc <port> <bitmask_decimal> <dir> <pulltype> <strength>
     *   port "pb" → argv[1][1]='b' → PORT_B (firmware indexes second char)
     *   bitmask 32 = 0x20 = PB5 (decimal required; atoi("0x20")=0, not 32)
     *   dir "o" = output, pulltype 8 = GPIO_PIN_TYPE_STD, strength 1 = GPIO_STRENGTH_2MA
     * Command format: gpiow <port> <bitmask_decimal> <value_decimal> <timer_ms>
     *   value 32 = 0x20 = PB5 high, timer 0 = no auto-restore
     */
    printf("Asserting WAKE1 (PB5) high...\n");
    {
        char gpio_response[256];
        int32_t rc;

        rc = serial_send_command("gpioc pb 32 o 8 1");
        if (rc != 0 ||
                (rc = serial_read_response(gpio_response, sizeof(gpio_response))) < 0 ||
                strstr(gpio_response, "STATUS: OK") == NULL) {
            fprintf(stderr, "ERROR: gpioc pb 32 o 8 1 failed (rc=%d): %s\n", rc, gpio_response);
            serial_close();
            exit(1);
        }

        /* Drive WAKE1 low first to generate a guaranteed rising edge.
         * If the device is in STANDBY from a previous test run it needs
         * a LOW-to-HIGH transition to wake up; simply driving HIGH when
         * the pin is already HIGH provides no edge and the device stays
         * in STANDBY, causing SPI reads to fail. */
        rc = serial_send_command("gpiow pb 32 0 0");
        if (rc != 0 ||
                (rc = serial_read_response(gpio_response, sizeof(gpio_response))) < 0 ||
                strstr(gpio_response, "STATUS: OK") == NULL) {
            fprintf(stderr, "ERROR: gpiow pb 32 0 0 failed (rc=%d): %s\n", rc, gpio_response);
            serial_close();
            exit(1);
        }
        usleep(5000);  /* 5ms LOW hold - ensures clean edge */

        rc = serial_send_command("gpiow pb 32 32 0");
        if (rc != 0 ||
                (rc = serial_read_response(gpio_response, sizeof(gpio_response))) < 0 ||
                strstr(gpio_response, "STATUS: OK") == NULL) {
            fprintf(stderr, "ERROR: gpiow pb 32 32 0 failed (rc=%d): %s\n", rc, gpio_response);
            serial_close();
            exit(1);
        }
    }
    usleep(100000);  /* 100ms for device to wake from STANDBY and stabilize */

    /* Configure PA2 as GPIO output for ESM_IN (PMIC GPI1). Drive HIGH initially
     * to provide a valid "good" level-mode signal when ESM is enabled.
     * Port A is already enabled by firmware (UART0 uses PA0/PA1); PA2 is free.
     *   bitmask 4 = 0x04 = PA2 (decimal required)
     */
    printf("Configuring ESM_IN (PA2) high...\n");
    {
        char gpio_response[256];
        int32_t rc;

        rc = serial_send_command("gpioc pa 4 o 8 1");
        if (rc != 0 ||
                (rc = serial_read_response(gpio_response, sizeof(gpio_response))) < 0 ||
                strstr(gpio_response, "STATUS: OK") == NULL) {
            fprintf(stderr, "ERROR: gpioc pa 4 o 8 1 failed (rc=%d): %s\n", rc, gpio_response);
            serial_close();
            exit(1);
        }

        rc = serial_send_command("gpiow pa 4 4 0");
        if (rc != 0 ||
                (rc = serial_read_response(gpio_response, sizeof(gpio_response))) < 0 ||
                strstr(gpio_response, "STATUS: OK") == NULL) {
            fprintf(stderr, "ERROR: gpiow pa 4 4 0 failed (rc=%d): %s\n", rc, gpio_response);
            serial_close();
            exit(1);
        }
    }

    printf("Host-controlled platform initialized successfully\n\n");

    /* Set initialization state before CRC disable so platform helpers work */
    g_platform_initialized = true;

    /* Unlock registers and disable CFG CRC monitoring on first init. */
    platform_unlockRegisters();

    PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "Initialization complete - SPI mode");
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
}

void platform_wakeFromStandby(void)
{
#ifdef BUILD_HOST
    char gpio_response[256];
    int32_t rc;

    /* Rising edge on WAKE1 (PB5) wakes device from STANDBY.
     * Serial round-trip latency provides sufficient hold time between commands. */
    rc = serial_send_command("gpiow pb 32 0 0");
    if (rc == 0)
    {
        (void)serial_read_response(gpio_response, sizeof(gpio_response));
    }

    rc = serial_send_command("gpiow pb 32 32 0");
    if (rc == 0)
    {
        (void)serial_read_response(gpio_response, sizeof(gpio_response));
    }
    (void)rc;
#endif
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

void platform_setupTests(void)
{
    /* IRQ state is now cleared per-test in setUp() */
}

void platform_tearDownTests(void)
{
#ifdef BUILD_HOST
    platform_resetDevice();
#endif
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

int32_t platform_txByte(
    const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen)
{
#ifdef BUILD_HOST
    int32_t status;

    (void)page;  /* TPS65386x-Q1: Page mapping not used for SPI */

    /* Validate platform is initialized */
    if (!g_platform_initialized) {
        return PMIC_ST_ERR_SPI_COMM_FAIL;
    }

    /* Parameter validation */
    if ((handle == NULL) || (handle->commHandle0 == NULL) || (buffer == NULL)) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (bufLen == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    /* Unlock registers before every write — EXCEPT the unlock registers
     * themselves (0x03 = CFG_REG_UNLOCK_SEQ_REG, 0x04 = CNT_REG_UNLOCK_SEQ_REG).
     * Those registers are always writable and require a precise 2-byte sequence;
     * inserting an unlock sequence between the two bytes corrupts it. */
    if (regAddr != 0x03U && regAddr != 0x04U) {
        platform_unlockRegisters();
    }

    /* Write via SPI (2MHz = 2000 kHz) */
    status = platform_serial_write(buffer, bufLen, 2000);
    if (status != 0) {
        return PMIC_ST_ERR_SPI_COMM_FAIL;
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

int32_t platform_rxByte(
    const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr, uint8_t *buffer, uint8_t bufLen)
{
#ifdef BUILD_HOST
    int32_t status;

    (void)page;  /* TPS65386x-Q1: Page mapping not used for SPI */

    /* Validate platform is initialized */
    if (!g_platform_initialized) {
        return PMIC_ST_ERR_SPI_COMM_FAIL;
    }

    /* Parameter validation */
    if ((handle == NULL) || (handle->commHandle0 == NULL) || (buffer == NULL)) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (bufLen == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    /* Read via SPI (2MHz = 2000 kHz) */
    status = platform_serial_read(regAddr, buffer, bufLen, 2000);
    if (status != 0) {
        return PMIC_ST_ERR_SPI_COMM_FAIL;
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

#ifdef BUILD_HOST
static uint8_t platform_crc8(const uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0xFFU;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            uint8_t msb = crc & 0x80U;
            crc = (uint8_t)((unsigned int)crc << 1U);
            if (msb != 0U) { crc ^= 0x07U; }
        }
    }
    return crc;
}
#endif

void platform_resetDevice(void)
{
#ifdef BUILD_HOST
    if (!g_platform_initialized) {
        return;
    }

    /* Issue PMIC_OFF_REQUEST (5) by writing STATE_REQ bits [2:0] in STATE_CTRL_REG (0x16).
     * Uses raw SPI to avoid needing a pmicHandle (each module owns its own static handle). */
    uint8_t regData = 0U;
    uint8_t frame[4U] = {0x16U, 0x10U, 0x00U, 0x00U};
    frame[3U] = platform_crc8(frame, 3U);
    if (platform_serial_read(0U, frame, 4U, 2000U) == 0) {
        regData = frame[2U];
    }
    regData = (regData & ~(uint8_t)0x07U) | (uint8_t)0x05U;  /* STATE_REQ = PMIC_OFF_REQUEST */
    uint8_t wframe[4U] = {0x16U, 0x00U, regData, 0x00U};
    wframe[3U] = platform_crc8(wframe, 3U);
    (void)platform_serial_write(wframe, 4U, 2000U);

    /* Wait for device to complete power-down sequence */
    usleep(100000U);  /* 100ms */

    /* Wake device: drive WAKE1 LOW→HIGH to start PWRU_SEQ → ACTIVE */
    platform_wakeFromStandby();

    /* Wait for PWRU_SEQ → ACTIVE to stabilize */
    usleep(100000U);  /* 100ms */

    /* Re-unlock registers (reset clears unlock state) */
    platform_unlockRegisters();
#endif
}

void platform_irqClrAll(void)
{
#ifdef BUILD_HOST
    static const uint8_t irq_stat_regs[] = {
        0x09U, 0x0BU, 0x0FU, 0x10U, 0x46U, 0x51U,
        0x5AU, 0x5CU, 0x5EU, 0x5FU, 0x62U, 0x65U,
        0x66U, 0x67U, 0x77U, 0x78U
    };

    if (!g_platform_initialized) {
        return;
    }

    /* Clear OFF_STATE_STAT1 (0x0F) and OFF_STATE_STAT2 (0x10) via OFF_STATE_CLR_REG (0x11) */
    {
        uint8_t frame[4U] = {0x11U, 0x00U, 0x01U, 0x00U};
        frame[3U] = platform_crc8(frame, 3U);
        (void)platform_serial_write(frame, 4U, 2000U);
    }

    /* Clear DEV_ERR_STAT (0x66): set bit 7 only, preserve DEV_ERR_CNT bits 0-4 */
    {
        uint8_t rframe[4U] = {0x66U, 0x10U, 0x00U, 0x00U};
        rframe[3U] = platform_crc8(rframe, 3U);
        if (platform_serial_read(0U, rframe, 4U, 2000U) == 0) {
            uint8_t val = (rframe[2U] & 0x1FU) | 0x80U;
            uint8_t wframe[4U] = {0x66U, 0x00U, val, 0x00U};
            wframe[3U] = platform_crc8(wframe, 3U);
            (void)platform_serial_write(wframe, 4U, 2000U);
        }
    }

    /* W1C clear all other status registers; check 0x67 after each write to
     * identify which register address triggers ADDR_ERR (bit 2 of 0x67). */
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(irq_stat_regs) / sizeof(irq_stat_regs[0])); i++) {
        uint8_t reg = irq_stat_regs[i];
        if (reg == 0x0FU || reg == 0x10U || reg == 0x66U) {
            continue;
        }
        uint8_t frame[4U] = {reg, 0x00U, 0xFFU, 0x00U};
        frame[3U] = platform_crc8(frame, 3U);
        (void)platform_serial_write(frame, 4U, 2000U);

    }
#endif
}

static void platform_dumpAllRegisters(void)
{
    static const uint8_t reg_addrs[] = {
        0x00U, 0x01U, 0x02U, 0x07U, 0x09U, 0x0AU, 0x0BU, 0x0CU, 0x0DU, 0x0EU,
        0x0FU, 0x10U, 0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U, 0x18U,
        0x19U, 0x1AU, 0x1BU, 0x1EU, 0x1FU, 0x20U, 0x21U, 0x22U, 0x23U, 0x24U,
        0x25U, 0x26U, 0x27U, 0x28U, 0x29U, 0x2AU, 0x2BU, 0x2CU, 0x2FU, 0x30U,
        0x31U, 0x32U, 0x33U, 0x34U, 0x35U, 0x36U, 0x37U, 0x38U, 0x39U, 0x3AU,
        0x3BU, 0x3CU, 0x3DU, 0x3EU, 0x3FU, 0x40U, 0x41U, 0x42U, 0x43U, 0x44U,
        0x45U, 0x46U, 0x47U, 0x48U, 0x49U, 0x4AU, 0x4BU, 0x4CU, 0x4DU, 0x4EU,
        0x4FU, 0x50U, 0x51U, 0x52U, 0x54U, 0x55U, 0x56U, 0x57U, 0x58U, 0x59U,
        0x5AU, 0x5BU, 0x5CU, 0x5EU, 0x5FU, 0x60U, 0x61U, 0x62U, 0x63U, 0x64U,
        0x65U, 0x66U, 0x67U, 0x68U, 0x69U, 0x6AU, 0x6BU, 0x6CU, 0x6DU, 0x6EU,
        0x6FU, 0x70U, 0x77U, 0x78U, 0x79U, 0x7AU, 0x7BU, 0x7CU, 0x7DU, 0x7EU,
        0x7FU, 0x80U, 0x81U, 0x82U
    };
    const uint32_t count = sizeof(reg_addrs) / sizeof(reg_addrs[0]);

    printf("=== REGISTER DUMP (pre-test) ===\n");
    for (uint32_t i = 0U; i < count; i++) {
        uint8_t frame[4U] = {reg_addrs[i], 0x10U, 0x00U, 0x00U};
        frame[3U] = platform_crc8(frame, 3U);
        if (platform_serial_read(0U, frame, 4U, 2000U) == 0) {
            printf("0x%02X: 0x%02X\n", reg_addrs[i], frame[2U]);
        } else {
            printf("0x%02X: ERR\n", reg_addrs[i]);
        }
    }
    printf("================================\n");
}

void platform_unlockRegisters(void)
{
#ifdef BUILD_HOST
    /* Unlock CFG registers: write 0x98 then 0xB8 to reg 0x03 (CFG_REG_UNLOCK_SEQ_REG).
     * Unlock CNT registers: write 0x13 then 0x7D to reg 0x04 (CNT_REG_UNLOCK_SEQ_REG).
     * Frame: [addr, ctrl, data, crc8]. CRC8 poly=0x07 init=0xFF.
     * Uses platform_serial_write (not raw serial_send_command) because TIVA always
     * sends STATUS: OK + RESULTS — platform_serial_write drains both lines. */
    /* CFG window remains open until either a state transition (ACTIVE/SAFE/OFF)
     * or a write to reg 0x03 with a value other than the unlock sequence. There
     * is no one-write limit; all CFG-bank writes succeed until the window closes.
     * NOTE: Whether reg 0x08 (SAFETY_CTRL) is in the CFG bank is unconfirmed —
     * the Step 2 write below may be silently rejected. Pending logic analyzer
     * verification. */

    static const uint8_t cnt_unlock[2U][4U] = {
        {0x04U, 0x00U, 0x13U, 0xF9U},  /* CNT seq byte 1 */
        {0x04U, 0x00U, 0x7DU, 0xF4U},  /* CNT seq byte 2 */
    };
    static const uint8_t cfg_unlock[2U][4U] = {
        {0x03U, 0x00U, 0x98U, 0x57U},  /* CFG seq byte 1 */
        {0x03U, 0x00U, 0xB8U, 0xB7U},  /* CFG seq byte 2 */
    };

    if (!g_platform_initialized) {
        return;
    }

    /* Step 1: Unlock CNT registers.
     * Force to locked state first — writing any non-unlock byte re-locks the bank.
     * This makes the sequence idempotent when registers are already unlocked. */
    {
        uint8_t frame[4U] = {0x04U, 0x00U, 0x00U, 0x00U};
        frame[3U] = platform_crc8(frame, 3U);
        (void)platform_serial_write(frame, 4U, 2000U);
    }
    for (uint8_t i = 0U; i < 2U; i++) {
        (void)platform_serial_write(cnt_unlock[i], 4U, 2000U);
    }

    /* Step 2: Re-disable CFG CRC monitoring before opening the CFG write window.
     * NOTE: CFG-bank membership of reg 0x08 (SAFETY_CTRL) is unconfirmed — if it
     * is CFG-protected this write is silently rejected until the unlock in Step 4. */
    {
        uint8_t frame[4U] = {0x08U, 0x10U, 0x00U, 0x00U};
        frame[3U] = platform_crc8(frame, 3U);
        if (platform_serial_read(0U, frame, 4U, 2000U) == 0)
        {
            uint8_t safetyCtrl = frame[2U] & ~(uint8_t)0x01U;
            uint8_t wframe[4U] = {0x08U, 0x00U, safetyCtrl, 0x00U};
            wframe[3U] = platform_crc8(wframe, 3U);
            (void)platform_serial_write(wframe, 4U, 2000U);
        }
    }

    /* Step 3: Clear the CFG CRC error sticky flag (REG_STAT reg 0x09, bit 0, W1C).
     * Must come before the CFG unlock (same reason as Step 2). */
    {
        uint8_t frame[4U] = {0x09U, 0x00U, 0x01U, 0x00U};
        frame[3U] = platform_crc8(frame, 3U);
        (void)platform_serial_write(frame, 4U, 2000U);
    }

    /* Step 4: Unlock CFG registers. Window remains open until next state
     * transition or a non-unlock write to reg 0x03.
     * Force to locked state first for the same reason as Step 1. */
    {
        uint8_t frame[4U] = {0x03U, 0x00U, 0x00U, 0x00U};
        frame[3U] = platform_crc8(frame, 3U);
        (void)platform_serial_write(frame, 4U, 2000U);
    }
    for (uint8_t i = 0U; i < 2U; i++) {
        (void)platform_serial_write(cfg_unlock[i], 4U, 2000U);
    }
#endif
    /* Mock: No-op (mock doesn't enforce register locking) */
}

void platform_checkDevState(const char *testName)
{
#ifdef BUILD_HOST
    static uint8_t prev_state = 0xFFU;

    if (!g_platform_initialized) {
        return;
    }

    uint8_t frame[4U] = {0x17U, 0x10U, 0x00U, 0x00U};
    frame[3U] = platform_crc8(frame, 3U);
    if (platform_serial_read(0U, frame, 4U, 2000U) != 0) {
        return;
    }
    uint8_t state = frame[2U] & 0x0FU;

    if (state != prev_state) {
        PLATFORM_DEBUG(DEBUG_LEVEL_INFO, "[STATE] 0x%X -> 0x%X  after: %s", prev_state, state, testName);
        if (state == 0x9U) {
            PLATFORM_DEBUG(DEBUG_LEVEL_WARNING, "[STATE] *** SAFE STATE ENTERED — nRST asserted ***");
        }
        prev_state = state;
    }
#endif
}

void platform_runTestLoop(void (*testCallback)(void))
{
#ifdef BUILD_HOST
    platform_init();
#ifdef PMIC_DUMP_REGISTERS
    platform_dumpAllRegisters();
#endif
    testCallback();
#else
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
