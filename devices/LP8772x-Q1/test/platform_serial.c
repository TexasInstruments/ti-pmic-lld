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
 * @file platform_serial.c
 * @brief Serial communication implementation for host-controlled PMIC testing
 */

#include "platform_serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/* Platform-specific headers */
#ifndef _WIN32
    #include <termios.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/select.h>
#else
    #include <windows.h>
#endif

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @brief Serial communication handle
 */
typedef struct {
#ifndef _WIN32
    int fd;                                     /* File descriptor (POSIX) */
    struct termios orig_termios;                /* Original terminal settings */
#else
    HANDLE hSerial;                             /* Serial handle (Windows) */
    DCB dcbOrig;                                /* Original DCB settings */
#endif
    uint32_t timeout_ms;                        /* Read timeout in milliseconds */
    char error_msg[256];                        /* Last error message */
    int initialized;                            /* Initialization flag */
} SerialHandle_t;

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static SerialHandle_t g_serial = {
#ifndef _WIN32
    .fd = -1,
#else
    .hSerial = INVALID_HANDLE_VALUE,
#endif
    .timeout_ms = SERIAL_DEFAULT_TIMEOUT_MS,
    .error_msg = {0},
    .initialized = 0
};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

#ifndef _WIN32
/* POSIX implementation */

int32_t serial_init(const char *port, uint32_t baud)
{
    struct termios tty;
    speed_t baud_rate;

    /* Use environment variable or default if port not specified */
    if (port == NULL) {
        port = getenv("PMIC_SERIAL_PORT");
        if (port == NULL) {
            port = SERIAL_DEFAULT_PORT;
        }
    }

    /* Validate baud rate and convert to termios constant */
    switch (baud) {
        case 9600:   baud_rate = B9600;   break;
        case 19200:  baud_rate = B19200;  break;
        case 38400:  baud_rate = B38400;  break;
        case 57600:  baud_rate = B57600;  break;
        case 115200: baud_rate = B115200; break;
        default:
            snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                     "Unsupported baud rate: %u", baud);
            return -3;
    }

    /* Open serial port */
    g_serial.fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_serial.fd < 0) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to open %s: %s", port, strerror(errno));
        return -1;
    }

    /* Save original terminal settings */
    if (tcgetattr(g_serial.fd, &g_serial.orig_termios) != 0) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to get terminal attributes: %s", strerror(errno));
        close(g_serial.fd);
        g_serial.fd = -1;
        return -2;
    }

    /* Configure terminal for raw mode */
    memset(&tty, 0, sizeof(tty));

    /* Control modes */
    tty.c_cflag = CS8 | CREAD | CLOCAL;  /* 8 bits, enable receiver, ignore modem lines */

    /* Input modes */
    tty.c_iflag = IGNPAR;                /* Ignore parity errors */

    /* Output modes */
    tty.c_oflag = 0;                     /* Raw output */

    /* Local modes */
    tty.c_lflag = 0;                     /* Non-canonical, no echo */

    /* Control characters */
    tty.c_cc[VMIN] = 0;                  /* Non-blocking read */
    tty.c_cc[VTIME] = 0;                 /* No timeout (we handle it ourselves) */

    /* Set baud rate */
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    /* Apply settings */
    if (tcsetattr(g_serial.fd, TCSANOW, &tty) != 0) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to set terminal attributes: %s", strerror(errno));
        close(g_serial.fd);
        g_serial.fd = -1;
        return -2;
    }

    /* Flush any stale data */
    tcflush(g_serial.fd, TCIOFLUSH);

    g_serial.initialized = 1;
    snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
             "Serial port %s opened successfully at %u baud", port, baud);

    return 0;
}

int32_t serial_send_command(const char *cmd)
{
    ssize_t bytes_written;
    size_t cmd_len;

    if (!g_serial.initialized || g_serial.fd < 0) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Serial port not initialized");
        return -1;
    }

    if (cmd == NULL) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Invalid command (NULL)");
        return -3;
    }

    cmd_len = strlen(cmd);
    if (cmd_len == 0 || cmd_len >= SERIAL_MAX_CMD_LEN) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Invalid command length: %zu", cmd_len);
        return -3;
    }

    /* Write command */
    bytes_written = write(g_serial.fd, cmd, cmd_len);
    if (bytes_written != (ssize_t)cmd_len) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to write command: %s", strerror(errno));
        return -2;
    }

    /* Write DOS newline (CR+LF) for pmic-tiva-host firmware */
    bytes_written = write(g_serial.fd, "\r\n", 2);
    if (bytes_written != 2) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to write newline: %s", strerror(errno));
        return -2;
    }

    return 0;
}

int32_t serial_read_response(char *response, size_t max_len)
{
    fd_set read_fds;
    struct timeval timeout;
    size_t pos = 0;
    int result;

    if (!g_serial.initialized || g_serial.fd < 0) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Serial port not initialized");
        return -1;
    }

    if (response == NULL || max_len == 0) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Invalid response buffer");
        return -3;
    }

    /* Read until newline or timeout */
    while (pos < max_len - 1) {
        /* Setup select for timeout */
        FD_ZERO(&read_fds);
        FD_SET(g_serial.fd, &read_fds);

        timeout.tv_sec = g_serial.timeout_ms / 1000;
        timeout.tv_usec = (g_serial.timeout_ms % 1000) * 1000;

        result = select(g_serial.fd + 1, &read_fds, NULL, NULL, &timeout);

        if (result < 0) {
            snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                     "Select failed: %s", strerror(errno));
            response[pos] = '\0';
            return -2;
        } else if (result == 0) {
            /* Timeout */
            snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                     "Timeout waiting for response");
            response[pos] = '\0';
            return -2;
        }

        /* Data available, read one byte */
        ssize_t n = read(g_serial.fd, &response[pos], 1);

        if (n < 0) {
            snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                     "Read failed: %s", strerror(errno));
            response[pos] = '\0';
            return -2;
        } else if (n == 0) {
            continue;  /* No data yet, try again */
        }

        /* Check for newline (end of response) */
        if (response[pos] == '\n' || response[pos] == '\r') {
            char line_end = response[pos];
            response[pos] = '\0';

            /* If we got CR, consume the following LF (CRLF line ending) */
            if (line_end == '\r') {
                /* Use select with short timeout to check for LF */
                FD_ZERO(&read_fds);
                FD_SET(g_serial.fd, &read_fds);
                timeout.tv_sec = 0;
                timeout.tv_usec = 10000;  /* 10ms timeout for LF */

                if (select(g_serial.fd + 1, &read_fds, NULL, NULL, &timeout) > 0) {
                    char lf;
                    ssize_t n = read(g_serial.fd, &lf, 1);
                    /* Consume the LF if present (we don't check, just consume) */
                    (void)n;  /* Suppress unused variable warning */
                }
            }
            /* If we got LF first, line ending is complete */

            return (int32_t)pos;
        }

        pos++;
    }

    /* Buffer full without finding newline */
    response[max_len - 1] = '\0';
    snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
             "Response buffer too small");
    return -4;
}

int32_t serial_set_timeout(uint32_t timeout_ms)
{
    if (!g_serial.initialized) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Serial port not initialized");
        return -1;
    }

    g_serial.timeout_ms = timeout_ms;
    return 0;
}

int32_t serial_flush(void)
{
    if (!g_serial.initialized || g_serial.fd < 0) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Serial port not initialized");
        return -1;
    }

    tcflush(g_serial.fd, TCIFLUSH);
    return 0;
}

void serial_close(void)
{
    if (g_serial.initialized && g_serial.fd >= 0) {
        /* Restore original terminal settings */
        tcsetattr(g_serial.fd, TCSANOW, &g_serial.orig_termios);

        close(g_serial.fd);
        g_serial.fd = -1;
    }

    g_serial.initialized = 0;
}

#else
/* Windows implementation */

int32_t serial_init(const char *port, uint32_t baud)
{
    DCB dcb;
    COMMTIMEOUTS timeouts;

    /* Use environment variable or default if port not specified */
    if (port == NULL) {
        port = getenv("PMIC_SERIAL_PORT");
        if (port == NULL) {
            port = "COM3";  /* Windows default */
        }
    }

    /* On Windows, COM ports above COM9 require the \\.\COMxx path format.
     * Prepend the prefix if not already present (safe for all COM ports). */
    char port_path[32];
    if (strncmp(port, "\\\\.\\", 4) != 0) {
        snprintf(port_path, sizeof(port_path), "\\\\.\\%s", port);
        port = port_path;
    }

    /* Open serial port */
    g_serial.hSerial = CreateFileA(port,
                                   GENERIC_READ | GENERIC_WRITE,
                                   0,
                                   NULL,
                                   OPEN_EXISTING,
                                   FILE_ATTRIBUTE_NORMAL,
                                   NULL);

    if (g_serial.hSerial == INVALID_HANDLE_VALUE) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to open %s: Error %lu", port, GetLastError());
        return -1;
    }

    /* Save original DCB */
    if (!GetCommState(g_serial.hSerial, &g_serial.dcbOrig)) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to get comm state: Error %lu", GetLastError());
        CloseHandle(g_serial.hSerial);
        g_serial.hSerial = INVALID_HANDLE_VALUE;
        return -2;
    }

    /* Configure DCB */
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(g_serial.hSerial, &dcb)) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to set comm state: Error %lu", GetLastError());
        CloseHandle(g_serial.hSerial);
        g_serial.hSerial = INVALID_HANDLE_VALUE;
        return -2;
    }

    /* Configure timeouts */
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = g_serial.timeout_ms;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 1000;

    if (!SetCommTimeouts(g_serial.hSerial, &timeouts)) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to set timeouts: Error %lu", GetLastError());
        CloseHandle(g_serial.hSerial);
        g_serial.hSerial = INVALID_HANDLE_VALUE;
        return -2;
    }

    /* Flush any stale data */
    PurgeComm(g_serial.hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

    g_serial.initialized = 1;
    snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
             "Serial port %s opened successfully at %u baud", port, baud);

    return 0;
}

int32_t serial_send_command(const char *cmd)
{
    DWORD bytes_written;
    DWORD cmd_len;

    if (!g_serial.initialized || g_serial.hSerial == INVALID_HANDLE_VALUE) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Serial port not initialized");
        return -1;
    }

    if (cmd == NULL) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Invalid command (NULL)");
        return -3;
    }

    cmd_len = (DWORD)strlen(cmd);
    if (cmd_len == 0 || cmd_len >= SERIAL_MAX_CMD_LEN) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Invalid command length: %lu", cmd_len);
        return -3;
    }

    /* Write command */
    if (!WriteFile(g_serial.hSerial, cmd, cmd_len, &bytes_written, NULL)) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to write command: Error %lu", GetLastError());
        return -2;
    }

    /* Write CR+LF — firmware triggers on '\r' (VCOMPeek/UARTPeek check for '\r') */
    if (!WriteFile(g_serial.hSerial, "\r\n", 2, &bytes_written, NULL)) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to write newline: Error %lu", GetLastError());
        return -2;
    }

    return 0;
}

int32_t serial_read_response(char *response, size_t max_len)
{
    DWORD bytes_read;
    size_t pos = 0;

    if (!g_serial.initialized || g_serial.hSerial == INVALID_HANDLE_VALUE) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Serial port not initialized");
        return -1;
    }

    if (response == NULL || max_len == 0) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Invalid response buffer");
        return -3;
    }

    /* Read until newline or timeout */
    while (pos < max_len - 1) {
        if (!ReadFile(g_serial.hSerial, &response[pos], 1, &bytes_read, NULL)) {
            snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                     "Read failed: Error %lu", GetLastError());
            response[pos] = '\0';
            return -2;
        }

        if (bytes_read == 0) {
            /* Timeout */
            snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                     "Timeout waiting for response");
            response[pos] = '\0';
            return -2;
        }

        /* Check for newline (end of response) */
        if (response[pos] == '\n' || response[pos] == '\r') {
            char line_end = response[pos];
            response[pos] = '\0';

            /* If we got CR, consume the following LF (CRLF line ending) */
            if (line_end == '\r') {
                /* Read the following LF with a short timeout */
                COMMTIMEOUTS timeouts;
                GetCommTimeouts(g_serial.hSerial, &timeouts);

                /* Temporarily set short timeout for LF */
                COMMTIMEOUTS temp_timeouts = timeouts;
                temp_timeouts.ReadTotalTimeoutConstant = 10;  /* 10ms timeout */
                SetCommTimeouts(g_serial.hSerial, &temp_timeouts);

                /* Try to read the LF */
                char lf;
                DWORD lf_read;
                ReadFile(g_serial.hSerial, &lf, 1, &lf_read, NULL);
                /* Consume the LF if present (we don't check, just consume) */

                /* Restore original timeout */
                SetCommTimeouts(g_serial.hSerial, &timeouts);
            }
            /* If we got LF first, line ending is complete */

            return (int32_t)pos;
        }

        pos++;
    }

    /* Buffer full without finding newline */
    response[max_len - 1] = '\0';
    snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
             "Response buffer too small");
    return -4;
}

int32_t serial_set_timeout(uint32_t timeout_ms)
{
    COMMTIMEOUTS timeouts;

    if (!g_serial.initialized || g_serial.hSerial == INVALID_HANDLE_VALUE) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Serial port not initialized");
        return -1;
    }

    g_serial.timeout_ms = timeout_ms;

    /* Update Windows timeout settings */
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = timeout_ms;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 1000;

    if (!SetCommTimeouts(g_serial.hSerial, &timeouts)) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Failed to set timeouts: Error %lu", GetLastError());
        return -2;
    }

    return 0;
}

int32_t serial_flush(void)
{
    if (!g_serial.initialized || g_serial.hSerial == INVALID_HANDLE_VALUE) {
        snprintf(g_serial.error_msg, sizeof(g_serial.error_msg),
                 "Serial port not initialized");
        return -1;
    }

    PurgeComm(g_serial.hSerial, PURGE_RXCLEAR);
    return 0;
}

void serial_close(void)
{
    if (g_serial.initialized && g_serial.hSerial != INVALID_HANDLE_VALUE) {
        /* Restore original DCB (optional, but clean) */
        SetCommState(g_serial.hSerial, &g_serial.dcbOrig);

        CloseHandle(g_serial.hSerial);
        g_serial.hSerial = INVALID_HANDLE_VALUE;
    }

    g_serial.initialized = 0;
}

#endif

/* Common implementation */

const char* serial_get_last_error(void)
{
    return g_serial.error_msg;
}
