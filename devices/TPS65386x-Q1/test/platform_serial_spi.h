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
 * @file platform_serial_spi.h
 * @brief SPI serial communication layer for host-controlled PMIC testing
 *
 * This module provides SPI-based serial communication to interface with
 * pmic-tiva-host firmware running on TM4C123. SPI commands are sent via UART
 * and responses are parsed to implement the platform abstraction layer.
 *
 * TPS65386x-Q1 uses SPI interface, not I2C.
 */

#ifndef PLATFORM_SERIAL_SPI_H
#define PLATFORM_SERIAL_SPI_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Constants                            */
/* ========================================================================= */

/**
 * @brief Maximum command buffer size
 */
#define SERIAL_MAX_CMD_LEN          (512U)

/**
 * @brief Maximum response buffer size
 */
#define SERIAL_MAX_RESPONSE_SIZE    (4096U)
#define SERIAL_MAX_RESPONSE_LEN     SERIAL_MAX_RESPONSE_SIZE  /* Alias for compatibility */

/**
 * @brief Default serial port device path
 */
#define SERIAL_DEFAULT_PORT         "/dev/cu.usbmodem1131201"

/**
 * @brief Default baud rate
 */
#define SERIAL_DEFAULT_BAUD         (115200U)

/**
 * @brief Default serial port timeout in milliseconds
 */
#define SERIAL_DEFAULT_TIMEOUT_MS   (1000U)

/* ========================================================================= */
/*                             Return Codes                                  */
/* ========================================================================= */

#define SERIAL_SUCCESS              (0)
#define SERIAL_ERR_OPEN_FAILED      (-1)
#define SERIAL_ERR_CONFIG_FAILED    (-2)
#define SERIAL_ERR_WRITE_FAILED     (-3)
#define SERIAL_ERR_READ_TIMEOUT     (-4)
#define SERIAL_ERR_READ_FAILED      (-5)
#define SERIAL_ERR_INVALID_PARAM    (-6)
#define SERIAL_ERR_NOT_INITIALIZED  (-7)
#define SERIAL_ERR_BUFFER_OVERFLOW  (-8)

/* ========================================================================= */
/*                           Function Declarations                           */
/* ========================================================================= */

/**
 * @brief Initialize serial port connection to pmic-tiva-host firmware
 *
 * @param port      Serial port device path (e.g., "/dev/ttyUSB0", "/dev/tty.usbserial")
 * @param baud      Baud rate (typically 115200)
 *
 * @return SERIAL_SUCCESS on success, error code otherwise
 */
int32_t serial_init(const char *port, uint32_t baud);

/**
 * @brief Send command string to firmware via serial port
 *
 * Automatically appends newline character if not present.
 *
 * @param cmd       Command string to send (null-terminated)
 *
 * @return SERIAL_SUCCESS on success, error code otherwise
 */
int32_t serial_send_command(const char *cmd);

/**
 * @brief Read response from firmware via serial port
 *
 * Reads until newline or timeout. Response buffer is null-terminated.
 * Blocks until complete response received or timeout expires.
 *
 * @param response  Buffer to store response (null-terminated)
 * @param max_len   Maximum buffer size (including null terminator)
 *
 * @return Number of bytes read (excluding null terminator) on success,
 *         error code otherwise (negative values)
 */
int32_t serial_read_response(char *response, size_t max_len);

/**
 * @brief Close serial port and cleanup resources
 */
void serial_close(void);

/**
 * @brief Set serial port timeout for read operations
 *
 * @param timeout_ms    Timeout in milliseconds
 *
 * @return SERIAL_SUCCESS on success, error code otherwise
 */
int32_t serial_set_timeout(uint32_t timeout_ms);

/**
 * @brief Get current serial port timeout
 *
 * @return Timeout in milliseconds
 */
uint32_t serial_get_timeout(void);

/**
 * @brief Check if serial port is initialized and ready
 *
 * @return 1 if ready, 0 otherwise
 */
int32_t serial_is_ready(void);

/**
 * @brief Flush any pending data in serial buffers
 *
 * @return SERIAL_SUCCESS on success, error code otherwise
 */
int32_t serial_flush(void);

/**
 * @brief Get human-readable error message for last serial operation
 *
 * @return Pointer to error message string (do not free)
 */
const char* serial_get_last_error(void);

/* ========================================================================= */
/*                      SPI-Specific Helper Functions                        */
/* ========================================================================= */

/**
 * @brief Initialize SPI peripheral on firmware side
 *
 * Sends "spic 0" command to initialize SSI2 (SPI host 0)
 *
 * @return 0 on success, negative on error
 */
int32_t platform_serial_init(void);

/**
 * @brief Uninitialize SPI peripheral on firmware side
 *
 * Sends "spiu 0" command to uninitialize SSI2
 *
 * @return 0 on success, negative on error
 */
int32_t platform_serial_uninit(void);

/**
 * @brief Write data via SPI
 *
 * Sends "spi 0 <speed> 0 <data>... 0 0" command
 * Format: spi <host> <speed_kHz> <cs> <data_bytes>... <read_len> <access>
 *
 * @param regAddr   Register address to write
 * @param buffer    Data buffer to write
 * @param bufLen    Number of bytes to write
 * @param speed_khz SPI speed in kHz (e.g., 2000 for 2MHz)
 *
 * @return 0 on success, negative on error
 */
int32_t platform_serial_write(uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen, uint32_t speed_khz);

/**
 * @brief Read data via SPI
 *
 * Sends "spi 0 <speed> 0 <reg_addr> <read_len> 0" command
 * Format: spi <host> <speed_kHz> <cs> <reg_addr> <read_len> <access>
 *
 * @param regAddr   Register address to read from
 * @param buffer    Buffer to store read data
 * @param bufLen    Number of bytes to read
 * @param speed_khz SPI speed in kHz (e.g., 2000 for 2MHz)
 *
 * @return 0 on success, negative on error
 */
int32_t platform_serial_read(uint8_t regAddr, uint8_t *buffer, uint8_t bufLen, uint32_t speed_khz);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_SERIAL_SPI_H */
