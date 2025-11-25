/******************************************************************************
 *Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/*========================================================================== */
/*                            Include Files                                  */
/*========================================================================== */
#include <stdint.h>
#include <stddef.h>

#include "pmic.h"
#include "pmic_io.h"

/*========================================================================== */
/*                          Macros & Typedefs                                */
/*========================================================================== */
#define PMIC_IO_BUF_SIZE (4U)
#define PMIC_IO_REQ_RW   (uint8_t)(1U << 4U)
#define PMIC_COMM_CRC_INITIAL_VALUE (uint32_t)(0xFF)

/* SPI protocol constants */
#define SPI_PAGE_MASK         ((uint8_t)0x07U)  /* 3-bit page number extraction */
#define SPI_ADDR_MASK         ((uint8_t)0xFFU)  /* 8-bit register address mask */
#define SPI_PAGE_SHIFT        ((uint8_t)5U)     /* Page field position in command byte */
#define SPI_ADDR_BYTE_SHIFT   ((uint8_t)8U)     /* Extract page from 16-bit address */

/*========================================================================== */
/*                         Function Definitions                              */
/*========================================================================== */
/**
 *  Used CRC Polynomial:  x^8 + x^2 + x + 1
 *   Evaluation of CRC Polynomial value from equation:
 *     1*x^8 + 0*x^7 + 0*x^6 + 0*x^5 + 0*x^4 + 0*x^3 + 1*x^2 + 1*x + 1
 *  =  1     + 0     + 0     + 0     + 0     + 0     + 1     + 1   + 1
 *  =  0x107(After discarding MSB bit) = 0x7
 *
 *  CRC Polynomial value: 0x07, Initial Value: 0x00, Final Value: 0x00
 *  link to generate custom CRC table from polynomial:
 *  http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
 */
const static uint8_t CRC8_TABLE[] = {
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

/**
 * @brief Helper function to get CRC8 data.
 *
 * @details Initial value: 0xFF
 * Big-endian bit stream order
 *
 * @param data [IN] Data for which CRC is to be determined.
 *
 * @param length [IN] Length of the buffer in bytes.
 *
 * @return CRC value for data.
 */
static uint8_t getCRC8Val(const uint8_t *data, uint8_t len) {
    uint8_t crc = PMIC_COMM_CRC_INITIAL_VALUE;

    for (uint8_t i = 0U; i < len; i++) {
        crc = CRC8_TABLE[data[i] ^ crc];
    }

    return crc;
}

int32_t Pmic_ioRxByte(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t *rxBuffer) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t spiBuf[PMIC_IO_BUF_SIZE] = {0U};
    uint8_t bufLen = 0U;

    // Validate handle
    if (handle == NULL) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    // Validate rxBuffer parameter
    if ((status == PMIC_ST_SUCCESS) && (rxBuffer == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        // spiBuf[0] = Target register address
        spiBuf[0U] = (uint8_t)(regAddr & SPI_ADDR_MASK);

        // spiBuf[1] = page, R/W, reserved bits
        // bits 7:5 -> page, bit 4 -> R/W, bit 3:0 -> reserved bits
        spiBuf[1U] = (uint8_t)((uint8_t)((uint8_t)(regAddr >> SPI_ADDR_BYTE_SHIFT) & SPI_PAGE_MASK) << SPI_PAGE_SHIFT);
        spiBuf[1U] |= PMIC_IO_REQ_RW;
        bufLen = 3U;

        // spiBuf[3] = CCRC calculated based on spiBuf[0], spiBuf[1], and spiBuf[2]
        spiBuf[3U] = getCRC8Val(spiBuf, bufLen);
        bufLen = 4U;

        // User-implemented hook transmits spiBuf then overwrites spiBuf with received data
        status = handle->ioRead(handle, 0, (uint8_t)regAddr, spiBuf, bufLen);

        // Validate PCRC
        if (status == PMIC_ST_SUCCESS) {
            if (getCRC8Val(spiBuf, bufLen - 1U) != spiBuf[3U]) {
                status = PMIC_ST_ERR_DATA_IO_CRC;
            }
        }

        // Save RDATA[7:0]
        if (status == PMIC_ST_SUCCESS) {
            *rxBuffer = spiBuf[2U];
        }
    }

    return status;
}

int32_t Pmic_ioRxByte_CS(Pmic_Handle_t *handle, uint16_t regAddr, uint8_t *rxBuffer) {
    int32_t status = PMIC_ST_SUCCESS;

    // Validate handle before critical section
    if (handle == NULL) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    // Validate rxBuffer parameter
    if ((status == PMIC_ST_SUCCESS) && (rxBuffer == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, regAddr, rxBuffer);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_ioTxByte(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t txData) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t spiBuf[PMIC_IO_BUF_SIZE] = {0U};
    uint8_t bufLen = 0U;

    // Validate handle
    if (handle == NULL) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if (status == PMIC_ST_SUCCESS) {
        // spiBuf[0] = Target register address
        spiBuf[0U] = (uint8_t)(regAddr & SPI_ADDR_MASK);

        // spiBuf[1] = page, R/W, reserved bits
        // bits 7:5 -> page, bit 4 -> R/W, bit 3:0 -> reserved bits
        spiBuf[1U] = (uint8_t)((uint8_t)((uint8_t)(regAddr >> SPI_ADDR_BYTE_SHIFT) & SPI_PAGE_MASK) << SPI_PAGE_SHIFT);
        spiBuf[1U] &= (uint8_t)(~PMIC_IO_REQ_RW);

        // spiBuf[2] = WDATA[7:0]
        spiBuf[2U] = txData;
        bufLen = 3U;

        // spiBuf[3] = CCRC calculated based on spiBuf[0], spiBuf[1], and spiBuf[2]
        spiBuf[3U] = getCRC8Val(spiBuf, bufLen);
        bufLen = 4U;

        // User-implemented hook transmits spiBuf
        status = handle->ioWrite(handle, 0, (uint8_t)regAddr, spiBuf, bufLen);
    }

    return status;
}

int32_t Pmic_ioTxByte_CS(Pmic_Handle_t *handle, uint16_t regAddr, uint8_t txData) {
    int32_t status = PMIC_ST_SUCCESS;

    // Validate handle before critical section
    if (handle == NULL) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, regAddr, txData);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_ioUpdateByte(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(pmicHandle, regAddr, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, shift, mask, value);

        status = Pmic_ioTxByte(pmicHandle, regAddr, regData);
    }

    return status;
}

int32_t Pmic_ioUpdateByte_CS(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioUpdateByte(pmicHandle, regAddr, shift, mask, value);
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_ioUpdateByte_b(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t shift, bool value)
{
    return Pmic_ioUpdateByte(pmicHandle, regAddr, shift, (uint8_t)(1U << shift), value ? 1U : 0U);
}

int32_t Pmic_ioUpdateByte_bCS(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t shift, bool value)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioUpdateByte_b(pmicHandle, regAddr, shift, value);
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_ioTxWordSeq(Pmic_Handle_t *handle, uint16_t baseAddr, uint32_t txData, uint8_t count) {
    int32_t status = PMIC_ST_SUCCESS;

    // Validate that `count` is within bounds of the storage type
    if (count > sizeof(uint32_t))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        for (uint8_t i = 0; ((i < count) && (status == PMIC_ST_SUCCESS)); i++)
        {
            status = Pmic_ioTxByte(handle, baseAddr + (uint16_t)i, (uint8_t)((txData >> (i * 8U)) & 0xFFU));
        }
    }

    return status;
}

int32_t Pmic_ioRxWordSeq(Pmic_Handle_t *handle, uint16_t baseAddr, uint32_t *rxData, uint8_t count) {
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t value = 0U;
    uint8_t regData = 0U;

    // Validate rxData parameter
    if (rxData == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Validate that `count` is within bounds of the storage type
    if ((status == PMIC_ST_SUCCESS) && (count > sizeof(uint32_t)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read from registers sequentially, starting at baseAddr
        for (uint8_t i = 0; ((i < count) && (status == PMIC_ST_SUCCESS)); i++)
        {
            status = Pmic_ioRxByte(handle, baseAddr + (uint16_t)i, &regData);
            value |= (uint32_t)(regData << (i * 8U));
        }
    }

    // Store composite value back to the user facing param if all transactions were successful.
    if (status == PMIC_ST_SUCCESS)
    {
        *rxData = value;
    }

    return status;
}
