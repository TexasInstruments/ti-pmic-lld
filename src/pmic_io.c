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

#include "pmic.h"
#include "pmic_io.h"

/*========================================================================== */
/*                          Macros & Typedefs                                */
/*========================================================================== */
#define PMIC_IO_BUF_SIZE (4U)
#define PMIC_IO_REQ_RW   (uint8_t)(1U << 4U)
#define PMIC_COMM_CRC_INITIAL_VALUE (uint32_t)(0xFF)

/*========================================================================== */
/*                         Function Definitions                              */
/*========================================================================== */
static uint8_t IO_calcCRC8(uint8_t cmd, uint8_t rdwr, uint8_t dat) {
    uint32_t crc = PMIC_COMM_CRC_INITIAL_VALUE;
    uint32_t tmp = (uint32_t)((uint32_t)cmd << 16) | ((uint32_t)rdwr << 8) | (uint32_t)dat;

    /* Standard CRC-8 polynomial, X8 + X2 + X1 + 1, is used to calculate the
     * checksum value based on the command and data which the MCU transmits to
     * the TPS653850A-Q1 device. */
    for (int8_t i = 0; i < 24; i++) {
        uint64_t D = (uint64_t)((uint64_t)tmp & (uint64_t)0x800000) / (uint64_t)0x800000;
        tmp = (tmp & (uint32_t)0x7FFFFF) * (uint32_t)2;

        D = D ^ ((uint64_t)((uint64_t)crc & (uint64_t)0x80) / (uint64_t)0x80);
        crc = (crc & 0x7FU) * 2U;

        D *= (uint64_t)7;
        crc ^= (uint32_t)D;
    }

    /* Return the PMIC SPI MCRC value */
    return (uint8_t)crc;
}

int32_t Pmic_ioRxByte(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t *pRxBuffer) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t rxBuf[PMIC_IO_BUF_SIZE] = {0};

    /*
     *Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm
     *explained in PMIC TRM
     */

    /*Set ADDR to pRxBuf[0], with ADDR[7:0] */
    rxBuf[0U] = (uint8_t)(regAddr & 0xFFU);

    /*Set PAGE to pRxBuf[1] 7:5 bits, with PAGE[2:0] */
    /*Set R/W in pRxBuf[1] as bit-4, for read Request */
    rxBuf[1U] = (uint8_t)((uint8_t)((uint8_t)(regAddr >> 8U) & 0x7U) << 5U);
    rxBuf[1U] |= PMIC_IO_REQ_RW;

    status = handle->pFnPmicCommIoRead(handle, (uint8_t)PMIC_MAIN_INST, regAddr, &rxBuf[0], 4U);

    if (status == PMIC_ST_SUCCESS) {
        /*Copy data which shall be in rxBuf[2]/rxBuf[0] to pRxBuffer */
        *pRxBuffer = rxBuf[2];
    }

    return status;
}

int32_t Pmic_ioTxByte(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t txData) {
    /*Set write data to txBuf[2], with WDATA[7:0] */
    uint8_t txBuf[PMIC_IO_BUF_SIZE] = {0U, 0U, txData, 0U};

    /* Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm explained
     * in PMIC TRM */

    /* Set ADDR to txbuf[0] with ADDR[7:0] */
    txBuf[0U] = (uint8_t)(regAddr & 0xFFU);

    /*Set PAGE to txBuf[1] 7:5 bits with PAGE[2:0] */
    /*Set R/W in txBuf[1] as bit-4, for Write Request */
    txBuf[1U] = (uint8_t)((uint8_t)((uint8_t)(regAddr >> 8U) & 0x7U) << 5U);
    txBuf[1U] &= (uint8_t)(~PMIC_IO_REQ_RW);

    /*Set CRC data to txBuf[3], Bits 25-32 CRC */
    txBuf[3U] = IO_calcCRC8(txBuf[0], txBuf[1], txData);

    return handle->pFnPmicCommIoWrite(handle, (uint8_t)PMIC_MAIN_INST, regAddr, &txBuf[0], 4U);
}
