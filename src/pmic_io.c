/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
#include "regmap/io.h"

/*========================================================================== */
/*                          Macros & Typedefs                                */
/*========================================================================== */
#define PMIC_IO_BUF_SIZE (4U)
#define PMIC_COMM_CRC_INITIAL_VALUE (uint32_t)(0xFF)

/*========================================================================== */
/*                         Function Definitions                              */
/*========================================================================== */
static uint8_t IO_calcCRC8(uint8_t cmd, uint8_t rdwr, uint8_t dat) {
    uint32_t crc = PMIC_COMM_CRC_INITIAL_VALUE;
    uint32_t tmp = (uint32_t)((uint32_t)cmd << 16) | ((uint32_t)rdwr << 8) | (uint32_t)dat;

    /* Standard CRC-8 polynomial, X8 + X2 + X1 + 1, is used to calculate the
     * checksum value based on the command and data which the MCU transmits to
     * the PMIC device. */
    for (int8_t i = 0; i < 24; i++) {
        uint64_t D = (uint64_t)((uint64_t)tmp & (uint64_t)0x800000) / (uint64_t)0x800000;
        tmp = (tmp & (uint32_t)0x7FFFFF) * (uint32_t)2;

        D = D ^ ((uint64_t)((uint64_t)crc & (uint64_t)0x80) / (uint64_t)0x80);
        crc = (crc & 0x7FU) * 2U;

        D *= (uint64_t)7;
        crc ^= (uint32_t)D;
    }

    /* Return the PMIC MCRC value */
    return (uint8_t)crc;
}

int32_t Pmic_ioRxByte(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t *pRxBuffer) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t rxBuf[PMIC_IO_BUF_SIZE] = {0};

    status = handle->pFnPmicCommIoRd(handle, (uint8_t)PMIC_MAIN_INST, regAddr, &rxBuf[0], 2U);

    if (status == PMIC_ST_SUCCESS) {
        *pRxBuffer = rxBuf[0];
    }

    return status;
}

int32_t Pmic_ioTxByte(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t txData) {
    /* Set write data to txBuf[2], with WDATA[7:0] */
    uint8_t txBuf[PMIC_IO_BUF_SIZE] = {0U, 0U, txData, 0U};
    const uint8_t frameSize = handle->crcEnable ? 4U : 3U;

    /* Frame 3 Bytes with IO header+data as per PMIC I2C IO algorithm explained
     * in PMIC TRM */

    /* Set I2C_ID and ADDR values */
    txBuf[0U] = (uint8_t)(handle->slaveAddr << 1U) | 0U;
    txBuf[1U] = (uint8_t)(regAddr & 0xFF);

    /* If CRC is enabled, set CRC data to txBuf[3], Bits 25-32 CRC */
    if (handle->crcEnable) {
        txBuf[3U] = IO_calcCRC8(txBuf[0], txBuf[1], txData);
    }

    return handle->pFnPmicCommIoWr(handle, (uint8_t)PMIC_MAIN_INST, regAddr, &txBuf[0], frameSize);
}

int32_t Pmic_ioGetCrcEnableState(Pmic_CoreHandle_t *handle, bool *isEnabled) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Read the INTERFACE_CONF register
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, PMIC_IO_INTERFACE_CONF, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract the relevant bit
    if (status == PMIC_ST_SUCCESS) {
        *isEnabled = Pmic_getBitField_b(regData, PMIC_INTF_CONF_I2C_CRC_EN_SHIFT);
    }

    return status;
}

int32_t Pmic_ioSetCrcEnableState(Pmic_CoreHandle_t *handle, bool enable) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Read the INTERFACE_CONF register
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, PMIC_IO_INTERFACE_CONF, &regData);
    }

    // Set the I2C_CRC_EN bit accordingly and then write back to the register
    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regData, PMIC_INTF_CONF_I2C_CRC_EN_SHIFT, enable);
        status = Pmic_ioTxByte(handle, PMIC_IO_INTERFACE_CONF, regData);
    }
    Pmic_criticalSectionStop(handle);

    // Update the handle crcEnable property to track with the HW status
    if (status == PMIC_ST_SUCCESS) {
        handle->crcEnable = enable;
    }

    return status;
}

int32_t Pmic_ioCrcEnable(Pmic_CoreHandle_t *handle) {
    return Pmic_ioSetCrcEnableState(handle, PMIC_ENABLE);
}

int32_t Pmic_ioCrcDisable(Pmic_CoreHandle_t *handle) {
    return Pmic_ioSetCrcEnableState(handle, PMIC_DISABLE);
}
