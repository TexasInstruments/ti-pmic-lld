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
#define CMD_RD_EN       (0x10)
#define CMD_WR_EN       (0x00)
#define CMD_SHIFT       (24U)
#define RW_SHIFT        (16U)
#define DAT_SHIFT       (8U)
#define CMD_DEVICE_ID   (0x00)

/**
 * @brief: PMIC SERIAL_IF_CONFIG register address (Bank/Page 1 Register address)
 *         Application can only read this register to check I2C1SPI/I2C2 CRC
 *         is enabled or not
 */
#define PMIC_SERIAL_IF_CONFIG_PAGEADDR (0x100U)
#define PMIC_SERIAL_IF_CONFIG_PAGEADDR_MASK (0xFFU)

/** @brief: SPI R/W bit Position */
#define PMIC_IO_REQ_RW (((uint32_t)1U) << 4U)

/** @brief: IO Buffer Size */
#define PMIC_IO_BUF_SIZE (4U)

/** @brief: Initial value for CRC */
#define PMIC_COMM_CRC_INITIAL_VALUE (0xFF)

/** @brief: IO READ bits */
#define PMIC_IO_READ (0x01U)

/*========================================================================== */
/*                         Function Declarations                             */
/*========================================================================== */

/*========================================================================== */
/*                         Function Definitions                              */
/*========================================================================== */

/**
 * @brief Calculate CRC-8 checksum for PMIC SPI communication.
 * This function calculates the CRC-8 checksum based on the command, read/write
 * flag, and data provided for communication between the MCU and the
 * TPS653850A-Q1 device.
 *
 * @param cmd Command byte for communication.
 * @param rdwr Read/Write flag for communication (CMD_RD_EN or CMD_WR_EN).
 * @param dat Data byte for communication.
 * @return crc Calculated CRC-8 checksum value.
 */
uint8_t Pmic_calcCRC8(uint8_t cmd, uint8_t rdwr, uint8_t dat) {
    int8_t i = 0;
    uint32_t crc;
    uint32_t tmp;

    tmp = ((uint32_t)cmd << 16) | ((uint32_t) rdwr << 8) | (uint32_t) dat;
    crc = (uint32_t)0xFF;

    /* Standard CRC-8 polynomial ,X8 + X2 + X1 + 1.,is used to calculate the
     * checksum value based on the command and data which the MCU transmits
     * to the TPS653850A-Q1 device.
     */

    for (i = 0; i < 24; i++) {
        uint64_t D;
        D = (uint64_t)((uint64_t)tmp & (uint64_t) 0x800000) / (uint64_t) 8388608;
        tmp = (tmp & (uint32_t) 0x7FFFFF) * (uint32_t) 2;
        D = D ^ ((uint64_t)((uint64_t) crc & (uint64_t) 0x80) / (uint64_t) 128);
        crc = (crc & 0x7FU) * 2U;
        D = D * (uint64_t) 7;
        crc = crc ^ (uint32_t) D;
    }

    /* Return the PMIC SPI MCRC value */
    return (uint8_t) crc;
}

int32_t Pmic_commIntf_sendByte(Pmic_CoreHandle_t *pPmicCoreHandle,
    uint16_t regAddr, uint8_t txData) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t buffLength = 1U;
    uint8_t instanceType = (uint8_t)PMIC_MAIN_INST;
    uint8_t txBuf[PMIC_IO_BUF_SIZE] = {0U};
    uint16_t pmicRegAddr = regAddr;
    uint8_t data = txData;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /*
         *Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm
         *explained in PMIC TRM
         */

        buffLength = 0;
        /*Set ADDR to txbuf[0] with ADDR[7:0] */
        txBuf[buffLength] = (uint8_t)(pmicRegAddr & 0xFFU);
        buffLength++;

        /*Set PAGE to txBuf[1] 7:5 bits with PAGE[2:0] */
        txBuf[buffLength] = (uint8_t)((uint8_t)((uint8_t)(pmicRegAddr >> 8U) & 0x7U) << 5U);

        /*Set R/W in txBuf[1] as bit-4, for Write Request */
        txBuf[buffLength] &= (uint8_t) ~PMIC_IO_REQ_RW;
        buffLength++;

        /*Set write data to txBuf[2], with WDATA[7:0] */
        txBuf[buffLength] = data;
        buffLength++;

        /*Set CRC data to txBuf[3], Bits 25-32 CRC */
        txBuf[buffLength] =
            Pmic_calcCRC8((uint8_t) txBuf[0], (uint8_t) txBuf[1], (uint8_t) txBuf[2]);
        /*Increment 1 more byte to store CRC8 */
        buffLength++;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = pPmicCoreHandle -> pFnPmicCommIoWrite(
            pPmicCoreHandle, instanceType, pmicRegAddr, txBuf, buffLength);
    }
    return pmicStatus;
}

static int32_t Pmic_commIoReadData(Pmic_CoreHandle_t *pPmicCoreHandle,
    uint16_t *pRegAddr, uint8_t *pBuffLength,
    uint8_t pRxBuf[], uint8_t *pinstanceType) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t buffLength = *pBuffLength;
    uint16_t pmicRegAddr = *pRegAddr;
    uint8_t *tempCfg = pinstanceType;
    uint8_t instanceType = *tempCfg;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /*
         *Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm
         *explained in PMIC TRM
         */
        buffLength = 0U;
        /*Set ADDR to pRxBuf[0], with ADDR[7:0] */
        pRxBuf[buffLength] = (uint8_t)(pmicRegAddr & 0xFFU);

        buffLength++;
        /*Set PAGE to pRxBuf[1] 7:5 bits, with PAGE[2:0] */
        pRxBuf[buffLength] = (uint8_t)((uint8_t)((uint8_t)(pmicRegAddr >> 8U) & 0x7U) << 5U);

        /*Set R/W in pRxBuf[1] as bit-4, for read Request */
        pRxBuf[buffLength] |= (uint8_t)PMIC_IO_REQ_RW;
        buffLength++;
        /*Increment 1 more byte for 8-bit data read from PMIC register */
        buffLength++;

        /*Increment 1 more byte to read CRC8 */
        buffLength++;

        *pBuffLength = buffLength;
        *pRegAddr = pmicRegAddr;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = pPmicCoreHandle -> pFnPmicCommIoRead(
            pPmicCoreHandle, instanceType, pmicRegAddr, pRxBuf, buffLength);
    }

    return pmicStatus;
}

int32_t Pmic_commIntf_recvByte(Pmic_CoreHandle_t *pPmicCoreHandle, uint16_t regAddr, uint8_t *pRxBuffer) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t buffLength = 1U;
    uint8_t rxBuf[PMIC_IO_BUF_SIZE] = {0};
    uint8_t crcData[PMIC_IO_BUF_SIZE] = {0};
    uint8_t instanceType = (uint8_t)PMIC_MAIN_INST;
    uint16_t pmicRegAddr = regAddr;

    pmicStatus = Pmic_commIoReadData(pPmicCoreHandle, &pmicRegAddr, &buffLength, rxBuf, &instanceType);

    if (pmicStatus == PMIC_ST_SUCCESS) {
        if (pPmicCoreHandle->commMode == PMIC_INTF_SPI) {
            /*Copy SPI frame data to crcData */
            for (uint8_t crcDataLen = 0U; crcDataLen < (PMIC_IO_BUF_SIZE - 1U); crcDataLen++) {
                crcData[crcDataLen] = rxBuf[crcDataLen];
            }
        } else {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }
        buffLength--;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /*Copy data which shall be in rxBuf[2]/rxBuf[0] to pRxBuffer */
        *pRxBuffer = rxBuf[2];
    }

    return pmicStatus;
}
