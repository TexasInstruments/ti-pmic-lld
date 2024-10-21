/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 * @file pmic_io.c
 *
 * @brief PMIC I/O source file containing definitions to I/O related
 * functionalities.
 */
#include "pmic.h"
#include "pmic_core.h"
#include "pmic_io.h"

// Types of errors that can be indicated by the PMIC in its response during an
// I/O transaction.
#define STATUS_ERR                  ((uint8_t)1U << 7U)
#define WDG_ERR                     ((uint8_t)1U << 6U)
#define VMON_SAM_ERR                ((uint8_t)1U << 5U)
#define ESM_ERR                     ((uint8_t)1U << 3U)
#define SDO_RDBK_ERR                ((uint8_t)1U << 1U)
#define PREV_SPI_FRAME_INV_ERR      ((uint8_t)1U << 0U)

// Bytes of a TPS65385XX SPI frame.
#define SPI_M_CMD_BYTE              ((uint8_t)0U)
#define SPI_M_DATA_BYTE             ((uint8_t)1U)
#define SPI_M_MCRC_BYTE             ((uint8_t)2U)
#define SPI_S_STATUS_BYTE           ((uint8_t)0U)
#define SPI_S_RESPONSE_BYTE         ((uint8_t)1U)
#define SPI_S_SCRC_BYTE             ((uint8_t)2U)
#define NUM_BYTES_IN_SPI_FRAME      ((uint8_t)3U)

// Number of bytes that are transferred in a standard SPI frame.
#define SPI_BYTE_XFER_CNT           ((uint8_t)3U)

// Check the PMIC status for fault/error indication(s).
static inline bool Pmic_ioStatusCheck(uint8_t pmicStatus)
{
    const uint8_t statusErrors = (STATUS_ERR | WDG_ERR | VMON_SAM_ERR |
        ESM_ERR | SDO_RDBK_ERR | PREV_SPI_FRAME_INV_ERR);

    return ((bool)((pmicStatus & statusErrors) != 0U));
}

int32_t Pmic_ioTransfer(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t wData, uint8_t *rData)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t spiTxBuf = 0U, spiRxBuf = 0xFFU;
    uint8_t spiFrame[NUM_BYTES_IN_SPI_FRAME] = {0U};

    // Check parameters
    if ((pmicHandle == NULL) || (pmicHandle->ioTransfer == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Perform SPI transfer
    if (status == PMIC_ST_SUCCESS)
    {
        spiFrame[SPI_M_CMD_BYTE] = cmd;
        spiFrame[SPI_M_DATA_BYTE] = wData;

        // Get the CRC of the command and write data; form the transmit data.
        // Bits 23-16: CMD, bits 15-8: data, bits 7-4: Reserved, bits 3-0: MCRC
        spiFrame[SPI_M_MCRC_BYTE] = Pmic_getCRC8Val(spiFrame, 2U);
        spiTxBuf = (((uint32_t)spiFrame[SPI_M_CMD_BYTE] << 16U) |
                    ((uint32_t)spiFrame[SPI_M_DATA_BYTE] << 8U) |
                    ((uint32_t)spiFrame[SPI_M_MCRC_BYTE] & 0xFU));

        // SPI transfer
        status = pmicHandle->ioTransfer(pmicHandle, spiTxBuf, &spiRxBuf, SPI_BYTE_XFER_CNT);
    }

    // Convert platform specific error code to driver error code
    if (status != PMIC_ST_SUCCESS)
    {
        status = PMIC_ST_ERR_SPI_COMM_FAIL;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Decompose received SPI frame
        spiFrame[SPI_S_STATUS_BYTE] = (spiRxBuf >> 16U) & 0xFFU;
        spiFrame[SPI_S_RESPONSE_BYTE] = (spiRxBuf >> 8U) & 0xFFU;
        spiFrame[SPI_S_SCRC_BYTE] = spiRxBuf & 0xFFU;

        // Compare SCRC against calculated CRC
        if ((Pmic_getCRC8Val(spiFrame, 2U) & 0xFU) != (spiFrame[SPI_S_SCRC_BYTE] & 0xFU))
        {
            status = PMIC_ST_ERR_INV_CRC;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Check for PMIC faults/errors
        if (Pmic_ioStatusCheck(spiFrame[SPI_S_STATUS_BYTE]))
        {
            Pmic_irqResponse(pmicHandle);
        }

        // Store PMIC response
        if (rData != NULL)
        {
            *rData = spiFrame[SPI_S_RESPONSE_BYTE];
        }
    }

    return status;
}

int32_t Pmic_ioTransfer_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t wData, uint8_t *rData)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioTransfer(pmicHandle, cmd, wData, rData);
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_ioTxByte_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t wData)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioTxByte(pmicHandle, cmd, wData);
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_ioRxByte_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t *rData)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRxByte(pmicHandle, cmd, rData);
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}
