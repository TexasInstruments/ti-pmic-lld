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
 *  @file pmic_core_tps65386.c
 *
 *  @brief This file contains TPS65386x PMIC Core related APIs
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_core_tps65386x.h"
#include <board/pmic.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/mcspi.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Private Functions */
int32_t PMIC_mcspiReadRegister(MCSPI_Handle handle,
                               MCSPI_Transaction *spiTransaction, uint8_t cmd,
                               uint8_t *data);
int32_t PMIC_mcspiWriteRegister(MCSPI_Handle handle,
                                MCSPI_Transaction *spiTransaction, uint8_t cmd,
                                uint8_t data);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint32_t gPmicMcspiTxBuffer[PMIC_MCSPI_MSGSIZE];
uint32_t gPmicMcspiRxBuffer[PMIC_MCSPI_MSGSIZE];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

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
uint8_t PMIC_calcCRC8(char cmd, char rdwr, char dat) {
  int i = 0;
  uint8_t crc;
  uint32_t tmp;

  tmp = ((cmd << 16) | (rdwr << 8) | dat);
  crc = 0xFF;

  /* Standard CRC-8 polynomial ,X8 + X2 + X1 + 1.,is used to calculate the
   * checksum value based on the command and data which the MCU transmits
   * to the TPS653850A-Q1 device.
   */

  for (i = 0; i < 24; i++) {
    long D;
    D = (tmp & 0x800000) / 8388608;
    tmp = (tmp & 0x7FFFFF) * 2;
    D = (D ^ (((long)crc & 0x80) / 128));
    crc = (crc & 0x7Fu) * 2u;
    D = D * 7;
    crc = (crc ^ (unsigned char)D);
  }

  /* Return the PMIC SPI MCRC value */
  return crc;
}

/**
 * @brief Initialize PMIC communication interface and resources.
 * This function initializes the PMIC communication interface and resources
 * based on the provided configuration and parameters.
 *
 * @param config Pointer to the PMIC configuration structure.
 * @param params Pointer to the PMIC parameters structure.
 * @return status Status of the operation (SystemP_SUCCESS if successful,
 * SystemP_FAILURE otherwise).
 */
int32_t PMIC_tps653860xxOpen(PMIC_Config *config, const PMIC_Params *params) {
  int32_t status = SystemP_SUCCESS;
  PMIC_Object *object;

  if ((NULL == config) || (NULL == params)) {
    status = SystemP_FAILURE;
  }

  if (status == SystemP_SUCCESS) {
    object = (PMIC_Object *)config->object;
    object->mcspiInstance = params->mcspiInstance;
    object->mcspiHandle = MCSPI_getHandle(object->mcspiInstance);
    if (NULL == object->mcspiHandle) {
      status = SystemP_FAILURE;
    }
  }

  return status;
}

/**
 * @brief Deinitialize PMIC communication interface and resources.
 * This function deinitializes the PMIC communication interface and resources.
 *
 * @param config Pointer to the PMIC configuration structure.
 * @return void No return value.
 */
void PMIC_tps653860xxClose(PMIC_Config *config) {
  int32_t status = SystemP_SUCCESS;
  PMIC_Object *object;

  if (NULL == config) {
    status = SystemP_FAILURE;
  }

  if (status == SystemP_SUCCESS) {
    object = (PMIC_Object *)config->object;

    object->mcspiHandle = NULL;
  }

  return;
}

/**
 * @brief Write data to a register via the MCSPI interface.
 * This function writes data to a register via the MCSPI interface, including
 * the calculated CRC-8 checksum.
 *
 * @param handle MCSPI handle for communication.
 * @param spiTransaction Pointer to the MCSPI transaction structure.
 * @param cmd Command byte for communication.
 * @param data Data byte to be written.
 * @return status Status of the operation (SystemP_SUCCESS if successful,
 * SystemP_FAILURE otherwise).
 */
int32_t PMIC_mcspiWriteRegister(MCSPI_Handle handle,
                                MCSPI_Transaction *spiTransaction, uint8_t cmd,
                                uint8_t data) {
  int32_t status = SystemP_SUCCESS;
  uint8_t crc = 0;

  crc = PMIC_calcCRC8(cmd, CMD_WR_EN, data);
  gPmicMcspiTxBuffer[0] =
      (cmd << CMD_SHIFT) | (CMD_WR_EN << RW_SHIFT) | (data << DAT_SHIFT) | crc;
  gPmicMcspiRxBuffer[0] = 0;

  status = MCSPI_transfer(handle, spiTransaction);

  if ((status != SystemP_SUCCESS) ||
      (MCSPI_TRANSFER_COMPLETED != spiTransaction->status)) {
    /* MCSPI transfer failed!! */
    DebugP_assert(FALSE);
    status = SystemP_FAILURE;
  }

  return status;
}

/**
 * @brief Read data from a register via the MCSPI interface.
 * This function reads data from a register via the MCSPI interface, including
 * the calculated CRC-8 checksum.
 *
 * @param handle MCSPI handle for communication.
 * @param spiTransaction Pointer to the MCSPI transaction structure.
 * @param cmd Command byte for communication.
 * @param data Pointer to store the read data.
 * @return status Status of the operation (SystemP_SUCCESS if successful,
 * SystemP_FAILURE otherwise).
 */
int32_t PMIC_mcspiReadRegister(MCSPI_Handle handle,
                               MCSPI_Transaction *spiTransaction, uint8_t cmd,
                               uint8_t *data) {
  int32_t status = SystemP_SUCCESS;
  uint8_t crc = 0;

  crc = PMIC_calcCRC8(cmd, CMD_RD_EN, 0);
  gPmicMcspiTxBuffer[0] =
      (cmd << CMD_SHIFT) | (CMD_RD_EN << RW_SHIFT) | (0 << DAT_SHIFT) | crc;
  gPmicMcspiRxBuffer[0] = 0;

  status = MCSPI_transfer(handle, spiTransaction);

  if ((status != SystemP_SUCCESS) ||
      (MCSPI_TRANSFER_COMPLETED != spiTransaction->status)) {
    /* MCSPI transfer failed!! */
    DebugP_assert(FALSE);
    status = SystemP_FAILURE;
  } else {
    *data = (gPmicMcspiRxBuffer[0] >> 8) & 0xFF;
  }

  return status;
}
