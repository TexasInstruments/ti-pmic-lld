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
#ifndef __PMIC_IO_H__
#define __PMIC_IO_H__

/**
 * @file pmic_io.h
 * @brief PMIC Driver Communications I/O API
 */

/**
 * @defgroup DRV_PMIC_IO_MODULE PMIC I/O Module
 * @brief Communications APIs to read from and write to PMIC registers.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
/**
 * @anchor Pmic_CrcEnableCfg
 * @name PMIC CRC Enable/Disable Configuration
 *
 *  @{
 */
#define PMIC_CRC_DISABLE    (0U)
#define PMIC_CRC_ENABLE     (1U)
/** @} */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/**
 * @ingroup DRV_PMIC_IO_MODULE
 * @brief Write a byte to the given PMIC `regAddr`, performing CRC on
 * communications if necessary and enabled.
 *
 * @param handle  [IN] PMIC Interface Handle
 * @param regAddr [IN] Register address to write to
 * @param txData  [IN] Data to send to `regAddr`
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxByte(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t txData);

/**
 * @ingroup DRV_PMIC_IO_MODULE
 * @brief Read a byte from the given PMIC `regAddr`, extracting the desired
 * register data from the CRC framed data returned by the PMIC.
 *
 * @param handle   [IN] PMIC Interface Handle
 * @param regAddr  [IN] Register address to read from
 * @param rxBuffer [IN] Buffer to store result data in
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxByte(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t *rxBuffer);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__PMIC_IO_H__*/
