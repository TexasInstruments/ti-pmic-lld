/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file  pmic_io_priv.h
 *
 *  \brief  This file contains LLD-Communication wrappers with CRC8 support for
 *          I2C/SPI
 */

#ifndef PMIC_IO_PRIV_H_
#define PMIC_IO_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*!
 * \brief: WatchDog register I2C access
 */
#define PMIC_WDG_PAGEADDR                      (0x400U)
#define PMIC_WDG_PAGEADDR_MASK                 (0x3FFU)

/*!
 * \brief: SPI R/W bit Position
 */
#define PMIC_IO_REQ_RW                         (((uint32_t)1U) << 11U)

/*!
 * \brief: IO Buffer Size
 */
#define PMIC_IO_BUF_SIZE                       (4U)

/*!
 * \brief: Initial value for CRC
 */
#define PMIC_COMM_CRC_INITIAL_VALUE            (0xFF)

/*!
 * \brief: IO READ bits
 */
#define PMIC_IO_READ                           (0x01U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/*!
 * \brief: PMIC I2C/SPI IO Wrapper function used internally by driver to send
 *         register values with or without CRC.
 *         All PMIC Driver API functions which are called after successful PMIC
 *         Instance Setup shall use this Send byte function to do IO with
 *         PMIC to control and monitor register values.
 *         This function will call application initilized Communication IO write
 *         callback functions
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle
 * \param   regAddr           [IN]    Register address
 * \param   txData            [IN]    Data to be written
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_commIntf_sendByte(Pmic_CoreHandle_t *pPmicCoreHandle,
                               uint16_t           regAddr,
                               uint8_t            txData);

/*!
 * \brief: PMIC I2C/SPI IO Wrapper function pointers used internally by driver
 *         to receive register values with or without CRC.
 *         All PMIC Driver API functions which are called after successful PMIC
 *         Instance Setup shall use this receive byte function to do IO with
 *         PMIC to control and monitor register values.
 *         This function will call application initilized Communication IO read
 *         callback functions
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   regAddr           [IN]    Register address.
 * \param   pRxBuffer         [OUT]   Pointer to Buffer to receive data
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_commIntf_recvByte(Pmic_CoreHandle_t *pPmicCoreHandle,
                               uint16_t           regAddr,
                               uint8_t           *pRxBuffer);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /*PMIC_IO_PRIV_H_*/
