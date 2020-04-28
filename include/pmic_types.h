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
 *  \ingroup DRV_PMIC_MODULE
 *  \defgroup DRV_PMIC_TYPE_MODULE PMIC Driver Common Data types
 *            This is PMIC driver common parameters and API
 *
 *  @{
 */

/**
 *  \file pmic_types.h
 *
 *  \brief PMIC Low Level Driver API/interface data types file.
 */

#ifndef PMIC_TYPES_H_
#define PMIC_TYPES_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */


/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
 * \brief: PMIC Subsystem is supported or not
 *
 *  \param   gpioEnable    PMIC GPIO SubSystem is supported or not
 *  \param   rtcEnable     PMIC RTC SubSystem is supported or not
 *  \param   wdgEnable     PMIC Watchdog SubSystem is supported or not
 */
typedef struct Pmic_DevSubSysInfo_s
{
   bool    gpioEnable;
   bool    rtcEnable;
   bool    wdgEnable;
} Pmic_DevSubSysInfo_t;

/*!
 * \brief: PMIC Interface Handle.
 *         Contains various PMIC driver instance specific info -
 *         the PMIC device type, PMIC interface mode, Slave address,
 *         various application defined API function pointers for
 *         LLD and Critical sections
 *         Note - Driver will configure the Pmic Handle info. User should Ensure that
 *         Application shall not update/configure the PMIC Handle info.
 *
 *  \param   pPmic_SubSysInfo             PMIC driver subsystem information
 *  \param   drvInitStatus                PMIC Driver initialization status
 *                                        Valid driver status:
 *                                        Main instance:
 *                                           DRV_INIT_STATUS | PMIC_MAIN_INST
 *                                        QA instance:
 *                                           DRV_INIT_STATUS | PMIC_QA_INST
 *  \param   pmicDeviceType               PMIC device type
 *  \param   commMode                     Interface mode - Single I2C, Dual
 *                                        I2C or SPI
 *  \param   slaveAddr                    Main Interface Slave Address
 *  \param   qaSlaveAddr                  WDOG QA Interface Slave Address
 *  \param   crcEnable                    Parameter to enable/disable CRC
 *  \param   pFnPmicCommIoRead            Pointer to I2C/SPI Comm LLD Read
 *                                        Function
 *  \param   pFnPmicCommIoWrite           Pointer to I2C/SPI Comm LLD Write
 *                                        Function
 *  \param   pCommHandle                  Pointer to Handle for I2C1/SPI
 *                                        Main Interface
 *  \param   pQACommHandle                Pointer to Handle for I2C2-QA
 *                                        Interface
 *  \param   pFnPmicCritSecStart          Pointer to Pmic Critical-Section
 *                                        Start Function
 *  \param   pFnPmicCritSecStop           Pointer to Pmic Critical-Section
 *                                        Stop Function
 */
typedef struct Pmic_CoreHandle_s {
    Pmic_DevSubSysInfo_t *pPmic_SubSysInfo;
    uint32_t              drvInitStatus;
    uint8_t               pmicDeviceType;
    uint8_t               commMode;
    uint8_t               slaveAddr;
    uint8_t               qaSlaveAddr;
    bool                  crcEnable;
    void                 *pCommHandle;
    void                 *pQACommHandle;
    int32_t (*pFnPmicCommIoRead)(struct Pmic_CoreHandle_s  *pmicCorehandle,
                                 uint8_t                    instType,
                                 uint16_t                   regAddr,
                                 uint8_t                   *pRxBuf,
                                 uint8_t                    bufLen);
    int32_t (*pFnPmicCommIoWrite)(struct Pmic_CoreHandle_s *pmicCorehandle,
                                  uint8_t                   instType,
                                  uint16_t                  regAddr,
                                  uint8_t                  *pTxBuf,
                                  uint8_t                   bufLen);
    void (*pFnPmicCritSecStart)(void);
    void (*pFnPmicCritSecStop)(void);
} Pmic_CoreHandle_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_TYPES_H_ */

/* @} */
