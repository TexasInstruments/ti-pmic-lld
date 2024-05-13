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
#ifndef __PMIC_TYPES_H__
#define __PMIC_TYPES_H__

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
typedef struct Pmic_DevSubSysInfo_s {
    bool gpioEnable;
    bool rtcEnable;
    bool wdgEnable;
    bool buckEnable;
    bool ldoEnable;
    bool esmEnable;
} Pmic_DevSubSysInfo_t;

typedef struct Pmic_CoreHandle_s {
    const Pmic_DevSubSysInfo_t *pPmic_SubSysInfo;
    uint32_t drvInitStatus;
    uint8_t pmicDeviceType;
    uint8_t pmicDevRev;
    uint8_t pmicDevSiliconRev;
    uint8_t commMode;
    uint8_t slaveAddr;
    uint8_t qaSlaveAddr;
    uint8_t nvmSlaveAddr;
    uint8_t i2c1Speed;
    uint8_t i2c2Speed;
    bool crcEnable;
    void *pCommHandle;
    void *pQACommHandle;
    int32_t (*pFnPmicCommIoRead)(struct Pmic_CoreHandle_s *pmicCorehandle,
                                 uint8_t instType, uint16_t regAddr,
                                 uint8_t *pRxBuf, uint8_t bufLen);
    int32_t (*pFnPmicCommIoWrite)(struct Pmic_CoreHandle_s *pmicCorehandle,
                                  uint8_t instType, uint16_t regAddr,
                                  uint8_t *pTxBuf, uint8_t bufLen);
    void (*pFnPmicCritSecStart)(void);
    void (*pFnPmicCritSecStop)(void);
} Pmic_CoreHandle_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PMIC_TYPES_H__ */
