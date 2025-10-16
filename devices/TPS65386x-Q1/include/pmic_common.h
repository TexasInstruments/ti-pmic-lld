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
#ifndef __PMIC_COMMON_H__
#define __PMIC_COMMON_H__

/**
 * @file pmic_common.h
 * @brief APIs and macros/typedefs commonly used across PMIC LLD.
 */

/**
 * @defgroup DRV_PMIC_COMMON_MODULE PMIC Common Module
 * @brief APIs and macros/typedefs commonly used across PMIC LLD.
 */

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/*                          Macros and Defines                              */
/*==========================================================================*/

#define COUNT(x) (sizeof(x) / sizeof(x[0]))

// Used to clear statuses
#define PMIC_CLEAR_STAT  ((bool)true)
#define PMIC_RETAIN_STAT ((bool)false)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @anchor Pmic_DevSubSysInfo
 * @name PMIC Device Subsystem Information
 *
 * @brief Used by LLD to figure out which subsystems are enabled for the PMIC
 * device.
 *
 * @param gpioEnable GPIO subsystem enabled/disabled status.
 *
 * @param rtcEnable RTC subsystem enabled/disabled status.
 *
 * @param wdgEnable WDG subsystem enabled/disabled status.
 *
 * @param buckEnable Buck converter subsystem enabled/disabled status.
 *
 * @param ldoEnable LDO regulator subsystem enabled/disabled status.
 *
 * @param esmEnable ESM subsystem enabled/disabled status.
 */
typedef struct Pmic_DevSubSysInfo_s {
    bool gpioEnable;
    bool rtcEnable;
    bool wdgEnable;
    bool buckEnable;
    bool ldoEnable;
    bool esmEnable;
} Pmic_DevSubSysInfo_t;

/**
 * @anchor Pmic_CoreHandle
 * @name PMIC Core Handle
 *
 * @brief Handle used by LLD to abstract platform and OS specific functionality.
 * Also contains PMIC device information.
 *
 * @attention This structure is a central resource used by almost all LLD APIs
 * and must be initialized via 'Pmic_init()' before it can be used by other LLD
 * APIs. End-users should not modify the contents of this structure after it has
 * been initialized.
 *
 * @param pPmic_SubSysInfo Structure used to indicate enabled/disabled subsystems.
 *
 * @param drvInitStatus Driver initialization status. Used by LLD as a measure to
 * prevent corrupted handle usage.
 *
 * @param pmicDeviceType PMIC device type.
 *
 * @param pmicDevRev PMIC device revision.
 *
 * @param pmicDevSiliconRev PMIC device silicon revision.
 *
 * @param commMode Communication mode of the PMIC. Some PMICs may only have one
 * communication mode while others could have multiple (e.g., single I2C, dual I2C,
 * SPI).
 *
 * @param slaveAddr Main PMIC device address.
 *
 * @param qaSlaveAddr Address for interacting with PMIC WDG Q&A.
 *
 * @param nvmSlaveAddr Address for interacting with PMIC NVM space.
 *
 * @param i2c1Speed I2C1 speed.
 *
 * @param i2c2Speed I2C2 speed.
 *
 * @param crcEnable Status of whether serial communication CRC is enabled. Set to true
 * if enabled, false otherwise.
 *
 * @param configCrcEnable Status of whether configuration CRC is enabled. Set to true
 * if enabled, false otherwise.
 *
 * @param pCommHandle Pointer to serial communication handle for the PMIC device.
 *
 * @param pQACommHandle Pointer to serial communication handle for PMIC WDG.
 *
 * @param pFnPmicCommIoRd Function pointer to platform-specific serial communication
 * read API.
 *
 * @param pFnPmicCommIoWr Function pointer to platform-specific serial communication
 * write API.
 *
 * @param pFnPmicCritSecStart Function pointer to OS-specific critical section start.
 *
 * @param pFnPmicCritSecStop Function pointer to OS-specific critical section stop.
 *
 * @param pFnPmicPseudoIrq Function pointer to application-specific IRQ response
 * when an IRQ is detected during WDG servicing.
 */
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

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @brief Checks whether a parameter is valid.
 *
 * @param validParamVal [IN] Valid parameter value. Each bit represents whether a
 * parameter is valid.
 *
 * @param bitPos [IN] Valid parameter to check for.
 *
 * @return True if parameter is valid, false otherwise.
 */
bool Pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos);

/**
 * @brief Checks whether a parameter is valid and whether the status code is
 * equal to LLD success code.
 *
 * @param vpv [IN] Valid parameter value. Each bit represents whether a parameter
 * is valid.
 *
 * @param bPos [IN] valid parameter to check for.
 *
 * @param status [IN] The API checks whether the value of this parameter is
 * equal to the PMIC LLD success code.
 *
 * @return True if the status code is equal to the LLD success code and the
 * parameter is valid.
 */
#define Pmic_validParamStatusCheck(vpv, bPos, status) ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(vpv, bPos))

/**
 * @brief Start a critical section for PMIC operations.
 * This function starts a critical section for PMIC operations, if the critical
 * section start function pointer is not NULL.
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *handle);

/**
 * @brief Stop a critical section for PMIC operations.
 * This function stops a critical section for PMIC operations, if the critical
 * section stop function pointer is not NULL.
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *handle);

/**
 * @brief Sets the bit field of an 8-bit unsigned integer to the desired value.
 *
 * @param regData [OUT] The API modifies the desired bit field of the value held
 * at this address.
 *
 * @param shift [IN] Bit field position.
 *
 * @param mask [IN] Bit field mask.
 *
 * @param value [IN] Desired bit field value to set.
 */
static inline void Pmic_setBitField(uint8_t *regData, uint8_t shift, uint8_t mask, uint8_t value)
{
    *regData = ((*regData & ~mask) | ((value << shift) & mask));
}

/**
 * @brief Set the value of a bitfield based on the "NAME" of the field, rather
 * than providing individual SHIFT/MASK values. A simplified version of
 * `Pmic_setBitField()`
 *
 * @param reg [OUT] The API modifies the desired bit field of the value held
 * at this address.
 *
 * @param name [IN] Bit field name.
 *
 * @param val [IN] Desired value to set the bit field to.
 */
#define Pmic_setBitFieldByName(reg, name, val) (Pmic_setBitField(reg, name##_SHIFT, name##_MASK, val))

/**
 * @brief Sets the bit field of an 8-bit unsigned integer to the desired boolean
 * value.
 *
 * @param regData [OUT] The API modifies the desired bit field of the value held
 * at this address.
 *
 * @param shift [IN] Bit field position.
 *
 * @param value [IN] Bit field value (either true or false).
 */
static inline void Pmic_setBitField_b(uint8_t *regData, uint8_t shift, bool value)
{
    Pmic_setBitField(regData, shift, (uint8_t)(1U << shift), value ? 1U : 0U);
}

/**
 * @brief Get desired bit field of an 8-bit unsigned integer.
 *
 * @param regData [IN] The API gets the desired bit field from this value.
 *
 * @param shift [IN] Bit field location.
 *
 * @param mask [IN] Bit field mask.
 *
 * @return Value of the desired bit field.
 */
static inline uint8_t Pmic_getBitField(uint8_t regData, uint8_t shift, uint8_t mask)
{
    return ((regData & mask) >> shift);
}

/**
 * @brief Retrieve the value of a bitfield based on the "NAME" of the field,
 * rather than providing individual SHIFT/MASK values. A simplified version of
 * `Pmic_getBitField()`
 *
 * @param reg [IN] The API gets the desired bit field from this value.
 *
 * @param name [IN] Bit field name.
 *
 * @return Value of the desired bit field.
 */
#define Pmic_getBitFieldByName(reg, name) (Pmic_getBitField(reg, name##_SHIFT, name##_MASK))

/**
 * @brief Get desired bit field of an 8-bit unsigned integer, casted as boolean.
 *
 * @param regData [IN] The API gets the desired bit field from this value.
 *
 * @param shift [IN] Bit field location.
 *
 * @return Value of the desired bit field.
 */
static inline bool Pmic_getBitField_b(uint8_t regData, uint8_t shift)
{
    return Pmic_getBitField(regData, shift, (uint8_t)(1U << shift)) == 1;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_COMMON_H__ */
