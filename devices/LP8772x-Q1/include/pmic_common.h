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
#ifndef PMIC_COMMON_H
#define PMIC_COMMON_H

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/*                               Macros & Typedefs                          */
/*==========================================================================*/

#define COUNT(x)  (sizeof(x) / sizeof(x[0]))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

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
 * @param drvInitStat Driver initialization status. Used by LLD as a measure to
 * prevent corrupted handle usage.
 *
 * @param devId PMIC device type.
 *
 * @param pmicDevRev PMIC device revision.
 *
 * @param devSiRev PMIC device silicon revision.
 *
 * @param commMode Communication mode of the PMIC. Some PMICs may only have one
 * communication mode while others could have multiple (e.g., single I2C, dual I2C,
 * SPI).
 *
 * @param i2cAddr0 Main PMIC device address.
 *
 * @param i2cAddr1 Address for interacting with PMIC WDG Q&A.
 *
 * @param i2cAddr2 Address for interacting with PMIC NVM space.
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
 * @param commHandle0 Pointer to serial communication handle for the PMIC device.
 *
 * @param commHandle1 Pointer to serial communication handle for PMIC WDG.
 *
 * @param ioRead Function pointer to platform-specific serial communication
 * read API.
 *
 * @param ioWrite Function pointer to platform-specific serial communication
 * write API.
 *
 * @param criticalSectionStart Function pointer to OS-specific critical section start.
 *
 * @param criticalSectionStop Function pointer to OS-specific critical section stop.
 *
 * @param irqResponseCallback Function pointer to application-specific IRQ response
 * when an IRQ is detected during WDG servicing.
 */
typedef struct Pmic_CoreHandle_s {
    const Pmic_DevSubSysInfo_t *pPmic_SubSysInfo;
    uint32_t drvInitStat;
    uint8_t devId;
    uint8_t pmicDevRev;
    uint8_t devSiRev;
    uint8_t commMode;
    uint8_t i2cAddr0;
    uint8_t i2cAddr1;
    uint8_t i2cAddr2;
    uint8_t i2c1Speed;
    uint8_t i2c2Speed;
    bool crcEnable;
    bool configCrcEnable;
    void *commHandle0;
    void *commHandle1;
    int32_t (*ioRead)(const struct Pmic_CoreHandle_s *pmicCorehandle,
                      uint8_t instType, uint16_t regAddr,
                      uint8_t *pRxBuf, uint8_t bufLen);
    int32_t (*ioWrite)(const struct Pmic_CoreHandle_s *pmicCorehandle,
                       uint8_t instType, uint16_t regAddr,
                       uint8_t *pTxBuf, uint8_t bufLen);
    void (*criticalSectionStart)(void);
    void (*criticalSectionStop)(void);
    void (*irqResponseCallback)(void);
} Pmic_Handle_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @brief Checks whether a parameter is valid.
 *
 * Design: PMICDRV-571
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-521, PMICDRV-519
 *               PMICDRV-520
 *
 * @param validParamVal [IN] Valid parameter value. Each bit in this parameter
 * represents whether or not a parameter is valid.
 *
 * @param bitPos [IN] Valid parameter to check for.
 *
 * @return True if parameter is valid, false otherwise.
 */
bool Pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos);

/**
 * @brief Checks whether a parameter is valid and whether the status code is equal
 * to LLD success code.
 *
 * Design: PMICDRV-572
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-521, PMICDRV-519
 *               PMICDRV-520
 *
 * @param validParamVal [IN] Valid parameter value. Each bit in this parameter
 * represents whether or not a parameter is valid.
 *
 * @param bitPos [IN] Valid parameter value to check for.
 *
 * @param status [IN] API checks whether this parameter is equivalent to the
 * PMIC LLD success code.
 *
 * @return True if parameter is valid and status is equal to LLD success code,
 * false otherwise.
 */
bool Pmic_validParamStatusCheck(uint32_t validParamVal, uint8_t bitPos, int32_t status);

/**
 * @brief Start a critical section when usage of a shared resource such as an I2C or
 * SPI bus is required.
 *
 * Design: PMICDRV-573
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-502, PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *               PMICDRV-517, PMICDRV-505, PMICDRV-509
 *
 * @param handle Pointer to the PMIC core handle structure.
 */
void Pmic_criticalSectionStart(const Pmic_Handle_t *handle);

/**
 * @brief Stop a critical section after the usage of a shared resource such as an
 * I2C or SPI bus is complete.
 *
 * Design: PMICDRV-574
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-502, PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *               PMICDRV-517, PMICDRV-505, PMICDRV-509
 *
 * @param handle Pointer to the PMIC core handle structure.
 */
void Pmic_criticalSectionStop(const Pmic_Handle_t *handle);

/**
 * @brief Indicate via callback function that an INT event has been detected on
 * the PMIC.
 *
 * Design: PMICDRV-732
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-537, PMICDRV-521, PMICDRV-517
 *
 * @param handle Pointer to the PMIC core handle structure.
 */
void Pmic_irqResponseCallback(const Pmic_Handle_t *handle);

/**
 * @brief Sets the bit field of an 8-bit unsigned integer to the desired value.
 *
 * Design: PMICDRV-575
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-504, PMICDRV-522, PMICDRV-521
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
 * @brief Sets the bit field of an 8-bit unsigned integer to the desired boolean
 * value.
 *
 * Design: PMICDRV-577
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *
 * @param regData [OUT] The API modifies the desired bit field of the value held
 * at this address.
 *
 * @param shift [IN] Bit field position.
 *
 * @param value [IN] Desired bit field value to set (either true or false).
 */
static inline void Pmic_setBitField_b(uint8_t *regData, uint8_t shift, bool value)
{
    Pmic_setBitField(regData, shift, (uint8_t)(1U << shift), value ? 1U : 0U);
}

/**
 * @brief Get desired bit field of an 8-bit unsigned integer.
 *
 * Design: PMICDRV-578
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-504, PMICDRV-522, PMICDRV-521
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
 * @brief Gets the desired bit field of an 8-bit unsigned integer, casted as a
 * boolean.
 *
 * Design: PMICDRV-580
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *
 * @param regData [IN] The API gets the desired bit field from this value.
 *
 * @param shift [IN] Bit field location.
 *
 * @return Value of the desired bit field (either true or false).
 */
static inline bool Pmic_getBitField_b(uint8_t regData, uint8_t shift)
{
    return Pmic_getBitField(regData, shift, (uint8_t)(1U << shift)) == 1U;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_COMMON_H */
