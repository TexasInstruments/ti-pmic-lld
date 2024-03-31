/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
 * \file common_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Source file for common test application APIs
 * \version 1.0
 */
#include <stdint.h>
#include <stdlib.h>

#include "pmic.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

#include "common_test.h"

/**
 * \brief Helper function to reset a GPIO configuration struct with all valid parameters set.
 *        Driver APIs will consider a struct member as invalid if its corresponding validParams is not set.
 *
 * \param pGpioCfg  [OUT]   GPIO configuration struct to reset with valid parameters
 */
void resetGpioCfg_withAllValidParams(Pmic_GpioCfg_t *pGpioCfg)
{
    pGpioCfg->validParams = PMIC_GPIO_CFG_DIR_VALID_SHIFT | PMIC_GPIO_CFG_OD_VALID_SHIFT |
                            PMIC_GPIO_CFG_PULL_VALID_SHIFT | PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT |
                            PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT;
    pGpioCfg->pinDir = 0;
    pGpioCfg->outputSignalType = 0;
    pGpioCfg->pullCtrl = 0;
    pGpioCfg->deglitchEnable = 0;
    pGpioCfg->pinFunc = 0;
    pGpioCfg->pinPolarity = 0;
}

/**
 * \brief Helper function to reset a GPIO configuration struct with specific valid parameters set.
 *        Driver APIs will consider a struct member as invalid if its corresponding validParams is not set.
 *
 * \param pGpioCfg      [OUT]   GPIO configuration struct to reset with valid parameters
 * \param validParams   [IN]    Indication of the valid values contained within pGpioCfg
 */
void resetGpioCfg_withSpecificValidParams(Pmic_GpioCfg_t *pGpioCfg, uint8_t validParams)
{
    pGpioCfg->validParams = validParams;
    pGpioCfg->pinDir = 0;
    pGpioCfg->outputSignalType = 0;
    pGpioCfg->pullCtrl = 0;
    pGpioCfg->deglitchEnable = 0;
    pGpioCfg->pinFunc = 0;
    pGpioCfg->pinPolarity = 0;
}

/**
 * \brief Unity uses this API in all its write, put, or print APIs
 *
 * \param ucData    [IN]    Character to write to the terminal
 */
void unityCharPut(uint8_t ucData)
{
    UARTCharPut(UART0_BASE, ucData);
    if (ucData == '\n')
    {
        UARTCharPut(UART0_BASE, '\r');
    }
}

static void resetAllTps6522xBuckRegisters(Pmic_CoreHandle_t pmicHandle)
{
    uint8_t i = 0;
    uint8_t regData = 0;
    const uint8_t intBuckRegAddr = 0x5B;

    // Set values of all BUCK registers to be zero
    (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[0].buckRailSelRegAddr, &regData, 1);
    for (i = 0; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[i].buckCtrlRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[i].buckConfRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[i].buckVoutRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[i].buckPgWindowRegAddr, &regData, 1);
    }

    // Clear BUCK interrupts
    (void)pmicI2CRead(&pmicHandle, PMIC_MAIN_INST, intBuckRegAddr, &regData, 1);
    regData |= 0xFU;
    (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, intBuckRegAddr, &regData, 1);
}

static void resetAllTps6522xLdoRegisters(Pmic_CoreHandle_t pmicHandle)
{
    uint8_t i = 0;
    uint8_t regData = 0;
    const uint8_t intLdoVmonRegAddr = 0x5F;

    // Set values of all LDO registers to be zero
    (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xLdoRegisters[0].ldoRailSelRegAddr, &regData, 1);
    for (i = 0; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xLdoRegisters[i].ldoCtrlRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xLdoRegisters[i].ldoVoutRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xLdoRegisters[i].ldoPgWindowRegAddr, &regData, 1);
    }

    // Clear LDO interrupts
    (void)pmicI2CRead(&pmicHandle, PMIC_MAIN_INST, intLdoVmonRegAddr, &regData, 1);
    regData |= 0b111;
    (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, intLdoVmonRegAddr, &regData, 1);
}

static void resetAllTps6522xVccaVmonRegisters(Pmic_CoreHandle_t pmicHandle)
{
    uint8_t i = 0;
    uint8_t regData = 0;
    const uint8_t intLdoVmonRegAddr = 0x5F;

    // Set values of all VCCA_MON/VMONx registers to be zero
    (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[0].vccaVmonCtrlRegAddr, &regData, 1);
    (void)pmicI2CWrite(
        &pmicHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[0].vccaVmonRailSelRegAddr, &regData, 1);
    for (i = 0; i < TPS6522X_MAX_VOLTAGE_MONITOR_NUM; i++)
    {
        switch (i)
        {
            case TPS6522X_VOLTAGE_MONITOR_VMON1:
            case TPS6522X_VOLTAGE_MONITOR_VMON2:
                (void)pmicI2CWrite(
                    &pmicHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[i].vmonPgLevelRegAddr, &regData, 1);
                (void)pmicI2CWrite(
                    &pmicHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[i].vmonPgWindowRegAddr, &regData, 1);

                break;
            case TPS6522X_VOLTAGE_MONITOR_VCCA_VMON:
                (void)pmicI2CWrite(
                    &pmicHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[i].vccaPgWindowRegAddr, &regData, 1);

                break;
            // Invalid VCCA_VMON/VMON
            default:
                break;
        }
    }

    // Clear VMON interrupts
    (void)pmicI2CRead(&pmicHandle, PMIC_MAIN_INST, intLdoVmonRegAddr, &regData, 1);
    regData |= (0b111 << 4);
    (void)pmicI2CWrite(&pmicHandle, PMIC_MAIN_INST, intLdoVmonRegAddr, &regData, 1);
}

void disablePmicPowerResources(Pmic_CoreHandle_t pmicHandle)
{
    const uint8_t devIdReg = 0x01;
    uint8_t pmicDevId = 0x00;
    int32_t status = PMIC_ST_SUCCESS;

    if (pmicHandle.pmicDeviceType == PMIC_DEV_BURTON_TPS6522X)
    {
        // Wait for PMIC connection
        while ((bool)true)
        {
            status = pmicI2CRead(&pmicHandle, PMIC_MAIN_INST, devIdReg, &pmicDevId, 1);

            if (status == PMIC_ST_SUCCESS)
            {
                break;
            }
        }

        // Set all BUCK registers to zero
        resetAllTps6522xBuckRegisters(pmicHandle);

        // Set all LDO registers to zero
        resetAllTps6522xLdoRegisters(pmicHandle);

        // Set all VCCA_VMON/VMONx registers to zero
        resetAllTps6522xVccaVmonRegisters(pmicHandle);
    }
}
