/**
 * \file common_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Source file for common test application APIs
 * \version 1.0
 * \date 2023-10-12
 *
 * \copyright Copyright (c) 2023
 */
#include <stdio.h>
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
void unityCharPut(unsigned char ucData)
{
    UARTCharPut(UART0_BASE, ucData);
    if (ucData == '\n')
    {
        UARTCharPut(UART0_BASE, '\r');
    }
}

static void resetAllTps6522xBuckRegisters(Pmic_CoreHandle_t pmicCoreHandle)
{
    uint8_t       i = 0;
    uint8_t       regData = 0;
    const uint8_t intBuckRegAddr = 0x5B;

    // Set values of all BUCK registers to be zero
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[0].buckRailSelRegAddr, &regData, 1);
    for (i = 0; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[i].buckCtrlRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[i].buckConfRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[i].buckVoutRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xBuckRegisters[i].buckPgWindowRegAddr, &regData, 1);
    }

    // Clear BUCK interrupts
    (void)pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, intBuckRegAddr, &regData, 1);
    regData |= 0xF;
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, intBuckRegAddr, &regData, 1);
}

static void resetAllTps6522xLdoRegisters(Pmic_CoreHandle_t pmicCoreHandle)
{
    uint8_t       i = 0;
    uint8_t       regData = 0;
    const uint8_t intLdoVmonRegAddr = 0x5F;

    // Set values of all LDO registers to be zero
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xLdoRegisters[0].ldoRailSelRegAddr, &regData, 1);
    for (i = 0; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xLdoRegisters[i].ldoCtrlRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xLdoRegisters[i].ldoVoutRegAddr, &regData, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xLdoRegisters[i].ldoPgWindowRegAddr, &regData, 1);
    }

    // Clear LDO interrupts
    (void)pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, intLdoVmonRegAddr, &regData, 1);
    regData |= 0b111;
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, intLdoVmonRegAddr, &regData, 1);
}

static void resetAllTps6522xVccaVmonRegisters(Pmic_CoreHandle_t pmicCoreHandle)
{
    uint8_t       i = 0;
    uint8_t       regData = 0;
    const uint8_t intLdoVmonRegAddr = 0x5F;

    // Set values of all VCCA_MON/VMONx registers to be zero
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[0].vccaVmonCtrlRegAddr, &regData, 1);
    (void)pmicI2CWrite(
        &pmicCoreHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[0].vccaVmonRailSelRegAddr, &regData, 1);
    for (i = 0; i < TPS6522X_MAX_VOLTAGE_MONITOR_NUM; i++)
    {
        switch (i)
        {
            case TPS6522X_VOLTAGE_MONITOR_VMON1:
            case TPS6522X_VOLTAGE_MONITOR_VMON2:
                (void)pmicI2CWrite(
                    &pmicCoreHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[i].vmonPgLevelRegAddr, &regData, 1);
                (void)pmicI2CWrite(
                    &pmicCoreHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[i].vmonPgWindowRegAddr, &regData, 1);

                break;
            case TPS6522X_VOLTAGE_MONITOR_VCCA_VMON:
                (void)pmicI2CWrite(
                    &pmicCoreHandle, PMIC_MAIN_INST, gTps6522xVccaVmonRegisters[i].vccaPgWindowRegAddr, &regData, 1);

                break;
        }
    }

    // Clear VMON interrupts
    (void)pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, intLdoVmonRegAddr, &regData, 1);
    regData |= (0b111 << 4);
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, intLdoVmonRegAddr, &regData, 1);
}

void disablePmicPowerResources(Pmic_CoreHandle_t pmicCoreHandle)
{
    const uint8_t devIdReg = 0x01;
    uint8_t       pmicDevId = 0x00;
    int32_t       status = PMIC_ST_SUCCESS;

    if (pmicCoreHandle.pmicDeviceType != PMIC_DEV_BURTON_TPS6522X)
    {
        return;
    }

    // Wait for PMIC connection
    while (1)
    {
        status = pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, devIdReg, &pmicDevId, 1);

        if (status == PMIC_ST_SUCCESS)
        {
            break;
        }
    }

    // Set all BUCK registers to zero
    resetAllTps6522xBuckRegisters(pmicCoreHandle);

    // Set all LDO registers to zero
    resetAllTps6522xLdoRegisters(pmicCoreHandle);

    // Set all VCCA_VMON/VMONx registers to zero
    resetAllTps6522xVccaVmonRegisters(pmicCoreHandle);
}
