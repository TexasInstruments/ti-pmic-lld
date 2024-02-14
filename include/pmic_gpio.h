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
 * \file   pmic_gpio.h
 *
 * \brief: This file contains macro definitions, structures and function
 *         prototypes for driver specific PMIC gpio configuration
 */

#ifndef PMIC_GPIO_H_
#define PMIC_GPIO_H_

#include "pmic_core.h"
#include "pmic_types.h"

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
 * \brief   PMIC GPIO Pins with Input Ouput Configuration
 *
 * \param   regAddr          GPIO Pin Register Address
 * \param   outRegAddr       GPIO OUT Register Address
 * \param   inRegAddr        GPIO IN Register Address
 * \param   inRegBitPos      Bit position of GPIO IN Register bit position
 * \param   outRegBitPos     Bit position of GPIO OUT Register bit position
 */
typedef struct Pmic_GpioInOutCfg_s {
  uint8_t regAddr;
  uint8_t outRegAddr;
  uint8_t inRegAddr;
  uint8_t inRegBitPos;
  uint8_t outRegBitPos;
} Pmic_GpioInOutCfg_t;

typedef struct Pmic_GpioCfg_s {
  uint8_t validParams;
  uint8_t pinDir; // for EN_OUT in GPO_Cfg2
  uint8_t outputSignalType;
  uint8_t pullCtrl;
  uint8_t deglitchEnable;
  uint8_t pinFunc;
  uint8_t pinPolarity;

  union {
    struct {
      uint8_t gpo1Cfg; // Configuration for GPO1
      uint8_t gpo2Cfg; // Configuration for GPO2
    } gpoCfg1;
    struct {
      uint8_t gpo3Cfg; // Configuration for GPO3
      uint8_t gpo4Cfg; // Configuration for GPO4
    } gpoCfg2;
    uint8_t gpi1Cfg;
    uint8_t gpi4Cfg;
  };
} Pmic_GpioCfg_t;

typedef struct Pmic_GpioRdbkDglCfg_s {
  uint8_t validParams;
  uint8_t gpo1FDglConfig;
  uint8_t gpo1RDglConfig;
  uint8_t gpo2FDglConfig;
  uint8_t gpo2RDglConfig;
  uint8_t gpo3FDglConfig;
  uint8_t gpo3RDglConfig;
  uint8_t gpo4FDglConfig;
  uint8_t gpo4RDglConfig;
  uint8_t gpo1FDglData;
  uint8_t gpo1RDglData;
  uint8_t gpo2FDglData;
  uint8_t gpo2RDglData;
  uint8_t gpo3FDglData;
  uint8_t gpo3RDglData;
  uint8_t gpo4FDglData;
  uint8_t gpo4RDglData;

} Pmic_GpioRdbkDglCfg_t;

#define PMIC_LOW 0U
#define PMIC_HIGH 1U

/* Pins for GPO1, GPO2, GPO3, GPO4 */
#define PMIC_GPO1 0x01U
#define PMIC_GPO2 0x02U
#define PMIC_GPO3 0x03U
#define PMIC_GPO4 0x04U
#define PMIC_GPI1 0x05U
#define PMIC_GPI4 0x06U

#define PMIC_GPIO_CFG_PULL_VALID (0x02U)
#define PMIC_GPIO_CFG_DEGLITCH_VALID (0x03U)

/* MACROS for GPO_CFG1_GPO1 */
#define LOW_LEVEL 0U
#define HIGH_LEVEL 1U
#define N_EN 2U
#define nINT 3U
#define N_EN_HIGH_Z_1 4U
#define N_EN_HIGH_Z_2 5U
#define N_EN_HIGH_Z_3 6U
#define RO_CNTR 7U

/* MACROS for GPO_CFG1_GPO2*/
#define LOW_LEVEL 0U
#define HIGH_LEVEL 1U
#define N_EN 2U
#define COMP1_OUT 3U
#define EN_OUT2 4U
#define N_EN_HIGH_Z_4 5U
#define N_EN_HIGH_Z_5 6U
#define RO_CNTR 7U

/* MACROS for EN_OUT GPO_CFG2 */
#define PULL_UP_VDDIO 0U
#define PULL_UP_LDO_IN 1U
#define INTL_PULL_UP 2U

/* MACROS for GPO_CFG2_GPO3 */
#define LOW_LEVEL 0U
#define HIGH_LEVEL 1U
#define N_EN 2U
#define SAFE_OUT2 3U
#define N_EN_HIGH_Z_6 4U
#define N_EN_HIGH_Z_7 5U
#define N_EN_HIGH_Z_8 6U
#define RO_CNTR 7U

/* MACROS for GPO_CFG2_GPO4 */
#define LOW_LEVEL 0U
#define HIGH_LEVEL 1U
#define N_EN 2U
#define PGOOD 3U
#define COMP2_OUT 4U
#define N_EN_HIGH_Z_9 5U
#define N_EN_HIGH_Z_10 6U
#define RO_CNTR 7U

/* MACROS for GPI_CFG_GPI1 */
#define COMP1_IN 0U
#define WD_IN 1U
#define COS_N 2U

/* MACROS for GPI_CFG_GPI4 */
#define ESM_IN 0U
#define WD_IN 1U

int32_t Pmic_gpiSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t pin,
                                 const Pmic_GpioCfg_t gpioCfg);

int32_t Pmic_gpiGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t pin, Pmic_GpioCfg_t *pGpioCfg);

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

int32_t Pmic_gpioSetPinFunc(Pmic_CoreHandle_t *pPmicCoreHandle,
                            const uint8_t pin, const Pmic_GpioCfg_t gpioCfg);

int32_t
Pmic_gpioSetNPwronEnableDeglitchPullCtrlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            const Pmic_GpioCfg_t gpioCfg);

int32_t Pmic_gpioSetPinPolarity(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const Pmic_GpioCfg_t gpioCfg);

int32_t Pmic_gpioGetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t pin, Pmic_GpioCfg_t *pGpioCfg);

int32_t Pmic_gpioGetPinFunc(Pmic_CoreHandle_t *pPmicCoreHandle,
                            const uint8_t pin, Pmic_GpioCfg_t *pGpioCfg);

int32_t Pmic_gpioGetPullCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t pin, Pmic_GpioCfg_t *pGpioCfg);

int32_t Pmic_gpioGetPinPolarity(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_GpioCfg_t *pGpioCfg);

int32_t Pmic_gpo12SetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

int32_t Pmic_gpo12GetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

int32_t Pmic_gpo34SetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

int32_t Pmic_gpo34GetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

void Pmic_get_gpioInOutCfg(const Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_GpioInOutCfg_t *pGpioInOutCfg);

int32_t Pmic_gpioSetValue(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t pin,
                          const uint8_t pinValue);

int32_t Pmic_gpioGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const uint8_t pin, Pmic_GpioCfg_t *pGpioCfg,
                                  Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg);

int32_t Pmic_gpioSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const uint8_t pin,
                                  const Pmic_GpioCfg_t gpioCfg);

#endif /* PMIC_GPIO_H_ */
