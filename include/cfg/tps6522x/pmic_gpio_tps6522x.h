/******************************************************************************
 * Copyright (c) 2023 Texas Instruments Incorporated - http://www.ti.com
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
 *  \addtogroup DRV_PMIC_GPIO_MODULE
 *
 *  @{
 */

/**
 * \file   pmic_gpio_tps6522x.h
 *
 * \brief  PMIC TPS6522x BURTON PMIC GPIO API/interface file.
 *
 */

#ifndef PMIC_GPIO_TPS6522X_H_
#define PMIC_GPIO_TPS6522X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_Tps6522xBurton_GpioPin
 *  \name   PMIC GPIO supported pins for TPS6522x Burton Device
 *
 *  @{
 */
#define PMIC_TPS6522X_GPIO1_PIN                           (1U)
#define PMIC_TPS6522X_GPIO2_PIN                           (2U)
#define PMIC_TPS6522X_GPIO3_PIN                           (3U)
#define PMIC_TPS6522X_GPIO4_PIN                           (4U)
#define PMIC_TPS6522X_GPIO5_PIN                           (5U)
#define PMIC_TPS6522X_GPIO6_PIN                           (6U)
/** @} */

/**
 *  \anchor Pmic_Tps6522xBurton_GpioPinFunc
 *  \name   PMIC GPIO pin functions supported by TPS6522X BURTON PMIC
 *
 *  @{
 */
/** \brief Used to configure GPIO Pin Function.
 *         Valid for all GPIO Pins */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO                   (0U)
/** \brief Used to configure SDA_I2C2/SDO_SPI Function.
 *         Valid only for GPIO1 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO1_SDA_I2C2_SDO_SPI (1U)
/** \brief Used to configure nINT Function.
 *         Valid only for GPIO1 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO1_NINT             (2U)
/** \brief Used to configure nSLEEP2 Function.
 *         Valid only for GPIO1 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO1_NSLEEP2          (3U)
/** \brief Used to configure SCL_I2C2/CS_SPI Function.
 *         Valid only for GPIO2 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO2_SCL_I2C2_CS_SPI  (1U)
/** \brief Used to configure TRIG_WDOG Function.
 *         Valid only for GPIO2 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO2_TRIG_WDOG        (2U)
/** \brief Used to configure nSLEEP1 Function.
 *         Valid only for GPIO2 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO2_NSLEEP1          (3U)
/** \brief Used to configure VMON1_m Function.
 *         Valid only for GPIO3 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO3_VMON1_M          (1U)
/** \brief Used to configure Push-Button Function.
 *         Valid only for GPIO3 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO3_PUSH_BUTTON      (2U)
/** \brief Used to configure nSLEEP1 Function.
 *         Valid only for GPIO3 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO3_NSLEEP1          (3U)
/** \brief Used to configure VMON2 Function.
 *         Valid only for GPIO4 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO4_VMON2            (1U)
/** \brief Used to configure nSLEEP1 Function.
 *         Valid only for GPIO4 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO4_NSLEEP1          (2U)
/** \brief Used to configure ADC_IN Function.
 *         Valid only for GPIO4 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO4_ADC_IN           (3U)
/** \brief Used to configure WKUP Function.
 *         Valid only for GPIO5 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO5_WKUP             (1U)
/** \brief Used to configure SYNCCLKIN Function.
 *         Valid only for GPIO5 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO5_SYNCCLKIN        (2U)
/** \brief Used to configure ADC_IN Function.
 *         Valid only for GPIO5 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO5_ADC_IN           (3U)
/** \brief Used to configure NSLEEP2 Function.
 *         Valid only for GPIO6 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO6_NSLEEP2          (1U)
/** \brief Used to configure nERR_MCU Function.
 *         Valid only for GPIO6 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO6_NERR_MCU         (2U)
/** \brief Used to configure WKUP Function.
 *         Valid only for GPIO6 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO6_WKUP             (3U)
/** \brief Used to configure SYNCCLKIN Function.
 *         Valid only for GPIO6 Pin */
#define PMIC_TPS6522X_GPIO_PINFUNC_GPIO6_SYNCCLKIN        (4U)
/** @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_GPIO_TPS6522X_H_ */

/** @} */
