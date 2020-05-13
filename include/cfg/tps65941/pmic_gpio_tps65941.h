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
 *  \ingroup DRV_PMIC_GPIO_MODULE
 *  \defgroup DRV_PMIC_GPIO_LEO_MODULE PMIC GPIO TPS65941 Leo Driver API
 *            These are PMIC GPIO driver params and API for TPS65941 Leo PMIC
 *
 *  @{
 */

/**
 * \file   pmic_gpio_tps65941.h
 *
 * \brief  PMIC Low Level Driver API/interface file for TPS65941 Leo PMIC GPIO
 *         APIs
 *
 */

#ifndef PMIC_GPIO_TPS65941_H_
#define PMIC_GPIO_TPS65941_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*!
 * \brief  Macros for each member of structure Pmic_GpioCfg_s
 */
/*! Valid only for NPWRON pin */
#define PMIC_NPWRON_CFG_POLARITY_VALID          (0x05U)
#define PMIC_NPWRON_CFG_POLARITY_VALID_SHIFT    (1U << \
                                                PMIC_NPWRON_CFG_POLARITY_VALID)

/**
 *  \anchor Pmic_GpioPin
 *  \name   PMIC GPIO supported pins
 *
 *  @{
 */
#define PMIC_NPWRON_PIN                            (0U)
#define PMIC_GPIO1_PIN                             (1U)
#define PMIC_GPIO2_PIN                             (2U)
#define PMIC_GPIO3_PIN                             (3U)
#define PMIC_GPIO4_PIN                             (4U)
#define PMIC_GPIO5_PIN                             (5U)
#define PMIC_GPIO6_PIN                             (6U)
#define PMIC_GPIO7_PIN                             (7U)
#define PMIC_GPIO8_PIN                             (8U)
#define PMIC_GPIO9_PIN                             (9U)
#define PMIC_GPIO10_PIN                            (10U)
#define PMIC_GPIO11_PIN                            (11U)
/*  @} */

/**
 *  \anchor Pmic_GpioPinFunc
 *  \name   PMIC GPIO pin functions supported
 *
 *  @{
 */
#define PMIC_NPWRON_PINFUNC_ENABLE                 (0U)
#define PMIC_NPWRON_PINFUNC_NPWRON                 (1U)
#define PMIC_NPWRON_PINFUNC_NONE                   (2U)
#define PMIC_GPIO_PINFUNC_GPIO                     (0U)
#define PMIC_GPIO_PINFUNC_SCL_I2C2_CS_SPI          (1U)
#define PMIC_GPIO_PINFUNC_NRSTOUT_SOC              (2U)
#define PMIC_GPIO_PINFUNC_TRIG_WDOG                (1U)
#define PMIC_GPIO_PINFUNC_SDA_I2C2_SDO_SPI         (2U)
#define PMIC_GPIO_PINFUNC_CLK32KOUT                (1U)
#define PMIC_GPIO_PINFUNC_NERR_SOC                 (2U)
#define PMIC_GPIO_PINFUNC_SCLK_SPMI                (1U)
#define PMIC_GPIO_PINFUNC_SDATA_SPMI               (1U)
#define PMIC_GPIO_PINFUNC_NERR_MCU                 (1U)
#define PMIC_GPIO_PINFUNC_GPIO8_GPIO10_SYNCCLKOUT  (2U)
#define PMIC_GPIO_PINFUNC_GPIO8_DISABLE_WDOG       (3U)
#define PMIC_GPIO_PINFUNC_PGOOD                    (1U)
#define PMIC_GPIO_PINFUNC_GPIO9_DISABLE_WDOG       (2U)
#define PMIC_GPIO_PINFUNC_GPIO9_SYNCCLKOUT         (3U)
#define PMIC_GPIO_PINFUNC_SYNCCLKIN                (1U)
#define PMIC_GPIO_PINFUNC_GPIO10_CLK32KOUT         (3U)
#define PMIC_GPIO_PINFUNC_NSLEEP1                  (4U)
#define PMIC_GPIO_PINFUNC_NSLEEP2                  (5U)
#define PMIC_GPIO_PINFUNC_WKUP1                    (6U)
#define PMIC_GPIO_PINFUNC_WKUP2                    (7U)
#define PMIC_GPIO_PINFUNC_LP_WKUP1                 (6U)
#define PMIC_GPIO_PINFUNC_LP_WKUP2                 (7U)
#define PMIC_GPIO_PINFUNC_MAX                      (8U)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_GPIO_TPS65941_H_ */

/* @} */
