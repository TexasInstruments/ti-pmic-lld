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
 *  \addtogroup DRV_PMIC_IRQ_MODULE
 *
 *  @{
 */

/**
 * \file   pmic_irq_tps6522x.h
 *
 * \brief  TPS6522x Burton PMIC IRQ Driver API/interface file.
 *
 */

#ifndef PMIC_IRQ_TPS6522X_H_
#define PMIC_IRQ_TPS6522X_H_

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
 *  \anchor Pmic_tps6522x_IrqGpioNum
 *  \name PMIC GPIO Interrupt Mask values for tps6522x.
 *
 *  @{
 */
#define PMIC_TPS6522X_IRQ_GPIO_1_INT_MASK_NUM (0U)
#define PMIC_TPS6522X_IRQ_GPIO_2_INT_MASK_NUM (1U)
#define PMIC_TPS6522X_IRQ_GPIO_3_INT_MASK_NUM (2U)
#define PMIC_TPS6522X_IRQ_GPIO_4_INT_MASK_NUM (3U)
#define PMIC_TPS6522X_IRQ_GPIO_5_INT_MASK_NUM (4U)
#define PMIC_TPS6522X_IRQ_GPIO_6_INT_MASK_NUM (5U)

/* @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_IRQ_TPS6522X_H_ */

/* @} */
