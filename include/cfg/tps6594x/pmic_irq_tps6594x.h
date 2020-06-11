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
 *  \defgroup DRV_PMIC_IRQ_LEO_MODULE PMIC IRQ TPS6594x LEO Driver API
 *            These are PMIC IRQ driver params and API for TPS6594x LEO PMIC
 *
 *  @{
 */

/**
 * \file   pmic_irq_tps6594x.h
 *
 * \brief  PMIC Low Level Driver API/interface file for TPS6594x LEO PMIC IRQ
 *         APIs
 *
 */

#ifndef PMIC_IRQ_TPS6594X_H_
#define PMIC_IRQ_TPS6594X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_IrqInterruptMask
 *  \name   Interrupts Masks specific for PMIC LEO TPS6594x
 *
 *  Application can use below shifted values to mask/unmask an interrupt
 *  Bitfield breakup:
 *         Bits 0-7   : Interrupt Mask register address.
 *         Bits 8-15  : Actual interrupt Mask.
 *
 *  @{
 */
#define PMIC_IRQ_MASK_GPIO9_11_MASK                                           \
                                    (((uint16_t)PMIC_IRQ_MASK_GPIO9_11 << 8U) \
                                            | PMIC_IRQ_MASK_GPIO9_11_REGADDR)
#define PMIC_TPS6594X_IRQ_GPIO11_RISE_MASK                                    \
                                    (((uint16_t)PMIC_IRQ_GPIO11_RISE   << 8U) \
                                            | PMIC_IRQ_MASK_GPIO9_11_REGADDR)
#define PMIC_TPS6594X_IRQ_GPIO10_RISE_MASK                                    \
                                    (((uint16_t)PMIC_TPS6594X_IRQ_GPIO10_RISE \
                                                                       << 8U) \
                                            | PMIC_IRQ_MASK_GPIO9_11_REGADDR)
#define PMIC_TPS6594X_IRQ_GPIO9_RISE_MASK                                     \
                                    (((uint16_t)PMIC_TPS6594X_IRQ_GPIO9_RISE  \
                                                                       << 8U) \
                                            | PMIC_IRQ_MASK_GPIO9_11_REGADDR)
#define PMIC_TPS6594X_IRQ_GPIO11_FALL_MASK                                    \
                                    (((uint16_t)PMIC_IRQ_GPIO11_FALL   << 8U) \
                                            | PMIC_IRQ_MASK_GPIO9_11_REGADDR)
#define PMIC_TPS6594X_IRQ_GPIO10_FALL_MASK                                    \
                                    (((uint16_t)PMIC_TPS6594X_IRQ_GPIO10_FALL \
                                                                       << 8U) \
                                            | PMIC_IRQ_MASK_GPIO9_11_REGADDR)
#define PMIC_TPS6594X_IRQ_GPIO9_FALL_MASK                                     \
                                    (((uint16_t)PMIC_TPS6594X_IRQ_GPIO9_FALL  \
                                                                       << 8U) \
                                            | PMIC_IRQ_MASK_GPIO9_11_REGADDR)
/* @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_IRQ_TPS6594X_H_ */

/* @} */
