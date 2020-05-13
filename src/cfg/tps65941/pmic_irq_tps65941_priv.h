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
 * \file   pmic_irq_tps65941_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         configuring PMIC IRQ
 */

#ifndef PMIC_IRQ_TPS65941_PRIV_H_
#define PMIC_IRQ_TPS65941_PRIV_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

#define PMIC_INT_GPIO11_MASK                   (0x04U)
#define PMIC_INT_NPWRON_START_MASK             (0x01U)
#define PMIC_INT_RTC_STATUS_TIMER_MASK         (0x20U)
#define PMIC_INT_RTC_STATUS_ALARM_MASK         (0x40U)
#define PMIC_INT_RTC_STATUS_POWER_UP_MASK      (0x80U)
#define PMIC_INT_NPWRON_LONG_MASK              (0x20U)

/*!
 * \brief: IRQ Mask Bits to validate error bits
 */
#define PMIC_INT_GPIO9_11_MASK                 (0x07U)
#define PMIC_INT_RTC_MASK                      (0x60U)

/*!
 * \brief: INT_STARTUP Sources
 */
#define PMIC_INT_RTC_STATUS_REGADDDR           (0xC4U)


/*!
 * \brief: Interrupt MASK registers address
 */
#define PMIC_IRQ_MASK_GPIO9_11_REGADDR        (0x51U)

/*!
 * \brief: MASK value for Interrupts
 */
#define PMIC_IRQ_MASK_GPIO9_11                (0x3FU)

/*!
 * \brief: Individual interrupt bitmasks for GPIO11 RISE/FALL
 */
#define PMIC_IRQ_GPIO11_RISE                  (0x6U)
#define PMIC_IRQ_GPIO11_FALL                  (0x3U)

/*!
 * \brief: Individual interrupt bitmasks for GPIO9_11 FALL
 */
#define PMIC_IRQ_NPWRON_START                 (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for STARTUP Error
 */
#define PMIC_IRQ_NPWRON_LONG                  (0x6U)

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_TPS65941_PRIV_H_ */
