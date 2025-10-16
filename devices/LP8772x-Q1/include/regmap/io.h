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
#ifndef PMIC_REGMAP_IO_H
#define PMIC_REGMAP_IO_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PMIC I/O Related Register Offset */
#define PMIC_IO_INTERFACE_CONF (0x1DU)

/** @brief PMIC Interface Configuration Register (INTERFACE_CONF) */
#define PMIC_INTF_CONF_NINT_GPO_PU_EN_SHIFT (0U)
#define PMIC_INTF_CONF_NINT_GPO_POL_SHIFT   (1U)
#define PMIC_INTF_CONF_NINT_GPO_OD_SHIFT    (2U)
#define PMIC_INTF_CONF_NRSTOUT_PU_EN_SHIFT  (3U)
#define PMIC_INTF_CONF_NRSTOUT_POL_SHIFT    (4U)
#define PMIC_INTF_CONF_NRSTOUT_OD_SHIFT     (5U)
#define PMIC_INTF_CONF_NERR_PU_DIS_SHIFT    (6U)
#define PMIC_INTF_CONF_I2C_CRC_EN_SHIFT     (7U)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_REGMAP_IO_H */
