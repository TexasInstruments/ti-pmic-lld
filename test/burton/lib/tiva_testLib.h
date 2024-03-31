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
#ifndef TIVA_TEST_SUITE_H_
#define TIVA_TEST_SUITE_H_

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "tiva_vcp.h"
#include "tiva_i2c.h"
#include "tiva_pmic_intf.h"
#include "tiva_timer.h"
#include "tiva_gpio.h"
#include "tiva_pwm.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Burton I2C addresses */
#define BURTON_I2C_USER_PAGE_ADDRESS  (uint8_t)(0x48) // Page 0
#define BURTON_I2C_NVM_PAGE_ADDRESS   (uint8_t)(0x49)
#define BURTON_I2C_WDG_PAGE_ADDRESS   (uint8_t)(0x12)

/* Burton internal register addresses */
#define BURTON_DEVICE_ID_REG_ADDR     (uint8_t)(0x01)
#define BURTON_REGISTER_LOCK_REG_ADDR (uint8_t)(0xA1)

/* Burton register lock key */
#define BURTON_REGISTER_LOCK_KEY      (uint8_t)(0x9B)

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_TEST_SUITE_H_ */
