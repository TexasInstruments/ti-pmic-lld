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
#ifndef TIVA_PWM_H_
#define TIVA_PWM_H_

#include "driverlib/pwm.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct pwmHandle_s
{
    uint32_t sysPeriphPWM;
    uint32_t sysPeriphGPIO;
    uint32_t pwmGpioPortBase;
    uint8_t pwmGpioPin;
    uint32_t pwmBase;
    uint32_t pwmGen;
    uint32_t pwmOutNum;
    uint32_t pwmOutBit;
    uint32_t pwmSysCtlDiv;
    uint32_t gpioPinToPWM;
    uint32_t period_cycles;
    uint32_t pulseWidth_cycles;
} pwmHandle_t;

/**
 * \brief Function to initialize a handle to PWM output M0PWM3
 *
 * \param pwmHandle [OUT] PWM pin to initialize
 */
void initializePwmHandle(pwmHandle_t *pwmHandle);

/**
 * \brief Function to initialize a PWM pin given a PWM handle
 *
 * \param pwmHandle [IN] Handle used to initialze PWM pin
 */
void initializePwmPin(const pwmHandle_t pwmHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_PWM_H_ */
