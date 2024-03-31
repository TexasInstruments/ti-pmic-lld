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
#include "tiva_priv.h"
#include "tiva_pwm.h"

void initializePwmHandle(pwmHandle_t *pwmHandle)
{
    pwmHandle->sysPeriphPWM      = SYSCTL_PERIPH_PWM1;
    pwmHandle->sysPeriphGPIO     = SYSCTL_PERIPH_GPIOF;
    pwmHandle->pwmGpioPortBase   = GPIO_PORTF_BASE;
    pwmHandle->pwmGpioPin        = GPIO_PIN_1;
    pwmHandle->pwmBase           = PWM1_BASE;
    pwmHandle->pwmGen            = PWM_GEN_2;
    pwmHandle->pwmOutNum         = PWM_OUT_5;
    pwmHandle->pwmOutBit         = PWM_OUT_5_BIT;
    pwmHandle->pwmSysCtlDiv      = SYSCTL_PWMDIV_4; // Divide sys clk by 4
    pwmHandle->gpioPinToPWM      = GPIO_PF1_M1PWM5;
    pwmHandle->period_cycles     = (((SysCtlClockGet() / 4) / 1000000) * 4000); // 250 Hz
    pwmHandle->pulseWidth_cycles = (pwmHandle->period_cycles) / 2; // 50% duty cycle
}

void initializePwmPin(const pwmHandle_t pwmHandle)
{
    // Enable and provide a clock to PWM module
    SysCtlPeripheralEnable(pwmHandle.sysPeriphPWM);

    // Configure the PWM clock
    SysCtlPWMClockSet(pwmHandle.pwmSysCtlDiv);

    // Enable and provide a clock to PWM pin GPIO port
    SysCtlPeripheralEnable(pwmHandle.sysPeriphGPIO);

    // Ensure GPIO port is ready to be configured
    while (!SysCtlPeripheralReady(pwmHandle.sysPeriphGPIO))
    {
    }

    // Configure GPIO pin to PWM functionality
    GPIOPinConfigure(pwmHandle.gpioPinToPWM);

    // Configure GPIO pin for PWM operation
    GPIOPinTypePWM(pwmHandle.pwmGpioPortBase, pwmHandle.pwmGpioPin);

    // Ensure PWM module ready to be configured
    while (!SysCtlPeripheralReady(pwmHandle.sysPeriphPWM))
    {
    }

    // Disable PWM output and generator block before configuration of PWM
    PWMOutputState(pwmHandle.pwmBase, pwmHandle.pwmOutBit, false);
    PWMGenDisable(pwmHandle.pwmBase, pwmHandle.pwmGen);

    // Configure PWM0 to count down without synchronization
    PWMGenConfigure(pwmHandle.pwmBase, pwmHandle.pwmGen, (PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC));

    // Set the PWM period
    PWMGenPeriodSet(pwmHandle.pwmBase, pwmHandle.pwmGen, pwmHandle.period_cycles);

    // Set the pulse width of the pin
    PWMPulseWidthSet(pwmHandle.pwmBase, pwmHandle.pwmOutNum, pwmHandle.pulseWidth_cycles);

    // Enable the PWM generator block
    PWMGenEnable(pwmHandle.pwmBase, pwmHandle.pwmGen);
}
