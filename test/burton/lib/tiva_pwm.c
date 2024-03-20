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
