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
