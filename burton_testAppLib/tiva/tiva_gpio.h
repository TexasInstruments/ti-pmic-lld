#ifndef TIVA_GPIO_H
#define TIVA_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

// clang-format off
typedef struct gpioPinHandle_s
{
    uint32_t sysPeriphGPIO;
    uint32_t gpioPortBase;
    uint32_t gpioPin;
    uint32_t gpioPinDir;
    uint32_t gpioShiftVal;
}gpioPinHandle_t;
// clang-format on

/**
 * \brief Initializes an array of GPIO pin handles for use as GPIO input pin handles
 *
 * \param gpioPinHandle     [OUT]    Reference to array of handles to GPIO pins
 *
 */
void initializeGpioInputPinHandles(gpioPinHandle_t **gpioPinHandle);

/**
 * \brief Initialize all GPIO for use as input pins
 *
 * \param gpioPinHandle  [IN]   array of handles used to configure Tiva GPIO pins as input/output
 *
 */
void initializeGpioPins(gpioPinHandle_t *gpioPinHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_GPIO_H */
