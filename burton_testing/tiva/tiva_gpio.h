#ifndef TIVA_GPIO_H
#define TIVA_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct gpioPinHandle_s
{
    uint32_t sysPeriphGPIO;
    uint32_t gpioPortBase;
    uint32_t gpioPin;
    uint32_t gpioPinDir;
    uint32_t gpioShiftVal;
}gpioPinHandle_t;

/**
 * \brief Initializes an array of GPIO pin handles
 *
 * \param gpioPinHandle     [OUT]    Reference to array of handles to GPIO pins
 * \param bOutput           [IN]     This parameter set as true will configure every handle to be handles to output
 *                                   pins. Otherwise, it will configure every handle to be handles to input pins
 */
void initializeGpioPinHandles(gpioPinHandle_t *gpioPinHandle, bool bOutput);

/**
 * \brief Initialize all GPIO for use as input pins
 *
 * \param gpioPinHandle     [IN]    array of handles used to configure Tiva GPIO pins as input/output
 */
void initializeGpioPins(gpioPinHandle_t *gpioPinHandle);

/**
 * \brief Initialize a GPIO for use as an input or output pin. Whether GPIO is input or output
 *        is dependent on the gpioPinDir struct member of gpioPinHandle parameter
 *
 * \param gpioPinHandle     [IN]    Handle to a GPIO pin
 */
void initializeGpioPin(const gpioPinHandle_t gpioPinHandle);

/**
 * \brief Initialize a GPIO handle for use as a handle to an MCU pin interfacing the PMIC nERR_MCU pin.
 *        Target pin will be PC6 on the Tiva.
 *
 * \param gpioPinHandle     [OUT]    GPIO pin handle to initialize
 */
void initializeEsmGpioOutputHandle(gpioPinHandle_t *gpioPinHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_GPIO_H */
