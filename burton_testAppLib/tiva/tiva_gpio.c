#include "tiva_priv.h"
#include "tiva_gpio.h"
#include "burton_tests/gpio_test.h"

/**
 * \brief Initializes an array of GPIO pin handles for use as GPIO input pin handles
 *
 * \param gpioPinHandle     [OUT]    Reference to array of handles to GPIO pins
 *
 */
void initializeGpioInputPinHandles(gpioPinHandle_t **gpioPinHandle)
{
    static gpioPinHandle_t gpioPinHandle_internal[6];

    gpioPinHandle_internal[0].sysPeriphGPIO = SYSCTL_PERIPH_GPIOE;
    gpioPinHandle_internal[0].gpioPortBase  = GPIO_PORTE_BASE;
    gpioPinHandle_internal[0].gpioPin       = GPIO_PIN_0;
    gpioPinHandle_internal[0].gpioPinDir    = GPIO_DIR_MODE_IN;
    gpioPinHandle_internal[0].gpioShiftVal  = 0;

    gpioPinHandle_internal[1].sysPeriphGPIO = SYSCTL_PERIPH_GPIOB;
    gpioPinHandle_internal[1].gpioPortBase  = GPIO_PORTB_BASE;
    gpioPinHandle_internal[1].gpioPin       = GPIO_PIN_7;
    gpioPinHandle_internal[1].gpioPinDir    = GPIO_DIR_MODE_IN;
    gpioPinHandle_internal[1].gpioShiftVal  = 7;

    gpioPinHandle_internal[2].sysPeriphGPIO = SYSCTL_PERIPH_GPIOB;
    gpioPinHandle_internal[2].gpioPortBase  = GPIO_PORTB_BASE;
    gpioPinHandle_internal[2].gpioPin       = GPIO_PIN_6;
    gpioPinHandle_internal[2].gpioPinDir    = GPIO_DIR_MODE_IN;
    gpioPinHandle_internal[2].gpioShiftVal  = 6;

    gpioPinHandle_internal[3].sysPeriphGPIO = SYSCTL_PERIPH_GPIOC;
    gpioPinHandle_internal[3].gpioPortBase  = GPIO_PORTC_BASE;
    gpioPinHandle_internal[3].gpioPin       = GPIO_PIN_4;
    gpioPinHandle_internal[3].gpioPinDir    = GPIO_DIR_MODE_IN;
    gpioPinHandle_internal[3].gpioShiftVal  = 4;

    gpioPinHandle_internal[4].sysPeriphGPIO = SYSCTL_PERIPH_GPIOC;
    gpioPinHandle_internal[4].gpioPortBase  = GPIO_PORTC_BASE;
    gpioPinHandle_internal[4].gpioPin       = GPIO_PIN_5;
    gpioPinHandle_internal[4].gpioPinDir    = GPIO_DIR_MODE_IN;
    gpioPinHandle_internal[4].gpioShiftVal  = 5;

    gpioPinHandle_internal[5].sysPeriphGPIO = SYSCTL_PERIPH_GPIOC;
    gpioPinHandle_internal[5].gpioPortBase  = GPIO_PORTC_BASE;
    gpioPinHandle_internal[5].gpioPin       = GPIO_PIN_6;
    gpioPinHandle_internal[5].gpioPinDir    = GPIO_DIR_MODE_IN;
    gpioPinHandle_internal[5].gpioShiftVal  = 6;

    *gpioPinHandle = (gpioPinHandle_t*)(&gpioPinHandle_internal);
}

/**
 * \brief Initialize all GPIO for use as input pins
 *
 * \param gpioPinHandle  [IN]   array of handles used to configure Tiva GPIO pins as input/output
 *
 */
void initializeGpioPins(gpioPinHandle_t *gpioPinHandle)
{
    uint8_t pin = 0;

    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        SysCtlPeripheralEnable(gpioPinHandle[pin - 1].sysPeriphGPIO);

        while (!SysCtlPeripheralReady(gpioPinHandle[pin - 1].sysPeriphGPIO))
        {
        }

        if (gpioPinHandle[pin - 1].gpioPinDir == GPIO_DIR_MODE_IN)
        {
            GPIOPinTypeGPIOInput(gpioPinHandle[pin - 1].gpioPortBase, gpioPinHandle[pin - 1].gpioPin);
        }
        else if (gpioPinHandle[pin - 1].gpioPinDir == GPIO_DIR_MODE_OUT)
        {
            GPIOPinTypeGPIOInput(gpioPinHandle[pin - 1].gpioPortBase, gpioPinHandle[pin - 1].gpioPin);
        }
    }
}
