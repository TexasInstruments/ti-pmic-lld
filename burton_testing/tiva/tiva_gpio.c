#include "tiva_priv.h"
#include "tiva_gpio.h"
#include "gpio_test.h"

void initializeGpioPinHandles(gpioPinHandle_t *gpioPinHandle, bool bOutput)
{
    gpioPinHandle[0].sysPeriphGPIO = SYSCTL_PERIPH_GPIOE;
    gpioPinHandle[0].gpioPortBase = GPIO_PORTE_BASE;
    gpioPinHandle[0].gpioPin = GPIO_PIN_0;
    gpioPinHandle[0].gpioPinDir = (bOutput) ? GPIO_DIR_MODE_OUT : GPIO_DIR_MODE_IN;
    gpioPinHandle[0].gpioShiftVal = 0;

    gpioPinHandle[1].sysPeriphGPIO = SYSCTL_PERIPH_GPIOB;
    gpioPinHandle[1].gpioPortBase = GPIO_PORTB_BASE;
    gpioPinHandle[1].gpioPin = GPIO_PIN_7;
    gpioPinHandle[1].gpioPinDir = (bOutput) ? GPIO_DIR_MODE_OUT : GPIO_DIR_MODE_IN;
    gpioPinHandle[1].gpioShiftVal = 7;

    gpioPinHandle[2].sysPeriphGPIO = SYSCTL_PERIPH_GPIOB;
    gpioPinHandle[2].gpioPortBase = GPIO_PORTB_BASE;
    gpioPinHandle[2].gpioPin = GPIO_PIN_6;
    gpioPinHandle[2].gpioPinDir = (bOutput) ? GPIO_DIR_MODE_OUT : GPIO_DIR_MODE_IN;
    gpioPinHandle[2].gpioShiftVal = 6;

    gpioPinHandle[3].sysPeriphGPIO = SYSCTL_PERIPH_GPIOC;
    gpioPinHandle[3].gpioPortBase = GPIO_PORTC_BASE;
    gpioPinHandle[3].gpioPin = GPIO_PIN_4;
    gpioPinHandle[3].gpioPinDir = (bOutput) ? GPIO_DIR_MODE_OUT : GPIO_DIR_MODE_IN;
    gpioPinHandle[3].gpioShiftVal = 4;

    gpioPinHandle[4].sysPeriphGPIO = SYSCTL_PERIPH_GPIOC;
    gpioPinHandle[4].gpioPortBase = GPIO_PORTC_BASE;
    gpioPinHandle[4].gpioPin = GPIO_PIN_5;
    gpioPinHandle[4].gpioPinDir = (bOutput) ? GPIO_DIR_MODE_OUT : GPIO_DIR_MODE_IN;
    gpioPinHandle[4].gpioShiftVal = 5;

    gpioPinHandle[5].sysPeriphGPIO = SYSCTL_PERIPH_GPIOF;
    gpioPinHandle[5].gpioPortBase = GPIO_PORTF_BASE;
    gpioPinHandle[5].gpioPin = GPIO_PIN_1;
    gpioPinHandle[5].gpioPinDir = (bOutput) ? GPIO_DIR_MODE_OUT : GPIO_DIR_MODE_IN;
    gpioPinHandle[5].gpioShiftVal = 1;
}

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
            GPIOPinTypeGPIOOutput(gpioPinHandle[pin - 1].gpioPortBase, gpioPinHandle[pin - 1].gpioPin);
        }
    }
}

void initializeGpioPin(const gpioPinHandle_t gpioPinHandle)
{
    SysCtlPeripheralEnable(gpioPinHandle.sysPeriphGPIO);

    while (!SysCtlPeripheralReady(gpioPinHandle.sysPeriphGPIO))
    {
    }

    if (gpioPinHandle.gpioPinDir == GPIO_DIR_MODE_IN)
    {
        GPIOPinTypeGPIOInput(gpioPinHandle.gpioPortBase, gpioPinHandle.gpioPin);
    }
    else if (gpioPinHandle.gpioPinDir == GPIO_DIR_MODE_OUT)
    {
        GPIOPinTypeGPIOOutput(gpioPinHandle.gpioPortBase, gpioPinHandle.gpioPin);
    }
}

void initializeEsmGpioOutputHandle(gpioPinHandle_t *gpioPinHandle)
{
    gpioPinHandle->sysPeriphGPIO = SYSCTL_PERIPH_GPIOF;
    gpioPinHandle->gpioPortBase = GPIO_PORTF_BASE;
    gpioPinHandle->gpioPin = GPIO_PIN_1;
    gpioPinHandle->gpioPinDir = GPIO_DIR_MODE_OUT;
    gpioPinHandle->gpioShiftVal = 1;
}
