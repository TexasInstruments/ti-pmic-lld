#ifndef TIVA_PRIV_H_
#define TIVA_PRIV_H_

/**
 * \brief CCS already defines this but Visual Studio Code does not know and will
 *        not be able to recognize some defines specific to this part number
 */
#ifndef PART_TM4C123GH6PM
#define PART_TM4C123GH6PM
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"

#include "pmic_drv/pmic.h"

#include "tiva_testLib.h"

#include "unity/unity.h"

#endif /* TIVA_PRIV_H_ */
