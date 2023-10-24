/* Top level include file */

#ifndef TIVA_TEST_SUITE_H_
#define TIVA_TEST_SUITE_H_

#include "tiva_vcp.h"
#include "tiva_i2c.h"
#include "tiva_misc.h"
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

/* Status codes definitions */
#define INVALID_INPUT_PARAM           (int32_t)(-2)
#define FAILURE                       (int32_t)(-1)
#define SUCCESS                       (0U)

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_TEST_SUITE_H_ */
