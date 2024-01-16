/**
 * \file irq_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton interrupt Unity tests
 * \version 1.0
 * \date 2023-10-17
 *
 * \copyright Copyright (c) 2023
 */
#ifndef IRQ_TEST_H
#define IRQ_TEST_H

#include "common_test.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define INT_GPIO_REG_ADDR (0x63U)

#define INTERRUPT_CLEARED (0U)
#define INTERRUPT_RAISED  (1U)

/**
 * \brief test GPIO1 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 1
 */
void test_GPIO1_interrupt_enableDisable(void);

/**
 * \brief test GPIO2 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 2
 */
void test_GPIO2_interrupt_enableDisable(void);

/**
 * \brief test GPIO3 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 3
 */
void test_GPIO3_interrupt_enableDisable(void);

/**
 * \brief test GPIO4 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 4
 */
void test_GPIO4_interrupt_enableDisable(void);

/**
 * \brief test GPIO5 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 5
 */
void test_GPIO5_interrupt_enableDisable(void);

/**
 * \brief test GPIO6 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 6
 */
void test_GPIO6_interrupt_enableDisable(void);

/**
 * \brief test GPIO1 Falling Edge Interrupt: GPIO 1 falling edge interrupt test
 */
void test_GPIO1_interrupt_fallingEdge(void);

/**
 * \brief test GPIO1 Rise Edge Interrupt: GPIO 1 rising edge interrupt test
 */
void test_GPIO1_interrupt_risingEdge(void);

/**
 * \brief test GPIO2 Falling Edge Interrupt: GPIO 2 falling edge interrupt test
 */
void test_GPIO2_interrupt_fallingEdge(void);

/**
 * \brief test GPIO2 Rise Edge Interrupt: GPIO 2 rising edge interrupt test
 */
void test_GPIO2_interrupt_risingEdge(void);

/**
 * \brief test GPIO3 Falling Edge Interrupt: GPIO 3 falling edge interrupt test
 */
void test_GPIO3_interrupt_fallingEdge(void);

/**
 * \brief test GPIO3 Rise Edge Interrupt: GPIO 3 rising edge interrupt test
 */
void test_GPIO3_interrupt_risingEdge(void);

/**
 * \brief test GPIO4 Falling Edge Interrupt: GPIO 4 falling edge interrupt test
 */
void test_GPIO4_interrupt_fallingEdge(void);

/**
 * \brief test GPIO4 Rise Edge Interrupt: GPIO 4 rising edge interrupt test
 */
void test_GPIO4_interrupt_risingEdge(void);

/**
 * \brief test GPIO5 Falling Edge Interrupt: GPIO 5 falling edge interrupt test
 */
void test_GPIO5_interrupt_fallingEdge(void);

/**
 * \brief test GPIO5 Rise Edge Interrupt: GPIO 5 rising edge interrupt test
 */
void test_GPIO5_interrupt_risingEdge(void);

/**
 * \brief test GPIO6 Falling Edge Interrupt: GPIO 6 falling edge interrupt test
 */
void test_GPIO6_interrupt_fallingEdge(void);

/**
 * \brief test GPIO6 Rise Edge Interrupt: GPIO 6 rising edge interrupt test
 */
void test_GPIO6_interrupt_risingEdge(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* IRQ_TEST_H */
