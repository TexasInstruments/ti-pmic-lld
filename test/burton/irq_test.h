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
/**
 * \file irq_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton interrupt Unity tests
 * \version 1.0
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
