/******************************************************************************
 * Copyright (c) 2023 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file pmic_mainpage.h
 *
 *  \brief This file contains the main page doxygen details for PMIC LLD
 */

/**
 *  \mainpage  PMIC LLD
 *
 *
 *  This is an API guide for PMIC (Power Management Integrated Circuit) Driver.
 *  The PMIC Driver provides a set of low level APIs that aim to establish the
 *  foundation for the end-user's software application with PMICs and jumpstart
 *  development/evaluation. The PMIC Driver is designed to be MCU-agnostic and
 *  can be used in baremetal or RTOS applications. Using the PMIC driver alongside
 *  a supported PMIC, the end-user can manage the power of and provide power to MCUs,
 *  SoCs, and/or different components on embedded systems.
 *
 *  The PMIC Driver supports below mentioned PMIC devices and their
 *  features or modules.
 *
 *  Supported PMIC Devices are:
 *    1. TPS6594x (Leo PMIC Device)
 *    2. LP8764x  (Hera PMIC Device)
 *    3. TPS6522x (Burton PMIC Device)
 *
 *  PMIC features that are accessible and configurable by the PMIC Driver are the
 *  following: PMIC communication (such as communicating with or without CRC),
 *  general purpose input/output (GPIO), real time clock (RTC), interrupt requests
 *  (IRQ), error state monitor (ESM), finite state machine (FSM), watchdog (WDG),
 *  analog-to-digital converter (ADC), power (including BUCKs, LDOs, VMONs, thermal),
 *  and miscellaneous functions (such as writing to scratchpad registers).
 *
 *  The PMIC-LLD includes the following sub-modules/drivers:
 *
 *   - <b> Common </b> (See \ref DRV_PMIC_COMMON_MODULE) <br>
 *   - <b> GPIO </b> (See \ref DRV_PMIC_GPIO_MODULE) <br>
 *   - <b> RTC </b> (See \ref DRV_PMIC_RTC_MODULE) <br>
 *   - <b> IRQ </b> (See \ref DRV_PMIC_IRQ_MODULE) <br>
 *   - <b> ESM </b> (See \ref DRV_PMIC_ESM_MODULE) <br>
 *   - <b> FSM </b> (See \ref DRV_PMIC_FSM_MODULE) <br>
 *   - <b> POWER </b> (See \ref DRV_PMIC_POWER_MODULE) <br>
 *   - <b> WDG </b> (See \ref DRV_PMIC_WDG_MODULE) <br>
 *   - <b> ADC </b> (See \ref DRV_PMIC_ADC_MODULE) <br>
 *
 *  All above PMIC features/modules can be accessed or configured by using PMIC Driver
 *  APIs documented within this guide.
 *
 */
