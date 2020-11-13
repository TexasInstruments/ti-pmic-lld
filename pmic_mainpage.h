/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  This is an API guide for PMIC(Power Management Integrated Circuit) Driver.
 *  PMIC Driver is designed to power up different components on the embedded
 *  boards or provide supply to MCU(Micro Controller Unit) or
 *  SoC(System on chip) using APIs provided in the guide.
 *
 *  The PMIC Driver supports below mentioned PMIC devices and their
 *  features or Modules.
 *
 *  Supported PMIC Devices are:
 *    1. TPS6594x (Leo PMIC Device)
 *    2. LP8764x  (Hera PMIC Device)
 *
 *  Above PMICs have multiple functionalities and configurable features. Like,
 *  Real Time Clock (RTC) which provides Time, Calendar, Alarm and timer,
 *  Configurable GPIO pins to support wakeup, nSLEEP, PGOOD, nRESET for
 *  SOC/MCU, GPIOs with configurable PU/PD to enable other chips,
 *  Have number of BUCK(with different phases) and LDO regulators to provide
 *  supply to other modules on the board, Have the Voltage Monitor feature
 *  to monitor and notify for OV, UV, SC and Over Heat(Thermal Monitor).
 *  Have interrupt feature to notify severe, moderate, fsm Errors and provide
 *  asynchronous events for all supported features including GPIO External
 *  Interrupts, Have the WatchDog feature to monitor correct operation of the
 *  MCU using WDG trigger mode or using WDOG QA mode, Supports I2C(single and
 *  dual mode) and SPI communication protocols to access the registers for
 *  Read/Write operations with or without CRC.
 *
 *  The PMIC-LLD includes the following sub-modules/drivers
 *
 *   - <b> Common </b> (See \ref DRV_PMIC_COMMON_MODULE) <br>
 *   - <b> GPIO </b> (See \ref DRV_PMIC_GPIO_MODULE) <br>
 *   - <b> RTC </b> (See \ref DRV_PMIC_RTC_MODULE) <br>
 *   - <b> IRQ </b> (See \ref DRV_PMIC_IRQ_MODULE) <br>
 *   - <b> ESM </b> (See \ref DRV_PMIC_ESM_MODULE) <br>
 *   - <b> FSM </b> (See \ref DRV_PMIC_FSM_MODULE) <br>
 *   - <b> POWER </b> (See \ref DRV_PMIC_POWER_MODULE) <br>
 *   - <b> WDG </b> (See \ref DRV_PMIC_WDG_MODULE) <br>
 *
 *  All above PMICs features can be accessed or configured by using PMIC Driver
 *  APIs and illustrated in the guide.
 *
 */
