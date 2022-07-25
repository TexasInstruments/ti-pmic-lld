#
# Copyright (c) 2020, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# File: pmic_component.mk
#       This file is component include make file of pmic library.
# List of variables set in this file and their purpose:
# <mod>_RELPATH        - This is the relative path of the module, typically from
#                        top-level directory of the package
# <mod>_PATH           - This is the absolute path of the module. It derives from
#                        absolute path of the top-level directory (set in env.mk)
#                        and relative path set above
# <mod>_INCLUDE        - This is the path that has interface header files of the
#                        module. This can be multiple directories (space separated)
# <mod>_PKG_LIST       - Names of the modules (and sub-modules) that are a part
#                        part of this module, including itself.
# <mod>_BOARD_DEPENDENCY - "yes": means the code for this module depends on
#                             board and the compiled obj/lib has to be kept
#                             under <board> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no board dependent code and hence
#                             the obj/libs are not kept under <board> dir.
# <mod>_CORE_DEPENDENCY     - "yes": means the code for this module depends on
#                             core and the compiled obj/lib has to be kept
#                             under <core> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no core dependent code and hence
#                             the obj/libs are not kept under <core> dir.
# <mod>_APP_STAGE_FILES     - List of source files that belongs to the module
#                             <mod>, but that needs to be compiled at application
#                             build stage (in the context of the app). This is
#                             primarily for link time configurations or if the
#                             source file is dependent on options/defines that are
#                             application dependent. This can be left blank or
#                             not defined at all, in which case, it means there
#                             no source files in the module <mod> that are required
#                             to be compiled in the application build stage.
#
ifeq ($(pmic_component_make_include), )

drvpmic_BOARDLIST           = j721e_evm j7200_evm j721s2_evm
drvpmic_SOCLIST             = j721e j7200 j721s2
drvpmic_j721e_CORELIST      = mcu1_0 mcu1_1 mcu2_0 mcu2_1 mcu3_0 mcu3_1 mpu1_0 c66xdsp_1 c66xdsp_2 c7x_1
drvpmic_j7200_CORELIST      = mcu1_0 mcu1_1 mcu2_0 mcu2_1 mpu1_0
drvpmic_j721s2_CORELIST     = mcu1_0
drvpmic_app_j721e_CORELIST  = mcu1_0 mcu1_1 mcu2_0 mcu2_1 mcu3_0 mcu3_1
drvpmic_app_j7200_CORELIST  = mcu1_0 mcu1_1 mcu2_0 mcu2_1
drvpmic_fsm_mcu_app_CORELIST  = mcu1_0 mcu1_1

############################
# pmic package
# List of components included under pmic lib
# The components included here are built and will be part of pmic lib
############################
pmic_LIB_LIST =

############################
# pmic examples
# List of examples under pmic
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
pmic_EXAMPLE_LIST =

#
# PMIC Modules
#

# PMIC library
pmic_COMP_LIST = pmic
pmic_RELPATH = ti/drv/pmic
pmic_PATH = $(PDK_PMIC_COMP_PATH)
export pmic_LIBNAME = pmic
export pmic_LIBPATH = $(PDK_PMIC_COMP_PATH)/lib
export pmic_MAKEFILE = -fsrc/makefile
export pmic_BOARD_DEPENDENCY = yes
export pmic_CORE_DEPENDENCY = no
pmic_PKG_LIST = pmic
pmic_INCLUDE = $(pmic_PATH)
export pmic_SOCLIST = $(drvpmic_SOCLIST)
export pmic_$(SOC)_CORELIST =  $(drvpmic_$(SOC)_CORELIST)
pmic_LIB_LIST += pmic

#
# Applications Specific scripts
#

#
# RTC  test
#
export pmic_rtc_testapp_COMP_LIST = pmic_rtc_testapp
pmic_rtc_testapp_RELPATH = ti/drv/pmic/test/rtc_test
pmic_rtc_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/rtc_test
export pmic_rtc_testapp_BOARD_DEPENDENCY = yes
export pmic_rtc_testapp_CORE_DEPENDENCY = no
export pmic_rtc_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_rtc_testapp_PKG_LIST = pmic_rtc_testapp
pmic_rtc_testapp_INCLUDE = $(pmic_rtc_testapp_PATH)
export pmic_rtc_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_rtc_testapp_$(SOC)_CORELIST = $(drvpmic_app_$(SOC)_CORELIST)
pmic_rtc_testapp_SBL_APPIMAGEGEN = yes
export pmic_rtc_testapp_SBL_APPIMAGEGEN

pmic_EXAMPLE_LIST += pmic_rtc_testapp

#
# GPIO  test
#
export pmic_gpio_testapp_COMP_LIST = pmic_gpio_testapp
pmic_gpio_testapp_RELPATH = ti/drv/pmic/test/gpio_test
pmic_gpio_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/gpio_test
export pmic_gpio_testapp_BOARD_DEPENDENCY = yes
export pmic_gpio_testapp_CORE_DEPENDENCY = no
export pmic_gpio_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_gpio_testapp_PKG_LIST = pmic_gpio_testapp
pmic_gpio_testapp_INCLUDE = $(pmic_gpio_testapp_PATH)
export pmic_gpio_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_gpio_testapp_$(SOC)_CORELIST = $(drvpmic_app_$(SOC)_CORELIST)
pmic_gpio_testapp_SBL_APPIMAGEGEN = yes
export pmic_gpio_testapp_SBL_APPIMAGEGEN

pmic_EXAMPLE_LIST += pmic_gpio_testapp

#
# POWER  test
#
export pmic_power_testapp_COMP_LIST = pmic_power_testapp
pmic_power_testapp_RELPATH = ti/drv/pmic/test/power_test
pmic_power_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/power_test
export pmic_power_testapp_BOARD_DEPENDENCY = yes
export pmic_power_testapp_CORE_DEPENDENCY = no
export pmic_power_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_power_testapp_PKG_LIST = pmic_power_testapp
pmic_power_testapp_INCLUDE = $(pmic_power_testapp_PATH)
export pmic_power_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_power_testapp_$(SOC)_CORELIST = $(drvpmic_app_$(SOC)_CORELIST)
pmic_power_testapp_SBL_APPIMAGEGEN = yes
export pmic_power_testapp_SBL_APPIMAGEGEN

pmic_EXAMPLE_LIST += pmic_power_testapp

#
# WDG  test
#
export pmic_wdg_testapp_COMP_LIST = pmic_wdg_testapp
pmic_wdg_testapp_RELPATH = ti/drv/pmic/test/wdg_test
pmic_wdg_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/wdg_test
export pmic_wdg_testapp_BOARD_DEPENDENCY = yes
export pmic_wdg_testapp_CORE_DEPENDENCY = no
export pmic_wdg_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_wdg_testapp_PKG_LIST = pmic_wdg_testapp
pmic_wdg_testapp_INCLUDE = $(pmic_wdg_testapp_PATH)
export pmic_wdg_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_wdg_testapp_$(SOC)_CORELIST = $(drvpmic_app_$(SOC)_CORELIST)
pmic_wdg_testapp_SBL_APPIMAGEGEN = yes
export pmic_wdg_testapp_SBL_APPIMAGEGEN

pmic_EXAMPLE_LIST += pmic_wdg_testapp

#
# MISC  test
#
export pmic_misc_testapp_COMP_LIST = pmic_misc_testapp
pmic_misc_testapp_RELPATH = ti/drv/pmic/test/misc_test
pmic_misc_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/misc_test
export pmic_misc_testapp_BOARD_DEPENDENCY = yes
export pmic_misc_testapp_CORE_DEPENDENCY = no
export pmic_misc_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_misc_testapp_PKG_LIST = pmic_misc_testapp
pmic_misc_testapp_INCLUDE = $(pmic_misc_testapp_PATH)
export pmic_misc_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_misc_testapp_$(SOC)_CORELIST = $(drvpmic_app_$(SOC)_CORELIST)
pmic_misc_testapp_SBL_APPIMAGEGEN = yes
export pmic_misc_testapp_SBL_APPIMAGEGEN

pmic_EXAMPLE_LIST += pmic_misc_testapp

#
# ESM  test
#
export pmic_esm_testapp_COMP_LIST = pmic_esm_testapp
pmic_esm_testapp_RELPATH = ti/drv/pmic/test/esm_test
pmic_esm_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/esm_test
export pmic_esm_testapp_BOARD_DEPENDENCY = yes
export pmic_esm_testapp_CORE_DEPENDENCY = no
export pmic_esm_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_esm_testapp_PKG_LIST = pmic_esm_testapp
pmic_esm_testapp_INCLUDE = $(pmic_esm_testapp_PATH)
export pmic_esm_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_esm_testapp_$(SOC)_CORELIST = $(drvpmic_app_$(SOC)_CORELIST)
pmic_esm_testapp_SBL_APPIMAGEGEN = yes
export pmic_esm_testapp_SBL_APPIMAGEGEN

pmic_EXAMPLE_LIST += pmic_esm_testapp

#
# fsm  test
#
export pmic_fsm_testapp_COMP_LIST = pmic_fsm_testapp
pmic_fsm_testapp_RELPATH = ti/drv/pmic/test/fsm_test
pmic_fsm_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/fsm_test
export pmic_fsm_testapp_BOARD_DEPENDENCY = yes
export pmic_fsm_testapp_CORE_DEPENDENCY = no
export pmic_fsm_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_fsm_testapp_PKG_LIST = pmic_fsm_testapp
pmic_fsm_testapp_INCLUDE = $(pmic_fsm_testapp_PATH)
export pmic_fsm_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_fsm_testapp_$(SOC)_CORELIST = $(drvpmic_app_$(SOC)_CORELIST)
pmic_fsm_testapp_SBL_APPIMAGEGEN = yes
export pmic_fsm_testapp_SBL_APPIMAGEGEN

pmic_EXAMPLE_LIST += pmic_fsm_testapp

#
# BenchMark  test
#
export pmic_benchmark_testapp_COMP_LIST = pmic_benchmark_testapp
pmic_benchmark_testapp_RELPATH = ti/drv/pmic/test/benchmark_test
pmic_benchmark_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/benchmark_test
export pmic_benchmark_testapp_BOARD_DEPENDENCY = yes
export pmic_benchmark_testapp_CORE_DEPENDENCY = no
export pmic_benchmark_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_benchmark_testapp_PKG_LIST = pmic_benchmark_testapp
pmic_benchmark_testapp_INCLUDE = $(pmic_benchmark_testapp_PATH)
export pmic_benchmark_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_benchmark_testapp_$(SOC)_CORELIST = $(drvpmic_app_$(SOC)_CORELIST)
pmic_benchmark_testapp_SBL_APPIMAGEGEN = yes
export pmic_benchmark_testapp_SBL_APPIMAGEGEN

pmic_EXAMPLE_LIST += pmic_benchmark_testapp

#
# Stress  test
#
export pmic_stress_testapp_COMP_LIST = pmic_stress_testapp
pmic_stress_testapp_RELPATH = ti/drv/pmic/test/stress_test
pmic_stress_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/stress_test
export pmic_stress_testapp_BOARD_DEPENDENCY = yes
export pmic_stress_testapp_CORE_DEPENDENCY = no
export pmic_stress_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_stress_testapp_PKG_LIST = pmic_stress_testapp
pmic_stress_testapp_INCLUDE = $(pmic_stress_testapp_PATH)
export pmic_stress_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_stress_testapp_$(SOC)_CORELIST = $(drvpmic_app_$(SOC)_CORELIST)
pmic_stress_testapp_SBL_APPIMAGEGEN = yes
export pmic_stress_testapp_SBL_APPIMAGEGEN

pmic_EXAMPLE_LIST += pmic_stress_testapp

#
# FSM MCU State  test
#
export pmic_fsm_mcu_testapp_COMP_LIST = pmic_fsm_mcu_testapp
pmic_fsm_mcu_testapp_RELPATH = ti/drv/pmic/test/fsm_mcu_test
pmic_fsm_mcu_testapp_PATH = $(PDK_PMIC_COMP_PATH)/test/fsm_mcu_test
export pmic_fsm_mcu_testapp_BOARD_DEPENDENCY = yes
export pmic_fsm_mcu_testapp_CORE_DEPENDENCY = no
export pmic_fsm_mcu_testapp_MAKEFILE = -f makefile IS_BAREMETAL=yes
pmic_fsm_mcu_testapp_PKG_LIST = pmic_fsm_mcu_testapp
pmic_fsm_mcu_testapp_INCLUDE = $(pmic_fsm_mcu_testapp_PATH)
export pmic_fsm_mcu_testapp_BOARDLIST = $(drvpmic_BOARDLIST)
export pmic_fsm_mcu_testapp_$(SOC)_CORELIST = $(drvpmic_fsm_mcu_app_CORELIST)
pmic_fsm_mcu_testapp_SBL_APPIMAGEGEN = yes
export pmic_fsm_mcu_testapp_SBL_APPIMAGEGEN

ifeq ($(BUILD_PROFILE),release)
#pmic_EXAMPLE_LIST += pmic_fsm_mcu_testapp
endif

#
# Export Libraries and Apps to Build Env
#
export pmic_LIB_LIST
export pmic_EXAMPLE_LIST
export drvpmic_LIB_LIST = $(pmic_LIB_LIST)
export drvpmic_EXAMPLE_LIST = $(pmic_EXAMPLE_LIST)

PMIC_CFLAGS = $(PDK_CFLAGS)
export PMIC_CFLAGS
pmic_component_make_include := 1
endif
