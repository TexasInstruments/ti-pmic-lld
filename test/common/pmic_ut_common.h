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
 *  \file   pmic_ut_common.h
 *
 *  \brief  Haeder file for unit test related common APIs
 *
 */

#include <stdio.h>
#include <string.h>
#include <pmic.h>


/* CSL Header files */
#include <ti/csl/csl_types.h>
#include <ti/csl/soc.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/csl_timer.h>
#include <ti/csl/csl_gpio.h>

/* OSAL Header files */
#include <ti/osal/osal.h>

/* Board Header files */
#include <ti/board/board.h>
#include <ti/board/board_cfg.h>
#include <ti/board/src/devices/common/common.h>

/* I2C Header files */
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>

/* GPIO Header Files */
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

/* Timer specific headers */
#if defined(SOC_J721E)
#include <ti/board/src/j721e_evm/include/board_utils.h>
#endif
#if defined(SOC_J7200)
#include <ti/board/src/j7200_evm/include/board_utils.h>
#endif

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
           (defined(SOC_J721E) || defined(SOC_J7200))
#include <ti/build/unit-test/Unity/src/unity.h>
#endif

/*!
 * \brief   Slave addresses of J721E LEO PMIC-A and PMIC-B devices
 */
#define J721E_LEO_PMICA_SLAVE_ADDR            (0x48U)
#define J721E_LEO_PMICA_WDG_SLAVE_ADDR        (0x12U)

#define J721E_LEO_PMICB_SLAVE_ADDR            (0x4CU)
#define J721E_LEO_PMICB_WDG_SLAVE_ADDR        (0x13U)

/*!
 * \brief   Slave addresses of J7VCL HERA PMIC devices
 */
#define J7VCL_LEO_PMICA_SLAVE_ADDR            (0x48U)
#define J7VCL_LEO_PMICA_WDG_SLAVE_ADDR        (0x12U)

#define J7VCL_HERA_PMIC_SLAVE_ADDR            (0x4CU)
#define J7VCL_HERA_PMIC_WDG_SLAVE_ADDR        (0x13U)

/*!
 * \brief  PMIC devices Definitions
 */
#define PMICA_DEVICE                    (0x01U)
#define PMICB_DEVICE                    (0x02U)
#define PMIC_DEVICE_MASK                (0xFFU)

/*!
 * \brief  Supported Board definition
 */
#define J721E_BOARD                     (0x01U << 8U)
#define J7VCL_BOARD                     (0x02U << 8U)

/*!
 * \brief   LEO PMIC devices on J721E
 */
#define J721E_LEO_PMICA_DEVICE          (J721E_BOARD + PMICA_DEVICE)
#define J721E_LEO_PMICB_DEVICE          (J721E_BOARD + PMICB_DEVICE)

/*!
 * \brief   PMIC devices on J7VCL EVM
 */
#define J7VCL_LEO_PMICA_DEVICE          (J7VCL_BOARD + PMICA_DEVICE)
#define J7VCL_HERA_PMICB_DEVICE         (J7VCL_BOARD + PMICB_DEVICE)

/*!
 * \brief   PMIC Startup type
 */
#define PMIC_ENABLE_STARTUP_TYPE        (0x01U)
#define PMIC_NPWRON_STARTUP_TYPE        (0x02U)
#define PMIC_FSD_STARTUP_TYPE           (0x03U)

extern uint8_t startup_type;

/*!
 * \brief   PMIC UT test status
 */
#define PMIC_UT_SUCCESS            (true)
#define PMIC_UT_FAILURE            (false)

#define pmic_log              (UART_printf)

/*!
 * \brief   J7ES: use WAKEUP GPIO0_9
 */
#define J7_WAKEUP_GPIO0_PORT_NUM         0U  /* use WAKEUP GPIO0 */
#define J7_WAKEUP_GPIO0_9_PIN_NUM        9U  /* Pin 9 for J721E PMIC Intr */
#define J7_WAKEUP_GPIO0_84_PIN_NUM       84U /* Pin 84 for J7VCL PMIC Intr */

/*!
 *  \brief    Define the Pmic UT test interface
 *
 *  \param   testId              Test ID
 *  \param   testDesc            Test Description
 *
 */
typedef struct Pmic_Ut_Tests_s
{
    uint32_t  testId;
    char      testDesc[140U];
} Pmic_Ut_Tests_t;

/*!
 * \brief   PMIC I2C read function.
 *
 * \param   pmicCorehandle [IN]     PMIC Interface Handle.
 * \param   commMode       [IN]     Communication Mode
 * \param   slaveAddr      [IN]     I2c device slave address
 * \param   regAddr        [OUT]    Register address to read from.
 * \param   pBuf           [IN]     Buffer to store data
 * \param   bufLen         [IN]     Length of data to read.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values see \ref Pmic_ErrorCodes
 */
int32_t test_pmic_regRead(Pmic_CoreHandle_t  *pmicCorehandle,
                          uint8_t             instType,
                          uint16_t            regAddr,
                          uint8_t            *pBuf,
                          uint8_t             bufLen);

                     /*!
 * \brief   PMIC I2C write function.
 *
 * \param   pmicCorehandle   [IN]     PMIC Interface Handle.
 * \param   commMode         [IN]     Communication Mode
 * \param   slaveAddr        [IN]     I2c device slave address
 * \param   regAddr          [IN]     Register address to write.
 * \param   pBuf             [IN]     Buffer to store data to write
 * \param   bufLen           [IN]     Length of data to write.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values see \ref Pmic_ErrorCodes
 */
int32_t test_pmic_regWrite(Pmic_CoreHandle_t  *pmicCorehandle,
                           uint8_t             instType,
                           uint16_t            regAddr,
                           uint8_t            *pBuf,
                           uint8_t             bufLen);

/*!
 * \brief   PMIC Critical section Lock function
 *          This function should have OS specific locking setup and should
 *          assigned to 'pmicConfigData.pFnPmicCritSecStart'
 */
void test_pmic_criticalSectionStartFn(void);

/*!
 * \brief   PMIC Critical section Unlock function
 *          This function should have OS specific locking setup and should
 *          assigned to 'pmicConfigData.pFnPmicCritSecStop'
 */
void test_pmic_criticalSectionStopFn(void);

/*!
 * \brief   Initialize PMIC Instance and corresponding Interface.
 *
 * \param   pmicCoreHandle    [OUT]     PMIC Core Handle.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int32_t test_pmic_appInit(Pmic_CoreHandle_t **pmicCoreHandle,
                          Pmic_CoreCfg_t     *pmicConfigData);

/*!
 * \brief   Deinitialize PMIC Instance and corresponding Interface.
 *
 * \param   pmicCoreHandle    [OUT]     PMIC Core Handle.
 */
void test_pmic_appDeInit(Pmic_CoreHandle_t *pmicCoreHandle);

/*!
 * \brief   UART Configurations
 */
void Board_initUART(void);

/*!
 * \brief   GPIO Configurations.
 *          This API is required for Asynchronous Interrupts only
 */
void App_initGPIO(GPIO_CallbackFxn callback);

/*!
 * \brief   Function to print testcase info
 */
void test_pmic_print_unity_testcase_info(uint32_t         testId,
                                         Pmic_Ut_Tests_t *pTest,
                                         uint32_t         num_testcases);

/*!
 * \brief   GPIO Interrupt Router Configuration
 */
void GPIO_configIntRouter(uint32_t portNum,
                          uint32_t pinNum,
                          uint32_t gpioIntRtrOutIntNum,
                          GPIO_v0_HwAttrs *cfg);
