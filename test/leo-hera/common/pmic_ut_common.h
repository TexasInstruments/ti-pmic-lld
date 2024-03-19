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
#define J721E_LEO_PMICA_PAGE1_SLAVE_ADDR      (0x49U)
#define J721E_LEO_PMICA_WDG_SLAVE_ADDR        (0x12U)

#define J721E_LEO_PMICB_SLAVE_ADDR            (0x4CU)
#define J721E_LEO_PMICB_PAGE1_SLAVE_ADDR      (0x4DU)
#define J721E_LEO_PMICB_WDG_SLAVE_ADDR        (0x13U)

/*!
 * \brief   Slave addresses of J7VCL LEO PMIC-A and PMIC-B devices(Wakeup SoMs)
 */
#define J7VCL_WAKEUP_SOM_LEO_PMICA_SLAVE_ADDR            (0x48U)
#define J7VCL_WAKEUP_SOM_LEO_PMICA_PAGE1_SLAVE_ADDR      (0x49U)
#define J7VCL_WAKEUP_SOM_LEO_PMICA_WDG_SLAVE_ADDR        (0x12U)

#define J7VCL_WAKEUP_SOM_LEO_PMICB_SLAVE_ADDR            (0x4CU)
#define J7VCL_WAKEUP_SOM_LEO_PMICB_PAGE1_SLAVE_ADDR      (0x4DU)
#define J7VCL_WAKEUP_SOM_LEO_PMICB_WDG_SLAVE_ADDR        (0x13U)

/*!
 * \brief   Slave addresses of J7VCL HERA PMIC devices
 */
#define J7VCL_LEO_PMICA_SLAVE_ADDR            (0x48U)
#define J7VCL_LEO_PMICA_PAGE1_SLAVE_ADDR      (0x49U)
#define J7VCL_LEO_PMICA_WDG_SLAVE_ADDR        (0x12U)

#define J7VCL_HERA_PMIC_SLAVE_ADDR            (0x4CU)
#define J7VCL_HERA_PMIC_PAGE1_SLAVE_ADDR      (0x4DU)
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

/*!
 * \brief   CRC Status
 */
#define PMIC_STATUS_CRC_INIT_VAL            (0x0U)
#define PMIC_CFG_TO_ENABLE_CRC              (0x1U)
#define PMIC_STATUS_CRC_ENABLED             (0x2U)

extern uint8_t startup_type;

/*!
 * \brief   PMIC UT test status
 */
#define PMIC_UT_SUCCESS            (true)
#define PMIC_UT_FAILURE            (false)

/*!
 * \brief   PMIC UT test Options
 */
#define PMIC_UT_AUTOMATE_OPTION          (0U)
#define PMIC_UT_MANUAL_OPTION            (1U)

#define pmic_log              (UART_printf)

/*!
 * \brief   J7ES: use WAKEUP GPIO0_9
 */
#define J7_WAKEUP_GPIO0_PORT_NUM         0U  /* use WAKEUP GPIO0 */
#define J7_WAKEUP_GPIO0_9_PIN_NUM        9U  /* Pin 9 for J721E PMIC Intr */
#define J7_WAKEUP_GPIO0_84_PIN_NUM       84U /* Pin 84 for J7VCL PMIC Intr */

/*!
 * \brief   J7ES: use WAKEUP GPIO0_9
 */
#define PMIC_TEST_RESULT_FAIL            0U /* For Failed tests */
#define PMIC_TEST_RESULT_PASS            1U /* For Passed tests */
#define PMIC_TEST_RESULT_IGNORE          2U /* For Ignored tests */

/*!
 * \brief   PMIC FSM NSLEEP TRIGGER Register Address
 */
#define PMIC_UT_FSM_NSLEEP_TRIGGERS_REGADDR  (0x86U)

/*!
 * \brief   PMIC FSM I2C TRIGGER Register Address
 */
#define PMIC_UT_FSM_I2C_TRIGGERS_REGADDR     (0x85U)

/*!
 * \brief  PMIC Watch Dog Register Offsets
 */
#define PMIC_UT_WD_ERR_STATUS_REGADDR        (0x8U)
#define PMIC_UT_WD_FAIL_CNT_REG_REGADDR      (0xAU)

/*!
 * \brief  LDO voltage selection Register Address
 */
#define PMIC_UT_LDO3_VOUT_REGADDR            (0x25U)

/*!
 * \brief  PMIC Power status Register Address
 */
#define PMIC_UT_STAT_MISC_REGADDR            (0x74U)
#define PMIC_UT_STAT_MODERATE_ERR_REGADDR    (0x75U)
#define PMIC_UT_STAT_SEVERE_ERR_REGADDR      (0x76U)

/*!
 * \brief  PMIC Power status Register Address
 */
#define PMIC_UT_STAT_BUCK1_2_REGADDR         (0x6DU)

/*!
 * \brief  INT_STARTUP Sources
 */
#define PMIC_UT_RTC_STATUS_REGADDR           (0xC4U)

/*!
 * \brief: PMIC Recovery Counter Control and Status Registers
 */
#define PMIC_UT_RECOV_CNT_REG_2_REGADDR      (0x84U)

/*!
 * \brief  PMIC ENABLE_DRV_STAT register Addresses
 */
#define PMIC_UT_ENABLE_DRV_STAT_REGADDR      (0x82U)


/*!
 * \brief  Interrupt Hierarchy Level 0 Register offsets
 */
#define PMIC_UT_INT_TOP_REGADDR              (0x5AU)

/*!
 *  \brief  PMIC Interrupt Hierarchy Level 1 Register offsets
 */
#define PMIC_UT_INT_BUCK_REGADDR             (0x5BU)
#define PMIC_UT_INT_GPIO_REGADDR             (0x63U)
#define PMIC_UT_INT_STARTUP_REGADDR          (0x65U)
#define PMIC_UT_INT_MISC_REGADDR             (0x66U)
#define PMIC_UT_INT_MODERATE_ERR_REGADDR     (0x67U)
#define PMIC_UT_INT_SEVERE_ERR_REGADDR       (0x68U)
#define PMIC_UT_INT_FSM_ERR_REGADDR          (0x69U)

/*!
 *  \brief  PMIC Interrupt Hierarchy Level 1 Register offsets.
 */
#define PMIC_UT_INT_LDO_VMON_REGADDR         (0x5FU)

/** Interrupt Hierarchy Level 2 Register offsets */
/*!
 * \brief  INT_BUCK Sources
 */
#define PMIC_UT_INT_BUCK5_REGADDR            (0x5EU)
#define PMIC_UT_INT_BUCK1_2_REGADDR          (0x5CU)
#define PMIC_UT_INT_BUCK3_4_REGADDR          (0x5DU)

/*!
 * \brief  INT_GPIO Sources
 */
#define PMIC_UT_INT_GPIO1_8_REGADDR          (0x64U)

/*!
 * \brief  INT_LDO_VMON Sources
 */
#define PMIC_UT_INT_VMON_REGADDR             (0x62U)

/*!
 * \brief  Interrupt MASK registers address
 */
#define PMIC_UT_INT_LDO1_2_REGADDR           (0x60U)
#define PMIC_UT_INT_LDO3_4_REGADDR           (0x61U)

/*!
 * \brief  INT_FSM sources
 */
#define PMIC_UT_INT_COMM_ERR_REGADDR         (0x6AU)
#define PMIC_UT_INT_READBACK_ERR_REGADDR     (0x6BU)
#define PMIC_UT_INT_ESM_REGADDR              (0x6CU)

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
    uint8_t   testResult;
    bool      testValid;
    uint8_t   finalTestResult;
    bool      finalTestValid;
} Pmic_Ut_Tests_t;

/*!
 *  \brief    PMIC Fault Injection Test flags structure
 *
 *  \param   enableFaultInjectionRead       Flag to enable/disable Fault
 *                                          Injection in PMIC Register Read.
 *  \param   enableFaultInjectionWrite      Flag to enable/disable Fault
 *                                          Injection in PMIC Register Write.
 *  \param   readCount                      Flag to store current the Read
 *                                          Count value.
 *  \param   writeCount                     Flag to store current the Write
 *                                          Count value.
 *  \param   skipReadCount                  Count to skip fault injection for
 *                                          read.
 *  \param   skipWriteCount                 Count to skip fault injection for
 *                                          write.
 *  \param   commError                      Flag to return SPI/I2C
 *                                          communication error.
 */
typedef struct Pmic_Ut_FaultInject_s
{
    uint8_t enableFaultInjectionRead;
    uint8_t enableFaultInjectionWrite;
    uint8_t readCount;
    uint8_t writeCount;
    uint8_t skipReadCount;
    uint8_t skipWriteCount;
    int32_t commError;
} Pmic_Ut_FaultInject_t;

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

/*!
 * \brief   Wrapper to update ignore infofrmation and Ignore the test
 */
void pmic_testResultUpdate_ignore(uint32_t testId,
                                  Pmic_Ut_Tests_t *pTest,
                                  uint32_t num_testcases);

/*!
 * \brief   Wrapper to update pass infofrmation
 */
void pmic_testResultUpdate_pass(uint32_t testId,
                                Pmic_Ut_Tests_t *pTest,
                                uint32_t num_testcases);

/*!
 * \brief   initialize test results with default values
 */
void pmic_testResult_init(Pmic_Ut_Tests_t *pTest, uint32_t num_testcases);

/*!
 * \brief   Update final test results
 */
void pmic_updateTestResults(Pmic_Ut_Tests_t *pTest, uint32_t num_testcases);

/*!
 * \brief   print all tests resuluts in required pattern
 */
void pmic_printTestResult(Pmic_Ut_Tests_t *pTest, uint32_t num_testcases);

/*!
 * \brief    : Prints time taken for a given Valid string and returns delta.
 */
uint64_t print_timeTakenInUsecs(uint64_t t1, const char *str);

/*!
 * \brief   print Banner Message
 */
void pmic_print_banner(const char *str);

#if defined(SOC_J721E)
/*!
 * \brief  To Enable Crystal Oscillator for RTC on J721E PG2.0
 */
void test_pmic_rtc_setCfg_xtalOScEnType(Pmic_CoreHandle_t  *pPmicCorehandle);
#endif
