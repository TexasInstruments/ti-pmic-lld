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
 *  \file   pmic_ut_power.c
 *
 *  \brief  PMIC Unit Test for testing PMIC POWER APIs
 *
 */

#include <pmic_ut_power.h>

/* Pointer holds the pPmicCoreHandle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

/*!
 * \brief   PMIC POWER Test Cases
 */
static Pmic_Ut_Tests_t pmic_power_tests[] =
{
    /*! testID
     *  testDesc
     */
    {
        0,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for handle"
    },
    {
        00,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for handle"
    },
    {
        1,
        "Pmic_powerSetPwrResourceCfg : Test Enable residual voltage checking"
    },
    {
        2,
        "Pmic_powerSetPwrResourceCfg : Test Disable residual voltage checking"
    },
    {
        3,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for rvCheckEn."
    },
    {
        4,
        "Pmic_powerSetPwrResourceCfg : Test Enable buck pull down checking for Buck regulator"
    },
    {
        5,
        "Pmic_powerSetPwrResourceCfg : Test Disable buck pull down checking for Buck regulator"
    },
    {
        6,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckPullDownEn."
    },
    {
        7,
        "Pmic_powerSetPwrResourceCfg : Test Enable the Voltage monitor."
    },
    {
        8,
        "Pmic_powerSetPwrResourceCfg : Test Disable the Voltage monitor."
    },
    {
        9,
        "Pmic_powerSetPwrResourceCfg :  Parameter validation for Power Resource for vmonEn."
    },
    {
        10,
        "Pmic_powerSetPwrResourceCfg : Test Select output voltage register for buck as VOUT1."
    },
    {
        11,
        "Pmic_powerSetPwrResourceCfg : Test Select output voltage register for buck as VOUT2."
    },
    {
        12,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckVoutSel."
    },
    {
        13,
        "Pmic_powerSetPwrResourceCfg : Test Select Auto Mode for BUCK"
    },
    {
        14,
        "Pmic_powerSetPwrResourceCfg : Test Select PWM Mode for BUCK"
    },
    {
        15,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckFpwmMode."
    },
    {
        16,
        "Pmic_powerSetPwrResourceCfg : Test Select Multi phase with PWM Mode for BUCK"
    },
    {
        17,
        "Pmic_powerSetPwrResourceCfg : Test Select AUTO Mode with Automatic phase adding and shedding for BUCK"
    },
    {
        18,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckFpwmMpMode."
    },
    {
        19,
        "Pmic_powerSetPwrResourceCfg : Test Disable the power regulator."
    },
    {
        20,
        "Pmic_powerSetPwrResourceCfg : Test Enable the power regulator."
    },
    {
        21,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for regulatorEn."
    },
    {
        22,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 6.5Amp."
    },
    {
        23,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 5.5Amp."
    },
    {
        24,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 4.5Amp."
    },
    {
        25,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 3.5Amp."
    },
    {
        26,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 2.5Amp."
    },
    {
        27,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckCurrentLimit."
    },
    {
        28,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for buckCurrentLimit."
    },
    {
        29,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 33MV."
    },
    {
        30,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 20MV."
    },
    {
        31,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 10MV."
    },
    {
        32,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 5MV."
    },
    {
        33,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 2.5MV."
    },
    {
        34,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 1.3MV."
    },
    {
        35,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 0.63MV."
    },
    {
        36,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 0.31MV."
    },
    {
        37,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckVmonSlewRate."
    },
    {
        38,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for buckVmonSlewRate."
    },
    {
        39,
        "Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 50Kohm."
    },
    {
        40,
        "Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 125Kohm"
    },
    {
        41,
        "Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 250Kohm"
    },
    {
        42,
        "Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 500Kohm"
    },
    {
        43,
        "Pmic_powerSetPwrResourceCfg : Test Enable Slow Ramp for LDO"
    },
    {
        44,
        "Pmic_powerSetPwrResourceCfg : Test Disable Slow Ramp for LDO"
    },
    {
        45,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoPullDownSel."
    },
    {
        46,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for ldoPullDownSel."
    },
    {
        47,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoSlowRampEn."
    },
    {
        48,
        "Pmic_powerSetPwrResourceCfg : Test Voltage level in miliVolts for regulators"
    },
    {
        49,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for voltage_mV."
    },
    {
        50,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for voltage_mV."
    },
    {
        54,
        "Pmic_powerSetPwrResourceCfg : Test Powergood level for VCCA pin"
    },
    {
        55,
        "Pmic_powerSetPwrResourceCfg : Test Powergood level for VCCA pin"
    },
    {
        56,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for vccaPwrGudLvl."
    },
    {
        57,
        "Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group none."
    },
    {
        58,
        "Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group mcu."
    },
    {
        59,
        "Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group soc."
    },
    {
        60,
        "Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group other."
    },
    {
        61,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for railGrpSel."
    },
    {
        62,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for railGrpSel."
    },
    {
        63,
        "Pmic_powerSetPwrResourceCfg : Test BUCK switching frequency for 4.4M."
    },
    {
        64,
        "Pmic_powerSetPwrResourceCfg : Test BUCK switching frequency for 2.2M."
    },
    {
        65,
        "Pmic_powerSetPwrResourceCfg : Test BUCK switching frequency for 8.8M."
    },
    {
        66,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckFreqSel."
    },
    {
        67,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for buckFreqSel."
    },
    {
        68,
        "Pmic_powerSetPwrResourceCfg : Test LDO Bypass Regulator LDO mode"
    },
    {
        69,
        "Pmic_powerSetPwrResourceCfg : Test LDO Linear Regulator LDO mode"
    },
    {
        70,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoBypassModeEn."
    },
    {
        71,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 0.5MS."
    },
    {
        72,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 1MS."
    },
    {
        73,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 1.5MS."
    },
    {
        74,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 2MS."
    },
    {
        75,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 2.5MS."
    },
    {
        76,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 3MS."
    },
    {
        77,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 3.5MS."
    },
    {
        78,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 4MS."
    },
    {
        79,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 6MS."
    },
    {
        80,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 8MS."
    },
    {
        81,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 10MS."
    },
    {
        82,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 12MS."
    },
    {
        83,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 14MS."
    },
    {
        84,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 16MS."
    },
    {
        85,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoRvTimeoutSel."
    },
    {
        86,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for ldoRvTimeoutSel."
    },
    {
        87,
        "Pmic_powerSetPwrResourceCfg : Test OV/UV voltage monitoring range for vmonRange for range1."
    },
    {
        88,
        "Pmic_powerSetPwrResourceCfg : Test OV/UV voltage monitoring range for vmonRange for range2."
    },
    {
        89,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for vmonRange."
    },
    {
        90,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for Pmic_PowerResourceCfg_t"
    },
    {
        91,
        "Pmic_powerSetCommonConfig : Test pgoodWindow uv monitor enable"
    },
    {
        92,
        "Pmic_powerSetCommonConfig : Test pgoodWindow uv and ov monitor enable"
    },
    {
        93,
        "Pmic_powerSetCommonConfig : Test Enable pgoodPolarity high"
    },
    {
        94,
        "Pmic_powerSetCommonConfig : Test Enable pgoodPolarity low"
    },
    {
        95,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as voltage current"
    },
    {
        96,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as voltage"
    },
    {
        97,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as masked"
    },
    {
        98,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as NRSTOUT"
    },
    {
        99,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as NRSTOUT SOC"
    },
    {
        100,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as TDIE WARN"
    },
    {
        101,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as VCCA"
    },
    {
        102,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as VMON"
    },
    {
        103,
        "Pmic_powerGetPwrRsrcStat : Test get currentLimitLvlStat"
    },
    {
        104,
        "Pmic_powerGetPwrRsrcStat : Test get underVoltageTholdStat"
    },
    {
        105,
        "Pmic_powerGetPwrRsrcStat : Test get overVoltageTholdStat"
    },
    {
        106,
        "Pmic_powerGetPwrRsrcStat : Test get overVoltageProtectionLvlStat"
    },
    {
        107,
        "Pmic_powerSetThermalConfig : Test thermalWarnThold as low"
    },
    {
        108,
        "Pmic_powerSetThermalConfig : Test thermalWarnThold as high"
    },
    {
        109,
        "Pmic_powerSetThermalConfig : Test thermalShutdownThold as low"
    },
    {
        110,
        "Pmic_powerSetThermalConfig : Test thermalShutdownThold as high"
    },
    {
        111,
        "Pmic_powerSetPwrRsrcIntr : Test Enable OV interrupt"
    },
    {
        112,
        "Pmic_powerSetPwrRsrcIntr : Test Disable OV interrupt"
    },
    {
        113,
        "Pmic_powerSetPwrRsrcIntr : Test Enable UV interrupt"
    },
    {
        114,
        "Pmic_powerSetPwrRsrcIntr : Test Disable UV interrupt"
    },
    {
        115,
        "Pmic_powerSetPwrRsrcIntr : Test Enable ILIM interrupt"
    },
    {
        116,
        "Pmic_powerSetPwrRsrcIntr : Test Disable ILIM interrupt"
    },
    {
        117,
        "Pmic_powerSetIntr : Test Enable TWARN interrupt"
    },
    {
        118,
        "Pmic_powerSetIntr : Test Disable TWARN interrupt"
    },
    {
        119,
        "Pmic_powerSetIntr : Test Enable NRSTOUT_READBACK interrupt"
    },
    {
        120,
        "Pmic_powerSetIntr : Test Disable NRSTOUT_READBACK interrupt"
    },
    {
        121,
        "Pmic_powerSetIntr : Test Enable SOC_PWR_ERR interrupt"
    },
    {
        122,
        "Pmic_powerSetIntr : Test Disable SOC_PWR_ERR interrupt"
    },
    {
        123,
        "Pmic_powerSetIntr : Test Enable MCU_PWR_ERR interrupt"
    },
    {
        124,
        "Pmic_powerSetIntr : Test Disable MCU_PWR_ERR interrupt"
    },
    {
        125,
        "Pmic_powerSetIntr : Test Enable ORD_SHUTDOWN interrupt"
    },
    {
        126,
        "Pmic_powerSetIntr : Test Disable ORD_SHUTDOWN interrupt"
    },
    {
        127,
        "Pmic_powerSetIntr : Test Enable IMM_SHUTDOWN interrupt"
    },
    {
        128,
        "Pmic_powerSetIntr : Test Disable IMM_SHUTDOWN interrupt"
    },
    {
        129,
        "Pmic_powerSetIntr : Test Enable NRSTOUT_SOC_READBACK interrupt"
    },
    {
        130,
        "Pmic_powerSetIntr : Test Disable NRSTOUT_SOC_READBACK interrupt"
    },
    {
        131,
        "Pmic_powerSetIntr : Test Enable EN_DRV_READBACK interrupt"
    },
    {
        132,
        "Pmic_powerSetIntr : Test Disable EN_DRV_READBACK interrupt"
    },
    {
        133,
        "Pmic_powerSetCommonConfig : Test deglitchTimeSel as 4us"
    },
    {
        134,
        "Pmic_powerSetCommonConfig : Test deglitchTimeSel as 20us"
    },
    {
        135,
        "Pmic_powerSetCommonConfig : Test severeErrorTrig as Immediate shutdown"
    },
    {
        136,
        "Pmic_powerSetCommonConfig : Test severeErrorTrig as oderly shutdown"
    },
    {
        137,
        "Pmic_powerSetCommonConfig : Test severeErrorTrig as mcu power error"
    },
    {
        138,
        "Pmic_powerSetCommonConfig : Test severeErrorTrig as soc power error"
    },
    {
        139,
        "Pmic_powerSetCommonConfig : Test otherRailTrig as Immediate shutdown"
    },
    {
        140,
        "Pmic_powerSetCommonConfig : Test otherRailTrig as oderly shutdown"
    },
    {
        141,
        "Pmic_powerSetCommonConfig : Test otherRailTrig as mcu power error"
    },
    {
        142,
        "Pmic_powerSetCommonConfig : Test otherRailTrig as soc power error"
    },
    {
        143,
        "Pmic_powerSetCommonConfig : Test socRailTrig as Immediate shutdown"
    },
    {
        144,
        "Pmic_powerSetCommonConfig : Test socRailTrig as oderly shutdown"
    },
    {
        145,
        "Pmic_powerSetCommonConfig : Test socRailTrig as mcu power error"
    },
    {
        146,
        "Pmic_powerSetCommonConfig : Test socRailTrig as soc power error"
    },
    {
        147,
        "Pmic_powerSetCommonConfig : Test mcuRailTrig as Immediate shutdown"
    },
    {
        148,
        "Pmic_powerSetCommonConfig : Test mcuRailTrig as oderly shutdown"
    },
    {
        149,
        "Pmic_powerSetCommonConfig : Test mcuRailTrig as mcu power error"
    },
    {
        150,
        "Pmic_powerSetCommonConfig : Test mcuRailTrig as soc power error"
    },
    {
        151,
        "Pmic_powerSetCommonConfig : Test moderateRailTrig as Immediate shutdown"
    },
    {
        152,
        "Pmic_powerSetCommonConfig : Test moderateRailTrig as oderly shutdown"
    },
    {
        153,
        "Pmic_powerSetCommonConfig : Test moderateRailTrig as mcu power error"
    },
    {
        154,
        "Pmic_powerSetCommonConfig : Test moderateRailTrig as soc power error"
    },
    {
        155,
        "Pmic_powerSetLdoRtc : Test Enable ldortcRegulator"
    },
    {
        156,
        "Pmic_powerSetLdoRtc : Test Disable ldortcRegulator"
    },
    {
        157,
        "Pmic_powerSetPwrRsrcIntr : Test moderateRailTrig as mcu power error"
    },
    {
        158,
        "Pmic_powerSetPwrRsrcIntr : Parameter validation for intrType"
    },
    {
        159,
        "Pmic_powerSetCommonConfig : Parameter validation for severeErrorTrig"
    },
    {
        160,
        "Pmic_powerSetCommonConfig : Parameter validation for otherRailTrig"
    },
    {
        161,
        "Pmic_powerSetCommonConfig : Parameter validation for socRailTrig"
    },
    {
        162,
        "Pmic_powerSetCommonConfig : Parameter validation for mcuRailTrig"
    },
    {
        163,
        "Pmic_powerSetCommonConfig : Parameter validation for moderateRailTrig"
    },
    {
        166,
        "Pmic_powerGetCommonConfig : Parameter validation for handle"
    },
    {
        167,
        "Pmic_powerSetConfigPowerGood : Parameter validation for handle"
    },
    {
        168,
        "Pmic_powerGetConfigPowerGood : Parameter validation for handle"
    },
    {
        169,
        "Pmic_powerGetPwrRsrcStat : Parameter validation for handle"
    },
    {
        170,
        "Pmic_powerSetThermalConfig : Parameter validation for handle"
    },
    {
        171,
        "Pmic_powerGetThermalConfig : Parameter validation for handle"
    },
    {
        172,
        "Pmic_powerSetPwrRsrcIntr : Parameter validation for handle"
    },
    {
        173,
        "Pmic_powerSetIntr : Parameter validation for handle"
    },
    {
        174,
        "Pmic_powerSetPwrRsrcIntr : Test Set Enable OV interrupt"
    },
    {
        175,
        "Pmic_powerSetPwrRsrcIntr : Test Set Disable OV interrupt"
    },
    {
        176,
        "Pmic_powerSetPwrRsrcIntr : Test Set Enable UV interrupt"
    },
    {
        177,
        "Pmic_powerSetPwrRsrcIntr : Test Set Disable UV interrupt"
    },
    {
        178,
        "Pmic_powerSetPwrRsrcIntr : Test Set Enable ILIM interrupt"
    },
    {
        179,
        "Pmic_powerSetPwrRsrcIntr : Test Set Disable ILIM interrupt"
    },
    {
        180,
        "Pmic_powerSetIntr : Test Set Enable TWARN interrupt"
    },
    {
        181,
        "Pmic_powerSetIntr : Test Set Disable TWARN interrupt"
    },
    {
        182,
        "Pmic_powerSetIntr : Test Set Enable NRSTOUT_READBACK interrupt"
    },
    {
        183,
        "Pmic_powerSetIntr : Test Set Disable NRSTOUT_READBACK interrupt"
    },
    {
        184,
        "Pmic_powerSetIntr : Test Set Enable SOC_PWR_ERR interrupt"
    },
    {
        185,
        "Pmic_powerSetIntr : Test Set Disable SOC_PWR_ERR interrupt"
    },
    {
        186,
        "Pmic_powerSetIntr : Test Set Enable MCU_PWR_ERR interrupt"
    },
    {
        187,
        "Pmic_powerSetIntr : Test Set Disable MCU_PWR_ERR interrupt"
    },
    {
        188,
        "Pmic_powerSetIntr : Test Set Enable ORD_SHUTDOWN interrupt"
    },
    {
        189,
        "Pmic_powerSetIntr : Test Set Disable ORD_SHUTDOWN interrupt"
    },
    {
        190,
        "Pmic_powerSetIntr : Test Set Enable IMM_SHUTDOWN interrupt"
    },
    {
        191,
        "Pmic_powerSetIntr : Test Set Disable IMM_SHUTDOWN interrupt"
    },
    {
        192,
        "Pmic_powerSetIntr : Test Set Enable NRSTOUT_SOC_READBACK interrupt"
    },
    {
        193,
        "Pmic_powerSetIntr : Test Set Disable NRSTOUT_SOC_READBACK interrupt"
    },
    {
        194,
        "Pmic_powerSetIntr : Test Set Enable EN_DRV_READBACK interrupt"
    },
    {
        195,
        "Pmic_powerSetIntr : Test Set Disable EN_DRV_READBACK interrupt"
    },
    {
        201,
        "Pmic_powerSetCommonConfig : Parameter validation for handle"
    },

};

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for handle
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_handle(void)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;
    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(0,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrResourceCfg(NULL, pwrRsrc, pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for handle
 */
static void test_pmic_powerGetPowerResourceConfigPrmValTest_handle(void)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;
    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(00,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrResourceCfg(NULL, pwrRsrc, &pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Enable residual voltage checking
 */
static void test_pmic_powerSetPowerResourceConfig_rvCheckEn_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(1,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.rvCheckEn = PMIC_TPS6594X_REGULATOR_RV_SEL_ENABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.rvCheckEn = PMIC_LP8764X_REGULATOR_VMON_RV_SEL_ENABLE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.rvCheckEn, powerCfg_rd.rvCheckEn);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.rvCheckEn, powerCfg_rd.rvCheckEn);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Disable residual voltage checking
 */
static void test_pmic_powerSetPowerResourceConfig_rvCheckEn_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(2,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.rvCheckEn = PMIC_TPS6594X_REGULATOR_RV_SEL_DISABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.rvCheckEn = PMIC_LP8764X_REGULATOR_VMON_RV_SEL_DISABLE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.rvCheckEn, powerCfg_rd.rvCheckEn);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.rvCheckEn, powerCfg_rd.rvCheckEn);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for rvCheckEn.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_rvCheckEn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;

    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_LDO_MAX + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_VMON_MAX + 1;
    }

    test_pmic_print_unity_testcase_info(3,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Enable buck pull down checking for Buck regulator
 */
static void test_pmic_powerSetPowerResourceConfig_buckPullDownEn_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(4,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckPullDownEn = PMIC_TPS6594X_REGULATOR_BUCK_PLDN_ENABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckPullDownEn = PMIC_LP8764X_REGULATOR_BUCK_PLDN_ENABLE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckPullDownEn, powerCfg_rd.buckPullDownEn);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Disable buck pull down checking for Buck regulator
 */
static void test_pmic_powerSetPowerResourceConfig_buckPullDownEn_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(5,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckPullDownEn = PMIC_TPS6594X_REGULATOR_BUCK_PLDN_DISABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckPullDownEn = PMIC_LP8764X_REGULATOR_BUCK_PLDN_DISABLE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckPullDownEn, powerCfg_rd.buckPullDownEn);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckPullDownEn.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckPullDownEn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(6,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Enable the Voltage monitor.
 */
static void test_pmic_powerSetPowerResourceConfig_vmonEn_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonEn = PMIC_TPS6594X_VMON_ENABLE;
        pwrRsrcMin = PMIC_TPS6594X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonEn = PMIC_LP8764X_VMON_ENABLE;
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.vmonEn, powerCfg_rd.vmonEn);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.vmonEn, powerCfg_rd.vmonEn);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.vmonEn, powerCfg_rd.vmonEn);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Disable the Voltage monitor.
 */
static void test_pmic_powerSetPowerResourceConfig_vmonEn_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(8,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonEn = PMIC_TPS6594X_VMON_DISABLE;
        pwrRsrcMin = PMIC_TPS6594X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonEn = PMIC_LP8764X_VMON_DISABLE;
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.vmonEn, powerCfg_rd.vmonEn);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.vmonEn, powerCfg_rd.vmonEn);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.vmonEn, powerCfg_rd.vmonEn);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg :  Parameter validation for Power Resource for vmonEn.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_vmonEn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_EN_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_LDO_MAX + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_VMON_MAX + 1;
    }

    test_pmic_print_unity_testcase_info(9,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Select output voltage register for buck as VOUT1.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVoutSel_vout1(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVoutSel = PMIC_TPS6594X_REGULATOR_BUCK_VOUT_SEL_VOUT1;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVoutSel = PMIC_LP8764X_REGULATOR_BUCK_VOUT_SEL_VOUT1;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckVoutSel, powerCfg_rd.buckVoutSel);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Select output voltage register for buck as VOUT2.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVoutSel_vout2(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(11,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVoutSel = PMIC_TPS6594X_REGULATOR_BUCK_VOUT_SEL_VOUT2;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVoutSel = PMIC_LP8764X_REGULATOR_BUCK_VOUT_SEL_VOUT2;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                      pwrRsrc,
                                                      &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(pPowerCfg.buckVoutSel, powerCfg_rd.buckVoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckVoutSel.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckVoutSel(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(12,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Select Auto Mode for BUCK
 */
static void test_pmic_powerSetPowerResourceConfig_buckFpwmMode_auto(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_FPWM_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_FPWM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(13,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMode = PMIC_TPS6594X_REGULATOR_AUTO_PWM_PFM_MODE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMode = PMIC_LP8764X_REGULATOR_AUTO_PWM_PFM_MODE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckFpwmMode, powerCfg_rd.buckFpwmMode);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Select PWM Mode for BUCK
 */
static void test_pmic_powerSetPowerResourceConfig_buckFpwmMode_pwm(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_FPWM_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_FPWM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(14,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMode = PMIC_TPS6594X_REGULATOR_PWM_MODE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMode = PMIC_LP8764X_REGULATOR_PWM_MODE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckFpwmMode, powerCfg_rd.buckFpwmMode);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckFpwmMode.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckFpwmMode(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_FPWM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(15,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Select Multi phase with PWM Mode for BUCK
 */
static void test_pmic_powerSetPowerResourceConfig_buckFpwmMpMode_multiPhase(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(16,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMpMode = PMIC_TPS6594X_REGULATOR_PWM_MP_MODE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK3;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMpMode = PMIC_LP8764X_REGULATOR_PWM_MP_MODE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK3;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        /* Not supported by BUCK 2 */
        if((PMIC_TPS6594X_REGULATOR_BUCK2 == pwrRsrc) ||
           (PMIC_LP8764X_REGULATOR_BUCK2 == pwrRsrc))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckFpwmMpMode, powerCfg_rd.buckFpwmMpMode);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Select AUTO Mode with Automatic phase adding and shedding for BUCK
 */
static void test_pmic_powerSetPowerResourceConfig_buckFpwmMpMode_auto(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(17,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMpMode = PMIC_TPS6594X_REGULATOR_AUTO_PHASE_MODE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK3;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMpMode = PMIC_LP8764X_REGULATOR_AUTO_PHASE_MODE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK3;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        /* Not supported by BUCK 2 */
        if((PMIC_TPS6594X_REGULATOR_BUCK2 == pwrRsrc) ||
           (PMIC_LP8764X_REGULATOR_BUCK2 == pwrRsrc))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckFpwmMpMode, powerCfg_rd.buckFpwmMpMode);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckFpwmMpMode.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckFpwmMpMode(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(18,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_LDO1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

#if 0
/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Disable the power regulator.
 */
static void test_pmic_powerSetPowerResourceConfig_regulatorEn_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(19,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.regulatorEn = PMIC_TPS6594X_REGULATOR_DISABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.regulatorEn = PMIC_LP8764X_BUCK_REGULATOR_DISABLE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                      pwrRsrc,
                                                      &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(pPowerCfg.regulatorEn, powerCfg_rd.regulatorEn);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
                (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
                ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
            {
                continue;
            }

            if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
                (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
                ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
            {
                continue;
            }

            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                          pwrRsrc,
                                                          &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.regulatorEn, powerCfg_rd.regulatorEn);
        }
    }

}
#endif

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Enable the power regulator.
 */
static void test_pmic_powerSetPowerResourceConfig_regulatorEn_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(20,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.regulatorEn = PMIC_TPS6594X_REGULATOR_ENABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.regulatorEn = PMIC_LP8764X_BUCK_REGULATOR_ENABLE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.regulatorEn, powerCfg_rd.regulatorEn);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.regulatorEn, powerCfg_rd.regulatorEn);
        }
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for regulatorEn.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_regulatorEn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_REGULATOR_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(21,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 6.5Amp.
 */
static void test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_6A5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(22,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_6A5;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckCurrentLimit, powerCfg_rd.buckCurrentLimit);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 5.5Amp.
 */
static void test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_5A5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(23,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_5A5;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_5A5;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckCurrentLimit, powerCfg_rd.buckCurrentLimit);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 4.5Amp.
 */
static void test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_4A5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(24,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_4A5;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_4A5;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckCurrentLimit, powerCfg_rd.buckCurrentLimit);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 3.5Amp.
 */
static void test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_3A5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(25,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_3A5;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_3A5;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckCurrentLimit, powerCfg_rd.buckCurrentLimit);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 2.5Amp.
 */
static void test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_2A5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(26,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_2A5;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_2A5;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckCurrentLimit, powerCfg_rd.buckCurrentLimit);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckCurrentLimit.
*/
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckCurrentLimit(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(27,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_LDO1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for buckCurrentLimit.
 */
static void test_pmic_powerSetPowerResourceConfigPrmRangeTest_buckCurrentLimit(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(28,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
        pPowerCfg.buckCurrentLimit = PMIC_TPS6594X_BUCK1_4_CURRENT_LIMIT_MAX + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
        pPowerCfg.buckCurrentLimit = PMIC_LP8764X_BUCK_CURRENT_LIMIT_MAX + 1;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if(pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5)
        {
            pPowerCfg.buckCurrentLimit = PMIC_TPS6594X_BUCK5_CURRENT_LIMIT_MAX + 1;
        }
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 33MV.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_33MV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(29,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                    PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_33MV;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_33MV;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 20MV.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_20MV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(30,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                    PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_20MV;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_20MV;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 10MV.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_10MV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(31,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                    PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_10MV;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_10MV;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 5MV.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_05MV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(32,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                    PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_05MV;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_05MV;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 2.5MV.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_2MV5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(33,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                    PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_2MV5;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_2MV5;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 1.3MV.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_1MV3(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(34,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                    PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_1MV3;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_1MV3;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 0.63MV.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_0MV63(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(35,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                    PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_0MV63;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_0MV63;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 0.31MV.
 */
static void test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_0MV31(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(36,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                    PMIC_TPS6594X_REGULATOR_BUCK_SLEW_RATE_0MV31;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVmonSlewRate = \
                                PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_0MV31;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.buckVmonSlewRate, powerCfg_rd.buckVmonSlewRate);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckVmonSlewRate.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckVmonSlewRate(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(37,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for buckVmonSlewRate.
 */
static void test_pmic_powerSetPowerResourceConfigPrmRangeTest_buckVmonSlewRate(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(38,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
        pPowerCfg.buckVmonSlewRate = PMIC_TPS6594X_BUCK_SLEW_RATE_MAX + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
        pPowerCfg.buckVmonSlewRate = PMIC_LP8764X_BUCK_SLEW_RATE_MAX + 1;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
        }
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 50Kohm.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoPullDownSel_50KOHM(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(39,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoPullDownSel = PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_50KOHM;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoPullDownSel, powerCfg_rd.ldoPullDownSel);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 125Kohm
 */
static void test_pmic_powerSetPowerResourceConfig_ldoPullDownSel_125OHM(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(40,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoPullDownSel = PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_125OHM;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoPullDownSel, powerCfg_rd.ldoPullDownSel);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 250Kohm
 */
static void test_pmic_powerSetPowerResourceConfig_ldoPullDownSel_250OHM(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(41,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoPullDownSel = PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_250OHM;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoPullDownSel, powerCfg_rd.ldoPullDownSel);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 500Kohm
 */
static void test_pmic_powerSetPowerResourceConfig_ldoPullDownSel_500OHM(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(42,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoPullDownSel = PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_500OHM;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoPullDownSel, powerCfg_rd.ldoPullDownSel);
    }

}

#if 0
/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Enable Slow Ramp for LDO
 */
static void test_pmic_powerSetPowerResourceConfig_ldoSlowRampEn_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(43,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoSlowRampEn = PMIC_TPS6594X_REGULATOR_LDO_SLOW_RAMP_ENABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoSlowRampEn, powerCfg_rd.ldoSlowRampEn);
    }
}
#endif

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Disable Slow Ramp for LDO
 */
static void test_pmic_powerSetPowerResourceConfig_ldoSlowRampEn_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(44,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoSlowRampEn = PMIC_TPS6594X_REGULATOR_LDO_SLOW_RAMP_DISABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoSlowRampEn, powerCfg_rd.ldoSlowRampEn);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoPullDownSel.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_ldoPullDownSel(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(45,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for ldoPullDownSel.
 */
static void test_pmic_powerSetPowerResourceConfigPrmRangeTest_ldoPullDownSel(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(46,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
        pPowerCfg.ldoPullDownSel = PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_MAX + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoSlowRampEn.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_ldoSlowRampEn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(47,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Voltage level in miliVolts for regulators
 */
static void test_pmic_powerSetPowerResourceConfig_voltage_mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;
    uint32_t step=0;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(48,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        step=20;
        pPowerCfg.voltage_mV = 300U;
        for(;pPowerCfg.voltage_mV <= 3340;pPowerCfg.voltage_mV+=step)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.voltage_mV, powerCfg_rd.voltage_mV);
            pmic_log("passed for %dmv \n", pPowerCfg.voltage_mV);
            if(pPowerCfg.voltage_mV == 580)
            {
                pPowerCfg.voltage_mV = 595;step=5;
            }
            if(pPowerCfg.voltage_mV == 1095)
            {
                pPowerCfg.voltage_mV = 1210;step=10;
            }
            if(pPowerCfg.voltage_mV == 1650)
            {
                pPowerCfg.voltage_mV = 1640;step=20;
            }
        }

    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        step=50;
        pPowerCfg.voltage_mV = 600U;
        if(pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)
        {
            pPowerCfg.voltage_mV = 1200U;
            step=25;
        }

        for(;pPowerCfg.voltage_mV <= 3300;pPowerCfg.voltage_mV+=step)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(pPowerCfg.voltage_mV, powerCfg_rd.voltage_mV);
            pmic_log("passed for %dmv \n", pPowerCfg.voltage_mV);

            if((1500 == pPowerCfg.voltage_mV) &&
               (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO3))
            {
                break;
            }
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for voltage_mV.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_voltage_mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(49,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for voltage_mV.
 */
static void test_pmic_powerSetPowerResourceConfigPrmRangeTest_voltage_mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(50,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
        pPowerCfg.voltage_mV = PMIC_TPS6594X_REGULATOR_LDO_MAX_VOLTAGE + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
        pPowerCfg.voltage_mV = PMIC_LP8764X_RANGE1_VMON_MAX_VOLTAGE + 1;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
        pPowerCfg.voltage_mV = PMIC_TPS6594X_REGULATOR_LDO_MAX_VOLTAGE + 1;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Powergood level for VCCA pin
 */
static void test_pmic_powerSetPowerResourceConfig_vccaPwrGudLvl_5V(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT,
    };
    uint16_t pwrRsrc = 0U;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vccaPwrGudLvl = PMIC_TPS6594X_VCCA_PG_5V_LEVEL;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vccaPwrGudLvl = PMIC_LP8764X_VCCA_PG_5V_LEVEL;
    }

    test_pmic_print_unity_testcase_info(54,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                  pwrRsrc,
                                                  pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                  pwrRsrc,
                                                  &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.vccaPwrGudLvl, powerCfg_rd.vccaPwrGudLvl);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Powergood level for VCCA pin
 */
static void test_pmic_powerSetPowerResourceConfig_vccaPwrGudLvl_3V3(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT,
    };
    uint16_t pwrRsrc = 0U;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vccaPwrGudLvl = PMIC_TPS6594X_VCCA_PG_3V3_LEVEL;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vccaPwrGudLvl = PMIC_LP8764X_VCCA_PG_3V3_LEVEL;
    }

    test_pmic_print_unity_testcase_info(55,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                  pwrRsrc,
                                                  pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                  pwrRsrc,
                                                  &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.vccaPwrGudLvl, powerCfg_rd.vccaPwrGudLvl);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for vccaPwrGudLvl.
 */
 static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_vccaPwrGudLvl(void)
 {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT,
    };

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    test_pmic_print_unity_testcase_info(56,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group none.
 */
static void test_pmic_powerSetPowerResourceConfig_railGrpSel_none(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(57,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_TPS6594X_POWER_RAIL_SEL_NONE;
        pwrRsrcMin = PMIC_TPS6594X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_LP8764X_POWER_RAIL_SEL_NONE;
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
                (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
                ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)  ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
            {
                continue;
            }

            if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
                (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
                ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
            {
                continue;
            }
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
        {
            pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     pPowerCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                     pwrRsrc,
                                                     &powerCfg_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);

        }
    }
}
#if 0
/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group mcu.
 */
static void test_pmic_powerSetPowerResourceConfig_railGrpSel_mcu(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(58,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_TPS6594X_POWER_RAIL_SEL_MCU;
        pwrRsrcMin = PMIC_TPS6594X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_LP8764X_POWER_RAIL_SEL_MCU;
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group soc.
 */
static void test_pmic_powerSetPowerResourceConfig_railGrpSel_soc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(59,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_TPS6594X_POWER_RAIL_SEL_SOC;
        pwrRsrcMin = PMIC_TPS6594X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_LP8764X_POWER_RAIL_SEL_SOC;
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group other.
 */
static void test_pmic_powerSetPowerResourceConfig_railGrpSel_other(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(60,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_TPS6594X_POWER_RAIL_SEL_OTHER;
        pwrRsrcMin = PMIC_TPS6594X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_LP8764X_POWER_RAIL_SEL_OTHER;
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VCCA;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.railGrpSel, powerCfg_rd.railGrpSel);
    }
}
#endif

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for railGrpSel.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_railGrpSel(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(61,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_LDO_MAX + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_BUCK_MAX + 1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for railGrpSel.
 */
static void test_pmic_powerSetPowerResourceConfigPrmRangeTest_railGrpSel(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(62,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
        pPowerCfg.railGrpSel = PMIC_TPS6594X_POWER_RAIL_SEL_MAX + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
        pPowerCfg.railGrpSel = PMIC_LP8764X_POWER_RAIL_SEL_MAX + 1;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }
}
 #if 0
/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test BUCK switching frequency for 4.4M.
 */
static void test_pmic_powerSetPowerResourceConfig_buckFreqSel_4M4(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(63,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFreqSel = PMIC_LP8764X_BUCK_FREQ_SEL_4M4;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFreqSel = PMIC_LP8764X_BUCK_FREQ_SEL_4M4;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(pPowerCfg.buckFreqSel, powerCfg_rd.buckFreqSel);
    }
}
#endif

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test BUCK switching frequency for 2.2M.
 */
static void test_pmic_powerSetPowerResourceConfig_buckFreqSel_2M2(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(64,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFreqSel = PMIC_LP8764X_BUCK_FREQ_SEL_2M2;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFreqSel = PMIC_LP8764X_BUCK_FREQ_SEL_2M2;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckFreqSel, powerCfg_rd.buckFreqSel);

    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test BUCK switching frequency for 8.8M.
 */
static void test_pmic_powerSetPowerResourceConfig_buckFreqSel_8M8(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(65,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFreqSel = PMIC_LP8764X_BUCK_FREQ_SEL_8M8;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckFreqSel, powerCfg_rd.buckFreqSel);

    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckFreqSel.
 */
 static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckFreqSel(void)
 {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(66,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for buckFreqSel.
 */
static void test_pmic_powerSetPowerResourceConfigPrmRangeTest_buckFreqSel(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(67,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
        pPowerCfg.buckFreqSel = PMIC_TPS6594X_BUCK_FREQ_SE_MAX + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
        pPowerCfg.buckFreqSel = PMIC_LP8764X_BUCK_FREQ_SE_MAX + 1;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    }
}

#if 0
/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO Bypass Regulator LDO mode
 */
static void test_pmic_powerSetPowerResourceConfig_ldoBypassModeEn_bypass(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(68,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoBypassModeEn = PMIC_TPS6594X_REGULATOR_LDO_BYPASS_MODDE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO3;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICA_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (LEO_PMICB_SLAVE_ADDR == pPmicCoreHandle->slaveAddr) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(pPowerCfg.ldoBypassModeEn, powerCfg_rd.ldoBypassModeEn);
        }

}
#endif


#if 0
/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO Linear Regulator LDO mode
 */
static void test_pmic_powerSetPowerResourceConfig_ldoBypassModeEn_linear(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(69,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoBypassModeEn = PMIC_TPS6594X_REGULATOR_LDO_LINEAR_REGULATOR_MODE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoBypassModeEn, powerCfg_rd.ldoBypassModeEn);
    }
}
#endif

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoBypassModeEn.
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_ldoBypassModeEn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(70,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 0.5MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_0MS5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(71,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_0MS5;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 1MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_1MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(72,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_1MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 1.5MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_1MS5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(73,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_1MS5;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 2MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_2MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(74,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_2MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 2.5MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_2MS5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(75,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_2MS5;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 3MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_3MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(76,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_3MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 3.5MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_3MS5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(77,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_3MS5;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 4MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_4MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(78,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_4MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 6MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_6MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(79,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_6MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 8MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_8MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(80,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_8MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 10MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_10MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(81,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_10MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 12MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_12MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(82,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_12MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 14MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_14MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(83,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_14MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 16MS.
 */
static void test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_16MS(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(84,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_16MS;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoRvTimeoutSel, powerCfg_rd.ldoRvTimeoutSel);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoRvTimeoutSel.
 */
 static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_ldoRvTimeoutSel(void)
 {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(85,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for ldoRvTimeoutSel.
 */
static void test_pmic_powerSetPowerResourceConfigPrmRangeTest_ldoRvTimeoutSel(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(86,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoRvTimeoutSel =
                                PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_MAX + 1;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test OV/UV voltage monitoring range for vmonRange for range1.
 */
static void test_pmic_powerSetPowerResourceConfig_vmonRange_range1(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(87,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_0V3_3V34;
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoPullDownSel, powerCfg_rd.ldoPullDownSel);
    }

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test OV/UV voltage monitoring range for vmonRange for range2.
 */
static void test_pmic_powerSetPowerResourceConfig_vmonRange_range2(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(88,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_3V35_5V;
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.ldoPullDownSel, powerCfg_rd.ldoPullDownSel);
    }

}
/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for vmonRange.
 */
 static void test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_vmonRange(void)
 {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(89,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for Pmic_PowerResourceCfg_t
 */
static void test_pmic_powerGetPowerResourceConfigPrmValTest_Pmic_PowerResourceCfg_t(void)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;

    test_pmic_print_unity_testcase_info(90,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle, pwrRsrc, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetCommonConfig : Parameter validation for handle
 */
static void test_pmic_powerSetCommonConfigPrmValTest_handle(void)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_POWER_PGOOD_WINDOW_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(201,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetCommonConfig(NULL,  pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}


/*!
 * \brief   Pmic_powerSetCommonConfig : Test pgoodWindow uv monitor enable
 */
static void test_pmic_powerSetCommonConfig_pgoodWindow_uv(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_POWER_PGOOD_WINDOW_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_POWER_PGOOD_WINDOW_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(91,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.pgoodWindow = PMIC_POWER_GOOD_UV_MONITOR_ENABLE;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.pgoodWindow, powerCfg_rd.pgoodWindow);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test pgoodWindow uv and ov monitor enable
 */
static void test_pmic_powerSetCommonConfig_pgoodWindow_uv_ov(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_POWER_PGOOD_WINDOW_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_POWER_PGOOD_WINDOW_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(92,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.pgoodWindow = PMIC_POWER_GOOD_UV_OV_MONITOR_ENABLE;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle,
                                             pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.pgoodWindow, powerCfg_rd.pgoodWindow);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test Enable pgoodPolarity high
 */
static void test_pmic_powerSetCommonConfig_pgoodPolarity_high(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_POWER_PGOOD_POL_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_POWER_PGOOD_POL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(93,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pwrCommonCfg.pgoodPolarity = PMIC_POWER_PGOOD_POL_HIGH;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle,
                                             pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.pgoodPolarity, powerCfg_rd.pgoodPolarity);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test Enable pgoodPolarity low
 */
static void test_pmic_powerSetCommonConfig_pgoodPolarity_low(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(94,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.pgoodPolarity = PMIC_POWER_PGOOD_POL_LOW;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle,
                                             pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.pgoodPolarity, powerCfg_rd.pgoodPolarity);

}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Test pgoodSelType as voltage current
 */
static void test_pmic_powerSetConfigPowerGood_pgoodSelType_voltageCurrent(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;
    uint8_t  pPgoodSelType_rd;
    uint16_t pwrRsrcMin, pwrRsrcMax;

    test_pmic_print_unity_testcase_info(95,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_VOLTAGE_CURRENT;
        pwrRsrcMin = PMIC_TPS6594X_PGOOD_SOURCE_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_PGOOD_SOURCE_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_SRC_VOLTAGE_CURRENT;
        pwrRsrcMin = PMIC_LP8764X_PGOOD_SOURCE_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_PGOOD_SOURCE_BUCK4;
    }

    for(pgoodSrcSel = pwrRsrcMin; pgoodSrcSel <= pwrRsrcMax ; pgoodSrcSel++)
    {
        pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 pgoodSelType);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 &pPgoodSelType_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_VOLTAGE_CURRENT;
        pwrRsrcMin = PMIC_TPS6594X_PGOOD_SOURCE_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_PGOOD_SOURCE_LDO4;

        for(pgoodSrcSel = pwrRsrcMin; pgoodSrcSel <= pwrRsrcMax ; pgoodSrcSel++)
        {
            pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     pgoodSelType);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     &pPgoodSelType_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);
        }
    }

}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Test pgoodSelType as voltage
 */
static void test_pmic_powerSetConfigPowerGood_pgoodSelType_voltage(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;
    uint8_t  pPgoodSelType_rd;
    uint16_t pwrRsrcMin, pwrRsrcMax;

    test_pmic_print_unity_testcase_info(96,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_VOLTAGE;
        pwrRsrcMin = PMIC_TPS6594X_PGOOD_SOURCE_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_PGOOD_SOURCE_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_SRC_VOLTAGE;
        pwrRsrcMin = PMIC_LP8764X_PGOOD_SOURCE_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_PGOOD_SOURCE_BUCK4;
    }

    for(pgoodSrcSel = pwrRsrcMin; pgoodSrcSel <= pwrRsrcMax ; pgoodSrcSel++)
    {
        pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 pgoodSelType);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 &pPgoodSelType_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_VOLTAGE;
        pwrRsrcMin = PMIC_TPS6594X_PGOOD_SOURCE_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_PGOOD_SOURCE_LDO4;

        for(pgoodSrcSel = pwrRsrcMin; pgoodSrcSel <= pwrRsrcMax ; pgoodSrcSel++)
        {
            pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     pgoodSelType);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     &pPgoodSelType_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);
        }
    }

}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Test pgoodSelType as masked
 */
static void test_pmic_powerSetConfigPowerGood_pgoodSelType_masked(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;
    uint8_t  pPgoodSelType_rd;
    uint16_t pwrRsrcMin, pwrRsrcMax;

    test_pmic_print_unity_testcase_info(97,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_MASKED;
        pwrRsrcMin = PMIC_TPS6594X_PGOOD_SOURCE_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_PGOOD_SOURCE_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_SRC_MASKED;
        pwrRsrcMin = PMIC_LP8764X_PGOOD_SOURCE_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_PGOOD_SOURCE_BUCK4;
    }

    for(pgoodSrcSel = pwrRsrcMin; pgoodSrcSel <= pwrRsrcMax ; pgoodSrcSel++)
    {
        pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 pgoodSelType);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 &pPgoodSelType_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_MASKED;
        pwrRsrcMin = PMIC_TPS6594X_PGOOD_SOURCE_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_PGOOD_SOURCE_LDO4;

        for(pgoodSrcSel = pwrRsrcMin; pgoodSrcSel <= pwrRsrcMax ; pgoodSrcSel++)
        {
            pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     pgoodSelType);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     &pPgoodSelType_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT_MASKED;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT_MASKED;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             &pPgoodSelType_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT_SOC_MASKED;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT_SOC;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT_SOC_MASKED;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT_SOC;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             &pPgoodSelType_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_TDIE_WARN_MASKED;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_TDIE;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_TDIE_WARN_MASKED;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_TDIE;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             &pPgoodSelType_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_VCCA_DISABLE;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_DISABLE;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             &pPgoodSelType_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_DISABLE;
        pwrRsrcMin = PMIC_LP8764X_PGOOD_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_PGOOD_SOURCE_VMON2;

        for(pgoodSrcSel = pwrRsrcMin; pgoodSrcSel <= pwrRsrcMax ; pgoodSrcSel++)
        {
            pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     pgoodSelType);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     &pPgoodSelType_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);
        }
    }

}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Test pgoodSelType as NRSTOUT
 */
static void test_pmic_powerSetConfigPowerGood_pgoodSelType_nRSTOUT(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;
    uint8_t  pPgoodSelType_rd;

    test_pmic_print_unity_testcase_info(98,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             &pPgoodSelType_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Test pgoodSelType as NRSTOUT SOC
 */
static void test_pmic_powerSetConfigPowerGood_pgoodSelType_nRSTOUTSoc(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;
    uint8_t  pPgoodSelType_rd;

    test_pmic_print_unity_testcase_info(99,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT_SOC;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT_SOC;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT_SOC;
        pgoodSrcSel  = PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT_SOC;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             &pPgoodSelType_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);

}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Test pgoodSelType as TDIE WARN
 */
static void test_pmic_powerSetConfigPowerGood_pgoodSelType_tdieWarn(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;
    uint8_t  pPgoodSelType_rd;

    test_pmic_print_unity_testcase_info(100,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_TDIE_WARN;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_TDIE;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_TDIE_WARN;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_TDIE;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             &pPgoodSelType_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);

}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Test pgoodSelType as VCCA
 */
static void test_pmic_powerSetConfigPowerGood_pgoodSelType_vcca(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;
    uint8_t  pPgoodSelType_rd;

    test_pmic_print_unity_testcase_info(101,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_VCCA_ENABLE;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             &pPgoodSelType_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);

}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Test pgoodSelType as VMON
 */
static void test_pmic_powerSetConfigPowerGood_pgoodSelType_vmon(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;
    uint8_t  pPgoodSelType_rd;
    uint16_t pwrRsrcMin, pwrRsrcMax;

    test_pmic_print_unity_testcase_info(102,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE;
        pwrRsrcMin = PMIC_LP8764X_PGOOD_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_PGOOD_SOURCE_VMON2;

        for(pgoodSrcSel = pwrRsrcMin; pgoodSrcSel <= pwrRsrcMax ; pgoodSrcSel++)
        {
            pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     pgoodSelType);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                                     pgoodSrcSel,
                                                     &pPgoodSelType_rd);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            TEST_ASSERT_EQUAL(pgoodSelType, pPgoodSelType_rd);
        }
    }
}

/*!
 * \brief   Pmic_powerGetPwrRsrcStat : Test get currentLimitLvlStat
 */
static void test_pmic_powerGetPwrRsrcStat_currentLimitLvlStat(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceStat_t pPwrRsrcStatCfg =
    {
        PMIC_POWER_REGULATOR_ILIM_STAT_VALID_SHIFT,
    };

    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;

    test_pmic_print_unity_testcase_info(103,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                              pwrResource,
                                              &pPwrRsrcStatCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                                 pwrResource,
                                                 &pPwrRsrcStatCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }
}

/*!
 * \brief   Pmic_powerGetPwrRsrcStat : Test get underVoltageTholdStat
 */
static void test_pmic_powerGetPwrRsrcStat_underVoltageTholdStat(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceStat_t pPwrRsrcStatCfg =
    {
        PMIC_POWER_RESOURCE_UV_STAT_VALID_SHIFT,
    };

    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;

    test_pmic_print_unity_testcase_info(104,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                             pwrResource,
                                             &pPwrRsrcStatCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                                 pwrResource,
                                                 &pPwrRsrcStatCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                         pwrResource,
                                         &pPwrRsrcStatCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                                 pwrResource,
                                                 &pPwrRsrcStatCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

}

/*!
 * \brief   Pmic_powerGetPwrRsrcStat : Test get overVoltageTholdStat
 */
static void test_pmic_powerGetPwrRsrcStat_overVoltageTholdStat(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceStat_t pPwrRsrcStatCfg =
    {
        PMIC_POWER_RESOURCE_OV_STAT_VALID_SHIFT,
    };

    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;

    test_pmic_print_unity_testcase_info(105,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                             pwrResource,
                                             &pPwrRsrcStatCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                                 pwrResource,
                                                 &pPwrRsrcStatCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                         pwrResource,
                                         &pPwrRsrcStatCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                                 pwrResource,
                                                 &pPwrRsrcStatCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

}

/*!
 * \brief   Pmic_powerGetPwrRsrcStat : Test get overVoltageProtectionLvlStat
 */
static void test_pmic_powerGetPwrRsrcStat_overVoltageProtectionLvlStat(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceStat_t pPwrRsrcStatCfg =
    {
        PMIC_POWER_VCCA_OV_LVL_STAT_VALID_SHIFT,
    };

    uint16_t pwrResource;

    test_pmic_print_unity_testcase_info(106,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                         pwrResource,
                                         &pPwrRsrcStatCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetThermalConfig : Test thermalWarnThold as low
 */
static void test_pmic_powerSetThermalConfig_thermalWarnThold_low(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerThermalCfg_t thermalThreshold =
    {
        PMIC_THERMAL_WARN_VALID_SHIFT,
    };

    Pmic_PowerThermalCfg_t thermalThreshold_rd   =
    {
        PMIC_THERMAL_WARN_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_THERMAL_TEMP_WARN_130C;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold = PMIC_LP8764X_THERMAL_TEMP_WARN_120C;
    }

    test_pmic_print_unity_testcase_info(107,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(thermalThreshold.thermalWarnThold, thermalThreshold_rd.thermalWarnThold);

}

/*!
 * \brief   Pmic_powerSetThermalConfig : Test thermalWarnThold as high
 */
static void test_pmic_powerSetThermalConfig_thermalWarnThold_high(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerThermalCfg_t thermalThreshold =
    {
        PMIC_THERMAL_WARN_VALID_SHIFT,
    };

    Pmic_PowerThermalCfg_t thermalThreshold_rd   =
    {
        PMIC_THERMAL_WARN_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_THERMAL_TEMP_WARN_140C;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold = PMIC_LP8764X_THERMAL_TEMP_WARN_130C;
    }

    test_pmic_print_unity_testcase_info(108,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(thermalThreshold.thermalWarnThold, thermalThreshold_rd.thermalWarnThold);

}

/*!
 * \brief   Pmic_powerSetThermalConfig : Test thermalShutdownThold as low
 */
static void test_pmic_powerSetThermalConfig_thermalShutdownThold_low(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerThermalCfg_t thermalThreshold =
    {
        PMIC_THERMAL_SHTDWN_VALID_SHIFT,
    };

    Pmic_PowerThermalCfg_t thermalThreshold_rd   =
    {
        PMIC_THERMAL_SHTDWN_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalShutdownThold =
                                PMIC_TPS6594X_THERMAL_TEMP_TSD_ORD_140C;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    test_pmic_print_unity_testcase_info(109,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(thermalThreshold.thermalShutdownThold,
                      thermalThreshold_rd.thermalShutdownThold);

}

#if 0
/*!
 * \brief   Pmic_powerSetThermalConfig : Test thermalShutdownThold as high
 */
static void test_pmic_powerSetThermalConfig_thermalShutdownThold_high(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerThermalCfg_t thermalThreshold =
    {
        PMIC_THERMAL_SHTDWN_VALID_SHIFT,
    };

    Pmic_PowerThermalCfg_t thermalThreshold_rd   =
    {
        PMIC_THERMAL_SHTDWN_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalShutdownThold = PMIC_TPS6594X_THERMAL_TEMP_TSD_ORD_145C;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    test_pmic_print_unity_testcase_info(110,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(thermalThreshold.thermalShutdownThold,
                      thermalThreshold_rd.thermalShutdownThold);

}

#endif

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Enable OV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ov_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(111,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_OV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_OV_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                              pwrResource,
                                              intrType,
                                              intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Disable OV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ov_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(112,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_OV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_OV_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                              pwrResource,
                                              intrType,
                                              intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }
}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Enable UV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_uv_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(113,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_UV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_UV_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                              pwrResource,
                                              intrType,
                                              intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

}
/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Disable UV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_uv_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(114,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_UV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_UV_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                              pwrResource,
                                              intrType,
                                              intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Enable ILIM interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ilim_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(115,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_ILIM_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Disable ILIM interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ilim_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(116,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_ILIM_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin  = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax  = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }
}

/*!
 * \brief   Pmic_powerSetIntr : Test Enable TWARN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_twarn_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(117,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_TWARN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_TWARN;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Disable TWARN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_twarn_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(118,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_TWARN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_TWARN;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Enable NRSTOUT_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_readback_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(119,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_NRSTOUT_READBACK;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Disable NRSTOUT_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_readback_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(120,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_NRSTOUT_READBACK;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Enable SOC_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(121,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_SOC_PWR_ERR;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_SOC_PWR_ERR;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Disable SOC_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(122,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_SOC_PWR_ERR;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_SOC_PWR_ERR;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Enable MCU_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(123,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_MCU_PWR_ERR;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_MCU_PWR_ERR;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Disable MCU_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(124,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_MCU_PWR_ERR;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_MCU_PWR_ERR;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Enable ORD_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ord_shutdown_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(125,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_ORD_SHUTDOWN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_ORD_SHUTDOWN;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Disable ORD_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ord_shutdown_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(126,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_ORD_SHUTDOWN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_ORD_SHUTDOWN;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Enable IMM_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_imm_shutdown_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(127,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_IMM_SHUTDOWN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_IMM_SHUTDOWN;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Disable IMM_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_imm_shutdown_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(128,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_IMM_SHUTDOWN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_IMM_SHUTDOWN;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Enable NRSTOUT_SOC_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(129,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_SOC_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_NRSTOUT_SOC_READBACK;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Disable NRSTOUT_SOC_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(130,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_SOC_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_NRSTOUT_SOC_READBACK;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Enable EN_DRV_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_en_drv_readback_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(131,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_EN_DRV_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_EN_DRV_READBACK;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Disable EN_DRV_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_en_drv_readback_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(132,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_EN_DRV_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_EN_DRV_READBACK;
    }

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test deglitchTimeSel as 4us
 */
static void test_pmic_powerSetCommonConfig_deglitchTimeSel_4(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(133,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrCommonCfg.deglitchTimeSel =
                                  PMIC_TPS6594X_POWER_RESOURCE_DEGLITCH_SEL_4US;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrCommonCfg.deglitchTimeSel =
                                   PMIC_LP8764X_POWER_RESOURCE_DEGLITCH_SEL_4US;
    }
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.deglitchTimeSel, powerCfg_rd.deglitchTimeSel);

}

#if 0
/*!
 * \brief   Pmic_powerSetCommonConfig : Test deglitchTimeSel as 20us
 */
static void test_pmic_powerSetCommonConfig_deglitchTimeSel_20(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(134,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrCommonCfg.deglitchTimeSel =
                                  PMIC_TPS6594X_POWER_RESOURCE_DEGLITCH_SEL_20US;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrCommonCfg.deglitchTimeSel =
                                   PMIC_LP8764X_POWER_RESOURCE_DEGLITCH_SEL_20US;
    }
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.deglitchTimeSel, powerCfg_rd.deglitchTimeSel);

}
#endif

/*!
 * \brief   Pmic_powerSetCommonConfig : Test severeErrorTrig as Immediate shutdown
 */
static void test_pmic_powerSetCommonConfig_severeErrorTrig_imm(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(135,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.severeErrorTrig, powerCfg_rd.severeErrorTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test severeErrorTrig as oderly shutdown
 */
static void test_pmic_powerSetCommonConfig_severeErrorTrig_odrShtDwn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(136,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.severeErrorTrig, powerCfg_rd.severeErrorTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test severeErrorTrig as mcu power error
 */
static void test_pmic_powerSetCommonConfig_severeErrorTrig_McuPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(137,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.severeErrorTrig, powerCfg_rd.severeErrorTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test severeErrorTrig as soc power error
 */
static void test_pmic_powerSetCommonConfig_severeErrorTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(138,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.severeErrorTrig, powerCfg_rd.severeErrorTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test otherRailTrig as Immediate shutdown
 */
static void test_pmic_powerSetCommonConfig_otherRailTrig_imm(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_OTHER_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_OTHER_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(139,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.otherRailTrig, powerCfg_rd.otherRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test otherRailTrig as oderly shutdown
 */
static void test_pmic_powerSetCommonConfig_otherRailTrig_odrShtDwn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_OTHER_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_OTHER_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(140,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.otherRailTrig, powerCfg_rd.otherRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test otherRailTrig as mcu power error
 */
static void test_pmic_powerSetCommonConfig_otherRailTrig_McuPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_OTHER_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_OTHER_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(141,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.otherRailTrig, powerCfg_rd.otherRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test otherRailTrig as soc power error
 */
static void test_pmic_powerSetCommonConfig_otherRailTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_OTHER_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_OTHER_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(142,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.otherRailTrig, powerCfg_rd.otherRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test socRailTrig as Immediate shutdown
 */
static void test_pmic_powerSetCommonConfig_socRailTrig_imm(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_SOC_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SOC_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(143,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.socRailTrig, powerCfg_rd.socRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test socRailTrig as oderly shutdown
 */
static void test_pmic_powerSetCommonConfig_socRailTrig_odrShtDwn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_SOC_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SOC_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(144,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.socRailTrig, powerCfg_rd.socRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test socRailTrig as mcu power error
 */
static void test_pmic_powerSetCommonConfig_socRailTrig_McuPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_SOC_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SOC_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(145,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.socRailTrig, powerCfg_rd.socRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test socRailTrig as soc power error
 */
static void test_pmic_powerSetCommonConfig_socRailTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_SOC_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SOC_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(146,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.socRailTrig, powerCfg_rd.socRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test mcuRailTrig as Immediate shutdown
 */
static void test_pmic_powerSetCommonConfig_mcuRailTrig_imm(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_MCU_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MCU_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(147,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.mcuRailTrig, powerCfg_rd.mcuRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test mcuRailTrig as oderly shutdown
 */
static void test_pmic_powerSetCommonConfig_mcuRailTrig_odrShtDwn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_MCU_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MCU_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(148,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.mcuRailTrig, powerCfg_rd.mcuRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test mcuRailTrig as mcu power error
 */
static void test_pmic_powerSetCommonConfig_mcuRailTrig_McuPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_MCU_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MCU_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(149,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.mcuRailTrig, powerCfg_rd.mcuRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test mcuRailTrig as soc power error
 */
static void test_pmic_powerSetCommonConfig_mcuRailTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_MCU_RAIL_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MCU_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(150,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.mcuRailTrig, powerCfg_rd.mcuRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test moderateRailTrig as Immediate shutdown
 */
static void test_pmic_powerSetCommonConfig_moderateRailTrig_imm(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(151,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.moderateRailTrig, powerCfg_rd.moderateRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test moderateRailTrig as oderly shutdown
 */
static void test_pmic_powerSetCommonConfig_moderateRailTrig_odrShtDwn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(152,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.moderateRailTrig, powerCfg_rd.moderateRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test moderateRailTrig as mcu power error
 */
static void test_pmic_powerSetCommonConfig_moderateRailTrig_McuPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(153,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.moderateRailTrig, powerCfg_rd.moderateRailTrig);

}

/*!
 * \brief   Pmic_powerSetCommonConfig : Test moderateRailTrig as soc power error
 */
static void test_pmic_powerSetCommonConfig_moderateRailTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(154,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.moderateRailTrig, powerCfg_rd.moderateRailTrig);

}

/*!
 * \brief   Pmic_powerSetLdoRtc : Test Enable ldortcRegulator
 */
static void test_pmic_powerSetLdoRtc_ldortcEnable_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool ldortcEnable;
    bool ldortcEnable_rd;

    test_pmic_print_unity_testcase_info(155,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        ldortcEnable = PMIC_TPS6594X_REGULATOR_LDORTC_ENABLE;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_powerSetLdoRtc(pPmicCoreHandle, ldortcEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_powerGetLdoRtc(pPmicCoreHandle, &ldortcEnable_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(ldortcEnable, ldortcEnable_rd);

}

/*!
 * \brief   Pmic_powerSetLdoRtc : Test Disable ldortcRegulator
 */
static void test_pmic_powerSetLdoRtc_ldortcEnable_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool ldortcEnable;
    bool ldortcEnable_rd;

    test_pmic_print_unity_testcase_info(156,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        ldortcEnable = PMIC_TPS6594X_REGULATOR_LDORTC_DISABLE;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_powerSetLdoRtc(pPmicCoreHandle, ldortcEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_powerGetLdoRtc(pPmicCoreHandle, &ldortcEnable_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(ldortcEnable, ldortcEnable_rd);

}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr :  Parameter validation for Power Resource for intrEnable.
 */
static void test_Pmic_powerSetPwrRsrcIntrPrmValTest_PwrRsrc_intrEnable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    uint16_t pwrResource;
    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT;
        pwrResource = PMIC_LP8764X_VMON_MAX + 1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT;
        pwrResource = PMIC_TPS6594X_LDO_MAX + 1;
    }

    test_pmic_print_unity_testcase_info(157,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Parameter validation for intrType
 */
static void test_pmic_powerSetPowerResourceConfigPrmValTest_intrType(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(158,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_ILIM_INT + 1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin  = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax  = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
        }
    }
}

/*!
 * \brief   Pmic_powerSetCommonConfig : Parameter validation for severeErrorTrig
 */
static void test_pmic_powerSetCommonConfigPrmValTest_severeErrorTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(159,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetCommonConfig : Parameter validation for otherRailTrig
 */
static void test_pmic_powerSetCommonConfigPrmValTest_otherRailTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_OTHER_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(160,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetCommonConfig : Parameter validation for socRailTrig
 */
static void test_pmic_powerSetCommonConfigPrmValTest_socRailTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_SOC_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(161,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetCommonConfig : Parameter validation for mcuRailTrig
 */
static void test_pmic_powerSetCommonConfigPrmValTest_mcuRailTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MCU_RAIL_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(162,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetCommonConfig : Parameter validation for moderateRailTrig
 */
static void test_pmic_powerSetCommonConfigPrmValTest_moderateRailTrig_SocPwrErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(163,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

}

/*!
 * \brief   Pmic_powerGetCommonConfig : Parameter validation for handle
 */
static void test_pmic_powerGetCommonConfigPrmValTest_handle(void)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_POWER_PGOOD_WINDOW_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(166,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetCommonConfig(NULL,  &pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for handle
 */
static void test_pmic_powerSetConfigPowerGoodgPrmValTest_handle(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(167,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_VCCA_ENABLE;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(NULL,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Pmic_powerGetConfigPowerGood : Parameter validation for handle
 */
static void test_pmic_powerGetConfigPowerGoodPrmValTest_handle(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(168,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_VCCA_ENABLE;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerGetConfigPowerGood(NULL,
                                             pgoodSrcSel,
                                             &pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Pmic_powerGetPwrRsrcStat : Parameter validation for handle
 */
static void test_pmic_powerGetPwrRsrcStatPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceStat_t pPwrRsrcStatCfg =
    {
        PMIC_POWER_REGULATOR_ILIM_STAT_VALID_SHIFT,
    };

    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;

    test_pmic_print_unity_testcase_info(169,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerGetPwrRsrcStat(NULL,
                                              pwrResource,
                                              &pPwrRsrcStatCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
    }
}

/*!
 * \brief   Pmic_powerSetThermalConfig : Parameter validation for handle
 */
static void test_pmic_powerSetThermalConfigPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerThermalCfg_t thermalThreshold =
    {
        PMIC_THERMAL_WARN_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold =
                                       PMIC_TPS6594X_THERMAL_TEMP_WARN_140C + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold =
                                        PMIC_LP8764X_THERMAL_TEMP_WARN_130C + 1;
    }

    test_pmic_print_unity_testcase_info(170,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetThermalConfig(NULL, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Pmic_powerGetThermalConfig : Parameter validation for handle
 */
static void test_pmic_powerGetThermalConfigPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerThermalCfg_t thermalThreshold =
    {
        PMIC_THERMAL_WARN_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold =
                                       PMIC_TPS6594X_THERMAL_TEMP_WARN_140C + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold =
                                        PMIC_LP8764X_THERMAL_TEMP_WARN_130C + 1;
    }

    test_pmic_print_unity_testcase_info(171,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetThermalConfig(NULL, &thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Parameter validation for handle
 */
static void test_Pmic_powerSetPwrRsrcIntrConfigPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    uint16_t pwrResource;
    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)

    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT;
        pwrResource = PMIC_LP8764X_VMON_MAX;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)

    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT;
        pwrResource = PMIC_TPS6594X_LDO_MAX ;
    }

    test_pmic_print_unity_testcase_info(172,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrRsrcIntr(NULL,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Parameter validation for handle
 */
static void test_pmic_powerSetPwrRsrcIntrPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(173,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_EN_DRV_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_EN_DRV_READBACK;
    }

    pmicStatus = Pmic_powerSetIntr(NULL, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*   The below taest cases aer dummy, as power related interrupts cannot be generated
 *   to test masking APIs
 */
#if 0
/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Enable OV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ov_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;
    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(174,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_OV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_OV_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }
    /* The test code expects a over voltage interrupt */
    //dummy_func_generate_over_voltage();

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == status) &&
           ((errStat.intStatus[PMIC_TPS6594X_POWER_OV_INT/32U] &
             (1U << (PMIC_TPS6594X_POWER_OV_INT % 32U))) != 0U))
        {
            while(1)
            {
                status = Pmic_getNextErrorStatus(pHandle,
                                                 &errStat,
                                                 &irqNum);
                if(PMIC_TPS6594X_POWER_OV_INT == irqNum)
                    break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[PMIC_TPS6594X_POWER_OV_INT/32U] &
                 (1U << (PMIC_TPS6594X_POWER_OV_INT % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(PMIC_TPS6594X_POWER_OV_INT == irqNum)
                        break;
                }
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[PMIC_TPS6594X_POWER_OV_INT/32U] &
         (1U << (PMIC_TPS6594X_POWER_OV_INT % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(PMIC_TPS6594X_POWER_OV_INT == irqNum)
                break;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                              pwrResource,
                                              intrType,
                                              intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[PMIC_TPS6594X_POWER_OV_INT/32U] &
                 (1U << (PMIC_TPS6594X_POWER_OV_INT % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(PMIC_TPS6594X_POWER_OV_INT == irqNum)
                        break;
                }
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Disable OV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ov_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(175,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_OV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_OV_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }
    /* The test code expects a over voltage interrupt */
    //dummy_func_generate_over_voltage();

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == status) &&
           ((errStat.intStatus[intrType/32U] &
             (1U << (intrType % 32U))) != 0U))
        {
            while(1)
            {
                status = Pmic_getNextErrorStatus(pHandle,
                                                 &errStat,
                                                 &irqNum);
                if(intrType == irqNum)
                {
                    pmicStatus = PMIC_ST_ERR_INV_PARAM;
                    break;
                }
            }
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                              pwrResource,
                                              intrType,
                                              intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == status) &&
           ((errStat.intStatus[intrType/32U] &
             (1U << (intrType % 32U))) != 0U))
        {
            while(1)
            {
                status = Pmic_getNextErrorStatus(pHandle,
                                                 &errStat,
                                                 &irqNum);
                if(intrType == irqNum)
                {
                    pmicStatus = PMIC_ST_ERR_INV_PARAM;
                    break;
                }
            }
        }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }
}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Enable UV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_uv_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(176,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_UV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_UV_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }
    /* The test code expects a under voltage interrupt */
    //dummy_func_generate_under_voltage();

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == status) &&
           ((errStat.intStatus[intrType/32U] &
             (1U << (intrType % 32U))) != 0U))
        {
            while(1)
            {
                status = Pmic_getNextErrorStatus(pHandle,
                                                 &errStat,
                                                 &irqNum);
                if(intrType == irqNum)
                    break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                        break;
                }
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                        break;
                }
            }
        }
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}
/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Disable UV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_uv_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(177,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_UV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_UV_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }
    /* The test code expects a under voltage interrupt */
    //dummy_func_generate_under_voltage();

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                pwrResource,
                                                intrType,
                                                intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Enable ILIM interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ilim_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(178,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_ILIM_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }
    /* The test code expects a current limit interrupt */
    //dummy_func_generate_current_limit();

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == status) &&
           ((errStat.intStatus[intrType/32U] &
             (1U << (intrType % 32U))) != 0U))
        {
            while(1)
            {
                status = Pmic_getNextErrorStatus(pHandle,
                                                 &errStat,
                                                 &irqNum);
                if(intrType == irqNum)
                    break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                        break;
                }
            }
        }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Disable ILIM interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ilim_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(179,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_ILIM_INT;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin  = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax  = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }
    /* The test code expects a current limit interrupt */
    //dummy_func_generate_current_limit();

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == status) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                while(1)
                {
                    status = Pmic_getNextErrorStatus(pHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable TWARN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_twarn_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(180,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_TWARN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_TWARN;
    }
    /* The test code expects a thermal warning interrupt */
    //dummy_func_generate_thermal_warning_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable TWARN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_twarn_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(181,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_TWARN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_TWARN;
    }
    /* The test code expects a thermal warning interrupt */
    //dummy_func_generate_thermal_warning_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable NRSTOUT_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_readback_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(182,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_NRSTOUT_READBACK;
    }
    /* The test code expects a nRstOut Readback interrupt */
    //dummy_func_generate_nRstOut_RbBck_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable NRSTOUT_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_readback_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(183,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_NRSTOUT_READBACK;
    }
    /* The test code expects a nRstOut Readback interrupt */
    //dummy_func_generate_nRstOut_RbBck_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable SOC_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(184,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_SOC_PWR_ERR;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_SOC_PWR_ERR;
    }
    /* The test code expects a SOC_PWR_ERR interrupt */
    //dummy_func_generate_soc_pwr_err_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable SOC_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(185,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_SOC_PWR_ERR;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_SOC_PWR_ERR;
    }
    /* The test code expects a SOC_PWR_ERR interrupt */
    //dummy_func_generate_soc_pwr_err_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable MCU_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(186,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_MCU_PWR_ERR;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_MCU_PWR_ERR;
    }
    /* The test code expects a MCU_PWR_ERR interrupt */
    //dummy_func_generate_mcu_pwr_err_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable MCU_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(187,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_MCU_PWR_ERR;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_MCU_PWR_ERR;
    }
    /* The test code expects a MCU_PWR_ERR interrupt */
    //dummy_func_generate_mcu_pwr_err_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable ORD_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ord_shutdown_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(188,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_ORD_SHUTDOWN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_ORD_SHUTDOWN;
    }
    /* The test code expects a ORD_SHUTDOWN interrupt */
    //dummy_func_generate_ord_shutdown_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable ORD_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ord_shutdown_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(190
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_ORD_SHUTDOWN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_ORD_SHUTDOWN;
    }
    /* The test code expects a ORD_SHUTDOWN interrupt */
    //dummy_func_generate_ord_shutdown_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable IMM_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_imm_shutdown_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(191,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_IMM_SHUTDOWN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_IMM_SHUTDOWN;
    }
    /* The test code expects a IMM_SHUTDOWN interrupt */
    //dummy_func_generate_imm_shutdown_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable IMM_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_imm_shutdown_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(192,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_IMM_SHUTDOWN;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_IMM_SHUTDOWN;
    }
    /* The test code expects a IMM_SHUTDOWN interrupt */
    //dummy_func_generate_imm_shutdown_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable NRSTOUT_SOC_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(193,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_SOC_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_NRSTOUT_SOC_READBACK;
    }
    /* The test code expects a NRSTOUT_SOC_READBACK interrupt */
    //dummy_func_generate_nrstout_soc_readback_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable NRSTOUT_SOC_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(194,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_NRSTOUT_SOC_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_NRSTOUT_SOC_READBACK;
    }
    /* The test code expects a NRSTOUT_SOC_READBACK interrupt */
    //dummy_func_generate_nrstout_soc_readback_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable EN_DRV_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_en_drv_readback_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(195,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_EN_DRV_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_EN_DRV_READBACK;
    }
    /* The test code expects a EN_DRV_READBACK interrupt */
    //dummy_func_generate_en_drv_readback_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable EN_DRV_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_en_drv_readback_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum = 0U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(195,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_INTERRUPT_EN_DRV_READBACK;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_INTERRUPT_EN_DRV_READBACK;
    }
    /* The test code expects a EN_DRV_READBACK interrupt */
    //dummy_func_generate_en_drv_readback_irq();

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(1)
        {
            status = Pmic_getNextErrorStatus(pHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

#endif

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run power unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_handle);
RUN_TEST(test_pmic_powerGetPowerResourceConfigPrmValTest_handle);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_rvCheckEn_enable);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_rvCheckEn_disable);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_rvCheckEn);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckPullDownEn_enable);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckPullDownEn_disable);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckPullDownEn);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_vmonEn_enable);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_vmonEn_disable);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_vmonEn);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVoutSel_vout1);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVoutSel_vout2);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckVoutSel);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckFpwmMode_auto);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckFpwmMode_pwm);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckFpwmMode);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckFpwmMpMode_multiPhase);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckFpwmMpMode_auto);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckFpwmMpMode);
#if 0
RUN_TEST(test_pmic_powerSetPowerResourceConfig_regulatorEn_disable);
#endif
RUN_TEST(test_pmic_powerSetPowerResourceConfig_regulatorEn_enable);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_regulatorEn);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_6A5);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_5A5);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_4A5);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_3A5);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckCurrentLimit_2A5);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckCurrentLimit);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_buckCurrentLimit);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_33MV);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_20MV);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_10MV);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_2MV5);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_1MV3);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_0MV63);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_05MV);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckVmonSlewRate_0MV31);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckVmonSlewRate);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_buckVmonSlewRate);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoPullDownSel_50KOHM);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoPullDownSel_125OHM);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoPullDownSel_250OHM);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoPullDownSel_500OHM);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_ldoPullDownSel);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_ldoPullDownSel);
#if 0
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoSlowRampEn_enable);
#endif
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoSlowRampEn_disable);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_ldoSlowRampEn);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_voltage_mV);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_voltage_mV);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_voltage_mV);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_vccaPwrGudLvl_5V);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_vccaPwrGudLvl_3V3);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_vccaPwrGudLvl);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_railGrpSel_none);
#if 0
RUN_TEST(test_pmic_powerSetPowerResourceConfig_railGrpSel_mcu);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_railGrpSel_soc);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_railGrpSel_other);
#endif
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_railGrpSel);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_railGrpSel);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckFreqSel_2M2);
#if 0
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckFreqSel_4M4);
#endif
RUN_TEST(test_pmic_powerSetPowerResourceConfig_buckFreqSel_8M8);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_buckFreqSel);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_buckFreqSel);
#if 0
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoBypassModeEn_bypass);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoBypassModeEn_linear);
#endif
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_ldoBypassModeEn);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_0MS5);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_1MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_1MS5);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_2MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_2MS5);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_3MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_3MS5);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_4MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_6MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_8MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_10MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_12MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_14MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoRvTimeoutSel_16MS);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_ldoRvTimeoutSel);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_ldoRvTimeoutSel);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_vmonRange_range1);
RUN_TEST(test_pmic_powerSetPowerResourceConfig_vmonRange_range2);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_vmonRange);
RUN_TEST(test_pmic_powerGetPowerResourceConfigPrmValTest_Pmic_PowerResourceCfg_t);
RUN_TEST(test_pmic_powerSetCommonConfigPrmValTest_handle);
RUN_TEST(test_pmic_powerGetCommonConfigPrmValTest_handle);
RUN_TEST(test_pmic_powerSetCommonConfig_pgoodWindow_uv);
RUN_TEST(test_pmic_powerSetCommonConfig_pgoodWindow_uv_ov);
RUN_TEST(test_pmic_powerSetCommonConfig_pgoodPolarity_high);
RUN_TEST(test_pmic_powerSetCommonConfig_pgoodPolarity_low);
RUN_TEST(test_pmic_powerSetConfigPowerGood_pgoodSelType_voltageCurrent);
RUN_TEST(test_pmic_powerSetConfigPowerGood_pgoodSelType_voltage);
RUN_TEST(test_pmic_powerSetConfigPowerGood_pgoodSelType_masked);
RUN_TEST(test_pmic_powerSetConfigPowerGood_pgoodSelType_nRSTOUT);
RUN_TEST(test_pmic_powerSetConfigPowerGood_pgoodSelType_nRSTOUTSoc);
RUN_TEST(test_pmic_powerSetConfigPowerGood_pgoodSelType_tdieWarn);
RUN_TEST(test_pmic_powerSetConfigPowerGood_pgoodSelType_vcca);
RUN_TEST(test_pmic_powerSetConfigPowerGood_pgoodSelType_vmon);
RUN_TEST(test_pmic_powerGetPwrRsrcStat_currentLimitLvlStat);
RUN_TEST(test_pmic_powerGetPwrRsrcStat_underVoltageTholdStat);
RUN_TEST(test_pmic_powerGetPwrRsrcStat_overVoltageTholdStat);
RUN_TEST(test_pmic_powerGetPwrRsrcStat_overVoltageProtectionLvlStat);
RUN_TEST(test_pmic_powerSetThermalConfig_thermalWarnThold_low);
RUN_TEST(test_pmic_powerSetThermalConfig_thermalWarnThold_high);
RUN_TEST(test_pmic_powerSetThermalConfig_thermalShutdownThold_low);
#if 0
RUN_TEST(test_pmic_powerSetThermalConfig_thermalShutdownThold_high);
#endif
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ov_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ov_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_uv_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_uv_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ilim_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ilim_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_twarn_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_twarn_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_readback_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_readback_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ord_shutdown_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ord_shutdown_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_imm_shutdown_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_imm_shutdown_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_en_drv_readback_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_en_drv_readback_disable);
RUN_TEST(test_pmic_powerSetCommonConfig_deglitchTimeSel_4);
#if 0
RUN_TEST(test_pmic_powerSetCommonConfig_deglitchTimeSel_20);
#endif
RUN_TEST(test_pmic_powerSetCommonConfig_severeErrorTrig_imm);
RUN_TEST(test_pmic_powerSetCommonConfig_severeErrorTrig_odrShtDwn);
RUN_TEST(test_pmic_powerSetCommonConfig_severeErrorTrig_McuPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfig_severeErrorTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfig_otherRailTrig_imm);
RUN_TEST(test_pmic_powerSetCommonConfig_otherRailTrig_odrShtDwn);
RUN_TEST(test_pmic_powerSetCommonConfig_otherRailTrig_McuPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfig_otherRailTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfig_socRailTrig_imm);
RUN_TEST(test_pmic_powerSetCommonConfig_socRailTrig_odrShtDwn);
RUN_TEST(test_pmic_powerSetCommonConfig_socRailTrig_McuPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfig_socRailTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfig_mcuRailTrig_imm);
RUN_TEST(test_pmic_powerSetCommonConfig_mcuRailTrig_odrShtDwn);
RUN_TEST(test_pmic_powerSetCommonConfig_mcuRailTrig_McuPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfig_mcuRailTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfig_moderateRailTrig_imm);
RUN_TEST(test_pmic_powerSetCommonConfig_moderateRailTrig_odrShtDwn);
RUN_TEST(test_pmic_powerSetCommonConfig_moderateRailTrig_McuPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfig_moderateRailTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetLdoRtc_ldortcEnable_enable);
RUN_TEST(test_pmic_powerSetLdoRtc_ldortcEnable_disable);
RUN_TEST(test_Pmic_powerSetPwrRsrcIntrPrmValTest_PwrRsrc_intrEnable);
RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_intrType);
RUN_TEST(test_pmic_powerSetCommonConfigPrmValTest_severeErrorTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfigPrmValTest_otherRailTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfigPrmValTest_socRailTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfigPrmValTest_mcuRailTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetCommonConfigPrmValTest_moderateRailTrig_SocPwrErr);
RUN_TEST(test_pmic_powerSetConfigPowerGoodgPrmValTest_handle);
RUN_TEST(test_pmic_powerGetConfigPowerGoodPrmValTest_handle);
RUN_TEST(test_pmic_powerGetPwrRsrcStatPrmValTest_handle);
RUN_TEST(test_pmic_powerSetThermalConfigPrmValTest_handle);
RUN_TEST(test_pmic_powerGetThermalConfigPrmValTest_handle);
RUN_TEST(test_Pmic_powerSetPwrRsrcIntrConfigPrmValTest_handle);
RUN_TEST(test_pmic_powerSetPwrRsrcIntrPrmValTest_handle);
/*   The below taest cases aer dummy, as power related interrupts cannot be generated
 *   to test masking APIs
 */
#if 0
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ov_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ov_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_uv_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_uv_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ilim_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ilim_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_twarn_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_twarn_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_readback_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_readback_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ord_shutdown_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ord_shutdown_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_imm_shutdown_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_imm_shutdown_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_disable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_en_drv_readback_enable);
RUN_TEST(test_pmic_powerSetPwrRsrcIntr_en_drv_readback_disable);
#endif
    UNITY_END();
}

/*!
 * \brief   POWER Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_power_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType     = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode           = PMIC_INTF_DUAL_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr          = LEO_PMICA_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr        = LEO_PMICA_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead   = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite  = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop  = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

/*!
 * \brief   POWER Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_power_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType     = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode           = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr          = LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr        = LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead   = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite  = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop  = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

/*!
 * \brief   POWER Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_power_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType     = PMIC_DEV_HERA_LP8764X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode           = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr          = HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr        = HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead   = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite  = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop  = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM)"
    " \r\n 2: Pmic Hera device"
    " \r\n 3: quit"
    " \r\n"
    " \r\n Enter option: "
};

/*!
 * \brief   Function to register POWER Unity Test App wrapper to Unity framework
 */
static void test_pmic_power_testapp_runner(void)
{
    /* @description : Test runner for POWER Test App
     *
     * @requirements: PDK-5841
                      PDK-5850
     *
     * @cores       : mcu1_0, mcu1_1
     */

    int8_t num = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestAppMenu);
        if(UART_scanFmt("%d", &num) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }
        switch(num)
        {
           case 0U:
               /* POWER Unity Test App wrapper Function for LEO PMIC-A */
               test_pmic_leo_pmicA_power_testApp();
               /* Run power test cases for Leo PMIC-A */
               test_pmic_run_testcases();
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 1U:
               /* POWER Unity Test App wrapper Function for LEO PMIC-B */
               test_pmic_leo_pmicB_power_testApp();
               /* Run power test cases for Leo PMIC-B */
               test_pmic_run_testcases();
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 2U:
               /* POWER Unity Test App wrapper Function for HERA PMIC */
               test_pmic_hera_power_testApp();
               /* Run power test cases for Hera PMIC */
               test_pmic_run_testcases();
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 3U:
               pmic_log(" \r\n Quit from application\n");
               return;
           default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}
#endif

/*!
 * \brief   TI RTOS specific POWER TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    test_pmic_uartInit();

    pmic_log("PMIC POWER Unit Test started...\n");

    pmic_log("%s(): %d: %s(%s)\n", __func__, __LINE__, __TIME__, __DATE__);

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_power_testapp_runner();
#endif
}
