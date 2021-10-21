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

extern uint16_t pmic_device_info;
extern int32_t gCrcTestFlag_J721E;
extern int32_t gCrcTestFlag_J7VCL;

extern int8_t gLdoBypassModeEnTestFlag;
extern int8_t gThermalStatusTestFlag;
extern int8_t gPwrRsrcStatusTestFlag;

extern Pmic_Ut_FaultInject_t gPmic_faultInjectCfg;

/*!
 * \brief   PMIC POWER Test Cases
 */
static Pmic_Ut_Tests_t pmic_power_tests[] =
{
    /*! testID
     *  testDesc
     */
    {
        7130,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for handle"
    },
    {
        7299,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for handle"
    },
    {
        7131,
        "Pmic_powerSetPwrResourceCfg : Test Enable residual voltage checking"
    },
    {
        7132,
        "Pmic_powerSetPwrResourceCfg : Test Disable residual voltage checking"
    },
    {
        7133,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for rvCheckEn."
    },
    {
        7134,
        "Pmic_powerSetPwrResourceCfg : Test Enable buck pull down checking for Buck regulator"
    },
    {
        7135,
        "Pmic_powerSetPwrResourceCfg : Test Disable buck pull down checking for Buck regulator"
    },
    {
        7136,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckPullDownEn."
    },
    {
        7137,
        "Pmic_powerSetPwrResourceCfg : Test Enable the Voltage monitor."
    },
    {
        7138,
        "Pmic_powerSetPwrResourceCfg : Test Disable the Voltage monitor."
    },
    {
        7139,
        "Pmic_powerSetPwrResourceCfg :  Parameter validation for Power Resource for vmonEn."
    },
    {
        7143,
        "Pmic_powerSetPwrResourceCfg : Test Select output voltage register for buck as VOUT1."
    },
    {
        7144,
        "Pmic_powerSetPwrResourceCfg : Test Select output voltage register for buck as VOUT2."
    },
    {
        7145,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckVoutSel."
    },
    {
        7146,
        "Pmic_powerSetPwrResourceCfg : Test Select Auto Mode for BUCK"
    },
    {
        7147,
        "Pmic_powerSetPwrResourceCfg : Test Select PWM Mode for BUCK"
    },
    {
        7148,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckFpwmMode."
    },
    {
        7149,
        "Pmic_powerSetPwrResourceCfg : Test Select Multi phase with PWM Mode for BUCK"
    },
    {
        7150,
        "Pmic_powerSetPwrResourceCfg : Test Select AUTO Mode with Automatic phase adding and shedding for BUCK"
    },
    {
        7151,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckFpwmMpMode."
    },
    {
        7152,
        "Pmic_powerSetPwrResourceCfg : Test Disable the power regulator."
    },
    {
        7153,
        "Pmic_powerSetPwrResourceCfg : Test Enable the power regulator."
    },
    {
        7154,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for regulatorEn."
    },
    {
        7155,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 6.5Amp."
    },
    {
        7156,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 5.5Amp."
    },
    {
        7157,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 4.5Amp."
    },
    {
        7158,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 3.5Amp."
    },
    {
        7159,
        "Pmic_powerSetPwrResourceCfg : Test Switch peak current limit for BUCK regulator for 2.5Amp."
    },
    {
        7160,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckCurrentLimit."
    },
    {
        7161,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for buckCurrentLimit."
    },
    {
        7162,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 33MV."
    },
    {
        7163,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 20MV."
    },
    {
        7164,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 10MV."
    },
    {
        7165,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 5MV."
    },
    {
        7166,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 2.5MV."
    },
    {
        7167,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 1.3MV."
    },
    {
        7168,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 0.63MV."
    },
    {
        7169,
        "Pmic_powerSetPwrResourceCfg : Test Output voltage slew rate for BUCK & VMON for 0.31MV."
    },
    {
        7170,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for buckVmonSlewRate."
    },
    {
        7171,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for buckVmonSlewRate."
    },
    {
        7172,
        "Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 50Kohm."
    },
    {
        7173,
        "Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 125Kohm"
    },
    {
        7174,
        "Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 250Kohm"
    },
    {
        7175,
        "Pmic_powerSetPwrResourceCfg : Test Selects the resistor value for output pull-down resistor for LDO for 500Kohm"
    },
    {
        7176,
        "Pmic_powerSetPwrResourceCfg : Test Enable Slow Ramp for LDO"
    },
    {
        7177,
        "Pmic_powerSetPwrResourceCfg : Test Disable Slow Ramp for LDO"
    },
    {
        7178,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoPullDownSel."
    },
    {
        7179,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for ldoPullDownSel."
    },
    {
        7180,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoSlowRampEn."
    },
    {
        7181,
        "Pmic_powerSetPwrResourceCfg : Test Voltage level in miliVolts for regulators"
    },
    {
        7182,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for voltage_mV."
    },
    {
        7183,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for voltage_mV max value."
    },
    {
        7184,
        "Pmic_powerSetPwrResourceCfg : Test Powergood level 5V for VCCA pin"
    },
    {
        7185,
        "Pmic_powerSetPwrResourceCfg : Test Powergood level 3.3V for VCCA pin"
    },
    {
        7186,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for vccaPwrGudLvl."
    },
    {
        7187,
        "Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group none."
    },
    {
        7188,
        "Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group mcu."
    },
    {
        7189,
        "Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group soc."
    },
    {
        7190,
        "Pmic_powerSetPwrResourceCfg : Test Rail group selection for power resources as group other."
    },
    {
        7191,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for railGrpSel."
    },
    {
        7192,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for railGrpSel."
    },
    {
        7198,
        "Pmic_powerSetPwrResourceCfg : Test LDO Bypass Regulator LDO mode"
    },
    {
        7199,
        "Pmic_powerSetPwrResourceCfg : Test LDO Linear Regulator LDO mode"
    },
    {
        7200,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoBypassModeEn."
    },
    {
        7201,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 0.5MS."
    },
    {
        7202,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 1MS."
    },
    {
        7203,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 1.5MS."
    },
    {
        7204,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 2MS."
    },
    {
        7205,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 2.5MS."
    },
    {
        7206,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 3MS."
    },
    {
        7207,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 3.5MS."
    },
    {
        7208,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 4MS."
    },
    {
        7209,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 6MS."
    },
    {
        7210,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 8MS."
    },
    {
        7211,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 10MS."
    },
    {
        7212,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 12MS."
    },
    {
        7213,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 14MS."
    },
    {
        7214,
        "Pmic_powerSetPwrResourceCfg : Test LDO residual voltage check timeout select for 16MS."
    },
    {
        7215,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for ldoRvTimeoutSel."
    },
    {
        7216,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for ldoRvTimeoutSel."
    },
    {
        7217,
        "Pmic_powerSetPwrResourceCfg : Test OV/UV voltage monitoring range for vmonRange for range1."
    },
    {
        7218,
        "Pmic_powerSetPwrResourceCfg : Test OV/UV voltage monitoring range for vmonRange for range2."
    },
    {
        7219,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Power Resource for vmonRange."
    },
    {
        7300,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for Pmic_PowerResourceCfg_t"
    },
    {
        7221,
        "Pmic_powerSetCommonConfig : Test pgoodWindow uv monitor enable"
    },
    {
        7222,
        "Pmic_powerSetCommonConfig : Test pgoodWindow uv and ov monitor enable"
    },
    {
        7223,
        "Pmic_powerSetCommonConfig : Test Enable pgoodPolarity high"
    },
    {
        7224,
        "Pmic_powerSetCommonConfig : Test Enable pgoodPolarity low"
    },
    {
        7252,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as voltage current"
    },
    {
        7253,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as voltage"
    },
    {
        7254,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as masked"
    },
    {
        7255,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as NRSTOUT"
    },
    {
        7256,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as NRSTOUT SOC"
    },
    {
        7257,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as TDIE WARN"
    },
    {
        7258,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as VCCA"
    },
    {
        7259,
        "Pmic_powerSetConfigPowerGood : Test pgoodSelType as VMON"
    },
    {
        7261,
        "Pmic_powerGetPwrRsrcStat : Test get currentLimitLvlStat"
    },
    {
        7262,
        "Pmic_powerGetPwrRsrcStat : Test get underVoltageTholdStat"
    },
    {
        7263,
        "Pmic_powerGetPwrRsrcStat : Test get overVoltageTholdStat"
    },
    {
        7264,
        "Pmic_powerGetPwrRsrcStat : Test get overVoltageProtectionLvlStat"
    },
    {
        7266,
        "Pmic_powerSetThermalConfig : Test thermalWarnThold as low"
    },
    {
        7267,
        "Pmic_powerSetThermalConfig : Test thermalWarnThold as high"
    },
    {
        7268,
        "Pmic_powerSetThermalConfig : Test thermalShutdownThold as low"
    },
    {
        7269,
        "Pmic_powerSetThermalConfig : Test thermalShutdownThold as high"
    },
    {
        7271,
        "Pmic_powerSetPwrRsrcIntr : Test Enable OV interrupt"
    },
    {
        7272,
        "Pmic_powerSetPwrRsrcIntr : Test Disable OV interrupt"
    },
    {
        7273,
        "Pmic_powerSetPwrRsrcIntr : Test Enable UV interrupt"
    },
    {
        7274,
        "Pmic_powerSetPwrRsrcIntr : Test Disable UV interrupt"
    },
    {
        7275,
        "Pmic_powerSetPwrRsrcIntr : Test Enable ILIM interrupt"
    },
    {
        7276,
        "Pmic_powerSetPwrRsrcIntr : Test Disable ILIM interrupt"
    },
    {
        7280,
        "Pmic_powerSetIntr : Test Enable TWARN interrupt"
    },
    {
        7281,
        "Pmic_powerSetIntr : Test Disable TWARN interrupt"
    },
    {
        7282,
        "Pmic_powerSetIntr : Test Enable NRSTOUT_READBACK interrupt"
    },
    {
        7283,
        "Pmic_powerSetIntr : Test Disable NRSTOUT_READBACK interrupt"
    },
    {
        7284,
        "Pmic_powerSetIntr : Test Enable SOC_PWR_ERR interrupt"
    },
    {
        7285,
        "Pmic_powerSetIntr : Test Disable SOC_PWR_ERR interrupt"
    },
    {
        7286,
        "Pmic_powerSetIntr : Test Enable MCU_PWR_ERR interrupt"
    },
    {
        7287,
        "Pmic_powerSetIntr : Test Disable MCU_PWR_ERR interrupt"
    },
    {
        7288,
        "Pmic_powerSetIntr : Test Enable ORD_SHUTDOWN interrupt"
    },
    {
        7289,
        "Pmic_powerSetIntr : Test Disable ORD_SHUTDOWN interrupt"
    },
    {
        7290,
        "Pmic_powerSetIntr : Test Enable IMM_SHUTDOWN interrupt"
    },
    {
        7291,
        "Pmic_powerSetIntr : Test Disable IMM_SHUTDOWN interrupt"
    },
    {
        7292,
        "Pmic_powerSetIntr : Test Enable NRSTOUT_SOC_READBACK interrupt"
    },
    {
        7293,
        "Pmic_powerSetIntr : Test Disable NRSTOUT_SOC_READBACK interrupt"
    },
    {
        7294,
        "Pmic_powerSetIntr : Test Enable EN_DRV_READBACK interrupt"
    },
    {
        7295,
        "Pmic_powerSetIntr : Test Disable EN_DRV_READBACK interrupt"
    },
    {
        7225,
        "Pmic_powerSetCommonConfig : Test deglitchTimeSel as 4us"
    },
    {
        7226,
        "Pmic_powerSetCommonConfig : Test deglitchTimeSel as 20us"
    },
    {
        7227,
        "Pmic_powerSetCommonConfig : Test severeErrorTrig as Immediate shutdown"
    },
    {
        7228,
        "Pmic_powerSetCommonConfig : Test severeErrorTrig as oderly shutdown"
    },
    {
        7229,
        "Pmic_powerSetCommonConfig : Test severeErrorTrig as mcu power error"
    },
    {
        7230,
        "Pmic_powerSetCommonConfig : Test severeErrorTrig as soc power error"
    },
    {
        7231,
        "Pmic_powerSetCommonConfig : Test otherRailTrig as Immediate shutdown"
    },
    {
        7232,
        "Pmic_powerSetCommonConfig : Test otherRailTrig as oderly shutdown"
    },
    {
        7233,
        "Pmic_powerSetCommonConfig : Test otherRailTrig as mcu power error"
    },
    {
        7234,
        "Pmic_powerSetCommonConfig : Test otherRailTrig as soc power error"
    },
    {
        7235,
        "Pmic_powerSetCommonConfig : Test socRailTrig as Immediate shutdown"
    },
    {
        7236,
        "Pmic_powerSetCommonConfig : Test socRailTrig as oderly shutdown"
    },
    {
        7237,
        "Pmic_powerSetCommonConfig : Test socRailTrig as mcu power error"
    },
    {
        7238,
        "Pmic_powerSetCommonConfig : Test socRailTrig as soc power error"
    },
    {
        7239,
        "Pmic_powerSetCommonConfig : Test mcuRailTrig as Immediate shutdown"
    },
    {
        7240,
        "Pmic_powerSetCommonConfig : Test mcuRailTrig as oderly shutdown"
    },
    {
        7241,
        "Pmic_powerSetCommonConfig : Test mcuRailTrig as mcu power error"
    },
    {
        7242,
        "Pmic_powerSetCommonConfig : Test mcuRailTrig as soc power error"
    },
    {
        7243,
        "Pmic_powerSetCommonConfig : Test moderateRailTrig as Immediate shutdown"
    },
    {
        7244,
        "Pmic_powerSetCommonConfig : Test moderateRailTrig as oderly shutdown"
    },
    {
        7245,
        "Pmic_powerSetCommonConfig : Test moderateRailTrig as mcu power error"
    },
    {
        7246,
        "Pmic_powerSetCommonConfig : Test moderateRailTrig as soc power error"
    },
    {
        7297,
        "Pmic_powerSetLdoRtc : Test Enable ldortcRegulator"
    },
    {
        7298,
        "Pmic_powerSetLdoRtc : Test Disable ldortcRegulator"
    },
    {
        7277,
        "Pmic_powerSetPwrRsrcIntr : Parameter validation for Power Resource for intrEnable."
    },
    {
        7278,
        "Pmic_powerSetPwrRsrcIntr : Parameter validation for intrType"
    },
    {
        7247,
        "Pmic_powerSetCommonConfig : Parameter validation for severeErrorTrig"
    },
    {
        7248,
        "Pmic_powerSetCommonConfig : Parameter validation for otherRailTrig"
    },
    {
        7249,
        "Pmic_powerSetCommonConfig : Parameter validation for socRailTrig"
    },
    {
        7250,
        "Pmic_powerSetCommonConfig : Parameter validation for mcuRailTrig"
    },
    {
        7251,
        "Pmic_powerSetCommonConfig : Parameter validation for moderateRailTrig"
    },
    {
        7301,
        "Pmic_powerGetCommonConfig : Parameter validation for handle"
    },
    {
        7260,
        "Pmic_powerSetConfigPowerGood : Parameter validation for handle"
    },
    {
        7302,
        "Pmic_powerGetConfigPowerGood : Parameter validation for handle"
    },
    {
        7265,
        "Pmic_powerGetPwrRsrcStat : Parameter validation for handle"
    },
    {
        7270,
        "Pmic_powerSetThermalConfig : Parameter validation for handle"
    },
    {
        7303,
        "Pmic_powerGetThermalConfig : Parameter validation for handle"
    },
    {
        7279,
        "Pmic_powerSetPwrRsrcIntr : Parameter validation for handle"
    },
    {
        7296,
        "Pmic_powerSetIntr : Parameter validation for handle"
    },
    {
        0xAB00,
        "Pmic_powerSetPwrRsrcIntr : Test Set Enable OV interrupt"
    },
    {
        0xAB01,
        "Pmic_powerSetPwrRsrcIntr : Test Set Disable OV interrupt"
    },
    {
        0xAB02,
        "Pmic_powerSetPwrRsrcIntr : Test Set Enable UV interrupt"
    },
    {
        0xAB03,
        "Pmic_powerSetPwrRsrcIntr : Test Set Disable UV interrupt"
    },
    {
        0xAB04,
        "Pmic_powerSetPwrRsrcIntr : Test Set Enable ILIM interrupt"
    },
    {
        0xAB05,
        "Pmic_powerSetPwrRsrcIntr : Test Set Disable ILIM interrupt"
    },
    {
        0xAB06,
        "Pmic_powerSetIntr : Test Set Enable TWARN interrupt"
    },
    {
        0xAB07,
        "Pmic_powerSetIntr : Test Set Disable TWARN interrupt"
    },
    {
        0xAB08,
        "Pmic_powerSetIntr : Test Set Enable NRSTOUT_READBACK interrupt"
    },
    {
        0xAB09,
        "Pmic_powerSetIntr : Test Set Disable NRSTOUT_READBACK interrupt"
    },
    {
        0xAB0A,
        "Pmic_powerSetIntr : Test Set Enable SOC_PWR_ERR interrupt"
    },
    {
        0xAB0B,
        "Pmic_powerSetIntr : Test Set Disable SOC_PWR_ERR interrupt"
    },
    {
        0xAB0C,
        "Pmic_powerSetIntr : Test Set Enable MCU_PWR_ERR interrupt"
    },
    {
        0xAB0D,
        "Pmic_powerSetIntr : Test Set Disable MCU_PWR_ERR interrupt"
    },
    {
        0xAB0E,
        "Pmic_powerSetIntr : Test Set Enable ORD_SHUTDOWN interrupt"
    },
    {
        0xAB0F,
        "Pmic_powerSetIntr : Test Set Disable ORD_SHUTDOWN interrupt"
    },
    {
        0xAB11,
        "Pmic_powerSetIntr : Test Set Enable IMM_SHUTDOWN interrupt"
    },
    {
        0xAB12,
        "Pmic_powerSetIntr : Test Set Disable IMM_SHUTDOWN interrupt"
    },
    {
        0xAB13,
        "Pmic_powerSetIntr : Test Set Enable NRSTOUT_SOC_READBACK interrupt"
    },
    {
        0xAB14,
        "Pmic_powerSetIntr : Test Set Disable NRSTOUT_SOC_READBACK interrupt"
    },
    {
        0xAB15,
        "Pmic_powerSetIntr : Test Set Enable EN_DRV_READBACK interrupt"
    },
    {
        0xAB16,
        "Pmic_powerSetIntr : Test Set Disable EN_DRV_READBACK interrupt"
    },
    {
        7220,
        "Pmic_powerSetCommonConfig : Parameter validation for handle"
    },
    {
        7726,
        "Pmic_powerGetPwrThermalStat : Parameter validation for handle."
    },
    {
        7727,
        "Pmic_powerGetPwrThermalStat : Parameter validation for pPwrThermalStatCfg."
    },
    {
        7728,
        "Pmic_powerGetPwrThermalStat : Test Get Thermal Warn Status."
    },
    {
        7729,
        "Pmic_powerGetPwrThermalStat : Test Get Oderly Shutdown Status."
    },
    {
        7730,
        "Pmic_powerGetPwrThermalStat : Test Get immediate Shutdown Status."
    },
    {
        7872,
        "Pmic_powerSetPwrResourceCfg : Negative test LDO Pull down Select for HERA PMIC"
    },
    {
        7873,
        "Pmic_powerSetPwrResourceCfg : Negative test VMON for LEO PMIC."
    },
    {
        7874,
        "Pmic_powerSetThermalConfig : Negative test for thermalShutdownThold on Leo PMIC PG1.0"
    },
    {
        7875,
        "Pmic_powerGetThermalConfig : Negative test for thermalShutdownThold on Leo PMIC PG1.0"
    },
    {
        7876,
        "Pmic_powerSetLdoRtc : Negative test Disable ldortcRegulator for HERA"
    },
    {
        7878,
        "Pmic_powerGetPwrResourceCfg : Negative test Get Switch peak current limit for BUCK 5"
    },
    {
        10622,
        "Pmic_powerGetThermalConfig : Parameter validation for pThermalThreshold"
    },
    {
        10623,
        "Pmic_powerGetPwrRsrcStat : Parameter validation for pPwrRsrcStatCfg"
    },
    {
        10624,
        "Pmic_powerGetConfigPowerGood : Parameter validation for pPgoodSelType"
    },
    {
        10625,
        "Pmic_powerGetCommonConfig : Parameter validation for pPowerCommonCfg"
    },
    {
        10626,
        "Pmic_powerSetPwrResourceCfg : Negative test for ldoRvTimeoutSel configuration on Hera"
    },
    {
        10627,
        "Pmic_powerSetPwrResourceCfg : Negative test for Buck2 pwmMpMode configuration"
    },
    {
        10628,
        "Pmic_powerSetPwrResourceCfg : Negative test for Ldo4 bypass mode configuration"
    },
    {
        10629,
        "Pmic_powerSetIntr : Parameter Validation of intrType"
    },
    {
        10630,
        "Pmic_powerSetPwrResourceCfg : Test to set Power good high threshold level for Buck1"
    },
    {
        10631,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for powerGoodOvThr"
    },
    {
        10632,
        "Pmic_powerSetPwrResourceCfg : Test to set Power good low threshold level for Buck1"
    },
    {
        10633,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for powerGoodUvThr"
    },
    {
        10635,
        "Pmic_powerGetPwrRsrcStat : Parameter validation for ILIM status pwrResource"
    },
    {
        10636,
        "Pmic_powerGetPwrRsrcStat : Parameter validation for for VCCA OV LVL status pwrResource"
    },
    {
        10637,
        "Pmic_powerGetCommonConfig : Negative test for deglitchTimeSel on Leo"
    },
    {
        10638,
        "Pmic_powerSetCommonConfig : Negative test for deglitchTimeSel on Leo"
    },
    {
        10639,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for vccaPwrGudLvl pwrResource"
    },
    {
        10640,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for vmonRange pwrResource"
    },
    {
        10641,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for buckCurrentLimit pwrResource"
    },
    {
        10642,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for buckVmonSlewRate pwrResource"
    },
    {
        10643,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for residual voltage checking pwrResource"
    },
    {
        10644,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for residual voltage checking pwrResource"
    },
    {
        10646,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for buckPullDownEn pwrResource"
    },
    {
        10647,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for buckVoutSel pwrResource"
    },
    {
        10648,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for buckFpwmMode pwrResource"
    },
    {
        10649,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for buckFpwmMpMode pwrResource"
    },
    {
        10650,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for regulatorEn pwrResource"
    },
    {
        10651,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for ldoPullDownSel pwrResource"
    },
    {
        10652,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for buckCurrentLimit min value"
    },
    {
        10653,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for ldoBypassModeEn pwrResource"
    },
    {
        10654,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for ldoRvTimeoutSel pwrResource"
    },
    {
        10655,
        "Pmic_powerGetPwrResourceCfg : Parameter validation for voltage_mV pwrResource"
    },
    {
        10656,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV as 599mv min Value"
    },
    {
        10657,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV as 1099mv"
    },
    {
        10658,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV as 1659mV max Value"
    },
    {
        10659,
        "Pmic_powerSetPwrResourceCfg : Negative test for ldoSlowRampEn configuration on PG1.0 Silicon Revision"
    },
    {
        10660,
        "Pmic_powerSetLdoRtc : Parameter validation for handle"
    },
    {
        10661,
        "Pmic_powerGetLdoRtc : Parameter validation for handle"
    },
    {
        10662,
        "Pmic_powerGetLdoRtc : Parameter validation for pLdortcEnable"
    },
    {
        10663,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for min voltage level for buck"
    },
    {
        10664,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for max voltage level for buck"
    },
    {
        10665,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for min voltage level for ldo"
    },
    {
        10666,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for power resource type"
    },
    {
        10667,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Buck Max Value Power Resource limit"
    },
    {
        10668,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for Buck Min Value Power Resource limit"
    },
    {
        10669,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for LDO and VMON Min Value Power Resource limit"
    },
    {
        10670,
        "Pmic_powerSetPwrRsrcIntr :  Parameter validation for interrupt type pwrResource"
    },
    {
        10671,
        "Pmic_powerSetConfigPowerGood : Parameter validation for TDIE WARN pgoodSelType"
    },
    {
        10672,
        "Pmic_powerSetConfigPowerGood : Parameter validation for threshold voltage and current limit pgoodSelType"
    },
    {
        10673,
        "Pmic_powerSetConfigPowerGood : Parameter validation for NRSTOUT pgoodSelType"
    },
    {
        10674,
        "Pmic_powerSetConfigPowerGood : Parameter validation for NRSTOUT_SOC pgoodSelType"
    },
    {
        10675,
        "Pmic_powerSetConfigPowerGood : Parameter validation for VCCA/VMON Enable pgoodSelType"
    },
    {
        10676,
        "Pmic_powerGetLdoRtc : Negative test to get pLdortcEnable configuration on HERA"
    },
    {
        10677,
        "Pmic_powerSetConfigPowerGood : Parameter validation for Buck power good source max value"
    },
    {
        10678,
        "Pmic_powerSetConfigPowerGood : Parameter validation for Buck power good source min value"
    },
    {
        10679,
        "Pmic_powerSetConfigPowerGood : Parameter validation for VMON power good source max value"
    },
    {
        10680,
        "Pmic_powerSetConfigPowerGood : Parameter validation for VMON power good source min value"
    },
    {
        10681,
        "Pmic_powerSetConfigPowerGood : Parameter validation for LDO power good source max value"
    },
    {
        10682,
        "Pmic_powerSetConfigPowerGood : Parameter validation for LDO power good source min value"
    },
    {
        10683,
        "Pmic_powerSetConfigPowerGood : Parameter validation for NRSTOUT power good source"
    },
    {
        10684,
        "Pmic_powerSetConfigPowerGood : Parameter validation for NRSTOUT_SOC power good source"
    },
    {
        10685,
        "Pmic_powerSetConfigPowerGood : Parameter validation for TDIE_WARN power good source"
    },
    {
        10686,
        "Pmic_powerSetConfigPowerGood : Parameter validation for VCCA power good source"
    },
    {
        10687,
        "Pmic_powerSetConfigPowerGood : Parameter validation for power good source"
    },
    {
        10688,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for power Resource for rvCheckEn"
    },
    {
        10689,
        "Pmic_powerSetPwrResourceCfg : Test to set vmonRange as PMIC_LP8764X_VMON_RANGE_3V35_5V and voltage_mV as 3350mV "
    },
    {
        10690,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV when vmonRange as 0V3_3V34 and voltage_mV as 299 mV min Val"
    },
    {
        10691,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV when vmonRange as 0V3_3V34 and voltage_mV as 3341U mV max Val"
    },
    {
        10692,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV when vmonRange as 3V35_5V and voltage_mV as 3349 mV min Val"
    },
    {
        10693,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV when vmonRange as 3V35_5V and voltage_mV as 5001 mV max Val"
    },
    {
        10694,
        "Pmic_powerSetPwrResourceCfg : Parameter validation for vccaPwrGudLvl power resource"
    },
    {
        10695,
        "Pmic_powerGetPwrResourceCfg : Negative for ldoSlowRampEn configuration on PG1.0 Silicon Revision"
    },
    {
        10696,
        "Pmic_powerSetPwrResourceCfg : Parameter range validation for voltage_mV min value"
    },
    {
        10697,
        "Pmic_powerTests : Fault Injection and Coverage Gaps"
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

    test_pmic_print_unity_testcase_info(7130,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrResourceCfg(NULL, pwrRsrc, pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7130,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7299,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrResourceCfg(NULL, pwrRsrc, &pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7299,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7131,
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

    pmic_testResultUpdate_pass(7131,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7132,
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

    pmic_testResultUpdate_pass(7132,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);

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

    test_pmic_print_unity_testcase_info(7133,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7133,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7134,
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

    pmic_testResultUpdate_pass(7134,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7135,
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

    pmic_testResultUpdate_pass(7135,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7136,
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

    pmic_testResultUpdate_pass(7136,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7137,
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
        if(((J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
            (PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)) &&
            (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
        {
            if((PMIC_TPS6594X_REGULATOR_BUCK3 == pwrRsrc) ||
               (PMIC_TPS6594X_REGULATOR_BUCK4 == pwrRsrc))
            {
                /* VMON Enable for BUCK 3 and 4 for Leo PMIC-B on PG2.0 Leo PMIC results in warm reset.
                 * The issue is due to the FB_B3 and FB_B4 pins are currently not connected to any input,
                 * so when the VMON is enabled for these bucks they are expected to fail.
                 * Warm reset is because of the BIST_FAIL_INT which occurred on LeoB. If the regulator
                 * output is floating in the VMON tests, then this will cause the VMON_ABIST to fail and
                 * cause the warm reset
                 */
                 continue;
            }
        }

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

    pmic_testResultUpdate_pass(7137,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7138,
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

    pmic_testResultUpdate_pass(7138,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7139,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7139,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7143,
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

    pmic_testResultUpdate_pass(7143,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7144,
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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        if((J7VCL_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4)))
        {
            continue;
        }

        if((J7VCL_HERA_PMICB_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_LP8764X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_LP8764X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_LP8764X_REGULATOR_BUCK3)))
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

    pmic_testResultUpdate_pass(7144,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7145,
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

    pmic_testResultUpdate_pass(7145,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7146,
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

    pmic_testResultUpdate_pass(7146,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7147,
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

    pmic_testResultUpdate_pass(7147,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7148,
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

    pmic_testResultUpdate_pass(7148,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7149,
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

    pmic_testResultUpdate_pass(7149,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7150,
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

    pmic_testResultUpdate_pass(7150,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7151,
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

    pmic_testResultUpdate_pass(7151,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7152,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    /* 7468 PMIC: Few PMIC Power related features can't be tested on J721E EVM */
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7152,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pPowerCfg.regulatorEn = PMIC_TPS6594X_REGULATOR_DISABLE;

    pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK2;

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.regulatorEn, powerCfg_rd.regulatorEn);

    pmic_testResultUpdate_pass(7152,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7153,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    /* 7468 PMIC: Few PMIC Power related features can't be tested on J721E EVM */
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7153,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK2;
    pPowerCfg.regulatorEn = PMIC_TPS6594X_REGULATOR_ENABLE;

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.regulatorEn, powerCfg_rd.regulatorEn);

    pmic_testResultUpdate_pass(7153,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7154,
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

    pmic_testResultUpdate_pass(7154,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7155,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Current limit 6.5 is not supported by LEO pmic */
        pmic_testResultUpdate_ignore(7155,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7155,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7156,
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

    pmic_testResultUpdate_pass(7156,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7157,
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

    pmic_testResultUpdate_pass(7157,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7158,
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

    pmic_testResultUpdate_pass(7158,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7159,
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

    pmic_testResultUpdate_pass(7159,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7160,
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

    pmic_testResultUpdate_pass(7160,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7161,
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

    pmic_testResultUpdate_pass(7161,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7162,
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

    pmic_testResultUpdate_pass(7162,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7163,
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

    pmic_testResultUpdate_pass(7163,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7164,
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

    pmic_testResultUpdate_pass(7164,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7165,
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

    pmic_testResultUpdate_pass(7165,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7166,
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

    pmic_testResultUpdate_pass(7166,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7167,
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

    pmic_testResultUpdate_pass(7167,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7168,
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

    pmic_testResultUpdate_pass(7168,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7169,
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

    pmic_testResultUpdate_pass(7169,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7170,
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

    pmic_testResultUpdate_pass(7170,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7171,
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

    pmic_testResultUpdate_pass(7171,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7172,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7172,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7172,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7173,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7173,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7173,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7174,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7174,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7174,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7175,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7175,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7175,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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

    test_pmic_print_unity_testcase_info(7176,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if((PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType) ||
       ((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
        (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)))
    {
        /* ldoSlowRamp feature is not supported in HERA PMIC and Leo PMIC PG1.0 */
        pmic_testResultUpdate_ignore(7176,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoSlowRampEn = PMIC_TPS6594X_REGULATOR_LDO_SLOW_RAMP_ENABLE;
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

        TEST_ASSERT_EQUAL(pPowerCfg.ldoSlowRampEn, powerCfg_rd.ldoSlowRampEn);
    }

    pmic_testResultUpdate_pass(7176,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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

    test_pmic_print_unity_testcase_info(7177,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        pPowerCfg.ldoSlowRampEn = PMIC_TPS6594X_REGULATOR_LDO_SLOW_RAMP_DISABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if((PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType) ||
       ((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
        (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)))
    {
        /* ldoSlowRamp feature is not supported in HERA PMIC and Leo PMIC PG1.0 */
        pmic_testResultUpdate_ignore(7177,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7177,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7178,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7178,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7178,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7179,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7179,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    }

    pmic_testResultUpdate_pass(7179,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7180,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if((PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType) ||
       ((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
        (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)))
    {
        /* ldoSlowRampEn feature is not supported in HERA PMIC and Leo PMIC PG1.0 */
        pmic_testResultUpdate_ignore(7180,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7180,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * 7483
 * PMIC: BUCK and LDO voltage configuration can't be tested for supported values
 * on J721E EVM.
 * voltages should be fixed at their default value, and not adjusted.
 * Tested voltages:
 *    PMIC A
 *      BUCK 2 -> 300mv
 *      BUCK 5 -> 300mv
 *      LDO  2 -> 1800mv
 *      LDO  3 -> 800mv
 *      LDO  4 -> 1800mv
 *
 *   PMIC B
 *     BUCK 2 -> 300mv
 *     BUCK 3 -> 300mv
 *     BUCK 4 -> 300mv
 *     LDO  1 -> 3300mv
 *     LDO  3 -> 1800mv
 */

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Voltage level in miliVolts for regulators
 */
static void test_pmic_powerSetPowerResourceConfig_voltage_mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7181,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if((J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        pmic_testResultUpdate_ignore(7181,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pPowerCfg.voltage_mV = powerCfg_rd.voltage_mV;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmic_log("BUCK-%d: passed for %dmv \n", (pwrRsrc - pwrRsrcMin + 1), pPowerCfg.voltage_mV);
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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO2)  ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)))
        {
            continue;
        }

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        pPowerCfg.voltage_mV = powerCfg_rd.voltage_mV;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            pmic_log("LDO-%d: passed for %dmv \n", (pwrRsrc - pwrRsrcMin + 1), pPowerCfg.voltage_mV);
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            pmic_log("VMON-%d: passed for %dmv \n", (pwrRsrc - pwrRsrcMin + 1), pPowerCfg.voltage_mV);
        }
    }

    pmic_testResultUpdate_pass(7181,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7182,
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

    pmic_testResultUpdate_pass(7182,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for voltage_mV max value.
 */
static void test_pmic_powerSetPowerResourceConfigPrmRangeTest_voltage_mV_maxVal(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7183,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
        pPowerCfg.voltage_mV = PMIC_UT_TPS6594X_REGULATOR_BUCK_MAX_VOLTAGE + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
        pPowerCfg.voltage_mV = PMIC_UT_LP8764X_REGULATOR_BUCK_MAX_VOLTAGE + 1;
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

    pmic_testResultUpdate_pass(7183,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Powergood level 5V for VCCA pin
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

    test_pmic_print_unity_testcase_info(7184,
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

    pmic_testResultUpdate_pass(7184,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Powergood level 3.3V for VCCA pin
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

    test_pmic_print_unity_testcase_info(7185,
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

    pmic_testResultUpdate_pass(7185,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for vccaPwrGudLvl.
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

    test_pmic_print_unity_testcase_info(7186,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7186,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7187,
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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
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
                (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
                ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)  ||
                 (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
            {
                continue;
            }

            if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
                (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
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

    pmic_testResultUpdate_pass(7187,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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

    test_pmic_print_unity_testcase_info(7188,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
       /*
        * Once the PFSM is in operation, user should not change rail grp
        * setting as this may compromise the system’s functional safety design
        */
        pmic_testResultUpdate_ignore(7188,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
       /*
        * Once the PFSM is in operation, user should not change rail grp
        * setting as this may compromise the system’s functional safety design
        */
        pmic_testResultUpdate_ignore(7188,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
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

    pmic_testResultUpdate_pass(7188,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7189,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /*
         * Once the PFSM is in operation, user should not change rail grp
         * setting as this may compromise the system’s functional safety design
         */
        pmic_testResultUpdate_ignore(7189,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /*
         * Once the PFSM is in operation, user should not change rail grp
         * setting as this may compromise the system’s functional safety design
         */
        pmic_testResultUpdate_ignore(7189,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
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

    pmic_testResultUpdate_pass(7189,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7190,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /*
         * Once the PFSM is in operation, user should not change rail grp
         * setting as this may compromise the system’s functional safety design
         */
        pmic_testResultUpdate_ignore(7190,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /*
         * Once the PFSM is in operation, user should not change rail grp
         * setting as this may compromise the system’s functional safety design
         */
        pmic_testResultUpdate_ignore(7190,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
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
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
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

    pmic_testResultUpdate_pass(7190,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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

    test_pmic_print_unity_testcase_info(7191,
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

    pmic_testResultUpdate_pass(7191,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7192,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7192,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    pmic_testResultUpdate_pass(7192,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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

    test_pmic_print_unity_testcase_info(7198,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    /* 7468 PMIC: Few PMIC Power related features can't be tested on J721E EVM */
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /*  Setting LDOs in bypass mode - Resulted in DDR issue */
        pmic_testResultUpdate_ignore(7198,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /*  Setting LDOs in bypass mode - Resulted in DDR issue */
        pmic_testResultUpdate_ignore(7198,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoBypassModeEn = PMIC_TPS6594X_REGULATOR_LDO_BYPASS_MODE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO3;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7198,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
            ((pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK1) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK2) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK3) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK4) ||
             (pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO1)))
        {
            continue;
        }

        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
            (J721E_LEO_PMICB_DEVICE == pmic_device_info) &&
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

    pmic_testResultUpdate_pass(7198,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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

    test_pmic_print_unity_testcase_info(7199,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

#if defined(SOC_J721E)
    /* 7468 PMIC: Few PMIC Power related features can't be tested on J721E EVM */
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7199,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }
#endif

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7199,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoBypassModeEn = PMIC_TPS6594X_REGULATOR_LDO_LINEAR_REGULATOR_MODE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO3;
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

    pmic_testResultUpdate_pass(7199,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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

    test_pmic_print_unity_testcase_info(7200,
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

    pmic_testResultUpdate_pass(7200,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7201,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7201,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7201,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7202,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7202,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7202,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7203,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7203,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7203,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7204,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7204,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7204,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7205,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7205,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7205,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7206,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7206,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7206,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7207,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7207,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7207,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7208,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7208,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7208,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7209,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7209,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7209,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7210,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7210,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7210,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7211,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7211,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7211,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7212,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7212,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7212,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7213,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7213,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7213,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7214,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7214,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7214,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7215,
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

    pmic_testResultUpdate_pass(7215,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7216,
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
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(7216,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    pmic_testResultUpdate_pass(7216,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7217,
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
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(7217,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

        TEST_ASSERT_EQUAL(pPowerCfg.vmonRange, powerCfg_rd.vmonRange);
    }

    pmic_testResultUpdate_pass(7217,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7218,
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
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(7218,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

        TEST_ASSERT_EQUAL(pPowerCfg.vmonRange, powerCfg_rd.vmonRange);
    }

    pmic_testResultUpdate_pass(7218,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7219,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(7219,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7219,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for Pmic_PowerResourceCfg_t
 */
static void test_pmic_powerGetPowerResourceConfigPrmValTest_Pmic_PowerResourceCfg_t(void)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;

    test_pmic_print_unity_testcase_info(7300,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle, pwrRsrc, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7300,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7220,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetCommonConfig(NULL,  pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7220,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7221,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.pgoodWindow = PMIC_POWER_GOOD_UV_MONITOR_ENABLE;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.pgoodWindow, powerCfg_rd.pgoodWindow);

    pmic_testResultUpdate_pass(7221,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7222,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.pgoodWindow = PMIC_POWER_GOOD_UV_OV_MONITOR_ENABLE;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle,
                                             pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.pgoodWindow, powerCfg_rd.pgoodWindow);

    pmic_testResultUpdate_pass(7222,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7223,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pwrCommonCfg.pgoodPolarity = PMIC_POWER_PGOOD_POL_HIGH;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle,
                                             pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.pgoodPolarity, powerCfg_rd.pgoodPolarity);

    pmic_testResultUpdate_pass(7223,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7224,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.pgoodPolarity = PMIC_POWER_PGOOD_POL_LOW;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle,
                                             pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.pgoodPolarity, powerCfg_rd.pgoodPolarity);

    pmic_testResultUpdate_pass(7224,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7252,
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

    pmic_testResultUpdate_pass(7252,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7253,
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

    pmic_testResultUpdate_pass(7253,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7254,
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

    pmic_testResultUpdate_pass(7254,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7255,
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

    pmic_testResultUpdate_pass(7255,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7256,
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

    pmic_testResultUpdate_pass(7256,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7257,
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

    pmic_testResultUpdate_pass(7257,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7258,
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

    pmic_testResultUpdate_pass(7258,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7259,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(7259,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(7259,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7261,
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

    pmic_testResultUpdate_pass(7261,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7262,
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

    pmic_testResultUpdate_pass(7262,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7263,
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

    pmic_testResultUpdate_pass(7263,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7264,
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

    pmic_testResultUpdate_pass(7264,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)&&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_PG_2_0_THERMAL_TEMP_WARN_130C;
    }

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)&&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_THERMAL_TEMP_WARN_120C;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold = PMIC_LP8764X_THERMAL_TEMP_WARN_120C;
    }

    test_pmic_print_unity_testcase_info(7266,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(thermalThreshold.thermalWarnThold, thermalThreshold_rd.thermalWarnThold);

    pmic_testResultUpdate_pass(7266,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)&&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_PG_2_0_THERMAL_TEMP_WARN_140C;
    }

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)&&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_THERMAL_TEMP_WARN_130C;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold = PMIC_LP8764X_THERMAL_TEMP_WARN_130C;
    }

    test_pmic_print_unity_testcase_info(7267,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(thermalThreshold.thermalWarnThold, thermalThreshold_rd.thermalWarnThold);

    pmic_testResultUpdate_pass(7267,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
        thermalThreshold.thermalShutdownThold =
                                PMIC_LP8764X_THERMAL_TEMP_TSD_ORD_140C;
    }

    test_pmic_print_unity_testcase_info(7268,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        /* thermalShutdownThold feature is not supported in Leo PMIC PG1.0 */
        pmic_testResultUpdate_ignore(7268,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(thermalThreshold.thermalShutdownThold,
                      thermalThreshold_rd.thermalShutdownThold);

    pmic_testResultUpdate_pass(7268,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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
        thermalThreshold.thermalShutdownThold = PMIC_LP8764X_THERMAL_TEMP_TSD_ORD_145C;
    }

    test_pmic_print_unity_testcase_info(7269,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        /* thermalShutdownThold feature is not supported in Leo PMIC PG1.0 */
        pmic_testResultUpdate_ignore(7269,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(thermalThreshold.thermalShutdownThold,
                      thermalThreshold_rd.thermalShutdownThold);

    pmic_testResultUpdate_pass(7269,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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
    test_pmic_print_unity_testcase_info(7271,
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

    pmic_testResultUpdate_pass(7271,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7272,
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

    pmic_testResultUpdate_pass(7272,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7273,
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

    pmic_testResultUpdate_pass(7273,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7274,
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

    pmic_testResultUpdate_pass(7274,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7275,
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

    pmic_testResultUpdate_pass(7275,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7276,
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

    pmic_testResultUpdate_pass(7276,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7280,
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

    pmic_testResultUpdate_pass(7280,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7281,
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

    pmic_testResultUpdate_pass(7281,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7282,
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

    pmic_testResultUpdate_pass(7282,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7283,
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

    pmic_testResultUpdate_pass(7283,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7284,
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

    pmic_testResultUpdate_pass(7284,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7285,
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

    pmic_testResultUpdate_pass(7285,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7286,
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

    pmic_testResultUpdate_pass(7286,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7287,
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

    pmic_testResultUpdate_pass(7287,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7288,
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

    pmic_testResultUpdate_pass(7288,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7289,
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

    pmic_testResultUpdate_pass(7289,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7290,
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

    pmic_testResultUpdate_pass(7290,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7291,
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

    pmic_testResultUpdate_pass(7291,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7292,
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

    pmic_testResultUpdate_pass(7292,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7293,
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

    pmic_testResultUpdate_pass(7293,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7294,
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

    pmic_testResultUpdate_pass(7294,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7295,
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

    pmic_testResultUpdate_pass(7295,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7225,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        /* deglitchTimeSel feature is not supported in Leo PMIC PG1.0 */
        pmic_testResultUpdate_ignore(7225,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    pmic_testResultUpdate_pass(7225,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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

    test_pmic_print_unity_testcase_info(7226,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        /* deglitchTimeSel feature is not supported in Leo PMIC PG1.0 */
        pmic_testResultUpdate_ignore(7226,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    pmic_testResultUpdate_pass(7226,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

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

    test_pmic_print_unity_testcase_info(7227,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.severeErrorTrig, powerCfg_rd.severeErrorTrig);

    pmic_testResultUpdate_pass(7227,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7228,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.severeErrorTrig, powerCfg_rd.severeErrorTrig);

    pmic_testResultUpdate_pass(7228,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7229,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.severeErrorTrig, powerCfg_rd.severeErrorTrig);

    pmic_testResultUpdate_pass(7229,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7230,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.severeErrorTrig, powerCfg_rd.severeErrorTrig);

    pmic_testResultUpdate_pass(7230,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7231,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.otherRailTrig, powerCfg_rd.otherRailTrig);

    pmic_testResultUpdate_pass(7231,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7232,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.otherRailTrig, powerCfg_rd.otherRailTrig);

    pmic_testResultUpdate_pass(7232,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7233,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.otherRailTrig, powerCfg_rd.otherRailTrig);

    pmic_testResultUpdate_pass(7233,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7234,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.otherRailTrig, powerCfg_rd.otherRailTrig);

    pmic_testResultUpdate_pass(7234,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7235,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.socRailTrig, powerCfg_rd.socRailTrig);

    pmic_testResultUpdate_pass(7235,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7236,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.socRailTrig, powerCfg_rd.socRailTrig);

    pmic_testResultUpdate_pass(7236,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7237,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.socRailTrig, powerCfg_rd.socRailTrig);

    pmic_testResultUpdate_pass(7237,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7238,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.socRailTrig, powerCfg_rd.socRailTrig);

    pmic_testResultUpdate_pass(7238,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7239,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.mcuRailTrig, powerCfg_rd.mcuRailTrig);

    pmic_testResultUpdate_pass(7239,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7240,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.mcuRailTrig, powerCfg_rd.mcuRailTrig);

    pmic_testResultUpdate_pass(7240,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7241,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.mcuRailTrig, powerCfg_rd.mcuRailTrig);

    pmic_testResultUpdate_pass(7241,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7242,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.mcuRailTrig, powerCfg_rd.mcuRailTrig);

    pmic_testResultUpdate_pass(7242,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7243,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_IMM_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.moderateRailTrig, powerCfg_rd.moderateRailTrig);

    pmic_testResultUpdate_pass(7243,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7244,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_ODERLY_SHUTDOWN;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.moderateRailTrig, powerCfg_rd.moderateRailTrig);

    pmic_testResultUpdate_pass(7244,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7245,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_MCU_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.moderateRailTrig, powerCfg_rd.moderateRailTrig);

    pmic_testResultUpdate_pass(7245,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7246,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pwrCommonCfg.moderateRailTrig, powerCfg_rd.moderateRailTrig);

    pmic_testResultUpdate_pass(7246,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetLdoRtc : Test Enable ldortcRegulator
 */
static void test_pmic_powerSetLdoRtc_ldortcEnable_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool ldortcEnable;
    bool ldortcEnable_rd;

    test_pmic_print_unity_testcase_info(7297,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        ldortcEnable = PMIC_TPS6594X_REGULATOR_LDORTC_ENABLE;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7297,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetLdoRtc(pPmicCoreHandle, ldortcEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_powerGetLdoRtc(pPmicCoreHandle, &ldortcEnable_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(ldortcEnable, ldortcEnable_rd);

    pmic_testResultUpdate_pass(7297,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetLdoRtc : Test Disable ldortcRegulator
 */
static void test_pmic_powerSetLdoRtc_ldortcEnable_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool ldortcEnable;
    bool ldortcEnable_rd;

    test_pmic_print_unity_testcase_info(7298,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        ldortcEnable = PMIC_TPS6594X_REGULATOR_LDORTC_DISABLE;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7298,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetLdoRtc(pPmicCoreHandle, ldortcEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_powerGetLdoRtc(pPmicCoreHandle, &ldortcEnable_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(ldortcEnable, ldortcEnable_rd);

    pmic_testResultUpdate_pass(7298,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7277,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7277,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7278,
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

    pmic_testResultUpdate_pass(7278,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7247,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7247,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7248,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7248,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7249,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7249,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7250,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7250,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7251,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR + 1;

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7251,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7301,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetCommonConfig(NULL, &pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7301,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for handle
 */
static void test_pmic_powerSetConfigPowerGoodgPrmValTest_handle(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(7260,
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

    pmic_testResultUpdate_pass(7260,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetConfigPowerGood : Parameter validation for handle
 */
static void test_pmic_powerGetConfigPowerGoodPrmValTest_handle(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(7302,
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

    pmic_testResultUpdate_pass(7302,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7265,
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

    pmic_testResultUpdate_pass(7265,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_PG_2_0_THERMAL_TEMP_WARN_140C;
    }

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_THERMAL_TEMP_WARN_130C;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold = PMIC_LP8764X_THERMAL_TEMP_WARN_130C;
    }

    test_pmic_print_unity_testcase_info(7270,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetThermalConfig(NULL, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7270,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_PG_2_0_THERMAL_TEMP_WARN_140C;
    }

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_THERMAL_TEMP_WARN_130C;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold = PMIC_LP8764X_THERMAL_TEMP_WARN_130C;
    }

    test_pmic_print_unity_testcase_info(7303,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetThermalConfig(NULL, &thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7303,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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

    test_pmic_print_unity_testcase_info(7279,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrRsrcIntr(NULL,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7279,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
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
    test_pmic_print_unity_testcase_info(7296,
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

    pmic_testResultUpdate_pass(7296,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

#if defined(ENABLE_SAMPLE_TESTCASES)
/*!
 * The below test case is dummy, as power related interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Enable OV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ov_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB00,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB00,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB00,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_POWER_OV_INT/32U] &
             (1U << (PMIC_TPS6594X_POWER_OV_INT % 32U))) != 0U))
        {
            timeout = 10U;

            while(timeout--)
            {
               /* Delay added to avoid timeout */
                Osal_delay(1000);

                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                 &errStat,
                                                 &irqNum);
                if(PMIC_TPS6594X_POWER_OV_INT == irqNum)
                    break;
            }

            if(0 > timeout)
            {
                pmicStatus = PMIC_ST_ERR_FAIL;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[PMIC_TPS6594X_POWER_OV_INT/32U] &
                 (1U << (PMIC_TPS6594X_POWER_OV_INT % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(PMIC_TPS6594X_POWER_OV_INT == irqNum)
                        break;
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[PMIC_TPS6594X_POWER_OV_INT/32U] &
         (1U << (PMIC_TPS6594X_POWER_OV_INT % 32U))) != 0U))
    {
        timeout = 10U;

        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(PMIC_TPS6594X_POWER_OV_INT == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                              pwrResource,
                                              intrType,
                                              intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[PMIC_TPS6594X_POWER_OV_INT/32U] &
                 (1U << (PMIC_TPS6594X_POWER_OV_INT % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(PMIC_TPS6594X_POWER_OV_INT == irqNum)
                        break;
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
                    break;
                }
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB00,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power related interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Disable OV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ov_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB01,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB01,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB01,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[intrType/32U] &
             (1U << (intrType % 32U))) != 0U))
        {
            timeout = 10U;

            while(timeout--)
            {
               /* Delay added to avoid timeout */
                Osal_delay(1000);

                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                 &errStat,
                                                 &irqNum);
                if(intrType == irqNum)
                {
                    pmicStatus = PMIC_ST_ERR_INV_PARAM;
                    break;
                }
            }

            if(0 > timeout)
            {
                pmicStatus = PMIC_ST_ERR_FAIL;
                break;
            }
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
                    break;
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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        timeout = 10U;

        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                              pwrResource,
                                              intrType,
                                              intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[intrType/32U] &
             (1U << (intrType % 32U))) != 0U))
        {
            timeout = 10U;

            while(timeout--)
            {
               /* Delay added to avoid timeout */
                Osal_delay(1000);

                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                 &errStat,
                                                 &irqNum);
                if(intrType == irqNum)
                {
                    pmicStatus = PMIC_ST_ERR_INV_PARAM;
                    break;
                }
            }

            if(0 > timeout)
            {
                pmicStatus = PMIC_ST_ERR_FAIL;
                break;
            }
        }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    pmic_testResultUpdate_pass(0xAB01,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power related interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Enable UV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_uv_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB02,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB02,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB02,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[intrType/32U] &
             (1U << (intrType % 32U))) != 0U))
        {
            timeout = 10U;

            while(timeout--)
            {
               /* Delay added to avoid timeout */
                Osal_delay(1000);

                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                 &errStat,
                                                 &irqNum);
                if(intrType == irqNum)
                    break;
            }

            if(0 > timeout)
            {
                pmicStatus = PMIC_ST_ERR_FAIL;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                        break;
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        timeout = 0U;

        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                        break;
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
                    break;
                }
            }
        }
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB02,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Disable UV interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_uv_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB03,
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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
                    break;
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
                    break;
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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        timeout = 10U;

        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pwrRsrcMax = PMIC_LP8764X_POWER_SOURCE_VMON2;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                pwrResource,
                                                intrType,
                                                intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
                    break;
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    pmic_testResultUpdate_pass(0xAB03,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Enable ILIM interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ilim_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB04,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB04,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB04,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[intrType/32U] &
             (1U << (intrType % 32U))) != 0U))
        {
            timeout = 10U;

            while(timeout--)
            {
               /* Delay added to avoid timeout */
                Osal_delay(1000);

                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                 &errStat,
                                                 &irqNum);
                if(intrType == irqNum)
                    break;
            }

            if(0 > timeout)
            {
                pmicStatus = PMIC_ST_ERR_FAIL;
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                        break;
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
                    break;
                }
            }
        }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    pmic_testResultUpdate_pass(0xAB04,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr : Test Set Disable ILIM interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ilim_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB05,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB05,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB05,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
    {
        pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                             pwrResource,
                                             intrType,
                                             intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
                    break;
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_LDO1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_LDO4;

        /* To clear the interrupts*/
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        for(pwrResource = pwrRsrcMin; pwrResource <= pwrRsrcMax ; pwrResource++)
        {
            pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                                  pwrResource,
                                                  intrType,
                                                  intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[intrType/32U] &
                 (1U << (intrType % 32U))) != 0U))
            {
                timeout = 10U;

                while(timeout--)
                {
                   /* Delay added to avoid timeout */
                    Osal_delay(1000);

                    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
                    if(intrType == irqNum)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_PARAM;
                        break;
                    }
                }

                if(0 > timeout)
                {
                    pmicStatus = PMIC_ST_ERR_FAIL;
                    break;
                }
            }

            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
    }

    pmic_testResultUpdate_pass(0xAB05,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as thermal interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable TWARN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_twarn_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB06,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB06,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB06,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB06,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as thermal interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable TWARN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_twarn_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB07,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB07,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB07,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB07,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable NRSTOUT_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_readback_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB08,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB08,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB08,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB08,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable NRSTOUT_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_readback_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB09,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB09,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB09,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB09,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable SOC_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB0A,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0A,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0A,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB0A,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable SOC_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB0B,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0B,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0B,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB0B,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable MCU_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB0C,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0C,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0C,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB0C,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable MCU_PWR_ERR interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB0D,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0D,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0D,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB0D,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable ORD_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ord_shutdown_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB0E,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0E,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0E,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB0E,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable ORD_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_ord_shutdown_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB0F,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0F,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB0F,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB0F,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable IMM_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_imm_shutdown_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB11,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB11,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB11,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB11,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable IMM_SHUTDOWN interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_imm_shutdown_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB12,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB12,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB12,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB12,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable NRSTOUT_SOC_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB13,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB13,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB13,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB13,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable NRSTOUT_SOC_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB14,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB14,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB14,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB14,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Enable EN_DRV_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_en_drv_readback_enabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(0xAB15,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB15,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB15,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
                break;
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB15,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * The below test case is dummy, as power interrupts cannot be generated
 * to test masking APIs.
 * 7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features
 * can't be tested
 */

/*!
 * \brief   Pmic_powerSetIntr : Test Set Disable EN_DRV_READBACK interrupt
 */
static void test_pmic_powerSetPwrRsrcIntr_en_drv_readback_disabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum = 0U;
    int8_t timeout = 10U;

    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;
    test_pmic_print_unity_testcase_info(0xAB16,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB16,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB16,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

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

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((errStat.intStatus[intrType/32U] &
         (1U << (intrType % 32U))) != 0U))
    {
        while(timeout--)
        {
           /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(intrType == irqNum)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB16,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}
#endif

/*!
 * \brief   Pmic_powerGetPwrThermalStat : Parameter validation for handle.
 */
static void test_pmic_powerGetPwrThermalStatPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerThermalStat_t pPwrThermalStatCfg =
    {
       PMIC_THERMAL_STAT_WARN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7726,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrThermalStat(NULL, &pPwrThermalStatCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7726,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrThermalStat : Parameter validation for pPwrThermalStatCfg.
 */
static void test_pmic_powerGetPwrThermalStatPrmValTest_pPwrThermalStatCfg(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7727,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrThermalStat(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7727,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrThermalStat : Test Get Thermal Warn Status.
 */
static void test_pmic_powerGetPwrThermalStat_thermalStatus(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerThermalStat_t pPwrThermalStatCfg =
    {
       PMIC_THERMAL_STAT_WARN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7728,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrThermalStat(pPmicCoreHandle, &pPwrThermalStatCfg);

    pmic_log("Status: %d\n",pPwrThermalStatCfg.thermalStateWarning);
    if(pPwrThermalStatCfg.thermalStateWarning != 0)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7728,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrThermalStat : Test Get Oderly Shutdown Status.
 */
static void test_pmic_powerGetPwrThermalStat_OderlyShtDwnStatus(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_PowerThermalStat_t pPwrThermalStatCfg =
    {
       PMIC_THERMAL_STAT_ORD_SHTDWN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7729,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrThermalStat(pPmicCoreHandle, &pPwrThermalStatCfg);

    pmic_log("Status: %d\n",pPwrThermalStatCfg.thermalStateOderlyShtDwn);
    if(pPwrThermalStatCfg.thermalStateOderlyShtDwn != 0)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7729,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrThermalStat : Test Get immediate Shutdown Status.
 */
static void test_pmic_powerGetPwrThermalStat_ImmShtDwnStatus(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_PowerThermalStat_t pPwrThermalStatCfg =
    {
       PMIC_THERMAL_STAT_IMM_SHTDWN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7730,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrThermalStat(pPmicCoreHandle, &pPwrThermalStatCfg);

    pmic_log("Status: %d\n",pPwrThermalStatCfg.thermalStateImmShtDwn);
    if(pPwrThermalStatCfg.thermalStateImmShtDwn != 0)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7730,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Negative test LDO Pull down Select for HERA PMIC.
 */
static void test_pmic_powerSetPowerResourceConfig_hera_ldo(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7872,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.ldoPullDownSel = PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_50KOHM;
        pwrRsrc = PMIC_TPS6594X_REGULATOR_LDO1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7872,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7872,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Negative test VMON range for LEO PMIC.
 */
static void test_pmic_powerSetPowerResourceConfig_leo_vmon(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7873,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7873,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_0V3_3V34;
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7873,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetThermalConfig : Negative test for thermalShutdownThold on Leo PMIC PG1.0
 */
static void test_pmic_powerSetThermalConfig_thermalShutdownThold(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerThermalCfg_t thermalThreshold =
    {
        PMIC_THERMAL_SHTDWN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7874,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
        (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)) ||
        (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType))
    {
        pmic_testResultUpdate_ignore(7874,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }


    thermalThreshold.thermalShutdownThold =
                            PMIC_TPS6594X_THERMAL_TEMP_TSD_ORD_140C;

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, pmicStatus);

    pmic_testResultUpdate_pass(7874,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetThermalConfig : Negative test for thermalShutdownThold on Leo PMIC PG1.0
 */
static void test_pmic_powerGetThermalConfig_thermalShutdownThold(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7875,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    Pmic_PowerThermalCfg_t thermalThreshold =
    {
        PMIC_THERMAL_SHTDWN_VALID_SHIFT,
    };

    if(((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
        (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)) ||
        (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType))
    {
        pmic_testResultUpdate_ignore(7875,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    thermalThreshold.thermalShutdownThold =
                                        PMIC_TPS6594X_THERMAL_TEMP_TSD_ORD_145C;

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, pmicStatus);

    pmic_testResultUpdate_pass(7875,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetLdoRtc : Negative test Disable ldortcRegulator for HERA
 */
static void test_pmic_powerSetLdoRtc_HERA_ldortcEnable_disable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool ldortcEnable;

    test_pmic_print_unity_testcase_info(7876,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7876,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        ldortcEnable = PMIC_TPS6594X_REGULATOR_LDORTC_DISABLE;
    }

    pmicStatus = Pmic_powerSetLdoRtc(pPmicCoreHandle, ldortcEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7876,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Negative test Get Switch peak current limit for BUCK 5
 */
static void test_pmic_powerGetPowerResourceConfig_buck5(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd   =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(7878,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK5;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

   }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK5;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    }

    pmic_testResultUpdate_pass(7878,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}
/*!
 * \brief   Pmic_powerGetThermalConfig : Parameter validation for pThermalThreshold
 */
static void test_pmic_powerGetThermalConfigPrmValTest_pThermalThreshold(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10622,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10622,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrRsrcStat : Parameter validation for pPwrRsrcStatCfg
 */
static void test_pmic_powerGetPwrRsrcStatPrmValTest_pPwrRsrcStatCfg(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrResource;
    uint16_t pwrRsrcMin, pwrRsrcMax;

    test_pmic_print_unity_testcase_info(10623,
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
                                              NULL);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
    }

    pmic_testResultUpdate_pass(10623,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetConfigPowerGood : Parameter validation for pPgoodSelType
 */
static void test_pmic_powerGetConfigPowerGoodPrmValTest_pPgoodSelType(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;

    test_pmic_print_unity_testcase_info(10624,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                              pgoodSrcSel,
                                              NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10624,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetCommonConfig : Parameter validation for pPowerCommonCfg
 */
static void test_pmic_powerGetCommonConfigPrmValTest_pPowerCommonCfg(void)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10625,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10625,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Negative test for ldoRvTimeoutSel configuration on Hera
 */
 static void test_pmic_powerSetPwrResourceCfg_ldoRvTimeoutSel(void)
 {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10626,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
         pmic_testResultUpdate_ignore(10626,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pwrRsrc = PMIC_TPS6594X_REGULATOR_LDO1;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    pmic_testResultUpdate_pass(10626,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Negative test for Buck2 pwmMpMode configuration
 */
static void test_pmic_powerSetPwrResourceCfg_buck2PwmMpMode(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10627,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMpMode = PMIC_TPS6594X_REGULATOR_PWM_MP_MODE;
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK2;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMpMode = PMIC_LP8764X_REGULATOR_PWM_MP_MODE;
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK2;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10627,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Negative test for Ldo4 bypass mode configuration
 */
static void test_pmic_powerSetPwrResourceCfg_ldo4ByPassMode(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10628,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_LDO4;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10628,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10628,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetIntr : Parameter Validation of intrType
 */
static void test_pmic_powerSetIntrPrmValTest_intrType(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;

    intrEnable = PMIC_POWER_INTERRUPT_ENABLE;
    test_pmic_print_unity_testcase_info(10629,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    intrType = 8U;
    pmicStatus = Pmic_powerSetIntr(pPmicCoreHandle, intrType, intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10629,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

#if defined(SOC_J7200)
/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test to set Power good high threshold level for Buck1
 */
static void test_pmic_powerSetPwrResourceCfg_pgOvThresholdLvl(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID_SHIFT,
    };
    Pmic_PowerResourceCfg_t powerCfg_default =
    {
        PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID_SHIFT,
    };
    uint16_t pwrRsrc;

    test_pmic_print_unity_testcase_info(10630,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             powerCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(powerCfg_default.pgOvThresholdLvl, powerCfg_rd.pgOvThresholdLvl);

    pmic_testResultUpdate_pass(10630,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}
#endif

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for pgOvThresholdLvl
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_pgOvThresholdLvl(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10631,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgOvThresholdLvl = 8U;
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgOvThresholdLvl = 8U;
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10631,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test to set Power good low threshold level for Buck1
 */
static void test_pmic_powerSetPwrResourceCfg_pgUvThresholdLvl(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID_SHIFT,
    };
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10632,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgUvThresholdLvl = PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_30_OR_3;
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgUvThresholdLvl = PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_30_OR_3;
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.pgUvThresholdLvl, powerCfg_rd.pgUvThresholdLvl);

    pmic_testResultUpdate_pass(10632,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for pgUvThresholdLvl
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_pgUvThresholdLvl(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10633,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgUvThresholdLvl = 8U;
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgUvThresholdLvl = 8U;
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10633,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrRsrcStat : Parameter validation for ILIM status pwrResource
 */
static void test_pmic_powerGetPwrRsrcStatPrmValTest_ilimStatPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceStat_t pPwrRsrcStatCfg =
    {
        PMIC_POWER_REGULATOR_ILIM_STAT_VALID_SHIFT,
    };

    uint16_t pwrResource;

    test_pmic_print_unity_testcase_info(10635,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_POWER_RESOURCE_TYPE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA;
    }

    pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                          pwrResource,
                                          &pPwrRsrcStatCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10635,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrRsrcStat : Parameter validation for VCCA OV LVL status pwrResource
 */
static void test_pmic_powerGetPwrRsrcStatPrmValTest_vccaOvLvlStatPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceStat_t pPwrRsrcStatCfg =
    {
        PMIC_POWER_VCCA_OV_LVL_STAT_VALID_SHIFT,
    };

    uint16_t pwrResource;

    test_pmic_print_unity_testcase_info(10636,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                          pwrResource,
                                          &pPwrRsrcStatCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10636,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetCommonConfig : Negative test for deglitchTimeSel on Leo
 */
static void test_pmic_powerGetCommonConfig_deglitchTimeSel(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerCommonCfg_t powerCfg_rd =
    {
        PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10637,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
        (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)) ||
        (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType))
    {
        pmic_testResultUpdate_ignore(10637,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, pmicStatus);

    pmic_testResultUpdate_pass(10637,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetCommonConfig : Negative test for deglitchTimeSel on Leo
 */
static void test_pmic_powerSetCommonConfig_deglitchTimeSel(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10638,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
        (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)) ||
        (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType))
    {
        pmic_testResultUpdate_ignore(10638,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, pmicStatus);

    pmic_testResultUpdate_pass(10638,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for vccaPwrGudLvl pwrResource
 */
 static void test_pmic_powerGetPwrResourceCfgPrmValTest_vccaPwrGudLvlPwrRsrc(void)
 {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
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

    test_pmic_print_unity_testcase_info(10639,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10639,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for vmonRange pwrResource
 */
 static void test_pmic_powerGetPwrResourceCfgPrmValTest_vmonRangePwrRsrc(void)
 {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10640,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(10640,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10640,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for buckCurrentLimit pwrResource
*/
static void test_pmic_powerGetPwrResourceCfgPrmValTest_buckIlimPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10641,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10641,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for buckVmonSlewRate pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_buckVmonSlewRatePwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10642,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10642,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for residual voltage checking pwrResource
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_rvCheckPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;

    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    test_pmic_print_unity_testcase_info(10643,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10643,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for residual voltage checking pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_rvCheckPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10644,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10644,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for buckPullDownEn pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_buckPullDownEnPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10646,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10646,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for buckVoutSel pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_buckVoutSelPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10647,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10647,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for buckFpwmMode pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_buckFpwmModePwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_FPWM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10648,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10648,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for buckFpwmMpMode pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_buckFpwmMpModePwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10649,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10649,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for regulatorEn pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_regulatorEnPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10650,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10650,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for ldoPullDownSel pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_ldoPullDownSelPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10651,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* LDOs are not present in HERA pmic */
        pmic_testResultUpdate_ignore(10651,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10651,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for buckCurrentLimit min value
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_buckIlimMinVal(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10652,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
        pPowerCfg.buckCurrentLimit = 0x01;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
        pPowerCfg.buckCurrentLimit = 0x01;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    pmic_testResultUpdate_pass(10652,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for ldoBypassModeEn pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_ldoBypassModeEnPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10653,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10653,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for ldoRvTimeoutSel pwrResource
 */
 static void test_pmic_powerGetPwrResourceCfgPrmValTest_ldoRvTimeoutSelPwrRsrc(void)
 {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10654,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10654,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Parameter validation for voltage_mV pwrResource
 */
static void test_pmic_powerGetPwrResourceCfgPrmValTest_voltage_mV_pwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10655,
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

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10655,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV as 599mv
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_voltage_mV_599mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10656,
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

    pPowerCfg.voltage_mV = 599U;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10656,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV as 1099mv
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_voltage_mV_1099mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10657,
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

    pPowerCfg.voltage_mV = 1099U;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10657,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV as 1659mV max Value
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_voltage_mV_1659mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10658,
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

    pPowerCfg.voltage_mV = 1659U;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10658,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Negative test for ldoSlowRampEn configuration on PG1.0 Silicon Revision
 */
static void test_pmic_powerSetPwrResourceCfg_ldoSlowRampEn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10659,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        pmic_testResultUpdate_ignore(10659,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_LDO1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, pmicStatus);

    pmic_testResultUpdate_pass(10659,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetLdoRtc : Parameter validation for handle
 */
static void test_pmic_powerSetLdoRtcPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool ldortcEnable;

    test_pmic_print_unity_testcase_info(10660,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        ldortcEnable = PMIC_TPS6594X_REGULATOR_LDORTC_ENABLE;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10660,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetLdoRtc(NULL, ldortcEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(10660,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetLdoRtc : Parameter validation for handle
 */
static void test_pmic_powerGetLdoRtcPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool ldortcEnable_rd;

    test_pmic_print_unity_testcase_info(10661,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10661,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerGetLdoRtc(NULL, &ldortcEnable_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(10661,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetLdoRtc : Parameter validation for pLdortcEnable
 */
static void test_pmic_powerGetLdoRtcPrmValTest_pLdortcEnable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10662,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10662,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerGetLdoRtc(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10662,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for min voltage level for buck
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_buckMinVoltage(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10663,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK2;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK2;
    }

    pPowerCfg.voltage_mV = 299U;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10663,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for max voltage level for buck
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_buckMaxVoltage(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10664,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK2;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK2;
    }

    pPowerCfg.voltage_mV = 3341U;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10664,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for min voltage level for ldo
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_ldoMinVoltage(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10665,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_LDO3;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10665,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pPowerCfg.voltage_mV = 599U;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10665,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for power resource type
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_vccaMinVoltage(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg  =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10666,
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

    pPowerCfg.voltage_mV = 599U;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10666,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Buck Max Value Power Resource limit
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_buckPwrRsrcLimitMaxVal(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;

    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK5 + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK4 + 1;
    }

    test_pmic_print_unity_testcase_info(10667,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10667,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for Buck Min Value Power Resource limit
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_buckPwrRsrcLimitMinVal(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;

    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1 - 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1 - 1;
    }

    test_pmic_print_unity_testcase_info(10668,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10668,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for LDO and VMON Min Value Power Resource limit
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_ldoVmonPwrRsrcLimitMinVal(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;

    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_VMON_EN_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_LDO1 - 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON1 - 1;
    }

    test_pmic_print_unity_testcase_info(10669,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10669,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrRsrcIntr :  Parameter validation for interrupt type pwrResource
 */
static void test_Pmic_powerSetPwrRsrcIntrPrmValTest_intrTypePwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t intrType;
    bool intrEnable;
    uint16_t pwrResource;
    intrEnable = PMIC_POWER_INTERRUPT_DISABLE;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_LP8764X_POWER_ILIM_INT;
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intrType = PMIC_TPS6594X_POWER_ILIM_INT;
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    test_pmic_print_unity_testcase_info(10670,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    pmicStatus = Pmic_powerSetPwrRsrcIntr(pPmicCoreHandle,
                                          pwrResource,
                                          intrType,
                                          intrEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10670,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for TDIE WARN pgoodSelType
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_tdieWarnPgoodSelType(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10671,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_TDIE_WARN + 1;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_TDIE;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_TDIE_WARN + 1;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_TDIE;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10671,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for threshold voltage and current limit pgoodSelType
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_voltCurrentPgoodSelType(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10672,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_VOLTAGE_CURRENT + 1;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_SRC_VOLTAGE_CURRENT + 1;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_BUCK1;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10672,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation NRSTOUT pgoodSelType
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_nRstOutPgoodSelType(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10673,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT + 1;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT + 1;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10673,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for NRSTOUT_SOC pgoodSelType
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_nRstOutSocPgoodSelType(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10674,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT_SOC + 1;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_NRSTOUT_SOC;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT_SOC + 1;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT_SOC;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10674,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for VCCA/VMON Enable pgoodSelType
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_vccaVmonPgoodSelType(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10675,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_VCCA_ENABLE + 1;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE + 1;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_VCCA;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10675,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetLdoRtc : Negative test to get pLdortcEnable configuration on HERA
 */
static void test_pmic_powerGetLdoRtc_pLdortcEnable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool ldortcEnable_rd;

    test_pmic_print_unity_testcase_info(10676,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10676,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerGetLdoRtc(pPmicCoreHandle, &ldortcEnable_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10676,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for Buck power good source max value
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_buckPgoodSrcMaxVal(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10677,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_MASKED;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_BUCK5 + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_SRC_MASKED;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_BUCK4 + 1;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10677,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for Buck power good source min value
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_buckPgoodSrcMinVal(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10678,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_MASKED;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_BUCK1 - 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_SRC_MASKED;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_BUCK1 - 1;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10678,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for VMON power good source max value
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_vmonPgoodSrcMaxVal(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10679,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(10679,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_VMON2 + 1;

        pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 pgoodSelType);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10679,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for VMON power good source min value
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_vmonPgoodSrcMinVal(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10680,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(10680,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE;
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_VMON1 - 1;

        pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 pgoodSelType);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10680,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for LDO power good source max value
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_ldoPgoodSrcMaxVal(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10681,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10681,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_MASKED;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_LDO4 + 1;

        pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 pgoodSelType);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10681,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for LDO power good source min value
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_ldoPgoodSrcMinVal(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10682,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10682,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_MASKED;
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_LDO1 - 1;

        pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                                 pgoodSrcSel,
                                                 pgoodSelType);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10682,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for NRSTOUT power good source
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_nRstOutPgoodSrc(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10683,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT;
        pgoodSrcSel  = 0x204U;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT;
        pgoodSrcSel  = 0x205U;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10683,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for NRSTOUT_SOC power good source
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_nRstOutSocPgoodSrc(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10684,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT_SOC_MASKED;
        pgoodSrcSel  = 0x305U;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_NRSTOUT_SOC_MASKED;
        pgoodSrcSel  = 0x306U;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10684,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for TDIE_WARN power good source
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_tdieWarnPgoodSrc(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10685,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_TDIE_WARN;
        pgoodSrcSel  = 0x406U;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_TDIE_WARN;
        pgoodSrcSel  = 0x407U;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10685,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for VCCA power good source
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_vccaPgoodSrc(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10686,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE;
        pgoodSrcSel  = 0x1U;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_VCCA_ENABLE;
        pgoodSrcSel  = 0x1U;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10686,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetConfigPowerGood : Parameter validation for power good source
 */
static void test_pmic_powerSetConfigPowerGoodPrmValTest_pgoodSrcType(void)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pgoodSrcSel;
    uint8_t  pgoodSelType;

    test_pmic_print_unity_testcase_info(10687,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE;
        pgoodSrcSel  = 0x507U;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSelType = PMIC_TPS6594X_POWER_PGOOD_SEL_SRC_MASKED;
        pgoodSrcSel  = 0x607U;
    }

    pmicStatus = Pmic_powerSetConfigPowerGood(pPmicCoreHandle,
                                             pgoodSrcSel,
                                             pgoodSelType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10687,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for power resource
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_rvCheckEnPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;

    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = 0x406U;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = 0x406U;
    }

    test_pmic_print_unity_testcase_info(10688,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10688,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test to set vmonRange as
 *          PMIC_LP8764X_VMON_RANGE_3V35_5V and voltage_mV as 3350mV
 */
static void test_pmic_powerSetPwrResourceCfg_vmonRange(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10689,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_3V35_5V;
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(10689,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.vmonRange, powerCfg_rd.vmonRange);

    powerCfg_rd.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
        pPowerCfg.voltage_mV = 3350U;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(10689,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for vmonRange as
 *          PMIC_LP8764X_VMON_RANGE_0V3_3V34 and voltage_mV as 299mV minVal
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_vmonRangeVoltage_mV_299mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10690,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_0V3_3V34;
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(10690,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.vmonRange, powerCfg_rd.vmonRange);

    powerCfg_rd.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
        pPowerCfg.voltage_mV = 299U;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);


    pmic_testResultUpdate_pass(10690,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV
 *          when vmonRange as PMIC_LP8764X_VMON_RANGE_0V3_3V34 and voltage_mV
 *          as 3341 mV max Val
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_vmonRangeVoltage_mV_3341mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10691,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_0V3_3V34;
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(10691,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.vmonRange, powerCfg_rd.vmonRange);

    powerCfg_rd.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
        pPowerCfg.voltage_mV = 3341U;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);


    pmic_testResultUpdate_pass(10691,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV
 *          when vmonRange as PMIC_LP8764X_VMON_RANGE_3V35_5V and voltage_mV
 *          as 3349 mV max Val
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_vmonRangeVoltage_mV_3349mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10692,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_3V35_5V;
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(10692,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.vmonRange, powerCfg_rd.vmonRange);

    powerCfg_rd.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
        pPowerCfg.voltage_mV = 3349U;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);


    pmic_testResultUpdate_pass(10692,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for voltage_mV
 *          when vmonRange as PMIC_LP8764X_VMON_RANGE_3V35_5V and voltage_mV
 *          as 5001 mV max Val
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_vmonRangeVoltage_mV_5001mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VMON_RANGE_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10693,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_3V35_5V;
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* VMON1 and VMON2 are not present in LEO pmic */
        pmic_testResultUpdate_ignore(10693,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pPowerCfg.vmonRange, powerCfg_rd.vmonRange);

    powerCfg_rd.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_POWER_SOURCE_VMON2;
        pPowerCfg.voltage_mV = 5001U;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);


    pmic_testResultUpdate_pass(10693,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter validation for vccaPwrGudLvl power resource
 */
static void test_pmic_powerSetPwrResourceCfgPrmValTest_vccaPwrGudLvlPwrRsrc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc = 0U;

    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = 0x1U;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = 0x1U;
    }

    test_pmic_print_unity_testcase_info(10694,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10694,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerGetPwrResourceCfg : Negative for ldoSlowRampEn configuration on PG1.0 Silicon Revision
 */
static void test_pmic_powerGetPwrResourceCfg_ldoSlowRampEn(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrRsrc;

    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10695,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        pmic_testResultUpdate_ignore(10695,
                                     pmic_power_tests,
                                     PMIC_POWER_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrc = PMIC_TPS6594X_REGULATOR_LDO1;
    }

    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             &powerCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, pmicStatus);

    pmic_testResultUpdate_pass(10695,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Parameter range validation for voltage_mV as (min value+1)
 */
static void test_pmic_powerSetPowerResourceConfigPrmRangeTest_voltage_mV(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT,
    };

    test_pmic_print_unity_testcase_info(10696,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
        pPowerCfg.voltage_mV = PMIC_UT_TPS6594X_REGULATOR_BUCK_MIN_VOLTAGE + 1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
        pPowerCfg.voltage_mV = PMIC_UT_LP8764X_REGULATOR_BUCK_MIN_VOLTAGE + 1;
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
        pPowerCfg.voltage_mV = PMIC_UT_TPS6594X_POWER_LDO1_2_3_MIN_VOLTAGE + 1;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        if(pwrRsrc == PMIC_TPS6594X_REGULATOR_LDO4)
        {
            pPowerCfg.voltage_mV = PMIC_UT_TPS6594X_POWER_LDO4_MIN_VOLTAGE + 1;
        }
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    pmic_testResultUpdate_pass(10696,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}

/*!
 * \brief   Added for Coverage
 */
static void test_pmic_power_coverageGaps(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t pwrResource;
    uint8_t  pPgoodSelType_rd;
    uint16_t pgoodSrcSel;
    bool ldortcEnable, ldortcEnable_rd;
    Pmic_PowerResourceStat_t pPwrRsrcStatCfg =
    {
        PMIC_POWER_RESOURCE_UV_STAT_VALID_SHIFT,
    };
    Pmic_PowerCommonCfg_t pwrCommonCfg_rd =
    {
        PMIC_SEVERE_ERR_TRIG_VALID_SHIFT,
    };
    Pmic_PowerCommonCfg_t pwrCommonCfg   =
    {
        PMIC_MODERATE_ERR_TRIG_VALID_SHIFT,
    };
    Pmic_PowerResourceCfg_t powerRsrcCfg_rd =
    {
        PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT,
    };
    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT,
    };
    Pmic_PowerThermalCfg_t thermalThreshold_rd   =
    {
        PMIC_THERMAL_SHTDWN_VALID_SHIFT,
    };
    Pmic_PowerThermalCfg_t thermalThreshold =
    {
        PMIC_THERMAL_SHTDWN_VALID_SHIFT,
    };
    Pmic_PowerThermalStat_t pPwrThermalStatCfg =
    {
       PMIC_THERMAL_STAT_IMM_SHTDWN_VALID_SHIFT,
    };
    int32_t crcTestFlag = PMIC_STATUS_CRC_INIT_VAL;

#if defined(SOC_J721E)
    crcTestFlag = gCrcTestFlag_J721E;
#endif

#if defined(SOC_J7200)
    crcTestFlag = gCrcTestFlag_J7VCL;
#endif

    test_pmic_print_unity_testcase_info(10697,
                                        pmic_power_tests,
                                        PMIC_POWER_NUM_OF_TESTCASES);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;

    //Pmic_powerGetUvOvVccaVoltageStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPwrRsrcStatCfg.validParams = PMIC_POWER_RESOURCE_UV_STAT_VALID_SHIFT;
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }
    pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                           pwrResource,
                                           &pPwrRsrcStatCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetOverVoltageTholdStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPwrRsrcStatCfg.validParams = PMIC_POWER_RESOURCE_OV_STAT_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                          pwrResource,
                                          &pPwrRsrcStatCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);


    //Pmic_powerGetUnderVoltageTholdStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPwrRsrcStatCfg.validParams = PMIC_POWER_RESOURCE_UV_STAT_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                          pwrResource,
                                          &pPwrRsrcStatCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetCurrentLimitLvlStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPwrRsrcStatCfg.validParams = PMIC_POWER_REGULATOR_ILIM_STAT_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                          pwrResource,
                                          &pPwrRsrcStatCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetOverVoltageProtectionLvlStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPwrRsrcStatCfg.validParams = PMIC_POWER_VCCA_OV_LVL_STAT_VALID_SHIFT;

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
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);


    //Pmic_powerGetTriggerSelCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg_rd.validParams = PMIC_SEVERE_ERR_TRIG_VALID_SHIFT;
    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &pwrCommonCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        //Pmic_powerSetLdoRvTimeoutSel
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pwrResource = PMIC_TPS6594X_REGULATOR_LDO1;
        pPowerCfg.ldoRvTimeoutSel = PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_16MS;
        pPowerCfg.validParams = PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrResource,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerSetLdoBypassModeEn
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pwrResource = PMIC_TPS6594X_REGULATOR_LDO1;
        pPowerCfg.ldoBypassModeEn = PMIC_TPS6594X_REGULATOR_LDO_LINEAR_REGULATOR_MODE;
        pPowerCfg.validParams = PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrResource,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerGetLdobypassLdorvtoselRailgrpselVoltagemvCfg
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrResource,
                                                 &powerRsrcCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerGetLdoRvTimeoutSel
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                pwrResource,
                                                &powerRsrcCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerSetRegulatorEn
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pPowerCfg.validParams = PMIC_CFG_REGULATOR_EN_VALID_SHIFT;
        pPowerCfg.regulatorEn = PMIC_TPS6594X_REGULATOR_ENABLE;
        pwrResource = PMIC_TPS6594X_REGULATOR_LDO1;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerGetRegulatorEn
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_EN_VALID_SHIFT;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                pwrResource,
                                                &powerRsrcCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerSetLdoPullDownSel
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pPowerCfg.validParams = PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT;
        pPowerCfg.ldoPullDownSel = PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_500OHM;
        pwrResource = PMIC_TPS6594X_REGULATOR_LDO1;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerGetLdoPullDownSel
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrResource,
                                                 &powerRsrcCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerLdoRtcEnable
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            ldortcEnable = PMIC_TPS6594X_REGULATOR_LDORTC_ENABLE;
        }
        pmicStatus = Pmic_powerSetLdoRtc(pPmicCoreHandle, ldortcEnable);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerGetLdoRtcEnable
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pmicStatus = Pmic_powerGetLdoRtc(pPmicCoreHandle, &ldortcEnable_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);
    }

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        //Pmic_powerSetLdoSlowRampEn
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pPowerCfg.validParams = PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT;
        pPowerCfg.ldoSlowRampEn = PMIC_TPS6594X_REGULATOR_LDO_SLOW_RAMP_DISABLE;
        pwrResource = PMIC_TPS6594X_REGULATOR_LDO1;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerGetLdoSlowRampEn
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                pwrResource,
                                                &powerRsrcCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        //Pmic_powerSetVmonRange
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pPowerCfg.validParams = PMIC_CFG_VMON_RANGE_VALID_SHIFT;
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_3V35_5V;
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerGetVmonRange
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        powerRsrcCfg_rd.validParams = PMIC_CFG_VMON_RANGE_VALID_SHIFT;
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrResource,
                                                 &powerRsrcCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerLP8764xConvertVoltage2VSetVal
        pPowerCfg.validParams = PMIC_CFG_VMON_RANGE_VALID_SHIFT;
        pPowerCfg.vmonRange = PMIC_LP8764X_VMON_RANGE_0V3_3V34;
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VMON1;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrResource,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 2;
        pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;
        pPowerCfg.voltage_mV = 600U;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerLP8764xValidateVoltageLevel
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;
        pPowerCfg.voltage_mV = 3351U;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerLP8764xConvertVSetVal2Voltage
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 2;
        powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrResource,
                                                 &powerRsrcCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);
    }

    //Pmic_getThermalWarnThold
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    thermalThreshold_rd.validParams = PMIC_THERMAL_WARN_VALID_SHIFT;
    pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_setThermalWarnThold
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    thermalThreshold.validParams = PMIC_THERMAL_WARN_VALID_SHIFT;

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)&&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_PG_2_0_THERMAL_TEMP_WARN_140C;
    }

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)&&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
    {
        thermalThreshold.thermalWarnThold = PMIC_TPS6594X_THERMAL_TEMP_WARN_130C;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        thermalThreshold.thermalWarnThold = PMIC_LP8764X_THERMAL_TEMP_WARN_130C;
    }

    pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_getImmediateShutdownStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPwrThermalStatCfg.validParams = PMIC_THERMAL_STAT_IMM_SHTDWN_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrThermalStat(pPmicCoreHandle, &pPwrThermalStatCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_getOderlyShutdownStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPwrThermalStatCfg.validParams = PMIC_THERMAL_STAT_ORD_SHTDWN_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrThermalStat(pPmicCoreHandle, &pPwrThermalStatCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_getThermalWarnStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPwrThermalStatCfg.validParams = PMIC_THERMAL_STAT_WARN_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrThermalStat(pPmicCoreHandle, &pPwrThermalStatCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetModerateRailTrig
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg.validParams = PMIC_MODERATE_ERR_TRIG_VALID_SHIFT;
    pwrCommonCfg.moderateRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetMcuRailTrig
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg.validParams = PMIC_MCU_RAIL_TRIG_VALID_SHIFT;
    pwrCommonCfg.mcuRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetSocRailTrig
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg.validParams = PMIC_SOC_RAIL_TRIG_VALID_SHIFT;
    pwrCommonCfg.socRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetOtherRailTrig
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg.validParams = PMIC_OTHER_RAIL_TRIG_VALID_SHIFT;
    pwrCommonCfg.otherRailTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetSevereErrorTrig
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg.validParams = PMIC_SEVERE_ERR_TRIG_VALID_SHIFT;
    pwrCommonCfg.severeErrorTrig = PMIC_POWER_TRIG_SOC_PWR_ERR;
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetModerateRailTrig
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg_rd.validParams = PMIC_MODERATE_ERR_TRIG_VALID_SHIFT;
    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &pwrCommonCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetMcuRailTrig
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg_rd.validParams = PMIC_MCU_RAIL_TRIG_VALID_SHIFT;
    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &pwrCommonCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetSocRailTrig
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg_rd.validParams = PMIC_SOC_RAIL_TRIG_VALID_SHIFT;
    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &pwrCommonCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetOtherRailTrig
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg_rd.validParams = PMIC_OTHER_RAIL_TRIG_VALID_SHIFT;
    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &pwrCommonCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetPgoodPolarity
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg.validParams = PMIC_POWER_PGOOD_POL_VALID_SHIFT;
    pwrCommonCfg.pgoodPolarity = PMIC_POWER_PGOOD_POL_HIGH;
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetPgoodPolarity
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg_rd.validParams = PMIC_POWER_PGOOD_POL_VALID_SHIFT;
    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &pwrCommonCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetPgoodWindow
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg.validParams = PMIC_POWER_PGOOD_WINDOW_VALID_SHIFT;
    pwrCommonCfg.pgoodWindow = PMIC_POWER_GOOD_UV_OV_MONITOR_ENABLE;
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetVoltageMonitoringPg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg_rd.validParams = PMIC_POWER_PGOOD_WINDOW_VALID_SHIFT;
    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &pwrCommonCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetVccaPwrGudLvl
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vccaPwrGudLvl = PMIC_TPS6594X_VCCA_PG_3V3_LEVEL;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vccaPwrGudLvl = PMIC_LP8764X_VCCA_PG_3V3_LEVEL;
    }
    pwrResource = 0U;
    pPowerCfg.validParams = PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetVccaPwrGudLvl
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_setPwrRsrcPgoodRegBitfieldCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVoutSel = PMIC_TPS6594X_REGULATOR_BUCK_VOUT_SEL_VOUT1;
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVoutSel = PMIC_LP8764X_REGULATOR_BUCK_VOUT_SEL_VOUT1;
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;
    pPowerCfg.voltage_mV = 600U;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetRvCheckEn
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetRvCheckEn
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetBuckVoutSel
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPowerCfg.validParams = PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID_SHIFT;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVoutSel = PMIC_TPS6594X_REGULATOR_BUCK_VOUT_SEL_VOUT2;
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckVoutSel = PMIC_LP8764X_REGULATOR_BUCK_VOUT_SEL_VOUT2;
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetBuckVoutSel
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);


    //Pmic_powerSetBuckFpwmMode
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPowerCfg.validParams = PMIC_CFG_REGULATOR_BUCK_FPWM_VALID_SHIFT;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMode = PMIC_TPS6594X_REGULATOR_PWM_MODE;
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMode = PMIC_LP8764X_REGULATOR_PWM_MODE;
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetBuckFpwmMode
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_BUCK_FPWM_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetBuckFpwmMpMode
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPowerCfg.validParams = PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMpMode = PMIC_TPS6594X_REGULATOR_AUTO_PHASE_MODE;
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckFpwmMpMode = PMIC_LP8764X_REGULATOR_AUTO_PHASE_MODE;
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);


    //Pmic_powerGetBuckPwmMpMode
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetLowThreshold
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPowerCfg.validParams = PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID_SHIFT;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgUvThresholdLvl = PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_30_OR_3;
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgUvThresholdLvl = PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_30_OR_3;
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetLowThreshold
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetHighThreshold
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPowerCfg.validParams = PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID_SHIFT;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgUvThresholdLvl = PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_30_OR_3;
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.pgUvThresholdLvl = PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_30_OR_3;
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetHighThreshold
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetBuckVoutRegAddr
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetRailGrpSel
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPowerCfg.validParams = PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT;
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_TPS6594X_POWER_RAIL_SEL_OTHER;
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.railGrpSel = PMIC_LP8764X_POWER_RAIL_SEL_OTHER;
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetRailGrpSel
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetVmonEn
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonEn = PMIC_TPS6594X_VMON_DISABLE;
        pwrResource = PMIC_TPS6594X_POWER_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.vmonEn = PMIC_LP8764X_VMON_DISABLE;
        pwrResource = PMIC_LP8764X_POWER_SOURCE_VCCA;
    }

    pPowerCfg.validParams = PMIC_CFG_VMON_EN_VALID_SHIFT;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetvmonEn
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_VMON_EN_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetVoltage
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }

    powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pPowerCfg.voltage_mV = powerRsrcCfg_rd.voltage_mV;
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPowerCfg.validParams = PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetBuckPullDownEn
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg.validParams = PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT;
    pwrCommonCfg.pgoodPolarity = PMIC_POWER_PGOOD_POL_LOW;
    pmicStatus = Pmic_powerSetCommonConfig(pPmicCoreHandle, pwrCommonCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetBuckPullDownEn
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pwrCommonCfg_rd.validParams = PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT;
    pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &pwrCommonCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_getPgoodSelType
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSrcSel = PMIC_TPS6594X_PGOOD_SOURCE_VCCA;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pgoodSrcSel = PMIC_LP8764X_PGOOD_SOURCE_VCCA;
    }
    pmicStatus = Pmic_powerGetConfigPowerGood(pPmicCoreHandle,
                                              pgoodSrcSel,
                                              &pPgoodSelType_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetBuckPullDownEn
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPowerCfg.validParams = PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT;
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckPullDownEn = PMIC_TPS6594X_REGULATOR_BUCK_PLDN_DISABLE;
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckPullDownEn = PMIC_LP8764X_REGULATOR_BUCK_PLDN_DISABLE;
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetBuckPullDownEn
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetBuckVmonSlewRate
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT;
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerSetBuckCurrentLimit
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pPowerCfg.validParams = PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT;
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_TPS6594X_REGULATOR_BUCK_CURRENT_LIMIT_2A5;
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckCurrentLimit = PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_2A5;
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_powerGetBuckCurrentLimit
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT;
    pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                             pwrResource,
                                             &powerRsrcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    if(((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)&&
       (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)) ||
       ((PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)&&
       (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)))
    {
        //Pmic_setThermalShutdownThold
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        thermalThreshold.validParams = PMIC_THERMAL_SHTDWN_VALID_SHIFT;

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            thermalThreshold.thermalShutdownThold =
                                    PMIC_TPS6594X_THERMAL_TEMP_TSD_ORD_140C;
        }

        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            thermalThreshold.thermalShutdownThold =
                                PMIC_LP8764X_THERMAL_TEMP_TSD_ORD_140C;
        }
        pmicStatus = Pmic_powerSetThermalConfig(pPmicCoreHandle, thermalThreshold);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_getThermalShutdownThold
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        thermalThreshold_rd.validParams = PMIC_THERMAL_SHTDWN_VALID_SHIFT;
        pmicStatus = Pmic_powerGetThermalConfig(pPmicCoreHandle, &thermalThreshold_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerSetDeglitchTimeSel
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pwrCommonCfg.validParams = PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT;

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
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_powerGetDeglitchTimeSel
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pwrCommonCfg_rd.validParams = PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT;
        pmicStatus = Pmic_powerGetCommonConfig(pPmicCoreHandle, &pwrCommonCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);
    }

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;

    if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
       (PMIC_STATUS_CRC_INIT_VAL == crcTestFlag))
    {
        //Pmic_powerSetLdoBypassModeEn
        gLdoBypassModeEnTestFlag = 1U;
        pPowerCfg.validParams = PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT;
        pwrResource = PMIC_TPS6594X_REGULATOR_LDO3;
        pPowerCfg.ldoBypassModeEn = PMIC_TPS6594X_REGULATOR_LDO_BYPASS_MODE;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrResource,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_powerGetLdoBypassModeEn
        powerRsrcCfg_rd.validParams = PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT;
        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrResource,
                                                 &powerRsrcCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gLdoBypassModeEnTestFlag = 0U;
    }

    if(PMIC_STATUS_CRC_INIT_VAL == crcTestFlag)
    {
        //Pmic_getThermalWarnStat
        //Pmic_getOderlyShutdownStat
        //Pmic_getImmediateShutdownStat
        gThermalStatusTestFlag = 1U;
        pPwrThermalStatCfg.validParams = PMIC_THERMAL_STAT_WARN_VALID_SHIFT |
                                         PMIC_THERMAL_STAT_ORD_SHTDWN_VALID_SHIFT |
                                         PMIC_THERMAL_STAT_IMM_SHTDWN_VALID_SHIFT;
        pmicStatus = Pmic_powerGetPwrThermalStat(pPmicCoreHandle, &pPwrThermalStatCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gThermalStatusTestFlag = 0U;

        //Pmic_powerGetUnderVoltageTholdStat
        //Pmic_powerGetCurrentLimitLvlStat
        gPwrRsrcStatusTestFlag = 1U;
        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
        }

        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
        }
        pPwrRsrcStatCfg.validParams = PMIC_POWER_RESOURCE_UV_STAT_VALID_SHIFT |
                                      PMIC_POWER_REGULATOR_ILIM_STAT_VALID_SHIFT;
        pmicStatus = Pmic_powerGetPwrRsrcStat(pPmicCoreHandle,
                                              pwrResource,
                                              &pPwrRsrcStatCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_powerGetOverVoltageProtectionLvlStat
        pPwrRsrcStatCfg.validParams = PMIC_POWER_VCCA_OV_LVL_STAT_VALID_SHIFT;

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
        gPwrRsrcStatusTestFlag = 0U;
    }

    Pmic_DevSubSysInfo_t pmicDevSubSysInfo =
    {
        .gpioEnable = (bool)true,
        .rtcEnable  = (bool)true,
        .wdgEnable  = (bool)true,
        .buckEnable = (bool)false,
        .ldoEnable  = (bool)false,
        .esmEnable  = (bool)true
    };

    //Pmic_powerLP8764xValidatePwrRsrcLimit
    //Pmic_powerTPS6594xValidatePwrRsrcLimit
    pPmicCoreHandle->pPmic_SubSysInfo = (&pmicDevSubSysInfo);
    pwrResource = PMIC_TPS6594X_REGULATOR_LDO1;
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_REGULATOR_BUCK1;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_LP8764X_REGULATOR_BUCK1;
    }
    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    Pmic_DevSubSysInfo_t testpmicDevSubSysInfo =
    {
        .gpioEnable = (bool)true,
        .rtcEnable  = (bool)true,
        .wdgEnable  = (bool)true,
        .buckEnable = (bool)true,
        .ldoEnable  = (bool)true,
        .esmEnable  = (bool)true
    };

    pPmicCoreHandle->pPmic_SubSysInfo = (&testpmicDevSubSysInfo);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pwrResource = PMIC_TPS6594X_REGULATOR_LDO1;
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle, pwrResource, pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
    }

    pmic_testResultUpdate_pass(10697,
                               pmic_power_tests,
                               PMIC_POWER_NUM_OF_TESTCASES);
}


#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run power unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_power_tests, PMIC_POWER_NUM_OF_TESTCASES);

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
    RUN_TEST(test_pmic_powerSetPowerResourceConfig_regulatorEn_disable);
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

    RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoSlowRampEn_enable);

    RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoSlowRampEn_disable);
    RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_ldoSlowRampEn);

    RUN_TEST(test_pmic_powerSetPowerResourceConfig_voltage_mV);
    RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_voltage_mV);
    RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_voltage_mV_maxVal);
    RUN_TEST(test_pmic_powerSetPowerResourceConfig_vccaPwrGudLvl_5V);
    RUN_TEST(test_pmic_powerSetPowerResourceConfig_vccaPwrGudLvl_3V3);
    RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_vccaPwrGudLvl);
    RUN_TEST(test_pmic_powerSetPowerResourceConfig_railGrpSel_none);

    RUN_TEST(test_pmic_powerSetPowerResourceConfig_railGrpSel_mcu);
    RUN_TEST(test_pmic_powerSetPowerResourceConfig_railGrpSel_soc);
    RUN_TEST(test_pmic_powerSetPowerResourceConfig_railGrpSel_other);

    RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmValTest_PwrRsrc_railGrpSel);
    RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_railGrpSel);
    RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoBypassModeEn_bypass);
    RUN_TEST(test_pmic_powerSetPowerResourceConfig_ldoBypassModeEn_linear);

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

    RUN_TEST(test_pmic_powerSetThermalConfig_thermalShutdownThold_high);

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

    RUN_TEST(test_pmic_powerSetCommonConfig_deglitchTimeSel_20);

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

#if defined(ENABLE_SAMPLE_TESTCASES)
/*   The below taest cases aer dummy, as power/thermal related interrupts cannot
 *   be generated to test masking APIs
 *
 *   7463 PMIC: PMIC Power regulators, VCC and VMON Interrupts features can't
 *   be tested
 */
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ov_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ov_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_uv_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_uv_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ilim_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ilim_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_twarn_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_twarn_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_readback_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_readback_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_soc_pwr_err_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_mcu_pwr_err_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ord_shutdown_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_ord_shutdown_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_imm_shutdown_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_imm_shutdown_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_nrstout_soc_readback_disabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_en_drv_readback_enabled);
    RUN_TEST(test_pmic_powerSetPwrRsrcIntr_en_drv_readback_disabled);
#endif

    RUN_TEST(test_pmic_powerGetPwrThermalStatPrmValTest_handle);
    RUN_TEST(test_pmic_powerGetPwrThermalStatPrmValTest_pPwrThermalStatCfg);
    RUN_TEST(test_pmic_powerGetPwrThermalStat_thermalStatus);
    RUN_TEST(test_pmic_powerGetPwrThermalStat_OderlyShtDwnStatus);
    RUN_TEST(test_pmic_powerGetPwrThermalStat_ImmShtDwnStatus);

    RUN_TEST(test_pmic_powerSetPowerResourceConfig_hera_ldo);
    RUN_TEST(test_pmic_powerSetPowerResourceConfig_leo_vmon);

    RUN_TEST(test_pmic_powerSetThermalConfig_thermalShutdownThold);
    RUN_TEST(test_pmic_powerGetThermalConfig_thermalShutdownThold);

    RUN_TEST(test_pmic_powerSetLdoRtc_HERA_ldortcEnable_disable);
    RUN_TEST(test_pmic_powerGetPowerResourceConfig_buck5);

    RUN_TEST(test_pmic_powerGetThermalConfigPrmValTest_pThermalThreshold);
    RUN_TEST(test_pmic_powerGetPwrRsrcStatPrmValTest_pPwrRsrcStatCfg);
    RUN_TEST(test_pmic_powerGetConfigPowerGoodPrmValTest_pPgoodSelType);
    RUN_TEST(test_pmic_powerGetCommonConfigPrmValTest_pPowerCommonCfg);
    RUN_TEST(test_pmic_powerSetPwrResourceCfg_ldoRvTimeoutSel);
    RUN_TEST(test_pmic_powerSetPwrResourceCfg_buck2PwmMpMode);
    RUN_TEST(test_pmic_powerSetPwrResourceCfg_ldo4ByPassMode);
    RUN_TEST(test_pmic_powerSetIntrPrmValTest_intrType);
#if defined(SOC_J7200)
    RUN_TEST(test_pmic_powerSetPwrResourceCfg_pgOvThresholdLvl);
#endif
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_pgOvThresholdLvl);
    RUN_TEST(test_pmic_powerSetPwrResourceCfg_pgUvThresholdLvl);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_pgUvThresholdLvl);
    RUN_TEST(test_pmic_powerGetPwrRsrcStatPrmValTest_ilimStatPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrRsrcStatPrmValTest_vccaOvLvlStatPwrRsrc);

    RUN_TEST(test_pmic_powerGetCommonConfig_deglitchTimeSel);
    RUN_TEST(test_pmic_powerSetCommonConfig_deglitchTimeSel);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_vccaPwrGudLvlPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_vmonRangePwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_buckIlimPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_buckVmonSlewRatePwrRsrc);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_rvCheckPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_rvCheckPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_buckPullDownEnPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_buckVoutSelPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_buckFpwmModePwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_buckFpwmMpModePwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_regulatorEnPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_ldoPullDownSelPwrRsrc);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_buckIlimMinVal);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_ldoBypassModeEnPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_ldoRvTimeoutSelPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfgPrmValTest_voltage_mV_pwrRsrc);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_voltage_mV_599mV);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_voltage_mV_1099mV);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_voltage_mV_1659mV);
    RUN_TEST(test_pmic_powerSetPwrResourceCfg_ldoSlowRampEn);
    RUN_TEST(test_pmic_powerSetLdoRtcPrmValTest_handle);
    RUN_TEST(test_pmic_powerGetLdoRtcPrmValTest_handle);
    RUN_TEST(test_pmic_powerGetLdoRtcPrmValTest_pLdortcEnable);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_buckMinVoltage);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_buckMaxVoltage);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_ldoMinVoltage);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_vccaMinVoltage);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_buckPwrRsrcLimitMaxVal);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_buckPwrRsrcLimitMinVal);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_ldoVmonPwrRsrcLimitMinVal);
    RUN_TEST(test_Pmic_powerSetPwrRsrcIntrPrmValTest_intrTypePwrRsrc);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_tdieWarnPgoodSelType);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_voltCurrentPgoodSelType);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_nRstOutPgoodSelType);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_nRstOutSocPgoodSelType);

    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_vccaVmonPgoodSelType);
    RUN_TEST(test_pmic_powerGetLdoRtc_pLdortcEnable);

    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_buckPgoodSrcMaxVal);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_buckPgoodSrcMinVal);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_vmonPgoodSrcMaxVal);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_vmonPgoodSrcMinVal);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_ldoPgoodSrcMaxVal);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_ldoPgoodSrcMinVal);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_nRstOutPgoodSrc);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_nRstOutSocPgoodSrc);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_tdieWarnPgoodSrc);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_vccaPgoodSrc);
    RUN_TEST(test_pmic_powerSetConfigPowerGoodPrmValTest_pgoodSrcType);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_rvCheckEnPwrRsrc);

    RUN_TEST(test_pmic_powerSetPwrResourceCfg_vmonRange);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_vmonRangeVoltage_mV_299mV);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_vmonRangeVoltage_mV_3341mV);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_vmonRangeVoltage_mV_3349mV);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_vmonRangeVoltage_mV_5001mV);
    RUN_TEST(test_pmic_powerSetPwrResourceCfgPrmValTest_vccaPwrGudLvlPwrRsrc);
    RUN_TEST(test_pmic_powerGetPwrResourceCfg_ldoSlowRampEn);
    RUN_TEST(test_pmic_powerSetPowerResourceConfigPrmRangeTest_voltage_mV);
    RUN_TEST(test_pmic_power_coverageGaps);

    pmic_updateTestResults(pmic_power_tests, PMIC_POWER_NUM_OF_TESTCASES);

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

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.i2c2Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C2_SPEED_VALID_SHIFT;

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J721E_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

        pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICA_PAGE1_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    }
    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J7VCL_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J7VCL_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

        pmicConfigData.nvmSlaveAddr        = J7VCL_LEO_PMICA_PAGE1_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;
    }

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

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr          = J721E_LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr        = J721E_LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICB_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

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

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr          = J7VCL_HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr        = J7VCL_HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J7VCL_HERA_PMIC_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

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
 * \brief   POWER Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_power_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SPI;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

static int32_t setup_pmic_interrupt(uint32_t board, bool enableCRC)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(J721E_BOARD == board)
    {
        if(enableCRC == true)
        {
            if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J721E)
            {
                gCrcTestFlag_J721E = PMIC_CFG_TO_ENABLE_CRC;
            }
        }
        else
        {
            gCrcTestFlag_J721E = PMIC_STATUS_CRC_INIT_VAL;
        }

        pmic_device_info = J721E_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_power_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            status = test_pmic_leo_pmicB_power_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else if(J7VCL_BOARD == board)
    {
        if(enableCRC == true)
        {
            if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J7VCL)
            {
                gCrcTestFlag_J7VCL = PMIC_CFG_TO_ENABLE_CRC;
            }
        }
        else
        {
            gCrcTestFlag_J7VCL = PMIC_STATUS_CRC_INIT_VAL;
        }

        pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_power_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_power_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    return status;
}

static const char pmicTestMenu[] =
{
    " \r\n ================================================================="
    " \r\n Test Menu:"
    " \r\n ================================================================="
    " \r\n 0: Automatic run for all board specific Power options"
    " \r\n 1: Manual run for Power options"
    " \r\n 2: quit"
    " \r\n"
    " \r\n Enter option: "
};

volatile static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM)"
    " \r\n 2: Pmic Leo device(PMIC A on J7VCL EVM)"
    " \r\n 3: Pmic Hera device(PMIC B on J7VCL EVM)"
    " \r\n 4: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 5: Pmic Leo device(PMIC A on J7VCL EVM Using SPI Stub Functions)"
    " \r\n 6: Pmic Leo device(PMIC A on J721E EVM Manual Testcase with CRC Disable on PG2.0 Silicon revision)"
    " \r\n 7: Back to Test Menu"
    " \r\n"
    " \r\n Enter option: "
};

static void test_pmic_power_testapp_run_options(int8_t option)
{
    int8_t num = -1;
    int8_t idx = 0;
#if defined(SOC_J721E)
    int8_t automatic_options[] = {0, 1, 4};
    int8_t  userInput = 0;
#elif defined(SOC_J7200)
    int8_t automatic_options[] = {2, 3, 5};
#endif

    while(1U)
    {
        if(idx >= (sizeof(automatic_options)/sizeof(automatic_options[0])))
        {
            pmic_printTestResult(pmic_power_tests, PMIC_POWER_NUM_OF_TESTCASES);
        }
        pmic_log("%s", pmicTestAppMenu);
        if(option == PMIC_UT_AUTOMATE_OPTION)
        {
            if(idx < (sizeof(automatic_options)/sizeof(automatic_options[0])))
            {
                num = automatic_options[idx++];
            }
            else
            {
                num = 7;
            }
            pmic_log("%d\n", num);
        }
        else
        {
            if(UART_scanFmt("%d", &num) != 0U)
            {
                pmic_log("Read from UART Console failed\n");
                return;
            }
        }
        switch(num)
        {
            case 0U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                   pmic_device_info = J721E_LEO_PMICA_DEVICE;

                   /* POWER Unity Test App wrapper Function for LEO PMIC-A */
                   if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_power_testApp())
                   {
                       /* Run power test cases for Leo PMIC-A */
                       test_pmic_run_testcases();
                   }
                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 1U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                   pmic_device_info = J721E_LEO_PMICB_DEVICE;

                   /* POWER Unity Test App wrapper Function for LEO PMIC-B */
                   if(PMIC_ST_SUCCESS == test_pmic_leo_pmicB_power_testApp())
                   {
                       /* Run power test cases for Leo PMIC-B */
                       test_pmic_run_testcases();
                   }
                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 2U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                   pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                   /* POWER Unity Test App wrapper Function for LEO PMIC-A */
                   if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_power_testApp())
                   {
                       /* Run power test cases for Leo PMIC-A */
                       test_pmic_run_testcases();
                   }
                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 3U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                   pmic_device_info = J7VCL_HERA_PMICB_DEVICE;

                   /* POWER Unity Test App wrapper Function for HERA PMIC */
                   if(PMIC_ST_SUCCESS == test_pmic_hera_power_testApp())
                   {
                       /* Run power test cases for Hera PMIC */
                       test_pmic_run_testcases();
                   }
                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 4U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* POWER Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                    if(PMIC_ST_SUCCESS ==
                           test_pmic_leo_pmicA_spiStub_power_testApp())
                    {
                        /* Run power test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 5U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* POWER Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                     if(PMIC_ST_SUCCESS ==
                            test_pmic_leo_pmicA_spiStub_power_testApp())
                    {
                        /* Run power test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 6U:
#if defined(SOC_J721E)
                pmic_log("\r\n Run the test only on Leo PMIC-A PG2.0 after Board Power cycle !!!\n");
                pmic_log("\r\n Enter 1 to continue");
                UART_scanFmt("%d", &userInput);

                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, false))
                {
                   pmic_device_info = J721E_LEO_PMICA_DEVICE;

                   /* POWER Unity Test App wrapper Function for LEO PMIC-A */
                   if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_power_testApp())
                   {
                       /* Run power test cases for Leo PMIC-A */
                       test_pmic_run_testcases();
                   }
                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 7U:
               pmic_log(" \r\n Back to Test Menu options\n");
               return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

/*!
 * \brief   Function to register POWER Unity Test App wrapper to Unity framework
 */
static void test_pmic_power_testapp_runner(void)
{
    /* @description : Test runner for POWER Test App
     *
     * @requirements: 5841, 5850
     *
     * @cores       : mcu1_0, mcu1_1
     */

    int8_t option = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestMenu);
        if(UART_scanFmt("%d", &option) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(option)
        {
            case PMIC_UT_AUTOMATE_OPTION:
                test_pmic_power_testapp_run_options(PMIC_UT_AUTOMATE_OPTION);
               break;
            case PMIC_UT_MANUAL_OPTION:
                test_pmic_power_testapp_run_options(PMIC_UT_MANUAL_OPTION);
               break;
            case 2U:
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
    Board_initUART();

    pmic_print_banner("PMIC Power Unity Test Application");

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_power_testapp_runner();
#endif
}
