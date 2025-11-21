## Summary
Wrong validation flag used for watchdog threshold2 configuration in TPS65386x-Q1

## Description
When setting watchdog threshold2 value, the code validates using the threshold1 flag (PMIC_CFG_WDG_THRESHOLD_1_VALID) instead of the threshold2 flag (PMIC_CFG_WDG_THRESHOLD_2_VALID).

This means threshold2 can be set without proper validation when only threshold1 is marked valid, or threshold2 won't be set when it should be.

**Location:** devices/TPS65386x-Q1/src/pmic_wdg.c line 177

**Fix:** Change validation check from PMIC_CFG_WDG_THRESHOLD_1_VALID to PMIC_CFG_WDG_THRESHOLD_2_VALID.

**Violations:** PMICDRV-519, PMICDRV-523

## Priority
P1

## Labels
blackbird

## Release Version
REL.PMIC.TPS65386X.00.11.01
