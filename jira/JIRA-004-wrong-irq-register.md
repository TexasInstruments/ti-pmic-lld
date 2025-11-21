## Summary
IRQ handler reads wrong register for BUCK3/LDO/LS1/VMON1 interrupts in LP8772x-Q1

## Description
The IRQ handler for BUCK3/LDO/LS1/VMON1 interrupts reads from `INT_COMM_ERR_REG` instead of `INT_BUCK3_LDO_LS1_VMON1_REG`.

The code comment explicitly states "read and extract bits from L1 register INT_BUCK3_LDO_LS1_VMON1" but the implementation reads the wrong register.

This causes:
- BUCK3, LDO, LS1, and VMON1 interrupts not properly detected
- Interrupt status not cleared, potentially causing interrupt storms
- Incorrect power rail status reporting

**Location:** devices/LP8772x-Q1/src/pmic_irq.c line 622

**Fix:** Change `INT_COMM_ERR_REG` to `INT_BUCK3_LDO_LS1_VMON1_REG`.

**Violations:** PMICDRV-537 (aid_irq_001)

## Priority
P1

## Labels
coach

## Release Version
REL.PMIC.LP8772X.00.13.00
