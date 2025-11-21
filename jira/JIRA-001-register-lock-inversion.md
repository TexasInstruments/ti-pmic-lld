## Summary
Register lock security mechanism operates backwards in TPS65036x-Q1

## Description
The register lock enable/disable functions have inverted logic:
- `Pmic_enableRegLock()` calls `Pmic_setRegLockState(PMIC_UNLOCK)` - unlocks instead of locking
- `Pmic_disableRegLock()` calls `Pmic_setRegLockState(PMIC_LOCK)` - locks instead of unlocking

This completely inverts the security model. Users calling enable expecting protection actually unlock registers, exposing them to modification.

**Location:** devices/TPS65036x-Q1/src/pmic_core.c lines 552-560

**Fix:** Swap PMIC_LOCK and PMIC_UNLOCK constants in both functions.

**Violations:** PMICDRV-546 (aid_core_reglock_001)

## Priority
P1

## Labels
derby

## Release Version
REL.PMIC.TPS65036X.00.05.02
