## Summary
Critical section functions missing handle NULL check in TPS65386x-Q1

## Description
The critical section functions `Pmic_criticalSectionStart()` and `Pmic_criticalSectionStop()` dereference the handle pointer to check the function pointer without first validating that handle itself is not NULL.

If a NULL handle is passed, the code immediately dereferences it when checking `handle->pFnPmicCritSecStart`, causing a crash.

This is particularly critical because:
- Critical sections protect multi-threaded access
- This is safety-critical code
- Other devices (LP8772x-Q1) properly check for NULL handle

**Location:** devices/TPS65386x-Q1/src/pmic_common.c lines 50-60

**Fix:** Add handle NULL check before dereferencing, following the pattern used in LP8772x-Q1.

**Violations:** PMICDRV-504 (error detection), PMICDRV-502 (safety requirements)

**Note:** TODO.txt incorrectly lists this as LP8772x-Q1 bug, but LP8772x-Q1 implementation is correct.

## Priority
P1

## Labels
blackbird

## Release Version
REL.PMIC.TPS65386X.00.11.01
