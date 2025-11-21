## Summary
Write function checks read pointer instead of write pointer in LP8772x-Q1

## Description
In `Pmic_ioTxByte()` write function, the code validates the read function pointer (`pFnPmicCommIoRd`) instead of the write function pointer (`pFnPmicCommIoWr`) before performing a write operation.

If the write function pointer is actually NULL, the check passes (because it's checking the wrong pointer), then the code attempts to call the NULL write pointer, causing a crash or undefined behavior.

**Location:** devices/LP8772x-Q1/src/pmic_io.c line 193

**Fix:** Change `pFnPmicCommIoRd` to `pFnPmicCommIoWr` in the NULL pointer check.

**Violations:** PMICDRV-504 (aid_error_detection_and_reporting)

## Priority
P1

## Labels
coach

## Release Version
REL.PMIC.LP8772X.00.13.00
