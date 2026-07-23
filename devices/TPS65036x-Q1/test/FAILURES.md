# TPS65036x-Q1 Test Failure Tracker

Last updated: 2026-05-12  
Last test run: `run_023.log` — **818 Tests / 5 Failures / 28 Ignored**  
Suite runtime: **~38 seconds**

## Next Session — Start Here

**Current stable failure set (4):**
- `test_pos_fsm_getClrRecovCnt` — recovery counter not clearing to 0 (Category 3)
- `test_pos_fsm_getClrResetCnt` — reset counter not clearing to 0 (Category 3)
- `test_pos_power_buckSetGetCfg_multiParam_buck2_fullConfig` — intermittent CTRL write rejection (Category 4)
- `test_pos_power_buckSetGetCfg_vsetActive_buck2_boundary` — stable VSET write failure (Category 4)

**Intermittent (appeared in run_023, not stable):**
- `test_pos_power_buckSetGetCfg_multiParam_buck2_enableVsetActiveThresholds` — I2C COMM FAIL on SetCfg (Category 4)

**Intermittent (run_020 only, not seen since):**
- `test_pos_power_buckSetGetCfg_multiParam_buck1_enableVsetPldnFpwm` — CTRL write rejected (Category 4)

**Investigation theories are documented below (Category 3 and 4).** Discuss with user before pursuing any fix.

**⚠ VSYS power cycle required between runs** (see Category 5 below).

**To run next session:**
```bash
cd devices/TPS65036x-Q1/test
PMIC_SERIAL_PORT=/dev/cu.usbmodem2132201 make test BUILD=host 2>&1 | tee run_024.log
```

---

## Status Legend
- `[ ]` Pending investigation
- `[~]` Root cause identified, fix not yet applied
- `[x]` Fixed and verified
- `[I]` Ignored (deferred)

---

## Category 0 — COLD BOOT reset architecture `[x]`

**Root cause (discovered this session):** `platform_softReboot()` was issuing a
WARM RESET (0xCC). WARM RESET restores only a short list of registers from NVM
(BUCKx_VSET, WD state, ESM). All other configuration registers — MON_CONF, CTRL
mode bits, SEQ_TRIG, SEQ_DLY, fault-response CONF — survive WARM RESET unchanged.
Any test that modifies these registers leaves the PMIC unable to complete its
power-up sequence on the next setUp() warm reset, causing the nRSTOUT poll to time
out at 202ms, entering SAFE state, accumulating RECOV_CNT, and eventually locking
the PMIC permanently until a VSYS power cycle.

**Fix:** Replace WARM RESET (0xCC) with **COLD BOOT** (0x55) in
`platform_softReboot()`. COLD BOOT runs the full INIT → NVM reload → ABIST →
STANDBY → ACTIVE sequence, restoring every NVM-backed register to factory defaults.
This eliminates all register-persistence cascades with no per-register bookkeeping.

**Result:** Zero 202ms timeouts across 818 tests. Reboot time increased from ~2ms
(warm reset) to ~12ms (cold boot) per setUp(), which is acceptable.

**Also removed** (now dead code): `platform_restorePreRebootState()`, the
`g_porSnapshot` capture, and the BUCKx_EN re-enable RMW in setUp(). These were all
workarounds for WARM RESET not restoring registers. The snapshot infrastructure
remains in platform.c but is no longer called.

---

## Category 1 — WDG SetCfg (8 tests) `[x]`

**Resolution:** CONFIG_CRC-16 monitor (`I2C_CRC_EN=1` from NVM) was silently
discarding all writes to config registers. Fixed in `platform_unlockRegisters()` by
writing `0x00` to `CONFIG_CRC_CONFIG (0x5F)` after every soft reboot. All 8 tests
now pass.

---

## Category 2 — WDG QA Sequence + SwTrigger (10 tests) `[I]`

**Tests ignored:**
- `test_pos_wdg_wdgSendSwTrigger_detectNoErrors` ← new (added this session)
- `test_pos_wdg_wdgSendSwTrigger_detectTrigEarlyErr`
- `test_pos_wdg_wdgQaSequence_detectNoErrors` ← new (added this session)
- `test_pos_wdg_wdgQaSequence_detectAnswErr`
- `test_pos_wdg_wdgQaSequence_detectSeqErr`
- `test_pos_wdg_wdgQaSequence_detectAnswEarlyErr`
- `test_pos_wdg_wdgQaSequence_detectTimeoutErr`
- `test_pos_wdg_wdgQaSequence_detectLongWinTimeoutErr`
- `test_pos_wdg_wdgQaSequence_detectFailInt`
- `test_pos_wdg_wdgQaSequence_detectRstInt`

**Known root cause:** WDG FSM must be frozen in Long Window before calling
`Pmic_wdgSetCfg()`. The fix (`Pmic_wdgSetPowerHold(PMIC_ENABLE)` +
`platform_timerWaitMs(25U)`) is already present in `wdg_enableAndConfigure()` but
the QA sequence tests still fail, suggesting the WDG window timing itself (not just
config writes) is the issue.

Now that COLD BOOT is in place the WDG LFSR is always freshly reset before each
test, removing the LFSR-state carry-over that was masking the real failure. These
tests should be revisited with a logic analyzer to characterise the window timing.

---

## Category 3 — FSM counter failures (2 tests) `[~]`

### 3a. `test_pos_fsm_getClrRecovCnt`

Counter does not clear to `0U` after `Pmic_fsmClrRecovCnt()` following a
`PMIC_SAFE_RECOVERY_REQUEST`. Final assertion at `fsm_test.c:327` fails.

### 3b. `test_pos_fsm_getClrResetCnt`

Counter does not clear to `0U` after `Pmic_fsmClrResetCnt()` following a
`PMIC_WARM_RESET_REQUEST`. Final assertion at `fsm_test.c:364` fails.

### Investigated theories (2026-05-11)

**Test flow (both tests):**
1. Read counter (RECOV_CNT_REG_1, 0x5F — bits [3:0] for RECOV_CNT, [7:4] for RESET_CNT)
2. Issue FSM command (SAFE_RECOVERY or WARM_RESET) to FSM_COMMAND_REG (0x05) — I2C fail tolerated
3. `Pmic_irqClrAllFlags()` — no explicit wait for PMIC to return to ACTIVE
4. Call `Pmic_fsmClrXxx()` — writes 0x01 or 0x02 to RECOV_CNT_CONTROL (0x07) — I2C fail tolerated
5. Assert counter == 0  ← **FAILS**

**Theory 1 (Most likely): PMIC not yet in ACTIVE when clear is called**

The FSM command puts the PMIC into SAFE or WARM RESET state. The recovery sequence
takes some time before the device returns to ACTIVE. The clear write to 0x07 arrives
while the PMIC is mid-recovery and is silently ignored — exactly the same mechanism
documented in Category 5 for the prior RECOV_CNT_CLR bug. No wait / nRSTOUT poll
separates the FSM command from the clear call.

Supported by: FAILURES.md Category 5 explicitly documents that RECOV_CNT_CLR
writes during mid-recovery are ignored.

**Theory 2 (High): Register re-lock after in-test FSM command**

WARM_RESET and SAFE_RECOVERY both trigger a PMIC reset sequence. After the reset,
NVM is reloaded, which may restore REGISTER_LOCK. The test does NOT call any
unlock after the FSM command (only setUp's cold-boot path calls
`platform_unlockRegisters()`). If the register lock re-engages, the write to
RECOV_CNT_CONTROL (0x07) is silently discarded.

**Next steps:**
- [ ] Add a `platform_timerWaitMs(20)` after `Pmic_irqClrAllFlags()` in both tests,
      then re-run. If tests pass → Theory 1 confirmed.
- [ ] If still failing, add `platform_unlockRegisters()` call after the wait.
      If tests then pass → Theory 2 confirmed (or both together required).
- [ ] Logic analyzer on I2C: confirm whether nRSTOUT is high before the 0x07 write.

---

## Category 4 — Hardware write-protected registers `[~]`

### Power test failure overview

| Test | Type | Failing assertion | Register |
|---|---|---|---|
| `buckSetGetCfg_vsetActive_buck2_boundary` | Stable | readback ≠ written | BUCK2_VOUT_ACTIVE (0x12) |
| `buckSetGetCfg_uvloFalling_buck1_allValues` | **Fixed** | — | BUCK1_UVLO (0x1B) bits [7:4] |
| `buckSetGetCfg_multiParam_buck2_fullConfig` | Intermittent | pldnEn readback mismatch / status error | BUCK2_CTRL (0x19) |
| `buckSetGetCfg_multiParam_buck1_enableVsetPldnFpwm` | Intermittent | fpwmEn readback mismatch | BUCK1_CTRL (0x18) |
| `buckSetGetCfg_multiParam_buck2_enableVsetActiveThresholds` | Intermittent | PMIC_ST_ERR_I2C_COMM_FAIL | BUCK2_CTRL (0x19) or VOUT (0x12) |

**`uvloFalling` fix (2026-05-12):** Test was sweeping FALLING values without first setting RISING to max. Hardware enforces a minimum hysteresis (FALLING ≤ RISING − 4 codes). Fix: pre-set RISING to max (0xF), restrict loop to `fallingMax = 0xB`. Now passes consistently.

### 4a. Stable failures — vsetActive_buck2_boundary and uvloFalling_buck1_allValues

**Values written:**
- `vsetActive`: 0x05 (BUCK2_3_VSET_MIN+1) and 0x44 (BUCK2_3_VSET_MAX-1). Both are within the
  driver's accepted range [0x04, 0x45]. Encoding is identity (no offset/translation).
- `uvloFalling`: 0–15 (all values in 4-bit field, bits [7:4] of BUCK1_UVLO_REG 0x1B).
  Written via RMW that preserves bits [3:0] (uvloRising).

**Theory 1 (Most likely): `pmicHandle.crcEnable` is stale after cold boot**

`Pmic_init()` is called once at module start and sets `pmicHandle.crcEnable` from
hardware state. Every cold boot in setUp() reloads NVM which restores
`I2C_CRC_EN = 1`. `setUp()` correctly updates `g_pmic_crc_enabled = true` (used by
`platform_writeReg`), but it does NOT update `pmicHandle.crcEnable`. If any prior
test changed the handle's CRC field (e.g., via `Pmic_ioSetCrcEnableState`), all
subsequent driver calls silently drop writes because the driver sends frames without
a CRC byte while the hardware expects one. The hardware ACKs the address but ignores
the write (same root cause as Category 1, but manifesting through the driver handle
rather than platform_writeReg).

Supported by: Category 1 fix was exactly this issue in platform_writeReg. The
driver handle is the parallel, unfixed path.

**Theory 2 (Medium): CONFIG_CRC_CONFIG (0x64) write lands on wrong register for A0 silicon**

`platform_unlockRegisters()` writes 0x00 to 0x64 to disable the CONFIG_CRC monitor.
`Pmic_ioTxByte`/`Pmic_ioRxByte` apply an address offset of -3 for A0 silicon
(addresses ≥ 0x4D shift to 0x4A+). However `platform_writeReg` uses raw I2C and
does NOT apply the A0 offset. If the device is A0 silicon, the write to 0x64 hits
the wrong register and the CONFIG_CRC monitor remains active, silently discarding
subsequent configuration writes.

Supported by: All failing register addresses (0x12, 0x1B) are below 0x4D so the
A0 offset wouldn't affect the failing writes themselves — but the unlock write to
0x64 would be misaddressed, leaving the monitor active.

**Next steps:**
- [ ] Print `pmicHandle.crcEnable` at the start of both failing tests. If 0 (false)
      while hardware CRC is on, Theory 1 is confirmed.
- [ ] Check whether any earlier power test calls `Pmic_ioSetCrcEnableState()` which
      would flip the handle field and corrupt all subsequent tests.
- [ ] Check `pmicHandle.isA0` / `pmicHandle.devRev` to evaluate Theory 2.
- [ ] Logic analyzer: capture I2C during a failing test — check whether the write
      frame to BUCK2_VOUT_ACTIVE (0x12) or BUCK1_UVLO (0x1B) is 2 bytes (no CRC)
      or 3 bytes (with CRC). If 2 bytes and hardware expects 3, that confirms Theory 1.

### 4b. Intermittent failures — multiParam BUCK1/BUCK2 tests

All three intermittent tests set `enable = true` and fail specifically on
`BUCKx_CTRL` register fields (`fpwmEn`, `pldnEn`), which live in BUCK1_CTRL (0x18)
/ BUCK2_CTRL (0x19).

**Theory 1 (Most likely): BUCKx_CTRL is write-protected when the regulator is actively enabled**

After COLD BOOT, NVM restores BUCK2 as enabled. The tests attempt to write
`fpwmEn` and `pldnEn` to the CTRL register while the regulator is already actively
regulating. PMICs commonly protect switching-mode control bits (FPWM, pulldown)
from being changed mid-operation to prevent output glitches. Intermittency comes
from a timing race: if the test executes quickly enough after setUp(), the
regulator may not yet be in the fully protected window.

Two failure modes observed:
- Silent discard (readback mismatch, PMIC_ST_SUCCESS returned) — hardware ignores
  write without NACKing
- I2C NACK (PMIC_ST_ERR_I2C_COMM_FAIL) — hardware actively rejects the write

Both are consistent with conditional write protection depending on FSM/regulator state.

**Theory 2 (Medium): CONFIG_CRC monitor active at time of CTRL write**

Same as Theory 2 for stable failures. CONFIG_CRC monitor may fire during the
critical window between cold boot and `platform_unlockRegisters()`, silently
discarding subsequent writes. The intermittency could reflect whether the monitor
is still active at the time the BUCKx_CTRL write arrives.

**Next steps:**
- [ ] Logic analyzer: capture I2C during a run where `multiParam_buck2_fullConfig`
      fails. Check whether BUCK2_CTRL write (addr 0x19) is ACKed or NACKed, and
      whether the write arrives before or after the CONFIG_CRC disable write (0x64).
- [ ] Diagnostic-only test modification: write BUCK2_EN=0 to disable the regulator
      before calling `Pmic_pwrSetBuckCfg`, then re-enable after. If tests pass
      consistently, regulator-active write protection is confirmed.
- [ ] Check TPS65036x datasheet §7.6 for BUCK_CTRL register write conditions —
      look specifically for FPWM_EN and PLDN write-enable conditions.

---

## Category 5 — PMIC state degradation across runs `[x]`

**Root cause:** Each SAFE state entry during a run increments RECOV_CNT. Our
`RECOV_CNT_CLR` write (reg 0x07 bit 0) after each 202ms timeout lands while the
PMIC is still mid-recovery (not yet in ACTIVE), so the clear is ignored. RECOV_CNT
accumulates until it reaches `RECOV_CNT_THR`, after which the PMIC stays in SAFE
permanently until a VSYS power cycle. A TM4C123 MCU reset does not cycle VSYS.

**Fix:** The COLD BOOT change (Category 0) eliminates SAFE state entries entirely,
so RECOV_CNT never accumulates. No per-run workaround is needed.

**Residual risk:** If a future test introduces a new cascade (e.g., a direct write
to a register not covered by NVM restore), SAFE entries would return. If that
happens the PMIC board must be physically power-cycled before the next run.

---

## I/O Retry Tests — Mock-only `[x]`

`test_pos_io_ioTxByte_withRetryOnFailure` and `test_pos_io_ioTxByte_retrySucceedsOnLastAttempt`
wrapped in `#ifdef BUILD_MOCK` with `TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK")`.
Both ignored on BUILD=host, passing on BUILD=mock.

---

## Notes

- All hardware failures are "Expression Evaluated To FALSE" — readback does not
  match written value. No unexpected `PMIC_ST_ERR_*` status codes (except intermittent
  `PMIC_ST_ERR_I2C_COMM_FAIL` on `multiParam_buck2_enableVsetActiveThresholds`).
- Register lock (`REGISTER_LOCK = 0x09`) is not the cause for most failures.
  `platform_softReboot()` calls `platform_unlockRegisters()` after every cold boot.
- I2C CRC is working in `platform_writeReg`. The driver handle path (`pmicHandle.crcEnable`)
  is a separate potential issue — see Category 4, Theory 1.
- A0 silicon address translation is not a factor for the failing register addresses
  (all below 0x4D), but may affect the CONFIG_CRC_CONFIG unlock write (0x64).
- COLD BOOT reboot time is ~12ms vs ~2ms for WARM RESET. This is acceptable and
  the total suite runtime (~39s) is well within budget.
