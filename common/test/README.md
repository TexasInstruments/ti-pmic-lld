# PMIC Test Infrastructure

This directory contains shared test infrastructure used across all PMIC device test harnesses.

## Infrastructure Components

### 1. Debug Logging System (`debug.h` / `debug.c`)
Zero-overhead debug logging with runtime control

### 2. Test Filtering System (`test_filter.h` / `test_filter.c`)
Flexible test execution control without recompilation

### 3. Performance Timing System (`test_timer.h` / `test_timer.c`)
Hierarchical timing instrumentation for performance tracking

---

## 1. Debug Logging System

### Overview

The debug logging system provides module-based, level-controlled logging with **zero overhead** when disabled. The system compiles to no-ops when `PMIC_DEBUG` is not defined, ensuring production builds have no debug code.

### Features

- **5 Log Levels**: ERROR, WARN, INFO, DEBUG, TRACE
- **Module-Based Filtering**: Control logging per module (PLATFORM, TEST, WDG, IRQ, etc.)
- **Runtime Control**: Configure via environment variables
- **Zero Overhead**: Compiles to no-ops when `PMIC_DEBUG` undefined
- **Timestamped Output**: Microsecond-precision timestamps for performance analysis

### Log Levels

| Level | Macro | Use Case |
|-------|-------|----------|
| ERROR | `DEBUG_ERROR()` | Critical failures requiring immediate attention |
| WARN  | `DEBUG_WARN()` | Potential issues that don't stop execution |
| INFO  | `DEBUG_INFO()` | High-level execution flow |
| DEBUG | `DEBUG_DEBUG()` | Detailed debugging information |
| TRACE | `DEBUG_TRACE()` | Very detailed execution traces |

### Debug Modules

- `PLATFORM` - Platform abstraction layer (serial, hardware control)
- `TEST` - Test framework and test orchestration
- `WDG` - Watchdog module
- `IRQ` - Interrupt module
- `POWER` - Power module
- `IO` - I/O module
- `FSM` - Finite state machine module
- `ESM` - Error signaling module
- `CORE` - Core module
- `PMIC` - PMIC initialization

### Usage

#### Enable Debug Logging

```bash
# Compile with debug support
make clean
make test BUILD=host CFLAGS=-DPMIC_DEBUG

# Or use convenience target
make test-debug BUILD=host
```

#### Configure via Environment Variables

```bash
# Set log level (1=ERROR, 2=WARN, 3=INFO, 4=DEBUG, 5=TRACE)
export PMIC_DEBUG_LEVEL=4

# Enable specific modules (comma-separated)
export PMIC_DEBUG_MODULES="PLATFORM,TEST"

# Enable all modules
export PMIC_DEBUG_MODULES="ALL"

# Run tests
make test BUILD=host
```

#### Code Example

```c
#include "debug.h"

void myFunction(void) {
    DEBUG_INFO("Starting operation");
    DEBUG_DEBUG("Processing data: value=%d", someValue);

    if (error) {
        DEBUG_ERROR("Operation failed: code=%d", errorCode);
    }

    DEBUG_TRACE("Entering critical section");
}
```

#### Output Format

```
[000123.456789] [INFO] [PLATFORM] Initializing serial connection
[000123.789012] [DEBUG] [TEST] Running test: test_pos_irq_irqSetMask_enable
[000125.123456] [ERROR] [WDG] Watchdog timeout detected
```

### Zero-Overhead Implementation

When `PMIC_DEBUG` is not defined, all debug macros expand to empty statements:

```c
#ifndef PMIC_DEBUG
    #define DEBUG_ERROR(...)   do {} while(0)
    #define DEBUG_WARN(...)    do {} while(0)
    #define DEBUG_INFO(...)    do {} while(0)
    #define DEBUG_DEBUG(...)   do {} while(0)
    #define DEBUG_TRACE(...)   do {} while(0)
#endif
```

This ensures production builds have no debug code overhead.

### Best Practices

1. **Use Appropriate Levels**
   - ERROR: Actual failures requiring investigation
   - WARN: Unusual but handled conditions
   - INFO: High-level flow for understanding execution
   - DEBUG: Detailed internal state for debugging
   - TRACE: Very verbose execution traces

2. **Module Isolation**
   - Keep module logging focused on that module's concerns
   - Avoid cross-module logging dependencies

3. **Performance Considerations**
   - Debug builds may be slower due to I/O overhead
   - Use TRACE level sparingly (very verbose)
   - Disable debug logging for performance benchmarks

4. **Combine with Test Filtering**
   ```bash
   # Debug only IRQ tests
   export PMIC_TEST_MODULES="irq"
   export PMIC_DEBUG_LEVEL=5
   export PMIC_DEBUG_MODULES="IRQ,PLATFORM"
   make test-debug BUILD=host
   ```

---

## 2. Test Filtering System

### Overview

The test filtering system provides environment-variable-based test execution control without requiring recompilation. This allows selective test execution for debugging specific test failures.

### Features

- **Module Filtering**: Run specific test modules (e.g., only IRQ tests)
- **Pattern Filtering**: Run tests matching wildcard patterns (e.g., all tests with "mask" in the name)
- **Group Filtering**: Run only positive or negative tests
- **Combined Filtering**: Apply multiple filters simultaneously (AND logic)
- **Zero Overhead**: Minimal performance impact when no filters are active
- **Backward Compatible**: No filters = run all tests (existing behavior)

### Environment Variables

#### PMIC_TEST_MODULES
Comma-separated list of test modules to run.

**Available modules:**
- `common` - Common module tests
- `pmic` - PMIC initialization tests
- `core` - Core module tests
- `io` - I/O module tests
- `fsm` - FSM module tests
- `wdg` - Watchdog tests
- `esm` - ESM tests
- `irq` - IRQ tests
- `power` - Power tests

**Examples:**
```bash
export PMIC_TEST_MODULES="irq"              # Run only IRQ tests
export PMIC_TEST_MODULES="irq,power"        # Run IRQ and POWER tests
export PMIC_TEST_MODULES="common,core,io"  # Run multiple modules
```

#### PMIC_TEST_FILTER
Wildcard pattern for test name matching. Supports:
- `*` - Matches zero or more characters
- `?` - Matches exactly one character

**Examples:**
```bash
export PMIC_TEST_FILTER="test_pos_*"                      # All positive tests
export PMIC_TEST_FILTER="test_neg_*"                      # All negative tests
export PMIC_TEST_FILTER="*mask*"                          # Tests with "mask" in name
export PMIC_TEST_FILTER="test_pos_irq_irqSetMask_*"      # Specific test group
export PMIC_TEST_FILTER="*invalidHwState"                # Tests ending with this
```

#### PMIC_TEST_GROUPS
Filter tests by positive/negative classification.

**Valid values:**
- `all` - Run both positive and negative tests (default)
- `positive` or `pos` - Run only positive tests (test_pos_*)
- `negative` or `neg` - Run only negative tests (test_neg_*)

**Examples:**
```bash
export PMIC_TEST_GROUPS="positive"   # Only positive tests
export PMIC_TEST_GROUPS="negative"   # Only negative tests
export PMIC_TEST_GROUPS="all"        # All tests (default)
```

### Quick Start Examples

#### Debug a Single Test
```bash
export PMIC_TEST_FILTER="test_neg_power_pwrGetResourceCfg_ldoLs1Vmon1_invalidHwState"
make test BUILD=host
```

#### Run All IRQ Tests
```bash
export PMIC_TEST_MODULES="irq"
make test BUILD=host
```

#### Run All Negative Tests
```bash
export PMIC_TEST_GROUPS="negative"
make test BUILD=mock
```

#### Run Power Mask Tests Only
```bash
export PMIC_TEST_MODULES="power"
export PMIC_TEST_FILTER="*mask*"
make test BUILD=host
```

#### Debug IRQ Negative Tests with Pattern
```bash
export PMIC_TEST_MODULES="irq"
export PMIC_TEST_GROUPS="negative"
export PMIC_TEST_FILTER="*mask*"
make test BUILD=host
# Runs only negative IRQ tests with "mask" in the name
```

### Makefile Convenience Targets

#### test-module
Run tests for a specific module.

```bash
make test-module MODULE=irq BUILD=host
make test-module MODULE=power BUILD=mock
```

#### test-single
Run specific test(s) by name pattern.

```bash
make test-single TEST='test_pos_irq_irqSetMask_enable' BUILD=host
make test-single TEST='*mask*' BUILD=mock
```

#### test-group
Run only positive or negative tests.

```bash
make test-group GROUP=positive BUILD=host
make test-group GROUP=negative BUILD=mock
```

### Combining Filters

Filters are AND-ed together - all must pass for a test to run.

**Example: Complex Filter**
```bash
export PMIC_TEST_MODULES="irq,power"     # Only IRQ and POWER modules
export PMIC_TEST_FILTER="*mask*"         # Only tests with "mask" in name
export PMIC_TEST_GROUPS="negative"       # Only negative tests
make test BUILD=host
# Result: Runs only negative mask-related tests from IRQ and POWER modules
```

**Filter Logic:**
```
Test will run IF:
  (Module filter passes OR no module filter) AND
  (Name filter passes OR no name filter) AND
  (Group filter passes OR no group filter)
```

### Clear Filters / Run All Tests

To return to default behavior (run all tests):

```bash
unset PMIC_TEST_MODULES
unset PMIC_TEST_FILTER
unset PMIC_TEST_GROUPS
make test BUILD=host
```

### Performance

- Filter checks are simple string comparisons
- Negligible overhead when filters are active
- Skipped tests are not initialized or torn down
- Module filtering skips entire module setup/teardown

### Use Cases

#### Debugging a Failing Test
```bash
# Test failed in CI: test_neg_power_pwrGetResourceCfg_ldoLs1Vmon1_invalidHwState
export PMIC_TEST_FILTER="test_neg_power_pwrGetResourceCfg_ldoLs1Vmon1_invalidHwState"
make test BUILD=host
# Runs only this test for rapid iteration
```

#### Developing New IRQ Feature
```bash
# Work on IRQ module only
export PMIC_TEST_MODULES="irq"
make test BUILD=host
# Runs only IRQ tests (~50 tests instead of 773)
```

#### Verifying Negative Test Coverage
```bash
# Run all negative tests
export PMIC_TEST_GROUPS="negative"
make test BUILD=mock
# Ensures negative tests are comprehensive
```

#### Pre-commit Quick Check
```bash
# Modified only power module - test just power
export PMIC_TEST_MODULES="power"
make test BUILD=mock
# Quick verification before commit
```

#### Hardware Test Subset
```bash
# Hardware tests are slow - run subset first
export PMIC_TEST_MODULES="irq,power"
export PMIC_TEST_GROUPS="positive"
make test BUILD=host
# Verify positive cases before running full suite
```

### Output Format

When filters are active, the test runner displays the configuration:

```
=== Test Filter Configuration ===
Modules: irq, power (2 modules)
Filter: *mask* (pattern matching)
Groups: negative tests only
=================================
```

When no filters are active:

```
=== Test Filter Configuration ===
No filters active - running all tests
=================================
```

### Best Practices

1. **Start Narrow, Then Expand**
   - Debug with single test filter first
   - Gradually expand to module or group level
   - Run full suite before committing

2. **Use Makefile Shortcuts**
   - `make test-module MODULE=irq BUILD=host` is easier than setting env vars
   - Use shortcuts for common workflows

3. **Document Filter Usage in Bug Reports**
   ```bash
   # To reproduce issue XYZ:
   export PMIC_TEST_FILTER="test_neg_power_pwrGetResourceCfg_*"
   make test BUILD=host
   ```

4. **Clear Filters Between Sessions**
   - Filters persist in shell session
   - Use `unset` or start new shell for clean slate

5. **Combine with Debug Logging**
   ```bash
   export PMIC_TEST_MODULES="irq"
   export PMIC_DEBUG_LEVEL=5
   export PMIC_DEBUG_MODULES="ALL"
   make test-debug BUILD=host
   # Detailed logging for IRQ tests only
   ```

### Troubleshooting

#### Filter Not Working?
- Check filter syntax (case-sensitive)
- Verify module names match exactly (lowercase: "irq" not "IRQ")
- Test patterns with simple wildcards first (`*test_name*`)
- Check that filters are exported: `echo $PMIC_TEST_MODULES`

#### Running Too Many Tests?
- Filters are AND-ed - make sure all filters are set correctly
- Use `test-module` shortcut for clearest module filtering
- Check Unity output for actual test count

#### No Tests Running?
- Filters too restrictive - no tests match
- Try removing filters one at a time to identify issue
- Use `*` pattern to see all test names first

---

## 3. Performance Timing System

### Overview

The performance timing system provides hierarchical timing instrumentation for tracking test execution performance. It measures timing at three levels: individual tests, module groups, and the entire suite.

### Features

- **Per-Test Timing**: Displays execution time after each test
- **Per-Module Timing**: Aggregates timing for each test module
- **Suite-Level Timing**: Total execution time across all tests
- **Minimal Overhead**: Lightweight timing that doesn't significantly impact performance
- **Automatic Integration**: Integrates with Unity test framework

### Timing Hierarchy

```
Suite Level:        === TOTAL TEST SUITE: 12345.67 ms ===
  │
  ├─ Module Level:  [IRQ] Module total: 1234.56 ms
  │   ├─ Test:      test_pos_irq_irqSetMask_enable (12.34 ms)
  │   ├─ Test:      test_pos_irq_irqClearMask_enable (11.23 ms)
  │   └─ Test:      test_neg_irq_irqSetMask_invalid (10.12 ms)
  │
  └─ Module Level:  [POWER] Module total: 2345.67 ms
      ├─ Test:      test_pos_power_pwrSetResourceCfg_enable (23.45 ms)
      └─ Test:      test_neg_power_pwrGetResourceCfg_invalid (21.34 ms)
```

### Usage

#### Basic Usage

The timing system is automatically integrated into the test framework. No code changes are required for basic per-test timing.

**Output example:**
```
test_pos_irq_irqSetMask_enable (12.34 ms)
test_pos_irq_irqClearMask_enable (11.23 ms)
test_neg_irq_irqSetMask_invalid (10.12 ms)
```

#### Module-Level Timing

To enable module-level timing aggregation, add timing calls in your test module setup:

```c
#include "test_timer.h"

void setUp(void) {
    testTimer_startModule("IRQ");
    // ... module setup code
}

void tearDown(void) {
    // ... module teardown code
    testTimer_endModule("IRQ");
}
```

**Output example:**
```
[IRQ] Module total: 1234.56 ms (45 tests)
```

#### Suite-Level Timing

Suite-level timing is automatically displayed at the end of test execution:

```
=== TOTAL TEST SUITE: 12345.67 ms ===
```

### API Reference

#### testTimer_startModule(const char* moduleName)
Start timing for a test module. Call this in module setUp() before running tests.

**Parameters:**
- `moduleName` - Name of the module (e.g., "IRQ", "POWER")

#### testTimer_endModule(const char* moduleName)
End timing for a test module and display the total. Call this in module tearDown() after all tests complete.

**Parameters:**
- `moduleName` - Name of the module (must match startModule call)

#### testTimer_startTest(void)
Start timing for an individual test. Automatically called by the test framework.

#### testTimer_endTest(const char* testName)
End timing for an individual test and display the result. Automatically called by the test framework.

**Parameters:**
- `testName` - Name of the test

### Integration with Test Framework

The timing system integrates with the Unity test framework through the `PLATFORM_RUN_TEST` macro:

```c
#define PLATFORM_RUN_TEST(func) \
    do { \
        if (testFilter_shouldRunTest(#func)) { \
            testTimer_startTest(); \
            UnityDefaultTestRun(func, #func, __LINE__); \
            testTimer_endTest(#func); \
        } \
    } while(0)
```

This ensures every test is automatically timed without requiring changes to individual test files.

### Performance Analysis

#### Identify Slow Tests
```bash
make test BUILD=host 2>&1 | grep -E '\([0-9]+\.[0-9]+ ms\)' | sort -t'(' -k2 -n -r | head -20
# Shows 20 slowest tests
```

#### Compare Module Performance
```bash
make test BUILD=host 2>&1 | grep 'Module total'
# Shows timing for each module
```

#### Track Performance Regression
```bash
# Run tests and save timing
make test BUILD=host 2>&1 | tee baseline_timing.txt

# After code changes
make test BUILD=host 2>&1 | tee current_timing.txt

# Compare suite totals
grep "TOTAL TEST SUITE" baseline_timing.txt current_timing.txt
```

### Best Practices

1. **Module Timing**
   - Always call startModule/endModule in paired setUp/tearDown functions
   - Use consistent module names across test files

2. **Performance Monitoring**
   - Save timing output for performance regression detection
   - Investigate tests that take significantly longer than others
   - Consider mocking slow hardware operations in performance-critical paths

3. **Filtering for Performance**
   - Use test filtering to isolate slow modules for optimization
   - Profile individual tests with detailed timing

4. **Hardware vs Mock Builds**
   - Hardware builds (BUILD=host) will be much slower due to serial I/O
   - Mock builds (BUILD=mock) show pure test logic performance

---

## 4. Code Coverage Analysis

### Overview

Code coverage analysis measures how much of the PMIC source code is exercised during test execution. The test infrastructure supports coverage analysis for both BUILD=mock and BUILD=host configurations using LLVM's instrumentation-based coverage tools.

### Build Configurations

#### BUILD=mock Coverage (Default)
- **What it measures**: Full coverage of test code, PMIC source code, and mock library
- **Instrumentation**: All code compiled with coverage flags runs on the PC
- **Use cases**:
  - Verifying test suite completeness
  - Identifying untested code paths
  - Measuring branch/condition coverage
- **Command**: `make coverage` or `make coverage BUILD=mock`

#### BUILD=host Coverage
- **What it measures**: PC-side code only (test harness, serial communication, platform glue)
- **Limitations**: Microcontroller firmware cannot be instrumented from PC (PMIC code runs on target hardware)
- **Use cases**:
  - Verifying test infrastructure coverage
  - Analyzing serial communication paths
  - Debugging host-side test logic
- **Command**: `make coverage BUILD=host`
- **Requirements**: pmic-tiva-host firmware flashed to TM4C123 hardware

### Usage

```bash
# Mock coverage (default) - full PMIC source coverage
make coverage

# Explicit mock coverage
make coverage BUILD=mock

# Host coverage - PC-side code only
make coverage BUILD=host
```

### Output

Coverage reports are generated in multiple formats:
- **Text report**: Console output with per-file and per-function coverage percentages
- **JSON report**: Machine-readable format for CI integration
- **HTML report** (optional): Detailed line-by-line coverage visualization

### Best Practices

1. **Use BUILD=mock for PMIC coverage analysis** - This provides the most accurate measurement of test suite completeness
2. **BUILD=host coverage is for infrastructure debugging** - Don't expect high PMIC source coverage here
3. **Set coverage targets** - Aim for >90% line coverage, >80% branch coverage for production code
4. **Combine with test filtering** - Coverage analysis respects test filtering environment variables

---

## Integration Guide for Device Makefiles

To use this shared test infrastructure in a device test harness:

### 1. Add Common Test Directory Variable

```makefile
# Shared test infrastructure
TEST_COMMON_DIR = ../../../common/test
```

### 2. Update Include Path

```makefile
CFLAGS += -I$(TEST_COMMON_DIR)
```

### 3. Add Infrastructure Sources

```makefile
TEST_SOURCES = test_runner.c \
               $(TEST_COMMON_DIR)/test_filter.c \
               $(TEST_COMMON_DIR)/test_timer.c \
               $(wildcard */$(MODULE)_test.c)

# Debug infrastructure (only when PMIC_DEBUG defined)
ifdef PMIC_DEBUG
    TEST_SOURCES += $(TEST_COMMON_DIR)/debug.c
endif
```

### 4. Add Debug Build Target (Optional)

```makefile
.PHONY: test-debug
test-debug:
	$(MAKE) test BUILD=$(BUILD) CFLAGS="$(CFLAGS) -DPMIC_DEBUG"
```

### 5. Update Clean Target

```makefile
clean:
	rm -f $(TEST_COMMON_DIR)/*.o
	rm -f $(TEST_COMMON_DIR)/*.d
	# ... other clean rules
```

### Example Device Makefile Structure

```makefile
# Device: LP8772x-Q1
# Test Makefile

# Shared test infrastructure
TEST_COMMON_DIR = ../../../common/test

# Compiler flags
CFLAGS += -I$(TEST_COMMON_DIR)
CFLAGS += -Wall -Wextra -std=c11

# Test sources
TEST_SOURCES = test_runner.c \
               $(TEST_COMMON_DIR)/test_filter.c \
               $(TEST_COMMON_DIR)/test_timer.c \
               $(wildcard */$(MODULE)_test.c)

ifdef PMIC_DEBUG
    TEST_SOURCES += $(TEST_COMMON_DIR)/debug.c
endif

# Build and run tests
.PHONY: test
test:
	$(CC) $(CFLAGS) $(TEST_SOURCES) -o test_runner
	./test_runner

# Debug build with logging
.PHONY: test-debug
test-debug:
	$(MAKE) test BUILD=$(BUILD) CFLAGS="$(CFLAGS) -DPMIC_DEBUG"

# Clean
clean:
	rm -f test_runner *.o *.d
	rm -f $(TEST_COMMON_DIR)/*.o $(TEST_COMMON_DIR)/*.d
```

---

## File Reference

| File | Purpose | Lines |
|------|---------|-------|
| `debug.h` | Debug logging API and macros | ~220 |
| `debug.c` | Debug logging implementation | ~220 |
| `test_filter.h` | Test filtering API | ~280 |
| `test_filter.c` | Test filtering implementation | ~430 |
| `test_timer.h` | Performance timing API | ~110 |
| `test_timer.c` | Performance timing implementation | ~200 |
| `README.md` | This documentation | ~750 |

---

## Support

For questions or issues with the test infrastructure:

1. Check this README for usage examples
2. Review the header files for API documentation
3. Examine existing device test Makefiles for integration examples
4. Contact the test infrastructure maintainer

---

## Version History

- **v1.0.0** - Initial shared infrastructure release
  - Debug logging system
  - Test filtering system
  - Performance timing system
  - Comprehensive documentation
