# Makefile for LP8772x-Q1 PMIC Tests
# Supports hardware and mock builds
#
# Usage:
#   make test               # Build and run with mock backend (default)
#   make test BUILD=mock    # Build and run with mock backend
#   make test BUILD=host # Build and run with hardware (requires TI drivers)
#   make clean              # Clean build artifacts

# Build type (mock or hardware)
BUILD ?= mock

# Compiler default: clang on all platforms
ifeq ($(origin CC),default)
    CC := clang
endif

# On Windows, target MinGW ABI so clang finds headers via the GCC installation
# rather than looking for MSVC/Windows SDK headers.
ifeq ($(OS),Windows_NT)
    TARGET_FLAG := --target=x86_64-w64-mingw32
else
    TARGET_FLAG :=
endif

# Warning flags: -Weverything is clang-only; use -Wall -Wextra for gcc
ifeq ($(CC),gcc)
    WARN_FLAGS := -Wall -Wextra
else
    WARN_FLAGS := -Weverything
endif

# Coverage flags
COV_FLAGS = -fprofile-instr-generate -fcoverage-mapping -fcoverage-mcdc

# Coverage tools (OS-aware)
ifeq ($(OS),Windows_NT)
    LLVM_PROFDATA = llvm-profdata
    LLVM_COV = llvm-cov
else ifeq ($(shell uname -s),Darwin)
    LLVM_PROFDATA = /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/llvm-profdata
    LLVM_COV = /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/llvm-cov
else
    LLVM_PROFDATA = llvm-profdata
    LLVM_COV = llvm-cov
endif

# Unity source file
UNITY_SRC = $(UNITY_DIR)/src/unity.c

# Scripts directory
SCRIPTS_DIR = ../../../scripts

# Coverage tools from pmic-lld-utils (inherit from root or detect)
ifndef PMIC_COVERAGE
    PMIC_UTILS_DIR ?= ../../../../pmic-lld-utils
    PMIC_COVERAGE := $(shell which pmic-coverage 2>/dev/null)
    ifndef PMIC_COVERAGE
        PMIC_COVERAGE = uv run --project $(PMIC_UTILS_DIR)/coverage pmic-coverage
    endif
endif

# Directories
SRC_DIR = ../src
INC_DIR = ../include

# Unity directory - uses external Unity test framework
# Auto-detect from sibling directory (portable) or require explicit path
ifndef UNITY_DIR
    ifeq ($(wildcard ../../../../unity/src/unity.c),../../../../unity/src/unity.c)
        UNITY_DIR := ../../../../unity
    else
        $(error UNITY_DIR not found. Set UNITY_DIR environment variable or install Unity as sibling to pmic-lld repository)
    endif
else
    export UNITY_DIR
endif

# Mock directory - uses external pmic-lld-mocking repository
# Auto-detect from sibling directory (portable) or require explicit path
ifndef PMIC_MOCK_DIR
    ifeq ($(wildcard ../../../../pmic-lld-mocking/Makefile),../../../../pmic-lld-mocking/Makefile)
        PMIC_MOCK_DIR := ../../../../pmic-lld-mocking
    else
        $(error PMIC_MOCK_DIR not found. Set PMIC_MOCK_DIR environment variable or install pmic-lld-mocking as sibling to pmic-lld repository)
    endif
endif
# Normalize path: strip trailing slashes to avoid double-slash issues
# Note: PMIC_MOCK_DIR is converted to absolute path by root Makefile if needed
MOCK_DIR = $(patsubst %/,%,$(PMIC_MOCK_DIR))

# Shared test infrastructure
TEST_COMMON_DIR = ../../../common/test

# Auto-discover test subdirectories (any directory with .h files)
TEST_INCLUDE_DIRS = $(sort $(dir $(wildcard */*.h)))
TEST_INCLUDE_FLAGS = $(addprefix -I,$(TEST_INCLUDE_DIRS))

# Compiler flags (common)
CFLAGS = $(WARN_FLAGS) \
         -Wno-documentation \
         -Wno-padded \
         -Wno-poison-system-directories \
         -Wno-unused-parameter \
         -Wno-unused-function \
         -Wno-missing-prototypes \
         -Wno-declaration-after-statement \
         -Wno-unused-macros \
         -Wno-unused-variable \
         -Wno-shadow \
         -Wno-implicit-int-conversion \
         -Wno-pedantic \
         -Wno-missing-noreturn \
         -Wno-unreachable-code \
         -Wno-unreachable-code-return \
         -Wno-unreachable-code-break \
         -Wno-covered-switch-default \
         -Wno-missing-variable-declarations \
         -Wno-unsafe-buffer-usage \
		 -Wno-extra-semi-stmt \
         -Werror \
         -std=c99 $(TARGET_FLAG) -I$(INC_DIR) -I. -I$(TEST_COMMON_DIR) $(TEST_INCLUDE_FLAGS)

# PMIC driver source files (always needed)
PMIC_SOURCES = \
	$(SRC_DIR)/pmic.c \
	$(SRC_DIR)/pmic_common.c \
	$(SRC_DIR)/pmic_core.c \
	$(SRC_DIR)/pmic_io.c \
	$(SRC_DIR)/pmic_fsm.c \
	$(SRC_DIR)/pmic_irq.c \
	$(SRC_DIR)/pmic_power.c \
	$(SRC_DIR)/pmic_wdg.c \
	$(SRC_DIR)/pmic_esm.c

# Test source files
TEST_SOURCES = \
	test_runner.c \
	$(TEST_COMMON_DIR)/test_filter.c \
	$(TEST_COMMON_DIR)/test_timer.c \
	common/common_test.c \
	core/core_test.c \
	fsm/fsm_test.c \
	io/io_test.c \
	irq/irq_test.c \
	pmic/pmic_test.c \
	power/power_test.c \
	wdg/wdg_test.c \
	esm/esm_test.c

# Debug infrastructure (only when PMIC_DEBUG defined)
ifdef PMIC_DEBUG
    TEST_SOURCES += $(TEST_COMMON_DIR)/debug.c
endif

# Build-specific configuration
ifeq ($(BUILD), mock)
	# Mock build configuration - uses external pmic-lld-mocking repository
	CFLAGS += -I$(MOCK_DIR)/mock
	CFLAGS += -I$(MOCK_DIR)/adapters
	CFLAGS += -I$(MOCK_DIR)/utils
	CFLAGS += -I$(MOCK_DIR)/platforms/lp8772x
	CFLAGS += -I$(UNITY_DIR)/src
	CFLAGS += -DBUILD_MOCK -DUNITY_INCLUDE_DOUBLE

	# Platform glue compiled locally (from pmic-lld-mocking/platforms/lp8772x/)
	PLATFORM_SOURCES = $(MOCK_DIR)/platforms/lp8772x/platform_mock.c

	LDFLAGS = -L$(MOCK_DIR)/build/lib -lpmic_mock

	TARGET = test_runner

	PLATFORM_NAME = Mock Backend

else ifeq ($(BUILD), host)
	# Host-controlled build - tests run on PC, communicate with pmic-tiva-host firmware
	CFLAGS += -I$(UNITY_DIR)/src
	CFLAGS += -DBUILD_HOST -DUNITY_INCLUDE_DOUBLE

	# Platform sources - serial-based implementation
	PLATFORM_SOURCES = platform.c platform_serial.c

	# No special linking required for host build
	LDFLAGS =

	TARGET = test_runner

	PLATFORM_NAME = Host-Controlled (via pmic-tiva-host firmware)

else
	$(error Invalid BUILD type. Use: BUILD=mock or BUILD=host)
endif

# clock_gettime is in winpthread on MinGW; harmless on Linux
ifeq ($(OS),Windows_NT)
    LDFLAGS += -lpthread
endif

# Mock library path and sources (for dependency tracking)
MOCK_LIB = $(MOCK_DIR)/build/lib/libpmic_mock.a
MOCK_LIB_SRCS = $(wildcard $(MOCK_DIR)/mock/*.c) \
                $(wildcard $(MOCK_DIR)/adapters/*.c) \
                $(wildcard $(MOCK_DIR)/utils/*.c) \
                $(wildcard $(MOCK_DIR)/behaviors/*.c) \
                $(wildcard $(MOCK_DIR)/devices/*/*.c)

# Auto-build mock library if needed
.PHONY: check-mock-lib
check-mock-lib:
	@if [ ! -f "$(MOCK_LIB)" ]; then \
		echo "Mock library not found, building it..."; \
		"$(MAKE)" -C $(MOCK_DIR) CC=$(CC) || exit 1; \
	elif [ -n "$$(find $(MOCK_DIR) -name '*.c' -newer $(MOCK_LIB) 2>/dev/null | head -1)" ]; then \
		echo "Mock library out of date, rebuilding..."; \
		"$(MAKE)" -C $(MOCK_DIR) CC=$(CC) || exit 1; \
	else \
		echo "Mock library up to date"; \
	fi

# Default target
all: $(TARGET)

# Build test executable
ifeq ($(BUILD), mock)
$(TARGET): check-mock-lib $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES)
	@echo "Building $(TARGET) for $(PLATFORM_NAME)..."
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 $(TARGET_FLAG) -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources..."
	@$(CC) $(CFLAGS) $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o $@ $(LDFLAGS)
	@echo "Build complete: $@"
else ifeq ($(BUILD), host)
$(TARGET): $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES)
	@echo "Building $(TARGET) for $(PLATFORM_NAME)..."
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 $(TARGET_FLAG) -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources..."
	@$(CC) $(CFLAGS) -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o $@ $(LDFLAGS)
	@echo "Build complete: $@"
	@echo ""
	@echo "IMPORTANT: Before running tests, ensure pmic-tiva-host firmware is flashed:"
	@echo "  cd /path/to/pmic-tiva-host && make && lm4flash ppc_host.bin"
	@echo ""
	@echo "Set serial port (optional):"
	@echo "  export PMIC_SERIAL_PORT=/dev/cu.usbmodem1131201  # defaults to /dev/cu.usbmodem1131201"
endif

# Run tests (always rebuild to catch compilation issues)
.PHONY: test
ifeq ($(BUILD), mock)
test: check-mock-lib
	@echo "========================================"
	@echo "Building and running tests on $(PLATFORM_NAME)"
	@echo "========================================"
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 $(TARGET_FLAG) -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources..."
	@$(CC) $(CFLAGS) -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o $(TARGET) $(LDFLAGS)
	@echo "Running tests..."
	./$(TARGET)
else ifeq ($(BUILD), host)
test:
	@echo "========================================"
	@echo "Building and running tests on $(PLATFORM_NAME)"
	@echo "========================================"
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 $(TARGET_FLAG) -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources..."
	@$(CC) $(CFLAGS) -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o $(TARGET) $(LDFLAGS)
	@echo "Running tests (requires pmic-tiva-host firmware on TM4C123)..."
	./$(TARGET)
endif

# Run tests with debug logging enabled
# Usage:
#   export PMIC_DEBUG_LEVEL=5 && export PMIC_DEBUG_MODULES=ALL && make test-debug BUILD=host
#   export PMIC_DEBUG_LEVEL=3 && export PMIC_DEBUG_MODULES=PLATFORM && make test-debug BUILD=mock
#
# Debug levels: 0=NONE, 1=ERROR, 2=WARNING, 3=INFO, 4=DEBUG, 5=TRACE
# Debug modules: ALL, PLATFORM, TEST, LLD, SERIAL (comma-separated)
#
# Adds ~2KB to binary size, minimal runtime overhead when disabled
.PHONY: test-debug
ifeq ($(BUILD), mock)
test-debug: check-mock-lib
	@echo "========================================"
	@echo "Building and running tests with DEBUG on $(PLATFORM_NAME)"
	@echo "========================================"
	@echo "Debug level: $${PMIC_DEBUG_LEVEL:-not set (no output)}"
	@echo "Debug modules: $${PMIC_DEBUG_MODULES:-not set (no output)}"
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 $(TARGET_FLAG) -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources with debug support..."
	@$(CC) $(CFLAGS) -DPMIC_DEBUG=1 -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) debug.c $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o $(TARGET) $(LDFLAGS)
	@echo "Running tests with debug logging..."
	./$(TARGET)
else ifeq ($(BUILD), host)
test-debug:
	@echo "========================================"
	@echo "Building and running tests with DEBUG on $(PLATFORM_NAME)"
	@echo "========================================"
	@echo "Debug level: $${PMIC_DEBUG_LEVEL:-not set (no output)}"
	@echo "Debug modules: $${PMIC_DEBUG_MODULES:-not set (no output)}"
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 $(TARGET_FLAG) -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources with debug support..."
	@$(CC) $(CFLAGS) -DPMIC_DEBUG=1 -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) debug.c $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o $(TARGET) $(LDFLAGS)
	@echo "Running tests with debug logging (requires pmic-tiva-host firmware on TM4C123)..."
	./$(TARGET)
endif

# Code coverage target (uses clang coverage instrumentation)
# Usage:
#   make coverage            # Run coverage with BUILD=mock (default)
#   make coverage BUILD=mock # Explicit mock coverage
#   make coverage BUILD=host # Host coverage (PC-side code only, requires hardware)
.PHONY: coverage
ifeq ($(BUILD), mock)
coverage: check-mock-lib
else
coverage:
endif
	@echo "Running coverage analysis (BUILD=$(BUILD))..."
	@$(MAKE) clean
	@echo "Building with LLVM coverage instrumentation..."
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 $(TARGET_FLAG) -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE $(COV_FLAGS) -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources (strict warnings)..."
	@$(CC) $(CFLAGS) $(COV_FLAGS) -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o test_runner $(LDFLAGS)
	@echo "Removing previous coverage data..."
	@rm -f *.profraw *.profdata *.gcov
	@echo "Running tests..."
	@LLVM_PROFILE_FILE=default.profraw ./test_runner || echo "WARNING: Some tests failed, but continuing with coverage generation..."
	@echo "Merging coverage profile data..."
	@$(LLVM_PROFDATA) merge -sparse default.profraw -o test_runner.profdata
	@echo "Generating coverage reports (text and JSON)..."
	@$(PMIC_COVERAGE) generate test_runner test_runner.profdata $(PMIC_SOURCES)

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	@rm -f test_runner test_runner.exe test_hardware.elf test_hardware.bin test_hardware_demo *.o unity.o startup.o
	@rm -f *.profraw *.profdata *.gcov
	@rm -rf coverage
	@echo "Clean complete"

# Convenience targets for test filtering
# These targets provide shortcuts for common filtering scenarios

# Run tests for a specific module
# Usage: make test-module MODULE=irq BUILD=host
.PHONY: test-module
test-module:
ifndef MODULE
	@echo "ERROR: MODULE not specified"
	@echo "Usage: make test-module MODULE=<module> BUILD=<build>"
	@echo "Available modules: common, pmic, core, io, fsm, wdg, esm, irq, power"
	@exit 1
endif
	@export PMIC_TEST_MODULES=$(MODULE) && $(MAKE) test BUILD=$(BUILD)

# Run a specific test by name pattern
# Usage: make test-single TEST=test_pos_irq_irqSetMask_enable BUILD=host
.PHONY: test-single
test-single:
ifndef TEST
	@echo "ERROR: TEST not specified"
	@echo "Usage: make test-single TEST=<pattern> BUILD=<build>"
	@echo "Example: make test-single TEST='test_pos_irq_*' BUILD=host"
	@exit 1
endif
	@export PMIC_TEST_FILTER=$(TEST) && $(MAKE) test BUILD=$(BUILD)

# Run only positive or negative tests
# Usage: make test-group GROUP=positive BUILD=host
.PHONY: test-group
test-group:
ifndef GROUP
	@echo "ERROR: GROUP not specified"
	@echo "Usage: make test-group GROUP=<positive|negative> BUILD=<build>"
	@exit 1
endif
	@export PMIC_TEST_GROUPS=$(GROUP) && $(MAKE) test BUILD=$(BUILD)

# Help target
help:
	@echo "LP8772x-Q1 PMIC Test Makefile"
	@echo ""
	@echo "Usage:"
	@echo "  make [TARGET] [BUILD=mock|host]"
	@echo ""
	@echo "Targets:"
	@echo "  all              - Build test executable (default)"
	@echo "  test             - Build and run tests"
	@echo "  test-module      - Run tests for specific module(s)"
	@echo "  test-single      - Run specific test by name pattern"
	@echo "  test-group       - Run positive or negative tests only"
	@echo "  coverage         - Run tests with coverage analysis (respects BUILD variable)"
	@echo "  clean            - Remove build artifacts"
	@echo "  help             - Show this help message"
	@echo ""
	@echo "Build Options:"
	@echo "  BUILD=mock       - Use mock backend (no hardware required) [default]"
	@echo "  BUILD=host       - Host-controlled (requires pmic-tiva-host firmware on TM4C123)"
	@echo ""
	@echo "Test Filtering (via environment variables):"
	@echo "  PMIC_TEST_MODULES  - Comma-separated module list (e.g., 'irq,power')"
	@echo "  PMIC_TEST_FILTER   - Wildcard pattern (e.g., '*mask*', 'test_pos_*')"
	@echo "  PMIC_TEST_GROUPS   - 'positive', 'negative', or 'all' (default)"
	@echo ""
	@echo "Coverage Analysis:"
	@echo "  make coverage                              # Mock coverage (default)"
	@echo "  make coverage BUILD=mock                   # Explicit mock coverage"
	@echo "  make coverage BUILD=host                   # Host coverage (PC-side only)"
	@echo ""
	@echo "Examples:"
	@echo "  make test                                  # Run all tests with mock"
	@echo "  make test BUILD=host                       # Run all tests with hardware"
	@echo "  make test-module MODULE=irq BUILD=host     # Run only IRQ tests"
	@echo "  make test-single TEST='*mask*' BUILD=host  # Run tests with 'mask' in name"
	@echo "  make test-group GROUP=positive BUILD=mock  # Run only positive tests"
	@echo ""
	@echo "  # Advanced filtering (combine environment variables):"
	@echo "  export PMIC_TEST_MODULES='irq,power'"
	@echo "  export PMIC_TEST_FILTER='*mask*'"
	@echo "  export PMIC_TEST_GROUPS='negative'"
	@echo "  make test BUILD=host                       # Run only negative mask tests in IRQ/POWER"
	@echo ""
	@echo "Host-Controlled Mode Setup:"
	@echo "  1. Flash pmic-tiva-host firmware to TM4C123"
	@echo "  2. Connect USB cable"
	@echo "  3. Set PMIC_SERIAL_PORT (optional, defaults to /dev/cu.usbmodem1131201)"
	@echo "  4. Run: make test BUILD=host"

.PHONY: all test test-module test-single test-group clean help coverage
