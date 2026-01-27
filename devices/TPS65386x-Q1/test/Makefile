# Makefile for TPS65386x-Q1 PMIC Tests
# Supports hardware and mock builds
#
# Usage:
#   make test               # Build and run with mock backend (default)
#   make test BUILD=mock    # Build and run with mock backend
#   make test BUILD=hardware # Build and run with hardware (requires TI drivers)
#   make clean              # Clean build artifacts

# Build type (mock or hardware)
BUILD ?= mock

# Compiler (standardized for coverage reliability)
CC = clang

# Coverage flags
COV_FLAGS = -fprofile-arcs -ftest-coverage

# Coverage tool (standardized)
GCOV = llvm-cov-14 gcov

# Unity source file
UNITY_SRC = $(UNITY_DIR)/src/unity.c

# Scripts directory
SCRIPTS_DIR = ../../../scripts

# Directories
SRC_DIR = ../src
INC_DIR = ../include

# Unity directory - uses external Unity test framework
ifndef UNITY_DIR
    $(error UNITY_DIR environment variable must be set. Install Unity from https://github.com/ThrowTheSwitch/Unity.git)
endif

# Mock directory - uses external pmic-lld-mocking repository
ifndef PMIC_MOCK_DIR
    $(error PMIC_MOCK_DIR environment variable must be set for mock builds. Set to path of pmic-lld-mocking repository)
endif
# Normalize path: strip trailing slashes to avoid double-slash issues
# Note: PMIC_MOCK_DIR is converted to absolute path by root Makefile if needed
MOCK_DIR = $(patsubst %/,%,$(PMIC_MOCK_DIR))

# Compiler flags (common)
CFLAGS = -Weverything \
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
         -Werror \
         -std=c99 -I. -I$(INC_DIR)

# LDRA instrumentation support - relax warnings for instrumented code
ifdef LDRA_INSTRUMENT
    CFLAGS := $(filter-out $(COV_FLAGS),$(CFLAGS))
    CFLAGS := $(filter-out -Werror,$(CFLAGS))
    CFLAGS += -O0 -g -Wno-unsafe-buffer-usage -Wno-missing-prototypes -Wno-implicit-int-conversion
    CFLAGS += -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U
endif

# PMIC driver source files (always needed)
# For LDRA instrumentation: Use inszt_*.c or ipmic_*.c files if they exist (created by Phase 21q)
# Otherwise use original *.c files
# Note: Jenkins uses inszt_* prefix, local dev may use ipmic_* prefix
PMIC_SOURCES = \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic.c),inszt_pmic.c,$(if $(wildcard $(SRC_DIR)/ipmic.c),ipmic.c,pmic.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_common.c),inszt_pmic_common.c,$(if $(wildcard $(SRC_DIR)/ipmic_common.c),ipmic_common.c,pmic_common.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_core.c),inszt_pmic_core.c,$(if $(wildcard $(SRC_DIR)/ipmic_core.c),ipmic_core.c,pmic_core.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_diag.c),inszt_pmic_diag.c,$(if $(wildcard $(SRC_DIR)/ipmic_diag.c),ipmic_diag.c,pmic_diag.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_io.c),inszt_pmic_io.c,$(if $(wildcard $(SRC_DIR)/ipmic_io.c),ipmic_io.c,pmic_io.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_gpio.c),inszt_pmic_gpio.c,$(if $(wildcard $(SRC_DIR)/ipmic_gpio.c),ipmic_gpio.c,pmic_gpio.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_wdg.c),inszt_pmic_wdg.c,$(if $(wildcard $(SRC_DIR)/ipmic_wdg.c),ipmic_wdg.c,pmic_wdg.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_esm.c),inszt_pmic_esm.c,$(if $(wildcard $(SRC_DIR)/ipmic_esm.c),ipmic_esm.c,pmic_esm.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_fsm.c),inszt_pmic_fsm.c,$(if $(wildcard $(SRC_DIR)/ipmic_fsm.c),ipmic_fsm.c,pmic_fsm.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_irq.c),inszt_pmic_irq.c,$(if $(wildcard $(SRC_DIR)/ipmic_irq.c),ipmic_irq.c,pmic_irq.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_power.c),inszt_pmic_power.c,$(if $(wildcard $(SRC_DIR)/ipmic_power.c),ipmic_power.c,pmic_power.c)) \
	$(SRC_DIR)/$(if $(wildcard $(SRC_DIR)/inszt_pmic_timer.c),inszt_pmic_timer.c,$(if $(wildcard $(SRC_DIR)/ipmic_timer.c),ipmic_timer.c,pmic_timer.c))

# Test source files
TEST_SOURCES = \
	test_runner.c \
	test_utils.c \
	common/common_test.c \
	core/core_test.c \
	wdg/wdg_test.c \
	gpio/gpio_test.c \
	io/io_test.c \
	irq/irq_test.c \
	pmic/pmic_test.c \
	power/power_test.c \
	esm/esm_test.c \
	fsm/fsm_test.c \
	timer/timer_test.c

# Build-specific configuration
ifeq ($(BUILD), mock)
	# Mock build configuration - uses external pmic-lld-mocking repository
	CFLAGS += -I$(MOCK_DIR)/mock
	CFLAGS += -I$(MOCK_DIR)/adapters
	CFLAGS += -I$(MOCK_DIR)/utils
	CFLAGS += -I$(MOCK_DIR)/platforms/tps65386x
	CFLAGS += -I$(UNITY_DIR)/src
	CFLAGS += -DBUILD_MOCK -DUNITY_INCLUDE_DOUBLE

	# Platform glue compiled locally (from pmic-lld-mocking/platforms/tps65386x/)
	PLATFORM_SOURCES = $(MOCK_DIR)/platforms/tps65386x/platform_mock.c

	LDFLAGS = -L$(MOCK_DIR)/build/lib -lpmic_mock

	TARGET = test_runner

	PLATFORM_NAME = Mock Backend
else
	# Hardware build configuration
	CFLAGS += -I/path/to/ti_drivers

	PLATFORM_SOURCES = platform_hardware.c

	LDFLAGS = -L/path/to/ti_drivers -lmcspi

	TARGET = test_hardware_demo

	PLATFORM_NAME = Hardware (TI Drivers)
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
		$(MAKE) -C $(MOCK_DIR) CC=$(CC) || exit 1; \
	elif [ -n "$$(find $(MOCK_DIR) -name '*.c' -newer $(MOCK_LIB) 2>/dev/null | head -1)" ]; then \
		echo "Mock library out of date, rebuilding..."; \
		$(MAKE) -C $(MOCK_DIR) CC=$(CC) || exit 1; \
	else \
		echo "Mock library up to date"; \
	fi

# Default target
all: $(TARGET)

# Build test executable
$(TARGET): check-mock-lib $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES)
	@echo "Building $(TARGET) for $(PLATFORM_NAME)..."
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources..."
	@$(CC) $(CFLAGS) $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o $@ $(LDFLAGS)
	@echo "Build complete: $@"

# Run tests (always rebuild to catch compilation issues)
.PHONY: test
test: check-mock-lib
	@echo "========================================"
	@echo "Building and running tests on $(PLATFORM_NAME)"
	@echo "========================================"
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources..."
	@$(CC) $(CFLAGS) -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o $(TARGET) $(LDFLAGS)
	@echo "Running tests..."
	./$(TARGET)

# Run tests with coverage (generates both text and JSON reports)
.PHONY: coverage
coverage: check-mock-lib
	@echo "Building with coverage instrumentation..."
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE $(COV_FLAGS) -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources (strict warnings)..."
	@$(CC) $(CFLAGS) $(COV_FLAGS) -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o test_runner $(LDFLAGS)
	@echo "Removing previous coverage data..."
	@rm -f *.gcda *.gcov
	@echo "Running tests..."
	@./test_runner || echo "WARNING: Some tests failed, but continuing with coverage generation..."
	@echo "Generating coverage reports (text and JSON)..."
	@python3 $(SCRIPTS_DIR)/generate_coverage_report.py $(PMIC_SOURCES)

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	@rm -f $(TARGET) test_runner test_mock_demo test_hardware_demo *.o unity.o
	@rm -f *.gcda *.gcno *.gcov
	@rm -rf coverage
	@echo "Clean complete"

# Help target
help:
	@echo "TPS65386x-Q1 PMIC Test Makefile"
	@echo ""
	@echo "Usage:"
	@echo "  make [TARGET] [BUILD=mock|hardware]"
	@echo ""
	@echo "Targets:"
	@echo "  all           - Build test executable (default)"
	@echo "  test          - Build and run tests"
	@echo "  coverage      - Run tests with coverage analysis (generates text and JSON)"
	@echo "  clean         - Remove build artifacts"
	@echo "  help          - Show this help message"
	@echo ""
	@echo "Build Options:"
	@echo "  BUILD=mock       - Use mock backend (no hardware required) [default]"
	@echo "  BUILD=hardware   - Use TI drivers (requires hardware)"
	@echo ""
	@echo "Examples:"
	@echo "  make test                    # Run tests with mock"
	@echo "  make test BUILD=mock         # Run tests with mock"
	@echo "  make test BUILD=hardware     # Run tests with hardware"
	@echo "  make clean                   # Clean build"

.PHONY: all test clean help coverage
