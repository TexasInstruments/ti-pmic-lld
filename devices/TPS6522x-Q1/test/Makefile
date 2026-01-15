# ******************************************************************************
# Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# ******************************************************************************

# TPS6522x-Q1 (Burton) Test Makefile
# Build system for PMIC LLD test suite

# ============================================================================
# Build Configuration
# ============================================================================

# Default build mode: mock or hardware
BUILD ?= mock

# Compiler (standardized for coverage reliability)
CC := clang

# Coverage flags
COV_FLAGS = -fprofile-arcs -ftest-coverage

# Coverage tool (standardized)
GCOV = llvm-cov-14 gcov

# Scripts directory
SCRIPTS_DIR = ../../../scripts

# Project paths
PMIC_ROOT = ..
TEST_ROOT = .

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

# ============================================================================
# Source Files
# ============================================================================

# Test module sources
TEST_SOURCES = \
	test_runner.c \
	test_utils.c \
	common/common_test.c \
	pmic/pmic_test.c \
	core/core_test.c \
	adc/adc_test.c \
	power/power_test.c \
	gpio/gpio_test.c \
	wdg/wdg_test.c \
	esm/esm_test.c \
	io/io_test.c \
	irq/irq_test.c \
	fsm/fsm_test.c

# Platform sources (conditional on BUILD mode)
ifeq ($(BUILD),mock)
	# Platform glue compiled locally (from pmic-lld-mocking/platforms/tps6522x/)
	PLATFORM_SOURCES = $(MOCK_DIR)/platforms/tps6522x/platform_mock.c
else
	PLATFORM_SOURCES = platform.c
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

# All test sources
SOURCES = $(TEST_SOURCES) $(PLATFORM_SOURCES)

# PMIC driver sources (for coverage builds only)
# For LDRA instrumentation: Use inszt_*.c or ipmic_*.c files if they exist (created by Phase 21q)
# Otherwise use original *.c files
# Note: Jenkins uses inszt_* prefix, local dev may use ipmic_* prefix
PMIC_SOURCES = \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic.c),inszt_pmic.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic.c),ipmic.c,pmic.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_common.c),inszt_pmic_common.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_common.c),ipmic_common.c,pmic_common.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_core.c),inszt_pmic_core.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_core.c),ipmic_core.c,pmic_core.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_io.c),inszt_pmic_io.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_io.c),ipmic_io.c,pmic_io.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_fsm.c),inszt_pmic_fsm.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_fsm.c),ipmic_fsm.c,pmic_fsm.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_irq.c),inszt_pmic_irq.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_irq.c),ipmic_irq.c,pmic_irq.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_power.c),inszt_pmic_power.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_power.c),ipmic_power.c,pmic_power.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_wdg.c),inszt_pmic_wdg.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_wdg.c),ipmic_wdg.c,pmic_wdg.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_esm.c),inszt_pmic_esm.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_esm.c),ipmic_esm.c,pmic_esm.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_gpio.c),inszt_pmic_gpio.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_gpio.c),ipmic_gpio.c,pmic_gpio.c)) \
	$(PMIC_ROOT)/src/$(if $(wildcard $(PMIC_ROOT)/src/inszt_pmic_adc.c),inszt_pmic_adc.c,$(if $(wildcard $(PMIC_ROOT)/src/ipmic_adc.c),ipmic_adc.c,pmic_adc.c))

# ============================================================================
# Include Paths
# ============================================================================

INCLUDES = \
	-I$(TEST_ROOT) \
	-I$(PMIC_ROOT)/include \
	-I$(UNITY_DIR)/src \
	-Ipmic -Icore -Iadc -Ipower -Igpio -Iwdg -Iesm -Iio -Iirq -Ifsm

# ============================================================================
# Compiler Flags
# ============================================================================

# Base flags
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
         -Wno-covered-switch-default \
         -Wno-missing-variable-declarations \
         -Wno-sign-conversion \
         -Wno-bad-function-cast \
         -Werror \
         -std=c99 $(INCLUDES)

# LDRA instrumentation support - relax warnings for instrumented code
ifdef LDRA_INSTRUMENT
    CFLAGS := $(filter-out $(COV_FLAGS),$(CFLAGS))
    CFLAGS := $(filter-out -Werror,$(CFLAGS))
    CFLAGS += -O0 -g -Wno-unsafe-buffer-usage -Wno-missing-prototypes -Wno-implicit-int-conversion
endif

# Build mode specific flags
ifeq ($(BUILD),mock)
	CFLAGS += -DBUILD_MOCK -DUNITY_INCLUDE_DOUBLE
	CFLAGS += -I$(MOCK_DIR)/mock
	CFLAGS += -I$(MOCK_DIR)/adapters
	CFLAGS += -I$(MOCK_DIR)/utils
	CFLAGS += -I$(MOCK_DIR)/platforms/tps6522x
	MOCK_LDFLAGS = -L$(MOCK_DIR)/build/lib -lpmic_mock
else
	CFLAGS += -DBUILD_HARDWARE
	MOCK_LDFLAGS =
endif

# ============================================================================
# Libraries
# ============================================================================

# Unity framework
UNITY_SRC = $(UNITY_DIR)/src/unity.c

# Linker flags
LDFLAGS = $(MOCK_LDFLAGS)

# ============================================================================
# Build Targets
# ============================================================================

# Default target
.PHONY: all
all: test_runner

# Test runner executable
test_runner: check-mock-lib $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES)
	@echo "Building test runner for TPS6522x-Q1 (BUILD=$(BUILD))..."
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources..."
	@$(CC) $(CFLAGS) $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o $@ $(LDFLAGS)
	@echo "Build complete: $@"

# ============================================================================
# Run Tests
# ============================================================================

.PHONY: test
test: check-mock-lib
	@echo "================================="
	@echo "Building and running TPS6522x-Q1 test suite..."
	@echo "================================="
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources..."
	@$(CC) $(CFLAGS) -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o test_runner $(LDFLAGS)
	@echo "Running tests..."
	./test_runner

# Mock mode test (explicit)
.PHONY: test-mock
test-mock:
	$(MAKE) BUILD=mock test

# Hardware mode test (explicit)
.PHONY: test-hardware
test-hardware:
	$(MAKE) BUILD=hardware test

# ============================================================================
# Coverage Analysis
# ============================================================================

# Run tests with coverage (generates both text and JSON reports)
.PHONY: coverage
coverage: check-mock-lib $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES)
	@echo "Building with coverage instrumentation..."
	@echo "Compiling Unity framework (relaxed warnings)..."
	@$(CC) -std=c99 -I$(UNITY_DIR)/src -DUNITY_INCLUDE_DOUBLE $(COV_FLAGS) -c $(UNITY_SRC) -o unity.o
	@echo "Compiling test and PMIC sources (strict warnings)..."
	@$(CC) $(CFLAGS) $(COV_FLAGS) -DPMIC_ERR_CNT_OVERFLOW_THR=100U -DPMIC_WARN_CNT_OVERFLOW_THR=100U -DPMIC_RETRY_CNT_OVERFLOW_THR=100U $(TEST_SOURCES) $(PLATFORM_SOURCES) $(PMIC_SOURCES) unity.o -o test_runner $(MOCK_LDFLAGS)
	@echo "Removing previous coverage data..."
	@rm -f *.gcda *.gcov
	@echo "Running tests..."
	@./test_runner || echo "WARNING: Some tests failed, but continuing with coverage generation..."
	@echo "Generating coverage reports (text and JSON)..."
	@python3 $(SCRIPTS_DIR)/generate_coverage_report.py $(PMIC_SOURCES)

# ============================================================================
# Clean
# ============================================================================

.PHONY: clean
clean:
	@echo "Cleaning test artifacts..."
	rm -f test_runner
	rm -f *.o */*.o unity.o
	rm -f *~ */*~
	rm -f *.gcda *.gcno *.gcov
	rm -rf coverage
	@echo "Clean complete"

.PHONY: clean-all
clean-all: clean
	$(MAKE) -C $(PMIC_ROOT) clean

# ============================================================================
# Help
# ============================================================================

.PHONY: help
help:
	@echo "TPS6522x-Q1 (Burton) Test Makefile"
	@echo "=================================="
	@echo ""
	@echo "Targets:"
	@echo "  all           - Build test runner (default)"
	@echo "  test          - Build and run tests"
	@echo "  test-mock     - Run tests in mock mode"
	@echo "  test-hardware - Run tests in hardware mode"
	@echo "  coverage      - Run tests with coverage analysis (text output)"
	@echo "  coverage-json - Generate JSON coverage report (for CI/CD)"
	@echo "  clean         - Remove test artifacts"
	@echo "  clean-all     - Clean tests and PMIC library"
	@echo "  help          - Show this help message"
	@echo ""
	@echo "Build modes:"
	@echo "  BUILD=mock     - Mock hardware mode (default)"
	@echo "  BUILD=hardware - Real hardware mode"
	@echo ""
	@echo "Examples:"
	@echo "  make                    # Build test runner"
	@echo "  make test               # Build and run (mock mode)"
	@echo "  make BUILD=hardware test # Run on hardware"
	@echo "  make coverage BUILD=mock # Run with coverage analysis"
	@echo "  make clean              # Clean artifacts"
