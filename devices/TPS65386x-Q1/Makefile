# Copyright (c) 2026, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ifdef OS
RM = rm -f
RMDIR = rm -rf
MKDIR = mkdir -p
unix_to_dos = $(subst /,\,$1)
OSP = windows
else
ifeq ($(shell uname), Linux)
RM = rm -f
RMDIR = rm -rf
MKDIR = mkdir -p
OSP = unix
else
RM = rm -f
RMDIR = rm -rf
MKDIR = mkdir -p
OSP = macos
ifeq ($(CC),)
CC := clang
endif
endif
endif

# -fPIC is Linux/macOS only; Windows builds a static archive instead of .so
ifeq ($(OSP),windows)
    FPIC_FLAG :=
    LIB_EXT := a
else
    FPIC_FLAG := -fPIC
    LIB_EXT := so
endif

# Compiler selection (override Make's built-in CC=cc default)
ifeq ($(origin CC),default)
CC := clang
endif

INC	= -I./include

CFLAGS = $(FPIC_FLAG) $(INC) -Weverything \
				 -Wno-documentation \
				 -Wno-padded \
				 -Wno-poison-system-directories \
				 -Wno-covered-switch-default \
				 -Wno-unreachable-code-break \
				 -Werror
LIB_DIR = lib

DEBUGFLAGS = -O0 -g -D _DEBUG
RELEASEFLAGS = -O2 -D NDEBUG
build ?= release

ifeq ($(build), debug)
    CFLAGS += $(DEBUGFLAGS)
    TARGET = $(LIB_DIR)/libpmic.$(LIB_EXT)
endif

ifeq ($(build), release)
    CFLAGS += $(RELEASEFLAGS)
    TARGET = $(LIB_DIR)/libpmic.$(LIB_EXT)
endif

CFLAGS += $(CFLAGS_EXTRA)

SOURCES = \
	src/pmic.c \
	src/pmic_common.c \
	src/pmic_core.c \
	src/pmic_esm.c \
	src/pmic_fsm.c \
	src/pmic_gpio.c \
	src/pmic_io.c \
	src/pmic_irq.c \
	src/pmic_power.c \
	src/pmic_timer.c \
	src/pmic_wdg.c

OBJECTS = $(SOURCES:.c=.o)

# =============================================================================
# Test Configuration
# =============================================================================

# Unity framework path - can be overridden via environment variable
UNITY_PATH ?= ../../unity_framework
UNITY_SRC = $(UNITY_PATH)/src/unity.c

# Test directories
TEST_DIR = test
TEST_BUILD_DIR = build/test
TEST_COV_DIR = $(TEST_BUILD_DIR)/coverage

# Mock infrastructure source files
MOCK_SOURCES = \
	../../mock/mock/pmic_mock_core.c \
	../../mock/mock/pmic_mock_device.c \
	../../mock/adapters/io_adapter.c \
	../../mock/adapters/handle_adapter.c \
	../../mock/utils/crc_common.c \
	../../mock/behaviors/wdg_qa_common.c \
	../../mock/devices/tps6522x/tps6522x_behavior.c \
	../../mock/devices/tps6522x/tps6522x_regmap_adc.c \
	../../mock/devices/tps6522x/tps6522x_regmap_core.c \
	../../mock/devices/tps6522x/tps6522x_regmap_esm.c \
	../../mock/devices/tps6522x/tps6522x_regmap_fsm.c \
	../../mock/devices/tps6522x/tps6522x_regmap_gpio.c \
	../../mock/devices/tps6522x/tps6522x_regmap_io.c \
	../../mock/devices/tps6522x/tps6522x_regmap_irq.c \
	../../mock/devices/tps6522x/tps6522x_regmap_power.c \
	../../mock/devices/tps6522x/tps6522x_regmap_wdg.c \
	../../mock/devices/tps65386x/tps65386x_behavior.c \
	../../mock/devices/tps65386x/tps65386x_regmap_core.c \
	../../mock/devices/tps65386x/tps65386x_regmap_esm.c \
	../../mock/devices/tps65386x/tps65386x_regmap_gpio.c \
	../../mock/devices/tps65386x/tps65386x_regmap_io.c \
	../../mock/devices/tps65386x/tps65386x_regmap_irq.c \
	../../mock/devices/tps65386x/tps65386x_regmap_power.c \
	../../mock/devices/tps65386x/tps65386x_regmap_timer.c \
	../../mock/devices/tps65386x/tps65386x_regmap_wdg.c \
	../../mock/devices/lp8772x/lp8772x_behavior.c \
	../../mock/devices/lp8772x/lp8772x_regmap_core.c \
	../../mock/devices/lp8772x/lp8772x_regmap_fsm.c \
	../../mock/devices/lp8772x/lp8772x_regmap_io.c \
	../../mock/devices/lp8772x/lp8772x_regmap_irq.c \
	../../mock/devices/lp8772x/lp8772x_regmap_power.c \
	../../mock/devices/lp8772x/lp8772x_regmap_wdg.c \
	../../mock/devices/tps65036x/tps65036x_behavior.c \
	../../mock/devices/tps65036x/tps65036x_regmap_core.c \
	../../mock/devices/tps65036x/tps65036x_regmap_esm.c \
	../../mock/devices/tps65036x/tps65036x_regmap_gpio.c \
	../../mock/devices/tps65036x/tps65036x_regmap_io.c \
	../../mock/devices/tps65036x/tps65036x_regmap_irq.c \
	../../mock/devices/tps65036x/tps65036x_regmap_power.c \
	../../mock/devices/tps65036x/tps65036x_regmap_wdg.c

# Test source files
TEST_SOURCES = \
	$(TEST_DIR)/test_runner.c \
	$(TEST_DIR)/core/core_test.c \
	$(TEST_DIR)/wdg/wdg_test.c \
	$(TEST_DIR)/timer/timer_test.c \
	$(TEST_DIR)/esm/esm_test.c \
	$(TEST_DIR)/fsm/fsm_test.c \
	$(TEST_DIR)/gpio/gpio_test.c \
	$(TEST_DIR)/irq/irq_test.c \
	$(TEST_DIR)/io/io_test.c \
	$(TEST_DIR)/power/power_test.c \
	$(TEST_DIR)/test_common.c \
	$(TEST_DIR)/platform_mock.c

# Test executable names
TEST_EXEC_MOCK = $(TEST_BUILD_DIR)/pmic_test_mock
TEST_EXEC_HW = $(TEST_BUILD_DIR)/pmic_test_hw

# Test compiler flags
TEST_INC = -I$(TEST_DIR) -I./include -I./src -I$(UNITY_PATH)/src \
           -I../../mock/mock -I../../mock/adapters -I../../mock/devices -I../../mock/devices/tps65386x -I../../mock/utils
TEST_CFLAGS_BASE = $(TEST_INC) -Wall -Wextra -Werror \
				   -Wno-unused-parameter -Wno-unused-function
TEST_CFLAGS_MOCK = $(TEST_CFLAGS_BASE) -DPLATFORM_MOCK_MODE -DBUILD_MOCK -g -O0
TEST_CFLAGS_HW = $(TEST_CFLAGS_BASE) -g -O0
TEST_CFLAGS_COV = $(TEST_CFLAGS_MOCK) --coverage -fprofile-arcs -ftest-coverage
TEST_LDFLAGS = -lm
TEST_LDFLAGS_COV = $(TEST_LDFLAGS) --coverage

# Check if Unity framework exists
UNITY_EXISTS := $(shell test -f $(UNITY_SRC) && echo yes || echo no)

.PHONY: docs clean help test test-mock test-hardware test-run test-coverage test-clean test-check-unity

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(MKDIR) $(LIB_DIR)
ifeq ($(OSP),windows)
	ar rcs $@ $(OBJECTS)
else
	$(CC) $(CFLAGS) -shared -o $@ $(OBJECTS)
endif
	$(RM) $(OBJECTS)

docs:
	doxygen docs/ti.doxyfile

# =============================================================================
# Test Targets
# =============================================================================

# Check if Unity framework is available
test-check-unity:
ifneq ($(UNITY_EXISTS),yes)
	@echo "ERROR: Unity framework not found at $(UNITY_PATH)"
	@echo ""
	@echo "Please install Unity framework using one of these methods:"
	@echo ""
	@echo "Option 1: Add Unity as a git submodule (recommended):"
	@echo "  cd /path/to/pmic-lld"
	@echo "  mkdir -p third_party"
	@echo "  git submodule add https://github.com/ThrowTheSwitch/Unity.git third_party/unity"
	@echo ""
	@echo "Option 2: Clone Unity manually:"
	@echo "  cd /path/to/pmic-lld"
	@echo "  mkdir -p third_party"
	@echo "  git clone https://github.com/ThrowTheSwitch/Unity.git third_party/unity"
	@echo ""
	@echo "Option 3: Use a different Unity path:"
	@echo "  make test UNITY_PATH=/path/to/unity"
	@echo ""
	@exit 1
endif
	@echo "Unity framework found at $(UNITY_PATH)"

# Default test target - builds and runs mock tests
test: test-run

# Build tests with mock platform (for CI/CD)
test-mock: test-check-unity
	@echo "Building mock tests..."
	$(MKDIR) $(TEST_BUILD_DIR)
	$(CC) $(TEST_CFLAGS_MOCK) -o $(TEST_EXEC_MOCK) \
		$(SOURCES) $(MOCK_SOURCES) $(TEST_SOURCES) $(UNITY_SRC) \
		$(TEST_LDFLAGS)
	@echo "Mock test executable created: $(TEST_EXEC_MOCK)"
	@echo "Run with: make test-run"

# Build tests with real hardware
test-hardware: test-check-unity
	@echo "Building hardware tests..."
	$(MKDIR) $(TEST_BUILD_DIR)
	$(CC) $(TEST_CFLAGS_HW) -o $(TEST_EXEC_HW) \
		$(SOURCES) $(TEST_SOURCES) $(UNITY_SRC) \
		$(TEST_LDFLAGS)
	@echo "Hardware test executable created: $(TEST_EXEC_HW)"
	@echo "Run with: ./$(TEST_EXEC_HW)"

# Build and execute mock tests
test-run: test-mock
	@echo "Running mock tests..."
	@echo "================================"
	./$(TEST_EXEC_MOCK)
	@echo "================================"
	@echo "Tests completed!"

# Build with coverage, run tests, generate report
test-coverage: test-check-unity
	@echo "Building tests with coverage instrumentation..."
	$(MKDIR) $(TEST_BUILD_DIR)
	$(MKDIR) $(TEST_COV_DIR)
	$(CC) $(TEST_CFLAGS_COV) -o $(TEST_EXEC_MOCK) \
		$(SOURCES) $(MOCK_SOURCES) $(TEST_SOURCES) $(UNITY_SRC) \
		$(TEST_LDFLAGS_COV)
	@echo "Running tests with coverage..."
	./$(TEST_EXEC_MOCK)
	@echo "Generating coverage report..."
	@if command -v gcov >/dev/null 2>&1; then \
		gcov -o . $(SOURCES) $(TEST_SOURCES) > /dev/null 2>&1; \
		mv *.gcov $(TEST_COV_DIR)/ 2>/dev/null || true; \
		mv *.gcda $(TEST_COV_DIR)/ 2>/dev/null || true; \
		mv *.gcno $(TEST_COV_DIR)/ 2>/dev/null || true; \
		echo "Coverage files generated in $(TEST_COV_DIR)/"; \
		echo ""; \
		if command -v lcov >/dev/null 2>&1; then \
			echo "Generating HTML coverage report with lcov..."; \
			lcov --capture --directory . --output-file $(TEST_COV_DIR)/coverage.info; \
			lcov --remove $(TEST_COV_DIR)/coverage.info '/usr/*' '$(UNITY_PATH)/*' --output-file $(TEST_COV_DIR)/coverage_filtered.info; \
			if command -v genhtml >/dev/null 2>&1; then \
				genhtml $(TEST_COV_DIR)/coverage_filtered.info --output-directory $(TEST_COV_DIR)/html; \
				echo "HTML coverage report: $(TEST_COV_DIR)/html/index.html"; \
			else \
				echo "Install genhtml to generate HTML reports: sudo apt-get install lcov"; \
			fi; \
		else \
			echo "Install lcov for HTML reports: sudo apt-get install lcov"; \
		fi; \
	else \
		echo "ERROR: gcov not found. Install with: sudo apt-get install gcc"; \
	fi

# Clean test artifacts
test-clean:
	@echo "Cleaning test artifacts..."
	$(RM) $(TEST_EXEC_MOCK)
	$(RM) $(TEST_EXEC_HW)
	$(RM) *.gcov
	$(RM) *.gcda
	$(RM) *.gcno
	$(RMDIR) $(TEST_BUILD_DIR)
	@echo "Test artifacts cleaned."

help:
	@echo "# make help"
	@echo "# ------------------------------------------------------"
	@echo "#  make [OPTIONAL MAKE VARIABLES] Note: use gmake for windows"
	@echo "Supported targets: "
	@echo "------------------"
	@echo "all            : Builds the library"
	@echo "clean          : Cleans the library"
	@echo "docs           : Generates Doxygen documentation"
	@echo ""
	@echo "Test targets:"
	@echo "-------------"
	@echo "test           : Build mock tests (default test target)"
	@echo "test-mock      : Build tests with mock platform (for CI/CD)"
	@echo "test-hardware  : Build tests with real hardware platform"
	@echo "test-run       : Build and execute mock tests"
	@echo "test-coverage  : Build with coverage, run tests, generate report"
	@echo "test-clean     : Clean test artifacts"
	@echo ""
	@echo "Optional make variables:"
	@echo "------------------------"
	@echo "CC=[Cross-compiler to be used]"
	@echo "    Default: clang (gcc is a fallback)"
	@echo "BUILD_PROFILE=[release debug]"
	@echo "    Default: release"
	@echo "OS=[Windows_NT linux]"
	@echo "    Default: Windows_NT"
	@echo "UNITY_PATH=[Path to Unity framework]"
	@echo "    Default: ../../third_party/unity"

clean:
	@echo "Cleaning..."
	@if [ -e "$(TARGET)" ]; then rm -f "$(TARGET)" && echo "  Removed: $(TARGET)"; \
	  else echo "  Note: $(TARGET) not found, skipping"; fi
	@rm -f $(OBJECTS)
	@if [ -d "docs/html" ]; then rm -rf docs/html && echo "  Removed: docs/html/"; \
	  else echo "  Note: docs/html/ not found, skipping"; fi
	@if [ -d "$(LIB_DIR)" ]; then rm -rf "$(LIB_DIR)" && echo "  Removed: $(LIB_DIR)/"; \
	  else echo "  Note: $(LIB_DIR)/ not found, skipping"; fi
	@echo "Clean complete."
