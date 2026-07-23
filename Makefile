# Root-level Makefile for PMIC Low-Level Driver (LLD)
# Supports building all device variants from a single location

# Default compiler (can be overridden with CC=<compiler>)
# Use immediate assignment to override Make's built-in CC=cc default
ifeq ($(origin CC),default)
CC := clang
endif

# Normalize PMIC_MOCK_DIR if set: convert relative paths to absolute
# On Windows, paths like C:/... already contain a drive letter - skip abspath
# which would prepend CWD on any path that doesn't start with /
ifdef PMIC_MOCK_DIR
    ifeq ($(findstring :/,$(PMIC_MOCK_DIR)),)
        export PMIC_MOCK_DIR := $(abspath $(PMIC_MOCK_DIR))
    else
        export PMIC_MOCK_DIR := $(PMIC_MOCK_DIR)
    endif
endif

# Normalize UNITY_DIR if set: convert relative paths to absolute
ifdef UNITY_DIR
    ifeq ($(findstring :/,$(UNITY_DIR)),)
        export UNITY_DIR := $(abspath $(UNITY_DIR))
    else
        export UNITY_DIR := $(UNITY_DIR)
    endif
endif

# Validate UNITY_DIR points to valid Unity source directory
ifdef UNITY_DIR
    ifeq ($(wildcard $(UNITY_DIR)/src/unity.c),)
        $(error UNITY_DIR is set to '$(UNITY_DIR)' but unity.c not found. Expected: $(UNITY_DIR)/src/unity.c)
    endif
endif

# Coverage tools from pmic-lld-utils
# Auto-detect utils directory (sibling directory)
ifndef PMIC_UTILS_DIR
    PMIC_UTILS_DIR := $(shell test -d ../pmic-lld-utils && echo ../pmic-lld-utils)
endif

# Check if pmic-coverage is globally installed, otherwise use uv run
PMIC_COVERAGE := $(shell which pmic-coverage 2>/dev/null)
ifndef PMIC_COVERAGE
    PMIC_COVERAGE = uv run --project $(PMIC_UTILS_DIR)/coverage pmic-coverage
endif
export PMIC_COVERAGE

# List of all PMIC device variants
DEVICES := LP8772x-Q1 TPS65036x-Q1 TPS6522x-Q1 TPS65386x-Q1

# Device directory base path
DEVICE_DIR := devices

# Pass-through variables for device Makefiles
MAKE_VARS := CC=$(CC)
ifdef build
	MAKE_VARS += build=$(build)
endif

# Default target: build all devices
.PHONY: all
all:
	@echo "Building all PMIC device variants..."
	@for device in $(DEVICES); do \
		echo ""; \
		echo "==================== Building $$device ===================="; \
		"$(MAKE)" -C $(DEVICE_DIR)/$$device $(MAKE_VARS) || exit 1; \
	done

# Build a specific device
.PHONY: build-device
build-device:
ifndef DEVICE
	@echo "Error: DEVICE parameter is required"
	@echo "Usage: make DEVICE=<device-name> [build=release|debug] [CC=<compiler>]"
	@echo "Available devices: $(DEVICES)"
	@exit 1
endif
	@echo "Building device: $(DEVICE)"
	@if [ ! -d "$(DEVICE_DIR)/$(DEVICE)" ]; then \
		echo "Error: Device directory $(DEVICE_DIR)/$(DEVICE) not found"; \
		echo "Available devices: $(DEVICES)"; \
		exit 1; \
	fi
	"$(MAKE)" -C $(DEVICE_DIR)/$(DEVICE) $(MAKE_VARS)

# Clean all devices
.PHONY: clean
clean:
ifndef DEVICE
	@echo "Cleaning all PMIC device variants..."
	@for device in $(DEVICES); do \
		echo "Cleaning $$device..."; \
		"$(MAKE)" -C $(DEVICE_DIR)/$$device clean || exit 1; \
		if [ -d "$(DEVICE_DIR)/$$device/test" ]; then \
			"$(MAKE)" -C $(DEVICE_DIR)/$$device/test clean || exit 1; \
		fi; \
	done
	@echo "All devices cleaned!"
else
	@echo "Cleaning device: $(DEVICE)"
	@if [ ! -d "$(DEVICE_DIR)/$(DEVICE)" ]; then \
		echo "Error: Device directory $(DEVICE_DIR)/$(DEVICE) not found"; \
		echo "Available devices: $(DEVICES)"; \
		exit 1; \
	fi
	"$(MAKE)" -C $(DEVICE_DIR)/$(DEVICE) clean
	@if [ -d "$(DEVICE_DIR)/$(DEVICE)/test" ]; then \
		"$(MAKE)" -C $(DEVICE_DIR)/$(DEVICE)/test clean; \
	fi
endif

# Run tests (context-aware: all devices or specific device)
.PHONY: test
test:
ifndef DEVICE
	@echo "Running tests for all PMIC device variants (mock platform)..."
	@echo ""
	@results_file=$$(mktemp); \
	any_failed=0; \
	total_tests=0; \
	total_passed=0; \
	total_failed=0; \
	total_ignored=0; \
	for device in $(DEVICES); do \
		echo ""; \
		echo "==================== Testing $$device ===================="; \
		if [ -d "$(DEVICE_DIR)/$$device/test" ]; then \
			output_file=$$(mktemp); \
			result=0; \
			"$(MAKE)" -C $(DEVICE_DIR)/$$device/test test BUILD=mock > $$output_file 2>&1; \
			result=$$?; \
			cat $$output_file; \
			tests=0; failures=0; ignored=0; \
			if grep -q "Tests.*Failures.*Ignored" $$output_file; then \
				stats=$$(grep "Tests.*Failures.*Ignored" $$output_file | tail -1); \
				tests=$$(echo "$$stats" | awk '{print $$1}'); \
				failures=$$(echo "$$stats" | awk '{print $$3}'); \
				ignored=$$(echo "$$stats" | awk '{print $$5}'); \
			fi; \
			passed=$$((tests - failures)); \
			total_tests=$$((total_tests + tests)); \
			total_passed=$$((total_passed + passed)); \
			total_failed=$$((total_failed + failures)); \
			total_ignored=$$((total_ignored + ignored)); \
			if [ $$result -eq 0 ]; then \
				status="PASSED"; \
			else \
				status="FAILED"; \
				any_failed=1; \
			fi; \
			printf "%-20s %8s %8s %8s %8s   %s\n" "$$device" "$$tests" "$$passed" "$$failures" "$$ignored" "$$status" >> $$results_file; \
			rm -f $$output_file; \
		else \
			echo "No tests found for $$device"; \
		fi; \
	done; \
	echo ""; \
	echo "================================================================================"; \
	echo "Test Summary"; \
	echo "================================================================================"; \
	echo ""; \
	printf "%-20s %8s %8s %8s %8s   %s\n" "Device" "Tests" "Passed" "Failed" "Ignored" "Status"; \
	echo "--------------------------------------------------------------------------------"; \
	cat $$results_file; \
	echo "--------------------------------------------------------------------------------"; \
	printf "%-20s %8s %8s %8s %8s\n" "TOTAL" "$$total_tests" "$$total_passed" "$$total_failed" "$$total_ignored"; \
	echo "================================================================================"; \
	rm -f $$results_file; \
	echo ""; \
	if [ $$any_failed -eq 1 ]; then \
		exit 1; \
	else \
		echo "All device tests completed successfully!"; \
	fi
else
	@echo "Testing device: $(DEVICE) (mock platform)"
	@if [ ! -d "$(DEVICE_DIR)/$(DEVICE)/test" ]; then \
		echo "Error: Test directory $(DEVICE_DIR)/$(DEVICE)/test not found"; \
		exit 1; \
	fi
	"$(MAKE)" -C $(DEVICE_DIR)/$(DEVICE)/test test BUILD=mock
endif

# Generate coverage reports (context-aware: all devices or specific device)
.PHONY: coverage
coverage:
ifndef DEVICE
	@echo "Generating coverage reports for all PMIC device variants..."
	@rm -rf coverage
	@mkdir -p coverage
	@json_reports=""; \
	for device in $(DEVICES); do \
		echo ""; \
		echo "==================== Coverage for $$device ===================="; \
		if [ -d "$(DEVICE_DIR)/$$device/test" ]; then \
			"$(MAKE)" -C $(DEVICE_DIR)/$$device/test coverage BUILD=mock || exit 1; \
			latest_json=$$(ls -t $(DEVICE_DIR)/$$device/test/coverage/*.json 2>/dev/null | head -1); \
			if [ -n "$$latest_json" ]; then \
				json_reports="$$json_reports $$latest_json"; \
			fi; \
		else \
			echo "No tests found for $$device"; \
		fi; \
	done; \
	echo ""; \
	echo "==================== Generating Aggregate Report ===================="; \
	if [ -n "$$json_reports" ]; then \
		$(PMIC_COVERAGE) aggregate $$json_reports; \
	else \
		echo "ERROR: No coverage reports generated!"; \
		exit 1; \
	fi
	@echo ""
	@echo "Coverage analysis completed!"
	@echo "Individual reports: devices/*/test/coverage/"
	@echo "Aggregate report:   coverage/"
else
	@echo "Generating coverage report for device: $(DEVICE)"
	@if [ ! -d "$(DEVICE_DIR)/$(DEVICE)/test" ]; then \
		echo "Error: Test directory $(DEVICE_DIR)/$(DEVICE)/test not found"; \
		exit 1; \
	fi
	"$(MAKE)" -C $(DEVICE_DIR)/$(DEVICE)/test coverage BUILD=mock
	@echo ""
	@echo "Coverage reports generated:"
	@echo "  $(DEVICE_DIR)/$(DEVICE)/test/coverage/"
endif

# Display help information
.PHONY: help
help:
	@echo "PMIC Low-Level Driver (LLD) - Root Makefile"
	@echo ""
	@echo "Usage:"
	@echo "  make                                    Build all devices"
	@echo "  make all                                Build all devices"
	@echo "  make DEVICE=<name>                      Build a specific device"
	@echo "  make build:<device>                     Build a specific device (shorthand)"
	@echo "  make test                               Run tests for all devices (mock platform)"
	@echo "  make test:<device>                      Run tests for a specific device (shorthand)"
	@echo "  make coverage                           Generate coverage reports for all devices"
	@echo "  make coverage:<device>                  Generate coverage for a specific device (shorthand)"
	@echo "  make clean                              Clean all devices"
	@echo "  make clean:<device>                     Clean a specific device (shorthand)"
	@echo "  make help                               Show this help message"
	@echo ""
	@echo "Optional Variables:"
	@echo "  build=<profile>                         Build profile (release or debug)"
	@echo "                                          Default: release"
	@echo "  CC=<compiler>                           C compiler (clang, gcc, etc.)"
	@echo "                                          Default: clang"
	@echo ""
	@echo "Available Devices:"
	@for device in $(DEVICES); do \
		echo "  - $$device"; \
	done
	@echo ""
	@echo "Examples:"
	@echo "  make                                    # Build all devices (release mode)"
	@echo "  make build=debug                        # Build all devices (debug mode)"
	@echo "  make build:TPS6522x-Q1                  # Build TPS6522x-Q1 only"
	@echo "  make build:LP8772x-Q1 CC=gcc            # Build LP8772x-Q1 with gcc instead of clang"
	@echo "  make test                               # Run all device tests on mock platform"
	@echo "  make test:LP8772x-Q1                    # Run LP8772x-Q1 tests only"
	@echo "  make coverage                           # Generate coverage for all devices"
	@echo "  make coverage:TPS6522x-Q1               # Generate coverage for TPS6522x-Q1 only"
	@echo "  make clean                              # Clean all devices"
	@echo "  make clean:TPS65036x-Q1                 # Clean TPS65036x-Q1 only"

# Handle DEVICE parameter for default target
ifneq ($(DEVICE),)
.DEFAULT_GOAL := build-device
endif

# ============================================================================
# Convenience pattern rules: make target:device
# Supports intuitive syntax like "make clean:TPS65036x-Q1" or "make test:LP8772x-Q1"
# ============================================================================

# Pattern rule for clean:device
clean\:%:
	@"$(MAKE)" clean DEVICE=$*

# Pattern rule for test:device
test\:%:
	@"$(MAKE)" test DEVICE=$*

# Pattern rule for coverage:device
coverage\:%:
	@"$(MAKE)" coverage DEVICE=$*

# Pattern rule for build:device (explicit build target)
build\:%:
	@"$(MAKE)" DEVICE=$*
