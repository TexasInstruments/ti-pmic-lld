# Root-level Makefile for PMIC Low-Level Driver (LLD)
# Supports building all device variants from a single location

# List of all PMIC device variants
DEVICES := LP8772x-Q1 TPS65036x-Q1 TPS6522x-Q1 TPS65386x-Q1

# Device directory base path
DEVICE_DIR := devices

# Pass-through variables for device Makefiles
MAKE_VARS :=
ifdef build
	MAKE_VARS += build=$(build)
endif
ifdef CC
	MAKE_VARS += CC=$(CC)
endif

# Default target: build all devices
.PHONY: all
all:
	@echo "Building all PMIC device variants..."
	@for device in $(DEVICES); do \
		echo ""; \
		echo "==================== Building $$device ===================="; \
		$(MAKE) -C $(DEVICE_DIR)/$$device $(MAKE_VARS) || exit 1; \
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
	$(MAKE) -C $(DEVICE_DIR)/$(DEVICE) $(MAKE_VARS)

# Clean all devices
.PHONY: clean
clean:
ifndef DEVICE
	@echo "Cleaning all PMIC device variants..."
	@for device in $(DEVICES); do \
		echo "Cleaning $$device..."; \
		$(MAKE) -C $(DEVICE_DIR)/$$device clean; \
	done
	@echo "All devices cleaned!"
else
	@echo "Cleaning device: $(DEVICE)"
	@if [ ! -d "$(DEVICE_DIR)/$(DEVICE)" ]; then \
		echo "Error: Device directory $(DEVICE_DIR)/$(DEVICE) not found"; \
		echo "Available devices: $(DEVICES)"; \
		exit 1; \
	fi
	$(MAKE) -C $(DEVICE_DIR)/$(DEVICE) clean
endif

# Clean all devices (explicit target, same as 'make clean')
.PHONY: clean-all
clean-all:
	@echo "Cleaning all PMIC device variants..."
	@for device in $(DEVICES); do \
		echo "Cleaning $$device..."; \
		$(MAKE) -C $(DEVICE_DIR)/$$device clean; \
	done

# Display help information
.PHONY: help
help:
	@echo "PMIC Low-Level Driver (LLD) - Root Makefile"
	@echo ""
	@echo "Usage:"
	@echo "  make                                    Build all devices"
	@echo "  make all                                Build all devices"
	@echo "  make DEVICE=<name>                      Build a specific device"
	@echo "  make clean                              Clean all devices"
	@echo "  make clean DEVICE=<name>                Clean a specific device"
	@echo "  make clean-all                          Clean all devices"
	@echo "  make help                               Show this help message"
	@echo ""
	@echo "Optional Variables:"
	@echo "  build=<profile>                         Build profile (release or debug)"
	@echo "                                          Default: release"
	@echo "  CC=<compiler>                           C compiler (gcc, clang, etc.)"
	@echo "                                          Default: gcc (Linux/Windows), clang (macOS)"
	@echo ""
	@echo "Available Devices:"
	@for device in $(DEVICES); do \
		echo "  - $$device"; \
	done
	@echo ""
	@echo "Examples:"
	@echo "  make                                    # Build all devices (release mode)"
	@echo "  make build=debug                        # Build all devices (debug mode)"
	@echo "  make DEVICE=TPS6522x-Q1                 # Build TPS6522x-Q1 only"
	@echo "  make DEVICE=LP8772x-Q1 CC=gcc           # Build LP8772x-Q1 with gcc"
	@echo "  make clean                              # Clean all devices"
	@echo "  make clean DEVICE=TPS65036x-Q1          # Clean TPS65036x-Q1 only"

# Handle DEVICE parameter for default target
ifneq ($(DEVICE),)
.DEFAULT_GOAL := build-device
endif
