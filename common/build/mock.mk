# Mock library configuration for PMIC device tests
# Include this from device test Makefiles

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

# Mock library path
MOCK_LIB = $(MOCK_DIR)/build/lib/libpmic_mock.a

# Mock library sources (for dependency tracking)
MOCK_LIB_SRCS = $(wildcard $(MOCK_DIR)/mock/*.c) \
                $(wildcard $(MOCK_DIR)/adapters/*.c) \
                $(wildcard $(MOCK_DIR)/utils/*.c) \
                $(wildcard $(MOCK_DIR)/behaviors/*.c) \
                $(wildcard $(MOCK_DIR)/devices/*/*.c)

# Auto-build mock library if needed.
# Uses make's $(wildcard) function for Windows/Unix portability;
# avoids POSIX shell constructs ([ ], find) that fail on cmd.exe.
.PHONY: check-mock-lib
ifeq ($(wildcard $(MOCK_LIB)),)
check-mock-lib:
	@echo "Mock library not found, building it..."
	@"$(MAKE)" -C "$(MOCK_DIR)" CC=$(CC)
else
check-mock-lib:
	@echo "Mock library up to date"
endif
