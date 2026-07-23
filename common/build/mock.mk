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

# Lock directory used as a cross-process mutex for the mock library build.
# mkdir is atomic on Windows (Git Bash) and macOS/Linux: exactly one process
# succeeds; the rest detect the existing directory and wait. This prevents the
# ar rename race when multiple device test builds run in parallel.
MOCK_LOCK_DIR = $(MOCK_DIR)/.mock_build_lock

.PHONY: check-mock-lib
check-mock-lib:
	@if [ -f "$(MOCK_LIB)" ]; then \
		rmdir "$(MOCK_LOCK_DIR)" 2>/dev/null; \
		echo "Mock library up to date"; \
	elif mkdir "$(MOCK_LOCK_DIR)" 2>/dev/null; then \
		echo "Mock library not found, building it..."; \
		if "$(MAKE)" -C "$(MOCK_DIR)" CC=$(CC) PMIC_LLD_DIR=$(abspath $(MOCK_DIR)/../pmic-lld); then \
			rmdir "$(MOCK_LOCK_DIR)"; \
		else \
			rmdir "$(MOCK_LOCK_DIR)"; \
			exit 1; \
		fi; \
	else \
		echo "Waiting for mock library build..."; \
		while [ -d "$(MOCK_LOCK_DIR)" ]; do sleep 1; done; \
		if [ ! -f "$(MOCK_LIB)" ]; then \
			echo "ERROR: Mock library build failed in another process" >&2; exit 1; \
		fi; \
		echo "Mock library ready"; \
	fi
