# Coverage analysis configuration for PMIC device tests
# Include this from device test Makefiles

# Coverage flags
COV_FLAGS = -fprofile-instr-generate -fcoverage-mapping -fcoverage-mcdc

# Coverage tools - can be overridden via environment variables
LLVM_PROFDATA ?= llvm-profdata
LLVM_COV ?= llvm-cov

# Coverage tools from pmic-lld-utils (inherit from root or detect)
ifndef PMIC_COVERAGE
    PMIC_UTILS_DIR ?= ../../../../pmic-lld-utils
    PMIC_COVERAGE := $(shell which pmic-coverage 2>/dev/null)
    ifndef PMIC_COVERAGE
        PMIC_COVERAGE = uv run --project $(PMIC_UTILS_DIR)/coverage pmic-coverage
    endif
endif
