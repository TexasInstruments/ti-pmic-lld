# Coverage analysis configuration for PMIC device tests
# Include this from device test Makefiles

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

# Coverage tools from pmic-lld-utils (inherit from root or detect)
ifndef PMIC_COVERAGE
    PMIC_UTILS_DIR ?= ../../../../pmic-lld-utils
    PMIC_COVERAGE := $(shell which pmic-coverage 2>/dev/null)
    ifndef PMIC_COVERAGE
        PMIC_COVERAGE = uv run --project $(PMIC_UTILS_DIR)/coverage pmic-coverage
    endif
endif
