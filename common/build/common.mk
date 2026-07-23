# Common build configuration for PMIC device tests
# Include this from device test Makefiles

# Build type (mock or host)
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

# Unity source file
UNITY_SRC = $(UNITY_DIR)/src/unity.c

# Shared test infrastructure directory
TEST_COMMON_DIR = ../../../common/test

# Scripts directory
SCRIPTS_DIR = ../../../scripts

# Standard CFLAGS structure (device Makefiles should append device-specific includes)
BASE_CFLAGS = $(WARN_FLAGS) \
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
         -std=c99 $(TARGET_FLAG)

# clock_gettime is in winpthread on MinGW; harmless on Linux
ifeq ($(OS),Windows_NT)
    LDFLAGS += -lpthread
endif
