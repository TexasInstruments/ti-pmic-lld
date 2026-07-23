#!/usr/bin/env bash
# run_tests.sh — zero-config test runner for PMIC LLD mock test suites
#
# Usage:
#   ./run_tests.sh              # run all four device test suites
#   ./run_tests.sh LP8772x-Q1  # run one device only
#
# No environment variables need to be set before running this script.
# It resolves all paths internally and auto-builds libpmic_mock.a if needed.

set -euo pipefail

# ---------------------------------------------------------------------------
# Resolve the script's own directory as an absolute path.
# Linux has 'realpath'; MSYS2 may not — fall back to pwd -P.
# ---------------------------------------------------------------------------
if command -v realpath >/dev/null 2>&1; then
    SCRIPT_DIR="$(realpath "$(dirname "$0")")"
else
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd -P)"
fi

# ---------------------------------------------------------------------------
# Set MAKE=make so $(MAKE) in Makefiles never expands to a path-with-spaces
# (e.g. "C:/Program Files (x86)/GnuWin32/bin/make"). Harmless on Linux.
# ---------------------------------------------------------------------------
export MAKE=make

# ---------------------------------------------------------------------------
# Derive sibling directory paths from script location
# ---------------------------------------------------------------------------
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

export PMIC_LLD_DIR="$SCRIPT_DIR"
export PMIC_MOCK_DIR="$REPO_ROOT/pmic-lld-mocking"
export UNITY_DIR="$REPO_ROOT/unity"

# ---------------------------------------------------------------------------
# Validate that required sibling directories exist
# ---------------------------------------------------------------------------
if [ ! -d "$PMIC_MOCK_DIR" ]; then
    echo "ERROR: pmic-lld-mocking directory not found at: $PMIC_MOCK_DIR"
    echo "Expected directory layout:"
    echo "  <parent>/"
    echo "    pmic-lld/          <- run this script from here"
    echo "    pmic-lld-mocking/"
    echo "    unity/"
    exit 1
fi

if [ ! -d "$UNITY_DIR" ]; then
    echo "ERROR: unity directory not found at: $UNITY_DIR"
    echo "Expected directory layout:"
    echo "  <parent>/"
    echo "    pmic-lld/"
    echo "    pmic-lld-mocking/"
    echo "    unity/             <- must exist"
    exit 1
fi

if [ ! -f "$UNITY_DIR/src/unity.c" ]; then
    echo "ERROR: unity.c not found at: $UNITY_DIR/src/unity.c"
    exit 1
fi

# ---------------------------------------------------------------------------
# Auto-build libpmic_mock.a if it is missing or out of date
# ---------------------------------------------------------------------------
MOCK_LIB="$PMIC_MOCK_DIR/build/lib/libpmic_mock.a"

if [ ! -f "$MOCK_LIB" ]; then
    echo ">>> libpmic_mock.a not found — building mock library..."
    make -C "$PMIC_MOCK_DIR" PMIC_LLD_DIR="$PMIC_LLD_DIR"
    echo ">>> Mock library built."
    echo ""
fi

# ---------------------------------------------------------------------------
# Determine which devices to test
# ---------------------------------------------------------------------------
ALL_DEVICES="LP8772x-Q1 TPS65036x-Q1 TPS6522x-Q1 TPS65386x-Q1"

if [ $# -eq 0 ]; then
    DEVICES="$ALL_DEVICES"
elif [ $# -eq 1 ]; then
    DEVICE="$1"
    # Validate
    found=0
    for d in $ALL_DEVICES; do
        if [ "$d" = "$DEVICE" ]; then found=1; break; fi
    done
    if [ $found -eq 0 ]; then
        echo "ERROR: Unknown device '$DEVICE'"
        echo "Available devices: $ALL_DEVICES"
        exit 1
    fi
    DEVICES="$DEVICE"
else
    echo "Usage: $0 [DEVICE]"
    echo "  DEVICE: one of $ALL_DEVICES"
    exit 1
fi

# ---------------------------------------------------------------------------
# Run tests and collect results
# ---------------------------------------------------------------------------
PASS_LIST=""
FAIL_LIST=""

for device in $DEVICES; do
    test_dir="$SCRIPT_DIR/devices/$device/test"
    if [ ! -d "$test_dir" ]; then
        echo "WARNING: No test directory for $device ($test_dir) — skipping"
        continue
    fi

    echo ""
    echo "==================== Testing $device ===================="
    if make -C "$test_dir" test BUILD=mock \
            PMIC_MOCK_DIR="$PMIC_MOCK_DIR" \
            UNITY_DIR="$UNITY_DIR"; then
        PASS_LIST="$PASS_LIST $device"
    else
        FAIL_LIST="$FAIL_LIST $device"
    fi
done

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "========================================================================"
echo "Test Summary"
echo "========================================================================"

if [ -n "$PASS_LIST" ]; then
    for d in $PASS_LIST; do
        printf "  %-22s  PASSED\n" "$d"
    done
fi
if [ -n "$FAIL_LIST" ]; then
    for d in $FAIL_LIST; do
        printf "  %-22s  FAILED\n" "$d"
    done
fi

echo "========================================================================"

if [ -n "$FAIL_LIST" ]; then
    echo "RESULT: Some tests FAILED."
    exit 1
else
    echo "RESULT: All tests PASSED."
    exit 0
fi
