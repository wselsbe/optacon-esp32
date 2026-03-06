#!/usr/bin/env bash
# Lint Python (ruff) and C (clang-format) files.
# Usage: ./scripts/lint.sh [--fix]
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FIX=0
if [[ "${1:-}" == "--fix" ]]; then FIX=1; fi

ERRORS=0

# --- Python (ruff) ---
echo "=== Python (ruff) ==="
if command -v ruff &>/dev/null; then
    if [[ $FIX -eq 1 ]]; then
        ruff check --fix "$ROOT/python" "$ROOT/test" "$ROOT/scripts" || ERRORS=1
    else
        ruff check "$ROOT/python" "$ROOT/test" "$ROOT/scripts" || ERRORS=1
    fi
else
    echo "ruff not found, skipping Python lint"
fi

# --- C (clang-format) ---
echo "=== C (clang-format) ==="
if command -v clang-format &>/dev/null; then
    C_FILES=$(find "$ROOT/modules" -name '*.c' -o -name '*.h' 2>/dev/null)
    if [[ -n "$C_FILES" ]]; then
        if [[ $FIX -eq 1 ]]; then
            echo "$C_FILES" | xargs clang-format -i
            echo "Formatted $(echo "$C_FILES" | wc -l) files"
        else
            if echo "$C_FILES" | xargs clang-format --dry-run --Werror 2>&1; then
                echo "All C files formatted correctly"
            else
                ERRORS=1
            fi
        fi
    else
        echo "No C files found"
    fi
else
    echo "clang-format not found, skipping C lint"
fi

if [[ $ERRORS -ne 0 ]]; then
    echo ""; echo "Lint errors found. Run with --fix to auto-fix."
    exit 1
fi
echo ""; echo "All clean."
