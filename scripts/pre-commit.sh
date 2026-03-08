#!/usr/bin/env bash
# Git pre-commit hook: auto-fix and lint staged Python and C/H files.
# Install: ln -sf ../../scripts/pre-commit.sh .git/hooks/pre-commit

set -e

# Python: auto-fix then check
PY_FILES=$(git diff --cached --name-only --diff-filter=ACM -- "*.py")
if [ -n "$PY_FILES" ]; then
    for f in $PY_FILES; do
        [ -f "$f" ] || continue
        ruff check --fix "$f" 2>/dev/null && git add "$f"
    done
    ERRORS=""
    for f in $PY_FILES; do
        [ -f "$f" ] || continue
        OUT=$(ruff check "$f" 2>&1) || ERRORS="$ERRORS$OUT"$'\n'
    done
    if [ -n "$ERRORS" ]; then
        echo "ruff: unfixable lint errors in staged files:"
        echo "$ERRORS"
        exit 1
    fi
fi

# C/H: check formatting
CH_FILES=$(git diff --cached --name-only --diff-filter=ACM -- "*.c" "*.h")
if [ -n "$CH_FILES" ]; then
    ERRORS=""
    for f in $CH_FILES; do
        [ -f "$f" ] || continue
        OUT=$(clang-format --dry-run --Werror "$f" 2>&1) || ERRORS="$ERRORS$OUT"$'\n'
    done
    if [ -n "$ERRORS" ]; then
        echo "clang-format: formatting errors in staged files:"
        echo "$ERRORS"
        exit 1
    fi
fi
