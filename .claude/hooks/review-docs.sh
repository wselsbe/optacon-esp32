#!/usr/bin/env bash
# Pre-commit hook: require Claude to review staged documentation files.
# Blocks the commit until a hash-based marker confirms review was done.

CMD=$(jq -r .tool_input.command < /dev/stdin)

# Only trigger on git commit commands
case "$CMD" in *"git commit"*) ;; *) exit 0;; esac

# Find staged doc files (excluding docs/plans/)
DOC_FILES=$(git diff --cached --name-only --diff-filter=ACM -- \
  "docs/*.md" "docs/**/*.md" "README.md" "CLAUDE.md" "web/docs.html" \
  ".claude/skills/**/*.md" ".claude/agents/*.md" \
  | grep -v "^docs/plans/")

[ -z "$DOC_FILES" ] && exit 0

# Hash the staged doc content to detect changes after review
HASH=$(git diff --cached -- $DOC_FILES | git hash-object --stdin)
MARKER=".claude/.doc-reviewed"
STORED=$(cat "$MARKER" 2>/dev/null)

# If review marker matches current content, allow commit and clean up
if [ "$HASH" = "$STORED" ]; then
  rm -f "$MARKER"
  exit 0
fi

# Block commit and request review
echo "DOCUMENTATION REVIEW REQUIRED"
echo ""
echo "Staged documentation files:"
for f in $DOC_FILES; do echo "  - $f"; done
echo ""
echo "Review these files for accuracy, consistency with the codebase, and completeness."
echo "Check that code examples, API signatures, file paths, and pin mappings are correct."
echo ""
echo "After reviewing (and fixing any issues), run:"
echo "  echo $HASH > .claude/.doc-reviewed"
echo "Then re-run the commit."
exit 1
