#!/usr/bin/env bash
set -euo pipefail

REPO_NAME="laser-airmove-planner"
OWNER="qzxhsp9"
VISIBILITY="--public"

if ! command -v git >/dev/null 2>&1; then
  echo "git is required" >&2
  exit 1
fi

if ! command -v gh >/dev/null 2>&1; then
  echo "GitHub CLI 'gh' is not installed. Install it or create the repository on GitHub manually." >&2
  exit 1
fi

git init

git add .
if ! git diff --cached --quiet; then
  git commit -m "Initial laser air-move planner scaffold"
fi

gh repo create "${OWNER}/${REPO_NAME}" ${VISIBILITY} --source=. --remote=origin --push
