#!/usr/bin/env bash
# Publishes build/site (the page and the C++ results) to the gh-pages branch of origin.
# Run scripts/build.sh first. GitHub Pages then serves the branch; enable it once in
# Settings -> Pages -> Deploy from a branch -> gh-pages, / (root).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SITE="$ROOT/build/site"
[ -f "$SITE/index.html" ] || { echo "build/site is missing: run scripts/build.sh first"; exit 1; }
REMOTE="$(git -C "$ROOT" remote get-url origin)"
TMP="$(mktemp -d)"
cp -r "$SITE"/. "$TMP"/
cd "$TMP"
git init -q -b gh-pages
git add -A
git -c user.name="${GIT_AUTHOR_NAME:-Claude}" -c user.email="${GIT_AUTHOR_EMAIL:-noreply@anthropic.com}" \
  commit -q -m "Publish line follower simulator and C++ results from $(git -C "$ROOT" rev-parse --short HEAD)"
git push -q -f "$REMOTE" gh-pages:gh-pages
echo "published to $REMOTE (gh-pages)"
