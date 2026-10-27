#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for directory in build install log; do
  target="${workspace_dir}/${directory}"
  if [ -d "${target}" ]; then
    rm -rf -- "${target}"
  fi
done
find "${workspace_dir}/src" -type d \
  \( -name __pycache__ -o -name .pytest_cache \) \
  -prune -exec rm -rf -- {} +
