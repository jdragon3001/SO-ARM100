#!/bin/zsh
set -eu
cd "${0:A:h}"
if command -v uv >/dev/null 2>&1; then
  uv run python Simulation/Workbench/app.py
else
  print 'Install uv first: https://docs.astral.sh/uv/getting-started/installation/'
  read '?Press return to close'
fi
