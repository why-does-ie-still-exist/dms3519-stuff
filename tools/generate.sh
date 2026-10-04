#!/usr/bin/env bash
# Regenerate the modm library (modm/) from project.xml, then the Bazel files for it.
set -euo pipefail
cd "$(dirname "$0")/.."
lbuild build
python3 tools/modm_bazel.py
