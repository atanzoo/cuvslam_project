#!/bin/bash
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${PROJECT_DIR}"
exec /usr/bin/python3 tools/real_d435i_axis_web_gui.py --open-browser
