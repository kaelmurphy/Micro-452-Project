#!/usr/bin/env bash
set -e
PY=$(command -v python3 || command -v python)
[ -z "$PY" ] && { echo "Python not found"; exit 1; }
[ -d .venv ] || "$PY" -m venv .venv
source .venv/Scripts/activate 2>/dev/null || source .venv/bin/activate
python -m pip install -U pip
[ -f requirements.txt ] && pip install -r requirements.txt
echo "venv ready. activate later with: source .venv/Scripts/activate"
