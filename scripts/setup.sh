#!/usr/bin/env bash
set -e

[ -d .venv ] && rm -rf .venv

python -m venv .venv

source .venv/Scripts/activate

python -m pip install -U pip
pip install -r requirements.txt
