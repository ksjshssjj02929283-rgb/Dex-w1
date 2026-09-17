#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
CURDIR="$SCRIPT_DIR"

apt update -y && apt install -y python3.10-venv

echo "enter build"

echo "ensuring python build tooling ..."
pip3 install -U build setuptools wheel

echo "building python package ..."
cd "$CURDIR"
rm -rf "$CURDIR/dist"
python3 -m build --wheel --outdir "$CURDIR/dist"

echo "finish build"
