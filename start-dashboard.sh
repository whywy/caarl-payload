#!/usr/bin/env bash
# Starts the CAARL payload dashboard and opens it in your browser.
cd "$(dirname "$0")" && exec python3 dashboard.py "$@"
