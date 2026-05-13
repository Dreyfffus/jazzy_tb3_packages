#!/usr/bin/env bash

# Guard: must be sourced, not called
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Error: source this script, don't call it:"
    echo "  source ${BASH_SOURCE[0]}"
    exit 1
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SCRIPTS_DIR=$(dirname "$SCRIPT_DIR")
BIN_DIR="$SCRIPTS_DIR/bin"

# Make all scripts executable
chmod +x "$BIN_DIR"/*

# Add to current session immediately
export PATH="$BIN_DIR:$PATH"

# Persist for future sessions
if ! grep -q "$BIN_DIR" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# Added by setup.bash" >> "$HOME/.bashrc"
    echo "export PATH=\"$BIN_DIR:\$PATH\"" >> "$HOME/.bashrc"
    echo "Added $BIN_DIR to ~/.bashrc"
else
    echo "$BIN_DIR already in ~/.bashrc, skipping"
fi

echo "Done! $BIN_DIR is now in your PATH"
