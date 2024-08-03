#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Update package lists
sudo apt update

# Install system packages
echo "Installing system packages..."
sudo apt install -y clang-format xmlstarlet yamllint jq

# Install pip if not already installed
if ! command -v pip &> /dev/null; then
    echo "Installing pip..."
    sudo apt install -y python3-pip
fi

# Upgrade pip
pip install --upgrade pip

# Install Python packages
echo "Installing Python packages..."
pip install black cmake_format cpplint

# Verify installations
echo "Verifying installations..."

command_exists() {
    command -v "$1" >/dev/null 2>&1
}

tools=("clang-format" "black" "cmake-format" "xmllint" "yamllint" "jq" "cpplint")

for tool in "${tools[@]}"; do
    if command_exists "$tool"; then
        echo "$tool is installed."
    else
        echo "Error: $tool is not installed. Please install it manually."
    fi
done

echo "Installation complete. Please ensure all tools are correctly installed."
