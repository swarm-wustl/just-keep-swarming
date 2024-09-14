# Exit immediately if a command exits with a non-zero status
set -e

# Install Homebrew
if ! command -v brew &> /dev/null; then
    echo "Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
fi

# Install system packages
echo "Installing system packages..."
brew install clang-format xmlstarlet yamllint jq

# Install pipx if not already installed
if ! command -v pipx &> /dev/null; then
    echo "Installing pipx..."
    brew install pipx
    pipx ensurepath
    reset
fi

# Install Python packages
echo "Installing Python packages..."
pipx install black cmakelang cpplint

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



