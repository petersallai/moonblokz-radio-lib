#!/bin/bash
#
# MoonBlokz Node Build Script
#
# This script:
#   1. Builds the node binary for RP2040 (thumbv6m-none-eabi)
#   2. Converts ELF to UF2 format
#   3. Increments the version number
#   4. Calculates CRC32 of the UF2 file
#   5. Updates versioninfo/node/version.json
#   6. Copies UF2 to versioninfo/node with versioned filename
#
# Usage:
#   ./build.sh                    # Build and increment version
#   ./build.sh --no-increment     # Build without incrementing version
#

set -e

# Configuration
VERSION_FILE="versioninfo/node/version.json"
TARGET="thumbv6m-none-eabi"
BINARY_NAME="moonblokz-radio-embedded-test"
ELF_PATH="target/${TARGET}/release/${BINARY_NAME}"
UF2_PATH="target/${TARGET}/release/${BINARY_NAME}.uf2"

# Parse arguments
INCREMENT_VERSION=true

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-increment)
            INCREMENT_VERSION=false
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--no-increment]"
            echo ""
            echo "Options:"
            echo "  --no-increment  - Don't increment version number"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "========================================"
echo "MoonBlokz Node Build"
echo "========================================"
echo "Target: $TARGET"
echo ""

# Step 1: Build the binary
echo "[1/6] Building binary..."
cargo build --release --target "$TARGET"

if [ ! -f "$ELF_PATH" ]; then
    echo "ERROR: ELF binary not found at $ELF_PATH"
    exit 1
fi

echo "      Built: $ELF_PATH"

# Step 2: Convert ELF to UF2
echo "[2/6] Converting ELF to UF2..."

if ! command -v elf2uf2-rs &> /dev/null; then
    echo "ERROR: elf2uf2-rs not found. Install with: cargo install elf2uf2-rs"
    exit 1
fi

elf2uf2-rs "$ELF_PATH" "$UF2_PATH"

if [ ! -f "$UF2_PATH" ]; then
    echo "ERROR: UF2 file not found at $UF2_PATH"
    exit 1
fi

echo "      Created: $UF2_PATH"

# Step 3: Read current version and increment
echo "[3/6] Updating version info..."

if [ ! -f "$VERSION_FILE" ]; then
    echo "      Creating new version file"
    mkdir -p "$(dirname "$VERSION_FILE")"
    CURRENT_VERSION=0
else
    # Extract current version (handles both quoted and unquoted numbers)
    CURRENT_VERSION=$(grep -o '"version"[[:space:]]*:[[:space:]]*[0-9]*' "$VERSION_FILE" | grep -o '[0-9]*$' || echo "0")
    if [ -z "$CURRENT_VERSION" ]; then
        CURRENT_VERSION=0
    fi
fi

if [ "$INCREMENT_VERSION" = true ]; then
    NEW_VERSION=$((CURRENT_VERSION + 1))
    echo "      Version: $CURRENT_VERSION -> $NEW_VERSION"
else
    NEW_VERSION=$CURRENT_VERSION
    echo "      Version: $NEW_VERSION (not incremented)"
fi

# Step 4: Calculate CRC32 of UF2 file
echo "[4/6] Calculating CRC32..."

# Use crc32 command if available, otherwise use Python
if command -v crc32 &> /dev/null; then
    CRC32=$(crc32 "$UF2_PATH")
elif command -v python3 &> /dev/null; then
    CRC32=$(python3 -c "
import binascii
with open('$UF2_PATH', 'rb') as f:
    data = f.read()
print(format(binascii.crc32(data) & 0xffffffff, '08x'))
")
elif command -v python &> /dev/null; then
    CRC32=$(python -c "
import binascii
with open('$UF2_PATH', 'rb') as f:
    data = f.read()
print(format(binascii.crc32(data) & 0xffffffff, '08x'))
")
else
    echo "ERROR: No crc32 tool or Python available"
    exit 1
fi

echo "      CRC32: $CRC32"

# Step 5: Write updated version.json
echo "[5/6] Writing version.json..."

cat > "$VERSION_FILE" << EOF
{
  "version": $NEW_VERSION,
  "crc32": "$CRC32"
}
EOF

echo "      Updated: $VERSION_FILE"

# Step 6: Copy UF2 to versioninfo/node with versioned name
echo "[6/6] Copying UF2 to versioninfo/node..."

# Delete old versions
rm -f versioninfo/node/moonblokz_node_*.uf2

# Copy new version
VERSIONED_UF2="versioninfo/node/moonblokz_node_${NEW_VERSION}.uf2"
cp "$UF2_PATH" "$VERSIONED_UF2"
echo "      Copied to: $VERSIONED_UF2"

echo ""
echo "========================================"
echo "Build Summary"
echo "========================================"
echo "ELF Binary:    $ELF_PATH"
echo "UF2 File:      $UF2_PATH"
echo "Versioned UF2: $VERSIONED_UF2"
echo "Version:       $NEW_VERSION"
echo "CRC32:         $CRC32"
echo ""
echo "Version file updated: $VERSION_FILE"
echo ""

# Show file sizes
ELF_SIZE=$(ls -lh "$ELF_PATH" | awk '{print $5}')
UF2_SIZE=$(ls -lh "$UF2_PATH" | awk '{print $5}')
echo "ELF size: $ELF_SIZE"
echo "UF2 size: $UF2_SIZE"
