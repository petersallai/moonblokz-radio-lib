#!/usr/bin/env bash
set -euo pipefail

# Destination mount point for the RP2040 UF2 copy. Override by exporting DEST or
# passing DEST in the environment when invoking this script. Defaults to the
# standard RP2040 boot volume on macOS.
DEST="${DEST:-/Volumes/RPI-RP2}"

echo "[1/3] Building (release)..."
cargo build --release --target thumbv6m-none-eabi
echo "Build succeeded."

echo "[2/3] Converting ELF -> UF2..."
elf2uf2-rs target/thumbv6m-none-eabi/release/moonblokz-radio-embedded-test
echo "UF2 generated."

echo "[3/3] Copying UF2 to mounted RP2040 volume..."
UF2=target/thumbv6m-none-eabi/release/moonblokz-radio-embedded-test.uf2
if [ -d "${DEST}" ]; then
	cp "$UF2" "$DEST"/
	echo "Deploy complete (copied UF2)."
else
	echo "Skip copy: $DEST not available." >&2
fi