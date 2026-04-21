#!/usr/bin/env bash

set -e -u -o pipefail

UF2="/Users/stephenchung/mf-git/actual/pokemon/build/pokemon.uf2"
VOLUME="/Volumes/RPI-RP2"

# 1) Find Pico CDC device (stdio over USB shows up as usbmodem*)
PORT="$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1 || true)"
if [[ -n "${PORT}" ]]; then
  echo "Found Pico serial at ${PORT}; requesting BOOTSEL..."
  # Send single-byte 'r' (no newline)
  printf 'r' > "${PORT}" || true
else
  echo "No /dev/cu.usbmodem* found; assuming already in BOOTSEL."
fi


# 2) Wait for BOOTSEL drive to appear
echo "Waiting for ${VOLUME}..."
for _ in {1..50}; do
  [[ -d "${VOLUME}" ]] && break
  sleep 0.1
done

if [[ ! -d "${VOLUME}" ]]; then
  echo "BOOTSEL volume not found at ${VOLUME}."
  echo "If needed: hold BOOTSEL while plugging in once, then rerun."
  exit 1
fi

# 3) Copy UF2
echo "Copying ${UF2} -> ${VOLUME}/"
cp "${UF2}" "${VOLUME}/"
echo "Done."