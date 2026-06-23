#!/usr/bin/env bash
# Install the udev rule that grants the logged-in user read/write access to the
# Octavi IFR-1 HID device (needed to read inputs and drive the LEDs), then reload
# udev so it takes effect. Re-runs itself under sudo if not already root.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RULE="$ROOT/scripts/70-octavi.rules"
DST="/etc/udev/rules.d/70-octavi.rules"

if [ "$(id -u)" -ne 0 ]; then
  echo "Installing the udev rule needs root — re-running with sudo..."
  exec sudo "$0" "$@"
fi

cp "$RULE" "$DST"
udevadm control --reload-rules
udevadm trigger
echo "Installed udev rule: $DST"
echo "Re-plug the Octavi if it's connected — its /dev/hidraw* node will be read/writable."
