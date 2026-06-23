#!/usr/bin/env bash
# Install the built plugin (+ profiles) into an X-Plane 12 installation.
#
# The X-Plane path is auto-detected (see scripts/find-xplane.sh). Override it
# by passing a path as the first argument or via the $XP environment variable:
#   ./scripts/install.sh "/path/to/X-Plane 12"
#   make install XP="/path/to/X-Plane 12"
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/build/Octavi"

if [ ! -f "$SRC/lin_x64/Octavi.xpl" ]; then
  echo "Plugin not built. Run 'make' first." >&2
  exit 1
fi

if ! XP="$("$ROOT/scripts/find-xplane.sh" "${1:-}")"; then
  echo "Could not locate an X-Plane 12 installation." >&2
  echo "Pass the path explicitly:" >&2
  echo "  make install XP=\"/path/to/X-Plane 12\"" >&2
  exit 1
fi

DST="$XP/Resources/plugins/Octavi"
mkdir -p "$DST"
cp -r "$SRC/lin_x64" "$DST/"
cp -r "$SRC/profiles" "$DST/"
echo "Installed to: $DST"
echo "  $DST/lin_x64/Octavi.xpl"
echo "  $DST/profiles/"
