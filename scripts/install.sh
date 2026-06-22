#!/usr/bin/env bash
# Install the built plugin (+ profiles) into an X-Plane 12 installation.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
XP="${1:-/home/rego/.local/share/Steam/steamapps/common/X-Plane 12}"
SRC="$ROOT/build/Octavi"
DST="$XP/Resources/plugins/Octavi"

if [ ! -d "$XP" ]; then
  echo "X-Plane not found: $XP" >&2
  echo "Usage: $0 [path-to-X-Plane-12]" >&2
  exit 1
fi
if [ ! -f "$SRC/lin_x64/Octavi.xpl" ]; then
  echo "Plugin not built. Run 'make' first." >&2
  exit 1
fi

mkdir -p "$DST"
cp -r "$SRC/lin_x64" "$DST/"
cp -r "$SRC/profiles" "$DST/"
echo "Installed to: $DST"
echo "  $DST/lin_x64/Octavi.xpl"
echo "  $DST/profiles/"
