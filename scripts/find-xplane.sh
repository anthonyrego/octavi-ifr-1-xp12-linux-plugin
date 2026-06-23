#!/usr/bin/env bash
# Print the path to an X-Plane 12 installation, or exit non-zero if none found.
#
# Resolution order:
#   1. explicit path passed as $1, or the $XP environment variable
#   2. X-Plane's own install record (~/.x-plane/x-plane_install_12.txt)
#   3. common Steam / native-installer locations
set -euo pipefail

is_xp() { [ -d "$1" ] && [ -d "$1/Resources/plugins" ]; }

# 1. explicit override (arg wins over env)
cand="${1:-${XP:-}}"
if [ -n "$cand" ]; then
  cand="${cand%/}"
  if is_xp "$cand"; then echo "$cand"; exit 0; fi
  echo "Specified X-Plane path is not a valid X-Plane 12 install: $cand" >&2
  exit 1
fi

# 2. X-Plane's install record (one install path per line)
rec="$HOME/.x-plane/x-plane_install_12.txt"
if [ -f "$rec" ]; then
  while IFS= read -r line; do
    line="${line%/}"
    [ -n "$line" ] || continue
    if is_xp "$line"; then echo "$line"; exit 0; fi
  done < "$rec"
fi

# 3. common install locations
for p in \
  "$HOME/.local/share/Steam/steamapps/common/X-Plane 12" \
  "$HOME/.steam/steam/steamapps/common/X-Plane 12" \
  "$HOME/.var/app/com.valvesoftware.Steam/data/Steam/steamapps/common/X-Plane 12" \
  "$HOME/X-Plane 12" \
  "/opt/X-Plane 12"; do
  if is_xp "$p"; then echo "$p"; exit 0; fi
done

exit 1
