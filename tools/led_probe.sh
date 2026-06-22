#!/usr/bin/env bash
# Dump candidate autopilot/annunciator datarefs from a running X-Plane (NG loaded).
# Run it once per AP state (off / ALT armed / VS active / HDG / NAV / APR) and
# diff the columns to see which dataref tracks which annunciator.
#
#   tools/led_probe.sh            # one snapshot
#   watch -n1 tools/led_probe.sh  # live
set -euo pipefail
BASE=http://localhost:8086/api/v2

# Build name->id maps once.
map=$(curl -s "$BASE/datarefs" | python3 -c '
import sys,json
d=json.load(sys.stdin)["data"]
print("\n".join(f"{x[\"id\"]}\t{x[\"name\"]}" for x in d))')

want='
sim/cockpit/autopilot/autopilot_state
sim/cockpit/autopilot/autopilot_mode
sim/cockpit2/autopilot/servos_on
sim/cockpit2/autopilot/flight_director_mode
sim/cockpit2/autopilot/heading_status
sim/cockpit2/autopilot/nav_status
sim/cockpit2/autopilot/approach_status
sim/cockpit2/autopilot/altitude_hold_status
sim/cockpit2/autopilot/vvi_status
sim/cockpit2/autopilot/altitude_mode
'
# Plus every KAP140 dataref the aircraft exposes at runtime.
kap=$(printf '%s\n' "$map" | grep -iE 'KAP140' | awk '{print $2}' || true)

read_one() {
  local name="$1"
  local id
  id=$(printf '%s\n' "$map" | awk -v n="$name" '$2==n{print $1; exit}')
  if [ -z "$id" ]; then printf '%-55s  (no dref)\n' "$name"; return; fi
  local v
  v=$(curl -s "$BASE/datarefs/$id/value" | python3 -c 'import sys,json; print(json.load(sys.stdin).get("data"))' 2>/dev/null || echo '?')
  printf '%-55s  %s\n' "$name" "$v"
}

echo "== stock autopilot =="
for n in $want; do read_one "$n"; done
echo "== KAP140 ($(printf '%s\n' "$kap" | grep -c . || true) drefs) =="
for n in $kap; do read_one "$n"; done
