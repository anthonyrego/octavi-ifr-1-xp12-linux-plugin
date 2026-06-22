#!/usr/bin/env python3
"""Dump candidate autopilot/annunciator datarefs from a running X-Plane.

Run once per AP state (off / AP-on / VS-active / HDG / NAV / APR); diff the
output to see which dataref tracks which annunciator.
"""
import json, sys, urllib.request

BASE = "http://localhost:8086/api/v2"

def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.load(r)

drefs = get("/datarefs")["data"]
by_name = {d["name"]: d["id"] for d in drefs}

WANT = [
    "sim/cockpit/autopilot/autopilot_state",
    "sim/cockpit/autopilot/autopilot_mode",
    "sim/cockpit2/autopilot/servos_on",
    "sim/cockpit2/autopilot/flight_director_mode",
    "sim/cockpit2/autopilot/heading_status",
    "sim/cockpit2/autopilot/nav_status",
    "sim/cockpit2/autopilot/approach_status",
    "sim/cockpit2/autopilot/altitude_hold_status",
    "sim/cockpit2/autopilot/vvi_status",
    "sim/cockpit2/autopilot/altitude_mode",
]
kap = sorted(n for n in by_name if "KAP140" in n)

def read(name):
    did = by_name.get(name)
    if did is None:
        return "(no dref)"
    try:
        return get(f"/datarefs/{did}/value")["data"]
    except Exception as e:
        return f"ERR {e}"

print("== stock autopilot ==")
for n in WANT:
    print(f"{n:<54}  {read(n)}")
print(f"== KAP140 ({len(kap)} drefs) ==")
for n in kap:
    print(f"{n:<54}  {read(n)}")
