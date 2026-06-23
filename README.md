# Octavi IFR-1 — native X-Plane 12 driver for Linux

A standalone X-Plane 12 plugin (XPLM, written in C) that makes the
**Octavi IFR-1** fully usable on **Linux** — with **no FlyWithLua** and no
in-game overlay. The device is read directly over `hidraw`; inputs are mapped to
the sim through **per-aircraft INI profiles**. Ships with a tuned profile for the
default **Cessna 172 SP (analog)** plus a generic **`_default.ini`** fallback
that drives the stock avionics (COM/NAV/XPDR/heading/baro/OBS/autopilot) on any
aircraft that has no profile of its own — so the default GA fleet (172 SP &
seaplane, Baron 58, Cirrus SR22, …) works out of the box.

Octavi's official drivers are Windows/macOS only. The one existing Linux option
([cyberang3l/octavi-xplane-flywithlua](https://github.com/cyberang3l/octavi-xplane-flywithlua))
requires the FlyWithLua runtime and draws a visual overlay. This project is a
dependency-free reimplementation of the driver logic as a native plugin.

## Quick start

**Prebuilt download — no compiler needed (easiest):**

1. Download `Octavi-linux-x64.zip` from the
   [Releases](https://github.com/anthonyrego/octavi-ifr-1-xp12-linux-plugin/releases)
   page.
2. Extract it into your `X-Plane 12/Resources/plugins/` folder — you should end
   up with `…/plugins/Octavi/lin_x64/Octavi.xpl`.
3. One-time, if the knobs/LEDs don't respond: install the device-access rule
   (see [Device access](#device-access)).
4. Start X-Plane and load an aircraft.

**Build from source:**

```sh
make               # build the plugin (fetches the SDK headers on first run)
make install       # auto-detect X-Plane and install the plugin + profiles
make install-udev  # one-time: grant access to the device (uses sudo)
```

## How it works

The Octavi is a USB HID device (`04d8:e6d6`) that speaks a small custom protocol:
a modal **selector** (8 positions), a **primary/secondary** toggle (press the
centre knob), two relative-encoder knobs, a set of buttons, and writable
**LEDs**. The plugin polls the device every frame, decodes the report, and
dispatches each input to a dataref/command defined by the active aircraft's
profile. It also keeps the autopilot LEDs (AP/HDG/NAV/APR/ALT/VS) in sync.

```
device (hidraw) ──▶ decode ──▶ active function (selector + pri/sec)
                                     │
                                     ▼
                          aircraft profile (INI) ──▶ XPLM dataref/command
   autopilot state ──▶ LED bitmask ──▶ device (hidraw write)
```

## Requirements

- X-Plane 12 on **x86-64 Linux** (developed against 12.4; should work on any 12.x).
- The Octavi IFR-1 connected over USB.
- Read/write access to the device node (see *Device access* below).
- A C toolchain (`gcc`, `make`) — **only if building from source**; the prebuilt
  release needs no compiler.

## Build & install

```sh
make          # fetches the XPLM SDK headers (first run) and builds the .xpl
make install  # auto-detects X-Plane and copies the plugin + profiles in
```

`make install` finds your X-Plane 12 automatically — it reads X-Plane's own
install record (`~/.x-plane/x-plane_install_12.txt`) and falls back to the
common Steam / native-installer locations. If you keep X-Plane somewhere
unusual, point it there explicitly:

```sh
make install XP="/path/to/X-Plane 12"
```

If the device's knobs/LEDs don't respond, grant access to it once with
`make install-udev` (installs the udev rule below via `sudo`).

This installs to `…/X-Plane 12/Resources/plugins/Octavi/`:

```
Octavi/
├── lin_x64/Octavi.xpl
└── profiles/
    ├── _default.ini             # generic stock-avionics fallback (any GA aircraft)
    ├── Cessna_172SP.ini         # tuned profile - default 172 SP (analog)
    ├── Cessna_172SP_G1000.ini   # default 172 G1000 (glass - g1000n* FMS)
    └── C172_NG_ANALOG.ini       # AirfoilLabs C172 NG (study-level, custom cmds)
```

Start X-Plane, load the Cessna 172 SP (analog), and the device is live. Look for
`Octavi:` lines in `…/X-Plane 12/Log.txt` to confirm it found the device and
loaded the profile.

## Device access

The plugin needs read+write access to `/dev/hidrawN`. On many desktops,
game-controller-class HID devices already get `0666` automatically — check with:

```sh
# find the Octavi's hidraw node
for d in /sys/class/hidraw/hidraw*; do
  grep -q "04D8:0000E6D6" "$d/device/uevent" && echo "$(basename "$d")"; done
ls -l /dev/hidrawN   # want crw-rw-rw-
```

If it isn't writable, install the included udev rule. From a source checkout
that's just:

```sh
make install-udev
```

Or do it by hand (e.g. when using the prebuilt download — the rule ships in the
zip at `Octavi/udev/70-octavi.rules`):

```sh
sudo cp scripts/70-octavi.rules /etc/udev/rules.d/   # or Octavi/udev/70-octavi.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Verify the device without X-Plane

A standalone probe reuses the plugin's device code:

```sh
make probe
./build/octavi_probe        # prints decoded reports for ~5s
./build/octavi_probe led    # also cycles the LEDs to test the write path
```

## Control mapping (Cessna 172 SP analog)

Selector position + primary/secondary mode selects the **function**; the knobs
and buttons then act on it. Press the centre knob to toggle primary/secondary.

| Selector | Primary | Secondary |
|---|---|---|
| 1 | COM1 | HDG |
| 2 | COM2 | BARO |
| 3 | NAV1 | CRS1 |
| 4 | NAV2 | CRS2 |
| 5 | FMS1 | FMS1 |
| 6 | FMS2 | FMS2 |
| 7 | AP | AP |
| 8 | XPDR | XPDR mode |

- **COM/NAV**: large knob = MHz, small knob = kHz, **SHIFT** = swap active/standby.
- **HDG / CRS**: large = 10°, small = 1°.
- **BARO / XPDR mode**: either knob steps up/down.
- **XPDR**: large = first two octal digits, small = last two.
- **AP**: large = altitude, small = vertical speed; buttons AP/HDG/NAV/APR/ALT/VS
  toggle the autopilot modes; the LEDs reflect AP state.
- **FMS** (GNS530 = FMS1, GNS430 = FMS2): large = chapter, small = page, centre
  knob = cursor; buttons remap to CDI/OBS/MSG/FPL/VNAV/PROC/Direct/MENU/CLR/ENT;
  SHIFT+AP / SHIFT+HDG = zoom in / out.

## Adding another aircraft

Profiles are plain INI keyed by the aircraft `.acf` file name. The plugin loads
`profiles/<YourAcfBaseName>.ini` if it exists, otherwise it falls back to
`profiles/_default.ini` (the generic stock-avionics profile). So an aircraft only
needs its own file when the fallback isn't enough — typically a study-level plane
that drives its avionics through custom commands (see `profiles/C172_NG_ANALOG.ini`)
or a glass panel whose GPS/FMS isn't the stock GNS.

To make one, copy `profiles/_default.ini` to `profiles/<YourAcfBaseName>.ini` and
change the dataref/command strings to match that aircraft. No recompile needed —
restart X-Plane (or reload the aircraft) and the plugin loads the matching
profile (an exact-name match always wins over the fallback). Any dataref/command
that doesn't exist on the aircraft is logged and that single binding is skipped,
so partial profiles still work. See the recognised section keys in
`src/profile.c`.

## Troubleshooting

Everything the plugin does is logged to `…/X-Plane 12/Log.txt` with an `Octavi:`
prefix — that's the first place to look. Common cases:

- **Nothing happens in the sim.** Check `Log.txt` for `Octavi:` lines. No lines
  at all → the plugin didn't load (confirm `…/Resources/plugins/Octavi/lin_x64/Octavi.xpl`
  exists). A "device not found" line → see below.
- **Device not found.** Confirm it's connected: `lsusb` should list
  `04d8:e6d6`. If present but still not found, it's almost always permissions —
  run `make install-udev` (or install the udev rule by hand) and re-plug.
- **Knobs/buttons work but the LEDs never light** (or a permission-denied log
  line). The device node isn't writable → install the udev rule as above; the
  plugin needs read **and** write access.
- **Wrong or missing mappings for an aircraft.** The plugin loads
  `profiles/<AcfBaseName>.ini`, else `profiles/_default.ini`. `Log.txt` shows
  which profile it loaded and lists any bindings it skipped because the
  dataref/command doesn't exist on that aircraft. See *Adding another aircraft*.
- **`make install` can't find X-Plane.** Auto-detection failed (unusual install
  location); pass it explicitly: `make install XP="/path/to/X-Plane 12"`.

## Layout

```
src/        plugin source (plugin.c, hidraw.c, octavi.c, profile.c, log.h)
profiles/   per-aircraft INI profiles
tools/      octavi_probe.c — standalone device tester
scripts/    fetch-sdk.sh, install.sh, find-xplane.sh, install-udev.sh, 70-octavi.rules
.github/    CI: build check + prebuilt-release workflow
```

## Credits

Device protocol, dataref/command map, and overall approach are based on
[cyberang3l/octavi-xplane-flywithlua](https://github.com/cyberang3l/octavi-xplane-flywithlua).
This project reimplements that logic as a native, FlyWithLua-free plugin.
Licensed under the MIT License (see `LICENSE`).
