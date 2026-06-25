# Octavi IFR-1 — native X-Plane 12 plugin (Linux)

Native XPLM C plugin that maps the Octavi IFR-1 USB controller to X-Plane 12
commands/datarefs. Reads the device over hidraw; behaviour per aircraft is
driven by `.ini` profiles, not hard-coded. No external runtime deps (no
FlyWithLua). See `README.md` for the user-facing overview.

## Build / install / reload — IMPORTANT

`make` only compiles and stages into `build/Octavi/` — it does **not** touch the
running sim. X-Plane loads the plugin and profiles from its own plugins folder:

```
$XP/Resources/plugins/Octavi/{lin_x64/Octavi.xpl, profiles/}
```

So after editing C **or** a profile `.ini`, the full loop is:

1. `make install` — builds, then copies `.xpl` + `profiles/` into X-Plane
   (`scripts/install.sh`; X-Plane path auto-detected by `scripts/find-xplane.sh`,
   override with `make install XP="/path/to/X-Plane 12"`).
2. **Reload in the sim** — Plugins → Plugin Admin → Reload Plugins, or restart
   X-Plane. The plugin and the profile `.ini` are both read at load time, so a
   running session keeps the old behaviour until reloaded.

Gotcha that looks like "my change did nothing": you ran `make` (staged only) or
`make install` but didn't reload — the sim is still on the previously installed
copy. Verify with `ls -la "$XP/Resources/plugins/Octavi/lin_x64/Octavi.xpl"` and
by grepping the installed profile, not the repo one.

Second gotcha: `make` only re-copies `profiles/` into `build/` when a **source**
file changed (the copy lives in the `$(OUT)` recipe). After a profile-only edit,
a plain `make install` can ship the **stale** ini. Force it with `touch
src/profile.c && make install` (or `make clean && make install`), then grep the
installed ini to confirm.

Other targets: `make` (build only), `make probe` (host-side device tester, no
X-Plane), `make install-udev` (HID device permissions, uses sudo), `make clean`.

## Live diagnosis via X-Plane's UDP API — powerful

When X-Plane is running it listens on **UDP 49000** (the classic data API). From
the shell you can read/write datarefs and fire commands **live**, so you can
verify behaviour yourself instead of asking the user to watch a dataref viewer.
Check it's up with `ss -ulpn | grep 49000`.

Three message types (5-byte ASCII header + null, then a struct):
- **`RREF`** — subscribe to a dataref: `b"RREF\0"+pack("<ii",freq,id)+path` padded
  to 400 B. X-Plane streams back `b"RREF"`+repeating `(int id, float value)`
  8-byte records to your socket's port. Read any dataref's live value.
- **`DREF`** — write: `b"DREF\0"+pack("<f",value)+path` padded to 500 B.
- **`CMND`** — fire a command (≈ `XPLMCommandOnce`): `b"CMND\0"+path`.

This is how the GTX328 transponder was reverse-engineered: `RREF` showed the unit
mirrors its code **out** to `sim/cockpit/radios/transponder_code` (reads work); a
`DREF` write *stuck in the dref but did not drive the rendered display*; `CMND` on
`C172/cockpit/GTX328/key0..7` did set the code; and a `CMND` speed sweep proved
the code register accepts presses at any rate (down to a 0 s gap).

Key caveat: for **study-level / plugin-rendered instruments, a `DREF` write can
sit in the dref without driving the instrument** — reads are reliable, writes may
be cosmetic. Always confirm the actual display/behaviour, not just the dref echo.
Reusable RREF/DREF/CMND snippets were used in this work; rebuild from the formats
above (no fixed location — they were session scratch files).

## Architecture

- `src/octavi.{c,h}` — decode the device's HID report into an `octavi_report`
  (knob deltas, button edges, selected function, shift state).
- `src/hidraw.{c,h}` — open/read the hidraw device.
- `src/plugin.c` — XPLM entry points; per-frame loop reads the device and calls
  `profile_dispatch`; loads the matching profile for the current aircraft.
- `src/profile.c` — the INI profile engine: parsing, binding model, dispatch,
  LED sync. This is where almost all behaviour lives.
- `profiles/*.ini` — one per aircraft, matched by `.acf` filename;
  `_default.ini` is the fallback for unmatched aircraft.

## Profile model (src/profile.c)

Each `[SECTION]` (COM1, NAV1, AP, FMS1, HDG, …) binds a function to one of a few
handler **kinds**: `K_FREQ`, `K_WRAP360`, `K_KNOBCMD`, `K_OCTAL`, `K_AP`,
`K_FMS`, `K_CMDMAP`, `K_XPDRKEYS`. Stock-dataref aircraft write datarefs directly;
study-level aircraft (e.g. C172 NG analog) use `type = cmd` (`K_CMDMAP`) to fire
the aircraft's own commands per input.

Notes for editing profiles:
- **Press-and-hold buttons**: study-level button manipulators ignore an
  instantaneous `XPLMCommandOnce` — they need begin→hold→end. `cmd_down/up`
  (real button press) and `cmd_pulse` (one held pulse from a knob detent) handle
  this; `HOLD_MIN`/`HOLD_MAX` bound the hold.
- **Knob commands**, in `K_CMDMAP`, fire via `fire_knob()`: a single-activation
  detent uses `Once`; set `large_hold = 1` / `small_hold = 1` on the section to
  pulse that knob's commands instead (for knob roles wired to press-and-hold
  buttons, e.g. the KAP140 UP/DN keys driving vertical speed).
- **SHIFT+rotate override** (any kind, handled before the switch in
  `profile_dispatch`): while the `<->` shift button is held, the knobs fire
  `shift_large_inc/dec` + `shift_small_inc/dec` instead and the normal rotation
  is consumed (so the default action doesn't also fire). `shift_hold = 1` pulses
  them via `fire_knob()` for press-and-hold targets. Use this for a momentary
  alternate function on the dials — e.g. C172 NG `[AP]`: dials = altitude
  preselect, SHIFT+dials = vertical speed; HDG: bug normally, DG drift on SHIFT.
- **`cond` override**: a section can gate the knobs on a dataref condition
  (`cond = <dref> [op num] && …`) and fire `cond_*` commands while it holds.
  Prefer a fixed mapping when the behaviour shouldn't depend on sim state.
- **Keypad transponder** (`type = keys`, `K_XPDRKEYS`): for a GTX328-style unit
  that has no rotary code command and ignores writes to the stock squawk dref —
  the code is only settable by entering all four digits, in order, on its keypad.
  The handler reads the current code from `dref` (the unit mirrors its code out),
  computes the new one (bottom dial = first two octal digits, top = last two, each
  pair wrapping within itself), and "types" the full 4-digit code via `key0`–`key7`
  in `profile_tick`, one digit per `key_interval` seconds (`0.0` = one per frame).
  Spinning fast just re-types the final code (mid-entry target changes coalesce).
- **LEDs** (`[leds]`): per-LED condition form `key = <dref> [op num] [&& …]`
  (keys: ap hdg nav apr alt vs). Needed for autopilots like the KAP140 that
  don't follow X-Plane's stock `autopilot_state` bit layout.
