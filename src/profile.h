/*
 * profile.h - per-aircraft INI profiles + the input-to-sim dispatch.
 *
 * A profile is an INI file (e.g. profiles/Cessna_172SP.ini) whose section names
 * are Octavi functions (COM1, NAV1, AP, FMS1, HDG, BARO, XPDR, MODE, ...). Each
 * section supplies the dataref / command strings (and numeric params) this
 * aircraft uses; the per-function *behaviour* lives in C (resolved into XPLM
 * handles at load time). Missing datarefs/commands are logged and that one
 * binding is disabled, so a partial profile still works.
 */
#ifndef PROFILE_H
#define PROFILE_H

#include "octavi.h"

/* Directory to look for "<acf-basename>.ini" in (the plugin's profiles dir). */
void profile_set_dir(const char *dir);

/* Load the profile matching the given .acf filename (e.g. "Cessna_172SP.acf").
 * Resolves all datarefs/commands. Returns 1 if a profile file was found and
 * parsed, 0 otherwise. Safe to call repeatedly (e.g. on aircraft change). */
int profile_load(const char *acf_filename);

/* Name of the loaded profile file, or NULL if none. */
const char *profile_name(void);

/* Dispatch one decoded report against the active function's bindings.
 * `prev` is the previously processed report (for button edge detection). */
void profile_dispatch(int fn_id, const octavi_report *cur, const octavi_report *prev);

/* Current LED bitmask computed from the profile's [leds] datarefs, or -1 if the
 * profile has no usable [leds] section. */
int profile_led_value(void);

/* Advance held-command timers; call once per frame with the elapsed seconds.
 * Some aircraft (e.g. study-level) ignore an instantaneous command and need a
 * brief press-and-hold; profile_dispatch begins such commands and this ends
 * them a few ms later (self-releasing, independent of the active function). */
void profile_tick(float dt);

#endif /* PROFILE_H */
