/*
 * plugin.c - X-Plane 12 XPLM entry points for the Octavi IFR-1 driver.
 *
 * Polls the device each frame, dispatches inputs to the active aircraft profile,
 * and keeps the autopilot LEDs in sync. No external runtime dependencies.
 */
#include <stdint.h>
#include <string.h>

#include <XPLMDefs.h>
#include <XPLMPlanes.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>

#include "hidraw.h"
#include "log.h"
#include "octavi.h"
#include "profile.h"

static int           g_fd = -1;
static int           g_is_primary = 1;
static octavi_report g_prev;          /* last processed report (for edges) */
static int           g_last_led = -1;
static int           g_profile_loaded = 0;
static float         g_reopen_acc = 0.0f;

/* Strip the last '/'-delimited component of a path, in place. */
static void dirname_inplace(char *p) {
  char *slash = strrchr(p, '/');
  if (slash) *slash = '\0';
  else p[0] = '\0';
}

static void try_open_device(void) {
  char path[128] = "";
  g_fd = hidraw_open_octavi(OCTAVI_VENDOR_ID, OCTAVI_PRODUCT_ID, path, sizeof path);
  if (g_fd >= 0) {
    octavi_log("device found on %s", path);
    /* Trigger an initial report so we learn the current selector position.
     * The first read's knob bytes are garbage and are filtered by octavi_decode. */
    unsigned char trig[8] = { 0, 0, 0, 0, 0, 0, 0, 0xff };
    hidraw_write(g_fd, trig, sizeof trig);
    g_last_led = -1;
    memset(&g_prev, 0, sizeof g_prev);
  }
}

static void device_lost(void) {
  octavi_log("device read error - assuming unplugged, will retry");
  hidraw_close(g_fd);
  g_fd = -1;
  g_last_led = -1;
}

static void reload_profile(void) {
  char acf[256] = "", path[512] = "";
  XPLMGetNthAircraftModel(0, acf, path); /* index 0 = user aircraft */
  if (acf[0]) profile_load(acf);
  g_profile_loaded = 1;
}

static float flight_loop(float elapsed, float since_loop, int counter, void *ref) {
  (void)since_loop; (void)counter; (void)ref;

  if (!g_profile_loaded) reload_profile();

  if (g_fd < 0) {
    g_reopen_acc += elapsed;
    if (g_reopen_acc >= 2.0f) { g_reopen_acc = 0.0f; try_open_device(); }
    return -1.0f;
  }

  unsigned char buf[64];
  for (int i = 0; i < 32; i++) { /* drain any pending reports this frame */
    int n = hidraw_read(g_fd, buf, sizeof buf);
    if (n == 0) break;
    if (n < 0) { device_lost(); break; }

    octavi_report cur;
    if (!octavi_decode(buf, n, &cur)) continue;

    /* Knob press toggles primary/secondary on the rising edge. */
    if (cur.knob && !g_prev.knob) g_is_primary = !g_is_primary;

    int fn = octavi_function_id(cur.selector, g_is_primary);
    profile_dispatch(fn, &cur, &g_prev);
    g_prev = cur;
  }

  /* Keep the device LEDs in sync with the autopilot state. */
  int led = profile_led_value();
  if (led >= 0 && led != g_last_led) {
    unsigned char out[2] = { OCTAVI_REPORT_ID, (unsigned char)led };
    hidraw_write(g_fd, out, 2);
    g_last_led = led;
  }

  return -1.0f; /* call again next frame */
}

PLUGIN_API int XPluginStart(char *name, char *sig, char *desc) {
  strcpy(name, "Octavi IFR-1");
  strcpy(sig,  "ai.getaide.octavi.ifr1");
  strcpy(desc, "Native Octavi IFR-1 driver for X-Plane 12 on Linux (no FlyWithLua).");

  /* Locate the profiles directory that sits next to the .xpl:
   * .../plugins/Octavi/lin_x64/Octavi.xpl -> .../plugins/Octavi/profiles */
  char filepath[1024] = "";
  XPLMGetPluginInfo(XPLMGetMyID(), NULL, filepath, NULL, NULL);
  dirname_inplace(filepath); /* drop Octavi.xpl   -> .../Octavi/lin_x64 */
  dirname_inplace(filepath); /* drop lin_x64      -> .../Octavi         */
  char profiles[1200];
  snprintf(profiles, sizeof profiles, "%s/profiles", filepath);
  profile_set_dir(profiles);
  octavi_log("profiles directory: %s", profiles);

  try_open_device();
  if (g_fd < 0)
    octavi_log("device not found at startup; will keep retrying (is it plugged in?)");

  XPLMRegisterFlightLoopCallback(flight_loop, -1.0f, NULL);
  octavi_log("started");
  return 1;
}

PLUGIN_API void XPluginStop(void) {
  XPLMUnregisterFlightLoopCallback(flight_loop, NULL);
  if (g_fd >= 0) {
    unsigned char off[2] = { OCTAVI_REPORT_ID, 0x00 }; /* LEDs off on exit */
    hidraw_write(g_fd, off, 2);
    hidraw_close(g_fd);
    g_fd = -1;
  }
}

PLUGIN_API int  XPluginEnable(void)  { return 1; }
PLUGIN_API void XPluginDisable(void) { }

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int msg, void *param) {
  (void)from;
  /* On user-aircraft change, reload the matching profile on the next frame
   * (by which point the new aircraft's datarefs are registered). */
  if (msg == XPLM_MSG_PLANE_LOADED && (intptr_t)param == 0)
    g_profile_loaded = 0;
}
