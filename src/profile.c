/* profile.c - see profile.h */
#include "profile.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <XPLMDataAccess.h>
#include <XPLMUtilities.h>

/* ---- binding model ------------------------------------------------------- */

typedef enum { K_NONE, K_FREQ, K_WRAP360, K_KNOBCMD, K_OCTAL, K_AP, K_FMS,
               K_CMDMAP } kind_t;

/* FMS command slots. */
enum { F_CHAP_UP, F_CHAP_DN, F_PAGE_UP, F_PAGE_DN, F_CURSOR, F_CDI, F_OBS,
       F_MSG, F_FPL, F_VNAV, F_PROC, F_DIRECT, F_MENU, F_CLR, F_ENT,
       F_ZOOM_IN, F_ZOOM_OUT, F_NSLOTS };

typedef struct {
  kind_t kind;

  /* K_FREQ */
  XPLMDataRef    freq_dref;
  double         coarse_min, coarse_max, fine_step;
  XPLMCommandRef flip_cmd;
  XPLMDataRef    sel_dref;       /* g430_nav_com_sel quirk (optional) */
  int            sel_idx, sel_want, sel_have;
  XPLMCommandRef sel_cmd;

  /* K_WRAP360 */
  XPLMDataRef    dir_dref;
  double         coarse_step, dir_fine_step;

  /* K_KNOBCMD */
  XPLMCommandRef cmd_up, cmd_down;

  /* K_OCTAL */
  XPLMDataRef    oct_dref;

  /* K_AP */
  XPLMDataRef    alt_dref, vs_dref;
  double         alt_step, vs_step;
  XPLMCommandRef ap_btn[6];      /* AP,HDG,NAV,APR,ALT,VS */

  /* K_FMS */
  XPLMCommandRef fms[F_NSLOTS];

  /* K_CMDMAP (type = cmd): every input fires a command. For study-level
   * aircraft whose knobs/buttons are their own commands (not stock datarefs). */
  XPLMCommandRef m_large_inc, m_large_dec, m_small_inc, m_small_dec;
  XPLMCommandRef m_knob, m_shift;
  XPLMCommandRef m_btn_ap, m_btn_hdg, m_btn_nav, m_btn_apr, m_btn_alt, m_btn_vs;
  XPLMCommandRef m_btn_d, m_btn_menu, m_btn_clr, m_btn_ent;

  /* Optional conditional knob override: when cond_dref is non-zero, the knobs
   * pulse these commands instead (e.g. KAP140 alt knob -> VS up/dn in VS mode). */
  XPLMDataRef    c_cond_dref;
  XPLMCommandRef c_large_inc, c_large_dec, c_small_inc, c_small_dec;

  /* Optional SHIFT+rotate override (any function kind): while SHIFT is held the
   * knobs fire these instead (e.g. HDG bug normally, DG drift adjust with SHIFT). */
  XPLMCommandRef sh_large_inc, sh_large_dec, sh_small_inc, sh_small_dec;
} func_binding;

static func_binding g_bind[FN_COUNT];
static XPLMDataRef  g_led_ap_state;
static XPLMDataRef  g_led_approach;
static char         g_dir[512];
static char         g_loaded[320];

/* Press-and-hold support. Study-level aircraft model their buttons as
 * press-and-hold manipulators (ATTR_manip_command hand: CommandBegin on press,
 * CommandEnd on release) and ignore an instantaneous XPLMCommandOnce. So we
 * mirror the physical Octavi button: cmd_down() on press, cmd_up() on release.
 * The command is held for at least HOLD_MIN (so a quick tap still registers)
 * and for as long as the button is physically down, up to HOLD_MAX (a backstop
 * so a missed release - e.g. the function changed mid-press - can't stick). */
#define MAX_HELD 16
#define HOLD_MIN 0.18f
#define HOLD_MAX 4.0f
static struct { XPLMCommandRef cmd; float age; int down; } g_held[MAX_HELD];

static void cmd_down(XPLMCommandRef c) {
  if (!c) return;
  for (int i = 0; i < MAX_HELD; i++)             /* re-press while still held */
    if (g_held[i].cmd == c) { g_held[i].down = 1; g_held[i].age = 0.0f; return; }
  XPLMCommandBegin(c);
  for (int i = 0; i < MAX_HELD; i++)
    if (!g_held[i].cmd) { g_held[i].cmd = c; g_held[i].age = 0.0f; g_held[i].down = 1; return; }
  XPLMCommandEnd(c);                             /* table full: don't get stuck */
}

static void cmd_up(XPLMCommandRef c) {
  if (!c) return;
  for (int i = 0; i < MAX_HELD; i++)
    if (g_held[i].cmd == c) { g_held[i].down = 0; return; }
}

/* One held pulse (begin, auto-release after HOLD_MIN). For driving a
 * press-and-hold button command from a momentary knob detent, which has no
 * physical release of its own. Repeated detents within the window refresh it. */
static void cmd_pulse(XPLMCommandRef c) {
  if (!c) return;
  for (int i = 0; i < MAX_HELD; i++)
    if (g_held[i].cmd == c) { g_held[i].age = 0.0f; g_held[i].down = 0; return; }
  XPLMCommandBegin(c);
  for (int i = 0; i < MAX_HELD; i++)
    if (!g_held[i].cmd) { g_held[i].cmd = c; g_held[i].age = 0.0f; g_held[i].down = 0; return; }
  XPLMCommandEnd(c);
}

void profile_tick(float dt) {
  for (int i = 0; i < MAX_HELD; i++) {
    if (!g_held[i].cmd) continue;
    g_held[i].age += dt;
    if ((g_held[i].age >= HOLD_MIN && !g_held[i].down) || g_held[i].age >= HOLD_MAX) {
      XPLMCommandEnd(g_held[i].cmd);
      g_held[i].cmd = NULL;
    }
  }
}

/* ---- resolution helpers -------------------------------------------------- */

static XPLMDataRef find_dref(const char *name) {
  if (!name || !*name) return NULL;
  XPLMDataRef r = XPLMFindDataRef(name);
  if (!r) octavi_log("dataref not found (binding disabled): %s", name);
  return r;
}

static XPLMCommandRef find_cmd(const char *name) {
  if (!name || !*name) return NULL;
  XPLMCommandRef c = XPLMFindCommand(name);
  if (!c) octavi_log("command not found (binding disabled): %s", name);
  return c;
}

/* ---- tiny INI parser ----------------------------------------------------- */

#define MAX_KV 64
typedef struct { char key[48]; char val[192]; } kv_pair;

static char *trim(char *s) {
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  char *e = s + strlen(s);
  while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n'))
    *--e = '\0';
  return s;
}

static const char *kv_get(kv_pair *kv, int n, const char *key) {
  for (int i = 0; i < n; i++)
    if (strcasecmp(kv[i].key, key) == 0) return kv[i].val;
  return NULL;
}

static const char *kv_def(kv_pair *kv, int n, const char *key, const char *def) {
  const char *v = kv_get(kv, n, key);
  return v ? v : def;
}

/* Map a section name to its function id and handler kind. Returns -1 if the
 * section is not a function (e.g. "meta", "leds"). */
static int section_fn(const char *name, kind_t *kind) {
  static const struct { const char *n; int fn; kind_t k; } tbl[] = {
    {"COM1", FN_COM1, K_FREQ},    {"COM2", FN_COM2, K_FREQ},
    {"NAV1", FN_NAV1, K_FREQ},    {"NAV2", FN_NAV2, K_FREQ},
    {"HDG",  FN_HDG,  K_WRAP360}, {"CRS1", FN_CRS1, K_WRAP360},
    {"CRS2", FN_CRS2, K_WRAP360}, {"BARO", FN_BARO, K_KNOBCMD},
    {"MODE", FN_MODE, K_KNOBCMD}, {"XPDR", FN_XPDR, K_OCTAL},
    {"AP",   FN_AP,   K_AP},      {"FMS1", FN_FMS1, K_FMS},
    {"FMS2", FN_FMS2, K_FMS},
  };
  for (size_t i = 0; i < sizeof tbl / sizeof tbl[0]; i++)
    if (strcasecmp(name, tbl[i].n) == 0) { *kind = tbl[i].k; return tbl[i].fn; }
  return -1;
}

static void build_section(const char *section, kv_pair *kv, int nkv) {
  if (strcasecmp(section, "leds") == 0) {
    g_led_ap_state = find_dref(kv_get(kv, nkv, "ap_state"));
    g_led_approach = find_dref(kv_get(kv, nkv, "approach"));
    return;
  }
  if (strcasecmp(section, "meta") == 0) return;

  kind_t kind;
  int fn = section_fn(section, &kind);
  if (fn < 0) { octavi_log("unknown profile section: [%s]", section); return; }

  /* "type = cmd" overrides the name-based default with the generic command map. */
  const char *type = kv_get(kv, nkv, "type");
  if (type && strcasecmp(type, "cmd") == 0) kind = K_CMDMAP;

  func_binding *b = &g_bind[fn];
  memset(b, 0, sizeof *b);
  b->kind = kind;

  switch (kind) {
  case K_FREQ:
    b->freq_dref  = find_dref(kv_get(kv, nkv, "freq_dref"));
    b->coarse_min = atof(kv_def(kv, nkv, "coarse_min", "0"));
    b->coarse_max = atof(kv_def(kv, nkv, "coarse_max", "0"));
    b->fine_step  = atof(kv_def(kv, nkv, "fine_step", "1"));
    b->flip_cmd   = find_cmd(kv_get(kv, nkv, "flip_cmd"));
    if (kv_get(kv, nkv, "sel_dref")) {
      b->sel_dref = find_dref(kv_get(kv, nkv, "sel_dref"));
      b->sel_idx  = atoi(kv_def(kv, nkv, "sel_idx", "0"));
      b->sel_want = atoi(kv_def(kv, nkv, "sel_want", "0"));
      b->sel_cmd  = find_cmd(kv_get(kv, nkv, "sel_cmd"));
      b->sel_have = (b->sel_dref && b->sel_cmd);
    }
    break;
  case K_WRAP360:
    b->dir_dref      = find_dref(kv_get(kv, nkv, "dref"));
    b->coarse_step   = atof(kv_def(kv, nkv, "coarse_step", "10"));
    b->dir_fine_step = atof(kv_def(kv, nkv, "fine_step", "1"));
    break;
  case K_KNOBCMD:
    b->cmd_up   = find_cmd(kv_get(kv, nkv, "cmd_up"));
    b->cmd_down = find_cmd(kv_get(kv, nkv, "cmd_down"));
    break;
  case K_OCTAL:
    b->oct_dref = find_dref(kv_get(kv, nkv, "dref"));
    break;
  case K_AP:
    b->alt_dref   = find_dref(kv_get(kv, nkv, "alt_dref"));
    b->alt_step   = atof(kv_def(kv, nkv, "alt_step", "100"));
    b->vs_dref    = find_dref(kv_get(kv, nkv, "vs_dref"));
    b->vs_step    = atof(kv_def(kv, nkv, "vs_step", "100"));
    b->ap_btn[0]  = find_cmd(kv_get(kv, nkv, "btn_AP"));
    b->ap_btn[1]  = find_cmd(kv_get(kv, nkv, "btn_HDG"));
    b->ap_btn[2]  = find_cmd(kv_get(kv, nkv, "btn_NAV"));
    b->ap_btn[3]  = find_cmd(kv_get(kv, nkv, "btn_APR"));
    b->ap_btn[4]  = find_cmd(kv_get(kv, nkv, "btn_ALT"));
    b->ap_btn[5]  = find_cmd(kv_get(kv, nkv, "btn_VS"));
    break;
  case K_FMS:
    b->fms[F_CHAP_UP]  = find_cmd(kv_get(kv, nkv, "chapter_up"));
    b->fms[F_CHAP_DN]  = find_cmd(kv_get(kv, nkv, "chapter_dn"));
    b->fms[F_PAGE_UP]  = find_cmd(kv_get(kv, nkv, "page_up"));
    b->fms[F_PAGE_DN]  = find_cmd(kv_get(kv, nkv, "page_dn"));
    b->fms[F_CURSOR]   = find_cmd(kv_get(kv, nkv, "cursor"));
    b->fms[F_CDI]      = find_cmd(kv_get(kv, nkv, "cdi"));
    b->fms[F_OBS]      = find_cmd(kv_get(kv, nkv, "obs"));
    b->fms[F_MSG]      = find_cmd(kv_get(kv, nkv, "msg"));
    b->fms[F_FPL]      = find_cmd(kv_get(kv, nkv, "fpl"));
    b->fms[F_VNAV]     = find_cmd(kv_get(kv, nkv, "vnav"));
    b->fms[F_PROC]     = find_cmd(kv_get(kv, nkv, "proc"));
    b->fms[F_DIRECT]   = find_cmd(kv_get(kv, nkv, "direct"));
    b->fms[F_MENU]     = find_cmd(kv_get(kv, nkv, "menu"));
    b->fms[F_CLR]      = find_cmd(kv_get(kv, nkv, "clr"));
    b->fms[F_ENT]      = find_cmd(kv_get(kv, nkv, "ent"));
    b->fms[F_ZOOM_IN]  = find_cmd(kv_get(kv, nkv, "zoom_in"));
    b->fms[F_ZOOM_OUT] = find_cmd(kv_get(kv, nkv, "zoom_out"));
    break;
  case K_CMDMAP:
    b->m_large_inc = find_cmd(kv_get(kv, nkv, "large_inc"));
    b->m_large_dec = find_cmd(kv_get(kv, nkv, "large_dec"));
    b->m_small_inc = find_cmd(kv_get(kv, nkv, "small_inc"));
    b->m_small_dec = find_cmd(kv_get(kv, nkv, "small_dec"));
    b->m_knob      = find_cmd(kv_get(kv, nkv, "knob"));
    b->m_shift     = find_cmd(kv_get(kv, nkv, "shift"));
    b->m_btn_ap    = find_cmd(kv_get(kv, nkv, "btn_AP"));
    b->m_btn_hdg   = find_cmd(kv_get(kv, nkv, "btn_HDG"));
    b->m_btn_nav   = find_cmd(kv_get(kv, nkv, "btn_NAV"));
    b->m_btn_apr   = find_cmd(kv_get(kv, nkv, "btn_APR"));
    b->m_btn_alt   = find_cmd(kv_get(kv, nkv, "btn_ALT"));
    b->m_btn_vs    = find_cmd(kv_get(kv, nkv, "btn_VS"));
    b->m_btn_d     = find_cmd(kv_get(kv, nkv, "btn_D"));
    b->m_btn_menu  = find_cmd(kv_get(kv, nkv, "btn_MENU"));
    b->m_btn_clr   = find_cmd(kv_get(kv, nkv, "btn_CLR"));
    b->m_btn_ent   = find_cmd(kv_get(kv, nkv, "btn_ENT"));
    if (kv_get(kv, nkv, "cond_dref")) {
      b->c_cond_dref = find_dref(kv_get(kv, nkv, "cond_dref"));
      b->c_large_inc = find_cmd(kv_get(kv, nkv, "cond_large_inc"));
      b->c_large_dec = find_cmd(kv_get(kv, nkv, "cond_large_dec"));
      b->c_small_inc = find_cmd(kv_get(kv, nkv, "cond_small_inc"));
      b->c_small_dec = find_cmd(kv_get(kv, nkv, "cond_small_dec"));
    }
    break;
  default: break;
  }

  /* SHIFT+rotate override (optional, applies to any function kind). */
  b->sh_large_inc = find_cmd(kv_get(kv, nkv, "shift_large_inc"));
  b->sh_large_dec = find_cmd(kv_get(kv, nkv, "shift_large_dec"));
  b->sh_small_inc = find_cmd(kv_get(kv, nkv, "shift_small_inc"));
  b->sh_small_dec = find_cmd(kv_get(kv, nkv, "shift_small_dec"));
}

static void parse_ini(FILE *f) {
  char line[512];
  char section[48] = "";
  kv_pair kv[MAX_KV];
  int nkv = 0;

  while (fgets(line, sizeof line, f)) {
    char *s = trim(line);
    if (*s == '\0' || *s == ';' || *s == '#') continue;
    if (*s == '[') {
      char *end = strchr(s, ']');
      if (!end) continue;
      if (section[0]) build_section(section, kv, nkv);
      *end = '\0';
      strncpy(section, s + 1, sizeof section - 1);
      section[sizeof section - 1] = '\0';
      nkv = 0;
      continue;
    }
    char *eq = strchr(s, '=');
    if (!eq) continue;
    *eq = '\0';
    char *k = trim(s);
    char *v = trim(eq + 1);
    if (nkv < MAX_KV) {
      strncpy(kv[nkv].key, k, sizeof kv[nkv].key - 1);
      kv[nkv].key[sizeof kv[nkv].key - 1] = '\0';
      strncpy(kv[nkv].val, v, sizeof kv[nkv].val - 1);
      kv[nkv].val[sizeof kv[nkv].val - 1] = '\0';
      nkv++;
    }
  }
  if (section[0]) build_section(section, kv, nkv);
}

/* ---- public API ---------------------------------------------------------- */

void profile_set_dir(const char *dir) {
  strncpy(g_dir, dir, sizeof g_dir - 1);
  g_dir[sizeof g_dir - 1] = '\0';
}

int profile_load(const char *acf_filename) {
  memset(g_bind, 0, sizeof g_bind);
  g_led_ap_state = g_led_approach = NULL;
  g_loaded[0] = '\0';

  /* "Cessna_172SP.acf" -> "Cessna_172SP" */
  char base[256];
  strncpy(base, acf_filename, sizeof base - 1);
  base[sizeof base - 1] = '\0';
  char *dot = strrchr(base, '.');
  if (dot && strcasecmp(dot, ".acf") == 0) *dot = '\0';

  char path[1024];
  snprintf(path, sizeof path, "%s/%s.ini", g_dir, base);
  FILE *f = fopen(path, "r");
  if (!f) {
    octavi_log("no profile for '%s' (looked for %s) - device idle", acf_filename, path);
    return 0;
  }
  parse_ini(f);
  fclose(f);

  snprintf(g_loaded, sizeof g_loaded, "%s.ini", base);
  octavi_log("loaded profile %s", g_loaded);
  return 1;
}

const char *profile_name(void) {
  return g_loaded[0] ? g_loaded : NULL;
}

int profile_led_value(void) {
  if (!g_led_ap_state) return -1;
  int ap_state = XPLMGetDatai(g_led_ap_state);
  int approach = g_led_approach ? XPLMGetDatai(g_led_approach) : 0;
  return octavi_led_value(ap_state, approach);
}

void profile_dispatch(int fn, const octavi_report *cur, const octavi_report *prev) {
  if (fn < 0 || fn >= FN_COUNT) return;
  func_binding *b = &g_bind[fn];
  int L = cur->large_delta, S = cur->small_delta;

  /* SHIFT+rotate override (e.g. DG drift adjust on the HDG knob): while SHIFT is
   * held, the rotation drives the shift-knob commands and is consumed, so the
   * function's normal knob action does not also fire. */
  if (cur->shift && (b->sh_large_inc || b->sh_large_dec || b->sh_small_inc || b->sh_small_dec)) {
    if (L > 0 && b->sh_large_inc)      XPLMCommandOnce(b->sh_large_inc);
    else if (L < 0 && b->sh_large_dec) XPLMCommandOnce(b->sh_large_dec);
    if (S > 0 && b->sh_small_inc)      XPLMCommandOnce(b->sh_small_inc);
    else if (S < 0 && b->sh_small_dec) XPLMCommandOnce(b->sh_small_dec);
    L = 0;
    S = 0;
  }

#define EDGE(field) (cur->field && !prev->field)

  switch (b->kind) {
  case K_FREQ:
    if (b->sel_have) {
      int v = 0;
      XPLMGetDatavi(b->sel_dref, &v, b->sel_idx, 1);
      if (v != b->sel_want) XPLMCommandOnce(b->sel_cmd);
    }
    if (b->freq_dref && (L || S)) {
      int nv = octavi_calc_freq(XPLMGetDatai(b->freq_dref),
                                b->coarse_min, b->coarse_max, L, S, b->fine_step);
      XPLMSetDatai(b->freq_dref, nv);
    }
    if (EDGE(shift) && b->flip_cmd) XPLMCommandOnce(b->flip_cmd);
    break;

  case K_WRAP360:
    if (b->dir_dref && (L || S)) {
      double nv = octavi_wrap360(XPLMGetDataf(b->dir_dref), L, S,
                                 b->coarse_step, b->dir_fine_step);
      XPLMSetDataf(b->dir_dref, (float)nv);
    }
    break;

  case K_KNOBCMD:
    if ((L > 0 || S > 0) && b->cmd_up)        XPLMCommandOnce(b->cmd_up);
    else if ((L < 0 || S < 0) && b->cmd_down) XPLMCommandOnce(b->cmd_down);
    break;

  case K_OCTAL:
    if (b->oct_dref && (L || S)) {
      int nv = octavi_calc_xpdr(XPLMGetDatai(b->oct_dref), L, S);
      XPLMSetDatai(b->oct_dref, nv);
    }
    break;

  case K_AP:
    if (b->alt_dref && L)
      XPLMSetDataf(b->alt_dref, XPLMGetDataf(b->alt_dref) + (float)(L * b->alt_step));
    if (b->vs_dref && S)
      XPLMSetDataf(b->vs_dref, XPLMGetDataf(b->vs_dref) + (float)(S * b->vs_step));
    if (EDGE(ap)  && b->ap_btn[0]) XPLMCommandOnce(b->ap_btn[0]);
    if (EDGE(hdg) && b->ap_btn[1]) XPLMCommandOnce(b->ap_btn[1]);
    if (EDGE(nav) && b->ap_btn[2]) XPLMCommandOnce(b->ap_btn[2]);
    if (EDGE(apr) && b->ap_btn[3]) XPLMCommandOnce(b->ap_btn[3]);
    if (EDGE(alt) && b->ap_btn[4]) XPLMCommandOnce(b->ap_btn[4]);
    if (EDGE(vs)  && b->ap_btn[5]) XPLMCommandOnce(b->ap_btn[5]);
    break;

  case K_FMS:
    if (L > 0 && b->fms[F_CHAP_UP])      XPLMCommandOnce(b->fms[F_CHAP_UP]);
    else if (L < 0 && b->fms[F_CHAP_DN]) XPLMCommandOnce(b->fms[F_CHAP_DN]);
    if (S > 0 && b->fms[F_PAGE_UP])      XPLMCommandOnce(b->fms[F_PAGE_UP]);
    else if (S < 0 && b->fms[F_PAGE_DN]) XPLMCommandOnce(b->fms[F_PAGE_DN]);
    if (EDGE(knob) && b->fms[F_CURSOR])  XPLMCommandOnce(b->fms[F_CURSOR]);

    /* SHIFT+AP / SHIFT+HDG = zoom; otherwise AP=CDI, HDG=OBS (edge). */
    if (cur->shift && cur->ap) {
      if (b->fms[F_ZOOM_IN]) XPLMCommandOnce(b->fms[F_ZOOM_IN]);
    } else if (EDGE(ap) && b->fms[F_CDI]) {
      XPLMCommandOnce(b->fms[F_CDI]);
    }
    if (cur->shift && cur->hdg) {
      if (b->fms[F_ZOOM_OUT]) XPLMCommandOnce(b->fms[F_ZOOM_OUT]);
    } else if (EDGE(hdg) && b->fms[F_OBS]) {
      XPLMCommandOnce(b->fms[F_OBS]);
    }

    if (EDGE(nav)  && b->fms[F_MSG])    XPLMCommandOnce(b->fms[F_MSG]);
    if (EDGE(apr)  && b->fms[F_FPL])    XPLMCommandOnce(b->fms[F_FPL]);
    if (EDGE(alt)  && b->fms[F_VNAV])   XPLMCommandOnce(b->fms[F_VNAV]);
    if (EDGE(vs)   && b->fms[F_PROC])   XPLMCommandOnce(b->fms[F_PROC]);
    if (EDGE(d)    && b->fms[F_DIRECT]) XPLMCommandOnce(b->fms[F_DIRECT]);
    if (EDGE(menu) && b->fms[F_MENU])   XPLMCommandOnce(b->fms[F_MENU]);
    if (EDGE(clr)  && b->fms[F_CLR])    XPLMCommandOnce(b->fms[F_CLR]);
    if (EDGE(ent)  && b->fms[F_ENT])    XPLMCommandOnce(b->fms[F_ENT]);
    break;

  case K_CMDMAP:
    /* Knob detents are single-activation manipulators -> CommandOnce is right.
     * Buttons are press-and-hold manipulators -> begin on press, end on release. */
    if (b->c_cond_dref && XPLMGetDatai(b->c_cond_dref) != 0) {
      /* Conditional override active (e.g. VS mode): pulse the alt commands.
       * These target press-and-hold buttons, so a pulse (begin..hold..end). */
      if (L > 0)      cmd_pulse(b->c_large_inc);
      else if (L < 0) cmd_pulse(b->c_large_dec);
      if (S > 0)      cmd_pulse(b->c_small_inc);
      else if (S < 0) cmd_pulse(b->c_small_dec);
    } else {
      if (L > 0 && b->m_large_inc)      XPLMCommandOnce(b->m_large_inc);
      else if (L < 0 && b->m_large_dec) XPLMCommandOnce(b->m_large_dec);
      if (S > 0 && b->m_small_inc)      XPLMCommandOnce(b->m_small_inc);
      else if (S < 0 && b->m_small_dec) XPLMCommandOnce(b->m_small_dec);
    }
#define PRESS(field, cmd) \
    do { if (cur->field && !prev->field) cmd_down(cmd); \
         else if (!cur->field && prev->field) cmd_up(cmd); } while (0)
    PRESS(knob,  b->m_knob);
    PRESS(shift, b->m_shift);
    PRESS(ap,    b->m_btn_ap);
    PRESS(hdg,   b->m_btn_hdg);
    PRESS(nav,   b->m_btn_nav);
    PRESS(apr,   b->m_btn_apr);
    PRESS(alt,   b->m_btn_alt);
    PRESS(vs,    b->m_btn_vs);
    PRESS(d,     b->m_btn_d);
    PRESS(menu,  b->m_btn_menu);
    PRESS(clr,   b->m_btn_clr);
    PRESS(ent,   b->m_btn_ent);
#undef PRESS
    break;

  default:
    break;
  }

#undef EDGE
}
