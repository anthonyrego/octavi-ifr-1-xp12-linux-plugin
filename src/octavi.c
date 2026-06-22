/* octavi.c - see octavi.h. Faithful C port of the original Lua decode + math. */
#include "octavi.h"
#include <math.h>
#include <stdio.h>

int octavi_decode(const unsigned char *b, int len, octavi_report *r) {
  r->valid = 0;
  if (len < 8 || b[0] != OCTAVI_REPORT_ID) return 0;

  r->selector = b[7];
  r->shift = (b[2] >> 0) & 1;
  r->knob  = (b[2] >> 1) & 1;
  r->ap    = (b[2] >> 6) & 1;
  r->hdg   = (b[2] >> 7) & 1;
  r->nav   = (b[3] >> 0) & 1;
  r->apr   = (b[3] >> 1) & 1;
  r->alt   = (b[3] >> 2) & 1;
  r->vs    = (b[3] >> 3) & 1;
  r->d     = (b[1] >> 4) & 1;
  r->menu  = (b[1] >> 5) & 1;
  r->clr   = (b[1] >> 6) & 1;
  r->ent   = (b[1] >> 7) & 1;

  /* Knob bytes are relative deltas. Only single detents (0x01 / 0xff) are
   * treated as movement; anything else (including the garbage value seen on the
   * first read after a trigger write) is ignored. This matches the original. */
  r->large_delta = (b[5] == 0x01) ? 1 : (b[5] == 0xff ? -1 : 0);
  r->small_delta = (b[6] == 0x01) ? 1 : (b[6] == 0xff ? -1 : 0);

  r->valid = 1;
  return 1;
}

int octavi_function_id(int selector, int is_primary) {
  static const int primary[8]   = { FN_COM1, FN_COM2, FN_NAV1, FN_NAV2,
                                     FN_FMS1, FN_FMS2, FN_AP,   FN_XPDR };
  static const int secondary[8] = { FN_HDG,  FN_BARO, FN_CRS1, FN_CRS2,
                                     FN_FMS1, FN_FMS2, FN_AP,   FN_MODE };
  if (selector < 0 || selector > 7) return -1;
  return is_primary ? primary[selector] : secondary[selector];
}

int octavi_calc_freq(int freq, double coarse_min, double coarse_max,
                     int incr_coarse, int incr_fine, double step_fine) {
  double f = (double)freq;
  double freq_fine = fmod(f, 100.0);
  double freq_coarse = f - freq_fine;

  /* The instrument floors the frequency; recover the precise fine step. */
  double fine_err = fmod(freq_fine, step_fine);
  if (fine_err > 0) freq_fine = freq_fine - fine_err + step_fine;

  freq_coarse += incr_coarse * 100.0;
  freq_fine   += incr_fine * step_fine;

  if (freq_coarse >= coarse_max)      freq_coarse = coarse_min;
  else if (freq_coarse < coarse_min)  freq_coarse = coarse_max - 100.0;

  if (freq_fine >= 100.0) freq_fine -= 100.0;
  else if (freq_fine < 0) freq_fine += 100.0;

  return (int)floor(freq_coarse + freq_fine);
}

double octavi_wrap360(double dir, int incr_coarse, int incr_fine,
                      double step_coarse, double step_fine) {
  dir = dir + incr_coarse * step_coarse + incr_fine * step_fine;
  if (dir > 360.0)     dir -= 360.0;
  else if (dir < 0.0)  dir += 360.0;
  return dir;
}

/* Interpret the decimal digits of 'value' as an octal number -> decimal. */
static int oct_to_dec(int value) {
  char s[32];
  snprintf(s, sizeof s, "%d", value);
  int dec = 0;
  for (char *p = s; *p; ++p) {
    if (*p < '0' || *p > '7') return 0;
    dec = dec * 8 + (*p - '0');
  }
  return dec;
}

/* Inverse of oct_to_dec: decimal -> digits-as-octal representation. */
static int dec_to_oct(int value) {
  char s[32];
  snprintf(s, sizeof s, "%o", value);
  int out = 0;
  for (char *p = s; *p; ++p) out = out * 10 + (*p - '0');
  return out;
}

int octavi_calc_xpdr(int code, int incr_coarse, int incr_fine) {
  int dec = oct_to_dec(code);
  dec += incr_coarse * 64 + incr_fine; /* coarse steps the 3rd octal digit */
  if (dec < 0)      dec += 4096;       /* wrap a 4-digit octal code (0..4095) */
  if (dec >= 4096)  dec -= 4096;
  return dec_to_oct(dec);
}

int octavi_led_value(int ap_state, int approach_status) {
  int led = 0;
  /* Classic C172 (no glass): AP off reports 0x200000. Any other value -> AP on. */
  if (ap_state != 0x200000) led = 1;
  if (ap_state & (1 << 1))  led += 2;   /* HDG */
  if ((ap_state & (1 << 8)) || (ap_state & (1 << 9)) || (ap_state & (1 << 19)))
                            led += 4;   /* NAV */
  if (approach_status > 0)  led += 8;   /* APR */
  if ((ap_state & (1 << 5)) || (ap_state & (1 << 14)))
                            led += 16;  /* ALT */
  if (ap_state & (1 << 4))  led += 32;  /* VS */
  return led;
}
