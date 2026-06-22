/*
 * octavi.h - Octavi IFR-1 report decoding and value math.
 *
 * Pure logic, no X-Plane dependency. This is a faithful C port of the decoding
 * and the frequency / heading / transponder / LED math from the original
 * FlyWithLua project (cyberang3l/octavi-xplane-flywithlua).
 *
 * 8-byte input report (big-endian), report id 0x0b:
 *   b0  report id (0x0b)
 *   b1  buttons  (D=bit4, MENU=bit5, CLR=bit6, ENT=bit7)
 *   b2  buttons  (SHIFT=bit0, KNOB=bit1, AP=bit6, HDG=bit7)
 *   b3  buttons  (NAV=bit0, APR=bit1, ALT=bit2, VS=bit3)
 *   b4  unused
 *   b5  large knob delta (0x01 = +1 / right, 0xff = -1 / left, relative)
 *   b6  small knob delta (same encoding)
 *   b7  selector position (0..7) -> selects the active function
 */
#ifndef OCTAVI_H
#define OCTAVI_H

#define OCTAVI_VENDOR_ID  0x04d8
#define OCTAVI_PRODUCT_ID 0xe6d6
#define OCTAVI_REPORT_ID  0x0b

/* Function IDs (match the original Lua FunctionID table). */
enum {
  FN_COM1 = 0, FN_COM2 = 1, FN_NAV1 = 2, FN_NAV2 = 3,
  FN_FMS1 = 4, FN_FMS2 = 5, FN_AP = 6, FN_XPDR = 7,
  FN_HDG = 8, FN_BARO = 9, FN_CRS1 = 10, FN_CRS2 = 11,
  FN_MODE = 15,
  FN_COUNT = 16
};

/* A decoded input report: button held-states plus knob deltas. */
typedef struct {
  int valid;        /* 1 if the report decoded (report id matched) */
  int selector;     /* b7, 0..7 */
  /* button held states (1 = pressed) */
  int shift, knob, ap, hdg, nav, apr, alt, vs, d, menu, clr, ent;
  int large_delta;  /* -1, 0, +1 */
  int small_delta;  /* -1, 0, +1 */
} octavi_report;

/* Decode an 8-byte raw report. Returns 1 and fills *r when valid, else 0. */
int octavi_decode(const unsigned char *buf, int len, octavi_report *r);

/* Map (selector, is_primary) to a function id (one of the FN_* enum values). */
int octavi_function_id(int selector, int is_primary);

/* COM/NAV frequency stepping (port of CalcNewFreq). freq in 10 kHz units. */
int octavi_calc_freq(int freq, double coarse_min, double coarse_max,
                     int incr_coarse, int incr_fine, double step_fine);

/* Heading / course wrap 0..360 (port of calc_new_dir). */
double octavi_wrap360(double dir, int incr_coarse, int incr_fine,
                      double step_coarse, double step_fine);

/* Transponder octal squawk stepping (port of calc_new_xpdr_code). */
int octavi_calc_xpdr(int code, int incr_coarse, int incr_fine);

/* Compute the LED bitmask from the autopilot_state / approach_status datarefs
 * (port of GetLEDActivationValue). Bit order: AP=1 HDG=2 NAV=4 APR=8 ALT=16 VS=32. */
int octavi_led_value(int ap_state, int approach_status);

#endif /* OCTAVI_H */
