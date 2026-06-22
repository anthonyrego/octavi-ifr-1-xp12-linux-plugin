/*
 * octavi_probe.c - standalone host-side tester (no X-Plane needed).
 *
 * Reuses the plugin's hidraw + decode code to confirm the device works:
 *   - finds and opens the Octavi
 *   - prints decoded reports (selector, buttons, knob deltas) for a few seconds
 *   - with "led" argument, cycles the LEDs to confirm the write path
 *
 * Build:  make probe      Run:  ./build/octavi_probe [led]
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/select.h>

#include "../src/hidraw.h"
#include "../src/octavi.h"

static const char *fn_name(int fn) {
  switch (fn) {
    case FN_COM1: return "COM1"; case FN_COM2: return "COM2";
    case FN_NAV1: return "NAV1"; case FN_NAV2: return "NAV2";
    case FN_FMS1: return "FMS1"; case FN_FMS2: return "FMS2";
    case FN_AP:   return "AP";   case FN_XPDR: return "XPDR";
    case FN_HDG:  return "HDG";  case FN_BARO: return "BARO";
    case FN_CRS1: return "CRS1"; case FN_CRS2: return "CRS2";
    case FN_MODE: return "MODE"; default: return "?";
  }
}

int main(int argc, char **argv) {
  char path[128] = "";
  int fd = hidraw_open_octavi(OCTAVI_VENDOR_ID, OCTAVI_PRODUCT_ID, path, sizeof path);
  if (fd < 0) {
    fprintf(stderr, "Octavi not found (vid 04d8 pid e6d6). Check it is plugged in"
                    " and /dev/hidrawN is accessible.\n");
    return 1;
  }
  printf("Opened Octavi on %s\n", path);

  /* Trigger an initial report so we learn the current selector position. */
  unsigned char trig[8] = { 0, 0, 0, 0, 0, 0, 0, 0xff };
  hidraw_write(fd, trig, sizeof trig);

  if (argc > 1 && strcmp(argv[1], "led") == 0) {
    printf("Cycling LEDs (AP HDG NAV APR ALT VS)...\n");
    for (int v = 0; v <= 0x3f; v++) {
      unsigned char out[2] = { OCTAVI_REPORT_ID, (unsigned char)v };
      if (hidraw_write(fd, out, 2) != 2) { perror("led write"); break; }
      struct timespec ts = { 0, 40 * 1000 * 1000 }; /* 40 ms */
      nanosleep(&ts, NULL);
    }
    unsigned char off[2] = { OCTAVI_REPORT_ID, 0x00 };
    hidraw_write(fd, off, 2);
    printf("LED write path OK (no errors).\n");
  }

  printf("Reading for ~5s - turn the selector / press buttons / spin knobs...\n");
  int is_primary = 1;
  octavi_report prev; memset(&prev, 0, sizeof prev);

  time_t end = time(NULL) + 5;
  while (time(NULL) < end) {
    fd_set rs; FD_ZERO(&rs); FD_SET(fd, &rs);
    struct timeval tv = { 0, 200 * 1000 };
    if (select(fd + 1, &rs, NULL, NULL, &tv) <= 0) continue;

    unsigned char buf[64];
    int n = hidraw_read(fd, buf, sizeof buf);
    if (n <= 0) continue;
    octavi_report r;
    if (!octavi_decode(buf, n, &r)) continue;

    if (r.knob && !prev.knob) is_primary = !is_primary;
    int fn = octavi_function_id(r.selector, is_primary);

    printf("sel=%d %-9s [%s] L=%+d S=%+d  btn:", r.selector,
           fn_name(fn), is_primary ? "PRI" : "SEC", r.large_delta, r.small_delta);
    if (r.shift) printf(" SHIFT");
    if (r.knob)  printf(" KNOB");
    if (r.ap)    printf(" AP");
    if (r.hdg)   printf(" HDG");
    if (r.nav)   printf(" NAV");
    if (r.apr)   printf(" APR");
    if (r.alt)   printf(" ALT");
    if (r.vs)    printf(" VS");
    if (r.d)     printf(" D");
    if (r.menu)  printf(" MENU");
    if (r.clr)   printf(" CLR");
    if (r.ent)   printf(" ENT");
    printf("\n");
    prev = r;
  }

  hidraw_close(fd);
  printf("Done.\n");
  return 0;
}
