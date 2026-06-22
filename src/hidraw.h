/*
 * hidraw.h - minimal Linux hidraw access for the Octavi IFR-1.
 *
 * No external dependencies: the device node is found by scanning
 * /sys/class/hidraw/<n>/device/uevent for the VID/PID, then opened directly.
 */
#ifndef HIDRAW_H
#define HIDRAW_H

/* Find and open the Octavi (vendor 0x04d8, product 0xe6d6) in non-blocking RDWR
 * mode. On success returns a fd >= 0 and copies the /dev path into out_path
 * (if non-NULL, out_len bytes). Returns -1 if not found / not openable. */
int hidraw_open_octavi(unsigned short vid, unsigned short pid,
                       char *out_path, int out_len);

/* Non-blocking read of one report. Returns bytes read (>0), 0 if no data
 * pending, or -1 on a real error (e.g. device unplugged). */
int hidraw_read(int fd, unsigned char *buf, int len);

/* Write a raw output report (first byte is the report id). Returns bytes
 * written or -1 on error. */
int hidraw_write(int fd, const unsigned char *buf, int len);

void hidraw_close(int fd);

#endif /* HIDRAW_H */
