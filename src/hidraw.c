/* hidraw.c - see hidraw.h */
#include "hidraw.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* Parse a "HID_ID=0003:000004D8:0000E6D6" line. Returns 1 and sets vid/pid. */
static int parse_hid_id(const char *line, unsigned *vid, unsigned *pid) {
  unsigned bus;
  /* uevent values are uppercase hex; %x handles both cases. */
  return sscanf(line, "HID_ID=%x:%x:%x", &bus, vid, pid) == 3;
}

/* Read /sys/class/hidraw/<name>/device/uevent and check it matches vid/pid. */
static int hidraw_matches(const char *name, unsigned short vid, unsigned short pid) {
  char path[320];
  snprintf(path, sizeof path, "/sys/class/hidraw/%s/device/uevent", name);
  FILE *f = fopen(path, "r");
  if (!f) return 0;

  char line[512];
  int match = 0;
  while (fgets(line, sizeof line, f)) {
    unsigned v, p;
    if (parse_hid_id(line, &v, &p)) {
      match = (v == vid && p == pid);
      break;
    }
  }
  fclose(f);
  return match;
}

int hidraw_open_octavi(unsigned short vid, unsigned short pid,
                       char *out_path, int out_len) {
  DIR *d = opendir("/sys/class/hidraw");
  if (!d) return -1;

  int fd = -1;
  struct dirent *e;
  while ((e = readdir(d)) != NULL) {
    if (strncmp(e->d_name, "hidraw", 6) != 0) continue;
    if (!hidraw_matches(e->d_name, vid, pid)) continue;

    char dev[288];
    snprintf(dev, sizeof dev, "/dev/%s", e->d_name);
    fd = open(dev, O_RDWR | O_NONBLOCK);
    if (fd >= 0) {
      if (out_path && out_len > 0) {
        strncpy(out_path, dev, out_len - 1);
        out_path[out_len - 1] = '\0';
      }
      break;
    }
  }
  closedir(d);
  return fd;
}

int hidraw_read(int fd, unsigned char *buf, int len) {
  int n = (int)read(fd, buf, len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; /* nothing pending */
    return -1;                                             /* real error */
  }
  return n;
}

int hidraw_write(int fd, const unsigned char *buf, int len) {
  return (int)write(fd, buf, len);
}

void hidraw_close(int fd) {
  if (fd >= 0) close(fd);
}
