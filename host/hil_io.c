/*==============================================================================
 * Name        : hil_io.c
 * Description : Robust blocking byte I/O for the host software-HIL node.
===============================================================================*/
#define _DEFAULT_SOURCE
#include "hil_io.h"

#include <errno.h>
#include <unistd.h>

int hil_write_all(int fd, const uint8_t *buf, size_t n)
{
  size_t done = 0;
  while (done < n) {
    ssize_t w = write(fd, buf + done, n - done);
    if (w > 0) {
      done += (size_t)w;
    } else if (w == 0 || errno == EINTR || errno == EAGAIN) {
      continue;
    } else {
      return -1;
    }
  }
  return 0;
}

int hil_read_all(int fd, uint8_t *buf, size_t n)
{
  size_t done = 0;
  while (done < n) {
    ssize_t r = read(fd, buf + done, n - done);
    if (r > 0) {
      done += (size_t)r;
    } else if (r == 0) {
      return 0;
    } else if (errno == EINTR || errno == EAGAIN) {
      continue;
    } else {
      return -1;
    }
  }
  return 1;
}
