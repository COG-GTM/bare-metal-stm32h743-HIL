/*==============================================================================
 * Name        : hil_io.h
 * Description : Robust blocking byte I/O for the host software-HIL node.
 *
 * POSIX read()/write() may return short counts, 0 (EOF) or -1 (EINTR, EAGAIN,
 * EIO/EPIPE once the peer closes the pseudo-terminal). These helpers retry the
 * transient cases and report the fatal ones instead of spinning on them.
===============================================================================*/
#ifndef HIL_IO_H
#define HIL_IO_H

#include <stddef.h>
#include <stdint.h>

/* Write exactly n bytes. Returns 0 on success, -1 on a fatal write error. */
int hil_write_all(int fd, const uint8_t *buf, size_t n);

/* Read exactly n bytes. Returns 1 on success, 0 on EOF (peer closed), -1 on error. */
int hil_read_all(int fd, uint8_t *buf, size_t n);

#endif /* HIL_IO_H */
