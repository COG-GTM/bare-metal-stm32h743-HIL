/* Host unit tests for include/hil_protocol.h framing and host/hil_io.c. Build: make test */
#define _DEFAULT_SOURCE
#include "hil_protocol.h"
#include "hil_io.h"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok:   %s\n", msg); } while (0)

int main(void)
{
  signal(SIGPIPE, SIG_IGN);   /* like a PTY: a write to a closed peer must fail, not kill us */

  /* --- framing ------------------------------------------------------------ */
  uint8_t frame[HIL_FRAME_BYTES];
  hil_frame_encode(1.0f, frame);                    /* 0x3F800000 little endian */
  CHECK(HIL_FRAME_BYTES == 7, "frame is 7 bytes: header + float32 + CRC-8 + terminator");
  CHECK(frame[0] == 'H' && frame[HIL_FRAME_BYTES - 1] == '\0',
        "frame starts with 'H' and ends with NUL");
  CHECK(frame[1] == 0x00 && frame[2] == 0x00 && frame[3] == 0x80 && frame[4] == 0x3F,
        "payload is IEEE-754 float32 little endian (1.0f -> 00 00 80 3F)");
  CHECK(frame[HIL_CRC_OFFSET] == hil_crc8(frame, HIL_CRC_OFFSET),
        "CRC-8 byte covers header + payload");

  float back = 0;
  CHECK(hil_frame_decode(frame, &back) == 1 && back == 1.0f, "frame round-trips through decode");

  hil_frame_encode(6790.5f, frame);
  CHECK(hil_frame_decode(frame, &back) == 1 && back == 6790.5f,
        "firmware first-step output 6790.5 N frames correctly");

  /* Corruption is detected instead of being fed to the plant as a valid float. */
  for (int i = 0; i < HIL_FRAME_BYTES; i++) {
    uint8_t corrupt[HIL_FRAME_BYTES];
    memcpy(corrupt, frame, sizeof corrupt);
    corrupt[i] ^= 0x01;
    if (hil_frame_decode(corrupt, &back) != 0) { printf("FAIL: byte %d flip accepted\n", i); fails++; }
  }
  CHECK(1, "single-bit corruption of any frame byte is rejected");

  hil_frame_encode(6790.5f, frame);
  CHECK(hil_float_decode(&frame[HIL_PAYLOAD_OFFSET]) == 6790.5f,
        "payload decodes with explicit little-endian shifts");

  uint8_t le[HIL_FLOAT_BYTES];
  hil_float_encode(-1.5f, le);
  CHECK(le[0] == 0x00 && le[1] == 0x00 && le[2] == 0xC0 && le[3] == 0xBF,
        "-1.5f serialises to 00 00 C0 BF regardless of host byte order");

  /* --- hil_write_all / hil_read_all over a pipe ------------------------------ */
  int p[2];
  CHECK(pipe(p) == 0, "pipe created");
  uint8_t rx[HIL_FRAME_BYTES] = {0};
  CHECK(hil_write_all(p[1], frame, HIL_FRAME_BYTES) == 0, "write_all writes a full frame");
  CHECK(hil_read_all(p[0], rx, HIL_FRAME_BYTES) == 1, "read_all returns 1 after a full frame");
  CHECK(memcmp(rx, frame, HIL_FRAME_BYTES) == 0, "bytes received == bytes sent");

  /* Short reads: producer writes the frame in two chunks, consumer asks for all of it. */
  CHECK(hil_write_all(p[1], frame, 2) == 0 && hil_write_all(p[1], frame + 2, HIL_FRAME_BYTES - 2) == 0,
        "producer writes the frame in two chunks");
  memset(rx, 0, sizeof rx);
  CHECK(hil_read_all(p[0], rx, HIL_FRAME_BYTES) == 1 && memcmp(rx, frame, HIL_FRAME_BYTES) == 0,
        "read_all reassembles a frame delivered in two chunks");

  /* EOF: writer closes -> read_all reports 0, not a hang. */
  close(p[1]);
  CHECK(hil_read_all(p[0], rx, 1) == 0, "read_all returns 0 on EOF (peer closed)");
  close(p[0]);

  /* Fatal write: reader closed -> write() fails with EPIPE; write_all must return -1
     (the old UART_send_blocking spun forever on `write() != 1`). */
  CHECK(pipe(p) == 0, "second pipe created");
  close(p[0]);
  errno = 0;
  CHECK(hil_write_all(p[1], frame, HIL_FRAME_BYTES) == -1 && errno == EPIPE,
        "write_all returns -1 (EPIPE) when the peer is gone instead of spinning");
  close(p[1]);

  /* Fatal write on a bad descriptor. */
  CHECK(hil_write_all(-1, frame, 1) == -1, "write_all returns -1 on EBADF");
  CHECK(hil_read_all(-1, rx, 1) == -1, "read_all returns -1 on EBADF");

  printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
  return fails ? 1 : 0;
}
