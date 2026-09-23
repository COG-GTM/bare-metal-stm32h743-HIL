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

/* Feed stream[from:len] to the receive state machine, collect the frames it accepts. */
static int decode(const uint8_t *stream, int len, int from, float *out)
{
  hil_rx_t rx;
  custom_float_t value;
  int n = 0;
  hil_rx_init(&rx);
  for (int i = from; i < len; i++) {
    if (hil_rx_push(&rx, stream[i], &value)) { out[n++] = value.single; }
  }
  return n;
}

int main(void)
{
  signal(SIGPIPE, SIG_IGN);   /* like a PTY: a write to a closed peer must fail, not kill us */

  /* --- framing ------------------------------------------------------------ */
  uint8_t frame[HIL_FRAME_BYTES];
  hil_frame_encode(1.0f, frame);                    /* 0x3F800000 little endian */
  CHECK(HIL_FRAME_BYTES == 6, "frame is 6 bytes: header + float32 + terminator");
  CHECK(frame[0] == 'H' && frame[5] == '\0', "frame starts with 'H' and ends with NUL");
  CHECK(frame[1] == 0x00 && frame[2] == 0x00 && frame[3] == 0x80 && frame[4] == 0x3F,
        "payload is IEEE-754 float32 little endian (1.0f -> 00 00 80 3F)");

  custom_float_t back;
  memcpy(back.bytes, &frame[1], HIL_FLOAT_BYTES);
  CHECK(back.single == 1.0f, "payload round-trips through custom_float_t");

  /* --- receive resynchronisation ------------------------------------------ */
  {
    const float vals[] = {80.0f, 66.5f, 72.25f, 12345.6f, -3.5f};
    const int n_vals = (int)(sizeof vals / sizeof vals[0]);
    uint8_t stream[8 * HIL_FRAME_BYTES];
    float got[8];
    int len, n;

    len = 0;
    for (int i = 0; i < n_vals; i++) {
      hil_frame_encode(vals[i], &stream[len]);
      len += HIL_FRAME_BYTES;
    }
    n = decode(stream, len, 0, got);
    CHECK(n == n_vals && memcmp(got, vals, sizeof vals) == 0, "clean stream decodes every frame");

    n = decode(stream, len, 3, got);           /* receiver boots mid-frame */
    CHECK(n == n_vals - 1 && got[0] == vals[1], "mid-stream start resyncs on the next frame");

    uint8_t lost[sizeof stream];               /* one payload byte lost on the line */
    memcpy(lost, stream, (size_t)len);
    memmove(&lost[8], &lost[9], (size_t)(len - 9));
    n = decode(lost, len - 1, 0, got);
    CHECK(n == n_vals - 1 && got[0] == vals[0] && got[1] == vals[2] && got[3] == vals[4],
          "a dropped byte costs one frame, later frames still decode");

    uint8_t noisy[2 + sizeof stream];          /* line noise before the first header */
    noisy[0] = 0x5A;
    noisy[1] = 0x13;
    memcpy(&noisy[2], stream, (size_t)len);
    n = decode(noisy, len + 2, 0, got);
    CHECK(n == n_vals && memcmp(got, vals, sizeof vals) == 0, "leading line noise is discarded");
  }

  hil_frame_encode(6790.5f, frame);
  memcpy(back.bytes, &frame[1], HIL_FLOAT_BYTES);
  CHECK(back.single == 6790.5f && frame[0] == 'H' && frame[5] == '\0',
        "firmware first-step output 6790.5 N frames correctly");

  /* --- hil_write_all / hil_read_all over a pipe ------------------------------ */
  int p[2];
  CHECK(pipe(p) == 0, "pipe created");
  uint8_t rx[HIL_FRAME_BYTES] = {0};
  CHECK(hil_write_all(p[1], frame, HIL_FRAME_BYTES) == 0, "write_all writes a full frame");
  CHECK(hil_read_all(p[0], rx, HIL_FRAME_BYTES) == 1, "read_all returns 1 after a full frame");
  CHECK(memcmp(rx, frame, HIL_FRAME_BYTES) == 0, "bytes received == bytes sent");

  /* Short reads: producer writes the frame in two chunks, consumer asks for all 6. */
  CHECK(hil_write_all(p[1], frame, 2) == 0 && hil_write_all(p[1], frame + 2, 4) == 0,
        "producer writes 2 + 4 bytes");
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
