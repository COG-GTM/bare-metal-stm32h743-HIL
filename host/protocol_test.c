/* Host unit tests for the frame reassembly in include/hil_protocol.h. Build: make test */
#include "hil_protocol.h"
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok:   %s\n", msg); } while (0)

static uint8_t stream[256];
static int stream_len;

static void push_frame(float v)
{
  custom_float_t f;
  f.single = v;
  stream[stream_len++] = HIL_HEADER;
  memcpy(&stream[stream_len], f.bytes, HIL_FLOAT_BYTES);
  stream_len += HIL_FLOAT_BYTES;
  stream[stream_len++] = HIL_TERMINATOR;
}

/* Feed the stream from `from` and collect every frame the state machine accepts. */
static int decode(int from, float* out, int max)
{
  hil_rx_t rx;
  custom_float_t value;
  int n = 0;
  hil_rx_init(&rx);
  for (int i = from; i < stream_len && n < max; i++) {
    if (hil_rx_push(&rx, stream[i], &value)) { out[n++] = value.single; }
  }
  return n;
}

int main(void)
{
  const float vals[] = {80.0f, 66.5f, 72.25f, 12345.6f, -3.5f};
  const int n_vals = (int)(sizeof(vals) / sizeof(vals[0]));
  float got[16];
  int n;

  /* Clean stream: every sample decodes */
  stream_len = 0;
  for (int i = 0; i < n_vals; i++) push_frame(vals[i]);
  n = decode(0, got, 16);
  CHECK(n == n_vals && memcmp(got, vals, sizeof(vals)) == 0, "clean stream decodes every frame");

  /* A payload byte is lost: only that frame is lost, the phase recovers */
  stream_len = 0;
  for (int i = 0; i < n_vals; i++) push_frame(vals[i]);
  memmove(&stream[8], &stream[9], (size_t)(stream_len - 9));  /* byte 2 of frame 1 */
  stream_len--;
  n = decode(0, got, 16);
  CHECK(n == n_vals - 1 && got[0] == vals[0] && got[1] == vals[2] && got[2] == vals[3]
        && got[3] == vals[4], "dropped byte costs one frame, later frames decode");

  /* Spurious bytes between frames are discarded */
  stream_len = 0;
  stream[stream_len++] = 0x5A;
  stream[stream_len++] = 0x13;
  for (int i = 0; i < n_vals; i++) push_frame(vals[i]);
  n = decode(0, got, 16);
  CHECK(n == n_vals && memcmp(got, vals, sizeof(vals)) == 0, "leading line noise is discarded");

  /* Starting mid-frame (MCU booting mid-stream) resyncs on the next header */
  stream_len = 0;
  for (int i = 0; i < n_vals; i++) push_frame(vals[i]);
  n = decode(3, got, 16);
  CHECK(n == n_vals - 1 && got[0] == vals[1], "mid-stream start resyncs on the next frame");

  printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
  return fails ? 1 : 0;
}
