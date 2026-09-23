/*==============================================================================
 * Name        : pid_node.c
 * Description : Host build of the controller loop for software HIL.
 *
 * This is the firmware main loop (src/main.c) with the UART5 register accesses
 * replaced by blocking POSIX read()/write() on a serial device / pseudo-terminal.
 * The control law itself is the unmodified src/pid.c, and the byte protocol
 * (including the receive resynchronisation) is include/hil_protocol.h.
 *
 * Usage: pid_node <tty-device> [ref_TAS] [step_ref_TAS step_sample]
 *   ref_TAS         setpoint (m/s), default 80 as in the firmware
 *   step_ref_TAS    new setpoint applied from sample index step_sample onwards
 *                   (lets the harness exercise a setpoint step without touching
 *                   the byte protocol; on the target this is a re-flash)
===============================================================================*/
#define _DEFAULT_SOURCE
#include "pid.h"
#include "hil_protocol.h"
#include "hil_io.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

static int uart_fd = -1;

static void UART_Init(const char *dev)
{
  uart_fd = open(dev, O_RDWR | O_NOCTTY);
  if (uart_fd < 0) { perror(dev); exit(1); }
  struct termios tio;
  if (tcgetattr(uart_fd, &tio) == 0) {
    cfmakeraw(&tio);                 /* 8N1, no echo, no line discipline */
    cfsetispeed(&tio, B38400);
    cfsetospeed(&tio, B38400);
    tcsetattr(uart_fd, TCSANOW, &tio);
  }
}


int main(int argc, char **argv)
{
  if (argc < 2) { fprintf(stderr, "usage: %s <tty> [ref_TAS] [step_ref_TAS step_sample]\n", argv[0]); return 2; }
  UART_Init(argv[1]);

  float TAS = 0;
  float ref_TAS = (argc > 2) ? (float)atof(argv[2]) : 80.0f;
  float step_ref = (argc > 4) ? (float)atof(argv[3]) : ref_TAS;
  long  step_at  = (argc > 4) ? atol(argv[4]) : -1;
  long  k = 0;
  float u = 0;
  custom_float_t rcv;
  uint8_t frame[HIL_FRAME_BYTES];
  hil_rx_t rx;
  pid_ctrl_t pid;
  pid_init(&pid, 500.0f, 30.0f, 10.0f, 0.1f, 66.5f);
  hil_rx_init(&rx);

  while (1)
  {
    /* Reception from plant */
    uint8_t byte;
    do {
      int r = hil_read_all(uart_fd, &byte, 1);
      if (r == 0 || (r < 0 && errno == EIO)) return 0;     /* peer closed (PTY master hangup -> EIO) */
      if (r < 0) { perror("read"); return 1; }
    } while (!hil_rx_push(&rx, byte, &rcv));

    /* Controller (PID) */
    if (step_at >= 0 && k == step_at) ref_TAS = step_ref;
    k++;
    TAS = rcv.single;
    u = pid_step(&pid, ref_TAS, TAS);

    /* Transmission to plant */
    hil_frame_encode(u, frame);
    if (hil_write_all(uart_fd, frame, HIL_FRAME_BYTES) < 0) { perror("write"); return 1; }
  }
}
