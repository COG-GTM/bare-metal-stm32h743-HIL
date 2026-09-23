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

static inline void UART_send_blocking(uint8_t *byte)
{
  while (write(uart_fd, byte, 1) != 1) {}
}

static inline int UART_rcv_blocking(uint8_t *byte)
{
  ssize_t n;
  do { n = read(uart_fd, byte, 1); } while (n == 0);
  return n == 1;
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
  uint8_t ch = HIL_TERMINATOR;
  custom_float_t rcv;
  custom_float_t snd;
  hil_rx_t rx;
  pid_ctrl_t pid;
  pid_init(&pid, 500.0f, 30.0f, 10.0f, 0.1f, 66.5f);
  hil_rx_init(&rx);

  while (1)
  {
    /* Reception from plant */
    do
    {
      if (!UART_rcv_blocking(&ch)) return 0;   /* peer closed */
    } while (!hil_rx_push(&rx, ch, &rcv));

    /* Controller (PID) */
    if (step_at >= 0 && k == step_at) ref_TAS = step_ref;
    k++;
    TAS = rcv.single;
    u = pid_step(&pid, ref_TAS, TAS);
    snd.single = u;

    /* Transmission to plant */
    ch = HIL_HEADER;
    UART_send_blocking(&ch);
    for (int i = 0; i < HIL_FLOAT_BYTES; i++)
    {
      UART_send_blocking(&snd.bytes[i]);
    }
    ch = HIL_TERMINATOR;
    UART_send_blocking(&ch);
  }
}
