/*
 * Host-side stand-in for the STM32H743 firmware main loop.
 *
 * Compiles the SAME src/pid.c that goes into the target .elf and speaks the
 * SAME UART byte protocol as src/main.c:
 *   RX: 4 bytes  -> custom_float_t (float32 LE, True Airspeed [m/s])
 *   TX: 'H' + 4 bytes (float32 LE, commanded thrust [N]) + '\0'
 * Only the transport differs: UART5 registers are replaced by a serial device
 * (a PTY opened by demo/host/plant_hil.py, which plays the Simulink plant).
 *
 * usage: controller_host <tty> <ref_TAS_m_s> [<step_at_iteration> <ref2_TAS_m_s>]
 */
#define _DEFAULT_SOURCE
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include "pid.h"

static int fd;

static void UART_send_blocking(uint8_t *byte)
{
  while (write(fd, byte, 1) != 1) {}
}

static void UART_rcv_blocking(uint8_t *byte)
{
  while (read(fd, byte, 1) != 1) {}
}

int main(int argc, char **argv)
{
  if (argc < 3) {
    fprintf(stderr, "usage: %s <tty> <ref_TAS> [<step_iter> <ref2_TAS>]\n", argv[0]);
    return 1;
  }
  fd = open(argv[1], O_RDWR | O_NOCTTY);
  if (fd < 0) { perror("open tty"); return 1; }
  struct termios tio;
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);
  tcsetattr(fd, TCSANOW, &tio);

  float ref_TAS  = (float)atof(argv[2]);
  long  step_at  = argc > 4 ? atol(argv[3]) : -1;
  float ref2_TAS = argc > 4 ? (float)atof(argv[4]) : ref_TAS;

  /* Identical gains / sample time / initial condition to src/main.c */
  pid_ctrl_t pid;
  pid_init(&pid, 500.0f, 30.0f, 10.0f, 0.1f, 66.5f);

  custom_float_t rcv, snd;
  uint8_t ch;
  for (long k = 0;; k++) {
    if (k == step_at) ref_TAS = ref2_TAS;

    for (int i = 0; i < 4; i++) UART_rcv_blocking(&rcv.bytes[i]);

    float TAS = rcv.single;
    snd.single = pid_step(&pid, ref_TAS, TAS);

    ch = 'H';  UART_send_blocking(&ch);
    for (int i = 0; i < 4; i++) UART_send_blocking(&snd.bytes[i]);
    ch = '\0'; UART_send_blocking(&ch);

    fprintf(stderr, "[ctrl k=%4ld] TAS=%7.2f m/s  ref=%6.2f  u=%9.1f N\n", k, TAS, ref_TAS, snd.single);
  }
}
