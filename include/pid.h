/*==============================================================================
 * Name        : pid.h
 * Description : Discrete PID control law (True Airspeed -> commanded thrust).
 *               Pure C99, no HAL / register access: the same source is compiled
 *               for the STM32H743 target (arm-none-eabi-gcc) and for the host
 *               software-HIL harness (gcc) under demo/.
===============================================================================*/
#ifndef PID_H
#define PID_H

#include <stdint.h>

/* Union used to serialise a 4-byte float over the byte-oriented UART link. */
typedef union {
  float single;
  uint8_t bytes[4];
} custom_float_t;

typedef struct {
  float k_p;
  float k_i;
  float k_d;
  float d;        /* sample time (s) */
  float sum_err;  /* integral state */
  float old_TAS;  /* previous measurement for the derivative term */
} pid_ctrl_t;

void  pid_init(pid_ctrl_t *pid, float k_p, float k_i, float k_d, float d, float init_TAS);
float pid_step(pid_ctrl_t *pid, float ref_TAS, float TAS);

#endif /* PID_H */
