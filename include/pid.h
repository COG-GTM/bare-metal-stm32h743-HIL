/*==============================================================================
 * Name        : pid.h
 * Description : Discrete PID control law (hardware independent).
 *
 *   u_k = k_p e_k + k_i * sum_{i<=k} e_i d + k_d (v_{k-1} - v_k) / d
 *
 * This unit contains no HAL / register access so the exact same source is
 * compiled for the Cortex-M7 target and for the host (software HIL, unit tests).
===============================================================================*/
#ifndef PID_H
#define PID_H

typedef struct {
  float k_p;
  float k_i;
  float k_d;
  float d;         /* sample period (s) */
  float sum_err;   /* integral state */
  float old_meas;  /* previous measurement, for derivative on measurement */
} pid_ctrl_t;

void  pid_init(pid_ctrl_t *pid, float k_p, float k_i, float k_d, float d, float init_meas);
float pid_step(pid_ctrl_t *pid, float ref, float meas);

#endif /* PID_H */
