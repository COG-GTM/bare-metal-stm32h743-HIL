/*==============================================================================
 * Name        : pid.c
 * Description : Discrete PID control law
 *
 *   u_k = k_p (v*_k - v_k) + k_i * sum_{i<=k} (v*_i - v_i) d + k_d (v_{k-1} - v_k)/d
 *
 * Hardware independent: compiled unchanged for the STM32H743 target and for the
 * host software-HIL harness.
===============================================================================*/
#include "pid.h"

void pid_init(pid_ctrl_t *pid, float k_p, float k_i, float k_d, float d, float init_TAS)
{
  pid->k_p = k_p;
  pid->k_i = k_i;
  pid->k_d = k_d;
  pid->d = d;
  pid->sum_err = 0.0f;
  pid->old_TAS = init_TAS;
}

float pid_step(pid_ctrl_t *pid, float ref_TAS, float TAS)
{
  float err      = ref_TAS - TAS;                 // proportional action
  float diff_err = (pid->old_TAS - TAS) / pid->d; // derivative action
  pid->sum_err  += err * pid->d;                  // integral action
  float u = pid->k_p * err + pid->k_i * pid->sum_err + pid->k_d * diff_err;
  pid->old_TAS = TAS;
  return u;
}
