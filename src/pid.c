/*==============================================================================
 * Name        : pid.c
 * Description : Discrete PID control law (hardware independent).
===============================================================================*/
#include "pid.h"

void pid_init(pid_ctrl_t *pid, float k_p, float k_i, float k_d, float d, float init_meas)
{
  pid->k_p = k_p;
  pid->k_i = k_i;
  pid->k_d = k_d;
  pid->d = d;
  pid->sum_err = 0.0f;
  pid->old_meas = init_meas;
}

void pid_set_period(pid_ctrl_t *pid, float d)
{
  if (d > 0.0f)
  {
    pid->d = d;
  }
}

float pid_step(pid_ctrl_t *pid, float ref, float meas)
{
  float err      = ref - meas;                       /* proportional action */
  float diff_err = (pid->old_meas - meas) / pid->d;  /* derivative action   */
  pid->sum_err  += err * pid->d;                     /* integral action     */
  pid->old_meas  = meas;
  return pid->k_p * err + pid->k_i * pid->sum_err + pid->k_d * diff_err;
}
