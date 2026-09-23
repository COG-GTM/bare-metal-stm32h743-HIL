/*==============================================================================
 * Name        : pid.c
 * Description : Discrete PID control law used by the HIL simulation
 -------------------------------------------------------------------------------
 * The MIT License (MIT)
 * Copyright (c) 2022 Martin Doff-Sotta
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
===============================================================================*/

#include "pid.h"

void pid_init(pid_t* pid, float k_p, float k_i, float k_d, float ref,
              float initial_meas)
{
  pid->k_p       = k_p;
  pid->k_i       = k_i;
  pid->k_d       = k_d;
  pid->ref       = ref;
  pid->sum_err   = 0.0f;
  pid->prev_meas = initial_meas;
}

float pid_update(pid_t* pid, float meas, float dt)
{
  float err      = pid->ref - meas;                  // proportional action
  float diff_err = 0.0f;                             // derivative action

  if (dt > 0.0f)
  {
    diff_err      = (pid->prev_meas - meas)/dt;
    pid->sum_err += err*dt;                          // integral action
  }

  pid->prev_meas = meas;

  return pid->k_p*err + pid->k_i*pid->sum_err + pid->k_d*diff_err;  // control law
}
