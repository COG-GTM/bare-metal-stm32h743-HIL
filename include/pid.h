/*==============================================================================
 * Name        : pid.h
 * Description : Discrete PID control law, independent of any hardware so that
 *               it can be unit tested on the host (see test/test_pid.c).
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

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float k_p;        // proportional gain
  float k_i;        // integral gain
  float k_d;        // derivative gain
  float ref;        // setpoint
  float sum_err;    // integral state
  float prev_meas;  // previous measurement (derivative state)
} pid_t;

/**
  * Initialise the controller state.
  */
void pid_init(pid_t* pid, float k_p, float k_i, float k_d, float ref,
              float initial_meas);

/**
  * Compute the control law for a measurement sampled dt seconds after the
  * previous one. dt is the measured loop period: passing the true elapsed time
  * keeps the integral and derivative actions consistent with the real cadence.
  * A non-positive dt leaves the states untouched and yields the proportional
  * and integral actions only.
  */
float pid_update(pid_t* pid, float meas, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
