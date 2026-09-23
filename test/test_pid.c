/*==============================================================================
 * Name        : test_pid.c
 * Description : Host-side unit tests of the PID control law. Build and run with
 *               `make test` (uses the host compiler, no board required).
===============================================================================*/

#include "pid.h"
#include <math.h>
#include <stdio.h>

static int failures = 0;

static void check_close(const char* name, float got, float expected)
{
  float tol = 1e-4f*(1.0f + fabsf(expected));
  if (fabsf(got - expected) > tol)
  {
    printf("FAIL %s: got %.6f, expected %.6f\n", name, got, expected);
    failures++;
  }
  else
  {
    printf("ok   %s\n", name);
  }
}

/**
  * Reference implementation of the control law, kept independent of pid.c
  */
static float reference(float k_p, float k_i, float k_d, float ref,
                       float prev_meas, float sum_err, float meas, float dt)
{
  float err      = ref - meas;
  float diff_err = (prev_meas - meas)/dt;
  sum_err       += err*dt;
  return k_p*err + k_i*sum_err + k_d*diff_err;
}

/**
  * A single step must match the control law evaluated at the given dt
  */
static void test_single_step(void)
{
  pid_t pid;
  pid_init(&pid, 500, 30, 10, 80, 66.5f);
  float u = pid_update(&pid, 70.0f, 0.1f);
  check_close("single step at nominal dt",
              u, reference(500, 30, 10, 80, 66.5f, 0.0f, 70.0f, 0.1f));
}

/**
  * The integral and derivative actions must scale with the measured dt: a loop
  * running twice as fast must not integrate or differentiate as if it were
  * running at the nominal 0.1 s period
  */
static void test_dt_scaling(void)
{
  pid_t fast;
  pid_init(&fast, 500, 30, 10, 80, 66.5f);
  float u_fast = pid_update(&fast, 70.0f, 0.05f);
  check_close("dt scaling (half period)",
              u_fast, reference(500, 30, 10, 80, 66.5f, 0.0f, 70.0f, 0.05f));

  pid_t nominal;
  pid_init(&nominal, 500, 30, 10, 80, 66.5f);
  float u_nominal = pid_update(&nominal, 70.0f, 0.1f);
  if (fabsf(u_fast - u_nominal) < 1e-3f)
  {
    printf("FAIL dt scaling: output insensitive to dt\n");
    failures++;
  }
  else
  {
    printf("ok   dt scaling changes the output\n");
  }

  /* Two half-period steps integrate the same area as one full-period step at
     constant error */
  pid_t split;
  pid_init(&split, 0, 30, 0, 80, 66.5f);
  pid_update(&split, 70.0f, 0.05f);
  float u_split = pid_update(&split, 70.0f, 0.05f);

  pid_t whole;
  pid_init(&whole, 0, 30, 0, 80, 66.5f);
  float u_whole = pid_update(&whole, 70.0f, 0.1f);
  check_close("integral action independent of sampling split", u_split, u_whole);
}

/**
  * A steady measurement gives no derivative action, whatever the period
  */
static void test_no_derivative_when_steady(void)
{
  pid_t pid;
  pid_init(&pid, 0, 0, 10, 80, 70.0f);
  check_close("no derivative action when steady", pid_update(&pid, 70.0f, 0.3f), 0.0f);
}

/**
  * A non-positive dt must not produce infinities or corrupt the states
  */
static void test_non_positive_dt(void)
{
  pid_t pid;
  pid_init(&pid, 500, 30, 10, 80, 66.5f);
  float u = pid_update(&pid, 70.0f, 0.0f);
  check_close("zero dt yields proportional action only", u, 500.0f*(80.0f - 70.0f));
  check_close("zero dt leaves the integral state untouched", pid.sum_err, 0.0f);
}

int main(void)
{
  test_single_step();
  test_dt_scaling();
  test_no_derivative_when_steady();
  test_non_positive_dt();

  if (failures)
  {
    printf("%d test(s) failed\n", failures);
    return 1;
  }

  printf("all tests passed\n");
  return 0;
}
