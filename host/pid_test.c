/* Host unit tests for src/pid.c (no HAL). Build: make test */
#include "pid.h"
#include <math.h>
#include <stdio.h>

static int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok:   %s\n", msg); } while (0)
#define NEAR(a, b, tol) (fabsf((a) - (b)) < (tol))

int main(void)
{
  pid_ctrl_t pid;

  /* Proportional only: u = kp * e */
  pid_init(&pid, 2.0f, 0.0f, 0.0f, 0.1f, 10.0f);
  CHECK(NEAR(pid_step(&pid, 15.0f, 10.0f), 10.0f, 1e-5f), "P term: 2*(15-10) = 10");

  /* Integral accumulates e*d each step */
  pid_init(&pid, 0.0f, 1.0f, 0.0f, 0.1f, 0.0f);
  pid_step(&pid, 1.0f, 0.0f);
  CHECK(NEAR(pid_step(&pid, 1.0f, 0.0f), 0.2f, 1e-5f), "I term: two steps of e=1, d=0.1 -> 0.2");

  /* Derivative on measurement: (old - new)/d, first step uses init_meas */
  pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.1f, 66.5f);
  CHECK(NEAR(pid_step(&pid, 0.0f, 67.5f), -10.0f, 1e-4f), "D term: (66.5-67.5)/0.1 = -10");
  CHECK(NEAR(pid_step(&pid, 0.0f, 67.5f), 0.0f, 1e-5f), "D term: no change -> 0");

  /* Firmware gains, first sample: matches the original inline main.c arithmetic */
  pid_init(&pid, 500.0f, 30.0f, 10.0f, 0.1f, 66.5f);
  float u = pid_step(&pid, 80.0f, 66.5f);
  /* err=13.5, sum=1.35, diff=0 -> 500*13.5 + 30*1.35 = 6790.5 */
  CHECK(NEAR(u, 6790.5f, 1e-2f), "firmware gains first step = 6790.5 N");

  /* Measured period: the I and D actions follow the period set at run time,
     so a loop running at half the nominal cadence must not integrate or
     differentiate as if it were running at 0.1 s */
  pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.1f, 66.5f);
  pid_set_period(&pid, 0.05f);
  CHECK(NEAR(pid_step(&pid, 0.0f, 67.5f), -20.0f, 1e-4f), "D term uses the measured period: (66.5-67.5)/0.05 = -20");

  pid_init(&pid, 0.0f, 1.0f, 0.0f, 0.1f, 0.0f);
  pid_set_period(&pid, 0.05f);
  pid_step(&pid, 1.0f, 0.0f);
  CHECK(NEAR(pid_step(&pid, 1.0f, 0.0f), 0.1f, 1e-5f), "I term uses the measured period: two steps of e=1, d=0.05 -> 0.1");

  /* A non-positive measurement keeps the last valid period */
  pid_init(&pid, 0.0f, 1.0f, 0.0f, 0.1f, 0.0f);
  pid_set_period(&pid, 0.0f);
  CHECK(NEAR(pid_step(&pid, 1.0f, 0.0f), 0.1f, 1e-5f), "non-positive period ignored, keeps d=0.1");

  /* Zero error, steady state: output equals integral memory only */
  pid_init(&pid, 500.0f, 30.0f, 10.0f, 0.1f, 80.0f);
  CHECK(NEAR(pid_step(&pid, 80.0f, 80.0f), 0.0f, 1e-5f), "zero error, zero history -> 0");

  printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
  return fails ? 1 : 0;
}
