# Stage 3 — Bare-metal controller firmware & HIL

## Talk track (~140 words)

Stage 3 is the hand-written, safety-critical part: a bare-metal C PID on an STM32H743
that holds an aircraft's True Airspeed by commanding thrust. No HAL, no RTOS —
register-level UART5, a 480 MHz clock tree, and a discrete PID exchanging `float`s with
the Simulink plant in `uart.slx` over a six-byte UART frame.

First we cross-compile the real firmware with `arm-none-eabi-gcc` — the size report shows
3.4 kB of flash; the control law itself is 88 bytes of Thumb-2.

Then the trick: the PID lives in its own hardware-independent file, so we compile the
*same* `pid.c` with host gcc and close the loop against a Python point-mass aircraft — the
same plant parameters as the Simulink model — over a pseudo-terminal speaking the exact
firmware byte protocol. Setpoint 120 knots, step to 150: rise, small overshoot, integral
pulls the error to zero.

Same code, same protocol, same plant — tested before hardware, and again on hardware.

## What you're seeing

- `arm-none-eabi-size main.elf`: the real Cortex-M7 build, `pid_step` linked at a flash address.
- `firmware_footprint.png`: flash/RAM usage of the target image.
- Terminal: 600 closed-loop iterations, 10 UART bytes each, streaming TAS / thrust / error.
- `hil_step_response.png`: TAS vs setpoint, thrust command and error for the 120 -> 150 kt step.
- Everything except clock/GPIO/UART5 register init is the code that ships on the chip.

## Hand-off

Next, Stage 4 takes this validated controller and integrates it as an AUTOSAR-style
software component inside a full ECU simulation.
