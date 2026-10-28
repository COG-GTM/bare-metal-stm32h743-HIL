# Software HIL: same control code, same plant, before hardware

> Hand-written safety-critical control code, tested in the loop against the
> same plant model — before hardware, and again on hardware.

## Architecture

The PID control law lives in `src/pid.c` and knows nothing about registers.
The byte protocol (`custom_float_t`, `'H'` header, `'\0'` terminator) lives in
`include/hil_protocol.h`. Both files are compiled twice:

| Build            | Compiler          | I/O                             | Plant                              |
|------------------|-------------------|---------------------------------|------------------------------------|
| `make`           | `arm-none-eabi-gcc` (Cortex-M7, FPv5) | `UART5->TDR / RDR` registers | Simulink `uart.slx` over a USB-UART |
| `make host hil`  | host `gcc`        | POSIX `read()/write()` on a PTY | `host/plant.py` (same point-mass model, same parameters as `Simulink/param_init.m`) |

```mermaid
flowchart LR
    subgraph HW["Hardware-in-the-loop (target)"]
        SIM["Simulink plant<br/>uart.slx<br/>point-mass aircraft"]
        FW["STM32H743 firmware<br/>src/main.c + src/pid.c<br/>bare-metal UART5, 480 MHz"]
        SIM -- "float32 TAS (4 bytes)" --> FW
        FW -- "'H' + float32 thrust + '\\0'" --> SIM
    end

    subgraph SW["Software-in-the-loop (host, no hardware)"]
        PY["host/plant.py<br/>same point-mass model<br/>RK4, ZOH @ 0.1 s"]
        NODE["host/pid_node<br/>host gcc build of<br/>src/pid.c + firmware loop"]
        PY -- "float32 TAS (4 bytes)" --> NODE
        NODE -- "'H' + float32 thrust + '\\0'" --> PY
    end

    PID["src/pid.c<br/>include/hil_protocol.h<br/>(single source)"]
    PID -. "arm-none-eabi-gcc" .-> FW
    PID -. "gcc" .-> NODE
    PY -. "substitutes" .-> SIM
```

Both sides of the loop talk the identical UART byte stream (38400 8N1 on the
target; a pseudo-terminal on the host), so the host harness exercises the
framing, the byte/float union and the control law exactly as the target does.

## Running it

```sh
# 1. Target firmware (needs gcc-arm-none-eabi on PATH or in /opt/gcc-arm/bin)
make                     # -> main.elf, main.bin + arm-none-eabi-size report

# 2. Host build of the same PID + unit tests
make host test

# 3. Software HIL run: 120 kt hold, step to 150 kt at t=60 s, 120 s total
make hil                 # -> host/results.csv, host/results.png
python3 host/plant.py --ref-kt 120 --step-kt 150 --step-time 60 --duration 120
```

## Result: step 120 kt -> 150 kt (firmware gains k_p=500, k_i=30, k_d=10, d=0.1 s)

![software HIL step response](software_hil_step_120_150kt.png)

```
step 120.0 -> 150.0 kt: rise(90%)=9.2 s  overshoot=11.4%  settle(2%)=51.4 s  final err=-0.27 kt
```

The controller starts cold (zero integral state, zero thrust) at 120 kt, so the
first 40 s show the integrator winding up to trim thrust (~2.7 kN) against
drag. The 30 kt step then produces a 10.4 kN thrust spike, ~11 % overshoot and
a slow tail as the integrator re-trims — a useful, honest picture of the gains
before anything is flashed.

## Stretch: the real firmware ELF under Renode, same plant

Renode ships an STM32H743 platform, and the unmodified `main.elf` boots on it:
`SystemClock_Config()` runs to completion (Renode's PWR stub reports VOSRDY,
the RCC model tolerates the PLL writes), `UART_Init()` enables UART5, and the
firmware sits in `UART_rcv_blocking()` waiting for the plant. UART5 is bridged
to a TCP socket and `host/plant.py --tcp` closes the loop:

```sh
make                                         # main.elf
renode --disable-gui --console \
  -e '$elf=@main.elf; include @host/renode/stm32h743_hil.resc; start'
python3 host/plant.py --tcp 127.0.0.1:3456 --ref-kt 155.5 --step-kt 155.5 \
        --init-kt 130 --duration 60 --out host/results_renode
```

(The firmware's compiled-in setpoint is 80 m/s = 155.5 kt, hence the 130 -> 155.5 kt capture; 60 s of plant time runs in ~3 s wall clock.)

![firmware under Renode](renode_firmware_hil_155kt.png)

Comparing this run against the host `pid_node` build with identical initial
conditions *and the firmware's exact 80 m/s setpoint* gives **601/601 thrust
samples bit-identical** (max |thrust diff| = 0 N). An earlier 79 N residual came
from commanding 155.5 kt (79.996 m/s) on the host side, not from the compiler.

## Requirement gate and CI

`host/hil_check.py` turns a run into pass/fail against explicit limits
(`STEP_REQS`, `ACT_REQS`, `EQUIV_REQS`) and exits non-zero on any violation:

```sh
make hil-check                                # REQ-SPD-01..04 + REQ-ACT-01..02 on the 120->150 kt step
make fil RENODE=/path/to/renode               # main.elf on Renode vs host build: REQ-EQV-01..02 + REQ-ACT
```

`.github/workflows/build-and-hil.yml` runs `make`, `make host test`,
`make hil-check` and `make fil` on every push/PR, writes the requirement table
to the job summary and uploads `main.elf`, `main.map`, the size report and all
`host/results*` traces as the `hil-results` artifact.

## What's what

- `src/pid.c`, `include/pid.h` — `pid_ctrl_t`, `pid_init()`, `pid_step()`. Derivative is on the
  measurement `(v_{k-1} - v_k)/d`, as in the original firmware.
- `include/hil_protocol.h` — `custom_float_t` union, header/terminator, payload size.
- `host/pid_node.c` — the firmware main loop with `UART_send_blocking`/`UART_rcv_blocking`
  reimplemented on a file descriptor. Takes an optional setpoint step
  (`ref step_ref step_sample`) so the harness can command a step without changing the protocol.
- `host/pid_test.c` — host unit tests for the control law (`make test`).
- `host/plant.py` — plant, PTY plumbing, CSV + PNG output, step metrics.

Note: the README describes the header as `'A'`; the firmware (and therefore
`hil_protocol.h`) uses `'H'`. Keep the Simulink receive block in sync with the
constant in `hil_protocol.h`.
