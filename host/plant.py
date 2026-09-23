#!/usr/bin/env python3
"""Software-HIL plant: point-mass aircraft airspeed model over a UART byte protocol.

Stands in for Simulink/uart.slx. Opens a pseudo-terminal pair, launches the
host-built controller (host/pid_node, which compiles the same src/pid.c as the
firmware) on the slave side, and exchanges bytes with it using exactly the
protocol the STM32 firmware speaks:

    plant -> controller : b'H' + float32 TAS (4 bytes, little endian) + b'\\0'
    controller -> plant : b'H' + float32 thrust (4 bytes) + b'\\0'

Plant (from Simulink/param_init.m, level flight, lift = weight):
    alpha = (2 M g / (rho V^2 S) - Cl0) / Cla
    D     = 0.5 rho V^2 S (Cd0 + Cd1 alpha + Cd2 alpha^2)
    M dV/dt = T - D
"""
import argparse
import csv
import os
import pty
import struct
import subprocess
import sys
import time
import tty

KT2MS = 0.514444

# --- parameters (Simulink/param_init.m, get_rho.m) --------------------------
G = 9.81
M = 2994.0
S = 30.19
ALTITUDE = 3000.0
CL = (0.2, 4.8)
CD = (0.0357, 0.0763, 1.4515)
INIT_TAS = 66.5


def get_rho(h):
    p0, T0, g, L, R, Mair = 101325, 288.15, 9.80665, 0.0065, 8.31446, 0.0289652
    return p0 * Mair / (R * T0) * (1 - L * h / T0) ** (g * Mair / (R * L) - 1)


RHO = get_rho(ALTITUDE)


def drag(v):
    q = 0.5 * RHO * v * v * S
    alpha = (M * G / q - CL[0]) / CL[1]
    return q * (CD[0] + CD[1] * alpha + CD[2] * alpha ** 2)


def plant_step(v, thrust, dt, t_max):
    """One zero-order-hold sample period, integrated with RK4 on the continuous plant."""
    thrust = min(max(thrust, 0.0), t_max)  # actuator limits

    def f(vv):
        return (thrust - drag(vv)) / M

    k1 = f(v)
    k2 = f(v + 0.5 * dt * k1)
    k3 = f(v + 0.5 * dt * k2)
    k4 = f(v + dt * k3)
    return v + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4), thrust


# --- UART framing -----------------------------------------------------------
def read_exact(fd, n):
    buf = b""
    while len(buf) < n:
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            raise EOFError("controller closed the link")
        buf += chunk
    return buf


def send_frame(fd, value, drop_byte=False):
    """Frame a float32 as header + payload + terminator (drop_byte: line fault)."""
    frame = b"H" + struct.pack("<f", value) + b"\0"
    if drop_byte:
        frame = frame[:2] + frame[3:]
    os.write(fd, frame)


def recv_frame(fd):
    """Sync on header 'H', return float32 payload, expect '\\0' terminator."""
    while True:
        b = read_exact(fd, 1)
        if b == b"H":
            break
    payload = read_exact(fd, 4)
    term = read_exact(fd, 1)
    if term != b"\0":
        raise ValueError("bad terminator %r" % term)
    return struct.unpack("<f", payload)[0]


def open_link(args):
    """Return (fd, proc). Either spawn the host controller on a fresh PTY, or
    open an existing serial device (real board, or firmware under Renode)."""
    if args.tcp:
        import socket
        host, port = args.tcp.rsplit(":", 1)
        s = socket.create_connection((host, int(port)), timeout=30)
        s.setblocking(True)
        return os.dup(s.fileno()), None
    if args.dev:
        import termios
        fd = os.open(args.dev, os.O_RDWR | os.O_NOCTTY)
        tio = termios.tcgetattr(fd)
        tty.setraw(fd)
        tio = termios.tcgetattr(fd)
        tio[4] = tio[5] = termios.B38400
        termios.tcsetattr(fd, termios.TCSANOW, tio)
        return fd, None
    master, slave = pty.openpty()
    tty.setraw(slave)
    step_sample = int(round(args.step_time / args.dt))
    # Hand the slave end to the controller as an inherited fd; it opens it like
    # any other serial device path (robust to sandboxed /dev/pts namespaces).
    proc = subprocess.Popen([args.controller, "/dev/fd/%d" % slave, str(args.ref_ms),
                             str(args.sp_ms), str(step_sample)], pass_fds=[slave])
    os.close(slave)
    return master, proc


def run(args):
    master, proc = open_link(args)
    step_sample = int(round(args.step_time / args.dt))

    dt = args.dt
    v = args.init_tas
    t = 0.0
    u = 0.0
    log = []
    k = 0
    try:
        while t <= args.duration + 1e-9:
            sp = args.sp_ms if k >= step_sample else args.ref_ms
            k += 1
            drop = args.drop_byte_at is not None and abs(t - args.drop_byte_at) < dt / 2
            send_frame(master, v, drop_byte=drop)
            if not drop:
                u = recv_frame(master)
            # else: the controller discards the malformed frame and stays silent
            # for this sample, so the actuator holds the previous thrust command
            err = sp - v
            log.append((t, v, sp, u, err))
            v, _ = plant_step(v, u, dt, args.t_max)
            t += dt
            if args.realtime:
                time.sleep(dt)
    finally:
        os.close(master)
        if proc:
            proc.wait(timeout=5)
    return log


def write_outputs(log, out):
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    with open(out + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "tas_ms", "sp_ms", "thrust_N", "err_ms"])
        w.writerows(log)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r[0] for r in log]
    tas = [r[1] / KT2MS for r in log]
    sp = [r[2] / KT2MS for r in log]
    u = [r[3] / 1000 for r in log]
    err = [r[4] / KT2MS for r in log]

    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(t, sp, "k--", label="setpoint")
    ax[0].plot(t, tas, label="TAS (plant)")
    ax[0].set_ylabel("TAS [kt]")
    ax[0].legend()
    ax[0].grid(True)
    ax[0].set_title("Software HIL: src/pid.c (host gcc) vs point-mass aircraft plant over PTY/UART")
    ax[1].plot(t, u, color="tab:red")
    ax[1].set_ylabel("thrust cmd [kN]")
    ax[1].grid(True)
    ax[2].plot(t, err, color="tab:green")
    ax[2].axhline(0, color="k", lw=0.5)
    ax[2].set_ylabel("error [kt]")
    ax[2].set_xlabel("time [s]")
    ax[2].grid(True)
    fig.tight_layout()
    fig.savefig(out + ".png", dpi=120)
    print("wrote", out + ".csv", out + ".png")


def step_metrics(log, step_time):
    post = [r for r in log if r[0] >= step_time]
    if not post:
        return
    sp = post[0][2]
    v0 = post[0][1]
    delta = sp - v0
    peak = max(r[1] for r in post)
    overshoot = (peak - sp) / delta * 100 if delta else 0
    band = 0.02 * abs(delta)
    settle = None
    for i, r in enumerate(post):
        if all(abs(rr[1] - sp) <= band for rr in post[i:]):
            settle = r[0] - step_time
            break
    rise = next((r[0] - step_time for r in post if (r[1] - v0) / delta >= 0.9), None)
    print(f"step {v0/KT2MS:.1f} -> {sp/KT2MS:.1f} kt: rise(90%)={rise}s  "
          f"overshoot={overshoot:.1f}%  settle(2%)={settle}s  "
          f"final err={post[-1][4]/KT2MS:+.3f} kt")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--controller", default=os.path.join(os.path.dirname(__file__), "pid_node"))
    p.add_argument("--dev", default=None,
                   help="existing serial device to talk to instead of spawning --controller "
                        "(e.g. Renode UART pty or a real board; setpoint step then unavailable)")
    p.add_argument("--tcp", default=None,
                   help="host:port of a raw TCP UART bridge (Renode CreateServerSocketTerminal)")
    p.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "results", "hil"))
    p.add_argument("--dt", type=float, default=0.1, help="sample period [s] (firmware d)")
    p.add_argument("--duration", type=float, default=120.0)
    p.add_argument("--ref-kt", type=float, default=120.0, help="controller setpoint [kt]")
    p.add_argument("--step-kt", type=float, default=150.0, help="setpoint after step [kt]")
    p.add_argument("--step-time", type=float, default=60.0)
    p.add_argument("--init-kt", type=float, default=120.0, help="initial TAS [kt]")
    p.add_argument("--t-max", type=float, default=40e3, help="max thrust [N]")
    p.add_argument("--realtime", action="store_true", help="sleep dt each sample")
    p.add_argument("--drop-byte-at", type=float, default=None,
                   help="time [s] at which one byte of the frame sent to the controller "
                        "is dropped, to check that the controller resynchronises")
    args = p.parse_args()
    args.ref_ms = args.ref_kt * KT2MS
    args.sp_ms = args.step_kt * KT2MS
    args.init_tas = args.init_kt * KT2MS

    log = run(args)
    write_outputs(log, args.out)
    step_metrics(log, args.step_time)


if __name__ == "__main__":
    sys.exit(main())
