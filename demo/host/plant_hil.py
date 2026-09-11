#!/usr/bin/env python3
"""
Software-HIL harness: plays the role of the Simulink aircraft plant (Simulink/uart.slx)
and drives the controller over a pseudo-terminal using the firmware's UART byte protocol.

Plant = point-mass aircraft in level flight (parameters copied from Simulink/param_init.m):
    M dV/dt = T - D(V),   D = 1/2 rho V^2 S Cd(alpha),  L = W  (level flight)
Controller = demo/host/controller_host (src/pid.c compiled with gcc), spawned as a child
process on the PTY slave; this script owns the PTY master.

Protocol (little endian float32, 8N1 semantics irrelevant on a PTY):
    plant -> ctrl : 4 bytes TAS [m/s]
    ctrl  -> plant: 'H' + 4 bytes thrust [N] + '\\0'
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

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

KT = 1.0 / 0.514444  # m/s -> knots

# ---- Simulink/param_init.m -------------------------------------------------
g = 9.81
M = 2994.0
S = 30.19
altitude = 3000.0
Cl = (0.2, 4.8)
Cd = (0.0357, 0.0763, 1.4515)


def get_rho(h):  # Simulink/get_rho.m (ISA troposphere)
    p0, T0, g0, L, R, Mair = 101325, 288.15, 9.80665, 0.0065, 8.31446, 0.0289652
    return p0 * Mair / (R * T0) * (1 - L * h / T0) ** (g0 * Mair / (R * L) - 1)


rho = get_rho(altitude)


def drag(V):
    q = 0.5 * rho * V * V * S
    cl = M * g / q                          # level flight: lift = weight
    alpha = (cl - Cl[0]) / Cl[1]
    cd = Cd[0] + Cd[1] * alpha + Cd[2] * alpha * alpha
    return q * cd


def plant_step(V, T, dt, substeps=10):
    h = dt / substeps
    for _ in range(substeps):
        V += h * (T - drag(V)) / M
    return V


def read_exact(fd, n):
    buf = b""
    while len(buf) < n:
        buf += os.read(fd, n - len(buf))
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", default=os.path.join(os.path.dirname(__file__), "controller_host"))
    ap.add_argument("--init-kt", type=float, default=120.0, help="plant initial TAS")
    ap.add_argument("--ref-kt", type=float, default=120.0)
    ap.add_argument("--ref2-kt", type=float, default=150.0)
    ap.add_argument("--step-time", type=float, default=15.0, help="s, when setpoint changes")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--realtime", type=float, default=0.0, help="0=as fast as possible, 1=real time")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "out"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    ref1 = args.ref_kt / KT
    ref2 = args.ref2_kt / KT
    step_iter = int(round(args.step_time / args.dt))
    n_steps = int(round(args.duration / args.dt))

    master, slave = pty.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    slave_name = os.ttyname(slave)
    print(f"[plant] PTY {slave_name}  rho={rho:.3f} kg/m^3  M={M:.0f} kg  dt={args.dt}s")
    print(f"[plant] launching controller: {args.controller} {slave_name} ref={ref1:.2f} m/s "
          f"-> {ref2:.2f} m/s at k={step_iter}")
    ctrl = subprocess.Popen([args.controller, slave_name, f"{ref1}", str(step_iter), f"{ref2}"],
                            stderr=subprocess.DEVNULL, pass_fds=(slave,))
    os.close(slave)

    V = args.init_kt / KT  # param_init.m uses initTAS = 66.5 m/s; demo starts at the 120 kt setpoint
    T = 0.0
    rows = []
    t0 = time.perf_counter()
    try:
        for k in range(n_steps):
            t = k * args.dt
            ref = ref2 if k >= step_iter else ref1
            os.write(master, struct.pack("<f", V))          # Simulink "Byte pack" + "Send"
            frame = read_exact(master, 6)                   # Simulink "Receive" w/ header/terminator
            if frame[0] != ord("H") or frame[5] != 0:
                raise RuntimeError(f"framing error: {frame!r}")
            T = struct.unpack("<f", frame[1:5])[0]
            rows.append((t, V, ref, T, ref - V))
            if k % 10 == 0:
                bar = "#" * int(max(0.0, min(T, 40000.0)) / 1000)
                print(f"t={t:5.1f}s  TAS={V*KT:6.1f} kt  ref={ref*KT:5.1f} kt  "
                      f"err={(ref-V)*KT:6.2f} kt  thrust={T:8.0f} N |{bar}")
            V = plant_step(V, max(T, 0.0), args.dt)          # no reverse thrust
            if args.realtime > 0:
                time.sleep(args.dt * args.realtime)
    finally:
        ctrl.kill()
    wall = time.perf_counter() - t0
    print(f"[plant] {n_steps} closed-loop iterations, {6 + 4} bytes/iter over PTY, {wall:.2f}s wall")

    csv_path = os.path.join(args.out, "hil_run.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "TAS_m_s", "ref_m_s", "thrust_N", "err_m_s"])
        w.writerows(rows)

    # ---- metrics ------------------------------------------------------------
    ts = [r[0] for r in rows]
    tas = [r[1] * KT for r in rows]
    refk = [r[2] * KT for r in rows]
    thr = [r[3] for r in rows]
    err = [r[4] * KT for r in rows]
    post = [(t, v) for t, v in zip(ts, tas) if t >= args.step_time]
    d = args.ref2_kt - args.ref_kt
    rise = next((t - args.step_time for t, v in post if v >= args.ref_kt + 0.9 * d), float("nan"))
    overshoot = max(v for _, v in post) - args.ref2_kt
    settle = next((t - args.step_time for i, (t, _) in enumerate(post)
                   if all(abs(v - args.ref2_kt) <= 0.02 * d for _, v in post[i:])), float("nan"))
    final_err = err[-1]
    settle_s = f"{settle:.1f}s" if settle == settle else f">{args.duration - args.step_time:.0f}s"
    summary = (f"Step {args.ref_kt:.0f} -> {args.ref2_kt:.0f} kt @ t={args.step_time:.0f}s: "
               f"rise(90%)={rise:.1f}s  overshoot={overshoot:.2f} kt  "
               f"settle(2%)={settle_s}  final err={final_err:.2f} kt")
    print("[plant] " + summary)
    with open(os.path.join(args.out, "hil_summary.txt"), "w") as f:
        f.write(summary + "\n")

    # ---- plot ---------------------------------------------------------------
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    fig.suptitle("Software-HIL: STM32H7 PID (src/pid.c, gcc host build) vs point-mass aircraft plant\n"
                 "closed over the firmware UART byte protocol (PTY)", fontsize=12)
    ax[0].plot(ts, refk, "k--", label="setpoint")
    ax[0].plot(ts, tas, "tab:blue", lw=2, label="True Airspeed")
    ax[0].set_ylabel("TAS [kt]")
    ax[0].legend(loc="lower right")
    ax[0].grid(alpha=.3)
    ax[0].annotate(summary, xy=(0.01, 0.95), xycoords="axes fraction", fontsize=8, va="top",
                   bbox=dict(boxstyle="round", fc="w", alpha=.8))
    ax[1].plot(ts, thr, "tab:red", lw=2)
    ax[1].set_ylabel("Thrust cmd [N]")
    ax[1].grid(alpha=.3)
    ax[2].plot(ts, err, "tab:green", lw=2)
    ax[2].axhline(0, color="k", lw=.5)
    ax[2].set_ylabel("Error [kt]")
    ax[2].set_xlabel("time [s]")
    ax[2].grid(alpha=.3)
    for a in ax:
        a.axvline(args.step_time, color="gray", ls=":", lw=1)
    fig.tight_layout()
    png = os.path.join(args.out, "hil_step_response.png")
    fig.savefig(png, dpi=130)
    print(f"[plant] wrote {png}, {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
