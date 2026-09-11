#!/usr/bin/env bash
# Stage 3 demo: bare-metal STM32H743 PID firmware, cross-compiled for real,
# then the SAME PID source closed in a software-HIL loop against a Python
# aircraft plant over the firmware's UART byte protocol (PTY).
#
#   ./demo/run_demo.sh            # fast (plant runs as fast as possible)
#   REALTIME=0.05 ./demo/run_demo.sh   # slow the loop down for a live terminal view
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$PWD
OUT=$ROOT/demo/out
mkdir -p "$OUT"
REALTIME=${REALTIME:-0}

B=$'\e[1m'; C=$'\e[36m'; G=$'\e[32m'; Y=$'\e[33m'; N=$'\e[0m'
step() { echo; echo "${B}${C}== $* ==${N}"; }

step "1/4  Cross-compile firmware for STM32H743 (Cortex-M7, arm-none-eabi-gcc)"
TOOLCHAIN=$(dirname "$(command -v arm-none-eabi-gcc)")
arm-none-eabi-gcc --version | head -1
make clean >/dev/null
make TOOLCHAIN="$TOOLCHAIN" 2>&1 | grep -E "arm-none-eabi-(gcc|as|g\+\+|objcopy) " | sed 's/ -.*-I.\/drivers//' | cut -c1-140
echo
echo "${Y}arm-none-eabi-size main.elf${N}"
arm-none-eabi-size main.elf | tee "$OUT/size_report.txt"
arm-none-eabi-nm main.elf | grep -E " (pid_step|pid_init|main)$" | sed "s/^/  /"
python3 demo/host/memory_report.py main.elf "$OUT/firmware_footprint.png" "$OUT/size_report_full.txt" | tail -1
cp main.elf main.bin "$OUT/"
echo "${G}TARGET artifacts: demo/out/main.elf, main.bin  (flash with: make flash)${N}"

step "2/4  Build host harness: SAME src/pid.c compiled with native gcc (no HAL, no registers)"
gcc -std=c99 -Wall -O2 -Iinclude demo/host/controller_host.c src/pid.c -o demo/host/controller_host
echo "  gcc -std=c99 -Wall -O2 -Iinclude demo/host/controller_host.c src/pid.c -> demo/host/controller_host"
md5sum src/pid.c | sed 's/^/  pid.c md5 (identical source in .elf and host binary): /'

step "3/4  Software-HIL run: Python aircraft plant <-UART bytes over PTY-> PID controller"
echo "  plant  : point-mass aircraft, Simulink/param_init.m parameters (M=2994 kg, S=30.19 m2, h=3000 m)"
echo "  protocol: TAS float32 -> ctrl ; 'H' + thrust float32 + '\\0' -> plant   (custom_float_t)"
echo "  scenario: hold 120 kt, step to 150 kt at t=15 s, 0.1 s sample time"
python3 demo/host/plant_hil.py --realtime "$REALTIME" --out "$OUT"

step "4/4  Artifacts in demo/out/"
ls -1 "$OUT"
echo
echo "${G}${B}Done.${N} Host-executed: PID (src/pid.c) + plant + UART protocol.  Target-only: clock/UART5/GPIO init (main.elf)."
