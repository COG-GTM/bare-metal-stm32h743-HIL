#!/bin/sh
# Firmware-in-the-loop: boot the ARM main.elf on Renode's STM32H743, close
# UART5 against host/plant.py over TCP, then run the same scenario through the
# host build of src/pid.c so the two traces can be compared.
set -eu
RENODE=${RENODE:-renode}
PORT=${PORT:-3456}
OUT=${OUT:-host/results_fil}
# Firmware setpoint is 80 m/s (src/main.c); match it exactly on the host side.
REF_KT=$(python3 -c "print(80/0.514444)")
ARGS="--ref-kt $REF_KT --step-kt $REF_KT --init-kt 130 --step-time 0 --duration 60"

python3 host/plant.py $ARGS --out "${OUT}_host"

# Own process group so the renode wrapper and its dotnet child are both reaped.
setsid sh -c "tail -f /dev/null | \"$RENODE\" --disable-gui --console \
  -e '\$elf=@main.elf; \$port=$PORT; include @host/renode/stm32h743_hil.resc; start'" \
  > "${OUT}_renode.log" 2>&1 &
RENODE_PGID=$!
trap 'kill -- -$RENODE_PGID 2>/dev/null || true' EXIT
i=0
until ss -ltn | grep -q ":$PORT "; do
  i=$((i+1)); [ $i -gt 120 ] && { echo "renode UART socket never opened"; cat "${OUT}_renode.log"; exit 1; }
  sleep 0.5
done
sleep 3   # let the firmware reach UART_rcv_blocking() before the first byte
python3 host/plant.py --tcp 127.0.0.1:$PORT $ARGS --out "${OUT}_target"
