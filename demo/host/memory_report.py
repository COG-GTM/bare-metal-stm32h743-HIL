#!/usr/bin/env python3
"""Render arm-none-eabi-size output of main.elf as a flash/RAM footprint chart."""
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

elf, out_png, out_txt = sys.argv[1:4]
size = subprocess.check_output(["arm-none-eabi-size", "-A", elf], text=True)
nm = subprocess.check_output(["arm-none-eabi-nm", "--size-sort", "-S", elf], text=True)
with open(out_txt, "w") as f:
    f.write(size + "\nSymbols (largest last):\n" + nm)

sections = {}
for line in size.splitlines():
    parts = line.split()
    if len(parts) >= 3 and parts[0].startswith("."):
        try:
            sections[parts[0]] = int(parts[1])
        except ValueError:
            pass
flash_secs = {k: v for k, v in sections.items() if k in (".isr_vector", ".text", ".rodata", ".ARM", ".init_array", ".fini_array", ".data") and v}
ram_secs = {k: v for k, v in sections.items() if k in (".data", ".bss", "._user_heap_stack") and v}
flash = sum(flash_secs.values())
ram = sum(ram_secs.values())
FLASH_TOTAL, RAM_TOTAL = 2 * 1024 * 1024, 128 * 1024  # STM32H743: 2 MB flash, 128 KB DTCM (used by RAM region in .ld)

pid_syms = [l for l in nm.splitlines() if l.endswith(" pid_step") or l.endswith(" pid_init")]
pid_bytes = sum(int(l.split()[1], 16) for l in pid_syms)

fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
fig.suptitle("STM32H743 firmware footprint  (arm-none-eabi-gcc -Os, real cross-build of main.elf)", fontsize=12)
for a, secs, total, name in ((ax[0], flash_secs, FLASH_TOTAL, "FLASH"), (ax[1], ram_secs, RAM_TOTAL, "RAM (DTCM)")):
    left = 0
    for k, v in secs.items():
        a.barh([0], [v], left=left, label=f"{k} {v} B")
        left += v
    a.set_xlim(0, max(left * 1.3, 1))
    a.set_yticks([])
    a.set_xlabel("bytes")
    a.set_title(f"{name}: {left} B used of {total // 1024} KB  ({100 * left / total:.2f} %)")
    a.legend(loc="upper right", fontsize=8)
ax[0].text(0.01, -0.35, f"PID control law (pid_init + pid_step, src/pid.c): {pid_bytes} bytes of Thumb-2 code",
           transform=ax[0].transAxes, fontsize=9)
fig.tight_layout()
fig.savefig(out_png, dpi=130)
print(size)
print(f"FLASH used: {flash} B  RAM used: {ram} B  PID code: {pid_bytes} B  -> {out_png}")
