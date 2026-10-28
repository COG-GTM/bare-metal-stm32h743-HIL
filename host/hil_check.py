#!/usr/bin/env python3
"""Requirement-style pass/fail gate on a HIL run (host/plant.py CSV).

    hil_check.py RUN.csv                 step-response + actuator requirements
    hil_check.py RUN.csv --equiv REF.csv  target-vs-host equivalence + actuator

Exits non-zero if any requirement fails. Appends a Markdown table to
$GITHUB_STEP_SUMMARY when set, and writes RUN.req.json next to the CSV.
"""
import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plant import KT2MS, step_metrics  # noqa: E402

# id, description, metric key, limit, comparison
STEP_REQS = [
    ("REQ-SPD-01", "rise time (90%) to TAS step", "rise_s", 12.0, "<=", "s"),
    ("REQ-SPD-02", "TAS overshoot", "overshoot_pct", 15.0, "<=", "%"),
    ("REQ-SPD-03", "settling time (2% band)", "settle_s", 60.0, "<=", "s"),
    ("REQ-SPD-04", "steady-state TAS error (abs) at end of run", "abs_final_err_kt", 0.5, "<=", "kt"),
]
ACT_REQS = [
    ("REQ-ACT-01", "thrust command never negative", "min_thrust_N", 0.0, ">=", "N"),
    ("REQ-ACT-02", "thrust command within 40 kN engine limit", "max_thrust_N", 40e3, "<=", "N"),
]
EQUIV_REQS = [
    ("REQ-EQV-01", "max thrust diff (abs) target vs host", "max_dthrust_N", 1.0, "<=", "N"),
    ("REQ-EQV-02", "max TAS diff (abs) target vs host", "max_dtas_kt", 0.01, "<=", "kt"),
]


def load(path):
    with open(path) as f:
        return [tuple(float(x) for x in row) for row in list(csv.reader(f))[1:]]


def step_time_of(log):
    """First sample where the setpoint changes; 0 if it never does."""
    return next((r[0] for prev, r in zip(log, log[1:]) if r[2] != prev[2]), 0.0)


def evaluate(log, ref=None):
    u = [r[3] for r in log]
    metrics = {"min_thrust_N": min(u), "max_thrust_N": max(u), "samples": len(log)}
    reqs = list(ACT_REQS)
    if ref is None:
        m = step_metrics(log, step_time_of(log))
        metrics.update(m)
        metrics["abs_final_err_kt"] = abs(m["final_err_kt"])
        reqs = STEP_REQS + reqs
    else:
        if len(ref) != len(log):
            sys.exit(f"sample count mismatch: {len(log)} vs {len(ref)}")
        metrics["max_dthrust_N"] = max(abs(a[3] - b[3]) for a, b in zip(log, ref))
        metrics["max_dtas_kt"] = max(abs(a[1] - b[1]) for a, b in zip(log, ref)) / KT2MS
        metrics["bit_identical_samples"] = sum(a[3] == b[3] for a, b in zip(log, ref))
        reqs = EQUIV_REQS + reqs
    results = []
    for rid, desc, key, limit, op, unit in reqs:
        val = metrics.get(key)
        ok = val is not None and (val <= limit if op == "<=" else val >= limit)
        results.append(dict(id=rid, desc=desc, value=val, limit=limit, op=op, unit=unit, ok=ok))
    return metrics, results


def report(name, metrics, results):
    def fmt(v):
        return "n/a" if v is None else f"{v:.4g}"

    rows = [f"| {'PASS' if r['ok'] else 'FAIL'} | {r['id']} | {r['desc']} | "
            f"{fmt(r['value'])} {r['unit']} | {r['op']} {r['limit']:g} {r['unit']} |" for r in results]
    n_ok = sum(r["ok"] for r in results)
    md = [f"### HIL requirements: `{name}` — {n_ok}/{len(results)} pass", "",
          "| | ID | Requirement | Measured | Limit |", "|---|---|---|---|---|", *rows]
    if "bit_identical_samples" in metrics:
        md += ["", f"{metrics['bit_identical_samples']}/{metrics['samples']} thrust samples bit-identical."]
    text = "\n".join(md)
    print(text)
    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary:
        with open(summary, "a") as f:
            f.write(text + "\n\n")
    return n_ok == len(results)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run")
    p.add_argument("--equiv", metavar="REF.csv", help="reference run to compare against")
    args = p.parse_args()
    metrics, results = evaluate(load(args.run), load(args.equiv) if args.equiv else None)
    ok = report(os.path.basename(args.run), metrics, results)
    with open(os.path.splitext(args.run)[0] + ".req.json", "w") as f:
        json.dump(dict(metrics=metrics, requirements=results, passed=ok), f, indent=2)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
