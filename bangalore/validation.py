"""
Log-driven pass/fail validation reports - deliberately kept separate from
report.py. report.py answers "how does this run compare to other runs?";
this module answers a narrower, more binary question: "did this specific
run actually work correctly?" A run can be a perfectly valid data point for
report.py's comparison table while still failing a check here (e.g. a
signal that never went green, or an ambulance that never arrived) - so both
tools are needed side by side, not one replacing the other.

It works by reading the plain-text console output of a run (the same
stdout+stderr transcript you get from `python <script>.py > run.log 2>&1`)
and extracting evidence with regexes - no re-simulation, no dependency on
run_records/. That means it works just as well on a historical log sitting
on disk from weeks ago as it does on a run happening right now.

Two report types, matching the two things worth validating independently:

  1. VAC report - "is the core traffic-signal controller (SCOSCA) actually
     working?" Source: diagnose_stuck_signals.py's log.
  2. EV report - "is the emergency-vehicle green corridor actually working?"
     Source: demo_Bangalore_Ambulance.py's log.

`capture_run_log(path)` is the "for every run" half: a context manager that
redirects this process's real stdout+stderr (including everything a SUMO
subprocess launched via traci.start prints, since a subprocess inherits the
parent's OS file descriptors - not just Python's own print() calls) to a
file for the duration of a block, so scripts can capture a complete,
parseable transcript of themselves and validate it automatically at the end
without the caller needing to remember to redirect the shell manually.

Usage (after the fact, on any existing log):
    python validation.py --vac-log bangalore/stuck_signal_diag3.log
    python validation.py --ev-log bangalore/ambulance_run5.log
    python validation.py --vac-log X.log --ev-log Y.log --out-dir DIR

Usage (from another script, to auto-validate its own run):
    from validation import capture_run_log, validate_vac_log
    with capture_run_log(log_path):
        ... run the simulation, print() as usual ...
    validate_vac_log(log_path)
"""

import argparse
import contextlib
import os
import re
import sys
from datetime import datetime
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
VALIDATION_REPORTS_DIR = BASE_DIR / "validation_reports"


@contextlib.contextmanager
def capture_run_log(log_path):
    """Redirects the OS-level stdout+stderr (file descriptors 1 and 2) to
    `log_path` for the duration of the block. Uses os.dup2 rather than
    reassigning sys.stdout so that output from subprocesses (SUMO, launched
    via traci.start) is captured too, not just this process's own print()
    calls - a plain `sys.stdout = ...` swap would miss it entirely."""
    log_path = Path(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    sys.stdout.flush()
    sys.stderr.flush()
    saved_stdout_fd = os.dup(1)
    saved_stderr_fd = os.dup(2)
    log_fd = os.open(str(log_path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
    os.dup2(log_fd, 1)
    os.dup2(log_fd, 2)
    try:
        yield log_path
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os.dup2(saved_stdout_fd, 1)
        os.dup2(saved_stderr_fd, 2)
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(log_fd)


def _read_log(log_path):
    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        return f.read()


def _crash_facts(text):
    return {
        "has_traceback": "Traceback (most recent call last):" in text,
        "has_fatal_traci_error": "FatalTraCIError" in text,
    }


# ---------------------------------------------------------------------------
# VAC (traffic-signal controller) validation - source: diagnose_stuck_signals.py
# ---------------------------------------------------------------------------

def parse_vac_log(text):
    facts = dict(_crash_facts(text))

    m = re.search(r"=== Stuck-signal diagnostic over (\d+)s \((\d+) traffic lights\) ===", text)
    facts["configured_duration_sec"] = int(m.group(1)) if m else None
    facts["n_traffic_lights"] = int(m.group(2)) if m else None

    m = re.search(r"Simulation ended at time:\s*(\d+\.\d+)", text)
    facts["sim_end_time_sec"] = float(m.group(1)) if m else None

    never_green_block = re.search(
        r"Traffic lights that NEVER showed any green state at all \(genuinely broken\):\n(.*?)(?:\n\n|\Z)",
        text, re.S)
    facts["never_green_ids"] = _extract_list(never_green_block)

    stuck_block = re.search(
        r"Traffic lights with a phase-change gap longer than ([\d.]+)s \(possibly stuck for a while\):\n(.*?)(?:\n\n|\Z)",
        text, re.S)
    facts["stuck_threshold_sec"] = float(stuck_block.group(1)) if stuck_block else None
    facts["stuck_entries"] = _extract_list(stuck_block, group=2) if stuck_block else []

    m = re.search(r"Saved run record:\s*(\S+)", text)
    facts["run_id"] = m.group(1) if m else None

    return facts


def _extract_list(match, group=1):
    if not match:
        return []
    items = []
    for line in match.group(group).splitlines():
        line = line.strip()
        if line and line.lower() != "none":
            items.append(line)
    return items


def evaluate_vac(facts):
    checks = []
    checks.append((
        "Run completed without a Python crash",
        not facts["has_traceback"],
        "No traceback found in the log." if not facts["has_traceback"]
        else "A Python traceback was found - the run crashed before finishing; treat all other results here as unreliable.",
    ))
    checks.append((
        "No TraCI connection failures",
        not facts["has_fatal_traci_error"],
        "No FatalTraCIError found." if not facts["has_fatal_traci_error"]
        else "FatalTraCIError found - the SUMO connection dropped at some point during the run.",
    ))
    if facts["configured_duration_sec"] and facts["sim_end_time_sec"] is not None:
        ran_full_duration = facts["sim_end_time_sec"] >= facts["configured_duration_sec"] - 1
        checks.append((
            "Simulation ran its full configured duration",
            ran_full_duration,
            f"Configured for {facts['configured_duration_sec']}s, simulation clock reached {facts['sim_end_time_sec']:.2f}s.",
        ))
    checks.append((
        "No traffic light ever failed to turn green",
        len(facts["never_green_ids"]) == 0,
        "All traffic lights showed a green state at least once."
        if not facts["never_green_ids"] else
        f"{len(facts['never_green_ids'])} traffic light(s) never showed green - see list below.",
    ))
    if facts["stuck_threshold_sec"] is not None:
        checks.append((
            f"No traffic light exceeded the {facts['stuck_threshold_sec']:.0f}s stuck-signal gap threshold",
            len(facts["stuck_entries"]) == 0,
            "No traffic light's longest phase-change gap exceeded the threshold."
            if not facts["stuck_entries"] else
            f"{len(facts['stuck_entries'])} traffic light(s) exceeded it - see list below.",
        ))
    overall = all(passed for _, passed, _ in checks)
    return overall, checks


def render_vac_markdown(facts, checks, overall, source_log):
    lines = [
        "# VAC validation report",
        "",
        "Validates the core adaptive traffic-signal controller (SCOSCA) against a "
        "`diagnose_stuck_signals.py` run: does every traffic light actually cycle "
        "correctly, with no signal stuck or permanently unlit?",
        "",
        f"- **Source log:** `{source_log}`",
        f"- **Generated:** {datetime.now().isoformat(timespec='seconds')}",
    ]
    if facts.get("run_id"):
        lines.append(f"- **Run record:** `{facts['run_id']}` (see `run_records/`, comparable via `python report.py --run-type stuck_signal_diag`)")
    if facts.get("n_traffic_lights") is not None:
        lines.append(f"- **Traffic lights checked:** {facts['n_traffic_lights']}")
    lines += [
        "",
        f"## Verdict: {'PASS' if overall else 'FAIL'}",
        "",
        "| Check | Result | Detail |",
        "|---|---|---|",
    ]
    for name, passed, detail in checks:
        lines.append(f"| {name} | {'pass' if passed else 'FAIL'} | {detail} |")
    lines.append("")

    if facts["never_green_ids"]:
        lines.append("### Traffic lights that never turned green")
        lines += [f"- `{tl}`" for tl in facts["never_green_ids"]]
        lines.append("")
    if facts["stuck_entries"]:
        lines.append("### Traffic lights exceeding the stuck-signal gap threshold")
        lines += [f"- {entry}" for entry in facts["stuck_entries"]]
        lines.append("")

    lines.append("---")
    lines.append("*Generated by validation.py from a plain-text run log - no re-simulation involved.*")
    return "\n".join(lines)


def validate_vac_log(log_path, out_dir=VALIDATION_REPORTS_DIR):
    text = _read_log(log_path)
    facts = parse_vac_log(text)
    overall, checks = evaluate_vac(facts)
    md = render_vac_markdown(facts, checks, overall, log_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"vac_validation__{Path(log_path).stem}.md"
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(md)
    return out_path, overall


# ---------------------------------------------------------------------------
# EV (emergency-vehicle green corridor) validation - source: demo_Bangalore_Ambulance.py
# ---------------------------------------------------------------------------

def _parse_num_or_none(raw):
    raw = raw.strip()
    return None if raw == "None" else float(raw)


def parse_ev_log(text):
    facts = dict(_crash_facts(text))

    m = re.search(r"Ambulance route:\s*(\S+)\s*->\s*(\S+)\s*\((\d+) edges\)", text)
    if m:
        facts["route_from"], facts["route_to"], facts["route_edges"] = m.group(1), m.group(2), int(m.group(3))
    else:
        facts["route_from"] = facts["route_to"] = facts["route_edges"] = None

    m = re.search(r"Ambulance travel time WITHOUT corridor:\s*(\S+)", text)
    facts["travel_time_without_s"] = _parse_num_or_none(m.group(1)) if m else None
    m = re.search(r"Ambulance travel time WITH corridor:\s*(\S+)", text)
    facts["travel_time_with_s"] = _parse_num_or_none(m.group(1)) if m else None

    m = re.search(r"Preemption events \(with corridor\):\s*(\d+)", text)
    facts["preemption_events"] = int(m.group(1)) if m else None

    m = re.search(r"Background avg waitingTime WITHOUT corridor:\s*([\d.]+)", text)
    facts["bg_waiting_without_s"] = float(m.group(1)) if m else None
    m = re.search(r"Background avg waitingTime WITH corridor:\s*([\d.]+)", text)
    facts["bg_waiting_with_s"] = float(m.group(1)) if m else None

    ids = re.search(r"Saved run records:\s*(\S+),\s*(\S+)", text)
    facts["run_ids"] = [ids.group(1).rstrip(","), ids.group(2)] if ids else []

    return facts


def evaluate_ev(facts, bg_regression_tolerance_pct=10.0):
    checks = []
    checks.append((
        "Run completed without a Python crash",
        not facts["has_traceback"],
        "No traceback found in the log." if not facts["has_traceback"]
        else "A Python traceback was found - treat all other results here as unreliable.",
    ))
    checks.append((
        "No TraCI connection failures",
        not facts["has_fatal_traci_error"],
        "No FatalTraCIError found." if not facts["has_fatal_traci_error"]
        else "FatalTraCIError found - the SUMO connection dropped during at least one scenario.",
    ))

    both_completed = facts["travel_time_without_s"] is not None and facts["travel_time_with_s"] is not None
    checks.append((
        "Ambulance completed its trip in both scenarios",
        both_completed,
        "Both the no-corridor and with-corridor runs produced a travel time." if both_completed else
        "The ambulance never reached its destination in at least one scenario - increase --duration or check the route.",
    ))

    if both_completed:
        without, with_ = facts["travel_time_without_s"], facts["travel_time_with_s"]
        improved = with_ < without
        pct_change = ((with_ - without) / without * 100) if without else 0.0
        word = "faster" if pct_change < 0 else "slower"
        checks.append((
            "Green corridor reduced ambulance travel time",
            improved,
            f"{without:.1f}s -> {with_:.1f}s ({abs(pct_change):.1f}% {word}).",
        ))

    if facts["preemption_events"] is not None:
        fired = facts["preemption_events"] > 0
        checks.append((
            "Preemption actually engaged at least once",
            fired,
            f"{facts['preemption_events']} preemption event(s) recorded." if fired else
            "Zero preemption events - the corridor mechanism never actually triggered, so any travel-time improvement isn't from preemption.",
        ))

    if facts["bg_waiting_without_s"] is not None and facts["bg_waiting_with_s"] is not None:
        delta = facts["bg_waiting_with_s"] - facts["bg_waiting_without_s"]
        pct = (delta / facts["bg_waiting_without_s"] * 100) if facts["bg_waiting_without_s"] else 0.0
        no_regression = pct <= bg_regression_tolerance_pct
        checks.append((
            f"No significant background-traffic regression (<= {bg_regression_tolerance_pct:.0f}% worse avg. waiting time)",
            no_regression,
            f"{facts['bg_waiting_without_s']:.2f}s -> {facts['bg_waiting_with_s']:.2f}s ({pct:+.1f}%) for all other vehicles.",
        ))

    overall = all(passed for _, passed, _ in checks)
    return overall, checks


def render_ev_markdown(facts, checks, overall, source_log):
    lines = [
        "# EV (emergency-vehicle green corridor) validation report",
        "",
        "Validates the ambulance green-corridor feature against a "
        "`demo_Bangalore_Ambulance.py` run: did the ambulance actually get through "
        "faster, via real preemption events, without meaningfully disrupting "
        "background traffic?",
        "",
        f"- **Source log:** `{source_log}`",
        f"- **Generated:** {datetime.now().isoformat(timespec='seconds')}",
    ]
    if facts.get("route_from"):
        lines.append(f"- **Route:** `{facts['route_from']}` &rarr; `{facts['route_to']}` ({facts['route_edges']} edges)")
    if facts.get("run_ids"):
        lines.append(f"- **Run records:** `{'`, `'.join(facts['run_ids'])}` (comparable via `python report.py --run-type ambulance`)")
    lines += [
        "",
        f"## Verdict: {'PASS' if overall else 'FAIL'}",
        "",
        "| Check | Result | Detail |",
        "|---|---|---|",
    ]
    for name, passed, detail in checks:
        lines.append(f"| {name} | {'pass' if passed else 'FAIL'} | {detail} |")
    lines.append("")
    lines.append("---")
    lines.append("*Generated by validation.py from a plain-text run log - no re-simulation involved.*")
    return "\n".join(lines)


def validate_ev_log(log_path, out_dir=VALIDATION_REPORTS_DIR):
    text = _read_log(log_path)
    facts = parse_ev_log(text)
    overall, checks = evaluate_ev(facts)
    md = render_ev_markdown(facts, checks, overall, log_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"ev_validation__{Path(log_path).stem}.md"
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(md)
    return out_path, overall


def _cli():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--vac-log", type=str, default=None, help="path to a diagnose_stuck_signals.py log")
    parser.add_argument("--ev-log", type=str, default=None, help="path to a demo_Bangalore_Ambulance.py log")
    parser.add_argument("--out-dir", type=str, default=str(VALIDATION_REPORTS_DIR))
    args = parser.parse_args()

    if not args.vac_log and not args.ev_log:
        parser.error("pass at least one of --vac-log / --ev-log")

    if args.vac_log:
        out_path, overall = validate_vac_log(args.vac_log, args.out_dir)
        print(f"VAC validation: {'PASS' if overall else 'FAIL'} -> {out_path}")
    if args.ev_log:
        out_path, overall = validate_ev_log(args.ev_log, args.out_dir)
        print(f"EV validation: {'PASS' if overall else 'FAIL'} -> {out_path}")


if __name__ == "__main__":
    _cli()
