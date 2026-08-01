"""
Shared, future-proof run reporting for every EVATO/bangalore simulation.

The problem this solves: every script we've written (verify_scosca_tuning.py,
verify_scosca_scaleup.py, demo_Bangalore_Ambulance.py, diagnose_stuck_signals.py,
...) computed and displayed its own metrics in its own bespoke way, so no run
from one script could ever be lined up against a run from another, and a new
metric added later had nowhere consistent to live.

This module gives every one of those runs one shared vocabulary:

  1. METRIC_CATALOG — a single registry of "what this number means", each
     with a one-line, plain-English description and a comparison direction
     (higher/lower/neither is better). New metrics are just new dict entries;
     nothing else has to change, and any record that doesn't have a given
     metric simply renders that cell as "-". That's what makes this
     future-proof: old records never need migrating when new metrics appear.

  2. new_run_record(...) / save_run_record(...) — any script calls these
     after it finishes to persist a small standard-shaped JSON file to
     run_records/. That directory accumulates EVERY run, forever, across
     every script.

  3. load_run_records(...) / render_comparison_report_html(...) /
     build_comparison_report(...) — load some or all saved records (by
     run_type, by explicit id, or everything) and render one HTML table that
     lines them up side by side, with each metric's one-liner shown inline
     and a simple better/worse indicator relative to the first record.

Usage from another script:

    from report import new_run_record, save_run_record, tripinfo_list_metrics

    metrics = tripinfo_list_metrics(trips)
    metrics["ambulance_travel_time_s"] = travel_time
    record = new_run_record("ambulance", "with_corridor", metrics,
                             params=SCOSCA_PARAMS, seed=42, duration_sec=900)
    save_run_record(record)

Standalone:
    python report.py --list
    python report.py --run-type ambulance --out ambulance_compare.html
    python report.py --out all_runs.html   # every saved run, all types
"""

import argparse
import json
import os
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
RUN_RECORDS_DIR = BASE_DIR / "run_records"


# ---------------------------------------------------------------------------
# Metric catalog: the single source of truth for "what does this number mean
# and is bigger better". Add new keys here as new metrics come up — anything
# not listed still renders (generic fallback below), it just won't have a
# real description yet.
# ---------------------------------------------------------------------------
METRIC_CATALOG = {
    "avg_speed_mps": {
        "label": "Avg. speed", "unit": "m/s", "direction": "higher_better",
        "description": "Average vehicle speed across completed trips - higher means less time stuck at lights or in queues.",
    },
    "avg_trip_duration_s": {
        "label": "Avg. trip duration", "unit": "s", "direction": "lower_better",
        "description": "Average wall-clock time to complete a trip, start to finish.",
    },
    "avg_waiting_time_s": {
        "label": "Avg. waiting time", "unit": "s", "direction": "lower_better",
        "description": "Average time each vehicle spent stopped at signals - the number SCOSCA/CoSiCoSt tuning most directly targets.",
    },
    "avg_time_loss_s": {
        "label": "Avg. time loss", "unit": "s", "direction": "lower_better",
        "description": "Average delay vs. free-flow travel time per trip - captures stops AND slow-downs, not just full stops.",
    },
    "avg_route_length_m": {
        "label": "Avg. route length", "unit": "m", "direction": "context",
        "description": "Average trip distance - context only; if this shifts a lot between runs, raw time-based averages aren't a fair comparison (see time-loss/meter).",
    },
    "timeloss_per_meter": {
        "label": "Time loss / meter", "unit": "s/m", "direction": "lower_better",
        "description": "Time loss normalized by trip distance - the fair way to compare runs whose trip-length mix differs.",
    },
    "n_trips_completed": {
        "label": "Trips completed", "unit": "trips", "direction": "higher_better",
        "description": "Number of vehicle trips that finished inside the simulated window - also affects whether other raw averages are comparable (throughput bias).",
    },
    "avg_depart_delay_s": {
        "label": "Avg. depart delay", "unit": "s", "direction": "lower_better",
        "description": "Average time a vehicle waited to even enter the network after being scheduled - high values usually mean network-entry gridlock, not signal timing.",
    },
    "ambulance_travel_time_s": {
        "label": "Ambulance travel time", "unit": "s", "direction": "lower_better",
        "description": "Wall-clock time for the emergency vehicle to complete its corridor route - the headline number for the green-corridor feature.",
    },
    "preemption_events": {
        "label": "Preemption events", "unit": "events", "direction": "context",
        "description": "Number of traffic-light preemption engage/release events triggered by the emergency vehicle - context, not itself good or bad.",
    },
    "n_signals_never_green": {
        "label": "Signals never green", "unit": "signals", "direction": "lower_better",
        "description": "Count of traffic lights that never showed a green state during the run - should always be 0; nonzero means a genuine controller bug.",
    },
    "max_phase_gap_s": {
        "label": "Longest phase-change gap", "unit": "s", "direction": "lower_better",
        "description": "Longest time any single traffic light went without changing phase - a spike far beyond the configured cycle length indicates a stuck signal.",
    },
    "n_signals_flagged_gap": {
        "label": "Signals flagged (long gap)", "unit": "signals", "direction": "lower_better",
        "description": "Count of traffic lights whose longest phase-change gap exceeded the stuck-signal threshold used for that run.",
    },
    "n_traffic_lights": {
        "label": "Traffic lights checked", "unit": "signals", "direction": "context",
        "description": "Total number of traffic lights covered by this run - context for the two metrics above.",
    },
    "fallback_events_triggered": {
        "label": "Fallback events", "unit": "events", "direction": "context",
        "description": "Number of times a district fell back to Full Vehicle Actuation due to (real or simulated) detector/comms data loss.",
    },
}

_GENERIC_METRIC = {"label": None, "unit": "", "direction": "context",
                    "description": "(no description registered for this metric yet - add one to METRIC_CATALOG in report.py)"}


def _metric_info(key):
    info = METRIC_CATALOG.get(key, _GENERIC_METRIC)
    if info["label"] is None:
        info = dict(info, label=key)
    return info


# ---------------------------------------------------------------------------
# Run records: one small JSON file per run, accumulated forever in
# run_records/. This is what makes "compare every run" possible later -
# nothing needs to be re-run to compare against something done weeks ago.
# ---------------------------------------------------------------------------

def new_run_record(run_type, label, metrics, params=None, seed=None,
                    duration_sec=None, notes=None, run_id=None):
    """Builds a standard-shaped run record. `run_type` groups comparable runs
    (e.g. "ambulance", "scosca_tuning", "stuck_signal_diag", "scaleup").
    `label` distinguishes runs within a comparison (e.g. "baseline"/"tuned",
    "no_corridor"/"with_corridor"). `metrics` is a flat dict of
    metric_key -> numeric value; keys should match METRIC_CATALOG where
    possible but any key is accepted."""
    timestamp = datetime.now()
    if run_id is None:
        run_id = f"{run_type}__{label}__{timestamp.strftime('%Y%m%d_%H%M%S')}"
    return {
        "run_id": run_id,
        "run_type": run_type,
        "label": label,
        "timestamp": timestamp.isoformat(timespec="seconds"),
        "duration_sec": duration_sec,
        "seed": seed,
        "params": params or {},
        "metrics": metrics,
        "notes": notes or [],
    }


def save_run_record(record, run_records_dir=RUN_RECORDS_DIR):
    run_records_dir = Path(run_records_dir)
    run_records_dir.mkdir(parents=True, exist_ok=True)
    path = run_records_dir / f"{record['run_id']}.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(record, f, indent=2, default=str)
    return path


def load_run_records(run_records_dir=RUN_RECORDS_DIR, run_type=None, run_ids=None):
    run_records_dir = Path(run_records_dir)
    if not run_records_dir.exists():
        return []
    records = []
    for path in sorted(run_records_dir.glob("*.json")):
        try:
            with open(path, "r", encoding="utf-8") as f:
                record = json.load(f)
        except (json.JSONDecodeError, OSError):
            continue
        if run_type and record.get("run_type") != run_type:
            continue
        if run_ids and record.get("run_id") not in run_ids:
            continue
        records.append(record)
    records.sort(key=lambda r: r.get("timestamp", ""))
    return records


# ---------------------------------------------------------------------------
# Metric computation helpers shared across scripts, so "avg waiting time"
# means exactly the same thing everywhere it's reported.
# ---------------------------------------------------------------------------

def load_tripinfos_xml(path, exclude_id=None):
    """Parses a SUMO --tripinfo-output file into a list of plain dicts."""
    if not os.path.exists(path):
        return []
    root = ET.parse(path).getroot()
    trips = []
    for t in root.findall("tripinfo"):
        if exclude_id and t.get("id") == exclude_id:
            continue
        trips.append({
            "id": t.get("id"),
            "routeLength": float(t.get("routeLength")),
            "duration": float(t.get("duration")),
            "waitingTime": float(t.get("waitingTime")),
            "timeLoss": float(t.get("timeLoss")),
        })
    return trips


def tripinfo_list_metrics(trips):
    """Standard metric set computed directly from a list of tripinfo dicts
    (see load_tripinfos_xml) - this is the only metrics helper that can also
    produce timeloss_per_meter and n_trips_completed, since those need the
    raw per-trip list rather than SUMO's own aggregate statistics."""
    if not trips:
        return {"n_trips_completed": 0}
    n = len(trips)
    avg_speed = sum(t["routeLength"] / t["duration"] for t in trips if t["duration"] > 0) / n
    return {
        "avg_speed_mps": avg_speed,
        "avg_trip_duration_s": sum(t["duration"] for t in trips) / n,
        "avg_waiting_time_s": sum(t["waitingTime"] for t in trips) / n,
        "avg_time_loss_s": sum(t["timeLoss"] for t in trips) / n,
        "avg_route_length_m": sum(t["routeLength"] for t in trips) / n,
        "timeloss_per_meter": sum(t["timeLoss"] / max(1.0, t["routeLength"]) for t in trips) / n,
        "n_trips_completed": n,
    }


def stats_xml_metrics(stats_path):
    """Standard metric set from a SUMO --statistic-output aggregate XML file
    (vehicleTripStatistics). Use this when you don't have (or don't need) the
    raw per-trip list - it's what SUMO itself already averaged."""
    if not os.path.exists(stats_path):
        return {}
    root = ET.parse(stats_path).getroot()
    el = root.find("vehicleTripStatistics")
    if el is None:
        return {}
    return {
        "avg_route_length_m": float(el.get("routeLength", 0.0)),
        "avg_speed_mps": float(el.get("speed", 0.0)),
        "avg_trip_duration_s": float(el.get("duration", 0.0)),
        "avg_waiting_time_s": float(el.get("waitingTime", 0.0)),
        "avg_time_loss_s": float(el.get("timeLoss", 0.0)),
        "avg_depart_delay_s": float(el.get("departDelay", 0.0)),
    }


# ---------------------------------------------------------------------------
# Shared visual system: every report produced by any script (ambulance,
# scosca_tuning, stuck_signal_diag, scaleup, ...) uses this same stylesheet
# and page shell, so a "run report" always looks and reads the same way no
# matter which script produced it. Colors are tokenized as custom properties
# so light/dark both stay legible; the amber accent is decorative, the
# green/red pair is reserved for good/bad deltas and never reused decoratively.
# ---------------------------------------------------------------------------
REPORT_BASE_CSS = """
:root {
  --ink: #1b2027;
  --ink-soft: #545b62;
  --paper: #eef1f0;
  --panel: #ffffff;
  --line: #d7dbd8;
  --accent: #b5691f;
  --good: #1f8a5c;
  --bad: #b23b3b;
}
@media (prefers-color-scheme: dark) {
  :root {
    --ink: #e8e6df;
    --ink-soft: #a4aaa7;
    --paper: #14171a;
    --panel: #1b1f22;
    --line: #2b2f33;
    --accent: #e0983f;
    --good: #4cbf8e;
    --bad: #e0827f;
  }
}
:root[data-theme="dark"] {
  --ink: #e8e6df; --ink-soft: #a4aaa7; --paper: #14171a; --panel: #1b1f22;
  --line: #2b2f33; --accent: #e0983f; --good: #4cbf8e; --bad: #e0827f;
}
:root[data-theme="light"] {
  --ink: #1b2027; --ink-soft: #545b62; --paper: #eef1f0; --panel: #ffffff;
  --line: #d7dbd8; --accent: #b5691f; --good: #1f8a5c; --bad: #b23b3b;
}
.evato-report { background: var(--paper); color: var(--ink);
  font-family: -apple-system, "Segoe UI", "Helvetica Neue", Arial, sans-serif;
  line-height: 1.5; padding: 2.5rem 1.5rem 4rem; }
.evato-report .report-inner { max-width: 880px; margin: 0 auto; }
.evato-report h1 { font-family: Georgia, "Iowan Old Style", "Palatino Linotype", serif;
  font-size: 2rem; font-weight: 600; margin: 0 0 0.4rem; text-wrap: balance; }
.evato-report h2 { font-family: Georgia, "Iowan Old Style", "Palatino Linotype", serif;
  font-size: 1.3rem; font-weight: 600; margin: 2.4rem 0 0.6rem; padding-top: 1.4rem;
  border-top: 1px solid var(--line); text-wrap: balance; }
.evato-report h2:first-of-type { border-top: none; padding-top: 0; }
.evato-report p { color: var(--ink-soft); max-width: 65ch; margin: 0 0 0.9rem; }
.evato-report code { font-family: "SFMono-Regular", Consolas, "Liberation Mono", monospace;
  font-size: 0.85em; background: var(--panel); border: 1px solid var(--line);
  border-radius: 3px; padding: 0.05em 0.35em; }
.evato-report .report-meta { display: flex; flex-wrap: wrap; gap: 0.4rem 1.2rem;
  font-size: 0.85rem; color: var(--ink-soft); margin-bottom: 1.6rem; }
.evato-report .report-meta span b { color: var(--ink); font-weight: 600; }
.evato-report figure { margin: 1rem 0; background: var(--panel); border: 1px solid var(--line);
  border-radius: 8px; padding: 1rem; }
.evato-report figure img { max-width: 100%; display: block; margin: 0 auto; }
.evato-report figcaption { color: var(--ink-soft); font-size: 0.85rem; margin-top: 0.6rem; max-width: 65ch; }
.evato-report .table-scroll { overflow-x: auto; border: 1px solid var(--line); border-radius: 8px; }
.evato-report table { border-collapse: collapse; width: 100%; background: var(--panel); font-variant-numeric: tabular-nums; }
.evato-report th, .evato-report td { padding: 0.55rem 0.8rem; text-align: left;
  border-bottom: 1px solid var(--line); vertical-align: top; }
.evato-report thead th { font-size: 0.85rem; color: var(--ink-soft); font-weight: 600; }
.evato-report thead .run-timestamp { display: block; font-weight: normal; font-size: 0.75rem; color: var(--ink-soft); margin-top: 0.15rem; }
.evato-report tr:last-child td { border-bottom: none; }
.evato-report .meta-row td { color: var(--ink-soft); font-size: 0.85rem; font-style: italic; }
.evato-report .metric-name { max-width: 260px; }
.evato-report .metric-name b { display: block; }
.evato-report .metric-desc { font-size: 0.78rem; color: var(--ink-soft); font-weight: normal; margin-top: 0.15rem; }
.evato-report .delta-good { color: var(--good); font-size: 0.85rem; }
.evato-report .delta-bad { color: var(--bad); font-size: 0.85rem; }
.evato-report .delta-neutral { color: var(--ink-soft); font-size: 0.85rem; }
.evato-report footer { margin-top: 2.5rem; padding-top: 1rem; border-top: 1px solid var(--line);
  font-size: 0.78rem; color: var(--ink-soft); }
"""


def render_report_page(title, description, body_html, meta=None):
    """Wraps `body_html` (any mix of <h2> sections, <figure> charts, tables,
    ...) in the shared report shell: stylesheet, title, description, and an
    optional meta strip (list of (label, value) pairs) shown under the
    title - e.g. duration, seed, route. Every script's report should end
    with this so all reports share one visual language."""
    meta_html = ""
    if meta:
        meta_html = '<div class="report-meta">' + "".join(
            f"<span>{label}: <b>{value}</b></span>" for label, value in meta
        ) + "</div>"
    return f"""<div class="evato-report"><style>{REPORT_BASE_CSS}</style>
<div class="report-inner">
<h1>{title}</h1>
<p>{description}</p>
{meta_html}
{body_html}
<footer>Generated by report.py - part of EVATO's shared run-reporting module.</footer>
</div></div>"""


# ---------------------------------------------------------------------------
# Rendering: one comparison table, N runs wide, every metric ever recorded
# tall. Works for 1 run (just shows values) or many (adds a vs.-first-run
# delta so regressions/improvements jump out).
# ---------------------------------------------------------------------------

def _fmt(value, unit):
    if value is None:
        return "-"
    if isinstance(value, float):
        text = f"{value:,.2f}"
    else:
        text = f"{value:,}"
    return f"{text} {unit}".strip()


def _delta_html(value, ref_value, direction):
    if value is None or ref_value is None or direction == "context":
        return ""
    try:
        diff = value - ref_value
    except TypeError:
        return ""
    if ref_value != 0:
        pct = diff / abs(ref_value) * 100
        pct_text = f"{pct:+.1f}%"
    else:
        pct_text = f"{diff:+.2f}"
    if abs(diff) < 1e-9:
        return '<span class="delta-neutral">(=)</span>'
    improved = (diff < 0) if direction == "lower_better" else (diff > 0)
    cls = "delta-good" if improved else "delta-bad"
    arrow = "&#9660;" if diff < 0 else "&#9650;"
    return f'<span class="{cls}">{arrow} {pct_text}</span>'


def render_comparison_table_html(records, baseline_run_id=None):
    """Returns a standalone table (wrapped for horizontal scroll) comparing
    `records` (list of run-record dicts, e.g. from load_run_records). The
    first record (or `baseline_run_id` if given) is treated as the reference
    for the better/worse indicator on every other column. Styling comes from
    REPORT_BASE_CSS via render_report_page - this function only emits
    semantic markup (classes, no inline styles) so it looks consistent
    wherever it's embedded."""
    if not records:
        return "<p><i>No run records to compare.</i></p>"

    ref_idx = 0
    if baseline_run_id:
        for i, r in enumerate(records):
            if r["run_id"] == baseline_run_id:
                ref_idx = i
                break

    # Union of every metric key seen across all records, catalog order first
    # (so common metrics line up in a stable, readable order), then any
    # not-yet-catalogued keys appended alphabetically - new metric types
    # never break rendering of older records.
    seen_keys = set()
    for r in records:
        seen_keys.update(r.get("metrics", {}).keys())
    ordered_keys = [k for k in METRIC_CATALOG if k in seen_keys]
    ordered_keys += sorted(seen_keys - set(ordered_keys))

    header_cells = "".join(
        f'<th>{r["label"]}<span class="run-timestamp">{r["timestamp"]}</span></th>'
        for r in records
    )

    rows = []
    meta_rows = [
        ("run_type", [r.get("run_type", "-") for r in records]),
        ("seed", [r.get("seed", "-") for r in records]),
        ("duration_sec", [r.get("duration_sec", "-") for r in records]),
    ]
    for meta_label, values in meta_rows:
        cells = "".join(f"<td>{v}</td>" for v in values)
        rows.append(f'<tr class="meta-row"><td><i>{meta_label}</i></td>{cells}</tr>')

    for key in ordered_keys:
        info = _metric_info(key)
        ref_value = records[ref_idx].get("metrics", {}).get(key)
        cells = []
        for i, r in enumerate(records):
            value = r.get("metrics", {}).get(key)
            cell = _fmt(value, info["unit"])
            if i != ref_idx and len(records) > 1:
                delta = _delta_html(value, ref_value, info["direction"])
                if delta:
                    cell += f"<br>{delta}"
            cells.append(f"<td>{cell}</td>")
        rows.append(
            f'<tr><td class="metric-name"><b>{info["label"]}</b>'
            f'<div class="metric-desc">{info["description"]}</div>'
            f'</td>{"".join(cells)}</tr>'
        )

    return f"""<div class="table-scroll"><table>
<thead><tr><th>Metric</th>{header_cells}</tr></thead>
<tbody>
{"".join(rows)}
</tbody>
</table></div>"""


def render_comparison_report_html(records, title, description=""):
    table_html = render_comparison_table_html(records)
    body = f"""{table_html}
<p style="font-size:0.78rem">Percentages shown under each value compare it against the first (leftmost) run.</p>"""
    return render_report_page(title, description, body,
                               meta=[("saved run records", len(records))])


def build_comparison_report(out_path, run_type=None, run_ids=None, title=None,
                             description="", run_records_dir=RUN_RECORDS_DIR):
    """Loads saved run records (optionally filtered) and writes one
    comparison HTML report to `out_path`. This is the 'compare every run
    we've ever done' entry point - it needs nothing re-run, just whatever
    scripts have already called save_run_record()."""
    records = load_run_records(run_records_dir, run_type=run_type, run_ids=run_ids)
    if title is None:
        title = f"Run comparison: {run_type}" if run_type else "Run comparison: all runs"
    html = render_comparison_report_html(records, title, description)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return out_path


def _cli():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="list every saved run record and exit")
    parser.add_argument("--run-type", type=str, default=None, help="filter to one run_type (e.g. ambulance)")
    parser.add_argument("--out", type=str, default=str(RUN_RECORDS_DIR / "comparison_report.html"))
    args = parser.parse_args()

    if args.list:
        records = load_run_records(run_type=args.run_type)
        if not records:
            print("No run records found in", RUN_RECORDS_DIR)
            return
        for r in records:
            print(f"{r['timestamp']}  {r['run_type']:<18} {r['label']:<18} {r['run_id']}")
        return

    out_path = build_comparison_report(args.out, run_type=args.run_type)
    print(f"Comparison report written to: {out_path}")


if __name__ == "__main__":
    _cli()
