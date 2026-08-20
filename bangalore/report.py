"""
Shared, future-proof run reporting for every EVATO/bangalore simulation.

The problem this solves: every script we've written (verify_scosca_tuning.py,
verify_scosca_scaleup.py, demo_Bangalore_Ambulance.py, diagnose_stuck_signals.py,
...) computed and displayed its own metrics in its own bespoke way, so no run
from one script could ever be lined up against a run from another, and a new
metric added later had nowhere consistent to live.

This module gives every one of those runs one shared vocabulary:

  1. METRIC_CATALOG - a single registry of "what this number means", each
     with a one-line, plain-English description and a comparison direction
     (higher/lower/neither is better). New metrics are just new dict entries;
     nothing else has to change, and any record that doesn't have a given
     metric simply renders that cell as "-". That's what makes this
     future-proof: old records never need migrating when new metrics appear.

  2. new_run_record(...) / save_run_record(...) - any script calls these
     after it finishes to persist a small standard-shaped JSON file to
     run_records/. That directory accumulates EVERY run, forever, across
     every script. The JSON is the machine-readable archive.

  3. save_run_report_md(...) - writes a human-readable Markdown report for
     ONE run into run_reports/: headline results, the scenario's start/end
     points, throughput, and every recorded metric with its explanation.
     This is what you actually read or hand to someone.

  4. build_comparison_report(...) - loads some or all saved records (by
     run_type, by explicit id, or everything) and writes ONE Markdown table
     lining them up side by side, with a better/worse indicator relative to
     the first record.

All output is Markdown (.md) - plain text, diffable, readable in any editor,
and pasteable into a doc. This module deliberately does NOT emit HTML.

Usage from another script:

    from report import new_run_record, save_run_record, save_run_report_md

    metrics = tripinfo_list_metrics(trips)
    metrics["ambulance_travel_time_s"] = travel_time
    record = new_run_record("ambulance", "with_corridor", metrics,
                             params=SCOSCA_PARAMS, seed=42, duration_sec=900,
                             context={"route_from": a, "route_to": b})
    save_run_record(record)      # JSON archive, for later comparison
    save_run_report_md(record)   # readable .md report for this run

Standalone:
    python report.py --list
    python report.py --run-type ambulance --out ambulance_compare.md
    python report.py --out all_runs.md          # every saved run, all types
    python report.py --per-run                  # (re)write a .md per record
"""

import argparse
import json
import os
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
RUN_RECORDS_DIR = BASE_DIR / "run_records"
RUN_REPORTS_DIR = BASE_DIR / "run_reports"


# ---------------------------------------------------------------------------
# Metric catalog: the single source of truth for "what does this number mean
# and is bigger better". Add new keys here as new metrics come up - anything
# not listed still renders (generic fallback below), it just won't have a
# real description yet.
# ---------------------------------------------------------------------------
METRIC_CATALOG = {
    "ambulance_travel_time_s": {
        "label": "Ambulance travel time", "unit": "s", "direction": "lower_better",
        "description": "Wall-clock time for the emergency vehicle to complete its route - the headline number for the green-corridor feature.",
    },
    "ambulance_route_length_m": {
        "label": "Ambulance route length", "unit": "m", "direction": "context",
        "description": "Distance the emergency vehicle actually travelled start to finish.",
    },
    "ambulance_avg_speed_kmh": {
        "label": "Ambulance avg. speed", "unit": "km/h", "direction": "higher_better",
        "description": "Route length divided by travel time - how freely the emergency vehicle moved overall.",
    },
    "ambulance_waiting_time_s": {
        "label": "Ambulance waiting time", "unit": "s", "direction": "lower_better",
        "description": "Time the emergency vehicle spent fully stopped (typically at red lights) - what the green corridor is meant to drive toward zero.",
    },
    "preemption_events": {
        "label": "Preemption events", "unit": "events", "direction": "context",
        "duration_sensitive": True,
        "description": "Number of traffic-light preemption engage/release events triggered by the emergency vehicle - context, not itself good or bad.",
    },
    "throughput_veh_per_hour": {
        "label": "Throughput", "unit": "veh/h", "direction": "higher_better",
        "description": "Completed trips scaled to an hourly rate - the headline measure of how much traffic the network actually cleared.",
    },
    "n_trips_completed": {
        "label": "Trips completed", "unit": "trips", "direction": "higher_better",
        # A raw count over the simulated window: a longer run trivially
        # completes more trips, so a percentage against a run of a different
        # length would compare window sizes, not performance. Use Throughput
        # (already per-hour) to compare runs of unequal duration.
        "duration_sensitive": True,
        "description": "Number of vehicle trips that finished inside the simulated window - a raw count, so only comparable between runs of equal duration (use Throughput otherwise).",
    },
    "avg_speed_kmh": {
        "label": "Avg. speed", "unit": "km/h", "direction": "higher_better",
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
    "timeloss_per_km": {
        "label": "Time loss / km", "unit": "s/km", "direction": "lower_better",
        "description": "Time loss normalized by trip distance - the fair way to compare runs whose trip-length mix differs.",
    },
    "avg_depart_delay_s": {
        "label": "Avg. depart delay", "unit": "s", "direction": "lower_better",
        "description": "Average time a vehicle waited to even enter the network after being scheduled - high values usually mean network-entry gridlock, not signal timing.",
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

# Metrics promoted into each report's "headline results" block, in this order.
# Anything here that the run didn't record is simply skipped.
HEADLINE_METRICS = [
    "ambulance_travel_time_s",
    "ambulance_avg_speed_kmh",
    "ambulance_waiting_time_s",
    "preemption_events",
    "throughput_veh_per_hour",
    "n_trips_completed",
    "avg_waiting_time_s",
    "n_signals_never_green",
    "max_phase_gap_s",
]

# Human labels for the free-form `context` dict (scenario details that aren't
# numeric metrics). Unknown keys still render, using the raw key as the label.
CONTEXT_LABELS = {
    "route_from": "Start edge",
    "route_to": "Destination edge",
    "route_edges": "Route length (edges)",
    "corridor_enabled": "Green corridor enabled",
    "ambulance_depart_s": "Ambulance departs at",
    "network_file": "Network file",
    "stuck_threshold_sec": "Stuck-signal threshold (s)",
    "scenario": "Scenario",
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

def _derive_metrics(metrics, duration_sec):
    """Fills in metrics that are pure functions of ones already recorded, so
    each caller doesn't have to remember to (and they stay consistent across
    scripts). Applied both when a record is created AND when one is rendered,
    so records saved before a derived metric existed still show it.

    This is also where legacy units are upgraded: records written when speeds
    were stored in m/s (and time loss per metre) are converted to km/h and
    s/km on the way out, so old and new runs stay directly comparable in one
    table without having to rewrite the archived JSON."""
    n_trips = metrics.get("n_trips_completed")
    if n_trips and duration_sec:
        metrics.setdefault("throughput_veh_per_hour", n_trips / duration_sec * 3600.0)
    travel_time = metrics.get("ambulance_travel_time_s")
    route_length = metrics.get("ambulance_route_length_m")
    if travel_time and route_length:
        metrics.setdefault("ambulance_avg_speed_kmh", route_length / travel_time * 3.6)

    # Legacy unit upgrades (m/s -> km/h, s/m -> s/km).
    for old_key, new_key, factor in (
        ("avg_speed_mps", "avg_speed_kmh", 3.6),
        ("ambulance_avg_speed_mps", "ambulance_avg_speed_kmh", 3.6),
        ("timeloss_per_meter", "timeloss_per_km", 1000.0),
    ):
        if old_key in metrics:
            value = metrics.pop(old_key)
            if value is not None:
                metrics.setdefault(new_key, value * factor)
    return metrics


def make_run_group(prefix, when=None):
    """Builds the identifier shared by every scenario of a single invocation.

    One 'run' of the ambulance demo produces two scenario records (EVP and
    baseline). Without a shared group id those records are indistinguishable
    from every other run's records once they pile up in run_records/, which
    is exactly what makes a mixed comparison table unreadable. The group id
    keeps a run's own scenarios together and separable from other runs'."""
    when = when or datetime.now()
    return f"{prefix}__{when.strftime('%Y%m%d_%H%M%S')}"


def new_run_record(run_type, label, metrics, params=None, seed=None,
                    duration_sec=None, notes=None, run_id=None, context=None,
                    run_group=None, display_name=None):
    """Builds a standard-shaped run record. `run_type` groups comparable runs
    (e.g. "ambulance", "scosca_tuning", "stuck_signal_diag", "scaleup").
    `label` distinguishes scenarios within one run (e.g. "baseline"/"tuned",
    "no_corridor"/"with_corridor"). `run_group` ties the scenarios of a single
    invocation together - see make_run_group. `display_name` is the short
    human label used as this record's column heading in comparison tables
    (defaults to `label`). `metrics` is a flat dict of metric_key -> numeric
    value; keys should match METRIC_CATALOG where possible but any key is
    accepted. `context` holds non-numeric scenario details worth showing in
    the report (start/end edge, corridor on/off, network file, ...) - see
    CONTEXT_LABELS."""
    timestamp = datetime.now()
    if run_group is None:
        run_group = make_run_group(run_type, timestamp)
    if run_id is None:
        run_id = f"{run_group}__{label}"

    metrics = _derive_metrics(dict(metrics), duration_sec)

    return {
        "run_id": run_id,
        "run_group": run_group,
        "run_type": run_type,
        "label": label,
        "display_name": display_name or label,
        "timestamp": timestamp.isoformat(timespec="seconds"),
        "duration_sec": duration_sec,
        "seed": seed,
        "params": params or {},
        "context": context or {},
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


def load_run_records(run_records_dir=RUN_RECORDS_DIR, run_type=None, run_ids=None,
                      run_group=None):
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
        if run_group and record.get("run_group") != run_group:
            continue
        if run_ids and record.get("run_id") not in run_ids:
            continue
        records.append(record)
    records.sort(key=lambda r: r.get("timestamp", ""))
    return records


def group_run_records(records):
    """Buckets records by run_group, preserving chronological order.

    Records written before run groups existed have no group id, but the
    scenarios of one invocation were saved together and so share a run_type
    and timestamp - that pair is used to re-associate them, which keeps
    historical runs as readable as new ones."""
    groups = {}
    for record in records:
        key = record.get("run_group")
        if not key:
            key = f"{record.get('run_type', 'run')}__{record.get('timestamp', record['run_id'])}"
        groups.setdefault(key, []).append(record)
    return groups


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


def get_tripinfo_by_id(path, veh_id):
    """Pulls one specific vehicle's tripinfo (e.g. the ambulance) so its own
    route length / waiting time can be reported, not just the fleet average."""
    if not os.path.exists(path):
        return None
    root = ET.parse(path).getroot()
    for t in root.findall("tripinfo"):
        if t.get("id") == veh_id:
            return {
                "duration": float(t.get("duration")),
                "routeLength": float(t.get("routeLength")),
                "waitingTime": float(t.get("waitingTime")),
                "timeLoss": float(t.get("timeLoss")),
            }
    return None


def tripinfo_list_metrics(trips):
    """Standard metric set computed directly from a list of tripinfo dicts
    (see load_tripinfos_xml) - this is the only metrics helper that can also
    produce timeloss_per_meter and n_trips_completed, since those need the
    raw per-trip list rather than SUMO's own aggregate statistics."""
    if not trips:
        return {"n_trips_completed": 0}
    n = len(trips)
    avg_speed_mps = sum(t["routeLength"] / t["duration"] for t in trips if t["duration"] > 0) / n
    return {
        "avg_speed_kmh": avg_speed_mps * 3.6,
        "avg_trip_duration_s": sum(t["duration"] for t in trips) / n,
        "avg_waiting_time_s": sum(t["waitingTime"] for t in trips) / n,
        "avg_time_loss_s": sum(t["timeLoss"] for t in trips) / n,
        "avg_route_length_m": sum(t["routeLength"] for t in trips) / n,
        "timeloss_per_km": sum(t["timeLoss"] / max(1.0, t["routeLength"]) for t in trips) / n * 1000.0,
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
        # SUMO reports speed in m/s; the report speaks km/h throughout.
        "avg_speed_kmh": float(el.get("speed", 0.0)) * 3.6,
        "avg_trip_duration_s": float(el.get("duration", 0.0)),
        "avg_waiting_time_s": float(el.get("waitingTime", 0.0)),
        "avg_time_loss_s": float(el.get("timeLoss", 0.0)),
        "avg_depart_delay_s": float(el.get("departDelay", 0.0)),
    }


# ---------------------------------------------------------------------------
# Markdown rendering. Two shapes:
#   - render_run_markdown(record)          -> the full story of ONE run
#   - render_comparison_markdown(records)  -> N runs side by side
# ---------------------------------------------------------------------------

def _fmt(value, unit=""):
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        text = f"{value:,.2f}"
    elif isinstance(value, int):
        text = f"{value:,}"
    else:
        return str(value)
    return f"{text} {unit}".strip()


def _delta_text(value, ref_value, direction):
    """Plain-text better/worse indicator for the comparison table. Returns ''
    when a delta would be meaningless (missing value, or a context-only
    metric where neither direction is 'better')."""
    if value is None or ref_value is None or direction == "context":
        return ""
    try:
        diff = value - ref_value
    except TypeError:
        return ""
    if abs(diff) < 1e-9:
        return "(same)"
    pct_text = f"{diff / abs(ref_value) * 100:+.1f}%" if ref_value else f"{diff:+.2f}"
    improved = (diff < 0) if direction == "lower_better" else (diff > 0)
    return f"{pct_text} {'better' if improved else 'worse'}"


def _md_escape(text):
    """Keeps a stray pipe in an id/path from breaking a Markdown table row."""
    return str(text).replace("|", "\\|")


def render_run_markdown(record):
    """The full Markdown report for a single run: what was run, what the
    scenario was (start/end points, corridor on/off), the headline numbers,
    then every recorded metric with its plain-English explanation."""
    metrics = _derive_metrics(dict(record.get("metrics", {})), record.get("duration_sec"))
    context = record.get("context", {})
    lines = [
        f"# Run report: {record['run_type']} / {record.get('display_name', record['label'])}",
        "",
        f"- **Run:** `{record.get('run_group', '-')}`",
        f"- **Scenario:** `{record['label']}`",
        f"- **Run at:** {record['timestamp']}",
    ]
    if record.get("duration_sec") is not None:
        lines.append(f"- **Simulated duration:** {record['duration_sec']} s")
    if record.get("seed") is not None:
        lines.append(f"- **Seed:** {record['seed']}")
    lines.append("")

    if context:
        lines += ["## Scenario", "", "| Detail | Value |", "|---|---|"]
        for key, value in context.items():
            label = CONTEXT_LABELS.get(key, key)
            lines.append(f"| {label} | {_md_escape(_fmt(value))} |")
        lines.append("")

    headline = [k for k in HEADLINE_METRICS if metrics.get(k) is not None]
    if headline:
        lines += ["## Headline results", "", "| Metric | Value |", "|---|---|"]
        for key in headline:
            info = _metric_info(key)
            lines.append(f"| {info['label']} | **{_fmt(metrics[key], info['unit'])}** |")
        lines.append("")

    if metrics:
        lines += [
            "## All recorded metrics",
            "",
            "| Metric | Value | What it means |",
            "|---|---|---|",
        ]
        ordered_keys = [k for k in METRIC_CATALOG if k in metrics]
        ordered_keys += sorted(set(metrics) - set(ordered_keys))
        for key in ordered_keys:
            info = _metric_info(key)
            lines.append(
                f"| {info['label']} | {_fmt(metrics[key], info['unit'])} | {info['description']} |"
            )
        lines.append("")

    if record.get("notes"):
        lines += ["## Notes", ""]
        lines += [f"- {note}" for note in record["notes"]]
        lines.append("")

    if record.get("params"):
        lines += [
            "<details>",
            "<summary>Controller parameters used for this run</summary>",
            "",
            "| Parameter | Value |",
            "|---|---|",
        ]
        for key in sorted(record["params"]):
            lines.append(f"| `{key}` | {_md_escape(record['params'][key])} |")
        lines += ["", "</details>", ""]

    lines += [
        "---",
        "",
        f"*Generated by `report.py`. Machine-readable copy: `run_records/{record['run_id']}.json`. "
        f"Compare against other runs with `python report.py --run-type {record['run_type']}`.*",
    ]
    return "\n".join(lines)


def run_group_dir(record, run_reports_dir=RUN_REPORTS_DIR):
    """Every scenario of one invocation writes into a folder named for its
    run group, so a run's outputs stay together and successive runs never
    overwrite each other."""
    group = record.get("run_group") or record.get("run_id")
    return Path(run_reports_dir) / group


def save_run_report_md(record, run_reports_dir=RUN_REPORTS_DIR):
    """Writes the single-scenario Markdown report into its run's folder.
    Call this alongside save_run_record() so every run leaves both a JSON
    archive and a readable document behind."""
    out_dir = run_group_dir(record, run_reports_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{record['label']}.md"
    with open(path, "w", encoding="utf-8") as f:
        f.write(render_run_markdown(record))
    return path


def render_comparison_markdown(records, title=None, description="", baseline_run_id=None):
    """N runs side by side as one Markdown table, with a better/worse note
    under each value relative to the reference run. The first record (or
    `baseline_run_id` if given) is the reference."""
    if not records:
        return "# Run comparison\n\n*No run records to compare.*\n"

    ref_idx = 0
    if baseline_run_id:
        for i, r in enumerate(records):
            if r["run_id"] == baseline_run_id:
                ref_idx = i
                break

    metrics_by_run = [
        _derive_metrics(dict(r.get("metrics", {})), r.get("duration_sec")) for r in records
    ]

    # Union of every metric key seen across all records, catalog order first
    # (so common metrics line up in a stable, readable order), then any
    # not-yet-catalogued keys appended alphabetically - new metric types
    # never break rendering of older records.
    seen_keys = set()
    for m in metrics_by_run:
        seen_keys.update(m.keys())
    ordered_keys = [k for k in METRIC_CATALOG if k in seen_keys]
    ordered_keys += sorted(seen_keys - set(ordered_keys))

    lines = [f"# {title or 'Run comparison'}", ""]
    if description:
        lines += [description, ""]

    # When several runs are lined up, a column headed only "no_corridor" is
    # ambiguous - every run has one. Give each column a numbered key and list
    # what that key actually refers to, so the table can be read without
    # having to guess which run a column belongs to.
    groups = group_run_records(records)
    multi_run = len(groups) > 1
    if multi_run:
        lines += ["## Runs in this comparison", "",
                   "| Key | Run | Scenario | Hospital | Duration | Seed |", "|---|---|---|---|---|---|"]
        for idx, record in enumerate(records, start=1):
            ctx = record.get("context", {})
            lines.append(
                f"| **R{idx}** | `{_md_escape(record.get('run_group', record['run_id']))}` "
                f"| {_md_escape(record.get('display_name', record['label']))} "
                f"| {_md_escape(ctx.get('hospital', '-'))} "
                f"| {_md_escape(record.get('duration_sec', '-'))} s "
                f"| {_md_escape(record.get('seed', '-'))} |"
            )
        lines.append("")

    ref_label = records[ref_idx].get("display_name", records[ref_idx]["label"])
    ref_key = f"R{ref_idx + 1} " if multi_run else ""
    lines += [
        "## Metrics",
        "",
        f"Comparing **{len(records)}** run record(s) across **{len(groups)}** run(s). "
        f"Percentages compare each value against **{ref_key}{ref_label}**"
        + (f" ({records[ref_idx]['timestamp']})." if not multi_run else "."),
        "",
    ]

    if multi_run:
        headers = ["Metric"] + [
            f"R{i + 1}<br>{_md_escape(r.get('display_name', r['label']))}"
            for i, r in enumerate(records)
        ]
    else:
        headers = ["Metric"] + [
            _md_escape(r.get("display_name", r["label"])) for r in records
        ]
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("|" + "---|" * len(headers))

    # Only surface meta rows that actually differ between the runs being
    # compared - repeating an identical seed/duration across every column is
    # noise that pushes the real numbers off the screen.
    meta_keys = ("run_type", "timestamp", "seed", "duration_sec")
    for meta_key in meta_keys:
        values = [r.get(meta_key, "-") for r in records]
        if len(set(map(str, values))) == 1 and meta_key != "timestamp":
            continue
        cells = [_md_escape(v) for v in values]
        lines.append(f"| *{meta_key}* | " + " | ".join(cells) + " |")

    ref_duration = records[ref_idx].get("duration_sec")
    suppressed_any = False
    for key in ordered_keys:
        info = _metric_info(key)
        ref_value = metrics_by_run[ref_idx].get(key)
        cells = []
        for i, metrics in enumerate(metrics_by_run):
            value = metrics.get(key)
            cell = _fmt(value, info["unit"])
            if i != ref_idx and len(records) > 1:
                # A raw count compared against a run of a different length
                # would measure the window, not the system - so state that
                # plainly instead of printing a misleading percentage.
                if info.get("duration_sensitive") and records[i].get("duration_sec") != ref_duration:
                    if value is not None and ref_value is not None:
                        cell += "<br>*not comparable (different run length)*"
                        suppressed_any = True
                else:
                    delta = _delta_text(value, ref_value, info["direction"])
                    if delta:
                        cell += f"<br>{delta}"
            cells.append(cell)
        lines.append(f"| **{info['label']}** | " + " | ".join(cells) + " |")

    if suppressed_any:
        lines += [
            "",
            "> Some runs in this table have different simulated durations. Raw counts "
            "(trips completed, preemption events) scale with run length, so their "
            "percentages are withheld where the lengths differ - compare **Throughput** "
            "(per hour) instead, and treat the averages as the like-for-like measures.",
        ]

    lines += ["", "## What each metric means", ""]
    for key in ordered_keys:
        info = _metric_info(key)
        lines.append(f"- **{info['label']}** - {info['description']}")

    lines += [
        "",
        "---",
        "",
        "*Generated by `report.py` from saved run records in `run_records/`.*",
    ]
    return "\n".join(lines)


def render_evp_verdict_markdown(baseline_record, evp_record):
    """The headline EVP-vs-baseline conclusion for a paired ambulance run.

    Separates the two questions that actually matter for an emergency-vehicle
    preemption system, because they trade off against each other:
      1. Did the ambulance get through faster?  (the benefit)
      2. What did that cost everyone else?      (the price)
    A verdict that only reports (1) would hide the case where the corridor
    buys ambulance time by dumping delay onto the rest of the network."""
    base = _derive_metrics(dict(baseline_record.get("metrics", {})),
                            baseline_record.get("duration_sec"))
    evp = _derive_metrics(dict(evp_record.get("metrics", {})),
                           evp_record.get("duration_sec"))

    def pct(new, old):
        if new is None or old in (None, 0):
            return None
        return (new - old) / abs(old) * 100

    lines = ["## Verdict: EVP vs. baseline VAC", ""]

    bt, et = base.get("ambulance_travel_time_s"), evp.get("ambulance_travel_time_s")
    if bt and et:
        change = pct(et, bt)
        faster = change < 0
        lines += [
            f"**Emergency vehicle: {abs(change):.1f}% {'faster' if faster else 'SLOWER'} with EVP.**",
            "",
            f"- Travel time went {bt:.1f}s (baseline VAC) -> {et:.1f}s (EVP), "
            f"a saving of {bt - et:.1f}s." if faster else
            f"- Travel time went {bt:.1f}s (baseline VAC) -> {et:.1f}s (EVP), "
            f"a regression of {et - bt:.1f}s.",
        ]
        bw, ew = base.get("ambulance_waiting_time_s"), evp.get("ambulance_waiting_time_s")
        if bw is not None and ew is not None:
            lines.append(f"- Time spent stopped at signals: {bw:.1f}s -> {ew:.1f}s.")
        if evp.get("preemption_events") is not None:
            lines.append(f"- Preemption events fired: {evp['preemption_events']:.0f}.")
    else:
        lines += [
            "**Inconclusive - the ambulance did not complete its trip in both scenarios.**",
            "",
            "Increase `--duration` and rerun; the numbers below cannot be compared as they stand.",
        ]
    lines.append("")

    # The cost side: what the rest of the network paid for that priority.
    bwait, ewait = base.get("avg_waiting_time_s"), evp.get("avg_waiting_time_s")
    if bwait is not None and ewait is not None:
        change = pct(ewait, bwait)
        if change is None:
            verdict = "unchanged"
        elif change <= 1.0:
            verdict = "no meaningful cost to other traffic"
        elif change <= 10.0:
            verdict = "a small, acceptable cost to other traffic"
        else:
            verdict = "a NOTABLE cost to other traffic - worth investigating"
        lines += [
            f"**Rest of the network: {verdict}.**",
            "",
            f"- Average waiting time for all other vehicles: {bwait:.2f}s -> {ewait:.2f}s "
            f"({change:+.1f}%).",
        ]
        bl, el = base.get("avg_time_loss_s"), evp.get("avg_time_loss_s")
        if bl is not None and el is not None:
            lines.append(f"- Average time loss for all other vehicles: {bl:.2f}s -> {el:.2f}s "
                          f"({pct(el, bl):+.1f}%).")
        bth, eth = base.get("throughput_veh_per_hour"), evp.get("throughput_veh_per_hour")
        if bth is not None and eth is not None:
            lines.append(f"- Network throughput: {bth:,.0f} -> {eth:,.0f} veh/h "
                          f"({pct(eth, bth):+.1f}%).")
        lines.append("")
        lines.append(
            "> Both scenarios ran the same route, same seed and same duration, so these "
            "differences reflect the EVP layer rather than a change in demand. Note that "
            "one paired run is a single sample - repeat across seeds before treating any "
            "small difference as a real effect."
        )
    lines.append("")
    return "\n".join(lines)


def build_comparison_report(out_path, run_type=None, run_ids=None, title=None,
                             description="", run_records_dir=RUN_RECORDS_DIR,
                             run_group=None):
    """Loads saved run records (optionally filtered) and writes one Markdown
    comparison report to `out_path`. This is the 'compare every run we've
    ever done' entry point - it needs nothing re-run, just whatever scripts
    have already called save_run_record()."""
    records = load_run_records(run_records_dir, run_type=run_type, run_ids=run_ids,
                                run_group=run_group)
    if title is None:
        if run_group:
            title = f"Run comparison: {run_group}"
        elif run_type:
            title = f"Run comparison: {run_type}"
        else:
            title = "Run comparison: all runs"
    md = render_comparison_markdown(records, title, description)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(md)
    return out_path


def _cli():
    parser = argparse.ArgumentParser(description="EVATO run reporting (Markdown output).")
    parser.add_argument("--list", action="store_true",
                         help="list saved runs, grouped by run, and exit")
    parser.add_argument("--run-type", type=str, default=None, help="filter to one run_type (e.g. ambulance)")
    parser.add_argument("--run-group", type=str, default=None,
                         help="filter to a single run (the id shown by --list)")
    parser.add_argument("--per-run", action="store_true",
                         help="(re)generate the per-scenario .md report for every saved record into run_reports/<run>/")
    parser.add_argument("--out", type=str, default=str(RUN_RECORDS_DIR / "comparison_report.md"),
                         help="where to write the comparison .md")
    args = parser.parse_args()

    if args.list:
        records = load_run_records(run_type=args.run_type, run_group=args.run_group)
        if not records:
            print("No run records found in", RUN_RECORDS_DIR)
            return
        groups = group_run_records(records)
        print(f"{len(groups)} run(s), {len(records)} scenario record(s):\n")
        for group_id, group_records in groups.items():
            first = group_records[0]
            hospital = first.get("context", {}).get("hospital")
            head = f"{first['timestamp']}  {first['run_type']}  [{group_id}]"
            print(head + (f"  - {hospital}" if hospital else ""))
            for r in group_records:
                dur = r.get("duration_sec", "?")
                print(f"    - {r.get('display_name', r['label']):<28} ({dur}s)  {r['run_id']}")
            print()
        return

    if args.per_run:
        records = load_run_records(run_type=args.run_type, run_group=args.run_group)
        if not records:
            print("No run records found in", RUN_RECORDS_DIR)
            return
        for r in records:
            print(f"Wrote {save_run_report_md(r)}")
        return

    out_path = build_comparison_report(args.out, run_type=args.run_type,
                                        run_group=args.run_group)
    print(f"Comparison report written to: {out_path}")


if __name__ == "__main__":
    _cli()
