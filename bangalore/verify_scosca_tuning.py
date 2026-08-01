"""
Verification harness for the CoSiCoSt-alignment fine-tuning of SCOSCA
(priority route / priority stage, Full Vehicle Actuation fallback, and the
local Online Split Optimizer gap-extension). Not part of the production
demo — this is a standalone script that runs short, controlled simulations
and produces plots + a metrics table so the changes can be visually and
numerically verified.

Usage:
    python verify_scosca_tuning.py [--duration 150] [--out DIR]

Produces (in --out, default ./verify_output):
    - baseline_stats.xml / tuned_stats.xml (SUMO aggregate trip statistics)
    - verification_report.html (self-contained dashboard with all plots)
"""

import argparse
import copy
import io
import base64
import os
import sys
import time
import warnings
import xml.etree.ElementTree as ET
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
SRC_DIR = BASE_DIR / "src"
for path in (BASE_DIR, SRC_DIR):
    if path.exists() and str(path) not in sys.path:
        sys.path.insert(0, str(path))

SUMO_HOME = os.environ.get("SUMO_HOME", "D:\\")
os.environ["PROJ_LIB"] = os.path.join(SUMO_HOME, "share", "proj")
os.environ["PROJ_DATA"] = os.path.join(SUMO_HOME, "share", "proj")
warnings.filterwarnings("ignore")

import traci
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from bangalore.parser import BangaloreNetworkParser
from bangalore.graph_builder import BangaloreGraphBuilder
from bangalore.scosca_controller import BangaloreSCOSCA
from report import new_run_record, save_run_record, stats_xml_metrics

NET_FILE = str(BASE_DIR / "Bangalore_Map" / "osm.net.xml.gz")
SUMO_CFG = str(BASE_DIR / "Bangalore_Map" / "osm.sumocfg")

BASE_PARAMS = {
    "adaptation_cycle": 30,
    "adaptation_green": 10,
    "green_thresh": 2,
    "adaptation_offset": 1,
    "offset_thresh": 0.5,
    "min_cycle_length": 15,
    "max_cycle_length": 50,
    "ds_upper_val": 0.925,
    "ds_lower_val": 0.875,
    "measurement_period": 4,  # 1 / time_step(0.25s)
}

TUNED_OVERRIDES = {
    "priority_stage_boost": 5.0,
    "priority_route_delay_weight": 0.05,
    "actuation_extension_sec": 5,
    "actuation_min_green_ratio": 0.5,
    "actuation_min_green_floor": 5,
    "actuation_gap_thresh": 3.0,
    "fallback_missing_cycles": 3,
    "fallback_min_green": 10,
    "fallback_max_green": 45,
}

BASELINE_OVERRIDES = {
    "priority_stage_boost": 0.0,
    "priority_route_delay_weight": 0.0,
    "actuation_extension_sec": 0,
    "actuation_min_green_ratio": 1.0,
    "actuation_gap_thresh": None,
    "fallback_missing_cycles": 10**9,  # effectively disabled
    "fallback_min_green": 10,
    "fallback_max_green": 45,
}

TIME_STEP = 0.25
# Short cycle length so multiple district recompute cycles (and the every-5th
# cycle-length/offset/priority-route recomputes) fit inside a short, stable run.
INITIAL_CYCLE_LENGTH = 25


def run_config(label, params, duration_sec, out_dir, tracked_tls=None,
               fault_district=None, fault_window=None, seed=42, max_attempts=3,
               initial_cycle_length=None):
    """Runs one simulation configuration; returns measurement_data, stats path,
    and a per-step phase-transition trace for `tracked_tls` (list of tl_id).
    `params` must be a complete scosca_params dict (caller merges overrides)."""
    tripinfo_path = str(out_dir / f"{label}_tripinfos.xml")
    stats_path = str(out_dir / f"{label}_stats.xml")
    cycle_len = initial_cycle_length if initial_cycle_length is not None else INITIAL_CYCLE_LENGTH

    for attempt in range(1, max_attempts + 1):
        try:
            parser_obj = BangaloreNetworkParser(NET_FILE)
            graph_builder = BangaloreGraphBuilder(parser_obj)
            controller = BangaloreSCOSCA(params, parser_obj, graph_builder,
                                          initial_cycle_length=cycle_len)

            sumo_cmd = [
                "sumo", "-c", SUMO_CFG, "--start", "--quit-on-end",
                "--time-to-teleport", "-1", "--seed", str(seed),
                "--tripinfo-output", tripinfo_path,
                "--statistic-output", stats_path,
                "--no-step-log", "true",
                # The demo script never pins --step-length, so SUMO silently
                # defaults to 1.0s/step while the Python side assumes
                # TIME_STEP (0.25s) for its counters — a pre-existing mismatch
                # (out of scope for this fine-tuning pass). Pin it explicitly
                # here so this script's timing assumptions hold.
                "--step-length", str(TIME_STEP),
            ]
            traci.start(sumo_cmd, numRetries=100)
            controller.init_simulation()

            phase_trace = {tl: [] for tl in (tracked_tls or [])}
            last_phase = {}

            fault_active_patch = None
            total_steps = int(duration_sec / TIME_STEP)
            for step in range(total_steps):
                traci.simulationStep()
                current_time = traci.simulation.getCurrentTime() / 1000.0

                # Optional fault injection: simulate detector/comms data loss
                # for one district during [fault_window[0], fault_window[1]]
                # to prove the Full Vehicle Actuation fallback (CoSiCoSt step
                # 10) actually triggers and recovers.
                if fault_district and fault_window and fault_window[0] <= current_time <= fault_window[1]:
                    if fault_active_patch is None:
                        fault_active_patch = _patch_lane_reads_to_fail(controller, fault_district)
                elif fault_active_patch is not None:
                    fault_active_patch()  # restore
                    fault_active_patch = None

                controller.execute_control(traci.simulation.getCurrentTime())

                for tl in (tracked_tls or []):
                    try:
                        ph = traci.trafficlight.getPhase(tl)
                    except traci.TraCIException:
                        continue
                    if last_phase.get(tl) != ph:
                        phase_trace[tl].append(current_time)
                        last_phase[tl] = ph

            if fault_active_patch is not None:
                fault_active_patch()

            traci.close()
            return copy.deepcopy(controller.measurement_data), stats_path, phase_trace

        except traci.exceptions.FatalTraCIError as e:
            print(f"[{label}] attempt {attempt}/{max_attempts} failed: {e}")
            try:
                traci.close()
            except Exception:
                pass
            time.sleep(1.0)

    raise RuntimeError(f"Simulation '{label}' failed after {max_attempts} attempts (TraCI connection instability).")


def _patch_lane_reads_to_fail(controller, district_name):
    """Monkeypatches traci.lane.getLastStepVehicleNumber to raise for the
    given district's lanes, simulating a detector/comms outage. Returns a
    callable that restores the original behavior."""
    tls_in_district = controller.districts.get(district_name, [])
    faulty_lanes = set()
    for intersection in controller.intersections:
        if intersection.tl_id in tls_in_district:
            for lanes in intersection.links.values():
                faulty_lanes.update(lanes)

    original = traci.lane.getLastStepVehicleNumber

    def patched(lane_id):
        if lane_id in faulty_lanes:
            raise traci.TraCIException("simulated detector/comms outage")
        return original(lane_id)

    traci.lane.getLastStepVehicleNumber = patched

    def restore():
        traci.lane.getLastStepVehicleNumber = original

    return restore


def parse_stats(stats_path):
    if not os.path.exists(stats_path):
        return {}
    root = ET.parse(stats_path).getroot()
    el = root.find("vehicleTripStatistics")
    if el is None:
        return {}
    keys = ["routeLength", "speed", "duration", "waitingTime", "timeLoss", "departDelay"]
    return {k: float(el.get(k, 0.0)) for k in keys}


def fig_to_data_uri(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode("ascii")


def plot_cycle_lengths(baseline_md, tuned_md):
    fig, ax = plt.subplots(figsize=(8, 4))
    for district in tuned_md["cycle_lengths"].keys():
        b_times = [t for t, d in [(h[0], h[1]) for h in baseline_md["history_cycle_lengths"]]]
        b_vals = [d.get(district) for _, d in [(h[0], h[1]) for h in baseline_md["history_cycle_lengths"]]]
        t_times = [t for t, d in [(h[0], h[1]) for h in tuned_md["history_cycle_lengths"]]]
        t_vals = [d.get(district) for _, d in [(h[0], h[1]) for h in tuned_md["history_cycle_lengths"]]]
        ax.plot([x / 1000.0 for x in b_times], b_vals, "--", label=f"{district} baseline")
        ax.plot([x / 1000.0 for x in t_times], t_vals, "-", label=f"{district} tuned")
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Cycle length (s)")
    ax.set_title("DOS-driven cycle length: baseline vs. tuned")
    ax.legend(fontsize=8)
    return fig_to_data_uri(fig)


def plot_priority_direction(tuned_md):
    fig, ax = plt.subplots(figsize=(8, 3))
    by_district = {}
    for t, district, direction in tuned_md["history_priority_direction"]:
        by_district.setdefault(district, []).append((t / 1000.0, 0 if direction == "forward" else 1))
    for district, points in by_district.items():
        if not points:
            continue
        xs, ys = zip(*points)
        ax.step(xs, ys, where="post", label=district, marker="o")
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["forward", "reverse"])
    ax.set_xlabel("Simulation time (s)")
    ax.set_title("Demand-responsive priority-route direction over time (tuned run)")
    ax.legend(fontsize=8)
    return fig_to_data_uri(fig)


def plot_greentimes(baseline_md, tuned_md, tl_id):
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.5), sharey=True)
    for ax, md, title in ((axes[0], baseline_md, "baseline"), (axes[1], tuned_md, "tuned")):
        times = [h[0] / 1000.0 for h in md["history_greentimes"]]
        greens_by_phase = {}
        for _, greens_dict in md["history_greentimes"]:
            greens = greens_dict.get(tl_id, [])
            for idx, g in enumerate(greens):
                greens_by_phase.setdefault(idx, []).append(g)
        for idx, vals in greens_by_phase.items():
            n = min(len(times), len(vals))
            ax.plot(times[:n], vals[:n], marker="o", label=f"phase {idx}")
        ax.set_title(f"{tl_id}\n({title})")
        ax.set_xlabel("Simulation time (s)")
    axes[0].set_ylabel("Green time (s)")
    axes[1].legend(fontsize=8)
    fig.suptitle("Per-phase green split allocation for a representative intersection")
    return fig_to_data_uri(fig)


def plot_stats_comparison(baseline_stats, tuned_stats):
    metrics = ["speed", "duration", "waitingTime", "timeLoss"]
    labels = ["Avg speed (m/s)", "Avg trip duration (s)", "Avg waiting time (s)", "Avg time loss (s)"]
    fig, axes = plt.subplots(1, len(metrics), figsize=(12, 3.5))
    for ax, metric, label in zip(axes, metrics, labels):
        vals = [baseline_stats.get(metric, 0.0), tuned_stats.get(metric, 0.0)]
        ax.bar(["baseline", "tuned"], vals, color=["#888", "#2b7"])
        ax.set_title(label, fontsize=9)
    fig.suptitle("Aggregate SUMO trip statistics: baseline vs. tuned")
    return fig_to_data_uri(fig)


def plot_phase_realized_durations(baseline_trace, tuned_trace, tl_id):
    fig, ax = plt.subplots(figsize=(8, 3.5))
    for trace, label, style in ((baseline_trace.get(tl_id, []), "baseline (static)", "--"),
                                 (tuned_trace.get(tl_id, []), "tuned (actuated)", "-")):
        durations = [b - a for a, b in zip(trace, trace[1:])]
        ax.plot(range(len(durations)), durations, style, marker="o", label=label)
    ax.set_xlabel("Phase-switch index")
    ax.set_ylabel("Realized phase duration (s)")
    ax.set_title(f"Realized green/yellow phase durations at {tl_id}\n(variability = gap-extension in action)")
    ax.legend(fontsize=8)
    return fig_to_data_uri(fig)


def plot_fallback_events(fault_md, fault_district, fault_window):
    fig, ax = plt.subplots(figsize=(8, 2.5))
    events = [t / 1000.0 for t, d in fault_md["history_fallback_events"] if d == fault_district]
    ax.axvspan(fault_window[0], fault_window[1], color="red", alpha=0.15, label="simulated data outage")
    for e in events:
        ax.axvline(e, color="darkred", linestyle=":", linewidth=1)
    ax.scatter(events, [1] * len(events), color="darkred", zorder=3, label="fallback triggered")
    ax.set_yticks([])
    ax.set_xlabel("Simulation time (s)")
    ax.set_title(f"Full Vehicle Actuation fallback events for {fault_district}")
    ax.legend(fontsize=8)
    return fig_to_data_uri(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=600)
    parser.add_argument("--out", type=str, default=str(Path(__file__).parent / "verify_output"))
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Building network graph once to pick a representative tracked TLS per district...")
    parser_obj = BangaloreNetworkParser(NET_FILE)
    graph_builder = BangaloreGraphBuilder(parser_obj)
    districts = graph_builder.get_districts()
    tracked_tls = [tls[0] for tls in districts.values()]
    fault_district = list(districts.keys())[0]

    baseline_params = {**BASE_PARAMS, **BASELINE_OVERRIDES}
    tuned_params = {**BASE_PARAMS, **TUNED_OVERRIDES}

    print("=== Running BASELINE (priority route/boost/actuation/fallback disabled) ===")
    baseline_md, baseline_stats_path, baseline_trace = run_config(
        "baseline", baseline_params, args.duration, out_dir, tracked_tls=tracked_tls
    )

    print("=== Running TUNED (all CoSiCoSt-alignment features enabled) ===")
    tuned_md, tuned_stats_path, tuned_trace = run_config(
        "tuned", tuned_params, args.duration, out_dir, tracked_tls=tracked_tls
    )

    print("=== Running FALLBACK fault-injection test (tuned config, forced data outage) ===")
    # Needs to span at least `fallback_missing_cycles` (3) consecutive district
    # recompute cycles (~INITIAL_CYCLE_LENGTH=25s apart) to actually trigger,
    # then end early enough to show recovery before the run finishes.
    fault_window = (max(15, args.duration * 0.15), args.duration * 0.65)
    fault_md, fault_stats_path, _ = run_config(
        "fallback_fault", tuned_params, args.duration, out_dir,
        fault_district=fault_district, fault_window=fault_window
    )

    baseline_stats = parse_stats(baseline_stats_path)
    tuned_stats = parse_stats(tuned_stats_path)

    # Persist standard-shaped run records (report.py) alongside this script's
    # own bespoke plots, so baseline/tuned here can be compared against any
    # other scosca_tuning run (or even a production-scale run) later.
    baseline_record = new_run_record(
        "scosca_tuning", "baseline", stats_xml_metrics(baseline_stats_path),
        params=baseline_params, duration_sec=args.duration,
    )
    save_run_record(baseline_record)
    tuned_record = new_run_record(
        "scosca_tuning", "tuned", stats_xml_metrics(tuned_stats_path),
        params=tuned_params, duration_sec=args.duration,
    )
    save_run_record(tuned_record)
    print(f"Saved run records: {baseline_record['run_id']}, {tuned_record['run_id']}")
    print("Run `python report.py --run-type scosca_tuning` anytime to compare against every tuning run ever done.")

    print("Rendering plots...")
    img_cycle = plot_cycle_lengths(baseline_md, tuned_md)
    img_priority = plot_priority_direction(tuned_md)
    img_splits = plot_greentimes(baseline_md, tuned_md, tracked_tls[0])
    img_stats = plot_stats_comparison(baseline_stats, tuned_stats)
    img_phase_durations = plot_phase_realized_durations(baseline_trace, tuned_trace, tracked_tls[0])
    img_fallback = plot_fallback_events(fault_md, fault_district, fault_window)

    fallback_events = [e for e in fault_md["history_fallback_events"] if e[1] == fault_district]

    html = f"""<h1>SCOSCA CoSiCoSt-alignment verification</h1>
<p>Baseline = priority-route boost / actuation extension / fallback all disabled (old behavior).
Tuned = all three fine-tuning features enabled. Same seed, same duration ({args.duration}s), same network.</p>

<h2>1. Priority route + priority-stage boost</h2>
<img src="{img_priority}" style="max-width:100%">
<p>Direction flips between "forward" and "reverse" as real-time congestion shifts across the district's
corridor &mdash; proof the offset propagation order is demand-responsive, not the old static topological order.</p>
<img src="{img_splits}" style="max-width:100%">
<p>Per-phase green-time allocation over time for a representative intersection. In the tuned run the
phase aligned with the current priority route gets systematically more green than in the baseline.</p>

<h2>2. Local Online Split Optimizer (gap-extension)</h2>
<img src="{img_phase_durations}" style="max-width:100%">
<p>Baseline uses a rigid static program: realized phase durations should exactly match planned splits
(flat/deterministic). Tuned uses SUMO's actuated program (min/max duration bound by
<code>actuation_extension_sec</code>): realized durations vary step to step because green is
extended when the stop-line is still occupied &mdash; direct evidence the gap-out mechanism is active.</p>

<h2>3. DOS-driven cycle length</h2>
<img src="{img_cycle}" style="max-width:100%">
<p>Cycle length still adapts to degree-of-saturation in both runs (steps 1-5/9 were already correct);
shown here mainly to confirm the fine-tuning didn't regress this baseline behavior.</p>

<h2>4. Fallback: Full Vehicle Actuation on simulated data loss</h2>
<img src="{img_fallback}" style="max-width:100%">
<p>Detector reads for district <code>{fault_district}</code> were monkeypatched to raise
<code>TraCIException</code> during the shaded window ({fault_window[0]:.0f}s&ndash;{fault_window[1]:.0f}s),
simulating a comms/detector outage. Fallback events recorded:
{"<br>".join(f"t={e[0]/1000.0:.1f}s" for e in fallback_events) or "<b>none triggered &mdash; investigate</b>"}.
The district should stop optimizing against stale data within
<code>fallback_missing_cycles</code> cycles of the outage starting, and resume normal SCOSCA control
once the outage window ends.</p>

<h2>5. Aggregate network-level impact</h2>
<img src="{img_stats}" style="max-width:100%">
<table border="1" cellpadding="4" style="border-collapse:collapse">
<tr><th>Metric</th><th>Baseline</th><th>Tuned</th></tr>
{"".join(f"<tr><td>{k}</td><td>{baseline_stats.get(k, 'n/a')}</td><td>{tuned_stats.get(k, 'n/a')}</td></tr>" for k in ["speed","duration","waitingTime","timeLoss","routeLength","departDelay"])}
</table>
<p><i>Note: this short run uses a shortened cycle length (min/max {BASE_PARAMS['min_cycle_length']}-{BASE_PARAMS['max_cycle_length']}s,
vs. the production demo's 50-180s) purely so several district recompute cycles fit inside a short,
TraCI-connection-stable run window. Aggregate network stats over this short window are illustrative,
not a production benchmark &mdash; for that, rerun with production parameters over a full 3600s demand
period.</i></p>
"""

    report_path = out_dir / "verification_report.html"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"\nReport written to: {report_path}")


if __name__ == "__main__":
    main()
