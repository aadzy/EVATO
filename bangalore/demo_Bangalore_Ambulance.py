"""
Emergency-vehicle green-corridor demo: inserts a single ambulance on a long
cross-network trip and compares its travel time — and the surrounding
traffic's delay — with the EVATO green-corridor preemption ON vs. OFF.

This exercises BangaloreSCOSCA._update_emergency_preemption /
_preempt_intersection (bangalore/scosca_controller.py): every traffic light
on the ambulance's immediate path is forced to the phase serving its
approach (with a mandatory yellow transition first, never a green-to-green
jump), held only while the ambulance is still approaching, and released the
instant it clears — every other intersection keeps running normal SCOSCA
control the whole time.

Each run is anchored on one of the hospitals in hospitals.py: you pick which
hospital the run is about and whether it is the route's start or destination
(interactively, or via --hospital/--role), and the other endpoint is drawn at
random from the network. That single route is then driven TWICE - first with
the EVP green corridor enabled, then again with it disabled - so the two sets
of metrics differ only by the EVP layer, and the report ends with a direct
EVP-vs-baseline verdict.

Usage:
    python demo_Bangalore_Ambulance.py [--duration 900] [--out DIR]
    python demo_Bangalore_Ambulance.py --hospital kidwai --role destination
    python demo_Bangalore_Ambulance.py --validate   # also auto-generate an EV
                                                     # validation .md report
                                                     # (see validation.py) from
                                                     # this run's own log

Produces (in --out, default ./ambulance_output):
    - no_corridor_tripinfos.xml / with_corridor_tripinfos.xml
    - ambulance_report.md (comparison table + charts: ambulance trajectory,
      travel time comparison, and background-traffic impact)
    - ambulance_progress.png / ambulance_speed.png / background_traffic.png
Plus, via report.py: a JSON record in run_records/ and a per-scenario
Markdown report in run_reports/ for each of the two scenarios.
"""

import argparse
import os
import statistics
import sys
import time
import warnings
from datetime import datetime
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
from report import (
    new_run_record, save_run_record, save_run_report_md, tripinfo_list_metrics,
    load_tripinfos_xml, get_tripinfo_by_id, render_comparison_markdown,
    render_evp_verdict_markdown, make_run_group, RUN_REPORTS_DIR,
)
from hospitals import select_hospital, pick_route_with_hospital, HOSPITALS_BY_KEY
from validation import capture_run_log, validate_ev_log, VALIDATION_REPORTS_DIR

NET_FILE = str(BASE_DIR / "Bangalore_Map" / "osm.net.xml.gz")
SUMO_CFG = str(BASE_DIR / "Bangalore_Map" / "osm.sumocfg")
TIME_STEP = 0.25

SCOSCA_PARAMS = {
    "adaptation_cycle": 30,
    "adaptation_green": 10,
    "green_thresh": 2,
    "adaptation_offset": 1,
    "offset_thresh": 0.5,
    "min_cycle_length": 50,
    "max_cycle_length": 180,
    "ds_upper_val": 0.925,
    "ds_lower_val": 0.875,
    "measurement_period": int(1 / TIME_STEP),
    "priority_stage_boost": 5.0,
    "priority_route_delay_weight": 0.05,
    "actuation_extension_sec": 5,
    "actuation_min_green_ratio": 0.5,
    "actuation_min_green_floor": 5,
    "actuation_gap_thresh": 3.0,
    "fallback_missing_cycles": 3,
    "fallback_min_green": 10,
    "fallback_max_green": 45,
    "preemption_lookahead_edges": 2,
    "preemption_transition_yellow_sec": 3,
    "preemption_hold_refresh_sec": 5,
}
INITIAL_CYCLE_LENGTH = 120
AMBULANCE_DEPART = 60.0


def plan_hospital_route(hospital_key=None, role=None, seed=42, graph_builder=None):
    """Chooses this run's hospital, then builds the single route both
    scenarios will drive.

    The route is planned ONCE here and reused by the with-corridor and
    without-corridor scenarios: comparing EVP against baseline is only
    meaningful if the ambulance drives an identical route in both, so this
    is deliberately not re-planned per scenario.

    Uses a throwaway TraCI connection because routing needs a loaded
    network; the real runs open their own."""
    hospital, role = select_hospital(hospital_key, role)

    edge_to_tl = graph_builder.get_edge_to_tl() if graph_builder is not None else None

    sumo_cmd = ["sumo", "-c", SUMO_CFG, "--start", "--quit-on-end", "--no-step-log", "true"]
    traci.start(sumo_cmd, numRetries=100)
    try:
        # Pass edge_to_tl so the random endpoint is chosen to yield a
        # signal-rich corridor: the EVP layer can only act at traffic lights,
        # so a route through none would make both scenarios identical.
        from_edge, to_edge, route_edges, other_edge = pick_route_with_hospital(
            hospital, role, seed=seed, edge_to_tl=edge_to_tl)
        hospital_edge = to_edge if role == "destination" else from_edge
        n_signals = (len({edge_to_tl[e] for e in route_edges if e in edge_to_tl})
                     if edge_to_tl else None)
    finally:
        traci.close()

    return {
        "hospital": hospital,
        "role": role,
        "from_edge": from_edge,
        "to_edge": to_edge,
        "route_edges": route_edges,
        "hospital_edge": hospital_edge,
        "other_edge": other_edge,
        "n_signals": n_signals,
        "seed": seed,
    }


def run_scenario(label, corridor_enabled, route_edges, duration_sec, out_dir, seed=42):
    tripinfo_path = str(out_dir / f"{label}_tripinfos.xml")
    stats_path = str(out_dir / f"{label}_stats.xml")

    parser_obj = BangaloreNetworkParser(NET_FILE)
    graph_builder = BangaloreGraphBuilder(parser_obj)
    controller = BangaloreSCOSCA(SCOSCA_PARAMS, parser_obj, graph_builder,
                                  initial_cycle_length=INITIAL_CYCLE_LENGTH)
    controller.evato_override_enabled = corridor_enabled

    sumo_cmd = [
        "sumo", "-c", SUMO_CFG, "--start", "--quit-on-end",
        "--time-to-teleport", "-1", "--seed", str(seed),
        "--tripinfo-output", tripinfo_path,
        "--statistic-output", stats_path,
        "--no-step-log", "true",
        "--step-length", str(TIME_STEP),
    ]
    for attempt in range(3):
        try:
            traci.start(sumo_cmd, numRetries=100)
            break
        except traci.exceptions.FatalTraCIError:
            time.sleep(1.0)
    controller.init_simulation()

    if "emergency" not in traci.vehicletype.getIDList():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", "emergency")
        traci.vehicletype.setVehicleClass("emergency", "emergency")
        traci.vehicletype.setColor("emergency", (255, 0, 0, 255))
        traci.vehicletype.setShapeClass("emergency", "emergency")
        traci.vehicletype.setSpeedFactor("emergency", 1.3)
        traci.vehicletype.setMinGap("emergency", 1.5)

    ambulance_inserted = False
    ambulance_trace = []  # (t, edge, speed, distance_along_route)
    ambulance_depart_time = None
    ambulance_arrive_time = None

    total_steps = int(duration_sec / TIME_STEP)
    for step in range(total_steps):
        traci.simulationStep()
        t = traci.simulation.getCurrentTime() / 1000.0

        if not ambulance_inserted and t >= AMBULANCE_DEPART:
            traci.route.add("ambulance_route", route_edges)
            try:
                traci.vehicle.add("ambulance_0", routeID="ambulance_route", typeID="emergency", depart="now")
            except traci.TraCIException as e:
                print(f"[{label}] Could not insert ambulance: {e}")
            ambulance_inserted = True
            ambulance_depart_time = t

        controller.execute_control(traci.simulation.getCurrentTime())

        if ambulance_inserted and ambulance_arrive_time is None:
            if "ambulance_0" in traci.vehicle.getIDList():
                try:
                    edge = traci.vehicle.getRoadID("ambulance_0")
                    speed = traci.vehicle.getSpeed("ambulance_0")
                    dist = traci.vehicle.getDistance("ambulance_0")
                    ambulance_trace.append((t, edge, speed, dist))
                except traci.TraCIException:
                    pass
            elif "ambulance_0" in traci.simulation.getArrivedIDList():
                ambulance_arrive_time = t

    traci.close()

    return {
        "tripinfo_path": tripinfo_path,
        "ambulance_trace": ambulance_trace,
        "ambulance_depart_time": ambulance_depart_time,
        "ambulance_arrive_time": ambulance_arrive_time,
        "preemption_events": controller.measurement_data["history_preemption_events"],
    }


def save_fig(fig, path):
    """Charts are written as real .png files next to the report rather than
    embedded as data URIs, so the Markdown report can reference them and
    stays readable/diffable as plain text."""
    fig.savefig(path, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return Path(path).name


def plot_ambulance_progress(no_corridor_trace, with_corridor_trace, path):
    fig, ax = plt.subplots(figsize=(8, 4))
    for trace, label, style in ((no_corridor_trace, "no corridor", "--"), (with_corridor_trace, "with corridor", "-")):
        if not trace:
            continue
        t0 = trace[0][0]
        ts = [row[0] - t0 for row in trace]
        dist = [row[3] for row in trace]
        ax.plot(ts, dist, style, label=label)
    ax.set_xlabel("Time since ambulance departure (s)")
    ax.set_ylabel("Distance traveled (m)")
    ax.set_title("Ambulance progress: with vs. without green corridor")
    ax.legend()
    return save_fig(fig, path)


def plot_ambulance_speed(no_corridor_trace, with_corridor_trace, path):
    fig, ax = plt.subplots(figsize=(8, 3.5))
    for trace, label, style in ((no_corridor_trace, "no corridor", "--"), (with_corridor_trace, "with corridor", "-")):
        if not trace:
            continue
        t0 = trace[0][0]
        ts = [row[0] - t0 for row in trace]
        speed = [row[2] for row in trace]
        ax.plot(ts, speed, style, label=label, alpha=0.8)
    ax.set_xlabel("Time since ambulance departure (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Ambulance speed profile (stops = signal delay)")
    ax.legend()
    return save_fig(fig, path)


def plot_background_traffic(no_corridor_trips, with_corridor_trips, path):
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.5))
    for ax, metric, label in zip(axes, ["waitingTime", "timeLoss"], ["Avg waiting time (s)", "Avg time loss (s)"]):
        vals = [
            statistics.mean(t[metric] for t in no_corridor_trips) if no_corridor_trips else 0,
            statistics.mean(t[metric] for t in with_corridor_trips) if with_corridor_trips else 0,
        ]
        ax.bar(["no corridor", "with corridor"], vals, color=["#888", "#2b7"])
        ax.set_title(label, fontsize=9)
    fig.suptitle("Background traffic impact (all OTHER vehicles, ambulance excluded)")
    return save_fig(fig, path)


def _run(args):
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    parser_obj = BangaloreNetworkParser(NET_FILE)
    graph_builder = BangaloreGraphBuilder(parser_obj)

    # One hospital, one route - planned once and driven by BOTH scenarios.
    plan = plan_hospital_route(args.hospital, args.role, seed=args.seed,
                                graph_builder=graph_builder)
    hospital, role, route_edges = plan["hospital"], plan["role"], plan["route_edges"]
    from_edge, to_edge = plan["from_edge"], plan["to_edge"]

    # One label for this whole invocation, shared by both scenarios, so this
    # run's outputs stay together and never overwrite an earlier run's.
    run_group = args.label or make_run_group(f"ambulance_{hospital.key}_{role}")
    out_dir = out_dir / run_group

    print(f"\nRun:        {run_group}")
    print(f"Hospital:   {hospital.name}  (route {role}, colour {hospital.color_str()})")
    print(f"Route:      {from_edge} -> {to_edge}  ({len(route_edges)} edges"
          + (f", {plan['n_signals']} signalised junctions)" if plan["n_signals"] is not None else ")"))
    print(f"Other end:  {plan['other_edge']}  (randomly chosen, seed={args.seed})\n")

    out_dir.mkdir(parents=True, exist_ok=True)

    # Scenario 1 - the EVP solution under test.
    print("=== [1/2] Running WITH green corridor (EVP preemption enabled) ===")
    with_corridor = run_scenario("with_corridor", True, route_edges, args.duration, out_dir)

    # Scenario 2 - the same route again, as plain adaptive control, so the
    # only difference between the two numbers is the EVP layer itself.
    print("=== [2/2] Running WITHOUT green corridor (baseline VAC, no EVP) ===")
    no_corridor = run_scenario("no_corridor", False, route_edges, args.duration, out_dir)

    def travel_time(res):
        if res["ambulance_depart_time"] is None or res["ambulance_arrive_time"] is None:
            return None
        return res["ambulance_arrive_time"] - res["ambulance_depart_time"]

    tt_no_corridor = travel_time(no_corridor)
    tt_with_corridor = travel_time(with_corridor)

    no_corridor_bg = load_tripinfos_xml(no_corridor["tripinfo_path"], exclude_id="ambulance_0")
    with_corridor_bg = load_tripinfos_xml(with_corridor["tripinfo_path"], exclude_id="ambulance_0")

    # Persist a standard-shaped run record for each scenario (report.py) so
    # this run can be compared against every other ambulance run ever done,
    # not just the other scenario in this same invocation.
    def build_record(label, corridor_enabled, bg_trips, result, travel_time_s):
        metrics = tripinfo_list_metrics(bg_trips)
        metrics["ambulance_travel_time_s"] = travel_time_s
        metrics["preemption_events"] = len(result["preemption_events"])
        amb = get_tripinfo_by_id(result["tripinfo_path"], "ambulance_0")
        if amb:
            metrics["ambulance_route_length_m"] = amb["routeLength"]
            metrics["ambulance_waiting_time_s"] = amb["waitingTime"]
        record = new_run_record(
            "ambulance", label, metrics, params=SCOSCA_PARAMS,
            run_group=run_group,
            display_name="EVP (green corridor)" if corridor_enabled else "Baseline VAC (no EVP)",
            duration_sec=args.duration, seed=args.seed,
            context={
                "scenario": ("EVP green corridor ON" if corridor_enabled
                             else "baseline VAC - no EVP"),
                "corridor_enabled": corridor_enabled,
                "hospital": hospital.name,
                "hospital_role": f"route {role}",
                "hospital_color": hospital.color_str(),
                "route_from": from_edge,
                "route_to": to_edge,
                "route_edges": len(route_edges),
                "signalised_junctions_on_route": plan["n_signals"],
                "ambulance_depart_s": AMBULANCE_DEPART,
                "network_file": Path(NET_FILE).name,
            },
        )
        save_run_record(record)
        save_run_report_md(record)
        return record

    with_record = build_record("with_corridor", True, with_corridor_bg, with_corridor, tt_with_corridor)
    no_record = build_record("no_corridor", False, no_corridor_bg, no_corridor, tt_no_corridor)
    print(f"Saved run records: {no_record['run_id']}, {with_record['run_id']}")
    print("Run `python report.py --run-type ambulance` anytime to compare against every ambulance run ever done.")

    print("\n=== RESULTS ===")
    print(f"Hospital: {hospital.name} (route {role})")
    print(f"Ambulance travel time WITH corridor (EVP):    {tt_with_corridor}")
    print(f"Ambulance travel time WITHOUT corridor (VAC): {tt_no_corridor}")
    if tt_with_corridor and tt_no_corridor:
        saved = tt_no_corridor - tt_with_corridor
        print(f"  -> EVP saved {saved:.2f}s ({saved / tt_no_corridor * 100:.1f}% faster)")
    print(f"Preemption events (with corridor): {len(with_corridor['preemption_events'])}")
    if no_corridor_bg:
        print(f"Background avg waitingTime WITHOUT corridor: {statistics.mean(t['waitingTime'] for t in no_corridor_bg):.2f}")
    if with_corridor_bg:
        print(f"Background avg waitingTime WITH corridor:    {statistics.mean(t['waitingTime'] for t in with_corridor_bg):.2f}")

    img_progress = plot_ambulance_progress(
        no_corridor["ambulance_trace"], with_corridor["ambulance_trace"], out_dir / "ambulance_progress.png")
    img_speed = plot_ambulance_speed(
        no_corridor["ambulance_trace"], with_corridor["ambulance_trace"], out_dir / "ambulance_speed.png")
    img_bg = plot_background_traffic(
        no_corridor_bg, with_corridor_bg, out_dir / "background_traffic.png")

    incomplete_note = (
        "\n> **Warning:** the ambulance did not complete its trip in both scenarios - "
        "increase `--duration` before drawing conclusions from the travel-time numbers below.\n"
        if not (tt_no_corridor and tt_with_corridor) else ""
    )

    signals_txt = (f", {plan['n_signals']} signalised junctions"
                    if plan["n_signals"] is not None else "")
    run_header_md = f"""# EVATO run report: {hospital.name}

**Run:** `{run_group}`
**Hospital:** {hospital.name}  (this run's colour: `{hospital.color_str()}`)
**Hospital's role on the route:** {role}
**Route:** `{from_edge}` -> `{to_edge}`  ({len(route_edges)} edges{signals_txt})
**Randomly chosen other endpoint:** `{plan['other_edge']}`  (seed `{args.seed}`, so this route is reproducible)
**Ambulance departs:** t={AMBULANCE_DEPART:.0f}s of a {args.duration}s simulation

This run drives that one route **twice**, changing nothing but the EVP layer:

| # | Scenario | Description |
|---|---|---|
| 1 | **EVP green corridor ON** | `BangaloreSCOSCA._update_emergency_preemption` forces the phase serving the ambulance's approach at every signal on its immediate path, holds it only while the ambulance is still approaching, and releases it the instant it clears. |
| 2 | **Baseline VAC - no EVP** | Identical adaptive signal control, but the ambulance gets no priority; it is treated as any other vehicle. |

Because both scenarios drive an identical route with an identical seed, any
difference below is attributable to the EVP layer itself.
{incomplete_note}
"""

    verdict_md = render_evp_verdict_markdown(no_record, with_record)

    comparison_md = render_comparison_markdown(
        [no_record, with_record],
        title="Full metric comparison: baseline VAC vs. EVP",
        description=(
            "Every metric recorded for both scenarios. Percentages compare the EVP run "
            "against the baseline VAC run."
        ),
    )

    charts_md = f"""
## Charts

### Ambulance trajectory

![Ambulance distance traveled over time, with vs. without corridor]({img_progress})

Distance traveled over time since departure. Flat stretches = stopped at a red light.
Fewer/shorter flat stretches with the corridor enabled means fewer, shorter stops.

![Ambulance speed profile, with vs. without corridor]({img_speed})

Speed profile - drops to 0 indicate a stop. The corridor should visibly reduce how often and
how long the ambulance sits at 0 speed.

### Impact on background (normal) traffic

![Background traffic waiting time and time loss, with vs. without corridor]({img_bg})

"No further traffic created" is checked here directly: the preemption only holds a green
while the ambulance is genuinely on that intersection's immediate approach (lookahead =
{SCOSCA_PARAMS['preemption_lookahead_edges']} edges) and releases the instant it passes, so any
increase in background delay (see avg. waiting time / avg. time loss in the table above) should be
small and localized to the corridor's own cross streets - not a network-wide regression.
"""

    report_path = out_dir / "ambulance_report.md"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(run_header_md + "\n" + verdict_md + "\n" + comparison_md + "\n" + charts_md)
    print(f"\nReport written to: {report_path}")
    print(f"Per-run reports written to: {RUN_REPORTS_DIR}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=900)
    parser.add_argument("--out", type=str, default=str(Path(__file__).parent / "ambulance_output"))
    parser.add_argument("--hospital", type=str, default=None,
                         choices=sorted(HOSPITALS_BY_KEY),
                         help="which hospital this run is about; omit to be prompted to choose")
    parser.add_argument("--role", type=str, default=None,
                         choices=["start", "destination", "random"],
                         help="whether the hospital is the route's start or destination; omit to be prompted")
    parser.add_argument("--seed", type=int, default=42,
                         help="seed for the randomly chosen other endpoint, so a route is reproducible")
    parser.add_argument("--label", type=str, default=None,
                         help="name for this run (used for its output folder and report headings); "
                              "defaults to ambulance_<hospital>_<role>_<timestamp>")
    parser.add_argument("--validate", action="store_true",
                         help="capture this run's full output to a .log file and auto-generate an EV validation .md report from it (validation.py)")
    args = parser.parse_args()

    if not args.validate:
        _run(args)
        return

    log_path = VALIDATION_REPORTS_DIR / f"ev_run__{datetime.now():%Y%m%d_%H%M%S}.log"
    with capture_run_log(log_path):
        _run(args)
    report_path, overall = validate_ev_log(log_path)
    print(f"\nEV validation: {'PASS' if overall else 'FAIL'} -> {report_path}")


if __name__ == "__main__":
    main()
