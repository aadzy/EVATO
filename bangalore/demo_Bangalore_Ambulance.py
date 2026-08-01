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

Usage:
    python demo_Bangalore_Ambulance.py [--duration 900] [--out DIR]

Produces (in --out, default ./ambulance_output):
    - no_corridor_tripinfos.xml / with_corridor_tripinfos.xml
    - ambulance_report.html (dashboard: ambulance trajectory, travel time
      comparison, and background-traffic impact)
"""

import argparse
import io
import base64
import os
import statistics
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
from report import (
    new_run_record, save_run_record, tripinfo_list_metrics,
    load_tripinfos_xml, render_comparison_table_html, render_report_page,
)

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


def pick_ambulance_edges(graph_builder, max_route_edges=40, min_signals=3):
    """Picks a short, mostly-signalized route: candidates are drawn from a
    single district's critical_district_order chain (not a cross-district
    trip), and each candidate route is scored by how many of our known
    working traffic lights (edge_to_tl) it actually passes through, with a
    cap on total edge count so it stays a short, direct corridor rather than
    the long, mostly-unsignalized detours findRoute can otherwise return."""
    districts = graph_builder.get_districts()
    critical_order = graph_builder.get_critical_district_order()
    edge_to_tl = graph_builder.get_edge_to_tl()

    # Prefer the district with the most traffic lights — more chances for
    # the ambulance to actually interact with the green corridor.
    district_name = max(districts, key=lambda d: len(districts[d]))
    ordered_tls = critical_order.get(district_name, districts[district_name])
    if len(ordered_tls) < min_signals:
        raise RuntimeError(f"District '{district_name}' has too few signals ({len(ordered_tls)}) for a corridor demo")

    def incoming_edges(tl_id):
        return list({lane.rsplit("_", 1)[0] for lane in graph_builder.graph[tl_id]["incoming_lanes"]})

    n = len(ordered_tls)
    # Candidate (start, end) tl pairs along the chain: full span, then a few
    # shorter spans, so we can fall back to something shorter if the full
    # span is too long or too indirect.
    span_fracs = [(0, n - 1), (0, n // 2), (n // 2, n - 1), (0, max(1, n // 3)), (max(0, n - n // 3), n - 1)]

    best = None  # (score, -len(edges), from_edge, to_edge, edges)
    sumo_cmd = ["sumo", "-c", SUMO_CFG, "--start", "--quit-on-end", "--no-step-log", "true"]
    traci.start(sumo_cmd, numRetries=100)
    try:
        for start_idx, end_idx in span_fracs:
            start_tl, end_tl = ordered_tls[start_idx], ordered_tls[end_idx]
            for from_edge in incoming_edges(start_tl)[:5]:
                for to_edge in incoming_edges(end_tl)[:5]:
                    if from_edge == to_edge:
                        continue
                    try:
                        route = traci.simulation.findRoute(from_edge, to_edge)
                    except traci.TraCIException:
                        continue
                    route_edges = list(route.edges)
                    if not route_edges or len(route_edges) > max_route_edges:
                        continue
                    n_signals = len({edge_to_tl[e] for e in route_edges if e in edge_to_tl})
                    if n_signals < min_signals:
                        continue
                    score = (n_signals, -len(route_edges))
                    if best is None or score > best[0]:
                        best = (score, from_edge, to_edge, route_edges)
        if best is not None:
            _, from_edge, to_edge, route_edges = best
            return from_edge, to_edge, route_edges
    finally:
        traci.close()
    raise RuntimeError(
        f"Could not find a route through >= {min_signals} signalized intersections "
        f"within {max_route_edges} edges in district '{district_name}'"
    )


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


def get_ambulance_tripinfo(path):
    if not os.path.exists(path):
        return None
    root = ET.parse(path).getroot()
    for t in root.findall("tripinfo"):
        if t.get("id") == "ambulance_0":
            return {
                "duration": float(t.get("duration")),
                "routeLength": float(t.get("routeLength")),
                "waitingTime": float(t.get("waitingTime")),
                "timeLoss": float(t.get("timeLoss")),
            }
    return None


def fig_to_data_uri(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode("ascii")


def plot_ambulance_progress(no_corridor_trace, with_corridor_trace):
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
    return fig_to_data_uri(fig)


def plot_ambulance_speed(no_corridor_trace, with_corridor_trace):
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
    return fig_to_data_uri(fig)


def plot_background_traffic(no_corridor_trips, with_corridor_trips):
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.5))
    for ax, metric, label in zip(axes, ["waitingTime", "timeLoss"], ["Avg waiting time (s)", "Avg time loss (s)"]):
        vals = [
            statistics.mean(t[metric] for t in no_corridor_trips) if no_corridor_trips else 0,
            statistics.mean(t[metric] for t in with_corridor_trips) if with_corridor_trips else 0,
        ]
        ax.bar(["no corridor", "with corridor"], vals, color=["#888", "#2b7"])
        ax.set_title(label, fontsize=9)
    fig.suptitle("Background traffic impact (all OTHER vehicles, ambulance excluded)")
    return fig_to_data_uri(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=900)
    parser.add_argument("--out", type=str, default=str(Path(__file__).parent / "ambulance_output"))
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Picking a cross-network ambulance route...")
    parser_obj = BangaloreNetworkParser(NET_FILE)
    graph_builder = BangaloreGraphBuilder(parser_obj)
    from_edge, to_edge, route_edges = pick_ambulance_edges(graph_builder)
    print(f"Ambulance route: {from_edge} -> {to_edge} ({len(route_edges)} edges)")

    print("=== Running WITHOUT green corridor (preemption disabled) ===")
    no_corridor = run_scenario("no_corridor", False, route_edges, args.duration, out_dir)

    print("=== Running WITH green corridor (preemption enabled) ===")
    with_corridor = run_scenario("with_corridor", True, route_edges, args.duration, out_dir)

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
    no_metrics = tripinfo_list_metrics(no_corridor_bg)
    no_metrics["ambulance_travel_time_s"] = tt_no_corridor
    no_metrics["preemption_events"] = len(no_corridor["preemption_events"])
    no_record = new_run_record(
        "ambulance", "no_corridor", no_metrics, params=SCOSCA_PARAMS,
        duration_sec=args.duration,
        notes=[f"route: {from_edge} -> {to_edge} ({len(route_edges)} edges)"],
    )
    save_run_record(no_record)

    with_metrics = tripinfo_list_metrics(with_corridor_bg)
    with_metrics["ambulance_travel_time_s"] = tt_with_corridor
    with_metrics["preemption_events"] = len(with_corridor["preemption_events"])
    with_record = new_run_record(
        "ambulance", "with_corridor", with_metrics, params=SCOSCA_PARAMS,
        duration_sec=args.duration,
        notes=[f"route: {from_edge} -> {to_edge} ({len(route_edges)} edges)"],
    )
    save_run_record(with_record)
    print(f"Saved run records: {no_record['run_id']}, {with_record['run_id']}")
    print("Run `python report.py --run-type ambulance` anytime to compare against every ambulance run ever done.")

    print("\n=== RESULTS ===")
    print(f"Ambulance travel time WITHOUT corridor: {tt_no_corridor}")
    print(f"Ambulance travel time WITH corridor:    {tt_with_corridor}")
    print(f"Preemption events (with corridor): {len(with_corridor['preemption_events'])}")
    if no_corridor_bg:
        print(f"Background avg waitingTime WITHOUT corridor: {statistics.mean(t['waitingTime'] for t in no_corridor_bg):.2f}")
    if with_corridor_bg:
        print(f"Background avg waitingTime WITH corridor:    {statistics.mean(t['waitingTime'] for t in with_corridor_bg):.2f}")

    img_progress = plot_ambulance_progress(no_corridor["ambulance_trace"], with_corridor["ambulance_trace"])
    img_speed = plot_ambulance_speed(no_corridor["ambulance_trace"], with_corridor["ambulance_trace"])
    img_bg = plot_background_traffic(no_corridor_bg, with_corridor_bg)

    if not (tt_no_corridor and tt_with_corridor):
        travel_time_note = (
            '<p style="color:var(--bad)"><b>Ambulance did not complete its trip in both runs '
            "- increase --duration.</b></p>"
        )
    else:
        travel_time_note = ""

    body = f"""<h2>Standardized comparison</h2>
<p>Every metric below has a one-line explanation and a direction (higher/lower better) registered once in
<code>report.py</code>'s metric catalog, and both runs are saved to <code>run_records/</code> so they can be
compared against any other ambulance run later - not just against each other.</p>
{travel_time_note}
{render_comparison_table_html([no_record, with_record])}

<h2>Ambulance trajectory</h2>
<figure>
<img src="{img_progress}" alt="Ambulance distance traveled over time, with vs. without corridor">
<figcaption>Distance traveled over time since departure. Flat stretches = stopped at a red light.
Fewer/shorter flat stretches with the corridor enabled means fewer, shorter stops.</figcaption>
</figure>
<figure>
<img src="{img_speed}" alt="Ambulance speed profile, with vs. without corridor">
<figcaption>Speed profile - drops to 0 indicate a stop. The corridor should visibly reduce how often and
how long the ambulance sits at 0 speed.</figcaption>
</figure>

<h2>Impact on background (normal) traffic</h2>
<figure>
<img src="{img_bg}" alt="Background traffic waiting time and time loss, with vs. without corridor">
<figcaption>"No further traffic created" is checked here directly: the preemption only holds a green
while the ambulance is genuinely on that intersection's immediate approach (lookahead =
{SCOSCA_PARAMS['preemption_lookahead_edges']} edges) and releases the instant it passes, so any
increase in background delay (see avg. waiting time / avg. time loss in the table above) should be
small and localized to the corridor's own cross streets - not a network-wide regression.</figcaption>
</figure>
"""

    html = render_report_page(
        "EVATO emergency-vehicle green corridor: ambulance vs. normal traffic",
        f"Single ambulance ({from_edge} &rarr; {to_edge}, {len(route_edges)} edges, departs at "
        f"t={AMBULANCE_DEPART:.0f}s) inserted into the same {args.duration}s Bangalore SCOSCA simulation "
        "twice: once with green-corridor preemption disabled (normal SCOSCA treats it like any other "
        "vehicle), once enabled (BangaloreSCOSCA._update_emergency_preemption forces the phase serving "
        "its approach at every signal on its immediate path, holds only while it's still approaching, "
        "and releases the instant it clears).",
        body,
        meta=[("route edges", len(route_edges)), ("duration", f"{args.duration}s"),
              ("ambulance departs", f"t={AMBULANCE_DEPART:.0f}s")],
    )

    report_path = out_dir / "ambulance_report.html"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"\nReport written to: {report_path}")


if __name__ == "__main__":
    main()
