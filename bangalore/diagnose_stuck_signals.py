"""
Objective stuck-signal detector: runs the normal Bangalore SCOSCA simulation
headless for a set duration, tracking every traffic light's phase-change
history. Flags any tl_id whose longest gap between phase changes is
suspicious (either far longer than a real cycle should ever be, or that
never shows some of its own defined green states at all), instead of
relying on a fleeting visual impression from the GUI.

Usage:
    python diagnose_stuck_signals.py [--duration 900]
"""

import argparse
import os
import sys
import warnings
from collections import defaultdict
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

from bangalore.parser import BangaloreNetworkParser
from bangalore.graph_builder import BangaloreGraphBuilder
from bangalore.scosca_controller import BangaloreSCOSCA
from report import new_run_record, save_run_record

NET_FILE = str(BASE_DIR / "Bangalore_Map" / "osm.net.xml.gz")
SUMO_CFG = str(BASE_DIR / "Bangalore_Map" / "osm.sumocfg")
TIME_STEP = 0.25

SCOSCA_PARAMS = {
    "adaptation_cycle": 30, "adaptation_green": 10, "green_thresh": 2,
    "adaptation_offset": 1, "offset_thresh": 0.5, "min_cycle_length": 50,
    "max_cycle_length": 180, "ds_upper_val": 0.925, "ds_lower_val": 0.875,
    "measurement_period": int(1 / TIME_STEP),
    "priority_stage_boost": 5.0, "priority_route_delay_weight": 0.05,
    "actuation_extension_sec": 5, "actuation_min_green_ratio": 0.5,
    "actuation_min_green_floor": 5, "actuation_gap_thresh": 3.0,
    "fallback_missing_cycles": 3, "fallback_min_green": 10, "fallback_max_green": 45,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=900)
    parser.add_argument("--stuck_threshold_sec", type=float, default=150.0,
                         help="flag a tl if it goes this long without a green-state change")
    parser.add_argument("--label", type=str, default="run",
                         help="tag for this run's saved record, e.g. 'before_fix'/'after_fix', so later comparisons are meaningful")
    args = parser.parse_args()

    parser_obj = BangaloreNetworkParser(NET_FILE)
    graph_builder = BangaloreGraphBuilder(parser_obj)
    controller = BangaloreSCOSCA(SCOSCA_PARAMS, parser_obj, graph_builder, initial_cycle_length=120)

    sumo_cmd = ["sumo", "-c", SUMO_CFG, "--start", "--quit-on-end",
                "--time-to-teleport", "-1", "--seed", "42",
                "--no-step-log", "true", "--step-length", str(TIME_STEP)]
    traci.start(sumo_cmd, numRetries=100)
    controller.init_simulation()

    tl_ids = [i.tl_id for i in controller.intersections]
    last_state = {tl: None for tl in tl_ids}
    last_change_time = {tl: 0.0 for tl in tl_ids}
    longest_gap = {tl: 0.0 for tl in tl_ids}
    states_seen = defaultdict(set)  # tl -> set of distinct RYG states observed
    never_green_at_all = set(tl_ids)  # tls that have shown ANY green char, removed once seen

    total_steps = int(args.duration / TIME_STEP)
    for step in range(total_steps):
        traci.simulationStep()
        t = traci.simulation.getCurrentTime() / 1000.0
        controller.execute_control(traci.simulation.getCurrentTime())

        for tl in tl_ids:
            try:
                state = traci.trafficlight.getRedYellowGreenState(tl)
            except traci.TraCIException:
                continue
            states_seen[tl].add(state)
            if "g" in state.lower():
                never_green_at_all.discard(tl)
            if state != last_state[tl]:
                gap = t - last_change_time[tl]
                longest_gap[tl] = max(longest_gap[tl], gap)
                last_change_time[tl] = t
                last_state[tl] = state

    # Final gap check (from last change to end of run)
    for tl in tl_ids:
        gap = args.duration - last_change_time[tl]
        longest_gap[tl] = max(longest_gap[tl], gap)

    traci.close()

    print(f"\n=== Stuck-signal diagnostic over {args.duration}s ({len(tl_ids)} traffic lights) ===\n")

    print("Traffic lights that NEVER showed any green state at all (genuinely broken):")
    if never_green_at_all:
        for tl in sorted(never_green_at_all):
            print(f"  {tl}")
    else:
        print("  none")

    print(f"\nTraffic lights with a phase-change gap longer than {args.stuck_threshold_sec:.0f}s "
          f"(possibly stuck for a while):")
    flagged = {tl: g for tl, g in longest_gap.items() if g > args.stuck_threshold_sec}
    if flagged:
        for tl, gap in sorted(flagged.items(), key=lambda kv: -kv[1]):
            print(f"  {tl}: longest gap = {gap:.1f}s, distinct states shown = {len(states_seen[tl])}")
    else:
        print("  none")

    print(f"\nAll {len(tl_ids)} traffic lights, longest gap between phase changes (sorted worst-first):")
    for tl, gap in sorted(longest_gap.items(), key=lambda kv: -kv[1])[:10]:
        print(f"  {tl[:60]:60s}  longest_gap={gap:6.1f}s  distinct_states={len(states_seen[tl])}")

    record = new_run_record(
        "stuck_signal_diag", args.label,
        metrics={
            "n_signals_never_green": len(never_green_at_all),
            "max_phase_gap_s": max(longest_gap.values()) if longest_gap else 0.0,
            "n_signals_flagged_gap": len(flagged),
            "n_traffic_lights": len(tl_ids),
        },
        duration_sec=args.duration,
        notes=[f"stuck_threshold_sec={args.stuck_threshold_sec}"],
    )
    save_run_record(record)
    print(f"\nSaved run record: {record['run_id']}")
    print("Run `python report.py --run-type stuck_signal_diag` anytime to compare against every diagnostic run ever done.")


if __name__ == "__main__":
    main()
