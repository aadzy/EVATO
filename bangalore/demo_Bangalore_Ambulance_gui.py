"""
Interactive (sumo-gui) run of the ambulance green-corridor scenario, for
visually watching EVATO's emergency preemption in action — a single run
with the corridor ENABLED (see demo_Bangalore_Ambulance.py for the
programmatic with/without comparison used for the written report).

The ambulance is colored red (vType "emergency"). The run's hospital is
chosen first (interactively, or via --hospital), and the full route is drawn
as a polyline in THAT hospital's colour for the whole run, with a marker at
the hospital end and a dark marker at the other end - so the corridor on
screen is visually tied to the labelled hospital pin it serves. Watch for:
the traffic light immediately ahead of the ambulance switching to serve its
approach (after a brief mandatory yellow if something else was green), and
switching back to normal SCOSCA operation the moment the ambulance clears it.

Usage:
    python demo_Bangalore_Ambulance_gui.py [--duration 900] [--delay 100]
    python demo_Bangalore_Ambulance_gui.py --hospital kidwai --role destination
"""

import argparse
import os
import sys
import time
import warnings
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
from demo_Bangalore_Ambulance import (
    plan_hospital_route, SCOSCA_PARAMS, INITIAL_CYCLE_LENGTH,
    AMBULANCE_DEPART, NET_FILE, SUMO_CFG, TIME_STEP,
)
from hospitals import draw_route_corridor, HOSPITALS_BY_KEY


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=900)
    parser.add_argument("--delay", type=int, default=100, help="ms between rendered steps in the GUI")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--hospital", type=str, default=None, choices=sorted(HOSPITALS_BY_KEY),
                         help="which hospital this run is about; omit to be prompted to choose")
    parser.add_argument("--role", type=str, default=None,
                         choices=["start", "destination", "random"],
                         help="whether the hospital is the route's start or destination; omit to be prompted")
    args = parser.parse_args()

    parser_obj = BangaloreNetworkParser(NET_FILE)
    graph_builder = BangaloreGraphBuilder(parser_obj)
    plan = plan_hospital_route(args.hospital, args.role, seed=args.seed,
                                graph_builder=graph_builder)
    hospital, role, route_edges = plan["hospital"], plan["role"], plan["route_edges"]
    print(f"\nHospital: {hospital.name}  (route {role}, colour {hospital.color_str()})")
    print(f"Route:    {plan['from_edge']} -> {plan['to_edge']} ({len(route_edges)} edges)\n")

    # plan_hospital_route opens/closes its own throwaway TraCI connection to
    # validate the route — build fresh objects for the real (GUI) run.
    parser_obj = BangaloreNetworkParser(NET_FILE)
    graph_builder = BangaloreGraphBuilder(parser_obj)
    controller = BangaloreSCOSCA(SCOSCA_PARAMS, parser_obj, graph_builder,
                                  initial_cycle_length=INITIAL_CYCLE_LENGTH)
    controller.evato_override_enabled = True  # green corridor ON for this viewing run

    sumo_cmd = [
        "sumo-gui", "-c", SUMO_CFG, "--start",
        "--delay", str(args.delay),
        "--time-to-teleport", "-1", "--seed", str(args.seed),
        "--step-length", str(TIME_STEP),
    ]
    print("Launching sumo-gui — a window should open on your screen...")
    print("Command:", " ".join(sumo_cmd))
    for attempt in range(3):
        try:
            traci.start(sumo_cmd, numRetries=200)
            break
        except traci.exceptions.FatalTraCIError as e:
            print(f"attempt {attempt+1}/3 failed: {e}")
            time.sleep(1.0)
    controller.init_simulation()
    draw_route_corridor(route_edges, hospital, role)

    traci.vehicletype.copy("DEFAULT_VEHTYPE", "emergency")
    traci.vehicletype.setVehicleClass("emergency", "emergency")
    traci.vehicletype.setColor("emergency", (255, 0, 0, 255))
    traci.vehicletype.setShapeClass("emergency", "emergency")
    traci.vehicletype.setSpeedFactor("emergency", 1.3)
    traci.vehicletype.setMinGap("emergency", 1.5)

    ambulance_inserted = False
    total_steps = int(args.duration / TIME_STEP)
    print(f"Running for {args.duration}s ({total_steps} steps) — ambulance departs at t={AMBULANCE_DEPART:.0f}s...")

    try:
        for step in range(total_steps):
            traci.simulationStep()
            t = traci.simulation.getCurrentTime() / 1000.0

            if not ambulance_inserted and t >= AMBULANCE_DEPART:
                traci.route.add("ambulance_route", route_edges)
                traci.vehicle.add("ambulance_0", routeID="ambulance_route", typeID="emergency", depart="now")
                try:
                    traci.gui.trackVehicle("View #0", "ambulance_0")
                    traci.gui.setZoom("View #0", 800)
                except traci.TraCIException:
                    pass
                ambulance_inserted = True
                print(f"[t={t:.0f}s] Ambulance inserted, GUI camera now tracking it.")

            controller.execute_control(traci.simulation.getCurrentTime())

            if ambulance_inserted and "ambulance_0" in traci.simulation.getArrivedIDList():
                print(f"[t={t:.0f}s] Ambulance arrived.")
                break

            if step % 400 == 0:
                print(f"  Progress: {step}/{total_steps} (t={t:.0f}s), preemption events so far: {len(controller.measurement_data['history_preemption_events'])}")
    finally:
        print("Closing TraCI...")
        traci.close()


if __name__ == "__main__":
    main()
