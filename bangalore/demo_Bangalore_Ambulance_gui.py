"""
Interactive (sumo-gui) run of the ambulance green-corridor scenario, for
visually watching EVATO's emergency preemption in action — a single run
with the corridor ENABLED (see demo_Bangalore_Ambulance.py for the
programmatic with/without comparison used for the written report).

The ambulance is colored red (vType "emergency"). The full route is drawn
as a magenta polyline on the map for the whole run, with a green marker at
the start point and a red/black marker at the stop point, so the corridor
is visible even before/after the ambulance is actually on screen. Watch
for: the traffic light immediately ahead of it switching to serve its
approach (after a brief mandatory yellow if something else was green), and
switching back to normal SCOSCA operation the moment the ambulance clears it.

Usage:
    python demo_Bangalore_Ambulance_gui.py [--duration 900] [--delay 100]
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
    pick_ambulance_edges, SCOSCA_PARAMS, INITIAL_CYCLE_LENGTH,
    AMBULANCE_DEPART, NET_FILE, SUMO_CFG, TIME_STEP,
)


def build_route_shape(route_edges):
    """Concatenates each edge's lane-0 shape into one continuous polyline in
    network coordinates, for drawing the corridor as a GUI polygon."""
    shape = []
    for edge_id in route_edges:
        try:
            pts = traci.lane.getShape(f"{edge_id}_0")
        except traci.TraCIException:
            continue
        if shape and pts and shape[-1] == pts[0]:
            shape.extend(pts[1:])
        else:
            shape.extend(pts)
    return shape


def draw_route_markers(route_edges):
    """Highlights the ambulance's corridor: the full route as a magenta
    polyline, and distinct markers at its start and stop points."""
    shape = build_route_shape(route_edges)
    if not shape:
        return
    try:
        traci.polygon.add("ambulance_corridor", shape, (255, 0, 255, 200),
                           fill=False, layer=20, lineWidth=3)
        start_x, start_y = shape[0]
        end_x, end_y = shape[-1]
        traci.poi.add("ambulance_start", start_x, start_y, (0, 220, 0, 255),
                       poiType="ambulance_start", layer=25, width=20, height=20)
        traci.poi.add("ambulance_stop", end_x, end_y, (20, 20, 20, 255),
                       poiType="ambulance_stop", layer=25, width=20, height=20)
    except traci.TraCIException as e:
        print(f"Could not draw route markers: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=900)
    parser.add_argument("--delay", type=int, default=100, help="ms between rendered steps in the GUI")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    print("Picking a cross-network ambulance route...")
    parser_obj = BangaloreNetworkParser(NET_FILE)
    graph_builder = BangaloreGraphBuilder(parser_obj)
    from_edge, to_edge, route_edges = pick_ambulance_edges(graph_builder)
    print(f"Ambulance route: {from_edge} -> {to_edge} ({len(route_edges)} edges)")

    # pick_ambulance_edges opens/closes its own throwaway TraCI connection to
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
    draw_route_markers(route_edges)

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
