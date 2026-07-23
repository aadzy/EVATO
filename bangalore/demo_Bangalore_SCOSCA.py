import argparse
import os
import sys
from datetime import datetime
from pathlib import Path
import warnings

BASE_DIR = Path(__file__).resolve().parent.parent
SRC_DIR = BASE_DIR / "src"
# Add the project root and src directory to python path so the demo works from any cwd
for path in (BASE_DIR, SRC_DIR):
    if path.exists() and str(path) not in sys.path:
        sys.path.insert(0, str(path))

SUMO_HOME = os.environ.get("SUMO_HOME", "D:\\")
os.environ["PROJ_LIB"] = os.path.join(SUMO_HOME, "share", "proj")
os.environ["PROJ_DATA"] = os.path.join(SUMO_HOME, "share", "proj")
import traci

warnings.filterwarnings("ignore")

from bangalore.parser import BangaloreNetworkParser
from bangalore.graph_builder import BangaloreGraphBuilder
from bangalore.scosca_controller import BangaloreSCOSCA


def _resolve_default_config_file() -> Path:
    candidates = [
        BASE_DIR / "Bangalore_Map" / "osm.sumocfg",
        BASE_DIR / "Bangalore_Map" / "osm.sumocfg.xml",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def _resolve_default_network_file() -> Path:
    candidates = [
        BASE_DIR / "Bangalore_Map" / "osm.net.xml.gz",
        BASE_DIR / "Bangalore_Map" / "osm.net.xml",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


DEFAULT_CONFIG_FILE = _resolve_default_config_file()
DEFAULT_NETWORK_FILE = _resolve_default_network_file()

# Parse command line arguments
parser = argparse.ArgumentParser(description="Run Bangalore True SCOSCA Simulation.")
parser.add_argument("--headless", action="store_true", help="Run SUMO in headless mode")
parser.add_argument("--duration", type=int, default=3600, help="Simulation duration in seconds")
parser.add_argument("--seed", type=int, default=42, help="Random seed for SUMO")
args, unknown = parser.parse_known_args()

############## PARAMETERS
simulation_parameters = {
    "sumo_config_file": str(DEFAULT_CONFIG_FILE),
    "network_file": str(DEFAULT_NETWORK_FILE),
    "duration_sec": args.duration,
    "time_step": 0.25,
    "sumo_random_seed": args.seed,
}

# Determine SUMO command
SUMO_BINARY = "sumo" if args.headless else "sumo-gui"
SUMO_CMD = [
    SUMO_BINARY,
    "-c",
    simulation_parameters["sumo_config_file"],
    "--start",
    "--quit-on-end",
    "--time-to-teleport",
    "-1",
    "--seed",
    str(simulation_parameters["sumo_random_seed"]),
]

if args.headless:
    SUMO_CMD = ["sumo"] + SUMO_CMD[1:]

def main():
    print("==================================================")
    print("    Bangalore True SCOSCA Traffic Optimizer")
    print("==================================================")
    
    # 1. Parse network file
    net_path = simulation_parameters["network_file"]
    print(f"Parsing network file: {net_path}...")
    parser_obj = BangaloreNetworkParser(net_path)
    
    # 2. Build graph and group traffic lights
    print("Building adjacency graph and clustering traffic lights...")
    graph_builder = BangaloreGraphBuilder(parser_obj)
    
    districts = graph_builder.get_districts()
    print(f"Discovered {len(parser_obj.get_tls())} traffic lights partitioned into {len(districts)} districts:")
    for d_name, tls in districts.items():
        print(f"  - {d_name}: {len(tls)} traffic lights ({tls[:3]}...)")

    # 3. Instantiate SCOSCA controller
    scosca_params = {
        "adaptation_cycle": 30,
        "adaptation_green": 10,
        "green_thresh": 2,
        "adaptation_offset": 1,
        "offset_thresh": 0.5,
        "min_cycle_length": 50,
        "max_cycle_length": 180,
        "ds_upper_val": 0.925,
        "ds_lower_val": 0.875,
        "measurement_period": int(1 / simulation_parameters["time_step"]),
        # CoSiCoSt-alignment fine-tuning (see bangalore/optimizer.py & intersection.py):
        "priority_stage_boost": 5.0,        # extra DS-equivalent weight for the phase serving the current priority route
        "priority_route_delay_weight": 0.05, # weight of accumulated waiting-time vs. queue count in priority-route scoring
        "actuation_extension_sec": 5,        # bounded local green extension (Online Split Optimizer) if stop-line still occupied
        "actuation_min_green_ratio": 0.5,    # mandatory-minimum-green fraction of the computed split before early termination is allowed
        "actuation_min_green_floor": 5,      # absolute floor (s) under which a phase is never gapped out early
        "actuation_gap_thresh": 3.0,         # seconds of no detector actuation before a green phase gaps out
        "fallback_missing_cycles": 3,        # consecutive cycles of missing detector/comms data before a district falls back
        "fallback_min_green": 10,            # min green per phase while in Full Vehicle Actuation fallback
        "fallback_max_green": 45,            # max green per phase while in Full Vehicle Actuation fallback
    }
    
    controller = BangaloreSCOSCA(scosca_params, parser_obj, graph_builder, initial_cycle_length=120)
    
    # 4. Start SUMO
    print("\nLaunching SUMO simulation...")
    print("Command:", " ".join(SUMO_CMD))
    traci.start(SUMO_CMD, numRetries=400)
    print("TraCI connected successfully!")
        
    # Initialize lane lengths and simulator tools
    controller.init_simulation()
    
    # 5. Run simulation loop
    total_steps = int(simulation_parameters["duration_sec"] / simulation_parameters["time_step"])
    print(f"Running simulation for {simulation_parameters['duration_sec']}s ({total_steps} steps)...")
    
    try:
        for step in range(total_steps):
            traci.simulationStep()
            current_time = traci.simulation.getCurrentTime()
            controller.execute_control(current_time)
            
            if step % 400 == 0:
                print(f"  Progress: {step}/{total_steps} steps (Sim Time: {current_time/1000.0}s)")
    except Exception as e:
        print("Error during simulation:", e)
        raise e
    finally:
        print("Simulation finished. Closing TraCI...")
        traci.close()
        
    print("\nVerification Summary:")
    print(f"Successfully ran Bangalore SCOSCA control for {len(controller.intersections)} traffic lights.")
    print("Signal adaptation histories recorded in controller.measurement_data.")
    print("Done!")

if __name__ == "__main__":
    main()
