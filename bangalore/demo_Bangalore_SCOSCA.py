import os
import sys
import argparse
from datetime import datetime
import warnings

# Add src to python path to import sumoITScontrol modules correctly
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))

os.environ["QT_X11_NO_MITSHM"] = "1"
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
os.environ["DYLD_LIBRARY_PATH"] = "/opt/homebrew/opt/mesa/lib:" + os.environ.get("DYLD_LIBRARY_PATH", "")
os.environ["FONTCONFIG_FILE"] = "/opt/X11/etc/X11/fontconfig/fonts.conf"
os.environ["FONTCONFIG_PATH"] = "/opt/X11/etc/X11/fontconfig"
os.environ["PROJ_LIB"] = "/Library/Frameworks/EclipseSUMO.framework/Versions/1.27.0/EclipseSUMO/framework/EclipseSUMO.framework/Versions/1.27.0/EclipseSUMO/share/proj"
os.environ["PROJ_DATA"] = "/Library/Frameworks/EclipseSUMO.framework/Versions/1.27.0/EclipseSUMO/framework/EclipseSUMO.framework/Versions/1.27.0/EclipseSUMO/share/proj"
import traci

warnings.filterwarnings("ignore")

from bangalore.parser import BangaloreNetworkParser
from bangalore.graph_builder import BangaloreGraphBuilder
from bangalore.scosca_controller import BangaloreSCOSCA

# Parse command line arguments
parser = argparse.ArgumentParser(description="Run Bangalore True SCOSCA Simulation.")
parser.add_argument("--headless", action="store_true", help="Run SUMO in headless mode")
parser.add_argument("--gui-delay", type=int, default=100, help="Delay in milliseconds between GUI steps")
parser.add_argument("--duration", type=int, default=3600, help="Simulation duration in seconds")
args, unknown = parser.parse_known_args()

############## PARAMETERS
simulation_parameters = {
    "sumo_config_file": "./Bangalore_Map/osm.sumocfg",
    "duration_sec": args.duration,
    "time_step": 0.25,
    "sumo_random_seed": 42,
}

# Determine SUMO command
if args.headless:
    SUMO_BINARY = "/Users/sachindev/sumo-env/bin/sumo"
    SUMO_CMD = [
        SUMO_BINARY,
        "-c",
        simulation_parameters["sumo_config_file"],
        "--time-to-teleport",
        "-1",
        "--seed",
        str(simulation_parameters["sumo_random_seed"]),
    ]
else:
    SUMO_BINARY = "/Applications/SUMO sumo-gui.app/Contents/MacOS/SUMO sumo-gui"
    SUMO_CMD = [
        SUMO_BINARY,
        "-c",
        simulation_parameters["sumo_config_file"],
        "--start",
        "--delay",
        str(args.gui_delay),
        "--time-to-teleport",
        "-1",
        "--seed",
        str(simulation_parameters["sumo_random_seed"]),
    ]

def main():
    print("==================================================")
    print("    Bangalore True SCOSCA Traffic Optimizer")
    print("==================================================")
    
    # 1. Parse network file
    net_path = "./Bangalore_Map/osm.net.xml.gz"
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
    }
    
    controller = BangaloreSCOSCA(scosca_params, parser_obj, graph_builder, initial_cycle_length=120)
    
    # 4. Start SUMO
    print("\nLaunching SUMO simulation...")
    print("Command:", " ".join(SUMO_CMD))
    traci.start(SUMO_CMD, numRetries=400)
    print("TraCI connected successfully!")
    
    # Allow GUI to load completely
    if not args.headless:
        import time
        print("Waiting 5 seconds for SUMO GUI to initialize...")
        time.sleep(5.0)
        
    # Initialize lane lengths and simulator tools
    controller.init_simulation()
    
    # 5. Run simulation loop
    total_steps = int(simulation_parameters["duration_sec"] / simulation_parameters["time_step"])
    print(f"Running simulation for {simulation_parameters['duration_sec']}s ({total_steps} steps)...")
    
    try:
        for step in range(total_steps):
            traci.simulationStep()
            if not args.headless:
                import time
                time.sleep(args.gui_delay / 1000.0)
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
