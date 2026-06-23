import os
import sys

# 1. Base Directory and Library Alignment
base_dir = os.path.dirname(os.path.abspath(__file__))
sumo_tools = os.path.join(base_dir, ".venv", "Lib", "site-packages")

# FORCE HIGHEST PRIORITY: Insert the .venv site-packages at position 0
# This forces Python to look in the virtual environment BEFORE looking at your local folders
if sumo_tools in sys.path:
    sys.path.remove(sumo_tools)
sys.path.insert(0, sumo_tools)

os.environ["SUMO_HOME"] = os.path.join(sumo_tools, "sumo")

# Import TraCI and the sumoITScontrol library modules
import traci
from sumoITScontrol.controller import MaxPressureController

# 2. Map Configuration Anchors
MAP_FOLDER = "2026-06-18-09-55-06" 
CONFIG_FILE = os.path.join(base_dir, MAP_FOLDER, "osm.sumocfg")

# Your verified junction ID from earlier
TRAFFIC_LIGHT_ID = "joinedS_1668866516_320647365"

# 3. Booting the Simulation Pipeline
sumo_cmd = ["sumo-gui", "-c", CONFIG_FILE]
print(f"Launching Framework with Map: {MAP_FOLDER} via sumoITScontrol...")
traci.start(sumo_cmd)

try:
    # 4. Initialize the Framework Controller
    its_controller = MaxPressureController(
        tls_id=TRAFFIC_LIGHT_ID,
        min_green=5.0,     # Minimum green time
        max_green=60.0,    # Max cap limits to avoid endless phases
        lost_time=3.0      # Transition yellow time
    )

    step = 0
    while step < 3000:
        traci.simulationStep()
        
        # 5. Let the Framework calculate state pressures
        its_controller.update(step_time=1.0)
        
        if step % 100 == 0:
            current_phase = traci.trafficlight.getPhase(TRAFFIC_LIGHT_ID)
            print(f"Step {step}: Framework active. Junction Phase context: {current_phase}")
            
        time.sleep(0.01)
        step += 1

except Exception as e:
    print(f"\nExecution broke during logic deployment: {e}")
finally:
    try:
        traci.close()
    except:
        pass
    print("Simulation session closed safely.")