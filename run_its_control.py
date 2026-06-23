import time
from pathlib import Path
import traci

# ============================================================
# 1. CONFIG
# ============================================================
BASE_DIR = Path(__file__).resolve().parent
MAP_FOLDER = "2026-06-18-09-55-06"
CONFIG_FILE = BASE_DIR / MAP_FOLDER / "osm.sumocfg"

TRAFFIC_LIGHT_ID = "joinedS_570758310_9939398569_9939398599_9939407828"

# ------------------------------------------------------------
# YOUR CONTROLLED APPROACHES
# ------------------------------------------------------------
APPROACH_TO_LANES = {
    "A": ["1207956704#3_0"],
    "B": ["172891296#1_0", "172891296#1_1"],
}

# ------------------------------------------------------------
# IMPORTANT: YOU MUST SET THESE 2 PHASE NUMBERS
#
# Replace 0 and 2 with the actual phase indices:
# - phase that gives green to approach A
# - phase that gives green to approach B
# ------------------------------------------------------------
APPROACH_TO_GREEN_PHASE = {
    "A": 0,   # <-- change if needed
    "B": 2,   # <-- change if needed
}

# Emergency vehicle detection
EMERGENCY_KEYWORDS = {"ambulance", "police", "fire", "emergency"}
EV_DETECTION_DISTANCE = 120.0   # meters from junction
GREEN_HOLD_STEPS = 10           # how long to keep EV green
MAX_STEPS = 4000

# ============================================================
# 2. HELPER FUNCTIONS
# ============================================================
def is_emergency_vehicle(veh_id: str) -> bool:
    """
    Detect whether a vehicle should get EV priority.
    First checks vehicle class, then type id / vehicle id keywords.
    """
    try:
        if traci.vehicle.getVehicleClass(veh_id) == "emergency":
            return True
    except:
        pass

    try:
        type_id = traci.vehicle.getTypeID(veh_id).lower()
        if any(k in type_id for k in EMERGENCY_KEYWORDS):
            return True
    except:
        pass

    try:
        if any(k in veh_id.lower() for k in EMERGENCY_KEYWORDS):
            return True
    except:
        pass

    return False


def get_tls_position(tls_id: str):
    """
    Approximate junction position using the end of the first controlled lane.
    """
    lanes = traci.trafficlight.getControlledLanes(tls_id)
    if not lanes:
        return None

    lane_shape = traci.lane.getShape(lanes[0])
    if not lane_shape:
        return None

    return lane_shape[-1]   # (x, y)


def distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def get_vehicle_approach(veh_id: str):
    """
    Map a vehicle to approach A/B based on its current lane.
    """
    try:
        lane_id = traci.vehicle.getLaneID(veh_id)
    except:
        return None

    for approach, lanes in APPROACH_TO_LANES.items():
        if lane_id in lanes:
            return approach
    return None


def find_nearest_emergency_vehicle():
    """
    Find the closest emergency vehicle approaching this junction.
    Returns (veh_id, approach, dist) or None.
    """
    tls_pos = get_tls_position(TRAFFIC_LIGHT_ID)
    if tls_pos is None:
        return None

    best = None
    best_dist = float("inf")

    for veh_id in traci.vehicle.getIDList():
        if not is_emergency_vehicle(veh_id):
            continue

        approach = get_vehicle_approach(veh_id)
        if approach is None:
            continue

        try:
            veh_pos = traci.vehicle.getPosition(veh_id)
            dist = distance(veh_pos, tls_pos)
        except:
            continue

        if dist <= EV_DETECTION_DISTANCE and dist < best_dist:
            best_dist = dist
            best = (veh_id, approach, dist)

    return best


def set_green_for_approach(approach: str):
    """
    Force the TLS to the green phase for the given approach.
    """
    target_phase = APPROACH_TO_GREEN_PHASE[approach]
    current_phase = traci.trafficlight.getPhase(TRAFFIC_LIGHT_ID)

    if current_phase != target_phase:
        traci.trafficlight.setPhase(TRAFFIC_LIGHT_ID, target_phase)

    # keep that phase active for a while
    traci.trafficlight.setPhaseDuration(TRAFFIC_LIGHT_ID, GREEN_HOLD_STEPS)


# ============================================================
# 3. EVATO CONTROLLER
# ============================================================
class EVATOController:
    def __init__(self):
        self.active_ev = None
        self.active_approach = None
        self.hold_counter = 0

    def step(self, step_num: int):
        # If EV priority is already active, continue holding it
        if self.hold_counter > 0:
            self.hold_counter -= 1

            # If EV vanished from simulation, release priority
            if self.active_ev and self.active_ev not in traci.vehicle.getIDList():
                self.reset()
                return

            if self.active_approach:
                set_green_for_approach(self.active_approach)
            return

        # Otherwise search for nearest EV
        ev_info = find_nearest_emergency_vehicle()

        if ev_info is None:
            return  # no EV -> SUMO normal program continues

        veh_id, approach, dist = ev_info
        self.active_ev = veh_id
        self.active_approach = approach
        self.hold_counter = GREEN_HOLD_STEPS

        print(
            f"[EVATO] Step {step_num}: EV '{veh_id}' detected on approach {approach} "
            f"at distance {dist:.1f} m -> forcing green."
        )

        set_green_for_approach(approach)

    def reset(self):
        self.active_ev = None
        self.active_approach = None
        self.hold_counter = 0


# ============================================================
# 4. MAIN
# ============================================================
def main():
    if not CONFIG_FILE.exists():
        raise FileNotFoundError(f"SUMO config file not found: {CONFIG_FILE}")

    sumo_cmd = ["sumo-gui", "-c", str(CONFIG_FILE)]
    print(f"Starting SUMO GUI with: {CONFIG_FILE}")
    traci.start(sumo_cmd)

    controller = EVATOController()

    try:
        step = 0
        while step < MAX_STEPS:
            traci.simulationStep()
            controller.step(step)

            if step % 100 == 0:
                phase = traci.trafficlight.getPhase(TRAFFIC_LIGHT_ID)
                print(f"Step {step}: current phase = {phase}")

            time.sleep(0.01)
            step += 1

    except Exception as e:
        print(f"\nSimulation error: {e}")

    finally:
        try:
            traci.close()
        except:
            pass
        print("Simulation closed safely.")


if __name__ == "__main__":
    main()