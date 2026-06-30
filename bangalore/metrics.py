# pyrefly: ignore [missing-import]
import traci
from sumoITScontrol.simulation_tools import SimulationTools

class BangaloreMetrics:
    def __init__(self, intersections):
        self.intersections = intersections
        self.previous_vehicle_ids = {}
        self.throughput_counts = {i.tl_id: 0 for i in intersections}

    def update_throughput(self):
        # We compute throughput by checking vehicles entering outgoing lanes.
        # Track vehicles currently on outgoing lanes of each intersection,
        # and count how many new vehicles have arrived.
        for intersection in self.intersections:
            tl = intersection.tl_id
            
            # Extract all outgoing lanes from connections controlled by this traffic light
            outgoing_lanes = set()
            for conn in intersection.parser_connections:
                to_lane = f"{conn['to']}_{conn['toLane']}"
                outgoing_lanes.add(to_lane)
                
            current_vehs = set()
            for lane in outgoing_lanes:
                try:
                    vehs = traci.lane.getLastStepVehicleIDs(lane)
                    current_vehs.update(vehs)
                except traci.TraCIException:
                    pass
            
            prev_vehs = self.previous_vehicle_ids.get(tl, set())
            new_vehs = current_vehs - prev_vehs
            self.throughput_counts[tl] += len(new_vehs)
            self.previous_vehicle_ids[tl] = current_vehs

    def get_metrics(self, intersection):
        tl = intersection.tl_id
        
        # Collect all incoming lanes
        incoming_lanes = []
        for lanes in intersection.links.values():
            incoming_lanes.extend(lanes)
        incoming_lanes = list(set(incoming_lanes))
        
        total_queue = 0
        total_waiting_time = 0.0
        total_vehicles = 0
        total_length = 0.0
        weighted_speed = 0.0
        weighted_limit = 0.0
        
        for lane in incoming_lanes:
            try:
                # 1. Queue Length (halting vehicles)
                total_queue += traci.lane.getLastStepHaltingNumber(lane)
                
                # 2. Waiting Time
                total_waiting_time += traci.lane.getWaitingTime(lane)
                
                # For density & speed calculation
                veh_num = traci.lane.getLastStepVehicleNumber(lane)
                total_vehicles += veh_num
                
                lane_len = max(1.0, SimulationTools.get_lane_length_preloaded(lane) or traci.lane.getLength(lane))
                total_length += lane_len
                
                if veh_num > 0:
                    weighted_speed += traci.lane.getLastStepMeanSpeed(lane) * veh_num
                    weighted_limit += traci.lane.getMaxSpeed(lane) * veh_num
            except traci.TraCIException:
                pass
                
        # 3. Density (vehicles per meter, normalized)
        density = total_vehicles / max(1.0, total_length)
        
        # 4. Delay (relative speed loss)
        if total_vehicles > 0 and weighted_limit > 0:
            avg_speed = weighted_speed / total_vehicles
            avg_limit = weighted_limit / total_vehicles
            delay = max(0.0, 1.0 - (avg_speed / max(1.0, avg_limit)))
        else:
            delay = 0.0
            
        # 5. Throughput
        throughput = self.throughput_counts.get(tl, 0)
        
        return {
            "queue_length": total_queue,
            "waiting_time": total_waiting_time,
            "density": density,
            "delay": delay,
            "throughput": throughput
        }
        
    def reset_throughput(self):
        for tl in self.throughput_counts:
            self.throughput_counts[tl] = 0
