import copy
import traci
from sumoITScontrol.simulation_tools import SimulationTools
from .intersection import BangaloreIntersection
from .metrics import BangaloreMetrics
from .optimizer import BangaloreSCOSCAOptimizer
from .coordinator import BangaloreCorridorCoordinator

class BangaloreSCOSCA:
    def __init__(self, params, parser, graph_builder, initial_cycle_length=120):
        self.params = params
        self.parser = parser
        self.graph_builder = graph_builder
        
        # Build all intersections automatically
        self.intersections = []
        tls_data = parser.get_tls()
        for tl_id in tls_data.keys():
            intersection_conns = parser.get_tl_connections(tl_id)
            # Create instance
            int_obj = BangaloreIntersection(tl_id, tls_data[tl_id], intersection_conns)
            self.intersections.append(int_obj)
            
        # Init components
        self.metrics_collector = BangaloreMetrics(self.intersections)
        self.optimizer = BangaloreSCOSCAOptimizer(params)
        
        self.districts = graph_builder.get_districts()
        self.critical_district_order = graph_builder.get_critical_district_order()
        self.connection_between_intersections = graph_builder.get_connection_between_intersections()
        self.coordinator = BangaloreCorridorCoordinator(self.districts, graph_builder)
        
        # Setup mapping of intersection ID to district name
        self.intersection_to_district = {}
        for district_name, tls in self.districts.items():
            for tl_id in tls:
                self.intersection_to_district[tl_id] = district_name
                
        # Initialize measurement data structures
        self.measurement_data = {
            "measurement_counter": -1,
            "control_counter": 0,
            "update_counter": {d: 0 for d in self.districts},
            "last_cycle_update": {d: 0 for d in self.districts},
            "cycle_lengths": {d: initial_cycle_length for d in self.districts},
            "previous_effective_cycles": {
                i.tl_id: initial_cycle_length - 3 * len(i.phases) for i in self.intersections
            },
            "greentimes": {
                i.tl_id: [30] * len(i.phases) for i in self.intersections
            },
            "offsets": {
                i.tl_id: 0 for i in self.intersections
            },
            "history_cycle_lengths": [],
            "history_greentimes": [],
            "history_offsets": []
        }
        
        # Store lane lengths and speed limits
        self.lane_lengths = {}
        self.speed_limit = graph_builder.speed_limit
        
        # Flag for future EVATO override layer
        self.evato_override_enabled = False

    def init_simulation(self):
        # Preload lane lengths from SUMO via TraCI
        edge_ids = traci.edge.getIDList()
        for edge_id in edge_ids:
            try:
                num_lanes = traci.edge.getLaneNumber(edge_id)
                for lane_idx in range(num_lanes):
                    lane_id = f"{edge_id}_{lane_idx}"
                    SimulationTools.get_lane_length(traci, lane_id)
                    self.lane_lengths[lane_id] = SimulationTools.get_lane_length_preloaded(lane_id)
            except traci.TraCIException:
                pass

    def evato_override_active(self):
        # Returns True if an emergency vehicle is detected in the network
        if not self.evato_override_enabled:
            return False
        # Hook for EVATO priority check (e.g. check if emergency vehicle is present in TraCI)
        try:
            veh_ids = traci.vehicle.getIDList()
            for v_id in veh_ids:
                if traci.vehicle.getTypeID(v_id) == "emergency":
                    return True
        except traci.TraCIException:
            pass
        return False

    def execute_evato_override(self, current_time):
        # EVATO emergency override logic (to be extended in next phase)
        print(f"[{current_time}] EVATO Override active: prioritizing emergency corridors!")
        # For now, if override is active, we trigger standard green corridors or hold green signals
        pass

    def apply_signal_plans(self, greentimes, offsets):
        # Apply program logic and offsets to each traffic light
        for intersection in self.intersections:
            tl_id = intersection.tl_id
            greens = greentimes.get(tl_id, [])
            if not greens:
                continue
                
            # Apply phases/green program
            intersection.apply_tl_programme(greens, yellow_duration=3)
            
            # Apply offset shift
            shift = int(offsets.get(tl_id, 0))
            if shift == 0:
                traci.trafficlight.setPhase(tl_id, 0)
            elif shift < 3:
                traci.trafficlight.setPhase(tl_id, len(greens) * 2 - 1)
                traci.trafficlight.setPhaseDuration(tl_id, shift)
            elif shift < greens[-1] + 3:
                traci.trafficlight.setPhase(tl_id, len(greens) * 2 - 2)
                traci.trafficlight.setPhaseDuration(tl_id, shift - 3)
            elif shift < greens[-1] + 6:
                traci.trafficlight.setPhase(tl_id, len(greens) * 2 - 3)
                traci.trafficlight.setPhaseDuration(tl_id, shift - greens[-1] - 3)
            elif shift < greens[-1] + 6 + (greens[-2] if len(greens) > 1 else 0):
                traci.trafficlight.setPhase(tl_id, len(greens) * 2 - 4)
                traci.trafficlight.setPhaseDuration(tl_id, shift - greens[-1] - 6)
            else:
                if len(greens) * 2 > 4:
                    if shift < greens[-1] + 9 + (greens[-2] if len(greens) > 1 else 0):
                        traci.trafficlight.setPhase(tl_id, len(greens) * 2 - 5)
                        traci.trafficlight.setPhaseDuration(
                            tl_id,
                            shift - greens[-1] - (greens[-2] if len(greens) > 1 else 0) - 6,
                        )
                    elif shift < greens[-1] + 9 + (greens[-2] if len(greens) > 1 else 0) + (greens[-3] if len(greens) > 2 else 0):
                        traci.trafficlight.setPhase(tl_id, len(greens) * 2 - 6)
                        traci.trafficlight.setPhaseDuration(
                            tl_id,
                            shift - greens[-1] - (greens[-2] if len(greens) > 1 else 0) - 9,
                        )
                    else:
                        pass
                else:
                    pass

    def execute_control(self, current_time):
        # 1. EVATO Override Check
        if self.evato_override_active():
            self.execute_evato_override(current_time)
            return

        # Update throughput counts every step
        self.metrics_collector.update_throughput()

        self.measurement_data["measurement_counter"] += 1
        if self.measurement_data["measurement_counter"] == self.params["measurement_period"]:
            self.measurement_data["measurement_counter"] = 0
            self.measurement_data["control_counter"] += 1
            
            # Check which districts require update
            for district_name, tls in self.districts.items():
                cycle_len = self.measurement_data["cycle_lengths"][district_name]
                last_update = self.measurement_data["last_cycle_update"][district_name]
                
                if self.measurement_data["control_counter"] >= last_update + cycle_len:
                    self.measurement_data["last_cycle_update"][district_name] = self.measurement_data["control_counter"]
                    
                    # 2. Gather Traffic State Metrics
                    queue_lengths = {}
                    degree_of_sat = {}
                    
                    for tl_id in tls:
                        intersection = next(i for i in self.intersections if i.tl_id == tl_id)
                        queue_lengths[tl_id] = {}
                        degree_of_sat[tl_id] = {}
                        
                        # Fetch metrics using collector
                        metrics = self.metrics_collector.get_metrics(intersection)
                        
                        # Get list of lanes for the intersection links
                        lane_candidates = []
                        for lanes in intersection.links.values():
                            lane_candidates.extend(lanes)
                        lane_candidates = list(set(lane_candidates))
                        
                        for lane in lane_candidates:
                            try:
                                veh = traci.lane.getLastStepVehicleNumber(lane)
                                length = max(1.0, self.lane_lengths.get(lane, 10.0))
                            except traci.TraCIException:
                                veh = 0
                                length = 10.0
                            queue_lengths[tl_id][lane] = int(veh)
                            ds = min(1.0, float(veh) / max(1.0, (length / 7.0)))
                            degree_of_sat[tl_id][lane] = float(ds)

                    # 3. Optimize Cycle Length
                    update_cnt = self.measurement_data["update_counter"][district_name]
                    if update_cnt % 5 == 0 and update_cnt != 0:
                        new_cycles = self.optimizer.optimize_cycle_lengths(
                            {district_name: tls}, self.districts, degree_of_sat, self.measurement_data["cycle_lengths"]
                        )
                        self.measurement_data["cycle_lengths"][district_name] = new_cycles[district_name]
                        
                    # 4. Optimize Green Split Durations
                    if update_cnt != 0:
                        new_splits, new_effs = self.optimizer.optimize_green_splits(
                            [next(i for i in self.intersections if i.tl_id == tl_id) for tl_id in tls],
                            queue_lengths,
                            degree_of_sat,
                            self.measurement_data["greentimes"],
                            self.measurement_data["cycle_lengths"],
                            self.measurement_data["previous_effective_cycles"],
                            self.intersection_to_district
                        )
                        # Apply spillback prevention
                        adjusted_splits = self.coordinator.check_spillback_prevention(
                            queue_lengths, self.lane_lengths, new_splits, self.intersections
                        )
                        
                        # Update greentimes & previous effective cycles
                        for tl_id in tls:
                            self.measurement_data["greentimes"][tl_id] = adjusted_splits[tl_id]
                            self.measurement_data["previous_effective_cycles"][tl_id] = new_effs[tl_id]

                    # 5. Optimize Offsets
                    if update_cnt % 5 == 0 and update_cnt != 0:
                        # Extract travel times between neighbors
                        estimated_travel_times = {}
                        for tl_id in tls:
                            estimated_travel_times[tl_id] = self.graph_builder.graph[tl_id]["travel_time_to"]
                            
                        new_offsets = self.optimizer.optimize_offsets(
                            {district_name: tls},
                            self.critical_district_order,
                            queue_lengths,
                            self.lane_lengths,
                            estimated_travel_times,
                            self.measurement_data["cycle_lengths"],
                            self.measurement_data["offsets"]
                        )
                        for tl_id in tls:
                            self.measurement_data["offsets"][tl_id] = new_offsets[tl_id]
                            
                    # Update counts and apply signal plans
                    self.measurement_data["update_counter"][district_name] += 1
                    
                    # Apply changes to SUMO
                    self.apply_signal_plans(
                        self.measurement_data["greentimes"], self.measurement_data["offsets"]
                    )
                    
                    # Append history
                    self.measurement_data["history_greentimes"].append([current_time, copy.deepcopy(self.measurement_data["greentimes"])])
                    self.measurement_data["history_offsets"].append([current_time, copy.deepcopy(self.measurement_data["offsets"])])
                    self.measurement_data["history_cycle_lengths"].append([current_time, copy.deepcopy(self.measurement_data["cycle_lengths"])])
