import traci
from sumoITScontrol.simulation_tools import SimulationTools

class BangaloreIntersection:
    def __init__(self, tl_id, parser_tl_data, parser_connections):
        self.tl_id = tl_id
        self.parser_tl_data = parser_tl_data
        self.parser_connections = parser_connections
        
        self.phases = []
        self.links = {}
        self.green_states = []
        self.yellow_states = []
        self.pressure_source = "lanes"
        
        self._initialize_from_parser()

    def _initialize_from_parser(self):
        # parser_tl_data is a dict with {"type", "programID", "offset", "phases": [{"duration", "state"}]}
        all_phases = self.parser_tl_data.get("phases", [])
        
        # Group connections by linkIndex
        # Note: multiple connections can have the same linkIndex if they share the signal head
        link_to_lanes = {}
        for conn in self.parser_connections:
            link_idx = conn.get("linkIndex")
            if link_idx is not None:
                from_lane = f"{conn['from']}_{conn['fromLane']}"
                if link_idx not in link_to_lanes:
                    link_to_lanes[link_idx] = []
                if from_lane not in link_to_lanes[link_idx]:
                    link_to_lanes[link_idx].append(from_lane)

        # Iterate through phases to find the green phases
        for idx, phase in enumerate(all_phases):
            state = phase["state"]
            # A green phase has 'G' or 'g', and does not have 'y' or 'Y'
            is_green = ("G" in state or "g" in state) and not ("y" in state or "Y" in state)
            
            # If the traffic light is extremely simple (e.g. all-red or single phase), we must be careful.
            # But normally, green phases are those with G/g and no y.
            if is_green:
                self.phases.append(idx)
                self.green_states.append(state)
                
                # Determine active lanes for this green phase
                active_lanes = []
                for link_idx, lanes in link_to_lanes.items():
                    if link_idx < len(state) and state[link_idx] in ("G", "g"):
                        active_lanes.extend(lanes)
                self.links[idx] = list(set(active_lanes))
                
                # Determine corresponding yellow state
                # Look at the next phase. If it's a yellow phase, use its state
                next_idx = (idx + 1) % len(all_phases)
                next_state = all_phases[next_idx]["state"]
                if "y" in next_state or "Y" in next_state:
                    self.yellow_states.append(next_state)
                else:
                    # Construct fallback yellow state by replacing all G/g with y
                    fallback_yellow = "".join(["y" if c in ("G", "g") else c for c in state])
                    self.yellow_states.append(fallback_yellow)

        # Fallback: if no green phases were identified (e.g. static/actuated warning or unusual network setup)
        if not self.phases:
            for idx, phase in enumerate(all_phases):
                self.phases.append(idx)
                self.green_states.append(phase["state"])
                self.yellow_states.append("".join(["y" if c in ("G", "g") else c for c in phase["state"]]))
                
                active_lanes = []
                for link_idx, lanes in link_to_lanes.items():
                    active_lanes.extend(lanes)
                self.links[idx] = list(set(active_lanes))

    def set_signal_on_traffic_lights(self, phase):
        traci.trafficlight.setPhase(self.tl_id, phase)

    def get_queue_lengths_num_vehicles(self):
        # Computes number of vehicles queued in lanes feeding this intersection
        n_vehicles = {}
        pressures = []
        for phase in self.phases:
            pressure = 0
            lanes = self.links.get(phase, [])
            for lane in lanes:
                if lane not in n_vehicles:
                    try:
                        n_vehicles[lane] = traci.lane.getLastStepVehicleNumber(lane)
                    except traci.TraCIException:
                        n_vehicles[lane] = 0
            
            # Count "hidden" vehicles on internal intersection lanes
            SimulationTools.determine_hidden_vehicles(traci)
            edges = [l.split("_")[0] for l in lanes]
            hidden_vehicles_current_edge = [
                element
                for element in SimulationTools.hidden_vehicles_current_edge
                if element in edges
            ]
            
            for lane in lanes:
                pressure += n_vehicles[lane]
            pressure += len(hidden_vehicles_current_edge)
            pressures.append(pressure)
        return pressures

    def apply_tl_programme(self, greens, yellow_duration):
        phases = []
        for idx, g in enumerate(greens):
            gstate = (
                self.green_states[idx]
                if idx < len(self.green_states)
                else "G" * max(1, len(self.phases))
            )
            ystate = (
                self.yellow_states[idx]
                if idx < len(self.yellow_states)
                else "y" * len(gstate)
            )
            phases.append(traci.trafficlight.Phase(int(g), gstate))
            phases.append(traci.trafficlight.Phase(yellow_duration, ystate))
            
        logic = traci.trafficlight.Logic(
            programID=f"program_fixed_{self.tl_id}",
            type=traci.tc.TRAFFICLIGHT_TYPE_STATIC,
            currentPhaseIndex=0,
            phases=phases,
        )
        traci.trafficlight.setProgramLogic(self.tl_id, logic)
