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
        # Native phase(s) between this green's yellow and the next green that
        # aren't part of the adaptive split at all — typically a fixed
        # all-red pedestrian-clearance interval on a single-movement, minor
        # (e.g. 2-lane) approach. Parallel to phases/green_states/yellow_states;
        # each entry is a list of (duration, state) tuples, usually empty.
        self.clearance_phases = []
        self.pressure_source = "lanes"

        self._initialize_from_parser()

    def _initialize_from_parser(self):
        # parser_tl_data is a dict with {"type", "programID", "offset", "phases": [{"duration", "state"}]}
        all_phases = self.parser_tl_data.get("phases", [])
        n_all = len(all_phases)

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

        def is_green_state(state):
            # A green phase has 'G' or 'g', and does not have 'y' or 'Y'
            return ("G" in state or "g" in state) and not ("y" in state or "Y" in state)

        green_indices = [idx for idx, phase in enumerate(all_phases) if is_green_state(phase["state"])]

        for pos, idx in enumerate(green_indices):
            state = all_phases[idx]["state"]
            self.phases.append(idx)
            self.green_states.append(state)

            # Determine active lanes for this green phase
            active_lanes = []
            for link_idx, lanes in link_to_lanes.items():
                if link_idx < len(state) and state[link_idx] in ("G", "g"):
                    active_lanes.extend(lanes)
            self.links[idx] = list(set(active_lanes))

            # Walk every native phase between this green and the next green
            # (wrapping past the end of the cycle for the last one), in order,
            # so we know the FULL set of phases the original network defines
            # here instead of only the immediate next one.
            next_green_idx = green_indices[(pos + 1) % len(green_indices)] if len(green_indices) > 1 else idx
            between = []
            j = (idx + 1) % n_all
            while j != next_green_idx and len(between) <= n_all:
                between.append((all_phases[j]["duration"], all_phases[j]["state"]))
                j = (j + 1) % n_all

            if between and ("y" in between[0][1] or "Y" in between[0][1]):
                self.yellow_states.append(between[0][1])
                self.clearance_phases.append(between[1:])
            else:
                # Construct fallback yellow state by replacing all G/g with y
                fallback_yellow = "".join(["y" if c in ("G", "g") else c for c in state])
                self.yellow_states.append(fallback_yellow)
                self.clearance_phases.append(between)

        # Fallback: if no green phases were identified (e.g. static/actuated warning or unusual network setup)
        if not self.phases:
            for idx, phase in enumerate(all_phases):
                self.phases.append(idx)
                self.green_states.append(phase["state"])
                self.yellow_states.append("".join(["y" if c in ("G", "g") else c for c in phase["state"]]))
                self.clearance_phases.append([])

                active_lanes = []
                for link_idx, lanes in link_to_lanes.items():
                    active_lanes.extend(lanes)
                self.links[idx] = list(set(active_lanes))

    def program_index_for_green(self, green_pos):
        # SUMO program-phase index where the green at position `green_pos`
        # (in self.phases/green_states order) starts, given the actual
        # variable-length [green, yellow, *clearance] blocks apply_tl_programme
        # builds per green — needed anywhere code sets a phase index directly
        # (offset shifting, emergency preemption).
        idx = 0
        for i in range(green_pos):
            n_clearance = len(self.clearance_phases[i]) if i < len(self.clearance_phases) else 0
            idx += 2 + n_clearance
        return idx

    def green_program_indices(self):
        return {self.program_index_for_green(pos) for pos in range(len(self.phases))}

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

    def apply_tl_programme(self, greens, yellow_duration, extension=0, gap_thresh=None, min_green_ratio=1.0, min_green_floor=5):
        # CoSiCoSt's local intersection controller keeps autonomy to both
        # extend a phase (if vehicles are still present near the end of
        # green) AND terminate it early past a "mandatory minimum green"
        # once demand is served, handing the freed time to the next stage
        # ("Online Split Optimizer" — the papers describe both halves, not
        # just extension). We realize this as a SUMO actuated program with
        # minDur = mandatory minimum green (a fraction of the computed
        # split, floored at min_green_floor) and maxDur = computed split +
        # extension, instead of a rigid static program, whenever
        # extension > 0 or min_green_ratio < 1.0.
        phases = []
        is_actuated = extension > 0 or min_green_ratio < 1.0
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
            green_dur = int(g)
            if is_actuated:
                min_dur = max(min_green_floor, int(green_dur * min_green_ratio))
                min_dur = min(min_dur, green_dur)
                max_dur = green_dur + max(0, int(extension))
            else:
                min_dur = green_dur
                max_dur = green_dur
            phases.append(
                traci.trafficlight.Phase(green_dur, gstate, minDur=min_dur, maxDur=max_dur)
            )
            phases.append(
                traci.trafficlight.Phase(yellow_duration, ystate, minDur=yellow_duration, maxDur=yellow_duration)
            )
            # Preserve any native clearance phase(s) between this green and
            # the next (e.g. an all-red pedestrian-crossing interval) that
            # aren't part of the adaptive split — dropping them was
            # collapsing single-movement, minor (e.g. 2-lane) approaches down
            # to a bare green/yellow loop that never actually reached red.
            # These are fixed real-world intervals, so keep their native
            # duration rather than scaling them with the cycle.
            clearance = self.clearance_phases[idx] if idx < len(self.clearance_phases) else []
            for c_dur, c_state in clearance:
                c_dur_int = max(1, int(c_dur))
                phases.append(
                    traci.trafficlight.Phase(c_dur_int, c_state, minDur=c_dur_int, maxDur=c_dur_int)
                )

        program_type = (
            traci.tc.TRAFFICLIGHT_TYPE_ACTUATED if is_actuated else traci.tc.TRAFFICLIGHT_TYPE_STATIC
        )
        sub_parameter = {"max-gap": str(gap_thresh)} if (program_type == traci.tc.TRAFFICLIGHT_TYPE_ACTUATED and gap_thresh is not None) else {}
        logic = traci.trafficlight.Logic(
            programID=f"program_fixed_{self.tl_id}",
            type=program_type,
            currentPhaseIndex=0,
            phases=phases,
            subParameter=sub_parameter,
        )
        traci.trafficlight.setProgramLogic(self.tl_id, logic)

    def apply_actuated_fallback_programme(self, min_green, max_green, yellow_duration, gap_thresh=None):
        # CoSiCoSt step 10: fallback mode is "Full Vehicle Actuation" when the
        # central data/comms link is unavailable. Each intersection reverts to
        # a locally-actuated program (min/max green per phase) so it keeps
        # cycling sensibly without needing input from the district optimizer.
        phases = []
        for idx in range(len(self.phases)):
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
            phases.append(
                traci.trafficlight.Phase(min_green, gstate, minDur=min_green, maxDur=max_green)
            )
            phases.append(
                traci.trafficlight.Phase(yellow_duration, ystate, minDur=yellow_duration, maxDur=yellow_duration)
            )
            clearance = self.clearance_phases[idx] if idx < len(self.clearance_phases) else []
            for c_dur, c_state in clearance:
                c_dur_int = max(1, int(c_dur))
                phases.append(
                    traci.trafficlight.Phase(c_dur_int, c_state, minDur=c_dur_int, maxDur=c_dur_int)
                )

        sub_parameter = {"max-gap": str(gap_thresh)} if gap_thresh is not None else {}
        logic = traci.trafficlight.Logic(
            programID=f"program_fallback_{self.tl_id}",
            type=traci.tc.TRAFFICLIGHT_TYPE_ACTUATED,
            currentPhaseIndex=0,
            phases=phases,
            subParameter=sub_parameter,
        )
        traci.trafficlight.setProgramLogic(self.tl_id, logic)
