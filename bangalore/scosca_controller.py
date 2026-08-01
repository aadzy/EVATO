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
            "district_failure_streak": {d: 0 for d in self.districts},
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
            "history_offsets": [],
            "history_priority_direction": [],
            "history_fallback_events": [],
            "history_preemption_events": []
        }

        # Store lane lengths and speed limits
        self.lane_lengths = {}
        self.speed_limit = graph_builder.speed_limit

        # Tracks (greentimes, offset) last actually pushed to each traffic
        # light, so apply_signal_plans can skip reapplying an unchanged plan
        # instead of forcing a mid-cycle reset every recompute.
        self._last_applied_plan = {}
        # tl_ids currently running the Full Vehicle Actuation fallback program.
        self._fallback_active = set()

        # EVATO emergency-vehicle green-corridor preemption.
        self.evato_override_enabled = False
        self.edge_to_tl = graph_builder.get_edge_to_tl()
        self._intersection_by_id = {i.tl_id: i for i in self.intersections}
        self._preempted_tls = set()
        self._preemption_state = {}

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

    def _preempt_intersection(self, tl_id, edge_id, current_time):
        # Forces (or keeps forcing) the green phase serving `edge_id` at
        # `tl_id`, for as long as an emergency vehicle is still approaching
        # it. Uses the same setPhase/setPhaseDuration mechanism as the
        # normal offset-shift logic (apply_signal_plans), not a raw state
        # override, so it interacts predictably with the underlying
        # STATIC/ACTUATED program SCOSCA already installed.
        intersection = self._intersection_by_id.get(tl_id)
        if intersection is None:
            return

        target_phase_idx = None
        for idx, p_idx in enumerate(intersection.phases):
            lanes = intersection.links.get(p_idx, [])
            if any(lane.startswith(edge_id + "_") for lane in lanes):
                target_phase_idx = idx
                break
        if target_phase_idx is None:
            # Can't identify which phase serves this edge (e.g. a minor/
            # uncontrolled approach) — leave the signal to SCOSCA.
            return

        now = current_time / 1000.0
        # Program-phase index where this green actually starts — NOT simply
        # target_phase_idx * 2, since some intersections (single-movement,
        # minor/2-lane approaches with a native all-red pedestrian-clearance
        # phase) have a variable-length [green, yellow, *clearance] block per
        # green rather than a fixed green/yellow pair.
        target_program_idx = intersection.program_index_for_green(target_phase_idx)
        transition_sec = self.params.get("preemption_transition_yellow_sec", 3)
        hold_sec = self.params.get("preemption_hold_refresh_sec", 5)

        state = self._preemption_state.get(tl_id)
        if tl_id not in self._preempted_tls:
            self.measurement_data["history_preemption_events"].append([current_time, tl_id, "engaged"])
            try:
                current_program_idx = traci.trafficlight.getPhase(tl_id)
            except traci.TraCIException:
                current_program_idx = target_program_idx
            if current_program_idx == target_program_idx:
                state = {"phase": "hold"}
            elif current_program_idx in intersection.green_program_indices():
                # Currently showing a DIFFERENT green: give it a mandatory
                # yellow before jumping to the ambulance's green (safety) —
                # never switch green-to-green with no transition.
                traci.trafficlight.setPhase(tl_id, current_program_idx + 1)
                traci.trafficlight.setPhaseDuration(tl_id, transition_sec)
                state = {"phase": "transition", "until": now + transition_sec}
            else:
                # Already mid-yellow/clearance: let it finish naturally, then hold.
                state = {"phase": "transition", "until": now}

        if state.get("phase") == "transition" and now >= state.get("until", 0):
            state = {"phase": "hold"}

        if state.get("phase") == "hold":
            # Refresh every step while the ambulance is still approaching —
            # this is the "no further traffic created" bound: the hold only
            # persists as long as it's genuinely still needed, and stops the
            # instant the vehicle clears this intersection (see
            # _update_emergency_preemption's release logic below).
            traci.trafficlight.setPhase(tl_id, target_program_idx)
            traci.trafficlight.setPhaseDuration(tl_id, hold_sec)

        self._preemption_state[tl_id] = state

    def _update_emergency_preemption(self, current_time):
        # Selective green-corridor preemption (CoSiCoSt's documented green
        # corridors, e.g. the HDBRTS organ-transport corridors): only the
        # traffic lights on an active emergency vehicle's immediate path are
        # overridden. Every other intersection keeps running normal SCOSCA
        # control undisturbed — this is not a network-wide freeze.
        if not self.evato_override_enabled:
            return

        try:
            emergency_ids = [v for v in traci.vehicle.getIDList() if traci.vehicle.getTypeID(v) == "emergency"]
        except traci.TraCIException:
            return

        lookahead = self.params.get("preemption_lookahead_edges", 2)
        active_tls_this_step = set()

        for veh_id in emergency_ids:
            try:
                route = traci.vehicle.getRoute(veh_id)
                route_idx = traci.vehicle.getRouteIndex(veh_id)
            except traci.TraCIException:
                continue
            if route_idx < 0:
                continue
            for edge_id in route[route_idx:route_idx + lookahead]:
                tl_id = self.edge_to_tl.get(edge_id)
                if tl_id is None:
                    continue
                active_tls_this_step.add(tl_id)
                self._preempt_intersection(tl_id, edge_id, current_time)

        # Release intersections no longer on any active emergency vehicle's
        # immediate path — SCOSCA resumes controlling them right away rather
        # than waiting for the next scheduled recompute.
        for tl_id in (self._preempted_tls - active_tls_this_step):
            self._preemption_state.pop(tl_id, None)
            self._last_applied_plan.pop(tl_id, None)
            self.measurement_data["history_preemption_events"].append([current_time, tl_id, "released"])

        self._preempted_tls = active_tls_this_step

    @staticmethod
    def _phase_for_shift(greens, yellow_duration, shift, clearance_phases=None):
        # General replacement for the old hand-unrolled branch chain, which
        # only handled offsets up to 3 phases back from the cycle end and
        # silently did nothing (a bare `pass`) for any intersection with 4+
        # green phases, leaving it stuck. `shift` is "seconds remaining until
        # this intersection's phase 0 (green start) begins" — found by
        # walking the phase sequence backward from the end of the cycle
        # until the bucket containing `shift` is found, for ANY phase count.
        # `clearance_phases` (parallel to greens) lists any native phases
        # apply_tl_programme inserts after the yellow for that green (e.g. an
        # all-red pedestrian interval) — must mirror what it actually builds
        # so the returned program-phase index stays valid.
        if clearance_phases is None:
            clearance_phases = [[] for _ in greens]
        buckets = []
        program_idx = 0
        for idx, g in enumerate(greens):
            buckets.append((program_idx, int(g)))
            program_idx += 1
            buckets.append((program_idx, yellow_duration))
            program_idx += 1
            for c_dur, _ in (clearance_phases[idx] if idx < len(clearance_phases) else []):
                buckets.append((program_idx, max(1, int(c_dur))))
                program_idx += 1
        total = sum(dur for _, dur in buckets)
        if total <= 0:
            return 0, 0
        shift = shift % total
        if shift == 0:
            return 0, 0
        suffix = 0
        for phase_idx, dur in reversed(buckets):
            if shift < suffix + dur:
                return phase_idx, shift - suffix
            suffix += dur
        return 0, 0

    def apply_signal_plans(self, greentimes, offsets):
        # Apply program logic and offsets to each traffic light. Skip
        # reapplying anything unchanged since the last call: unconditionally
        # rebuilding a light's program every recompute cycle forces SUMO to
        # reset it to the target phase mid-cycle even when nothing about its
        # plan actually changed, which needlessly truncates whatever phase
        # was live (observed: a phase cut short and its yellow skipped
        # entirely on every district recompute).
        for intersection in self.intersections:
            tl_id = intersection.tl_id
            if tl_id in self._preempted_tls:
                continue  # an emergency vehicle currently has this light; don't fight it
            greens = greentimes.get(tl_id, [])
            if not greens:
                continue

            shift = int(offsets.get(tl_id, 0))
            signature = (tuple(greens), shift)
            if self._last_applied_plan.get(tl_id) == signature:
                continue
            self._last_applied_plan[tl_id] = signature

            # Apply phases/green program. CoSiCoSt's local "Online Split
            # Optimizer": let each phase extend by a small bounded increment
            # if vehicles are still present at the stop-line near the end of
            # the computed green, OR terminate early past a mandatory
            # minimum once demand is served, instead of a rigid static
            # duration for the full computed split either way.
            intersection.apply_tl_programme(
                greens,
                yellow_duration=3,
                extension=self.params.get("actuation_extension_sec", 0),
                gap_thresh=self.params.get("actuation_gap_thresh"),
                min_green_ratio=self.params.get("actuation_min_green_ratio", 1.0),
                min_green_floor=self.params.get("actuation_min_green_floor", 5),
            )

            if len(greens) < 2:
                # Nothing meaningful to offset-shift with a single stage.
                traci.trafficlight.setPhase(tl_id, 0)
                continue

            phase_idx, remaining = self._phase_for_shift(greens, 3, shift, intersection.clearance_phases)
            traci.trafficlight.setPhase(tl_id, phase_idx)
            if remaining > 0:
                traci.trafficlight.setPhaseDuration(tl_id, remaining)

    def execute_control(self, current_time):
        # Emergency-vehicle green-corridor preemption runs every step, on
        # top of (not instead of) normal SCOSCA control below — only
        # intersections on an active emergency vehicle's immediate path are
        # overridden; apply_signal_plans (and the fallback path) skip any
        # tl_id currently held here so they don't fight over the same light.
        self._update_emergency_preemption(current_time)

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
                    delay_by_tl = {}
                    district_had_error = False

                    for tl_id in tls:
                        intersection = next(i for i in self.intersections if i.tl_id == tl_id)
                        queue_lengths[tl_id] = {}
                        degree_of_sat[tl_id] = {}

                        # Fetch metrics using collector. CoSiCoSt's priority
                        # route is a weighted combination of delay AND number
                        # of stops, not queue count alone, so keep the
                        # waiting-time signal for the offset optimizer below.
                        metrics = self.metrics_collector.get_metrics(intersection)
                        delay_by_tl[tl_id] = metrics["waiting_time"]

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
                                district_had_error = True
                            queue_lengths[tl_id][lane] = int(veh)
                            ds = min(1.0, float(veh) / max(1.0, (length / 7.0)))
                            degree_of_sat[tl_id][lane] = float(ds)

                    # CoSiCoSt step 10: fall back to local Full Vehicle Actuation
                    # for this district if its detector/comms data has been
                    # unavailable for several consecutive control cycles, instead
                    # of optimizing against stale/zeroed-out data.
                    if district_had_error:
                        self.measurement_data["district_failure_streak"][district_name] += 1
                    else:
                        self.measurement_data["district_failure_streak"][district_name] = 0

                    district_degraded = (
                        self.measurement_data["district_failure_streak"][district_name]
                        >= self.params.get("fallback_missing_cycles", 3)
                    )

                    if district_degraded:
                        for tl_id in tls:
                            if tl_id in self._preempted_tls:
                                continue  # emergency vehicle has this light; don't fight it
                            # Only push the fallback program once per outage,
                            # not on every recompute while still degraded —
                            # same reasoning as the plan-change check below:
                            # SUMO's actuated logic keeps running this program
                            # on its own once installed, so reapplying it
                            # unnecessarily would just reset it mid-cycle.
                            if tl_id in self._fallback_active:
                                continue
                            intersection = next(i for i in self.intersections if i.tl_id == tl_id)
                            intersection.apply_actuated_fallback_programme(
                                min_green=self.params.get("fallback_min_green", 10),
                                max_green=self.params.get("fallback_max_green", 45),
                                yellow_duration=3,
                                gap_thresh=self.params.get("actuation_gap_thresh"),
                            )
                            self._fallback_active.add(tl_id)
                        self.measurement_data["update_counter"][district_name] += 1
                        self.measurement_data["history_fallback_events"].append([current_time, district_name])
                        continue
                    else:
                        # Recovering from an outage: force the next
                        # apply_signal_plans call to actually reinstall the
                        # normal SCOSCA program (it would otherwise think the
                        # plan is unchanged and skip re-applying it, leaving
                        # the fallback program running indefinitely).
                        for tl_id in tls:
                            if tl_id in self._fallback_active:
                                self._fallback_active.discard(tl_id)
                                self._last_applied_plan.pop(tl_id, None)

                    # 3. Optimize Cycle Length
                    update_cnt = self.measurement_data["update_counter"][district_name]
                    if update_cnt % 5 == 0 and update_cnt != 0:
                        new_cycles = self.optimizer.optimize_cycle_lengths(
                            {district_name: tls}, self.districts, degree_of_sat, self.measurement_data["cycle_lengths"]
                        )
                        self.measurement_data["cycle_lengths"][district_name] = new_cycles[district_name]
                        
                    # 4. Optimize Green Split Durations
                    if update_cnt != 0:
                        # CoSiCoSt "adjust the priority stage": look up the lanes
                        # leading towards the current priority route's next hop so
                        # optimize_green_splits can bias that phase's allocation.
                        priority_lanes = {}
                        for tl_id in tls:
                            next_tl = self.optimizer.get_priority_next_tl(tl_id)
                            if next_tl:
                                priority_lanes[tl_id] = self.graph_builder.graph.get(tl_id, {}).get(
                                    "connecting_lanes_to", {}
                                ).get(next_tl, [])

                        new_splits, new_effs = self.optimizer.optimize_green_splits(
                            [next(i for i in self.intersections if i.tl_id == tl_id) for tl_id in tls],
                            queue_lengths,
                            degree_of_sat,
                            self.measurement_data["greentimes"],
                            self.measurement_data["cycle_lengths"],
                            self.measurement_data["previous_effective_cycles"],
                            self.intersection_to_district,
                            priority_lanes
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
                            self.measurement_data["offsets"],
                            delay_by_tl
                        )
                        for tl_id in tls:
                            self.measurement_data["offsets"][tl_id] = new_offsets[tl_id]

                        # Record the demand-responsive priority-route direction
                        # chosen this round (CoSiCoSt "establish a priority route").
                        self.measurement_data["history_priority_direction"].append(
                            [current_time, district_name, self.optimizer._priority_direction.get(district_name)]
                        )
                            
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
