import traci

class BangaloreCorridorCoordinator:
    def __init__(self, districts, graph_builder):
        self.districts = districts
        self.graph_builder = graph_builder
        self.adjacency_graph = graph_builder.get_graph()
        self.spillback_threshold = 0.85  # queue/length ratio to trigger spillback prevention

    def check_spillback_prevention(self, queue_lengths, lane_lengths, green_splits, intersections):
        # We adjust green splits to prevent upstream lanes from spilling back.
        # If the connecting lanes between tl_A (upstream) and tl_B (downstream) are heavily congested,
        # we cap the green time of tl_A's phase that feeds tl_B.
        adjusted_splits = copy_splits = {tl: list(splits) for tl, splits in green_splits.items()}
        
        # Build map of intersections by ID for quick lookup
        int_map = {i.tl_id: i for i in intersections}
        
        for district_name, tls in self.districts.items():
            for upstream_tl in tls:
                if upstream_tl not in copy_splits:
                    continue
                    
                upstream_int = int_map.get(upstream_tl)
                if not upstream_int:
                    continue
                
                # Check downstream neighbors in the same district
                for downstream_tl in self.adjacency_graph.get(upstream_tl, {}).get("neighbors", []):
                    if downstream_tl not in tls:
                        continue
                        
                    # Find connecting lanes between upstream_tl and downstream_tl
                    conn_lanes = self.graph_builder.graph.get(upstream_tl, {}).get("connecting_lanes_to", {}).get(downstream_tl, [])
                    if not conn_lanes:
                        continue
                        
                    # Check congestion on these connecting lanes
                    spillback_detected = False
                    for lane in conn_lanes:
                        q_len = queue_lengths.get(downstream_tl, {}).get(lane, 0)
                        # Estimate capacity as lane_length / 7m (approx vehicle length + gap)
                        lane_len = lane_lengths.get(lane, 100.0)
                        capacity = max(1.0, lane_len / 7.0)
                        
                        if (q_len / capacity) >= self.spillback_threshold:
                            spillback_detected = True
                            break
                            
                    if spillback_detected:
                        # Find the phase index at upstream_tl that feeds downstream_tl
                        # If a connection feeds the connecting lane, identify its phase
                        for idx, p_idx in enumerate(upstream_int.phases):
                            active_lanes = upstream_int.links.get(p_idx, [])
                            # If this phase feeds downstream, we cap its green time
                            # In SUMO, the phase feeds downstream if its outgoing lanes include the connecting lanes
                            # Or if we can find if any incoming lane in this phase maps to downstream_tl
                            # Let's cap the phase if its links are feeding the congested connecting lanes.
                            feeds_congested_lane = any(l in conn_lanes for l in active_lanes)
                            if feeds_congested_lane:
                                # Cap this phase's green split at 5 seconds (spillback prevention)
                                # and redistribute the capped time to other phases
                                old_green = adjusted_splits[upstream_tl][idx]
                                if old_green > 5:
                                    capped_green = 5
                                    diff = old_green - capped_green
                                    adjusted_splits[upstream_tl][idx] = capped_green
                                    
                                    # Redistribute diff to other phases
                                    other_phases = [i for i in range(len(adjusted_splits[upstream_tl])) if i != idx]
                                    if other_phases:
                                        share = diff // len(other_phases)
                                        for o_idx in other_phases:
                                            adjusted_splits[upstream_tl][o_idx] += share
                                        # Adjust remainder
                                        adjusted_splits[upstream_tl][other_phases[0]] += (diff % len(other_phases))
                                        
        return adjusted_splits

    def predict_platoon_arrivals(self, queue_lengths):
        # Estimates expected vehicle arrivals from upstream neighbors
        platoons = {}
        for tl_id, data in self.adjacency_graph.items():
            platoons[tl_id] = 0
            for neighbor in data.get("neighbors", []):
                # If neighbor has significant queues and is upstream, we expect a platoon arrival
                # We can check neighbor's queue heading towards tl_id
                tt = data["travel_time_to"].get(neighbor, 10.0)
                n_queues = queue_lengths.get(neighbor, {})
                # Total queue at neighbor
                tot_q = sum(n_queues.values()) if n_queues else 0
                if tot_q > 5:
                    # Expect platoon arrival proportional to queue size and travel time
                    platoons[tl_id] += int(tot_q * 0.5)
        return platoons
