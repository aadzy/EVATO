import math
import copy

class BangaloreSCOSCAOptimizer:
    def __init__(self, params):
        self.params = params

    def optimize_cycle_lengths(self, districts, district_tls, degree_of_sat, current_cycle_lengths):
        # We optimize the cycle length for each district independently
        new_cycle_lengths = {}
        for district_name, tls in districts.items():
            current_cycle = current_cycle_lengths.get(district_name, 120)
            
            # Find the maximum degree of saturation in the district
            max_ds_in_district = 0.0
            for tl_id in tls:
                if tl_id in degree_of_sat and degree_of_sat[tl_id]:
                    max_ds = max(degree_of_sat[tl_id].values())
                    if max_ds > max_ds_in_district:
                        max_ds_in_district = max_ds
                        
            if max_ds_in_district >= self.params["ds_upper_val"]:
                new_length = current_cycle + (max_ds_in_district - self.params["ds_upper_val"]) * self.params["adaptation_cycle"]
                new_length = int(math.ceil(new_length))
                new_cycle_lengths[district_name] = min(new_length, self.params["max_cycle_length"])
            elif 0 < max_ds_in_district < self.params["ds_lower_val"]:
                new_length = current_cycle - (self.params["ds_lower_val"] - max_ds_in_district) * self.params["adaptation_cycle"]
                new_length = int(math.floor(new_length))
                new_cycle_lengths[district_name] = max(new_length, self.params["min_cycle_length"])
            else:
                new_cycle_lengths[district_name] = current_cycle
                
        return new_cycle_lengths

    def optimize_green_splits(self, intersections, queue_lengths, degree_of_sat, current_greentimes, cycle_lengths, previous_effective_cycles, district_mappings):
        # Optimize green splits for each intersection
        new_greentimes = {}
        new_previous_effective_cycles = {}
        
        for intersection in intersections:
            tl_id = intersection.tl_id
            greens = list(current_greentimes.get(tl_id, []))
            if not greens:
                continue
                
            district_name = district_mappings.get(tl_id)
            cycle_length = cycle_lengths.get(district_name, 120)
            effective_cycle = cycle_length - 3 * len(greens)  # Subtract yellow phases
            
            # Get the lane with the highest degree of saturation at this intersection
            intersection_ds = degree_of_sat.get(tl_id, {})
            intersection_queues = queue_lengths.get(tl_id, {})
            
            if not intersection_ds or not intersection_queues:
                # No data: keep current splits scaled
                prev_eff = previous_effective_cycles.get(tl_id, effective_cycle)
                scaled = [int(g * effective_cycle / max(1, prev_eff)) for g in greens]
                diff = effective_cycle - sum(scaled)
                if scaled:
                    scaled[0] += diff
                new_greentimes[tl_id] = [max(5, s) for s in scaled]
                new_previous_effective_cycles[tl_id] = effective_cycle
                continue
                
            max_lane = max(intersection_ds, key=intersection_ds.get)
            max_queue = intersection_queues.get(max_lane, 0)
            max_ds = intersection_ds.get(max_lane, 0.0)
            
            if max_queue > self.params["green_thresh"]:
                # Identify the phase containing max_lane
                # links maps phase_index -> list(lanes)
                phases_with_max_lane = [
                    idx for idx, p_idx in enumerate(intersection.phases)
                    if max_lane in intersection.links.get(p_idx, [])
                ]
                max_phase_idx = phases_with_max_lane[0] if phases_with_max_lane else 0
                
                # Collect all lanes in the max_phase (to exclude them from other phases)
                excluded_lanes = set(intersection.links.get(intersection.phases[max_phase_idx], []))
                
                # Get DS for all other phases (excluding max_lane and shared lanes)
                phase_ds_list = []
                for idx in range(len(greens)):
                    if idx == max_phase_idx:
                        continue
                    p_idx = intersection.phases[idx]
                    lanes = intersection.links.get(p_idx, [])
                    phase_lanes = [l for l in lanes if l not in excluded_lanes]
                    if phase_lanes:
                        ds_val = max([intersection_ds.get(l, 0.0) for l in phase_lanes])
                    else:
                        ds_val = 0.0
                    phase_ds_list.append((idx, ds_val))
                    
                # Sort remaining phases by degree of saturation in descending order
                phase_ds_sorted = sorted(phase_ds_list, key=lambda x: x[1], reverse=True)
                
                # 1st optimization: allocate more time to the max_phase
                lowest_ds = phase_ds_sorted[-1][1] if phase_ds_sorted else 0.0
                ds_diff_max = max(0.0, max_ds - lowest_ds)
                
                # New green time for max_phase
                new_max_green = int(min(
                    3 * effective_cycle / 4,
                    greens[max_phase_idx] + ds_diff_max * self.params["adaptation_green"]
                ))
                new_max_green = max(5, new_max_green)
                
                # Allocate remaining time to other phases
                updated_greens = [0] * len(greens)
                updated_greens[max_phase_idx] = new_max_green
                remaining_time = effective_cycle - new_max_green
                
                if len(greens) == 2:
                    other_phase_idx = phase_ds_sorted[0][0]
                    updated_greens[other_phase_idx] = max(5, remaining_time)
                elif len(greens) >= 3:
                    # Allocate recursively to remaining sorted phases
                    for i in range(len(phase_ds_sorted) - 1):
                        curr_phase_idx, curr_ds = phase_ds_sorted[i]
                        # Compute difference relative to the lowest DS phase
                        ds_diff = max(0.0, curr_ds - lowest_ds)
                        allocated = int(min(
                            2 * remaining_time / 3,
                            greens[curr_phase_idx] + ds_diff * self.params["adaptation_green"]
                        ))
                        allocated = max(5, allocated)
                        updated_greens[curr_phase_idx] = allocated
                        remaining_time -= allocated
                    # Give final remainder to the lowest DS phase
                    last_phase_idx = phase_ds_sorted[-1][0]
                    updated_greens[last_phase_idx] = max(5, remaining_time)
                else:
                    updated_greens[0] = effective_cycle
                    
                new_greentimes[tl_id] = updated_greens
            else:
                # No critical queues: scale splits based on cycle change
                prev_eff = previous_effective_cycles.get(tl_id, effective_cycle)
                scaled = [
                    int(g * effective_cycle / max(1, prev_eff))
                    for g in greens
                ]
                # Normalize to match effective_cycle exactly
                diff = effective_cycle - sum(scaled)
                if scaled:
                    scaled[0] += diff
                new_greentimes[tl_id] = [max(5, s) for s in scaled]
                
            new_previous_effective_cycles[tl_id] = effective_cycle
            
        return new_greentimes, new_previous_effective_cycles

    def optimize_offsets(self, districts, critical_district_order, queue_lengths, lane_lengths, estimated_travel_times, cycle_lengths, current_offsets):
        new_offsets = {}
        for district_name, tls in districts.items():
            cycle_length = cycle_lengths.get(district_name, 120)
            
            # Simple district ordering based on critical order
            ordered_tls = critical_district_order.get(district_name, tls)
            
            # Set offset for first intersection to 0
            new_offsets[ordered_tls[0]] = 0
            
            # For each subsequent intersection in the corridor, offset is travel time from predecessor
            for idx in range(1, len(ordered_tls)):
                prev_tl = ordered_tls[idx - 1]
                curr_tl = ordered_tls[idx]
                
                # Get the travel time between prev_tl and curr_tl
                travel_time = estimated_travel_times.get(prev_tl, {}).get(curr_tl, 0.0)
                
                # Apply offset adaptation formula
                prev_offset = new_offsets[prev_tl]
                new_offset = min(
                    prev_offset + travel_time * self.params["adaptation_offset"],
                    cycle_length
                )
                new_offsets[curr_tl] = int(new_offset)
                
        return new_offsets
