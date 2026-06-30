from collections import defaultdict, deque
from .parser import BangaloreNetworkParser

class BangaloreGraphBuilder:
    def __init__(self, parser: BangaloreNetworkParser, speed_limit=3.66, max_neighbor_distance=1500.0):
        self.parser = parser
        self.speed_limit = speed_limit
        self.max_neighbor_distance = max_neighbor_distance
        
        self.graph = {}
        self.districts = {}
        self.critical_district_order = {}
        self.connection_between_intersections = {}
        
        self._build_graph()
        self._cluster_districts()

    def _build_graph(self):
        edges = self.parser.get_edges()
        connections = self.parser.get_connections()
        tls_data = self.parser.get_tls()
        
        # Build mappings
        # 1. edge_connections[from_edge] = list of (to_edge, connection_info)
        edge_connections = defaultdict(list)
        for conn in connections:
            edge_connections[conn["from"]].append((conn["to"], conn))
            
        # 2. edge_to_tl[edge_id] = tl_id (if this edge is controlled by a tl)
        edge_to_tl = {}
        tl_incoming_lanes = defaultdict(set)
        tl_outgoing_lanes = defaultdict(set)
        
        for tl_id in tls_data.keys():
            tl_conns = self.parser.get_tl_connections(tl_id)
            for conn in tl_conns:
                from_edge = conn["from"]
                to_edge = conn["to"]
                from_lane = f"{from_edge}_{conn['fromLane']}"
                to_lane = f"{to_edge}_{conn['toLane']}"
                
                tl_incoming_lanes[tl_id].add(from_lane)
                tl_outgoing_lanes[tl_id].add(to_lane)
                edge_to_tl[from_edge] = tl_id

        # Trace neighbors for each traffic light
        for tl_id in tls_data.keys():
            # Outgoing edges are the starting edges for downstream search
            outgoing_edges = set(lane.split("_")[0] for lane in tl_outgoing_lanes[tl_id])
            
            neighbors = {}
            # List of lanes connecting this tl to its neighbors
            connecting_lanes_to = defaultdict(list)
            
            for start_edge in outgoing_edges:
                # Queue stores (current_edge, current_distance, current_travel_time, path_edges)
                start_len = edges.get(start_edge, {}).get("length", 0.0)
                start_speed = edges.get(start_edge, {}).get("speed", 10.0)
                start_tt = start_len / max(1.0, start_speed)
                
                queue = deque([(start_edge, start_len, start_tt, [start_edge])])
                visited = {start_edge}
                
                while queue:
                    curr_edge, curr_dist, curr_tt, path = queue.popleft()
                    
                    if curr_dist > self.max_neighbor_distance:
                        continue
                        
                    # Check if this edge is controlled by another traffic light
                    next_tl = edge_to_tl.get(curr_edge)
                    if next_tl is not None and next_tl != tl_id:
                        # Found a neighbor!
                        if next_tl not in neighbors or curr_dist < neighbors[next_tl]["distance"]:
                            neighbors[next_tl] = {
                                "distance": curr_dist,
                                "travel_time": curr_tt,
                                "path": path
                            }
                        # For the connection between these intersections, collect lanes on path edges
                        for edge_on_path in path:
                            edge_info = edges.get(edge_on_path, {})
                            for lane_id in edge_info.get("lanes", []):
                                if lane_id not in connecting_lanes_to[next_tl]:
                                    connecting_lanes_to[next_tl].append(lane_id)
                        continue  # Stop tracing along this branch
                        
                    # Otherwise traverse to downstream edges
                    for next_edge, _ in edge_connections[curr_edge]:
                        if next_edge not in visited:
                            visited.add(next_edge)
                            next_len = edges.get(next_edge, {}).get("length", 0.0)
                            next_speed = edges.get(next_edge, {}).get("speed", 10.0)
                            next_tt = next_len / max(1.0, next_speed)
                            
                            queue.append((
                                next_edge,
                                curr_dist + next_len,
                                curr_tt + next_tt,
                                path + [next_edge]
                            ))
                            
            self.graph[tl_id] = {
                "incoming_lanes": list(tl_incoming_lanes[tl_id]),
                "outgoing_lanes": list(tl_outgoing_lanes[tl_id]),
                "neighbors": list(neighbors.keys()),
                "distance_to": {k: v["distance"] for k, v in neighbors.items()},
                "travel_time_to": {k: v["travel_time"] for k, v in neighbors.items()},
                "connecting_lanes_to": dict(connecting_lanes_to)
            }

    def _cluster_districts(self):
        # We group traffic lights into districts based on proximity.
        # Construct an undirected adjacency list where an edge represents proximity < 1200.0.
        adj = {tl_id: set() for tl_id in self.graph.keys()}
        for tl_id in self.graph.keys():
            for neighbor in self.graph[tl_id]["neighbors"]:
                dist = self.graph[tl_id]["distance_to"][neighbor]
                if dist < 1200.0:
                    adj[tl_id].add(neighbor)
                    adj[neighbor].add(tl_id)
                    
        # Get connected components as districts
        components = []
        visited_comp = set()
        for tl_id in self.graph.keys():
            if tl_id not in visited_comp:
                comp = set()
                queue = deque([tl_id])
                visited_comp.add(tl_id)
                while queue:
                    curr = queue.popleft()
                    comp.add(curr)
                    for nxt in adj[curr]:
                        if nxt not in visited_comp:
                            visited_comp.add(nxt)
                            queue.append(nxt)
                components.append(comp)
        
        self.districts = {}
        self.critical_district_order = {}
        self.connection_between_intersections = {}
        
        for idx, comp in enumerate(components):
            district_name = f"district_{idx + 1}"
            tls_in_district = list(comp)
            self.districts[district_name] = tls_in_district
            
            # Identify the critical intersection: the one with the highest degree/neighbors within the district
            degrees = {}
            for tl in tls_in_district:
                degrees[tl] = len([n for n in adj[tl] if n in tls_in_district])
                
            critical_tl = max(degrees, key=degrees.get) if degrees else tls_in_district[0]
            
            # Order the district intersections using BFS starting from the critical traffic light
            ordered_tls = [critical_tl]
            visited = {critical_tl}
            queue = deque([critical_tl])
            while queue:
                curr = queue.popleft()
                for neighbor in adj[curr]:
                    if neighbor in tls_in_district and neighbor not in visited:
                        visited.add(neighbor)
                        ordered_tls.append(neighbor)
                        queue.append(neighbor)
                        
            # Guarantee all district tls are included
            for tl in tls_in_district:
                if tl not in visited:
                    ordered_tls.append(tl)
                    
            self.critical_district_order[district_name] = ordered_tls
            
            # Map connecting lanes
            for tl_id in tls_in_district:
                for neighbor in self.graph[tl_id]["neighbors"]:
                    if neighbor in tls_in_district:
                        key = (tl_id, neighbor)
                        self.connection_between_intersections[key] = self.graph[tl_id]["connecting_lanes_to"].get(neighbor, [])

    def get_graph(self):
        return self.graph

    def get_districts(self):
        return self.districts

    def get_critical_district_order(self):
        return self.critical_district_order

    def get_connection_between_intersections(self):
        # Flatten keys to a format suitable for SCOSCA
        connections = {}
        for district_name, tls in self.districts.items():
            for tl_id in tls:
                lanes = []
                for neighbor in self.graph[tl_id]["neighbors"]:
                    if neighbor in tls:
                        lanes.extend(self.graph[tl_id]["connecting_lanes_to"].get(neighbor, []))
                connections[tl_id] = list(set(lanes))
        return connections
