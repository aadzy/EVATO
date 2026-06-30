import gzip
import xml.etree.ElementTree as ET
from collections import defaultdict
import os

class BangaloreNetworkParser:
    def __init__(self, net_xml_path):
        self.net_xml_path = net_xml_path
        self.tls_data = {}
        self.edges_data = {}
        self.junctions_data = {}
        self.connections_data = []
        self.tl_to_connections = defaultdict(list)
        self.edge_to_tl = {}
        self._parse()

    def _parse(self):
        if not os.path.exists(self.net_xml_path):
            raise FileNotFoundError(f"Network file not found at {self.net_xml_path}")

        # Open gzipped or regular xml file
        if self.net_xml_path.endswith(".gz"):
            opener = gzip.open(self.net_xml_path, "rb")
        else:
            opener = open(self.net_xml_path, "rb")

        try:
            tree = ET.parse(opener)
            root = tree.getroot()
        finally:
            opener.close()

        # 1. Parse Edges
        for edge in root.findall("edge"):
            # Ignore internal synthetic connection edges
            if edge.get("function") == "internal":
                continue
            edge_id = edge.get("id")
            lanes = edge.findall("lane")
            if lanes:
                # Store lanes metadata
                lane_ids = [lane.get("id") for lane in lanes]
                lengths = [float(l.get("length")) for l in lanes]
                speeds = [float(l.get("speed")) for l in lanes]
                self.edges_data[edge_id] = {
                    "lanes": lane_ids,
                    "length": lengths[0],  # default representative length
                    "speed": speeds[0],    # default representative speed
                    "lane_lengths": {l_id: len_val for l_id, len_val in zip(lane_ids, lengths)},
                    "lane_speeds": {l_id: sp_val for l_id, sp_val in zip(lane_ids, speeds)},
                }

        # 2. Parse Junctions
        for junction in root.findall("junction"):
            j_id = junction.get("id")
            # Ignore internal junctions
            if j_id.startswith(":"):
                continue
            self.junctions_data[j_id] = {
                "type": junction.get("type"),
                "x": float(junction.get("x")) if junction.get("x") else 0.0,
                "y": float(junction.get("y")) if junction.get("y") else 0.0,
                "incLanes": junction.get("incLanes", "").split(),
                "intLanes": junction.get("intLanes", "").split(),
            }

        # 3. Parse Connections
        for conn in root.findall("connection"):
            from_edge = conn.get("from")
            to_edge = conn.get("to")
            
            # Ignore internal connections
            if from_edge.startswith(":") or to_edge.startswith(":"):
                continue

            conn_info = {
                "from": from_edge,
                "to": to_edge,
                "fromLane": int(conn.get("fromLane")),
                "toLane": int(conn.get("toLane")),
                "tl": conn.get("tl"),
                "linkIndex": int(conn.get("linkIndex")) if conn.get("linkIndex") is not None else None,
            }
            self.connections_data.append(conn_info)
            
            tl_id = conn_info["tl"]
            if tl_id is not None:
                self.tl_to_connections[tl_id].append(conn_info)
                self.edge_to_tl[from_edge] = tl_id

        # 4. Parse Traffic Lights (tlLogic)
        for tl in root.findall("tlLogic"):
            tl_id = tl.get("id")
            phases = []
            for p in tl.findall("phase"):
                phases.append({
                    "duration": float(p.get("duration")),
                    "state": p.get("state"),
                })
            self.tls_data[tl_id] = {
                "type": tl.get("type"),
                "programID": tl.get("programID"),
                "offset": float(tl.get("offset")) if tl.get("offset") else 0.0,
                "phases": phases,
            }

    def get_tls(self):
        return self.tls_data

    def get_edges(self):
        return self.edges_data

    def get_connections(self):
        return self.connections_data

    def get_tl_connections(self, tl_id):
        return self.tl_to_connections.get(tl_id, [])
