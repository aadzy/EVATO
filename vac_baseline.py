from __future__ import annotations

import csv
import json
import random
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import traci


BASE_DIR = Path(__file__).resolve().parent
SRC_DIR = BASE_DIR / "sumoITScontrol" / "src"
if SRC_DIR.exists() and str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from sumoITScontrol import Intersection  # noqa: E402
from sumoITScontrol.control.intersection_management import MaxPressure_Flex  # noqa: E402


def _resolve_default_config_file() -> Path:
    candidates = [
        BASE_DIR / "Bangalore_Map" / "osm.sumocfg",
        BASE_DIR / "Bangalore_Map" / "osm.sumocfg.xml",
        BASE_DIR / "2026-06-18-09-55-06" / "osm.sumocfg",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return BASE_DIR / "Bangalore_Map" / "osm.sumocfg.xml"


DEFAULT_CONFIG_FILE = _resolve_default_config_file()
DEFAULT_OUTPUT_FILE = BASE_DIR / "baseline_metrics.csv"


def build_sumo_cmd(use_gui: bool = True, config_file: Path = DEFAULT_CONFIG_FILE) -> List[str]:
    executable = "sumo-gui" if use_gui else "sumo"
    return [
        executable,
        "-c",
        str(config_file),
        "--start",
        "--quit-on-end",
        "--time-to-teleport",
        "-1",
    ]


class CorridorIntersection(Intersection):
    def __init__(
        self,
        tl_id: str,
        phases: Sequence[int],
        links: Dict[int, List[str]],
        phase_translation: Dict[int, int],
        green_states: Optional[Sequence[str]] = None,
        yellow_states: Optional[Sequence[str]] = None,
    ):
        super().__init__(
            tl_id=tl_id,
            phases=list(phases),
            links=links,
            green_states=list(green_states) if green_states is not None else None,
            yellow_states=list(yellow_states) if yellow_states is not None else None,
        )
        self.phase_translation = phase_translation

    def set_signal_on_traffic_lights(self, phase):
        actual_phase = self.phase_translation.get(phase, phase)
        traci.trafficlight.setPhase(self.tl_id, actual_phase)


class VACBaseline:
    def __init__(self, sumo_cmd, network_file):
        self.sumo_cmd = list(sumo_cmd)
        self.network_file = Path(network_file)
        self.controllers: List[MaxPressure_Flex] = []
        self.intersections: List[CorridorIntersection] = []
        self.rules_documentation: Dict[str, object] = {}
        self.metrics: List[Dict[str, object]] = []
        self.decision_log: List[Dict[str, object]] = []
        self.route_templates: List[List[str]] = []
        self.ambulance_schedule: List[Tuple[int, str, List[str]]] = []
        self.emergency_vehicle_ids = set()
        self.passenger_rate_vph = 1500
        self.step_length_seconds = 1.0
        self.measurement_period_steps = 1
        self.total_sim_steps = 0
        self.output_file = DEFAULT_OUTPUT_FILE
        self._passenger_spawn_credit = 0.0
        self._passenger_vehicle_counter = 0
        self._route_counter = 0
        self._initialized = False

    def _safe_time_seconds(self) -> float:
        try:
            return traci.simulation.getCurrentTime() / 1000.0
        except Exception:
            return 0.0

    def _lane_to_edge(self, lane_id: str) -> str:
        return lane_id.split("_")[0]

    def _get_tls_position(self, tls_id: str):
        try:
            controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
        except Exception:
            return None

        if not controlled_lanes:
            return None

        for lane_id in controlled_lanes:
            try:
                lane_shape = traci.lane.getShape(lane_id)
            except Exception:
                continue
            if lane_shape:
                return lane_shape[-1]

        return None

    def _sort_tls_ids(self, tls_ids: Sequence[str]) -> List[str]:
        positioned = []
        for tls_id in tls_ids:
            pos = self._get_tls_position(tls_id)
            if pos is None:
                positioned.append((tls_id, float("inf"), float("inf")))
            else:
                positioned.append((tls_id, pos[0], pos[1]))
        positioned.sort(key=lambda item: (item[1], item[2], item[0]))
        return [item[0] for item in positioned]

    def _edge_allows_default_vehicle(self, edge_id: str) -> bool:
        if not edge_id or edge_id.startswith(":"):
            return False

        try:
            lane_count = int(traci.edge.getLaneNumber(edge_id))
        except Exception:
            return False

        for lane_index in range(lane_count):
            lane_id = f"{edge_id}_{lane_index}"
            try:
                allowed_classes = traci.lane.getAllowed(lane_id)
            except Exception:
                continue

            if not allowed_classes:
                return True
            if "DEFAULT_VEHTYPE" in allowed_classes:
                return True
            if "passenger" in allowed_classes:
                return True

        return False

    def _extract_intersection_definition(self, tls_id: str, phase_offset: int) -> CorridorIntersection:
        program_logics = traci.trafficlight.getAllProgramLogics(tls_id)
        if not program_logics:
            raise RuntimeError(f"No traffic-light program found for {tls_id}")

        program = program_logics[0]
        controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        links: Dict[int, List[str]] = {}
        phase_translation: Dict[int, int] = {}
        green_states: List[str] = []
        yellow_states: List[str] = []

        controller_phase_id = phase_offset
        for actual_phase_index, phase in enumerate(program.phases):
            state = getattr(phase, "state", "")
            if not state:
                continue

            incoming_lanes: List[str] = []
            for signal_index, signal_links in enumerate(controlled_links):
                if signal_index >= len(state):
                    continue
                if state[signal_index] not in {"G", "g"}:
                    continue
                if not signal_links:
                    continue
                incoming_lane = signal_links[0][0]
                if incoming_lane not in incoming_lanes:
                    incoming_lanes.append(incoming_lane)

            if incoming_lanes:
                links[controller_phase_id] = incoming_lanes
                phase_translation[controller_phase_id] = actual_phase_index
                green_states.append(state)
                if actual_phase_index + 1 < len(program.phases):
                    yellow_state = getattr(program.phases[actual_phase_index + 1], "state", "")
                    yellow_states.append(yellow_state or ("y" * len(state)))
                else:
                    yellow_states.append("y" * len(state))
                controller_phase_id += 2

        if not links:
            raise RuntimeError(f"Unable to infer green phases for {tls_id}")

        return CorridorIntersection(
            tl_id=tls_id,
            phases=sorted(links.keys()),
            links=links,
            phase_translation=phase_translation,
            green_states=green_states,
            yellow_states=yellow_states,
        )

    def _build_corridor_layout(self) -> None:
        tls_ids = traci.trafficlight.getIDList()
        if len(tls_ids) < 3:
            raise RuntimeError(
                "VAC baseline requires at least 3 signalized intersections in the loaded SUMO network"
            )

        corridor_tls_ids = self._sort_tls_ids(tls_ids)[:3]
        self.intersections = []
        for idx, tls_id in enumerate(corridor_tls_ids):
            intersection = self._extract_intersection_definition(tls_id, phase_offset=idx * 2)
            self.intersections.append(intersection)

        self.controllers = [
            MaxPressure_Flex(self._controller_params(), intersection)
            for intersection in self.intersections
        ]

    def _controller_params(self) -> Dict[str, int]:
        return {
            "T_L": 3,
            "T_A": 5,
            "G_T_MIN": 5,
            "G_T_MAX": 45,
            "measurement_period": self.measurement_period_steps,
        }

    def _derive_corridor_edges(self) -> Tuple[List[str], List[str]]:
        if not self.intersections:
            return [], []

        entry_edges: List[str] = []
        exit_edges: List[str] = []

        for lanes in self.intersections[0].links.values():
            for lane in lanes:
                edge_id = self._lane_to_edge(lane)
                if edge_id not in entry_edges:
                    entry_edges.append(edge_id)

        for lanes in self.intersections[-1].links.values():
            for lane in lanes:
                edge_id = self._lane_to_edge(lane)
                if edge_id not in exit_edges:
                    exit_edges.append(edge_id)

        return entry_edges, exit_edges

    def _build_route_templates(self) -> None:
        route_templates: List[List[str]] = []
        edge_ids = [edge_id for edge_id in traci.edge.getIDList() if self._edge_allows_default_vehicle(edge_id)]
        entry_edges, exit_edges = self._derive_corridor_edges()
        entry_pool = [edge_id for edge_id in entry_edges if self._edge_allows_default_vehicle(edge_id)] or edge_ids
        exit_pool = [edge_id for edge_id in exit_edges if self._edge_allows_default_vehicle(edge_id)] or edge_ids

        if not edge_ids:
            raise RuntimeError("No valid departure edges were found for DEFAULT_VEHTYPE")

        for _ in range(80):
            from_edge = random.choice(entry_pool)
            to_edge = random.choice(exit_pool)
            if from_edge == to_edge:
                continue
            try:
                stage = traci.simulation.findRoute(from_edge, to_edge)
                edges = list(getattr(stage, "edges", []))
                if len(edges) >= 2 and edges not in route_templates:
                    route_templates.append(edges)
            except Exception:
                continue

        if not route_templates:
            for _ in range(40):
                from_edge = random.choice(edge_ids)
                to_edge = random.choice(edge_ids)
                if from_edge == to_edge:
                    continue
                try:
                    stage = traci.simulation.findRoute(from_edge, to_edge)
                    edges = list(getattr(stage, "edges", []))
                    if len(edges) >= 2 and edges not in route_templates:
                        route_templates.append(edges)
                except Exception:
                    continue

        if not route_templates:
            raise RuntimeError("Unable to build valid vehicle routes from the loaded SUMO network")

        self.route_templates = route_templates

    def _build_ambulance_schedule(self, duration_seconds: int) -> None:
        self.ambulance_schedule = []
        ambulance_count = random.randint(2, 3)
        sorted_routes = sorted(self.route_templates, key=len, reverse=True)

        for idx in range(ambulance_count):
            depart_time = random.randint(0, max(1, duration_seconds // 3))
            route_edges = sorted_routes[idx % len(sorted_routes)]
            veh_id = f"ambulance_{idx + 1}"
            self.ambulance_schedule.append((depart_time, veh_id, route_edges))

        self.ambulance_schedule.sort(key=lambda item: item[0])

    def _spawn_vehicle_on_route(self, veh_id: str, route_edges: Sequence[str], is_emergency: bool = False) -> bool:
        route_id = f"route_{self._route_counter}"
        self._route_counter += 1
        try:
            traci.route.add(route_id, list(route_edges))
            traci.vehicle.add(veh_id, route_id, typeID="DEFAULT_VEHTYPE")
            if is_emergency:
                self.emergency_vehicle_ids.add(veh_id)
                try:
                    traci.vehicle.setColor(veh_id, (255, 0, 0))
                except Exception:
                    pass
            return True
        except Exception:
            return False

    def _spawn_traffic(self, current_time_seconds: float) -> None:
        while self.ambulance_schedule and self.ambulance_schedule[0][0] <= int(current_time_seconds):
            _, veh_id, route_edges = self.ambulance_schedule.pop(0)
            if self._spawn_vehicle_on_route(veh_id, route_edges, is_emergency=True):
                print(f"[VAC] t={current_time_seconds:.1f}s spawned emergency vehicle {veh_id} on a randomized route")

        self._passenger_spawn_credit += self.passenger_rate_vph * self.step_length_seconds / 3600.0
        passenger_limit = 25
        spawned_passengers = 0
        while self._passenger_spawn_credit >= 1.0 and spawned_passengers < passenger_limit:
            route_edges = random.choice(self.route_templates)
            veh_id = f"passenger_{self._passenger_vehicle_counter}"
            self._passenger_vehicle_counter += 1
            if self._spawn_vehicle_on_route(veh_id, route_edges, is_emergency=False):
                spawned_passengers += 1
            self._passenger_spawn_credit = max(0.0, self._passenger_spawn_credit - 1.0)

    def _vehicle_delay(self, veh_id: str) -> float:
        try:
            return float(traci.vehicle.getTimeLoss(veh_id))
        except Exception:
            try:
                return float(traci.vehicle.getAccumulatedWaitingTime(veh_id))
            except Exception:
                return 0.0

    def _collect_step_metrics(self, current_time_seconds: float) -> None:
        vehicle_ids = list(traci.vehicle.getIDList())
        ev_delays = []
        passenger_delays = []
        queue_length = 0
        tl_phase_summary = []
        pressure_values = []

        for controller in self.controllers:
            intersection = controller.intersection
            phase_id = controller.measurement_data.get("current_signal_phase", 0)
            tl_phase_summary.append(f"{intersection.tl_id}:{phase_id}")

            pressures = controller.measurement_data.get("pressures") or intersection.get_queue_lengths_num_vehicles()
            if pressures:
                selected_index = min(len(pressures) - 1, max(0, phase_id // 2))
                pressure_values.append(float(pressures[selected_index]))
                queue_length += int(sum(pressures))

        for veh_id in vehicle_ids:
            delay = self._vehicle_delay(veh_id)
            if veh_id in self.emergency_vehicle_ids or "ambulance" in veh_id.lower():
                ev_delays.append(delay)
            else:
                passenger_delays.append(delay)

        self.metrics.append(
            {
                "time": round(current_time_seconds, 3),
                "ev_delay": round(float(np.mean(ev_delays)) if ev_delays else 0.0, 3),
                "passenger_delay": round(float(np.mean(passenger_delays)) if passenger_delays else 0.0, 3),
                "queue_length": int(queue_length),
                "tl_phase": "|".join(tl_phase_summary),
                "pressure_value": round(float(np.mean(pressure_values)) if pressure_values else 0.0, 3),
            }
        )

    def log_vac_decision(self, tl_id, phase, pressure):
        timestamp = self._safe_time_seconds()
        reason = "max_pressure"
        entry = {
            "time": round(timestamp, 3),
            "tl_id": tl_id,
            "phase": int(phase),
            "pressure": round(float(pressure), 3),
            "reason": reason,
        }
        self.decision_log.append(entry)
        print(
            f"[VAC] t={entry['time']:.3f}s tl={tl_id} phase={entry['phase']} pressure={entry['pressure']:.3f} reason={reason}"
        )

    def initialize_vac(self, duration_seconds: int = 3600) -> Dict[str, object]:
        try:
            delta_t_ms = float(traci.simulation.getDeltaT())
            self.step_length_seconds = max(delta_t_ms / 1000.0, 1e-6)
        except Exception:
            self.step_length_seconds = 1.0

        self.measurement_period_steps = max(1, int(round(1.0 / self.step_length_seconds)))

        self._build_corridor_layout()
        self._build_route_templates()
        self._build_ambulance_schedule(duration_seconds=duration_seconds)

        self.rules_documentation = {
            "control_strategy": "MaxPressure_Flex",
            "network_file": str(self.network_file),
            "corridor_intersections": [intersection.tl_id for intersection in self.intersections],
            "timing": {
                "step_length_seconds": self.step_length_seconds,
                "measurement_period_steps": self.measurement_period_steps,
                "cycle_duration_seconds": 90,
                "green_min_seconds": 5,
                "green_max_seconds": 45,
                "yellow_loss_seconds": 3,
                "adaptation_wait_seconds": 5,
            },
            "traffic_generation": {
                "passenger_vehicle_rate_vph": self.passenger_rate_vph,
                "ambulance_count": len(self.ambulance_schedule),
                "ambulance_routes_randomized": True,
            },
            "measurement_outputs": ["ev_delay", "passenger_delay", "queue_length", "tl_phase", "pressure_value"],
            "logging": {
                "decision_log": "timestamp, tl_id, phase, pressure, reason",
                "metrics_file": str(self.output_file),
            },
        }

        print("\nVAC baseline rules")
        print(json.dumps(self.rules_documentation, indent=2))
        for intersection, controller in zip(self.intersections, self.controllers):
            print(
                f"[VAC] {intersection.tl_id}: phases={intersection.phases}, phase_translation={intersection.phase_translation}, params={controller.params}"
            )

        self._initialized = True
        return self.rules_documentation

    def run_simulation(self, duration_seconds):
        if not self.network_file.exists():
            raise FileNotFoundError(f"SUMO config file not found: {self.network_file}")

        traci.start(self.sumo_cmd)
        try:
            if not self._initialized:
                self.initialize_vac(duration_seconds=duration_seconds)

            self.total_sim_steps = int(duration_seconds / max(self.step_length_seconds, 1e-6)) + 1
            for _ in range(self.total_sim_steps):
                current_time_seconds = self._safe_time_seconds()
                if current_time_seconds >= duration_seconds:
                    break

                self._spawn_traffic(current_time_seconds)
                traci.simulationStep()
                current_time_seconds = self._safe_time_seconds()

                for controller in self.controllers:
                    before_phase = controller.measurement_data.get("current_signal_phase", 0)
                    controller.execute_control(traci.simulation.getCurrentTime())
                    after_phase = controller.measurement_data.get("current_signal_phase", before_phase)
                    pressures = controller.measurement_data.get("pressures") or []
                    if pressures:
                        selected_index = min(len(pressures) - 1, max(0, after_phase // 2))
                        pressure_value = float(pressures[selected_index])
                    else:
                        pressure_value = 0.0

                    if after_phase != before_phase:
                        reason = "phase_changed_by_max_pressure"
                    else:
                        reason = "phase_held_by_max_pressure"
                    print(
                        f"[VAC] t={current_time_seconds:.3f}s tl={controller.intersection.tl_id} phase={after_phase} pressure={pressure_value:.3f} reason={reason}"
                    )
                    self.decision_log.append(
                        {
                            "time": round(current_time_seconds, 3),
                            "tl_id": controller.intersection.tl_id,
                            "phase": int(after_phase),
                            "pressure": round(pressure_value, 3),
                            "reason": reason,
                        }
                    )

                self._collect_step_metrics(current_time_seconds)
        finally:
            try:
                traci.close()
            finally:
                self.save_metrics(self.output_file)
                print(f"[VAC] metrics saved to {self.output_file}")

    def save_metrics(self, output_file):
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = ["time", "ev_delay", "passenger_delay", "queue_length", "tl_phase", "pressure_value"]

        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.metrics:
                writer.writerow({field: row.get(field, "") for field in fieldnames})


def main(
    duration_seconds: int = 3600,
    sumo_cmd=None,
    network_file=None,
    output_file=None,
    use_gui: bool = True,
):
    baseline = VACBaseline(
        sumo_cmd=sumo_cmd or build_sumo_cmd(use_gui=use_gui),
        network_file=network_file or DEFAULT_CONFIG_FILE,
    )
    if output_file is not None:
        baseline.output_file = Path(output_file)
    baseline.run_simulation(duration_seconds=duration_seconds)


if __name__ == "__main__":
    main()