from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from src.core.grid_map import GridMap
from src.core.node import Node
from src.costs.total_cost import CostWeights, weighted_segment_cost, path_cost_components
from src.utils.math_utils import euclidean_grid_distance

GridPosition = Tuple[int, int]

_INF_COMPONENTS = {
    "length": float("inf"),
    "risk": float("inf"),
    "energy": float("inf"),
    "collision_risk": float("inf"),
    "total": float("inf"),
}


@dataclass
class AStarResult:
    path: List[GridPosition]
    components: dict
    reached: bool
    expanded_nodes: int


class ModifiedAStar:
    """Modified A* for pairwise multi-objective path planning.

    The g-cost uses weighted length, cumulative radiation risk, turning energy,
    and per-cell collision risk (formula 11). Cells with radiation >= ri_max are
    treated as blocked when high-risk avoidance is enabled.
    """

    def __init__(self, grid_map: GridMap, weights: CostWeights, high_risk_avoidance: bool = True):
        self.grid_map = grid_map
        self.weights = weights
        self.high_risk_avoidance = high_risk_avoidance

    def heuristic(self, current: GridPosition, goal: GridPosition) -> float:
        # Admissible lower bound: straight-line length + estimated radiation dose.
        # Collision risk lower bound is 0, so it is omitted here.
        length = euclidean_grid_distance(current, goal, self.grid_map.grid_size)
        time_hours = (length / self.grid_map.robot_velocity) / 3600.0
        avg_radiation = (self.grid_map.radiation_at(current) + self.grid_map.radiation_at(goal)) / 2.0
        risk = avg_radiation * time_hours
        return self.weights.omega_length * length + self.weights.omega_risk * risk

    def search(self, start: GridPosition, goal: GridPosition) -> AStarResult:
        if not self.grid_map.is_passable(start, avoid_high_risk=self.high_risk_avoidance):
            return AStarResult([], dict(_INF_COMPONENTS), False, 0)
        if not self.grid_map.is_passable(goal, avoid_high_risk=self.high_risk_avoidance):
            return AStarResult([], dict(_INF_COMPONENTS), False, 0)

        open_heap: list[Node] = []
        h0 = self.heuristic(start, goal)
        heapq.heappush(open_heap, Node(f=h0, row=start[0], col=start[1], g=0.0, h=h0, parent=None))

        came_from: Dict[GridPosition, Optional[GridPosition]] = {start: None}
        g_score: Dict[GridPosition, float] = {start: 0.0}
        closed: set[GridPosition] = set()
        expanded = 0

        while open_heap:
            current_node = heapq.heappop(open_heap)
            current = current_node.position
            if current in closed:
                continue
            closed.add(current)
            expanded += 1

            if current == goal:
                path = self._reconstruct_path(came_from, current)
                return AStarResult(path, path_cost_components(self.grid_map, path, self.weights), True, expanded)

            prev_pos = came_from.get(current)
            for nxt in self.grid_map.neighbors(current, avoid_high_risk=self.high_risk_avoidance):
                if nxt in closed:
                    continue
                step_cost, _, _, _, _ = weighted_segment_cost(self.grid_map, prev_pos, current, nxt, self.weights)
                tentative_g = g_score[current] + step_cost
                if tentative_g < g_score.get(nxt, float("inf")):
                    came_from[nxt] = current
                    g_score[nxt] = tentative_g
                    h = self.heuristic(nxt, goal)
                    heapq.heappush(open_heap, Node(f=tentative_g + h, row=nxt[0], col=nxt[1], g=tentative_g, h=h, parent=current))

        return AStarResult([], dict(_INF_COMPONENTS), False, expanded)

    @staticmethod
    def _reconstruct_path(came_from: Dict[GridPosition, Optional[GridPosition]], current: GridPosition) -> List[GridPosition]:
        path = [current]
        while came_from[current] is not None:
            current = came_from[current]  # type: ignore[assignment]
            path.append(current)
        path.reverse()
        return path
