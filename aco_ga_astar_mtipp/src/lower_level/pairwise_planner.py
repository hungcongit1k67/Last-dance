from __future__ import annotations

from typing import Tuple

from src.core.grid_map import GridMap
from src.costs.total_cost import CostWeights
from src.lower_level.modified_astar import ModifiedAStar, AStarResult

GridPosition = Tuple[int, int]


class PairwisePlanner:
    def __init__(self, grid_map: GridMap, weights: CostWeights, high_risk_avoidance: bool = True):
        self.astar = ModifiedAStar(grid_map, weights, high_risk_avoidance=high_risk_avoidance)

    def plan(self, start: GridPosition, goal: GridPosition) -> AStarResult:
        return self.astar.search(start, goal)
