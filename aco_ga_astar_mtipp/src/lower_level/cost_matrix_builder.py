from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from src.core.grid_map import GridMap
from src.core.target import Target
from src.costs.total_cost import CostWeights
from src.lower_level.pairwise_planner import PairwisePlanner

GridPosition = Tuple[int, int]


@dataclass
class CostMatrices:
    length: np.ndarray
    risk: np.ndarray
    energy: np.ndarray
    total: np.ndarray
    pairwise_paths: Dict[Tuple[int, int], List[GridPosition]]


class CostMatrixBuilder:
    """Run modified A* for each ordered target pair and collect cost matrices."""

    def __init__(self, grid_map: GridMap, targets: List[Target], weights: CostWeights, high_risk_avoidance: bool = True):
        self.grid_map = grid_map
        self.targets = targets
        self.weights = weights
        self.pairwise_planner = PairwisePlanner(grid_map, weights, high_risk_avoidance=high_risk_avoidance)

    def build(self) -> CostMatrices:
        n = len(self.targets)
        length = np.full((n, n), np.inf)
        risk = np.full((n, n), np.inf)
        energy = np.full((n, n), np.inf)
        total = np.full((n, n), np.inf)
        paths: Dict[Tuple[int, int], List[GridPosition]] = {}

        for i, ti in enumerate(self.targets):
            for j, tj in enumerate(self.targets):
                if i == j:
                    length[i, j] = risk[i, j] = energy[i, j] = total[i, j] = 0.0
                    paths[(i, j)] = [ti.position]
                    continue
                result = self.pairwise_planner.plan(ti.position, tj.position)
                if result.reached:
                    length[i, j] = result.components["length"]
                    risk[i, j] = result.components["risk"]
                    energy[i, j] = result.components["energy"]
                    total[i, j] = result.components["total"]
                    paths[(i, j)] = result.path

        return CostMatrices(length=length, risk=risk, energy=energy, total=total, pairwise_paths=paths)
