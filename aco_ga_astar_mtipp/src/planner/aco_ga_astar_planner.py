from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from src.core.grid_map import GridMap
from src.core.target import Target
from src.costs.total_cost import CostWeights, path_cost_components
from src.lower_level.cost_matrix_builder import CostMatrices, CostMatrixBuilder
from src.upper_level.genetic_algorithm import GeneticAlgorithmTSP
from src.upper_level.pheromone_initializer import initialize_pheromone_from_ga
from src.upper_level.ant_colony import AntColonyTSP
import time

GridPosition = Tuple[int, int]


@dataclass
class PlannerResult:
    targets: List[Target]
    cost_matrices: CostMatrices
    ga_route: List[int]
    ga_cost: float
    aco_route: List[int]
    aco_cost: float
    full_path: List[GridPosition]
    full_path_components: dict
    ga_history: List[float]
    aco_history: List[float]
    timings: dict


class AcoGaAstarPlanner:
    """Bi-level ACO-GA-A* MTIPP planner."""

    def __init__(self, grid_map: GridMap, targets: List[Target], weights: CostWeights, config: dict):
        if len(targets) < 2:
            raise ValueError("At least two targets are required for MTIPP")
        self.grid_map = grid_map
        self.targets = targets
        self.weights = weights
        self.config = config

    def solve(self, start_index: int = 0) -> PlannerResult:
        astar_cfg = self.config.get("astar", {})
        builder = CostMatrixBuilder(
            self.grid_map,
            self.targets,
            self.weights,
            high_risk_avoidance=bool(astar_cfg.get("high_risk_avoidance", True)),
        )
        t_lower_start = time.perf_counter()
        matrices = builder.build()
        lower_level_time = time.perf_counter() - t_lower_start

        if not np.isfinite(matrices.total).any():
            raise RuntimeError("No finite pairwise path was found")

        ga_cfg = self.config.get("GA", self.config.get("ga", {}))
        ga = GeneticAlgorithmTSP(
            matrices.total,
            start_index=start_index,
            population_size=int(ga_cfg.get("population_size", 80)),
            max_generation=int(ga_cfg.get("max_generation", 150)),
            crossover_rate=float(ga_cfg.get("crossover_rate", 0.9)),
            mutation_rate=float(ga_cfg.get("mutation_rate", 0.08)),
            elite_size=int(ga_cfg.get("elite_size", 2)),
            gap=float(ga_cfg.get("gap", 0.9)),
        )
        t_ga_start = time.perf_counter()
        ga_result = ga.run()
        ga_time = time.perf_counter() - t_ga_start

        aco_cfg = self.config.get("ACO", self.config.get("aco", {}))
        pheromone = initialize_pheromone_from_ga(
            n_targets=len(self.targets),
            ga_route=ga_result.route,
            base=float(aco_cfg.get("initial_pheromone", 0.5)),
            bonus=float(aco_cfg.get("ga_pheromone_bonus", 0.5)),
        )
        aco = AntColonyTSP(
            matrices.total,
            initial_pheromone=pheromone,
            start_index=start_index,
            num_ants=int(aco_cfg.get("num_ants", 50)),
            max_iteration=int(aco_cfg.get("max_iteration", 180)),
            alpha=float(aco_cfg.get("alpha", 1.0)),
            beta=float(aco_cfg.get("beta", 5.0)),
            rho=float(aco_cfg.get("rho", 0.15)),
            q=float(aco_cfg.get("q", 100.0)),
            q0=float(aco_cfg.get("q0", 0.85)),
            min_pheromone=float(aco_cfg.get("min_pheromone", 1e-6)),
            max_pheromone=float(aco_cfg.get("max_pheromone", 10.0)),
        )
        t_aco_start = time.perf_counter()
        aco_result = aco.run()
        aco_time = time.perf_counter() - t_aco_start

        if not np.isfinite(aco_result.cost):
            raise RuntimeError("ACO returned an infeasible route. Check obstacle/radiation maps.")

        t_stitch_start = time.perf_counter()
        full_path = self._stitch_pairwise_paths(aco_result.route, matrices)
        full_path_components = path_cost_components(self.grid_map, full_path, self.weights)
        stitch_time = time.perf_counter() - t_stitch_start

        timings = {
            "lower_level_modified_astar_time_sec": lower_level_time,
            "upper_level_ga_time_sec": ga_time,
            "upper_level_aco_time_sec": aco_time,
            "upper_level_total_time_sec": ga_time + aco_time,
            "stitch_and_full_cost_time_sec": stitch_time,
            "algorithm_total_time_sec": lower_level_time + ga_time + aco_time + stitch_time,
        }

        return PlannerResult(
            targets=self.targets,
            cost_matrices=matrices,
            ga_route=ga_result.route,
            ga_cost=ga_result.cost,
            aco_route=aco_result.route,
            aco_cost=aco_result.cost,
            full_path=full_path,
            full_path_components=full_path_components,
            ga_history=ga_result.history,
            aco_history=aco_result.history,
            timings=timings,
        )

    @staticmethod
    def _stitch_pairwise_paths(route: List[int], matrices: CostMatrices) -> List[GridPosition]:
        full_path: List[GridPosition] = []
        for k, (a, b) in enumerate(zip(route[:-1], route[1:])):
            segment = matrices.pairwise_paths.get((a, b))
            if not segment:
                raise RuntimeError(f"Missing pairwise path for target edge {a}->{b}")
            if k == 0:
                full_path.extend(segment)
            else:
                full_path.extend(segment[1:])
        return full_path
