from __future__ import annotations

from src.costs.total_cost import CostWeights
from src.environment.map_loader import load_grid_map, load_targets
from src.planner.aco_ga_astar_planner import AcoGaAstarPlanner, PlannerResult
from src.utils.random_seed import set_random_seed


def solve_mtipp(config: dict) -> PlannerResult:
    if "random_seed" in config and config["random_seed"] is not None:
        set_random_seed(int(config["random_seed"]))
    grid_map = load_grid_map(config)
    targets = load_targets(config, grid_map)
    weights = CostWeights.from_config(config)
    planner = AcoGaAstarPlanner(grid_map, targets, weights, config)
    return planner.solve(start_index=0)
