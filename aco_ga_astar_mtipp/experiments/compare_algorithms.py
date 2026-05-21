"""Minimal comparison scaffold.

The paper compares ACO-GA-A* with ACO-A*, GA-A*, GA-ACO-A*, etc.
This file keeps the extension point for those baselines. The current
implementation runs the proposed ACO-GA-A* for all three sample scenarios.
"""

from src.planner.mtipp_solver import solve_mtipp
from src.utils.io_utils import load_yaml

for config_path in ["configs/scenario_small.yaml", "configs/scenario_medium.yaml", "configs/scenario_large.yaml"]:
    cfg = load_yaml(config_path)
    result = solve_mtipp(cfg)
    print(config_path, result.aco_route, result.aco_cost, result.full_path_components)
