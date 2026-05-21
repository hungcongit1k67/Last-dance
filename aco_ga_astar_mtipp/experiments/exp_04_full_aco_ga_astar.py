from src.planner.mtipp_solver import solve_mtipp
from src.utils.io_utils import load_yaml

cfg = load_yaml("configs/default.yaml")
result = solve_mtipp(cfg)
print(result.aco_route, result.aco_cost)
print(result.full_path_components)
