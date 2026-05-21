from src.planner.mtipp_solver import solve_mtipp
from src.utils.io_utils import load_yaml


def test_full_planner_runs_default():
    cfg = load_yaml("configs/default.yaml")
    cfg["GA"]["population_size"] = 20
    cfg["GA"]["max_generation"] = 10
    cfg["ACO"]["num_ants"] = 10
    cfg["ACO"]["max_iteration"] = 10
    result = solve_mtipp(cfg)
    assert result.aco_route[0] == 0
    assert result.aco_route[-1] == 0
    assert len(result.full_path) > 0
    assert result.full_path_components["total"] > 0
