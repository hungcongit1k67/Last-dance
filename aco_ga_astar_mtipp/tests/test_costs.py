from src.environment.map_loader import load_grid_map
from src.costs.total_cost import CostWeights, path_cost_components
from src.utils.io_utils import load_yaml


def test_path_cost_components_are_finite():
    cfg = load_yaml("configs/default.yaml")
    grid = load_grid_map(cfg)
    weights = CostWeights.from_config(cfg)
    path = [(0, 0), (1, 0), (2, 0)]
    comp = path_cost_components(grid, path, weights)
    assert comp["length"] > 0
    assert comp["risk"] >= 0
    assert comp["total"] > 0
