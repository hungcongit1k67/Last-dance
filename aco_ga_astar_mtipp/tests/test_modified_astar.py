from src.environment.map_loader import load_grid_map
from src.costs.total_cost import CostWeights
from src.lower_level.modified_astar import ModifiedAStar
from src.utils.io_utils import load_yaml


def test_modified_astar_finds_path_between_first_two_targets():
    cfg = load_yaml("configs/default.yaml")
    grid = load_grid_map(cfg)
    targets = grid.extract_targets()
    astar = ModifiedAStar(grid, CostWeights.from_config(cfg), high_risk_avoidance=True)
    result = astar.search(targets[0].position, targets[1].position)
    assert result.reached
    assert result.path[0] == targets[0].position
    assert result.path[-1] == targets[1].position
