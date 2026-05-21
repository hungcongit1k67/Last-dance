from src.environment.map_loader import load_grid_map
from src.costs.total_cost import CostWeights
from src.lower_level.modified_astar import ModifiedAStar
from src.utils.io_utils import load_yaml

cfg = load_yaml("configs/default.yaml")
grid = load_grid_map(cfg)
weights = CostWeights.from_config(cfg)
targets = grid.extract_targets()
astar = ModifiedAStar(grid, weights, high_risk_avoidance=True)
result = astar.search(targets[0].position, targets[1].position)
print(result.reached, result.components)
print(result.path)
