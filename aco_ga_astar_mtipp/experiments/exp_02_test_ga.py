from src.environment.map_loader import load_grid_map
from src.costs.total_cost import CostWeights
from src.lower_level.cost_matrix_builder import CostMatrixBuilder
from src.upper_level.genetic_algorithm import GeneticAlgorithmTSP
from src.utils.io_utils import load_yaml

cfg = load_yaml("configs/default.yaml")
grid = load_grid_map(cfg)
targets = grid.extract_targets()
weights = CostWeights.from_config(cfg)
matrices = CostMatrixBuilder(grid, targets, weights).build()
ga = GeneticAlgorithmTSP(matrices.total, population_size=80, max_generation=120)
result = ga.run()
print(result.route, result.cost)
