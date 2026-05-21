from src.environment.map_loader import load_grid_map
from src.costs.total_cost import CostWeights
from src.lower_level.cost_matrix_builder import CostMatrixBuilder
from src.upper_level.genetic_algorithm import GeneticAlgorithmTSP
from src.upper_level.pheromone_initializer import initialize_pheromone_from_ga
from src.upper_level.ant_colony import AntColonyTSP
from src.utils.io_utils import load_yaml

cfg = load_yaml("configs/default.yaml")
grid = load_grid_map(cfg)
targets = grid.extract_targets()
weights = CostWeights.from_config(cfg)
matrices = CostMatrixBuilder(grid, targets, weights).build()
ga_result = GeneticAlgorithmTSP(matrices.total).run()
pheromone = initialize_pheromone_from_ga(len(targets), ga_result.route)
aco_result = AntColonyTSP(matrices.total, pheromone).run()
print("GA", ga_result.route, ga_result.cost)
print("ACO-GA", aco_result.route, aco_result.cost)
