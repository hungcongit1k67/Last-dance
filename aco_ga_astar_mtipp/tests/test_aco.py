import numpy as np

from src.upper_level.ant_colony import AntColonyTSP


def test_aco_returns_closed_route():
    matrix = np.array([
        [0, 1, 2, 3],
        [1, 0, 4, 2],
        [2, 4, 0, 1],
        [3, 2, 1, 0],
    ], dtype=float)
    pheromone = np.ones_like(matrix) * 0.5
    np.fill_diagonal(pheromone, 0.0)
    result = AntColonyTSP(matrix, pheromone, num_ants=10, max_iteration=10).run()
    assert result.route[0] == 0
    assert result.route[-1] == 0
    assert len(set(result.route[:-1])) == 4
    assert result.cost > 0
