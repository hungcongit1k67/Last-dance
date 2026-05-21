import numpy as np

from src.upper_level.genetic_algorithm import GeneticAlgorithmTSP


def test_ga_returns_closed_route():
    matrix = np.array([
        [0, 1, 2, 3],
        [1, 0, 4, 2],
        [2, 4, 0, 1],
        [3, 2, 1, 0],
    ], dtype=float)
    result = GeneticAlgorithmTSP(matrix, population_size=20, max_generation=10).run()
    assert result.route[0] == 0
    assert result.route[-1] == 0
    assert len(set(result.route[:-1])) == 4
    assert result.cost > 0
