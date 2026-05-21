from typing import Tuple

from src.utils.math_utils import euclidean_grid_distance

GridPosition = Tuple[int, int]


def segment_length(a: GridPosition, b: GridPosition, grid_size: float = 1.0) -> float:
    return euclidean_grid_distance(a, b, grid_size=grid_size)


def path_length(path: list[GridPosition], grid_size: float = 1.0) -> float:
    return sum(segment_length(path[i], path[i + 1], grid_size) for i in range(len(path) - 1))
