from __future__ import annotations

import numpy as np


def initialize_pheromone_from_ga(
    n_targets: int,
    ga_route: list[int],
    base: float = 0.5,
    bonus: float = 0.5,
) -> np.ndarray:
    """Non-uniform initial pheromone based on GA route."""
    pheromone = np.full((n_targets, n_targets), float(base), dtype=float)
    np.fill_diagonal(pheromone, 0.0)
    for a, b in zip(ga_route[:-1], ga_route[1:]):
        pheromone[a, b] = base + bonus
        pheromone[b, a] = base + bonus
    return pheromone
