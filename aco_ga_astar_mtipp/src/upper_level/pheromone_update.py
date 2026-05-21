from __future__ import annotations

import numpy as np


def update_pheromone_elite(
    pheromone: np.ndarray,
    best_route: list[int],
    best_cost: float,
    worst_route: list[int] | None,
    worst_cost: float | None,
    rho: float,
    q: float,
    min_pheromone: float = 1e-6,
    max_pheromone: float = 10.0,
) -> np.ndarray:
    """Elite pheromone update with evaporation, best-route reinforcement and optional worst-route penalty."""
    new_pheromone = (1.0 - rho) * pheromone

    if np.isfinite(best_cost) and best_cost > 0:
        delta_best = rho * q / best_cost
        for a, b in zip(best_route[:-1], best_route[1:]):
            new_pheromone[a, b] += delta_best
            new_pheromone[b, a] += delta_best

    if worst_route is not None and worst_cost is not None and np.isfinite(worst_cost) and worst_cost > 0:
        delta_worst = rho * q / worst_cost
        for a, b in zip(worst_route[:-1], worst_route[1:]):
            new_pheromone[a, b] -= 0.25 * delta_worst
            new_pheromone[b, a] -= 0.25 * delta_worst

    np.fill_diagonal(new_pheromone, 0.0)
    return np.clip(new_pheromone, min_pheromone, max_pheromone)
