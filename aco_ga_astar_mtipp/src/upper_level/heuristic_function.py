from __future__ import annotations

import numpy as np


def adaptive_xi(current_iteration: int, max_iteration: int) -> float:
    if max_iteration <= 0:
        return 1.0
    if current_iteration >= max_iteration:
        return 1.0 / max_iteration
    return (max_iteration - current_iteration) / max_iteration


def heuristic_value(cost_matrix: np.ndarray, i: int, j: int, current_iteration: int, max_iteration: int) -> float:
    cost = float(cost_matrix[i, j])
    if not np.isfinite(cost) or cost <= 0:
        return 0.0
    return adaptive_xi(current_iteration, max_iteration) / cost
