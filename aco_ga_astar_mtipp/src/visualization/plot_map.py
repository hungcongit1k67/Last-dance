from __future__ import annotations

from typing import Optional

import matplotlib.pyplot as plt
import numpy as np

from src.core.grid_map import GridMap
from src.core.constants import OBSTACLE, TARGET


def plot_map(grid_map: GridMap, ax: Optional[plt.Axes] = None) -> plt.Axes:
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 6))
    ax.imshow(grid_map.radiation_grid, origin="upper")
    obstacle_mask = np.ma.masked_where(grid_map.obstacle_grid != OBSTACLE, grid_map.obstacle_grid)
    ax.imshow(obstacle_mask, origin="upper", alpha=0.65)
    target_positions = np.argwhere(grid_map.obstacle_grid == TARGET)
    if len(target_positions):
        ax.scatter(target_positions[:, 1], target_positions[:, 0], marker="*", s=160, label="Targets")
        for idx, (r, c) in enumerate(target_positions, start=1):
            ax.text(c + 0.15, r + 0.15, f"T{idx}", fontsize=9)
    ax.set_title("Two-layer map: radiation + obstacles + targets")
    ax.set_xlabel("col")
    ax.set_ylabel("row")
    ax.grid(True, linewidth=0.25)
    return ax
