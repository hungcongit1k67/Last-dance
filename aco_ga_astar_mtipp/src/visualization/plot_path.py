from __future__ import annotations

from pathlib import Path
from typing import Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

from src.core.grid_map import GridMap
from src.core.constants import OBSTACLE, TARGET

GridPosition = Tuple[int, int]


def _draw_cell_grid(ax: plt.Axes, rows: int, cols: int) -> None:
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)

    ax.grid(
        which="minor",
        color="black",
        linestyle="-",
        linewidth=1.6,
    )

    ax.tick_params(
        which="both",
        bottom=False,
        left=False,
        labelbottom=False,
        labelleft=False,
    )


def plot_path(
    grid_map: GridMap,
    path: Sequence[GridPosition],
    save_path: str | Path | None = None,
) -> None:
    obstacle_grid = grid_map.obstacle_grid
    radiation_grid = grid_map.radiation_grid

    rows, cols = obstacle_grid.shape

    fig_w = max(10, cols * 0.28)
    fig_h = max(8, rows * 0.28)

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # 1. Nền bản đồ phân loại rủi ro theo ngưỡng cố định từ colorbar:
    #   0: obstacle  -> đen
    #   1: low risk  -> xanh lam (cyan)   [radiation < 0.5]
    #   2: med risk  -> vàng              [0.5 <= radiation < 8.0]
    #   3: high risk -> hồng/đỏ nhạt     [radiation >= 8.0]
    LOW_THRESH = 0.5
    HIGH_THRESH = 8.0

    base = np.ones_like(obstacle_grid, dtype=int)  # mặc định low risk (1)

    if radiation_grid is not None:
        rad = np.array(radiation_grid, dtype=float)
        base[rad >= LOW_THRESH] = 2   # medium risk
        base[rad >= HIGH_THRESH] = 3  # high risk

    base[obstacle_grid == OBSTACLE] = 0  # obstacle đè lên tất cả

    cmap = ListedColormap([
        "#222222",  # 0: obstacle – đen
        "#87CEEB",  # 1: low risk – xanh lam (sky blue)
        "#FFFF66",  # 2: medium risk – vàng
        "#FFB6B6",  # 3: high risk – hồng nhạt (salmon)
    ])

    ax.imshow(base, cmap=cmap, vmin=0, vmax=3, origin="upper", interpolation="nearest")

    # 3. Vẽ đường đi bằng chấm tròn đỏ giống hình mẫu
    if path:
        xs = [p[1] for p in path]
        ys = [p[0] for p in path]

        ax.plot(
            xs,
            ys,
            color="red",
            linewidth=3.2,
            zorder=4,
        )

        ax.scatter(
            xs,
            ys,
            s=150,
            c="red",
            edgecolors="black",
            linewidths=1.2,
            zorder=5,
        )

    # 4. Vẽ target bằng ngôi sao xanh và đánh số
    target_positions = np.argwhere(obstacle_grid == TARGET)

    if len(target_positions):
        for idx, (r, c) in enumerate(target_positions, start=1):
            ax.scatter(
                c,
                r,
                marker="*",
                s=900,
                c="blue",
                edgecolors="blue",
                linewidths=1.0,
                zorder=8,
            )

            ax.text(
                c + 0.25,
                r + 0.25,
                str(idx),
                color="blue",
                fontsize=22,
                fontweight="bold",
                ha="left",
                va="center",
                zorder=9,
            )

    # 5. Kẻ ô vuông đen rõ nét
    _draw_cell_grid(ax, rows, cols)

    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)
    ax.set_aspect("equal")

    # Bỏ viền trắng dư
    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)

    if save_path is not None:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(
            save_path,
            dpi=200,
            bbox_inches="tight",
            pad_inches=0,
        )

    plt.close(fig)