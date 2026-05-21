from __future__ import annotations

from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt


def plot_convergence(history: Sequence[float], save_path: str | Path | None = None, title: str = "Convergence") -> None:
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(list(range(1, len(history) + 1)), history, linewidth=2.0)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Best cost")
    ax.set_title(title)
    ax.grid(True, linewidth=0.3)
    fig.tight_layout()
    if save_path is not None:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=200)
    plt.close(fig)
