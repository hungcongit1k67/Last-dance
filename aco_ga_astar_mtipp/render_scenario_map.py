"""render_scenario_map.py
=========================
Ve "map tong hop" cua mot kich ban: nen mau phong xa + vat can (den) + target
(sao xanh, danh so). Mau duoc quyet dinh boi DUNG 2 nguong, trung khop voi
plot_path.py:

    xanh (low risk) : radiation < LOW_RISK_THRESHOLD (= 0.5)
    vang (medium)   : LOW_RISK_THRESHOLD <= radiation < RI_MAX
    do/hong (high)  : radiation >= RI_MAX (= 8.0)
    den             : vat can (de len tat ca)

Dung:
    python render_scenario_map.py                      # mac dinh scenario4
    python render_scenario_map.py --scenario scenario4
    python render_scenario_map.py --grid data/scenario4/scenario4_grid.txt \
                                  --radiation data/scenario4/radiation_grid.txt \
                                  --out results/scenario4/scenario4_map.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

# 2 NGUONG DUY NHAT quyet dinh mau (giong plot_path.py)
LOW_RISK_THRESHOLD = 0.5
RI_MAX = 8.0

OBSTACLE = 1
TARGET = 2

# Bang mau trung voi plot_path.py
CMAP = ListedColormap([
    "#222222",  # 0: obstacle - den
    "#87CEEB",  # 1: low risk - xanh (sky blue)
    "#FFFF66",  # 2: medium risk - vang
    "#FFB6B6",  # 3: high risk - hong/do nhat
])

ROOT = Path(__file__).resolve().parent


def load_grid(path: Path, dtype) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append([dtype(x) for x in line.split()])
    return np.array(rows, dtype=dtype)


def render(grid_path: Path, rad_path: Path, out_path: Path,
           show_axes: bool = True) -> None:
    obstacle_grid = load_grid(grid_path, int)
    radiation_grid = load_grid(rad_path, float)
    if obstacle_grid.shape != radiation_grid.shape:
        raise ValueError(
            f"Shape mismatch: grid {obstacle_grid.shape} vs "
            f"radiation {radiation_grid.shape}"
        )
    rows, cols = obstacle_grid.shape

    # Phan loai mau theo 2 nguong
    base = np.ones_like(obstacle_grid, dtype=int)        # mac dinh low (1)
    base[radiation_grid >= LOW_RISK_THRESHOLD] = 2       # medium
    base[radiation_grid >= RI_MAX] = 3                   # high
    base[obstacle_grid == OBSTACLE] = 0                  # obstacle de len tat ca

    fig_w = max(8, cols * 0.22)
    fig_h = max(8, rows * 0.22)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # origin="lower" -> y tang len tren, dung he toa do paper (goc duoi-trai)
    ax.imshow(base, cmap=CMAP, vmin=0, vmax=3, origin="lower",
              interpolation="nearest", extent=(0, cols, 0, rows))

    # Luoi o vuong
    ax.set_xticks(np.arange(0, cols + 1, 1), minor=True)
    ax.set_yticks(np.arange(0, rows + 1, 1), minor=True)
    ax.grid(which="minor", color="black", linewidth=0.4, alpha=0.4)

    # Target: sao xanh, danh so theo thu tu quet (giong plot_path.py)
    target_rc = np.argwhere(obstacle_grid == TARGET)
    for idx, (r, c) in enumerate(target_rc, start=1):
        x, y = c + 0.5, r + 0.5
        ax.scatter(x, y, marker="*", s=420, c="blue",
                   edgecolors="black", linewidths=0.8, zorder=5)
        ax.text(x + 0.6, y + 0.6, str(idx), color="blue",
                fontsize=11, fontweight="bold", zorder=6)

    if show_axes:
        ax.set_xticks(np.arange(0, cols + 1, 5))
        ax.set_yticks(np.arange(0, rows + 1, 5))
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.tick_params(which="major", length=4)
    else:
        ax.set_xticks([])
        ax.set_yticks([])

    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_aspect("equal")
    fig.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    non_obs = obstacle_grid != OBSTACLE
    n = int(non_obs.sum())
    low = int((non_obs & (radiation_grid < LOW_RISK_THRESHOLD)).sum())
    high = int((non_obs & (radiation_grid >= RI_MAX)).sum())
    med = n - low - high
    print(f"Rendered {grid_path.name}: {rows}x{cols}, "
          f"targets={len(target_rc)}, obstacles={int((obstacle_grid == OBSTACLE).sum())}")
    print(f"  color mix -> blue {low/n:.0%} | yellow {med/n:.0%} | red {high/n:.0%}")
    print(f"Saved -> {out_path}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Ve map tong hop cua mot kich ban.")
    ap.add_argument("--scenario", default="scenario4",
                    help="Ten thu muc trong data/ (mac dinh: scenario4)")
    ap.add_argument("--grid", default=None, help="Duong dan grid (.txt) ghi de --scenario")
    ap.add_argument("--radiation", default=None, help="Duong dan radiation (.txt)")
    ap.add_argument("--out", default=None, help="Duong dan PNG xuat ra")
    ap.add_argument("--no-axes", action="store_true", help="An truc toa do")
    args = ap.parse_args()

    if args.grid and args.radiation:
        grid_path = Path(args.grid)
        rad_path = Path(args.radiation)
    else:
        d = ROOT / "data" / args.scenario
        grid_path = d / f"{args.scenario}_grid.txt"
        rad_path = d / "radiation_grid.txt"

    out_path = (Path(args.out) if args.out
                else ROOT / "results" / args.scenario / f"{args.scenario}_map.png")

    render(grid_path, rad_path, out_path, show_axes=not args.no_axes)


if __name__ == "__main__":
    main()
