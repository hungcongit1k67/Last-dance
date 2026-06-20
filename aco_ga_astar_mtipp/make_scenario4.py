"""make_scenario4.py
====================
Sinh lai mot cach TAI LAP DUOC bo du lieu scenario 4 (40x40) cho dung voi
bai bao goc (Zhang et al., 2025, Fig. 10(d) / Fig. 21).

Ba thanh phan:
  1. Truong phong xa (radiation_grid.txt) - sinh tu 16 NGUON DIEM CHINH XAC
     lay tu Table 6 cua bai bao, theo mo hinh suy giam 1/r^2 (Eq. 1).
  2. 20 inspection target  - doc gan dung tu Fig. 10(d)/21 (anh do phan giai
     thap nen chi la xap xi; generator se tu day target ra khoi vung do/vat can).
  3. Cac khoi vat can THUA (sparse) - doc gan dung tu hinh.

Mau hien thi do plot_path.py / render_scenario_map.py quyet dinh boi DUNG 2
nguong:  low_risk_threshold = 0.5 (xanh < 0.5 <= vang) va RI_max = 8.0
(>= 8.0 -> do/hong, high-risk). Vi vay chi can dieu chinh BASE_PEAK ben duoi
de mat do mau xanh/vang/do khop voi paper.

Quy uoc o luoi:  0 = trong, 1 = vat can, 2 = target.
Toa do paper la (x, y) tinh tu goc DUOI-TRAI; o luoi tinh theo (row, col) voi
row 0 o tren cung. Quy doi:  col = floor(x),  row = (R-1) - floor(y).
"""

from __future__ import annotations

import math
from collections import deque
from pathlib import Path

import numpy as np

# --------------------------------------------------------------------------
# Tham so quyet dinh MAU (phai trung voi plot_path.py / render_scenario_map.py)
# --------------------------------------------------------------------------
RI_MAX = 8.0               # >= 8.0 -> do (high-risk)
LOW_RISK_THRESHOLD = 0.5   # < 0.5 -> xanh; [0.5, 8.0) -> vang

# Cuong do dinh cua mot nguon 1.25 MeV (tai o ke nguon, r=1). Nguon 2.5/3.75 MeV
# duoc nhan ty le theo nang luong. Tang -> nhieu/lan rong vung do; giam -> it do hon.
BASE_PEAK = 18.0
# Chieu dai suy giam (so o luoi): mo phong suy giam theo khoang cach/khong khi
# (Monte Carlo, Section 3.2) -> vung xa nguon tat dan ve 0 (mau xanh o ria/goc).
# Nho -> nhieu xanh hon; lon -> it xanh hon (gan voi 1/r^2 thuan).
ATTEN_LAMBDA = 7.0
MAX_RADIATION = 4.0 * RI_MAX   # chan tran de vung chong lan nhieu nguon khong qua lon

GRID_ROWS = 40
GRID_COLS = 40

OBSTACLE = 1
TARGET = 2

OUT_DIR = Path(__file__).resolve().parent / "data" / "scenario4"

# --------------------------------------------------------------------------
# 16 nguon phong xa - TOA DO CHINH XAC tu Table 6 (kich ban 40x40).
# (x, y, nang_luong_MeV). Hoat do 1.12 GBq cho tat ca (khong anh huong hinh dang).
# --------------------------------------------------------------------------
SOURCES_XY = [
    (11.5, 35.5, 1.25),
    (34.5, 34.5, 1.25),
    (5.5, 30.5, 1.25),
    (22.5, 29.5, 2.5),
    (31.5, 29.5, 1.25),
    (12.5, 24.5, 1.25),
    (26.5, 25.5, 1.25),
    (17.5, 19.5, 2.5),
    (6.5, 16.5, 1.25),
    (33.5, 15.5, 1.25),
    (12.5, 12.5, 1.25),
    (32.5, 10.5, 1.25),
    (24.5, 8.5, 3.75),
    (7.5, 5.5, 1.25),
    (18.5, 5.5, 1.25),
    (34.5, 5.5, 1.25),
]

# --------------------------------------------------------------------------
# 20 inspection target - doc gan dung tu Fig. 10(d)/21 (paper x,y).
# --------------------------------------------------------------------------
TARGETS_XY = [
    (3, 34), (13, 37), (22, 38), (37, 35),
    (8, 27), (19, 27), (29, 28), (38, 23),
    (4, 20), (15, 17), (25, 18), (34, 19),
    (3, 12), (20, 13), (30, 12), (38, 9),
    (8, 8), (15, 4), (24, 3), (33, 5),
]

# --------------------------------------------------------------------------
# Cac khoi vat can THUA - doc gan dung tu hinh (paper x,y, inclusive).
# Moi phan tu: (x0, x1, y0, y1).
# --------------------------------------------------------------------------
OBSTACLE_RECTS = [
    (17, 19, 36, 38),   # top-center
    (20, 22, 32, 33),   # center-top
    (6, 8, 28, 30),     # left
    (30, 33, 27, 29),   # right-upper
    (37, 38, 24, 26),   # right edge
    (22, 24, 20, 22),   # center
    (30, 32, 16, 18),   # right
    (15, 17, 9, 11),    # lower-left-center
    (16, 18, 1, 3),     # bottom-center (tranh nguon 3.75 MeV tai (24.5, 8.5))
    (30, 32, 8, 10),    # bottom-right
]


def to_rc(x: float, y: float) -> tuple[int, int]:
    """Quy doi toa do paper (x, y) -> (row, col) cua o luoi."""
    col = int(math.floor(x))
    row = (GRID_ROWS - 1) - int(math.floor(y))
    return row, col


def build_radiation(obstacle_grid: np.ndarray) -> np.ndarray:
    row_idx, col_idx = np.indices((GRID_ROWS, GRID_COLS))
    rad = np.zeros((GRID_ROWS, GRID_COLS), dtype=float)
    for x, y, energy in SOURCES_XY:
        sr, sc = to_rc(x, y)
        dist2 = (row_idx - sr) ** 2 + (col_idx - sc) ** 2
        dist = np.sqrt(dist2)
        peak = BASE_PEAK * (energy / 1.25)
        rad += peak * np.exp(-dist / ATTEN_LAMBDA) / np.maximum(dist2, 1.0)
    return np.clip(rad, 0.0, MAX_RADIATION)


def passable_mask(obstacle_grid: np.ndarray, rad: np.ndarray) -> np.ndarray:
    return (obstacle_grid != OBSTACLE) & (rad < RI_MAX)


def nearest_passable(rc, passable: np.ndarray) -> tuple[int, int]:
    """BFS tim o passable gan nhat (de day target ra khoi do/vat can)."""
    if passable[rc]:
        return rc
    seen = {rc}
    q = deque([rc])
    while q:
        r, c = q.popleft()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if not (0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS):
                    continue
                if (nr, nc) in seen:
                    continue
                seen.add((nr, nc))
                if passable[nr, nc]:
                    return (nr, nc)
                q.append((nr, nc))
    raise RuntimeError("Khong tim duoc o passable nao gan target.")


def connected(targets, passable) -> bool:
    start = targets[0]
    seen = {start}
    q = deque([start])
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while q:
        r, c = q.popleft()
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS):
                continue
            if (nr, nc) in seen or not passable[nr, nc]:
                continue
            if dr != 0 and dc != 0:  # cam cat goc
                if not passable[r + dr, c] or not passable[r, c + dc]:
                    continue
            seen.add((nr, nc))
            q.append((nr, nc))
    return all(t in seen for t in targets)


def main() -> None:
    obstacle_grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)

    # 1) Vat can
    for x0, x1, y0, y1 in OBSTACLE_RECTS:
        r_top, c0 = to_rc(x0, y1)    # y lon -> row nho (tren cung)
        r_bot, c1 = to_rc(x1, y0)    # y nho -> row lon (duoi cung)
        obstacle_grid[r_top:r_bot + 1, c0:c1 + 1] = OBSTACLE

    # 2) Phong xa
    rad = build_radiation(obstacle_grid)
    passable = passable_mask(obstacle_grid, rad)

    # 3) Target: day moi target ra khoi vung do/vat can (neu can)
    placed: list[tuple[int, int]] = []
    moved = 0
    for x, y in TARGETS_XY:
        rc = to_rc(x, y)
        fixed = nearest_passable(rc, passable)
        if fixed != rc:
            moved += 1
        # tranh trung o
        while fixed in placed:
            fixed = nearest_passable((fixed[0], fixed[1] + 1), passable)
        placed.append(fixed)
        obstacle_grid[fixed] = TARGET

    # Kiem tra lien thong
    ok = connected(placed, passable_mask(obstacle_grid, rad) | (obstacle_grid == TARGET))

    # 4) Thong ke mau
    non_obs = obstacle_grid != OBSTACLE
    low = non_obs & (rad < LOW_RISK_THRESHOLD)
    high = non_obs & (rad >= RI_MAX)
    med = non_obs & ~low & ~high
    n = int(non_obs.sum())

    # 5) Ghi file
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    np.savetxt(OUT_DIR / "scenario4_grid.txt", obstacle_grid, fmt="%d")
    np.savetxt(OUT_DIR / "radiation_grid.txt", rad, fmt="%.4f")

    print(f"Grid {GRID_ROWS}x{GRID_COLS} | sources={len(SOURCES_XY)} "
          f"targets={len(placed)} obstacles={int((obstacle_grid == OBSTACLE).sum())}")
    print(f"Targets relocated off red/obstacle: {moved}")
    print(f"Connectivity of all targets: {'OK' if ok else 'FAIL'}")
    print(f"BASE_PEAK={BASE_PEAK}  rad range=[{rad[non_obs].min():.3f}, {rad[non_obs].max():.3f}]")
    print(f"  blue (<0.5)      : {int(low.sum()):4d}  ({low.sum()/n:.1%})")
    print(f"  yellow (0.5-8)   : {int(med.sum()):4d}  ({med.sum()/n:.1%})")
    print(f"  red (>=8)        : {int(high.sum()):4d}  ({high.sum()/n:.1%})")
    print(f"Saved -> {OUT_DIR/'scenario4_grid.txt'}")
    print(f"Saved -> {OUT_DIR/'radiation_grid.txt'}")


if __name__ == "__main__":
    main()
