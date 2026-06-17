"""create_waypoint_map.py
==========================
Rai / dieu chinh waypoint (cac diem 2) tren mot obstacle map CO DINH dua tren
mot radiation map CO DINH. Script nay KHONG sinh lai phong xa — truong phong xa
duoc giu nguyen, chi co cac waypoint thay doi. Nho vay khi danh gia voi nhieu
cau hinh waypoint khac nhau, moi truong (vat can + phong xa) van dong nhat
(thi nghiem co kiem soat).

Quy uoc gia tri o luoi:  0 = trong, 1 = vat can, 2 = waypoint (target).

Logic:
  * desired < so waypoint hien co  -> bot waypoint (giu lai tap trai deu nhat).
  * desired > so waypoint hien co  -> giu cac waypoint hop le, them moi sao cho:
        - khong nam tren vung phong xa cao (radiation < RI_max * ratio),
        - lien thong voi cac waypoint con lai (cung 1 thanh phan passable),
        - trai deu (farthest-point sampling + minimum_waypoint_distance).
  * Waypoint cu dang nam tren vung high-risk / vat can / mat lien thong se bi
    loai va bao cao lai.
"""

from __future__ import annotations

import json
from collections import Counter, deque
from pathlib import Path

import numpy as np


OBSTACLE = 1
TARGET = 2


# All relative paths below are resolved from the directory containing this file.
CONFIG = {
    # --- Dau vao (bat buoc) ---
    # Obstacle map (co the chua cac diem 2 cu). Quy uoc: 0 trong, 1 vat can, 2 waypoint.
    "obstacle_map_path": "data/maps/factory400/factory400_30.txt",
    # Radiation map co dinh, cung kich thuoc voi obstacle map.
    "radiation_map_path": "data/maps/factory400/radiation_grid_30.txt",
    # So waypoint mong muon sau khi dieu chinh.
    "desired_waypoint_count": 40,

    # --- Dau ra ---
    "output_map_path": "data/maps/factory400/factory400_30_40.txt",
    "metadata_path": "data/maps/factory400/factory400_waypoints.json",

    # === Tham so de xuat them ===

    # Nguong high-risk — PHAI khop voi RI_max dung khi sinh radiation map.
    "RI_max": 8.0,
    # Waypoint chi duoc dat o cell co radiation < RI_max * ratio.
    #   1.0  -> chi can duoi nguong high-risk (giong rang buoc cu cua generator).
    #   <1.0 -> chua bien an toan, tranh dat waypoint sat mep vung high-risk.
    "waypoint_radiation_ratio": 1.0,

    # --- Phan vung mau theo radiation (khop voi plot_path.py) ---
    #   xanh (low / an toan): radiation < low_risk_threshold
    #   vang (medium)       : low_risk_threshold <= radiation < RI_max*ratio
    #   do   (high)         : radiation >= RI_max*ratio  -> TUYET DOI khong dat waypoint
    "low_risk_threshold": 0.5,
    # True: uu tien dat waypoint o vung xanh; chi dung vung vang khi xanh khong du.
    "prefer_low_risk": True,
    # Ty le toi da waypoint duoc phep nam o vung vang (soft cap, tinh tren desired_count).
    #   0.0 -> co gang KHONG dung vung vang;  1.0 -> khong gioi han.
    # Neu vung xanh khong du de dat het so luong yeu cau, script se vuot cap nay
    # (lap them o vung vang) va in canh bao — de uu tien dat du so waypoint.
    "max_medium_risk_ratio": 0.2,

    # Khoang cach Euclid toi thieu giua 2 waypoint (don vi o luoi) -> trai deu, tranh chum.
    # 0 = khong rang buoc cung (FPS van trai deu).
    "minimum_waypoint_distance": 5.0,
    # True : neu khong dam bao duoc khoang cach toi thieu thi DUNG (uu tien giãn cach).
    # False: van dat cho du so luong yeu cau, chi canh bao (uu tien dung so luong).
    "strict_spacing": False,

    # Khong dat waypoint trong dai 'border_margin' o sat bien map. 0 = tat.
    "border_margin": 0,

    # Khi desired < so waypoint hien co: cach chon tap con giu lai.
    #   "spread" = giu cac diem trai deu nhat;  "random" = chon ngau nhien.
    "removal_strategy": "spread",

    # Dinh nghia lien thong (giong GridMap cua project) khi xac dinh vung dat hop le.
    "allow_diagonal": True,
    "prevent_corner_cutting": True,

    # Seed co dinh -> tai lap duoc (dung cho removal random va tie-break).
    "random_seed": 42,
}


# =========================================================
# IO helpers
# =========================================================
def resolve_path(value: str) -> Path:
    """Resolve a config path relative to this script."""
    return Path(__file__).resolve().parent / value


def validate_config(config: dict) -> None:
    """Fail early when a config value is invalid."""
    if int(config["desired_waypoint_count"]) < 1:
        raise ValueError("desired_waypoint_count must be >= 1")
    if float(config["RI_max"]) <= 0:
        raise ValueError("RI_max must be positive")
    if not 0 < float(config["waypoint_radiation_ratio"]) <= 1:
        raise ValueError("waypoint_radiation_ratio must be in (0, 1]")
    red_threshold = float(config["RI_max"]) * float(config["waypoint_radiation_ratio"])
    if not 0 < float(config["low_risk_threshold"]) < red_threshold:
        raise ValueError(
            "low_risk_threshold must be in (0, RI_max*waypoint_radiation_ratio)"
        )
    if not 0 <= float(config["max_medium_risk_ratio"]) <= 1:
        raise ValueError("max_medium_risk_ratio must be in [0, 1]")
    if float(config["minimum_waypoint_distance"]) < 0:
        raise ValueError("minimum_waypoint_distance cannot be negative")
    if int(config["border_margin"]) < 0:
        raise ValueError("border_margin cannot be negative")
    if config["removal_strategy"] not in ("spread", "random"):
        raise ValueError("removal_strategy must be 'spread' or 'random'")


def load_obstacle_map(path: Path) -> np.ndarray:
    """Load and validate an obstacle/target grid."""
    grid = np.loadtxt(path, dtype=int)
    if grid.ndim != 2:
        raise ValueError(f"Obstacle map must be two-dimensional: {path}")
    invalid = set(np.unique(grid)) - {0, OBSTACLE, TARGET}
    if invalid:
        raise ValueError(
            f"Obstacle map contains unsupported values: {sorted(invalid)}"
        )
    return grid


def load_radiation_map(path: Path) -> np.ndarray:
    """Load a radiation grid (float)."""
    rad = np.loadtxt(path, dtype=float)
    if rad.ndim != 2:
        raise ValueError(f"Radiation map must be two-dimensional: {path}")
    return rad


# =========================================================
# Geometry / connectivity helpers
# =========================================================
def passable_mask(
    obstacle_grid: np.ndarray, radiation_grid: np.ndarray, threshold: float
) -> np.ndarray:
    """Cells the robot may stand on: not an obstacle and below the high-risk threshold."""
    return (obstacle_grid != OBSTACLE) & (radiation_grid < threshold)


def connected_components(
    passable: np.ndarray, allow_diagonal: bool, prevent_corner_cutting: bool
) -> tuple[np.ndarray, int]:
    """Label passable cells into connected components (BFS, project's adjacency rules)."""
    rows, cols = passable.shape
    labels = -np.ones((rows, cols), dtype=int)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if allow_diagonal:
        directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

    comp = 0
    for sr in range(rows):
        for sc in range(cols):
            if not passable[sr, sc] or labels[sr, sc] != -1:
                continue
            labels[sr, sc] = comp
            queue = deque([(sr, sc)])
            while queue:
                r, c = queue.popleft()
                for dr, dc in directions:
                    nr, nc = r + dr, c + dc
                    if not (0 <= nr < rows and 0 <= nc < cols):
                        continue
                    if not passable[nr, nc] or labels[nr, nc] != -1:
                        continue
                    if prevent_corner_cutting and dr != 0 and dc != 0:
                        if not passable[r + dr, c] or not passable[r, c + dc]:
                            continue
                    labels[nr, nc] = comp
                    queue.append((nr, nc))
            comp += 1
    return labels, comp


def largest_component(labels: np.ndarray, ncomp: int) -> int:
    best, best_size = -1, -1
    for c in range(ncomp):
        size = int(np.sum(labels == c))
        if size > best_size:
            best, best_size = c, size
    return best


def apply_border(mask: np.ndarray, margin: int) -> np.ndarray:
    """Disable a strip of `margin` cells along every edge."""
    if margin <= 0:
        return mask
    out = mask.copy()
    out[:margin, :] = False
    out[-margin:, :] = False
    out[:, :margin] = False
    out[:, -margin:] = False
    return out


# =========================================================
# Even placement (farthest-point sampling)
# =========================================================
def add_spread_points(
    candidate_positions,
    initial_chosen,
    k: int,
    min_dist: float,
    strict: bool,
) -> list[tuple[int, int]]:
    """Greedily pick `k` cells that maximise the minimum distance to already-chosen cells.

    candidate_positions : iterable of (row, col) eligible cells.
    initial_chosen      : (row, col) already fixed (new picks stay far from these too).
    Returns the list of newly added (row, col).
    """
    if k <= 0:
        return []
    cand = np.asarray(list(candidate_positions), dtype=float)
    if cand.size == 0:
        return []
    cand = cand.reshape(-1, 2)
    m = len(cand)

    # Distance from each candidate to the nearest already-chosen point.
    dist = np.full(m, np.inf)
    avail = np.ones(m, dtype=bool)
    for p in initial_chosen:
        d = np.hypot(cand[:, 0] - p[0], cand[:, 1] - p[1])
        dist = np.minimum(dist, d)
        avail &= ~((cand[:, 0] == p[0]) & (cand[:, 1] == p[1]))

    has_seed = len(list(initial_chosen)) > 0
    added: list[tuple[int, int]] = []
    for _ in range(k):
        if not avail.any():
            break
        if not has_seed and not added:
            # Seed the very first point near the centroid for an even start.
            centroid = cand[avail].mean(axis=0)
            seed_d = np.where(avail, np.hypot(cand[:, 0] - centroid[0],
                                              cand[:, 1] - centroid[1]), np.inf)
            idx = int(np.argmin(seed_d))
        else:
            masked = np.where(avail, dist, -np.inf)
            idx = int(np.argmax(masked))
            if dist[idx] < min_dist and strict:
                break
        p = cand[idx]
        added.append((int(p[0]), int(p[1])))
        avail[idx] = False
        dist = np.minimum(dist, np.hypot(cand[:, 0] - p[0], cand[:, 1] - p[1]))
    return added


def pick_subset(pool, initial_chosen, k, strategy, min_dist, strict, rng):
    """Choose k cells from `pool`: 'spread' = farthest-point, 'random' = uniform."""
    if k <= 0 or len(pool) == 0:
        return []
    if strategy == "random":
        idx = rng.choice(len(pool), size=min(k, len(pool)), replace=False)
        return [tuple(pool[int(i)]) for i in idx]
    return add_spread_points(pool, initial_chosen, k, min_dist, strict)


def min_pairwise_distance(points) -> float:
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return float("inf")
    best = float("inf")
    for i in range(len(pts)):
        d = np.hypot(pts[i + 1:, 0] - pts[i, 0], pts[i + 1:, 1] - pts[i, 1])
        if d.size:
            best = min(best, float(d.min()))
    return best


# =========================================================
# Core
# =========================================================
def build_waypoints(obstacle_grid, radiation_grid, config, rng):
    """Return (final_waypoints, info) given fixed obstacle + radiation grids."""
    if obstacle_grid.shape != radiation_grid.shape:
        raise ValueError(
            f"Shape mismatch: obstacle {obstacle_grid.shape} vs "
            f"radiation {radiation_grid.shape}"
        )

    red_thr = float(config["RI_max"]) * float(config["waypoint_radiation_ratio"])
    blue_thr = float(config["low_risk_threshold"])
    desired = int(config["desired_waypoint_count"])
    min_dist = float(config["minimum_waypoint_distance"])
    strict = bool(config["strict_spacing"])
    strategy = config["removal_strategy"]
    prefer_low = bool(config["prefer_low_risk"])
    yellow_cap = int(np.floor(float(config["max_medium_risk_ratio"]) * desired))

    # Vung passable = khong vat can & duoi nguong do (red). Waypoint TUYET DOI khong vao red.
    passable = passable_mask(obstacle_grid, radiation_grid, red_thr)
    labels, ncomp = connected_components(
        passable, config["allow_diagonal"], config["prevent_corner_cutting"]
    )
    if ncomp == 0:
        raise RuntimeError("Khong co o passable nao duoi nguong high-risk.")

    existing = [tuple(p) for p in np.argwhere(obstacle_grid == TARGET)]

    # Chon thanh phan lien thong chinh: chua nhieu waypoint cu nhat, neu khong thi lon nhat.
    comp_counts = Counter(labels[r, c] for (r, c) in existing if labels[r, c] != -1)
    main_comp = comp_counts.most_common(1)[0][0] if comp_counts else largest_component(labels, ncomp)
    eligible = apply_border(labels == main_comp, int(config["border_margin"]))

    # Phan vung xanh/vang trong tap eligible (red da bi loai khoi eligible).
    is_blue = radiation_grid < blue_thr
    if prefer_low:
        blue_mask = eligible & is_blue
        yellow_mask = eligible & ~is_blue
    else:
        # Khong uu tien -> coi toan bo eligible nhu mot vung, khong gioi han vang.
        blue_mask = eligible
        yellow_mask = np.zeros_like(eligible)
        yellow_cap = desired

    valid_existing = [p for p in existing if eligible[p]]
    invalid_existing = [p for p in existing if not eligible[p]]
    blue_existing = [p for p in valid_existing if blue_mask[p]]
    yellow_existing = [p for p in valid_existing if yellow_mask[p]]

    def is_yellow(p):
        return bool(yellow_mask[p])

    if desired <= len(valid_existing):
        # --- Bot waypoint: giu vung xanh truoc, bo vung vang truoc ---
        keep_blue = pick_subset(blue_existing, [], min(desired, len(blue_existing)),
                                strategy, min_dist, False, rng)
        remaining = desired - len(keep_blue)
        keep_yellow = pick_subset(yellow_existing, keep_blue, remaining,
                                  strategy, min_dist, False, rng)
        final = list(keep_blue) + list(keep_yellow)
        added, removed = [], [p for p in valid_existing if p not in set(final)]
        shortfall = desired - len(final)
    else:
        # --- Them waypoint: lap vung xanh truoc, vung vang chi khi can ---
        existing_set = set(valid_existing)
        blue_pool = [tuple(p) for p in np.argwhere(blue_mask) if tuple(p) not in existing_set]
        yellow_pool = [tuple(p) for p in np.argwhere(yellow_mask) if tuple(p) not in existing_set]

        need = desired - len(valid_existing)
        # Pass 1: vung xanh
        new_blue = add_spread_points(blue_pool, valid_existing, need, min_dist, strict)
        chosen = list(valid_existing) + new_blue
        remaining = need - len(new_blue)

        # Pass 2: vung vang, uu tien khong vuot yellow_cap
        yellow_room = max(0, yellow_cap - len(yellow_existing))
        new_yellow = []
        if remaining > 0:
            take = min(remaining, yellow_room)
            new_yellow = add_spread_points(yellow_pool, chosen, take, min_dist, strict)
            chosen += new_yellow
            remaining -= len(new_yellow)
            # Pass 3 (fallback): neu van thieu, vuot yellow_cap de dat du so luong.
            if remaining > 0:
                extra = add_spread_points(yellow_pool, chosen, remaining, min_dist, strict)
                new_yellow += extra
                chosen += extra
                remaining -= len(extra)

        added = list(new_blue) + list(new_yellow)
        final = list(valid_existing) + added
        removed = []
        shortfall = remaining

    final_yellow = sum(1 for p in final if is_yellow(p))
    info = {
        "red_threshold": red_thr,
        "blue_threshold": blue_thr,
        "main_component": int(main_comp),
        "main_component_size": int(np.sum(labels == main_comp)),
        "existing_count": len(existing),
        "valid_existing_count": len(valid_existing),
        "invalid_existing": invalid_existing,
        "added": added,
        "removed": removed,
        "final_count": len(final),
        "final_blue_count": len(final) - final_yellow,
        "final_yellow_count": final_yellow,
        "yellow_cap": yellow_cap,
        "shortfall": shortfall,
        "min_pairwise_distance": min_pairwise_distance(final),
    }
    return final, info


def write_output(obstacle_grid, final_waypoints, info, config) -> None:
    out = obstacle_grid.copy()
    out[out == TARGET] = 0
    for (r, c) in final_waypoints:
        out[r, c] = TARGET

    output_path = resolve_path(config["output_map_path"])
    metadata_path = resolve_path(config["metadata_path"])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.parent.mkdir(parents=True, exist_ok=True)

    np.savetxt(output_path, out, fmt="%d")

    metadata = {
        "obstacle_map_path": config["obstacle_map_path"],
        "radiation_map_path": config["radiation_map_path"],
        "desired_waypoint_count": int(config["desired_waypoint_count"]),
        "final_waypoints": [[int(r), int(c)] for (r, c) in final_waypoints],
        "info": {
            "red_threshold": info["red_threshold"],
            "blue_threshold": info["blue_threshold"],
            "main_component": info["main_component"],
            "main_component_size": info["main_component_size"],
            "existing_count": info["existing_count"],
            "valid_existing_count": info["valid_existing_count"],
            "invalid_existing": [[int(r), int(c)] for (r, c) in info["invalid_existing"]],
            "added": [[int(r), int(c)] for (r, c) in info["added"]],
            "removed": [[int(r), int(c)] for (r, c) in info["removed"]],
            "final_count": info["final_count"],
            "final_blue_count": info["final_blue_count"],
            "final_yellow_count": info["final_yellow_count"],
            "yellow_cap": info["yellow_cap"],
            "shortfall": info["shortfall"],
            "min_pairwise_distance": info["min_pairwise_distance"],
        },
        "config": config,
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8"
    )


def main() -> None:
    validate_config(CONFIG)
    rng = np.random.default_rng(int(CONFIG["random_seed"]))

    obstacle_grid = load_obstacle_map(resolve_path(CONFIG["obstacle_map_path"]))
    radiation_grid = load_radiation_map(resolve_path(CONFIG["radiation_map_path"]))

    final, info = build_waypoints(obstacle_grid, radiation_grid, CONFIG, rng)
    write_output(obstacle_grid, final, info, CONFIG)

    print(f"Obstacle map : {resolve_path(CONFIG['obstacle_map_path'])}")
    print(f"Radiation map: {resolve_path(CONFIG['radiation_map_path'])}")
    print(f"Nguong do/high-risk (RI_max*ratio): {info['red_threshold']:.4f}  | "
          f"nguong xanh (low): {info['blue_threshold']:.4f}")
    print(f"Thanh phan lien thong chinh: #{info['main_component']} "
          f"({info['main_component_size']} o)")
    print(f"Waypoint cu: {info['existing_count']} "
          f"(hop le: {info['valid_existing_count']})")
    if info["invalid_existing"]:
        print(f"  [CANH BAO] {len(info['invalid_existing'])} waypoint cu bi loai "
              f"(tren vung high-risk / vat can / mat lien thong): {info['invalid_existing']}")
    if info["added"]:
        print(f"Them moi: {len(info['added'])} waypoint")
    if info["removed"]:
        print(f"Bot di : {len(info['removed'])} waypoint")
    print(f"Phan bo vung: xanh={info['final_blue_count']}  "
          f"vang={info['final_yellow_count']}  (cap vang ~ {info['yellow_cap']})  do=0")
    if info["final_yellow_count"] > info["yellow_cap"]:
        print(f"  [CANH BAO] So waypoint vung vang ({info['final_yellow_count']}) vuot cap "
              f"({info['yellow_cap']}) vi vung xanh khong du. "
              f"Tang map / giam minimum_waypoint_distance hoac noi long max_medium_risk_ratio.")
    if info["shortfall"] > 0:
        print(f"  [CANH BAO] Thieu {info['shortfall']} waypoint — khong con o hop le. "
              f"Giam minimum_waypoint_distance / border_margin, hoac tang nguong.")
    md = info["min_pairwise_distance"]
    if md != float("inf"):
        print(f"Khoang cach nho nhat giua 2 waypoint: {md:.2f} "
              f"(yeu cau >= {CONFIG['minimum_waypoint_distance']})")
        if md < float(CONFIG["minimum_waypoint_distance"]):
            print("  [CANH BAO] Khong dat duoc khoang cach toi thieu "
                  "(strict_spacing=False da uu tien du so luong).")
    print(f"Tong waypoint cuoi cung: {info['final_count']}")
    print(f"Da luu map -> {resolve_path(CONFIG['output_map_path'])}")
    print(f"Metadata    -> {resolve_path(CONFIG['metadata_path'])}")


if __name__ == "__main__":
    main()
