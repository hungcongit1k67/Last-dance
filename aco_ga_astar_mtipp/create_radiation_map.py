"""Generate a synthetic radiation grid for the ACO-GA-A* MTIPP project.

The radiation field follows the point-source approximation described in the
paper: contributions from all sources are accumulated and each contribution
decreases proportionally to 1 / r^2.

This is a synthetic approximation. It does not replace a Monte Carlo radiation
transport simulation that accounts for shielding, absorption, and scattering.
"""

from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path

import numpy as np


# All relative paths below are resolved from the directory containing this file.
CONFIG = {
    # Duong dan den obstacle map. Quy uoc cua project:
    # 0 = o trong, 1 = vat can, 2 = target.
    #"obstacle_map_path": "data/maps/factory400/factory400_30.txt",
    #"obstacle_map_path": "data/maps/mixed500/mixed500.txt",
    #"obstacle_map_path": "data/maps/triangle300/triangle300.txt",
    #"obstacle_map_path": "data/maps/mixed200/mixed2002.txt",
    #"obstacle_map_path": "data/warehouse4/warehouse4.txt",
    "obstacle_map_path": "data/maps/scenario_medium/obstacle_grid.txt",

    # Noi luu radiation grid sau khi sinh. File nay co cung kich thuoc voi
    # obstacle map va co the gan truc tiep vao radiation_file trong YAML.
    "output_path": "data/maps/generated_radiation_grid.txt",

    # File JSON luu vi tri, cuong do tung nguon va thong ke cua map. File nay
    # giup kiem tra, giai thich va tai tao thi nghiem.
    "source_metadata_path": "data/maps/generated_radiation_sources.json",

    # Nguong suat lieu toi da robot co the chiu. O co radiation >= RI_max
    # duoc xem la high-risk va Modified A* khong duoc di qua.
    "RI_max": 8.0,

    # Ty le o nguon tren tong so o khong phai vat can va khong phai target.
    # Map lon rat nhay voi tham so nay vi duoi 1/r^2 cua nhieu nguon se cong
    # chong tren toan map. Gia tri 0.002 tuong ung khoang 0.2% o hop le.
    "source_cell_ratio": 0.0015,

    # Seed co dinh de cung config sinh ra cung mot radiation map.
    "random_seed": 42,

    # Cuong do dinh cua moi nguon duoc lay ngau nhien theo RI_max:
    # source_peak = uniform(min_ratio, max_ratio) * RI_max.
    # min_ratio > 1 dam bao rieng o tam nguon da vuot RI_max.
    "source_peak_min_ratio": 1.2,
    "source_peak_max_ratio": 2.5,

    # Khoang cach Euclid toi thieu giua hai o nguon, tinh theo don vi o luoi.
    # Gia tri lon lam cac nguon phan bo deu hon va it bi chong len nhau.
    "minimum_source_distance": 5.0,

    # Khoang cach toi thieu tu nguon den target. Day chi la dieu kien khi dat
    # nguon; script van kiem tra tong radiation tai target sau khi cong nguon.
    "minimum_target_distance": 3.0,

    # Khoang cach nho nhat dung trong mau so cua cong thuc 1/r^2:
    # contribution = source_peak / max(r, distance_epsilon)^2.
    # Tham so nay tranh chia cho 0 tai chinh tam nguon.
    "distance_epsilon": 1.0,

    # Gioi han radiation cuoi cung theo boi so cua RI_max, tranh cac vung
    # nhieu nguon chong lan tao gia tri qua lon.
    "max_radiation_ratio": 4.0,

    # Radiation map chi duoc chap nhan neu ty le o high-risk nam trong khoang
    # nay. Ty le duoc tinh tren tat ca o khong phai vat can.
    "minimum_high_risk_ratio": 0.01,
    "maximum_high_risk_ratio": 0.15,

    # Nguong tach low-risk (mau xanh) va medium-risk (mau vang). Paper khong
    # quy dinh nguong nay; 0.5 la quy uoc dang duoc plot_path.py su dung.
    "low_risk_threshold": 0.5,

    # Radiation map chi duoc chap nhan neu ty le o low-risk nam trong khoang
    # nay. Rang buoc nay ngan truong hop duoi cua qua nhieu nguon cong chong
    # lam toan bo map thanh medium-risk mau vang.
    "minimum_low_risk_ratio": 0.40,
    "maximum_low_risk_ratio": 0.80,

    # So lan sinh lai toi da neu target bi high-risk, target mat lien thong,
    # hoac ty le o high-risk nam ngoai khoang yeu cau.
    "max_generation_attempts": 200,

    # Cho phep kiem tra lien thong target theo 8 huong nhu GridMap cua project.
    "allow_diagonal": True,

    # Neu True, buoc cheo khong duoc cat qua goc cua vat can/vung high-risk.
    "prevent_corner_cutting": True,
}


OBSTACLE = 1
TARGET = 2


def resolve_path(value: str) -> Path:
    """Resolve a config path relative to this script."""
    return Path(__file__).resolve().parent / value


def validate_config(config: dict) -> None:
    """Fail early when a config value would produce an invalid map."""
    ri_max = float(config["RI_max"])
    source_ratio = float(config["source_cell_ratio"])
    min_peak = float(config["source_peak_min_ratio"])
    max_peak = float(config["source_peak_max_ratio"])
    min_high = float(config["minimum_high_risk_ratio"])
    max_high = float(config["maximum_high_risk_ratio"])
    low_threshold = float(config["low_risk_threshold"])
    min_low = float(config["minimum_low_risk_ratio"])
    max_low = float(config["maximum_low_risk_ratio"])

    if ri_max <= 0:
        raise ValueError("RI_max must be positive")
    if not 0 < source_ratio <= 1:
        raise ValueError("source_cell_ratio must be in (0, 1]")
    if not 1 < min_peak <= max_peak:
        raise ValueError(
            "source_peak ratios must satisfy 1 < min_ratio <= max_ratio"
        )
    if float(config["minimum_source_distance"]) < 0:
        raise ValueError("minimum_source_distance cannot be negative")
    if float(config["minimum_target_distance"]) < 0:
        raise ValueError("minimum_target_distance cannot be negative")
    if float(config["distance_epsilon"]) <= 0:
        raise ValueError("distance_epsilon must be positive")
    if float(config["max_radiation_ratio"]) <= 1:
        raise ValueError("max_radiation_ratio must be greater than 1")
    if not 0 < min_high <= max_high < 1:
        raise ValueError(
            "high-risk ratios must satisfy 0 < minimum <= maximum < 1"
        )
    if not 0 < low_threshold < ri_max:
        raise ValueError("low_risk_threshold must be between 0 and RI_max")
    if not 0 <= min_low <= max_low <= 1:
        raise ValueError(
            "low-risk ratios must satisfy 0 <= minimum <= maximum <= 1"
        )
    if int(config["max_generation_attempts"]) <= 0:
        raise ValueError("max_generation_attempts must be positive")


def load_obstacle_map(path: Path) -> np.ndarray:
    """Load and validate an obstacle/target grid."""
    obstacle_grid = np.loadtxt(path, dtype=int)
    if obstacle_grid.ndim != 2:
        raise ValueError(f"Obstacle map must be two-dimensional: {path}")

    invalid_values = set(np.unique(obstacle_grid)) - {0, OBSTACLE, TARGET}
    if invalid_values:
        raise ValueError(
            f"Obstacle map contains unsupported values: {sorted(invalid_values)}"
        )
    return obstacle_grid


def squared_distances_to(
    row_grid: np.ndarray, col_grid: np.ndarray, row: int, col: int
) -> np.ndarray:
    """Return squared Euclidean distance from every cell to one source."""
    return (row_grid - row) ** 2 + (col_grid - col) ** 2


def choose_sources(
    obstacle_grid: np.ndarray, config: dict, rng: np.random.Generator
) -> list[dict]:
    """Randomly place separated sources away from obstacles and targets."""
    source_candidates = np.argwhere(obstacle_grid == 0)
    target_positions = np.argwhere(obstacle_grid == TARGET)
    desired_count = max(
        1,
        round(len(source_candidates) * float(config["source_cell_ratio"])),
    )

    rng.shuffle(source_candidates)
    selected: list[dict] = []
    min_source_distance = float(config["minimum_source_distance"])
    min_target_distance = float(config["minimum_target_distance"])
    ri_max = float(config["RI_max"])

    for row, col in source_candidates:
        row = int(row)
        col = int(col)

        if any(
            math.hypot(row - source["row"], col - source["col"])
            < min_source_distance
            for source in selected
        ):
            continue

        if len(target_positions) and np.any(
            np.hypot(
                target_positions[:, 0] - row,
                target_positions[:, 1] - col,
            )
            < min_target_distance
        ):
            continue

        peak_ratio = rng.uniform(
            float(config["source_peak_min_ratio"]),
            float(config["source_peak_max_ratio"]),
        )
        selected.append(
            {
                "row": row,
                "col": col,
                "peak_radiation": float(peak_ratio * ri_max),
            }
        )
        if len(selected) == desired_count:
            return selected

    raise ValueError(
        f"Could only place {len(selected)} of {desired_count} requested sources. "
        "Reduce source_cell_ratio/minimum distances or use a larger map."
    )


def build_radiation_grid(
    obstacle_grid: np.ndarray, sources: list[dict], config: dict
) -> np.ndarray:
    """Accumulate the inverse-square radiation contribution of all sources."""
    rows, cols = obstacle_grid.shape
    row_grid, col_grid = np.indices((rows, cols))
    radiation_grid = np.zeros((rows, cols), dtype=float)
    epsilon_squared = float(config["distance_epsilon"]) ** 2

    for source in sources:
        distance_squared = squared_distances_to(
            row_grid, col_grid, source["row"], source["col"]
        )
        radiation_grid += source["peak_radiation"] / np.maximum(
            distance_squared, epsilon_squared
        )

    max_radiation = (
        float(config["max_radiation_ratio"]) * float(config["RI_max"])
    )
    return np.clip(radiation_grid, 0.0, max_radiation)


def targets_are_connected(
    obstacle_grid: np.ndarray, radiation_grid: np.ndarray, config: dict
) -> bool:
    """Check that every target remains reachable outside high-risk cells."""
    targets = [tuple(pos) for pos in np.argwhere(obstacle_grid == TARGET)]
    if len(targets) <= 1:
        return True

    passable = (obstacle_grid != OBSTACLE) & (
        radiation_grid < float(config["RI_max"])
    )
    if any(not passable[target] for target in targets):
        return False

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if config["allow_diagonal"]:
        directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

    queue = deque([targets[0]])
    visited = {targets[0]}
    rows, cols = obstacle_grid.shape

    while queue:
        row, col = queue.popleft()
        for dr, dc in directions:
            next_row, next_col = row + dr, col + dc
            if not (0 <= next_row < rows and 0 <= next_col < cols):
                continue
            if not passable[next_row, next_col]:
                continue
            if config["prevent_corner_cutting"] and dr != 0 and dc != 0:
                if not passable[row + dr, col] or not passable[row, col + dc]:
                    continue
            neighbor = (next_row, next_col)
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)

    return all(target in visited for target in targets)


def map_statistics(
    obstacle_grid: np.ndarray, radiation_grid: np.ndarray, config: dict
) -> dict:
    """Calculate the acceptance metrics used by the generator."""
    non_obstacle = obstacle_grid != OBSTACLE
    high_risk = non_obstacle & (radiation_grid >= float(config["RI_max"]))
    low_risk = non_obstacle & (
        radiation_grid < float(config["low_risk_threshold"])
    )
    medium_risk = non_obstacle & ~low_risk & ~high_risk
    target_values = radiation_grid[obstacle_grid == TARGET]
    non_obstacle_values = radiation_grid[non_obstacle]

    return {
        "rows": int(obstacle_grid.shape[0]),
        "cols": int(obstacle_grid.shape[1]),
        "minimum_radiation": float(non_obstacle_values.min()),
        "maximum_radiation": float(non_obstacle_values.max()),
        "mean_radiation": float(non_obstacle_values.mean()),
        "high_risk_cell_count": int(high_risk.sum()),
        "high_risk_ratio": float(high_risk.sum() / non_obstacle.sum()),
        "medium_risk_cell_count": int(medium_risk.sum()),
        "medium_risk_ratio": float(medium_risk.sum() / non_obstacle.sum()),
        "low_risk_cell_count": int(low_risk.sum()),
        "low_risk_ratio": float(low_risk.sum() / non_obstacle.sum()),
        "target_count": int(len(target_values)),
        "maximum_target_radiation": (
            float(target_values.max()) if len(target_values) else None
        ),
    }


def generate_acceptable_map(
    obstacle_grid: np.ndarray, config: dict
) -> tuple[np.ndarray, list[dict], dict, int]:
    """Generate until high-risk ratio and target safety constraints are met."""
    seed = int(config["random_seed"])
    min_high = float(config["minimum_high_risk_ratio"])
    max_high = float(config["maximum_high_risk_ratio"])
    min_low = float(config["minimum_low_risk_ratio"])
    max_low = float(config["maximum_low_risk_ratio"])

    last_reason = "no attempt was made"
    for attempt in range(1, int(config["max_generation_attempts"]) + 1):
        # A deterministic but distinct RNG stream is used for each attempt.
        rng = np.random.default_rng(seed + attempt - 1)
        sources = choose_sources(obstacle_grid, config, rng)
        radiation_grid = build_radiation_grid(obstacle_grid, sources, config)
        statistics = map_statistics(obstacle_grid, radiation_grid, config)

        if not min_high <= statistics["high_risk_ratio"] <= max_high:
            last_reason = (
                f"high-risk ratio {statistics['high_risk_ratio']:.4f} is outside "
                f"[{min_high:.4f}, {max_high:.4f}]"
            )
            continue

        if not min_low <= statistics["low_risk_ratio"] <= max_low:
            last_reason = (
                f"low-risk ratio {statistics['low_risk_ratio']:.4f} is outside "
                f"[{min_low:.4f}, {max_low:.4f}]"
            )
            continue

        if statistics["maximum_target_radiation"] is not None and (
            statistics["maximum_target_radiation"] >= float(config["RI_max"])
        ):
            last_reason = "at least one target is in a high-risk cell"
            continue

        if not targets_are_connected(obstacle_grid, radiation_grid, config):
            last_reason = "targets are not mutually reachable"
            continue

        return radiation_grid, sources, statistics, attempt

    raise RuntimeError(
        "Could not generate an acceptable radiation map after "
        f"{config['max_generation_attempts']} attempts; last rejection: "
        f"{last_reason}. Adjust source ratio, source strength, or risk ratios."
    )


def save_outputs(
    radiation_grid: np.ndarray,
    sources: list[dict],
    statistics: dict,
    attempt: int,
    config: dict,
) -> None:
    """Save the generated grid and source metadata."""
    output_path = resolve_path(config["output_path"])
    metadata_path = resolve_path(config["source_metadata_path"])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.parent.mkdir(parents=True, exist_ok=True)

    np.savetxt(output_path, radiation_grid, fmt="%.6f")

    metadata = {
        "model": "superposition of gamma point sources with inverse-square decay",
        "formula": "R(x,y) = sum_k peak_k / max(distance_k, epsilon)^2",
        "accepted_attempt": attempt,
        "sources": sources,
        "statistics": statistics,
        "config": config,
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def main() -> None:
    validate_config(CONFIG)
    obstacle_path = resolve_path(CONFIG["obstacle_map_path"])
    obstacle_grid = load_obstacle_map(obstacle_path)

    radiation_grid, sources, statistics, attempt = generate_acceptable_map(
        obstacle_grid, CONFIG
    )
    save_outputs(radiation_grid, sources, statistics, attempt, CONFIG)

    print(f"Generated radiation map after {attempt} attempt(s).")
    print(f"Sources: {len(sources)}")
    print(
        "High-risk cells: "
        f"{statistics['high_risk_cell_count']} "
        f"({statistics['high_risk_ratio']:.2%})"
    )
    print(
        "Medium-risk cells: "
        f"{statistics['medium_risk_cell_count']} "
        f"({statistics['medium_risk_ratio']:.2%})"
    )
    print(
        "Low-risk cells: "
        f"{statistics['low_risk_cell_count']} "
        f"({statistics['low_risk_ratio']:.2%})"
    )
    print(
        "Radiation range: "
        f"{statistics['minimum_radiation']:.6f} - "
        f"{statistics['maximum_radiation']:.6f}"
    )
    print(f"Radiation map: {resolve_path(CONFIG['output_path'])}")
    print(f"Source metadata: {resolve_path(CONFIG['source_metadata_path'])}")


if __name__ == "__main__":
    main()
