from __future__ import annotations

import json
from pathlib import Path
from typing import List

import numpy as np

from src.core.grid_map import GridMap
from src.core.target import Target


def load_txt_grid(path: str | Path, dtype=float) -> np.ndarray:
    """Load a whitespace-separated txt grid."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                rows.append([dtype(x) for x in line.split()])
            except ValueError as exc:
                raise ValueError(f"Invalid value in {path} at line {line_no}") from exc
    if not rows:
        raise ValueError(f"No grid data found in {path}")
    width = len(rows[0])
    if any(len(row) != width for row in rows):
        raise ValueError(f"Grid file {path} has inconsistent row lengths")
    return np.array(rows, dtype=dtype)


def load_targets_json(path: str | Path) -> List[Target]:
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        raw = json.load(f)
    targets = [Target(id=str(item.get("id", f"T{i+1}")), row=int(item["row"]), col=int(item["col"])) for i, item in enumerate(raw)]
    return targets


def load_grid_map(config: dict) -> GridMap:
    map_cfg = config["map"]
    obstacle_grid = load_txt_grid(map_cfg["obstacle_file"], dtype=int)
    radiation_grid = load_txt_grid(map_cfg["radiation_file"], dtype=float)
    return GridMap(
        obstacle_grid=obstacle_grid,
        radiation_grid=radiation_grid,
        grid_size=float(map_cfg.get("grid_size", 1.0)),
        robot_velocity=float(map_cfg.get("robot_velocity", 1.0)),
        ri_max=float(map_cfg.get("ri_max", 3.0)),
        allow_diagonal=bool(map_cfg.get("allow_diagonal", True)),
        prevent_corner_cutting=bool(map_cfg.get("prevent_corner_cutting", True)),
    )


def load_targets(config: dict, grid_map: GridMap) -> List[Target]:
    map_cfg = config["map"]
    source = str(map_cfg.get("target_source", "map")).lower()
    if source == "json":
        return load_targets_json(map_cfg["target_file"])
    if source == "map":
        return grid_map.extract_targets()
    raise ValueError("target_source must be 'map' or 'json'")
