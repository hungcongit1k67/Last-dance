from typing import Tuple

from src.costs.length_cost import segment_length
from src.core.grid_map import GridMap

GridPosition = Tuple[int, int]


def segment_risk(grid_map: GridMap, a: GridPosition, b: GridPosition) -> float:
    """Cumulative dose along segment.

    radiation_grid is interpreted as dose-rate in arbitrary units/hour.
    Time is converted from seconds to hours using robot_velocity.
    """
    length_m = segment_length(a, b, grid_map.grid_size)
    time_hours = (length_m / grid_map.robot_velocity) # Có thể / 3600.0 để chuyển từ giây sang giờ
    avg_dose_rate = (grid_map.radiation_at(a) + grid_map.radiation_at(b)) / 2.0
    return avg_dose_rate * time_hours


def path_risk(grid_map: GridMap, path: list[GridPosition]) -> float:
    return sum(segment_risk(grid_map, path[i], path[i + 1]) for i in range(len(path) - 1))
