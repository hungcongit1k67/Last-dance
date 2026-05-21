from typing import Optional, Tuple

from src.utils.math_utils import turn_angle_degrees

GridPosition = Tuple[int, int]


def segment_turn_energy(
    prev_pos: Optional[GridPosition],
    current: GridPosition,
    nxt: GridPosition,
    turn_angle_weight: float = 0.7,
    turn_count_weight: float = 0.3,
) -> float:
    angle = turn_angle_degrees(prev_pos, current, nxt)
    if angle <= 1e-9:
        return 0.0
    # Normalize angle to [0, 1] by /180 and add a turn-count penalty.
    return turn_angle_weight * (angle / 180.0) + turn_count_weight


def path_energy(
    path: list[GridPosition],
    turn_angle_weight: float = 0.7,
    turn_count_weight: float = 0.3,
) -> float:
    if len(path) < 3:
        return 0.0
    return sum(
        segment_turn_energy(path[i - 1], path[i], path[i + 1], turn_angle_weight, turn_count_weight)
        for i in range(1, len(path) - 1)
    )
