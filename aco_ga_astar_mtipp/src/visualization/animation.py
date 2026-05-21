"""Optional animation hooks.

Kept intentionally minimal. You can extend this file with matplotlib.animation
or export the path to a robot simulator.
"""

from typing import Sequence, Tuple

GridPosition = Tuple[int, int]


def path_to_frames(path: Sequence[GridPosition]) -> list[GridPosition]:
    return list(path)
