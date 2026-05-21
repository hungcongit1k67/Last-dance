from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class Target:
    """Inspection target on a grid map."""

    id: str
    row: int
    col: int

    @property
    def position(self) -> Tuple[int, int]:
        return self.row, self.col
