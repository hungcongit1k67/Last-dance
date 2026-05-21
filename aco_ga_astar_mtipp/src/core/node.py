from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass(order=True)
class Node:
    """A* priority queue node."""

    f: float
    row: int = field(compare=False)
    col: int = field(compare=False)
    g: float = field(default=0.0, compare=False)
    h: float = field(default=0.0, compare=False)
    parent: Optional[Tuple[int, int]] = field(default=None, compare=False)

    @property
    def position(self) -> Tuple[int, int]:
        return self.row, self.col
