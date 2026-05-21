from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from src.core.grid_map import GridMap
from src.costs.length_cost import path_length, segment_length
from src.costs.risk_cost import path_risk, segment_risk
from src.costs.energy_cost import path_energy, segment_turn_energy

GridPosition = Tuple[int, int]


@dataclass(frozen=True)
class CostWeights:
    omega_length: float = 0.45
    omega_risk: float = 0.40
    omega_energy: float = 0.15
    turn_angle_weight: float = 0.7
    turn_count_weight: float = 0.3

    @classmethod
    def from_config(cls, config: dict) -> "CostWeights":
        cfg = config.get("cost_weights", {})
        return cls(
            omega_length=float(cfg.get("omega_length", 0.45)),
            omega_risk=float(cfg.get("omega_risk", 0.40)),
            omega_energy=float(cfg.get("omega_energy", 0.15)),
            turn_angle_weight=float(cfg.get("turn_angle_weight", 0.7)),
            turn_count_weight=float(cfg.get("turn_count_weight", 0.3)),
        )


def weighted_segment_cost(
    grid_map: GridMap,
    prev_pos: Optional[GridPosition],
    current: GridPosition,
    nxt: GridPosition,
    weights: CostWeights,
) -> tuple[float, float, float, float]:
    l = segment_length(current, nxt, grid_map.grid_size)
    r = segment_risk(grid_map, current, nxt)
    e = segment_turn_energy(prev_pos, current, nxt, weights.turn_angle_weight, weights.turn_count_weight)
    total = weights.omega_length * l + weights.omega_risk * r + weights.omega_energy * e
    return total, l, r, e


def path_cost_components(grid_map: GridMap, path: list[GridPosition], weights: CostWeights) -> dict:
    length = path_length(path, grid_map.grid_size)
    risk = path_risk(grid_map, path)
    energy = path_energy(path, weights.turn_angle_weight, weights.turn_count_weight)
    total = weights.omega_length * length + weights.omega_risk * risk + weights.omega_energy * energy
    return {
        "length": float(length),
        "risk": float(risk),
        "energy": float(energy),
        "total": float(total),
    }
