from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from src.core.grid_map import GridMap
from src.costs.length_cost import path_length, segment_length
from src.costs.risk_cost import path_risk, segment_risk
from src.costs.energy_cost import path_energy, segment_turn_energy
from src.costs.collision_risk_cost import path_collision_risk, cell_collision_risk

GridPosition = Tuple[int, int]


@dataclass(frozen=True)
class CostWeights:
    omega_length: float = 0.45
    omega_risk: float = 0.40
    omega_energy: float = 0.15
    omega_collision_risk: float = 0.0
    turn_angle_weight: float = 0.5
    turn_count_weight: float = 0.5
    safety_c1: float = 0.5
    safety_radius: int = 2
    safety_max_distance: float = 3.0

    @classmethod
    def from_config(cls, config: dict) -> "CostWeights":
        cfg = config.get("cost_weights", {})
        return cls(
            omega_length=float(cfg.get("omega_length", 0.45)),
            omega_risk=float(cfg.get("omega_risk", 0.40)),
            omega_energy=float(cfg.get("omega_energy", 0.15)),
            omega_collision_risk=float(cfg.get("omega_collision_risk", 0.0)),
            turn_angle_weight=float(cfg.get("turn_angle_weight", 0.5)),
            turn_count_weight=float(cfg.get("turn_count_weight", 0.5)),
            safety_c1=float(cfg.get("safety_c1", 0.5)),
            safety_radius=int(cfg.get("safety_radius", 2)),
            safety_max_distance=float(cfg.get("safety_max_distance", 3.0)),
        )


def weighted_segment_cost(
    grid_map: GridMap,
    prev_pos: Optional[GridPosition],
    current: GridPosition,
    nxt: GridPosition,
    weights: CostWeights,
) -> tuple[float, float, float, float, float]:
    """Returns (total, length, risk, energy, collision_risk) for one A* step.

    collision_risk is computed on `current` cell only; the goal cell is excluded
    when all steps are accumulated, matching formula 11: sum_{n=1}^{|p|-1} (1-S(p_n)).
    """
    l = segment_length(current, nxt, grid_map.grid_size)
    r = segment_risk(grid_map, current, nxt)
    e = segment_turn_energy(prev_pos, current, nxt, weights.turn_angle_weight, weights.turn_count_weight)
    cr = cell_collision_risk(grid_map, current, weights.safety_c1, weights.safety_radius, weights.safety_max_distance)
    total = (
        weights.omega_length * l
        + weights.omega_risk * r
        + weights.omega_energy * e
        + weights.omega_collision_risk * cr
    )
    return total, l, r, e, cr


def path_cost_components(grid_map: GridMap, path: list[GridPosition], weights: CostWeights) -> dict:
    length = path_length(path, grid_map.grid_size)
    risk = path_risk(grid_map, path)
    energy = path_energy(path, weights.turn_angle_weight, weights.turn_count_weight)
    collision_risk = path_collision_risk(
        grid_map=grid_map,
        path=path,
        c1=weights.safety_c1,
        radius=weights.safety_radius,
        max_distance=weights.safety_max_distance,
    )
    total = (
        weights.omega_length * length
        + weights.omega_risk * risk
        + weights.omega_energy * energy
        + weights.omega_collision_risk * collision_risk
    )
    return {
        "length": float(length),
        "risk": float(risk),
        "energy": float(energy),
        "collision_risk": float(collision_risk),
        "total": float(total),
    }
