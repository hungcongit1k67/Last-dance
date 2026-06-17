from __future__ import annotations

import argparse
import statistics
import time
from dataclasses import dataclass
from pathlib import Path

from src.planner.mtipp_solver import solve_mtipp
from src.utils.io_utils import load_yaml


@dataclass
class RunRecord:
    """All data collected from a single solver run."""

    total_cost: float
    wall_time: float                # wall-clock time of the whole solve() call
    phase1_time: float              # lower-level modified A* (cost matrix) time
    algorithm_time: float           # full algorithm time reported by the planner
    ga_time: float
    aco_time: float
    components: dict                # length / risk / energy / collision_risk / total
    num_targets: int
    visit_order: list[str]          # target ids in the order they are visited (closed tour)
    per_target_cost: dict[str, float]  # arrival segment cost for each target id


def _run_once(config: dict, run_index: int, base_seed: int | None, independent: bool) -> RunRecord:
    """Run solver once and collect detailed metrics."""
    run_cfg = dict(config)
    if independent:
        run_cfg["random_seed"] = None
    elif base_seed is not None:
        run_cfg["random_seed"] = base_seed + run_index

    t0 = time.perf_counter()
    result = solve_mtipp(run_cfg)
    wall_time = time.perf_counter() - t0

    timings = result.timings
    targets = result.targets
    route = result.aco_route
    total_mat = result.cost_matrices.total

    # Per-target arrival cost: cost of the segment that lands on each target.
    # The route is a closed tour [start, t1, ..., tk, start]; the final edge
    # returns to start, so the start target records its return-to-start cost.
    per_target_cost: dict[str, float] = {}
    for a, b in zip(route[:-1], route[1:]):
        per_target_cost[targets[b].id] = float(total_mat[a, b])

    visit_order = [targets[i].id for i in route]

    return RunRecord(
        total_cost=result.full_path_components["total"],
        wall_time=wall_time,
        phase1_time=float(timings.get("lower_level_modified_astar_time_sec", 0.0)),
        algorithm_time=float(timings.get("algorithm_total_time_sec", 0.0)),
        ga_time=float(timings.get("upper_level_ga_time_sec", 0.0)),
        aco_time=float(timings.get("upper_level_aco_time_sec", 0.0)),
        components=dict(result.full_path_components),
        num_targets=len(targets),
        visit_order=visit_order,
        per_target_cost=per_target_cost,
    )


def _mean_std(values: list[float]) -> tuple[float, float]:
    mean = statistics.mean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    return mean, std


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Evaluate ACO-GA-A* over n runs and report detailed per-run and aggregate statistics"
    )
    parser.add_argument("--config", default="configs/default.yaml", help="Path to YAML config")
    parser.add_argument("--n", type=int, default=10, help="Number of runs (default: 10)")
    parser.add_argument(
        "--independent",
        action="store_true",
        help="Ignore random_seed in config — use a fresh random seed each run for true independence",
    )
    args = parser.parse_args()

    config = load_yaml(args.config)
    n: int = args.n
    base_seed: int | None = config.get("random_seed")

    if base_seed is not None and not args.independent:
        seed_desc = f"base seed {base_seed}, offset per run"
    elif args.independent:
        seed_desc = "independent (random seed each run)"
    else:
        seed_desc = "random (no seed in config)"

    print(f"Config : {args.config}")
    print(f"Runs   : {n}")
    print(f"Seed   : {seed_desc}")
    print()

    records: list[RunRecord] = []
    width = len(str(n))

    for i in range(n):
        rec = _run_once(config, i, base_seed, args.independent)
        records.append(rec)

        print("-" * 70)
        print(f"  Run {i + 1:{width}d}/{n}")
        print(f"    total_cost      : {rec.total_cost:.6f}")
        print(
            f"    cost components : length={rec.components['length']:.4f}  "
            f"risk={rec.components['risk']:.4f}  energy={rec.components['energy']:.4f}  "
            f"collision_risk={rec.components['collision_risk']:.4f}"
        )
        print(f"    num targets     : {rec.num_targets}")
        print(f"    wall time       : {rec.wall_time:.3f}s")
        print(
            f"    phase 1 (A*)    : {rec.phase1_time:.3f}s   "
            f"GA={rec.ga_time:.3f}s   ACO={rec.aco_time:.3f}s   "
            f"algorithm_total={rec.algorithm_time:.3f}s"
        )
        print(f"    visit order     : {' -> '.join(rec.visit_order)}")
        print("    per-target arrival cost:")
        for tid, cost in rec.per_target_cost.items():
            print(f"        {tid:<8s} : {cost:.6f}")

    # ---- Aggregate statistics ----
    costs = [r.total_cost for r in records]
    phase1_times = [r.phase1_time for r in records]
    algo_times = [r.algorithm_time for r in records]
    wall_times = [r.wall_time for r in records]

    mean_cost, std_cost = _mean_std(costs)
    mean_p1, std_p1 = _mean_std(phase1_times)
    mean_algo, std_algo = _mean_std(algo_times)
    mean_wall, std_wall = _mean_std(wall_times)

    print()
    print("=" * 70)
    print("  Evaluation summary")
    print("=" * 70)
    print(f"  Runs                       : {n}")
    print()
    print("  Total cost")
    print(f"    mean / std               : {mean_cost:.6f} / {std_cost:.6f}")
    print(f"    min / max                : {min(costs):.6f} / {max(costs):.6f}")
    print()
    print("  Cost components (mean / std)")
    for comp in ("length", "risk", "energy", "collision_risk"):
        vals = [r.components[comp] for r in records]
        m, s = _mean_std(vals)
        print(f"    {comp:<24s} : {m:.6f} / {s:.6f}")
    print()
    print("  Phase 1 time - lower-level modified A* (cost matrix)")
    print(f"    mean / std               : {mean_p1:.3f}s / {std_p1:.3f}s")
    print()
    print("  Whole-algorithm time")
    print(f"    mean / std               : {mean_algo:.3f}s / {std_algo:.3f}s")
    print(f"    wall mean / std          : {mean_wall:.3f}s / {std_wall:.3f}s")
    print()

    # Per-target arrival cost across runs (keyed by target id, independent of visit order).
    all_ids: list[str] = []
    seen = set()
    for r in records:
        for tid in r.per_target_cost:
            if tid not in seen:
                seen.add(tid)
                all_ids.append(tid)

    print(f"  Per-target arrival cost (over {n} runs)")
    print(f"    {'target':<10s}{'mean':>14s}{'std':>14s}{'n':>6s}")
    for tid in all_ids:
        vals = [r.per_target_cost[tid] for r in records if tid in r.per_target_cost]
        m, s = _mean_std(vals)
        print(f"    {tid:<10s}{m:>14.6f}{s:>14.6f}{len(vals):>6d}")
    print("=" * 70)


if __name__ == "__main__":
    main()
