from __future__ import annotations

import argparse
import statistics
import time
from pathlib import Path

from src.planner.mtipp_solver import solve_mtipp
from src.utils.io_utils import load_yaml


def _run_once(config: dict, run_index: int, base_seed: int | None, independent: bool) -> tuple[float, float]:
    """Run solver once and return (total_cost, elapsed_seconds)."""
    run_cfg = dict(config)
    if independent:
        run_cfg["random_seed"] = None
    elif base_seed is not None:
        run_cfg["random_seed"] = base_seed + run_index

    t0 = time.perf_counter()
    result = solve_mtipp(run_cfg)
    elapsed = time.perf_counter() - t0
    return result.full_path_components["total"], elapsed


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate ACO-GA-A* over n runs and report mean/std of total_cost")
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

    costs: list[float] = []
    run_times: list[float] = []
    width = len(str(n))

    for i in range(n):
        cost, elapsed = _run_once(config, i, base_seed, args.independent)
        costs.append(cost)
        run_times.append(elapsed)
        print(f"  Run {i + 1:{width}d}/{n}  total_cost={cost:.6f}  time={elapsed:.3f}s")

    mean_cost = statistics.mean(costs)
    std_cost = statistics.stdev(costs) if n > 1 else 0.0
    mean_time = statistics.mean(run_times)

    print()
    print("=" * 42)
    print("  Evaluation summary")
    print("=" * 42)
    print(f"  Runs       : {n}")
    print(f"  Mean cost  : {mean_cost:.6f}")
    print(f"  Std  (bias): {std_cost:.6f}")
    print(f"  Min  cost  : {min(costs):.6f}")
    print(f"  Max  cost  : {max(costs):.6f}")
    print(f"  Mean time  : {mean_time:.3f}s")
    print("=" * 42)


if __name__ == "__main__":
    main()
