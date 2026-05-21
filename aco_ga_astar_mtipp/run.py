from __future__ import annotations

import argparse
from pathlib import Path

from src.environment.map_loader import load_grid_map
from src.planner.mtipp_solver import solve_mtipp
from src.utils.io_utils import load_yaml, save_json, save_matrix, ensure_dir
from src.visualization.plot_path import plot_path
from src.visualization.plot_convergence import plot_convergence
import time


def route_target_ids(result) -> list[str]:
    return [result.targets[i].id for i in result.aco_route]


def main() -> None:
    program_start = time.perf_counter()
    parser = argparse.ArgumentParser(description="Run ACO-GA-A* for MTIPP")
    parser.add_argument("--config", default="configs/default.yaml", help="Path to YAML config")
    args = parser.parse_args()

    config = load_yaml(args.config)
    out_dir = Path(config.get("project", {}).get("output_dir", "results"))
    ensure_dir(out_dir / "paths")
    ensure_dir(out_dir / "figures")
    ensure_dir(out_dir / "logs")
    ensure_dir(out_dir / "cost_matrices")

    result = solve_mtipp(config)
    grid_map = load_grid_map(config)

    save_matrix(result.cost_matrices.length, out_dir / "cost_matrices" / "length_matrix.txt")
    save_matrix(result.cost_matrices.risk, out_dir / "cost_matrices" / "risk_matrix.txt")
    save_matrix(result.cost_matrices.energy, out_dir / "cost_matrices" / "energy_matrix.txt")
    save_matrix(result.cost_matrices.total, out_dir / "cost_matrices" / "total_cost_matrix.txt")

    summary = {
        "ga_route_index": result.ga_route,
        "ga_cost": result.ga_cost,
        "aco_route_index": result.aco_route,
        "aco_route_target_ids": route_target_ids(result),
        "aco_cost_from_matrix": result.aco_cost,
        "full_path_components": result.full_path_components,
        "timings": result.timings,
        "full_path": result.full_path,
        "targets": [{"id": t.id, "row": t.row, "col": t.col} for t in result.targets],
    }
    save_json(summary, out_dir / "paths" / "route_summary.json")
    plot_path(grid_map, result.full_path, out_dir / "figures" / "path_result.png")
    plot_convergence(result.aco_history, out_dir / "figures" / "aco_convergence.png", title="ACO-GA-A* convergence")
    plot_convergence(result.ga_history, out_dir / "figures" / "ga_convergence.png", title="GA initial search convergence")
    program_total_time = time.perf_counter() - program_start
    
    print("Done.")
    print("Route:", " -> ".join(route_target_ids(result)))
    print("ACO matrix cost:", result.aco_cost) # đây là chi phí tính từ ma trận, có thể khác với chi phí thực tế trên đường đi do đường đi có thể không phải là đường đi ngắn nhất giữa 2 target
    print("Full path components:", result.full_path_components) # chi phí thực tế trên đường đi, được tính bằng cách cộng chi phí từng bước đi trên full path, có thể khác với chi phí từ ma trận nếu đường đi không phải là đường ngắn nhất giữa 2 target hoặc nếu có chi phí phi tuyến tính nào đó

    print("\nRuntime:")
    print(f"  Lower level - Modified A*: {result.timings['lower_level_modified_astar_time_sec']:.6f} s")
    print(f"  Upper level - GA:          {result.timings['upper_level_ga_time_sec']:.6f} s")
    print(f"  Upper level - ACO:         {result.timings['upper_level_aco_time_sec']:.6f} s")
    print(f"  Upper level - GA + ACO:    {result.timings['upper_level_total_time_sec']:.6f} s")
    print(f"  Stitch + full cost:        {result.timings['stitch_and_full_cost_time_sec']:.6f} s")
    print(f"  Algorithm total:           {result.timings['algorithm_total_time_sec']:.6f} s") # Thời gian chạy chỉ tính phần giải thuật, không tính visualization và lưu file
    print(f"  Program total:             {program_total_time:.6f} s") # Tính cả chạy visualization và lưu file

    print("Saved summary to:", out_dir / "paths" / "route_summary.json")
    print("Saved path figure to:", out_dir / "figures" / "path_result.png")


if __name__ == "__main__":
    main()
