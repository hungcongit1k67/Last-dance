# -*- coding: utf-8 -*-
"""
eval_tsp.py — Kịch bản 4: Đánh giá hiệu quả pha tối ưu thứ tự thăm (TSP)
========================================================================
Thiết kế:
  - Pha 1 (WP-MSDF) được CỐ ĐỊNH: với mỗi (map, bộ trọng số) ta build ma trận
    chi phí MỘT LẦN, rồi đưa CÙNG ma trận đó lần lượt vào 3 bộ giải TSP:
        * ortools  -> bộ giải đề xuất của đồ án
        * aco      -> ACO thuần
        * aco-ga   -> ACO-GA bi-level
  - Nhờ vậy so sánh là thuần chất lượng lời giải TSP (Total cost) + thời gian
    chạy Pha 2, trên cùng một input.

Thiết lập chạy:
  - 11 map  ×  3 bộ trọng số  = 33 tổ hợp
  - Mỗi tổ hợp chạy 3 thuật toán:
        ortools : ntest = 3   (gần tất định -> ít lần là đủ)
        aco     : ntest = 5
        aco-ga  : ntest = 5

Kết quả: in BẢNG TẤT CẢ CÁC LẦN CHẠY + bảng tổng hợp (mean ± std, gap%),
và lưu ra CSV cạnh script.

Cách dùng:
    python eval_tsp.py
"""
import sys
sys.stdout.reconfigure(encoding='utf-8')
import os
import csv
import time
import datetime
import numpy as np

import My_grid as My_grid
from ADR_main_ortools import solve_tsp, routing_enums_pb2

# =========================================================
# CẤU HÌNH ĐÁNH GIÁ
# =========================================================
MAPS = [
    r"E:\last_dance\LastDance\FMF_new\scenario4\scenario4_grid.txt",
    r"E:\last_dance\LastDance\FMF_new\scenario5\scenario5_grid.txt",
    r"E:\last_dance\LastDance\FMF_new\scenario6\scenario6_grid.txt",
    r"E:\last_dance\LastDance\FMF_new\scenario7_2\scenario7_grid2.txt",
    r"E:\last_dance\LastDance\FMF_new\scenario7\scenario7_grid.txt",
    r"E:\last_dance\LastDance\FMF_new\mixed2002\mixed2002.txt",
    r"E:\last_dance\LastDance\FMF_new\square400\square400.txt",
    r"E:\last_dance\LastDance\FMF_new\triangle300\triangle300.txt",
    r"E:\last_dance\LastDance\FMF_new\mixed500\mixed500.txt",
    r"E:\last_dance\LastDance\FMF_new\warehouse3\warehouse3.txt",
    r"E:\last_dance\LastDance\FMF_new\warehouse4\warehouse4.txt",
]

# 3 bộ trọng số (w1 + w2 + w3 = 1)
WEIGHT_SETS = [
    {"w1": 0.4, "w2": 0.3, "w3": 0.3},
    {"w1": 0.6, "w2": 0.2, "w3": 0.2},
    {"w1": 0.3, "w2": 0.5, "w3": 0.2},
]

# (tên solver, số lần chạy)
SOLVERS = [
    ("ortools", 3),   # bộ giải đề xuất của đồ án
    ("aco",     5),
    ("aco-ga",  5),
]

# Nhãn hiển thị cho từng solver
SOLVER_LABEL = {
    "ortools": "Đề xuất (OR-Tools)",
    "aco":     "ACO",
    "aco-ga":  "ACO-GA",
}

# Tham số chung — đồng bộ với CONFIG trong ADR_main_ortools.py
BASE_CONFIG = {
    "C1": 0.5,
    "a": 1.0,
    "v": 1.0,
    "safety_radius": 5.0,
    "safety_max_distance": 7.0,
    "RI_max": 8,
    "red_flag": True,
    "radiation_norm": False,
    "Solver_minimize": True,
    "supercover": True,
    "bresenham": True,
}
DISTANCE_SCALE = 1000
TIME_LIMIT_SEC = 5
SMOOTH = False   # khớp CONFIG: tính total cost trên path cell-by-cell


# =========================================================
# Build Pha 1 (CỐ ĐỊNH ma trận chi phí) cho 1 (map, bộ trọng số)
# =========================================================
def build_fixed_grid(map_path, weights):
    """Tạo grid, nạp map, build WP-MSDF MỘT LẦN. Trả về grid đã cố định
    ma trận chi phí (grid.dijk) và sẵn sàng cho getPath/pathTotalCost."""
    grid = My_grid.GridMap(mapSize=20)
    grid.config(
        w1=weights["w1"], w2=weights["w2"], w3=weights["w3"],
        C1=BASE_CONFIG["C1"], a=BASE_CONFIG["a"], v=BASE_CONFIG["v"],
        safety_radius=BASE_CONFIG["safety_radius"],
        safety_max_distance=BASE_CONFIG["safety_max_distance"],
        Solver_minimize=BASE_CONFIG["Solver_minimize"],
        supercover=BASE_CONFIG["supercover"],
        bresenham=BASE_CONFIG["bresenham"],
        RI_max=BASE_CONFIG["RI_max"],
        red_flag=BASE_CONFIG["red_flag"],
        radiation_norm=BASE_CONFIG["radiation_norm"],
    )
    grid.get_grid_from_file(map_path)

    t0 = time.time()
    grid.buildGraphAdvanced()
    if not SMOOTH:
        # Cần cell-by-cell để getPath/pathTotalCost khớp TSP cost (Solver_minimize)
        grid.twoPointTracing(smooth=False)
        grid.dijkstra()
    t_phase1 = time.time() - t0
    return grid, t_phase1


# =========================================================
# Chạy 1 solver ntest lần trên ma trận đã cố định
# =========================================================
def run_solver(grid, tsp, ntest):
    """Chạy solver `tsp` ntest lần trên grid.dijk (đã cố định).
    Trả về list các dict per-run: {run, total_cost, length, radiation, risk,
    weighted_cost, time_phase2}."""
    fss = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
           if routing_enums_pb2 is not None else None)
    lsm = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
           if routing_enums_pb2 is not None else None)

    runs = []
    for it in range(ntest):
        t0 = time.time()
        route, weighted_cost = solve_tsp(
            tsp, grid.dijk,
            distance_scale=DISTANCE_SCALE,
            time_limit_sec=TIME_LIMIT_SEC,
            first_solution_strategy=fss,
            local_search_metaheuristic=lsm,
        )
        t_phase2 = time.time() - t0

        cells = grid.getPath(route)
        total, length, radiation, risk = grid.pathTotalCost(cells)
        runs.append({
            "run": it + 1,
            "total_cost": float(total),
            "length": float(length),
            "radiation": float(radiation) if radiation is not None else 0.0,
            "risk": float(risk),
            "weighted_cost": float(weighted_cost),
            "time_phase2": float(t_phase2),
        })
    return runs


# =========================================================
# In bảng dạng text (tự canh cột)
# =========================================================
def print_table(headers, rows, title=None):
    if title:
        print("\n" + title)
    cols = list(zip(*([headers] + rows))) if rows else [[h] for h in headers]
    widths = [max(len(str(c)) for c in col) for col in cols]
    sep = "-+-".join("-" * w for w in widths)
    line = " | ".join(str(h).ljust(widths[i]) for i, h in enumerate(headers))
    print(line)
    print(sep)
    for r in rows:
        print(" | ".join(str(v).ljust(widths[i]) for i, v in enumerate(r)))


def f(x, nd=4):
    return f"{x:.{nd}f}"


# =========================================================
# MAIN
# =========================================================
def main():
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "eval_tsp_results")
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    per_run_rows = []     # cho bảng tất cả các lần chạy + CSV
    summary_rows = []     # cho bảng tổng hợp + CSV

    total_combos = len(MAPS) * len(WEIGHT_SETS)
    combo_idx = 0

    for map_path in MAPS:
        map_name = os.path.splitext(os.path.basename(map_path))[0]
        for ws in WEIGHT_SETS:
            combo_idx += 1
            w_tag = f"({ws['w1']},{ws['w2']},{ws['w3']})"
            print("\n" + "=" * 78)
            print(f"[{combo_idx}/{total_combos}] MAP={map_name}  weights={w_tag}")
            print("=" * 78)

            try:
                grid, t_phase1 = build_fixed_grid(map_path, ws)
            except Exception as e:
                print(f"  !! Lỗi build Pha 1: {e}")
                continue

            n_cp = grid.npos
            print(f"  Map {grid.mapSize}x{grid.mapSize}, {n_cp} checkpoints, "
                  f"Pha 1 = {t_phase1:.3f}s  -> ma trận chi phí đã cố định")

            # Chạy 3 solver trên CÙNG ma trận
            solver_stats = {}   # tsp -> (mean_total, std_total, mean_t2, std_t2, best_total)
            for tsp, ntest in SOLVERS:
                try:
                    runs = run_solver(grid, tsp, ntest)
                except Exception as e:
                    print(f"  !! Lỗi solver {tsp}: {e}")
                    continue

                totals = np.array([r["total_cost"] for r in runs])
                times = np.array([r["time_phase2"] for r in runs])

                for r in runs:
                    per_run_rows.append({
                        "map": map_name, "n": n_cp, "weights": w_tag,
                        "solver": tsp, "run": r["run"],
                        "total_cost": r["total_cost"],
                        "length": r["length"],
                        "radiation": r["radiation"],
                        "risk": r["risk"],
                        "time_phase2": r["time_phase2"],
                    })
                solver_stats[tsp] = {
                    "mean_total": float(totals.mean()),
                    "std_total": float(totals.std()),
                    "best_total": float(totals.min()),
                    "mean_t2": float(times.mean()),
                    "std_t2": float(times.std()),
                    "ntest": ntest,
                }

            if not solver_stats:
                continue

            # gap% so với lời giải tốt nhất giữa các solver trong tổ hợp này
            best_of_combo = min(s["best_total"] for s in solver_stats.values())
            for tsp, ntest in SOLVERS:
                if tsp not in solver_stats:
                    continue
                s = solver_stats[tsp]
                gap = (s["mean_total"] - best_of_combo) / best_of_combo * 100.0 \
                    if best_of_combo > 0 else 0.0
                summary_rows.append({
                    "map": map_name, "n": n_cp, "weights": w_tag,
                    "solver": tsp, "ntest": s["ntest"],
                    "mean_total": s["mean_total"], "std_total": s["std_total"],
                    "best_total": s["best_total"], "gap_pct": gap,
                    "mean_t2": s["mean_t2"], "std_t2": s["std_t2"],
                })

    # =========================================================
    # IN BẢNG TỔNG HỢP — 99 dòng (11 map × 3 trọng số × 3 solver)
    # Mỗi dòng: Map | Weights | Solver | T_pha2(mean±std) | TotalCost(mean±std)
    # =========================================================
    headers_sum = ["Map", "Weights", "Solver",
                   "T_pha2(mean±std,s)", "TotalCost(mean±std)"]
    rows_sum = [[
        r["map"], r["weights"], SOLVER_LABEL.get(r["solver"], r["solver"]),
        f"{f(r['mean_t2'], 3)} ± {f(r['std_t2'], 3)}",
        f"{f(r['mean_total'])} ± {f(r['std_total'])}",
    ] for r in summary_rows]
    print_table(headers_sum, rows_sum,
                title=f"\n##### BẢNG TỔNG HỢP ({len(rows_sum)} dòng) #####")

    # =========================================================
    # LƯU CSV
    # =========================================================
    run_csv = os.path.join(out_dir, f"per_run_{stamp}.csv")
    with open(run_csv, "w", newline="", encoding="utf-8-sig") as fp:
        wtr = csv.DictWriter(fp, fieldnames=list(per_run_rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(per_run_rows)

    sum_csv = os.path.join(out_dir, f"summary_{stamp}.csv")
    with open(sum_csv, "w", newline="", encoding="utf-8-sig") as fp:
        wtr = csv.DictWriter(fp, fieldnames=list(summary_rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(summary_rows)

    print(f"\nĐã lưu:\n  - {run_csv}\n  - {sum_csv}")


if __name__ == "__main__":
    main()
