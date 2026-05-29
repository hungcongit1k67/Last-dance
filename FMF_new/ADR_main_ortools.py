"""
ADR_main_ortools.py - Entry point cho pipeline WP-FMF + OR-Tools TSP
======================================================================
Pha 1: WP-FMF (Weighted Potential Fast Marching Firework) -> ma trận chi phí
Pha 2: OR-Tools Routing Solver -> giải TSP trên ma trận đó

Cách dùng nhanh:
    python ADR_main_ortools.py
Hoặc import hàm run_wpfmf_pipeline(...) để gọi trong notebook.
"""
# -*- coding: utf-8 -*-
import sys
sys.stdout.reconfigure(encoding='utf-8')
import os
import time
import json
import datetime
import numpy as np

import My_grid as My_grid

try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2
except ImportError:
    pywrapcp = None
    routing_enums_pb2 = None


# =========================================================
# CONFIG — Chỉnh sửa tất cả tham số tại đây
# =========================================================
CONFIG = {
    # --- Trọng số tổng hợp (bắt buộc: w1 + w2 + w3 = 1.0) ---
    "w1": 0.4,      # Chiều dài đường đi  length(P)
    "w2": 0.3,      # Độ rủi ro phóng xạ  R(P)
    "w3": 0.3,      # Độ rủi ro va chạm   risk(P)

    # --- Tham số an toàn va chạm (công thức 4) ---
    "C1": 0.5,      # C1→1: ưu tiên N_obs;  C1→0: ưu tiên d_min
    "safety_radius": 5.0,        # bán kính vùng lân cận tính S(c)
    "safety_max_distance": 7.0,  # khoảng cách chuẩn hóa d_min trong S(c)

    # --- Tham số vật lý (công thức 6) ---
    "a": 1.0,       # Kích thước ô lưới (m)
    "v": 1.0,       # Vận tốc robot (m/s)

    # --- Đường dẫn bản đồ ---
    # "map_path": r"E:\last_dance\LastDance\FMF_new\triangle300\triangle300.txt",
    "map_path": r"E:\last_dance\LastDance\FMF_new\default\obstacle_grid.txt",

    # --- Tham số OR-Tools TSP ---
    "ntest": 1, # Số lần chạy OR-Tools (với cùng tham số) để đánh giá độ ổn định của giải pháp
    "distance_scale": 1000, # Scale ma trận chi phí từ float sang int cho OR-Tools (ví dụ: 1.0 -> 1000, sqrt(2) -> 1414)
    "time_limit_sec": 10, # Thời gian tối đa cho mỗi lần chạy OR-Tools (giây)

    # --- Path output ---
    # True  → in/lưu path đã smooth (turning points, ~128 bước)
    # False → in/lưu full path cell-by-cell (đi qua từng ô lưới, ~1000+ bước)
    "smooth": True,
}


# =========================================================
# Results helpers
# =========================================================
def _results_dir():
    """Trả về đường dẫn thư mục results/ cạnh script, tạo nếu chưa có."""
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
    os.makedirs(d, exist_ok=True)
    return d


def _save_run(map_path, config_snapshot, run_data):
    """Lưu/cập nhật kết quả vào results/<map_name>.json, mỗi lần chạy append thêm 1 run."""
    map_name = os.path.splitext(os.path.basename(map_path))[0]
    json_path = os.path.join(_results_dir(), f"{map_name}.json")

    if os.path.exists(json_path):
        with open(json_path, 'r', encoding='utf-8') as f:
            record = json.load(f)
    else:
        record = {"map": map_name, "map_path": map_path, "runs": []}

    run_data["timestamp"] = datetime.datetime.now().isoformat(timespec='seconds')
    run_data["config"] = config_snapshot
    record["runs"].append(run_data)

    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(record, f, ensure_ascii=False, indent=2)

    print(f"\nResults saved -> {json_path}  (total runs: {len(record['runs'])})")


# =========================================================
# Timing helper
# =========================================================
def timeEval(grid, w1=None, w2=None, w3=None, C1=None):
    start = time.time()
    grid.buildGraphAdvanced(w1=w1, w2=w2, w3=w3, C1=C1)
    print("--- Phase 1 (WP-FMF) took %.4f seconds ---" % (time.time() - start))


# =========================================================
# OR-Tools helpers
# =========================================================
def _require_ortools():
    if pywrapcp is None or routing_enums_pb2 is None:
        raise ImportError("Chua cai OR-Tools. Cai bang: pip install ortools")


def _build_int_distance_matrix(dist_matrix, distance_scale=1000):
    """
    OR-Tools yeu cau cost nguyen. Scale ma tran float len distance_scale roi lam tron.
    Vi du: 1.0 -> 1000, sqrt(2) -> 1414.
    """
    n = len(dist_matrix)
    int_matrix = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                int_matrix[i][j] = 0
            else:
                val = float(dist_matrix[i][j])
                # Safeguard: inf hoặc NaN
                if not np.isfinite(val):
                    int_matrix[i][j] = 10 ** 9
                else:
                    int_matrix[i][j] = int(round(val * distance_scale))
    return int_matrix


def _route_cost_float(route, dist_matrix):
    if route is None or len(route) == 0:
        return float("inf")
    total = 0.0
    for i in range(len(route)):
        u = int(route[i])
        v = int(route[(i + 1) % len(route)])
        total += float(dist_matrix[u][v])
    return total


def solve_tsp_ortools(dist_matrix,
                      distance_scale=1000,
                      time_limit_sec=5,
                      first_solution_strategy=None,
                      local_search_metaheuristic=None):
    """Giai TSP chu trinh bang OR-Tools. Tra ve (route, real_cost_float)."""
    _require_ortools()

    n = len(dist_matrix)
    int_matrix = _build_int_distance_matrix(dist_matrix, distance_scale)

    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int_matrix[from_node][to_node]

    transit_cb_idx = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb_idx)

    params = pywrapcp.DefaultRoutingSearchParameters()
    if first_solution_strategy is None:
        first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    if local_search_metaheuristic is None:
        local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.first_solution_strategy = first_solution_strategy
    params.local_search_metaheuristic = local_search_metaheuristic
    params.time_limit.seconds = time_limit_sec

    solution = routing.SolveWithParameters(params)
    if solution is None:
        raise RuntimeError("OR-Tools khong tim thay nghiem TSP hop le.")

    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))

    real_cost = _route_cost_float(route, dist_matrix)
    return route, real_cost


# =========================================================
# Danh gia toan bo pipeline (phase 2 + metric)
# =========================================================
def evaluation_wpfmf(grid,
                    ntest=1,
                    distance_scale=1000,
                    time_limit_sec=5,
                    first_solution_strategy=None,
                    local_search_metaheuristic=None,
                    draw=True,
                    map_path=None,
                    smooth=True):
    """
    Chay OR-Tools ntest lan tren grid.dijk (da duoc tinh bang WP-FMF).
    Tra ve (best_path, best_cost).
    In ra: chieu dai, phong xa, rui ro va cham, va total cost.
    """
    _require_ortools()

    res = []
    res_lengths = []
    res_radiations = []
    res_risks = []
    res_totals = []
    best_path = None
    best_cost = float("inf")
    iterations_log = []
    has_radiation = grid.radiation_map is not None

    # Cần cell-by-cell để getPath/pathTotalCost cho từng iteration
    if not smooth:
        grid.twoPointTracing(smooth=False)

    for it in range(ntest):
        print(f"Iteration {it + 1}/{ntest}")
        t0 = time.time()
        path, cost = solve_tsp_ortools(
            grid.dijk,
            distance_scale=distance_scale,
            time_limit_sec=time_limit_sec,
            first_solution_strategy=first_solution_strategy,
            local_search_metaheuristic=local_search_metaheuristic,
        )
        elapsed = time.time() - t0

        cells_i = grid.getPath(path)
        total_i, length_i, rad_i, risk_i = grid.pathTotalCost(cells_i)

        res.append(cost)
        res_lengths.append(float(length_i))
        res_radiations.append(float(rad_i) if rad_i is not None else 0.0)
        res_risks.append(float(risk_i))
        res_totals.append(float(total_i))
        iterations_log.append({
            "iteration": it + 1,
            "cost": cost,
            "length":     round(float(length_i), 6),
            "radiation":  round(float(rad_i), 6) if has_radiation else None,
            "risk":       round(float(risk_i), 6),
            "total_cost": round(float(total_i), 6),
            "time_sec":   round(elapsed, 4),
        })
        if cost < best_cost:
            best_cost = cost
            best_path = list(path)

        print("  Route (OR-Tools):", path)
        print(f"  Weighted cost:   {cost:.4f}   "
              f"length={length_i:.4f}   "
              f"R={(rad_i if rad_i is not None else 0.0):.4f}   "
              f"risk={risk_i:.4f}   total={total_i:.4f}")
        print(f"  --- Iteration {it + 1}: {elapsed:.3f} seconds ---")

    res_arr        = np.array(res)
    res_lengths    = np.array(res_lengths)
    res_radiations = np.array(res_radiations)
    res_risks      = np.array(res_risks)
    res_totals     = np.array(res_totals)

    print("\n===== Phase 2 summary =====")
    print(f"Mean weighted cost: {res_arr.mean():.4f}")
    print(f"Std  weighted cost: {res_arr.std():.4f}")
    print(f"Best weighted cost: {best_cost:.4f}")

    print(f"\n===== Metrics mean ± std qua {ntest} lần chạy =====")
    print(f"  length(P)  : {res_lengths.mean():10.4f} ± {res_lengths.std():.4f}")
    if has_radiation:
        print(f"  R(P)       : {res_radiations.mean():10.4f} ± {res_radiations.std():.4f}")
    else:
        print(f"  R(P)       : N/A  (chưa nạp radiation_map)")
    print(f"  risk(P)    : {res_risks.mean():10.4f} ± {res_risks.std():.4f}")
    print(f"  Total cost : {res_totals.mean():10.4f} ± {res_totals.std():.4f}")

    # Best path metrics (theo TSP cost)
    cells = grid.getPath(best_path)
    total, length, radiation, risk = grid.pathTotalCost(cells)

    path_tuples = [(int(c[0]), int(c[1])) for c in cells]
    path_label  = "turning points (smoothed)" if smooth else "cell-by-cell (full)"

    print("\n===== Path metrics (thực tế trên grid) =====")
    print(f"Path ({len(path_tuples)} bước, {path_label}):")
    print(path_tuples)
    print(f"  length(P)    = {length:.4f}   (công thức 3)")
    if grid.radiation_map is not None:
        print(f"  R(P)         = {radiation:.4f}   (công thức 6, a={grid.a}, v={grid.v})")
    else:
        print(f"  R(P)         = N/A  (chưa nạp radiation_map)")
    print(f"  risk(P)      = {risk:.4f}   (công thức 5)")
    print(f"  Total cost   = {total:.4f}")
    print(f"  (w1={grid.w1:.2f}·length + w2={grid.w2:.2f}·R + w3={grid.w3:.2f}·risk)")

    # Khôi phục smooth=True để drawPath/drawFMComponent hoạt động bình thường
    if not smooth:
        grid.twoPointTracing(smooth=True)

    # Lưu kết quả ra JSON nếu có map_path
    if map_path is not None:
        config_snapshot = {
            "w1": grid.w1, "w2": grid.w2, "w3": grid.w3,
            "C1": grid.C1, "a": grid.a, "v": grid.v,
            "safety_radius": grid.safety_radius,
            "safety_max_distance": grid.safety_max_distance,
            "ntest": ntest, "distance_scale": distance_scale,
            "time_limit_sec": time_limit_sec,
            "map_size": grid.mapSize, "checkpoints": grid.npos,
            "smooth": smooth,
        }
        run_data = {
            "summary": {
                "mean_cost": round(float(res_arr.mean()), 6),
                "std_cost":  round(float(res_arr.std()),  6),
                "best_cost": round(float(best_cost),      6),
            },
            "metrics": {
                "length":     round(float(length),    6),
                "radiation":  round(float(radiation), 6) if grid.radiation_map is not None else None,
                "risk":       round(float(risk),      6),
                "total_cost": round(float(total),     6),
            },
            "metrics_mean": {
                "length":     {"mean": round(float(res_lengths.mean()),    6),
                               "std":  round(float(res_lengths.std()),     6)},
                "radiation":  ({"mean": round(float(res_radiations.mean()), 6),
                                "std":  round(float(res_radiations.std()),  6)}
                               if has_radiation else None),
                "risk":       {"mean": round(float(res_risks.mean()),      6),
                               "std":  round(float(res_risks.std()),       6)},
                "total_cost": {"mean": round(float(res_totals.mean()),     6),
                               "std":  round(float(res_totals.std()),      6)},
            },
            "path": path_tuples,
            "iterations": iterations_log,
        }
        _save_run(map_path, config_snapshot, run_data)

    if draw:
        grid.drawPath(cells)
        grid.drawFMComponent()
        grid.drawDijkstraWave()
        try:
            grid.drawSafety()
            grid.drawFCost()
            if grid.radiation_map is not None:
                grid.drawRadiation()
        except AttributeError:
            pass

    return best_path, best_cost


# =========================================================
# Full pipeline wrapper
# =========================================================
def run_wpfmf_pipeline(grid,
                      w1=None,
                      w2=None,
                      w3=None,
                      C1=None,
                      ntest=1,
                      distance_scale=1000,
                      time_limit_sec=5,
                      first_solution_strategy=None,
                      local_search_metaheuristic=None,
                      draw=True,
                      map_path=None,
                      smooth=True):
    """
    Pipeline đầy đủ:
      Pha 1: WP-FMF   -> grid.dijk (ma trận chi phí giữa các checkpoint)
      Pha 2: OR-Tools -> thứ tự thăm tối ưu

    Trọng số (w1+w2+w3=1):
      w1: length(P),  w2: R(P) phóng xạ,  w3: risk(P) va chạm
    C1: cân bằng N_obs vs d_min trong S(c)
    smooth: True  → path in ra là turning points (~128 bước)
            False → path in ra là từng ô cell-by-cell (~1000+ bước)

    Nếu không truyền tham số, dùng giá trị hiện tại của grid (đã set qua config()).
    """
    print(f"===== WP-FMF Pipeline (w1={w1 or grid.w1}, w2={w2 or grid.w2}, "
          f"w3={w3 or grid.w3}, C1={C1 or grid.C1}) =====")
    print("[Phase 1] Building WP-FMF cost matrix ...")
    timeEval(grid, w1=w1, w2=w2, w3=w3, C1=C1)

    print("\n[Phase 2] OR-Tools TSP on cost matrix ...")
    return evaluation_wpfmf(
        grid,
        ntest=ntest,
        distance_scale=distance_scale,
        time_limit_sec=time_limit_sec,
        first_solution_strategy=first_solution_strategy,
        local_search_metaheuristic=local_search_metaheuristic,
        draw=draw,
        map_path=map_path,
        smooth=smooth,
    )


# Giữ tên hàm cũ cho tương thích
def ADR_main(grid,
             ntest=1,
             distance_scale=1000,
             time_limit_sec=5,
             first_solution_strategy=None,
             local_search_metaheuristic=None,
             w1=None,
             w2=None,
             w3=None,
             C1=None,
             map_path=None,
             smooth=True):
    return run_wpfmf_pipeline(
        grid,
        w1=w1, w2=w2, w3=w3, C1=C1,
        ntest=ntest,
        distance_scale=distance_scale,
        time_limit_sec=time_limit_sec,
        first_solution_strategy=first_solution_strategy,
        local_search_metaheuristic=local_search_metaheuristic,
        map_path=map_path,
        smooth=smooth,
    )


def ADF(grid):
    return ADR_main(grid)


# =========================================================
# Main
# =========================================================
def main():
    grid = My_grid.GridMap(mapSize=20)

    # Áp dụng CONFIG vào grid
    grid.config(
        w1=CONFIG["w1"],
        w2=CONFIG["w2"],
        w3=CONFIG["w3"],
        C1=CONFIG["C1"],
        a=CONFIG["a"],
        v=CONFIG["v"],
        safety_radius=CONFIG["safety_radius"],
        safety_max_distance=CONFIG["safety_max_distance"],
    )

    map_path = CONFIG["map_path"]
    grid.get_grid_from_file(map_path)
    print(f"Loaded {map_path}: {grid.mapSize}x{grid.mapSize}, {grid.npos} checkpoints")
    for pos in grid.deslist:
        print(" ", pos)
    print()

    run_wpfmf_pipeline(
        grid,
        ntest=CONFIG["ntest"],
        distance_scale=CONFIG["distance_scale"],
        time_limit_sec=CONFIG["time_limit_sec"],
        first_solution_strategy=(
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
            if routing_enums_pb2 is not None else None),
        local_search_metaheuristic=(
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
            if routing_enums_pb2 is not None else None),
        map_path=map_path,
        smooth=CONFIG["smooth"],
    )


if __name__ == "__main__":
    main()
