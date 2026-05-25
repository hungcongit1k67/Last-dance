# -*- coding: utf-8 -*-
import sys
sys.stdout.reconfigure(encoding='utf-8')

import My_grid_patched as My_grid
import GA
import numpy as np
import time
import json
import os
import datetime

try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2
except ImportError:  # pragma: no cover
    pywrapcp = None
    routing_enums_pb2 = None


# =========================================================
# CONFIG — Chỉnh sửa tất cả tham số tại đây
# =========================================================
CONFIG = {
    # --- Trọng số báo cáo (bắt buộc: w1 + w2 + w3 = 1.0) ---
    # Không ảnh hưởng thuật toán FMF (vẫn minimize length thuần túy).
    # Chỉ dùng để tính và in total cost sau khi tìm được đường đi.
    "w1": 0.6,      # Chiều dài đường đi  length(P)
    "w2": 0.2,      # Độ rủi ro phóng xạ  R(P)
    "w3": 0.2,      # Độ rủi ro va chạm   risk(P)

    # --- Tham số an toàn va chạm S(c) (công thức 4) ---
    "C1": 0.5,      # C1→1: ưu tiên N_obs;  C1→0: ưu tiên d_min
    "safety_radius": 5.0,        # bán kính vùng lân cận tính S(c)
    "safety_max_distance": 7.0,  # khoảng cách chuẩn hóa d_min trong S(c)

    # --- Tham số vật lý (công thức 6 — tính R(P)) ---
    "a": 1.0,       # Kích thước ô lưới (m)
    "v": 1.0,       # Vận tốc robot (m/s)

    # --- Đường dẫn bản đồ ---
    # radiation_grid.txt trong cùng thư mục sẽ được nạp tự động.
    "map_path": r"E:\last_dance\LastDance\FMF\mixed200\mixed200.txt",

    # --- Tham số OR-Tools TSP ---
    "ntest": 1,
    "distance_scale": 1000,
    "time_limit_sec": 5,

    # --- Path output ---
    # True  → in/lưu path đã smooth (turning points, ~128 bước)
    # False → in/lưu full path cell-by-cell (đi qua từng ô lưới, ~1000+ bước)
    "smooth": True,
}


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


def timeEval(grid):
    start_time = time.time()
    grid.buildGraphAdvanced()
    print("--- %s seconds ---" % (time.time() - start_time))


# =========================
# Old evaluations are kept
# =========================
def evaluation1(grid, ntest=10):  # <----- iteration
    resGA = []
    resACO = []
    bestpath = None
    bestcost = grid.mapSize * grid.mapSize * grid.npos
    for iter in range(ntest):
        print("Iteration ", iter + 1, "/", ntest)
        a = time.time()
        ga = GA.GA(grid)
        mypath = ga.solve(500, 600)
        mycost = ga.c_cost(mypath)
        resACO.append(mycost)
        if bestcost > mycost:
            bestcost = mycost
            bestpath = mypath
        print(f"--- Iteration {iter+1}: {time.time() - a} seconds ---")

    resACO = np.array(resACO)
    sumACO = resACO.mean()
    stdACO = resACO.std()
    print("Mean GA cost: ", sumACO, " Std GA std", stdACO)
    print("Best cost:", bestcost)

    points = grid.getPath(bestpath)
    grid.drawPath(points)
    grid.drawFMComponent(rmv=[0])
    grid.drawDijkstraWave(rmv=[])


# =========================
# OR-Tools helpers
# =========================
def _require_ortools():
    if pywrapcp is None or routing_enums_pb2 is None:
        raise ImportError(
            "Chua cai OR-Tools. Hay cai bang lenh: pip install ortools"
        )



def _build_int_distance_matrix(dist_matrix, distance_scale=1000):
    """
    OR-Tools Routing Solver yeu cau cost la so nguyen.
    Ta scale ma tran khoang cach thuc len distance_scale roi lam tron.
    Vi du: 1 -> 1000, sqrt(2) -> 1414.
    """
    n = len(dist_matrix)
    int_matrix = [[0 for _ in range(n)] for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                int_matrix[i][j] = 0
            else:
                val = float(dist_matrix[i][j])
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



def solve_tsp_ortools(
    dist_matrix,
    distance_scale=1000,
    time_limit_sec=5,
    first_solution_strategy=None,
    local_search_metaheuristic=None,
):
    """
    Giai TSP chu trinh tren ma tran khoang cach bang OR-Tools.
    Tra ve route khong lap lai dinh dau o cuoi.
    """
    _require_ortools()

    n = len(dist_matrix)
    int_matrix = _build_int_distance_matrix(dist_matrix, distance_scale=distance_scale)

    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    if first_solution_strategy is None:
        first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    if local_search_metaheuristic is None:
        local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH

    search_parameters.first_solution_strategy = first_solution_strategy
    search_parameters.local_search_metaheuristic = local_search_metaheuristic
    search_parameters.time_limit.seconds = time_limit_sec

    solution = routing.SolveWithParameters(search_parameters)
    if solution is None:
        raise RuntimeError("OR-Tools khong tim thay nghiem TSP hop le.")

    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))

    # route dang la [0, ..., k], khong bao gom depot cuoi.
    # Tinh chi phi thuc tren ma tran float goc.
    real_cost = _route_cost_float(route, dist_matrix)
    return route, real_cost


# =========================
# New evaluation using OR-Tools
# =========================
def evaluation3(
    grid,
    ntest=10,
    distance_scale=1000,
    time_limit_sec=5,
    first_solution_strategy=None,
    local_search_metaheuristic=None,
    map_path=None,
    smooth=True,
):
    """
    Pha 2 TSP bang OR-Tools thay cho ACO.
    grid.dijk la ma tran chi phi ngan nhat giua cac checkpoint tren grid map.
    """
    _require_ortools()

    resORT = []
    bestpath = None
    bestcost = float("inf")
    iterations_log = []

    for iter in range(ntest):
        print("Iteration ", iter + 1, "/", ntest)
        a = time.time()

        path, cost = solve_tsp_ortools(
            grid.dijk,
            distance_scale=distance_scale,
            time_limit_sec=time_limit_sec,
            first_solution_strategy=first_solution_strategy,
            local_search_metaheuristic=local_search_metaheuristic,
        )

        elapsed = time.time() - a
        resORT.append(cost)
        iterations_log.append({"iteration": iter + 1, "cost": cost, "time_sec": round(elapsed, 4)})

        if cost < bestcost:
            bestcost = cost
            bestpath = path.copy()

        print("Route (OR-Tools):", path)
        print("Cost (float):", cost)
        print(f"--- Iteration {iter+1}: {elapsed} seconds ---")

    resORT = np.array(resORT)
    print("Mean OR-Tools cost:", resORT.mean(), "Std OR-Tools:", resORT.std())
    print("Best cost:", bestcost)

    if not smooth:
        grid.twoPointTracing(smooth=False)   # rebuild cell-by-cell
    points = grid.getPath(bestpath)

    total, length, radiation, risk = grid.pathTotalCost(points)

    path_tuples = [(int(p[0]), int(p[1])) for p in points]
    path_label  = "turning points (smoothed)" if smooth else "cell-by-cell (full)"
    print(f"\nPath ({len(path_tuples)} bước, {path_label}):")
    print(path_tuples)

    print("\n===== Path metrics =====")
    print(f"  length(P)  = {length:.4f}   (cong thuc 3)")
    if grid.radiation_map is not None:
        print(f"  R(P)       = {radiation:.4f}   (cong thuc 6, a={grid.a}, v={grid.v})")
    else:
        print(f"  R(P)       = N/A  (chua nap radiation_map)")
    print(f"  risk(P)    = {risk:.4f}   (cong thuc 5)")
    print(f"  Total cost = {total:.4f}")
    print(f"  (w1={grid.w1:.2f}*length + w2={grid.w2:.2f}*R + w3={grid.w3:.2f}*risk)")

    # Khôi phục smooth=True để drawPath hoạt động bình thường
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
                "mean_cost": round(float(resORT.mean()), 6),
                "std_cost":  round(float(resORT.std()),  6),
                "best_cost": round(float(bestcost),      6),
            },
            "metrics": {
                "length":    round(float(length),    6),
                "radiation": round(float(radiation), 6) if grid.radiation_map is not None else None,
                "risk":      round(float(risk),      6),
                "total_cost":round(float(total),     6),
            },
            "path": path_tuples,
            "iterations": iterations_log,
        }
        _save_run(map_path, config_snapshot, run_data)

    grid.drawPath(points)
    grid.drawFMComponent(rmv=[0])
    grid.drawDijkstraWave(rmv=[])

    return bestpath, bestcost



def ADR_main(
    grid,
    ntest=1,
    distance_scale=1000,
    time_limit_sec=5,
    first_solution_strategy=None,
    local_search_metaheuristic=None,
    map_path=None,
    smooth=True,
):
    """
    Quy trinh day du cho bai toan grid map:
    1) Build graph tren grid
    2) Dung Dijkstra/FMF de tao ma tran chi phi giua cac checkpoint
    3) Dung OR-Tools giai pha 2 TSP
    """
    timeEval(grid)
    return evaluation3(
        grid,
        ntest=ntest,
        distance_scale=distance_scale,
        time_limit_sec=time_limit_sec,
        first_solution_strategy=first_solution_strategy,
        local_search_metaheuristic=local_search_metaheuristic,
        map_path=map_path,
        smooth=smooth,
    )


# Giu ten cu de code cu van chay, nhung mac dinh goi OR-Tools.
def ADF(grid):
    return ADR_main(grid)



def main():
    grid = My_grid.GridMap(20)

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

    # Nạp bản đồ (radiation_grid.txt trong cùng thư mục được nạp tự động)
    grid.get_grid_from_file(CONFIG["map_path"])
    print(f"Loaded: {CONFIG['map_path']}")
    print(f"Map size: {grid.mapSize}x{grid.mapSize},  checkpoints: {grid.npos}")
    for pos in grid.deslist:
        print(" ", pos)
    print()

    ADR_main(
        grid,
        ntest=CONFIG["ntest"],
        distance_scale=CONFIG["distance_scale"],
        time_limit_sec=CONFIG["time_limit_sec"],
        first_solution_strategy=(
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
            if routing_enums_pb2 is not None else None
        ),
        local_search_metaheuristic=(
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
            if routing_enums_pb2 is not None else None
        ),
        map_path=CONFIG["map_path"],
        smooth=CONFIG["smooth"],
    )


if __name__ == "__main__":
    main()
