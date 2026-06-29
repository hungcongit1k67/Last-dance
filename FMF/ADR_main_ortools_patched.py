# -*- coding: utf-8 -*-
import sys
sys.stdout.reconfigure(encoding='utf-8')

import My_grid_patched as My_grid
import GA
import aco as aco_module
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
    "w1": 0.3,      # Chiều dài đường đi  length(P)
    "w2": 0.5,      # Độ rủi ro phóng xạ  R(P)
    "w3": 0.2,      # Độ rủi ro va chạm   risk(P)

    # --- Tham số an toàn va chạm S(c) (công thức 4) ---
    "C1": 0.5,      # C1→1: ưu tiên N_obs;  C1→0: ưu tiên d_min
    "safety_radius": 5.0,        # bán kính vùng lân cận tính S(c)
    "safety_max_distance": 7.0,  # khoảng cách chuẩn hóa d_min trong S(c)

    # --- Tham số vật lý (công thức 6 — tính R(P)) ---
    "a": 1.0,       # Kích thước ô lưới (m)
    "v": 1.0,       # Vận tốc robot (m/s)   

    # --- Bước chi phí Eikonal ---
    # False → dis dùng hằng số 2 (FMF gốc).
    # True  → dis dùng 2·f(x)², f(x)=w1+w2·R̄_norm(x)+w3·(1−S(x)).  (Eikonal có trọng số |∇T|=f)
    "cost_step": False,

    # --- Hàm mục tiêu của TSP solver ---
    # False → ma trận TSP = chiều dài hình học (length).
    # True  → ma trận TSP = Total cost (7a) w1·length+w2·R+w3·risk của từng đoạn
    #         → solver minimize đúng Total cost, và khi đó TSP cost == Total cost.
    "Solver_minimize": True,

    # --- Supercover line cho hàm mục tiêu ---
    # False → risk(P) & R(P) dùng công thức gốc (trung bình 2 đầu mút mỗi đoạn).
    # True  → khi smooth=True, đoạn nối hai turning points tính trung bình (1−S)/R̄
    #         trên mọi ô supercover line mà đoạn cắt qua; cell-by-cell vẫn dùng công thức gốc.
    "supercover": True,

    # --- Vùng đỏ phóng xạ (red flag) ---
    "RI_max": 8,        # Ngưỡng nồng độ phóng xạ tối đa
    # False → không thay đổi gì (hành vi gốc).
    # True  → mọi ô có nồng độ phóng xạ >= RI_max bị coi là vật cản,
    #         robot không được phép đi qua.
    "red_flag": True,

    # --- Đường dẫn bản đồ ---
    # radiation_grid.txt trong cùng thư mục sẽ được nạp tự động.
    #"map_path": r"E:\last_dance\LastDance\FMF\test_100\test_100.txt",
    #"map_path": r"E:\last_dance\LastDance\FMF\square400\square400.txt",
    #"map_path": r"E:\last_dance\LastDance\FMF\triangle300\triangle300.txt",
    #"map_path": r"E:\last_dance\LastDance\FMF\mixed2002\mixed2002.txt",
    #"map_path": r"E:\last_dance\LastDance\FMF\scenario4\scenario4_grid.txt",
    #"map_path": r"E:\last_dance\LastDance\FMF\factory400\factory400_30.txt",
    #"map_path": r"E:\last_dance\LastDance\FMF\mixed500\mixed500.txt",
    #"map_path": r"E:\last_dance\LastDance\FMF\scenario6\scenario6_grid.txt",
    #"map_path": r"E:\last_dance\LastDance\FMF\scenario7_2\scenario7_grid2.txt",
    #"map_path": r"E:\last_dance\LastDance\FMF\scenario7\scenario7_grid.txt",
    "map_path": r"E:\last_dance\LastDance\FMF\warehouse3\warehouse3.txt",

    # --- Bộ giải TSP ---
    # "ortools" | "aco" | "ga"
    "tsp": "aco",
    "ntest": 5,

    # --- Tham số OR-Tools TSP ---
    "distance_scale": 1000,
    "time_limit_sec": 5,

    # --- Tham số ACO TSP ---
    "aco_ant_count": 100, # sửa chỗ này để nặn 20
    "aco_generations": 200, # sửa chỗ này để nặn 100
    "aco_alpha": 1.0,
    "aco_beta": 5.0,
    "aco_rho": 0.5,
    "aco_q": 10,
    "aco_strategy": 0,    # 0=ant-cycle, 1=ant-quality, 2=ant-density

    # --- Tham số GA TSP ---
    "ga_population": 300,
    "ga_generations": 300,

    # --- Path output ---
    # True  → in/lưu path đã smooth (turning points, ~128 bước)
    # False → in/lưu full path cell-by-cell (đi qua từng ô lưới, ~1000+ bước)
    "smooth": False,
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
# ACO / GA wrappers
# =========================
def solve_tsp_aco(
    dist_matrix,
    ant_count=20,
    generations=100,
    alpha=1.0,
    beta=5.0,
    rho=0.5,
    q=10,
    strategy=0,
):
    """Giải TSP chu trình bằng Ant Colony Optimization."""
    n = len(dist_matrix)
    graph = aco_module.Graph([list(row) for row in dist_matrix], n)
    solver = aco_module.ACO(ant_count, generations, alpha, beta, rho, q, strategy)
    path, _ = solver.solve(graph)
    real_cost = _route_cost_float(path, dist_matrix)
    return path, real_cost


def solve_tsp_ga(grid, population=300, generations=300):
    """Giải TSP chu trình bằng Genetic Algorithm (dùng grid.dijk)."""
    ga = GA.GA(grid)
    path = ga.solve(population, generations)
    path = [int(x) for x in path]
    real_cost = _route_cost_float(path, grid.dijk)
    return path, real_cost


def _solve_tsp(grid, tsp_name, **params):
    """Dispatcher chọn bộ giải TSP theo tên."""
    name = (tsp_name or "ortools").lower()
    if name == "ortools":
        return solve_tsp_ortools(
            grid.dijk,
            distance_scale=params.get("distance_scale", 1000),
            time_limit_sec=params.get("time_limit_sec", 5),
            first_solution_strategy=params.get("first_solution_strategy"),
            local_search_metaheuristic=params.get("local_search_metaheuristic"),
        )
    if name == "aco":
        return solve_tsp_aco(
            grid.dijk,
            ant_count=params.get("aco_ant_count", 20),
            generations=params.get("aco_generations", 100),
            alpha=params.get("aco_alpha", 1.0),
            beta=params.get("aco_beta", 5.0),
            rho=params.get("aco_rho", 0.5),
            q=params.get("aco_q", 10),
            strategy=params.get("aco_strategy", 0),
        )
    if name == "ga":
        return solve_tsp_ga(
            grid,
            population=params.get("ga_population", 300),
            generations=params.get("ga_generations", 300),
        )
    raise ValueError(f"Khong biet tsp solver: {tsp_name!r}. Chon: 'ortools' | 'aco' | 'ga'.")


# =========================
# New evaluation using OR-Tools
# =========================
def evaluation3(
    grid,
    tsp="ortools",
    ntest=10,
    distance_scale=1000,
    time_limit_sec=5,
    first_solution_strategy=None,
    local_search_metaheuristic=None,
    aco_ant_count=20,
    aco_generations=100,
    aco_alpha=1.0,
    aco_beta=5.0,
    aco_rho=0.5,
    aco_q=10,
    aco_strategy=0,
    ga_population=300,
    ga_generations=300,
    map_path=None,
    smooth=True,
):
    """
    Pha 2 TSP — dispatch theo tham số `tsp`:
      "ortools" | "aco" | "ga"
    grid.dijk la ma tran chi phi ngan nhat giua cac checkpoint tren grid map.
    """
    if tsp == "ortools":
        _require_ortools()

    solver_params = dict(
        distance_scale=distance_scale,
        time_limit_sec=time_limit_sec,
        first_solution_strategy=first_solution_strategy,
        local_search_metaheuristic=local_search_metaheuristic,
        aco_ant_count=aco_ant_count,
        aco_generations=aco_generations,
        aco_alpha=aco_alpha,
        aco_beta=aco_beta,
        aco_rho=aco_rho,
        aco_q=aco_q,
        aco_strategy=aco_strategy,
        ga_population=ga_population,
        ga_generations=ga_generations,
    )

    res_costs = []
    res_lengths = []
    res_radiations = []
    res_risks = []
    res_totals = []
    res_t_phase1 = []   # thời gian Pha 1 (build graph + Dijkstra/FMF) mỗi iteration
    res_t_phase2 = []   # thời gian Pha 2 (TSP solver) mỗi iteration
    res_t_total  = []   # thời gian cả thuật toán (Pha 1 + Pha 2) mỗi iteration
    bestpath = None
    bestcost = float("inf")
    iterations_log = []
    has_radiation = grid.radiation_map is not None

    tsp_label = tsp.upper()
    for iter in range(ntest):
        print("Iteration ", iter + 1, "/", ntest)

        # --- Pha 1: build graph trên grid + tạo ma trận chi phí (Dijkstra/FMF) ---
        t_p1 = time.time()
        grid.buildGraphAdvanced()
        if not smooth:
            # twoPointTracing tính lại adj theo pathTrace mới (quan trọng khi Solver_minimize=True);
            # chạy lại dijkstra để dijk/dtra khớp pathTrace → giữ TSP cost == Total cost.
            grid.twoPointTracing(smooth=False)
            grid.dijkstra()
        t_phase1 = time.time() - t_p1

        # --- Pha 2: giải TSP ---
        a = time.time()
        path, cost = _solve_tsp(grid, tsp, **solver_params)
        t_phase2 = time.time() - a

        t_total = t_phase1 + t_phase2

        # Tính metrics cho từng iteration để lấy mean/std
        points_i = grid.getPath(path)
        total_i, length_i, rad_i, risk_i = grid.pathTotalCost(points_i)

        res_costs.append(cost)
        res_lengths.append(length_i)
        res_radiations.append(rad_i if rad_i is not None else 0.0)
        res_risks.append(risk_i)
        res_totals.append(total_i)
        res_t_phase1.append(t_phase1)
        res_t_phase2.append(t_phase2)
        res_t_total.append(t_total)
        iterations_log.append({
            "iteration": iter + 1,
            "cost": cost,
            "length": round(float(length_i), 6),
            "radiation": round(float(rad_i), 6) if has_radiation else None,
            "risk": round(float(risk_i), 6),
            "total_cost": round(float(total_i), 6),
            "time_phase1_sec": round(t_phase1, 4),
            "time_phase2_sec": round(t_phase2, 4),
            "time_total_sec": round(t_total, 4),
        })

        if cost < bestcost:
            bestcost = cost
            bestpath = list(path)

        print(f"Route ({tsp_label}):", path)
        print(f"Cost: {cost:.4f}   length={length_i:.4f}   "
              f"R={rad_i if rad_i is not None else 0.0:.4f}   "
              f"risk={risk_i:.4f}   total={total_i:.4f}")
        print(f"--- Iteration {iter+1}: pha1={t_phase1:.4f}s  "
              f"pha2={t_phase2:.4f}s  tong={t_total:.4f}s ---")

    res_costs      = np.array(res_costs)
    res_lengths    = np.array(res_lengths)
    res_radiations = np.array(res_radiations)
    res_risks      = np.array(res_risks)
    res_totals     = np.array(res_totals)
    res_t_phase1   = np.array(res_t_phase1)
    res_t_phase2   = np.array(res_t_phase2)
    res_t_total    = np.array(res_t_total)

    # Path tốt nhất (theo TSP cost) — dùng cho vẽ và lưu path_tuples
    points = grid.getPath(bestpath)
    total, length, radiation, risk = grid.pathTotalCost(points)

    path_tuples = [(int(p[0]), int(p[1])) for p in points]
    path_label  = "turning points (smoothed)" if smooth else "cell-by-cell (full)"
    print(f"\nPath ({len(path_tuples)} bước, {path_label}):")
    print(path_tuples)

    print(f"\n===== Path metrics  (mean ± std qua {ntest} lần chạy, best theo TSP cost) =====")
    print(f"  length(P)  : {res_lengths.mean():10.4f} ± {res_lengths.std():.4f}    "
          f"(best: {length:.4f})   (cong thuc 3)")
    if has_radiation:
        print(f"  R(P)       : {res_radiations.mean():10.4f} ± {res_radiations.std():.4f}    "
              f"(best: {radiation:.4f})   (cong thuc 6, a={grid.a}, v={grid.v})")
    else:
        print(f"  R(P)       : N/A  (chua nap radiation_map)")
    print(f"  risk(P)    : {res_risks.mean():10.4f} ± {res_risks.std():.4f}    "
          f"(best: {risk:.4f})   (cong thuc 5)")
    print(f"  Total cost : {res_totals.mean():10.4f} ± {res_totals.std():.4f}    "
          f"(best: {total:.4f})")
    print(f"  TSP cost   : {res_costs.mean():10.4f} ± {res_costs.std():.4f}    "
          f"(best: {bestcost:.4f})")
    print(f"  (w1={grid.w1:.2f}*length + w2={grid.w2:.2f}*R + w3={grid.w3:.2f}*risk)")

    print(f"\n===== Thoi gian chay  (mean ± std qua {ntest} lan chay) =====")
    print(f"  Pha 1 (build graph)   : {res_t_phase1.mean():10.4f} ± {res_t_phase1.std():.4f} s")
    print(f"  Pha 2 (TSP {tsp_label:<7}) : {res_t_phase2.mean():10.4f} ± {res_t_phase2.std():.4f} s")
    print(f"  Ca thuat toan         : {res_t_total.mean():10.4f} ± {res_t_total.std():.4f} s")

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
            "cost_step": grid.cost_step,
            "Solver_minimize": grid.Solver_minimize,
            "RI_max": grid.RI_max,
            "red_flag": grid.red_flag,
            "tsp": tsp, "ntest": ntest,
            "distance_scale": distance_scale,
            "time_limit_sec": time_limit_sec,
            "aco_ant_count": aco_ant_count,
            "aco_generations": aco_generations,
            "aco_alpha": aco_alpha, "aco_beta": aco_beta,
            "aco_rho": aco_rho, "aco_q": aco_q,
            "aco_strategy": aco_strategy,
            "ga_population": ga_population,
            "ga_generations": ga_generations,
            "map_size": grid.mapSize, "checkpoints": grid.npos,
            "smooth": smooth,
        }
        run_data = {
            "summary": {
                "tsp_cost":   {"mean": round(float(res_costs.mean()),      6),
                               "std":  round(float(res_costs.std()),       6),
                               "best": round(float(bestcost),              6)},
                "length":     {"mean": round(float(res_lengths.mean()),    6),
                               "std":  round(float(res_lengths.std()),     6),
                               "best": round(float(length),                6)},
                "radiation":  ({"mean": round(float(res_radiations.mean()), 6),
                                "std":  round(float(res_radiations.std()),  6),
                                "best": round(float(radiation),             6)}
                               if has_radiation else None),
                "risk":       {"mean": round(float(res_risks.mean()),      6),
                               "std":  round(float(res_risks.std()),       6),
                               "best": round(float(risk),                  6)},
                "total_cost": {"mean": round(float(res_totals.mean()),     6),
                               "std":  round(float(res_totals.std()),      6),
                               "best": round(float(total),                 6)},
                "time_phase1": {"mean": round(float(res_t_phase1.mean()),  6),
                                "std":  round(float(res_t_phase1.std()),   6)},
                "time_phase2": {"mean": round(float(res_t_phase2.mean()),  6),
                                "std":  round(float(res_t_phase2.std()),   6)},
                "time_total":  {"mean": round(float(res_t_total.mean()),   6),
                                "std":  round(float(res_t_total.std()),    6)},
            },
            "best_metrics": {
                "length":    round(float(length),    6),
                "radiation": round(float(radiation), 6) if has_radiation else None,
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
    tsp="ortools",
    ntest=1,
    distance_scale=1000,
    time_limit_sec=5,
    first_solution_strategy=None,
    local_search_metaheuristic=None,
    aco_ant_count=20,
    aco_generations=100,
    aco_alpha=1.0,
    aco_beta=5.0,
    aco_rho=0.5,
    aco_q=10,
    aco_strategy=0,
    ga_population=300,
    ga_generations=300,
    map_path=None,
    smooth=True,
):
    """
    Quy trinh day du cho bai toan grid map:
    1) Build graph tren grid
    2) Dung Dijkstra/FMF de tao ma tran chi phi giua cac checkpoint
    3) Dung TSP solver (chon bang tham so `tsp`): ortools | aco | ga

    Pha 1 (build graph + Dijkstra/FMF) duoc do thoi gian ben trong evaluation3,
    chay lai moi iteration de lay mean/std thoi gian.
    """
    return evaluation3(
        grid,
        tsp=tsp,
        ntest=ntest,
        distance_scale=distance_scale,
        time_limit_sec=time_limit_sec,
        first_solution_strategy=first_solution_strategy,
        local_search_metaheuristic=local_search_metaheuristic,
        aco_ant_count=aco_ant_count,
        aco_generations=aco_generations,
        aco_alpha=aco_alpha,
        aco_beta=aco_beta,
        aco_rho=aco_rho,
        aco_q=aco_q,
        aco_strategy=aco_strategy,
        ga_population=ga_population,
        ga_generations=ga_generations,
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
        cost_step=CONFIG["cost_step"],
        Solver_minimize=CONFIG["Solver_minimize"],
        supercover=CONFIG["supercover"],
        RI_max=CONFIG["RI_max"],
        red_flag=CONFIG["red_flag"],
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
        tsp=CONFIG["tsp"],
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
        aco_ant_count=CONFIG["aco_ant_count"],
        aco_generations=CONFIG["aco_generations"],
        aco_alpha=CONFIG["aco_alpha"],
        aco_beta=CONFIG["aco_beta"],
        aco_rho=CONFIG["aco_rho"],
        aco_q=CONFIG["aco_q"],
        aco_strategy=CONFIG["aco_strategy"],
        ga_population=CONFIG["ga_population"],
        ga_generations=CONFIG["ga_generations"],
        map_path=CONFIG["map_path"],
        smooth=CONFIG["smooth"],
    )


if __name__ == "__main__":
    main()
