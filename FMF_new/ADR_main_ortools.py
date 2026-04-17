"""
ADR_main_ortools.py - Entry point cho pipeline WP-FMF + OR-Tools TSP
======================================================================
Pha 1: WP-FMF (Weighted Potential Fast Marching Firework) -> ma trận chi phí
Pha 2: OR-Tools Routing Solver -> giải TSP trên ma trận đó

Cách dùng nhanh:
    python ADR_main_ortools.py
Hoặc import hàm run_wpfmf_pipeline(...) để gọi trong notebook.
"""

import os
import time
import numpy as np

import My_grid as My_grid

try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2
except ImportError:
    pywrapcp = None
    routing_enums_pb2 = None


# =========================================================
# Timing helper
# =========================================================
def timeEval(grid, w1=0.7, C1=0.5):
    start = time.time()
    grid.buildGraphAdvanced(w1=w1, C1=C1)
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
                    draw=True):
    """
    Chay OR-Tools ntest lan tren grid.dijk (da duoc tinh bang WP-FMF).
    Tra ve (best_path, best_cost).
    Ngoai ra, in ra chieu dai va risk cua duong di tot nhat theo cong thuc (3) va (5).
    """
    _require_ortools()

    res = []
    best_path = None
    best_cost = float("inf")

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
        res.append(cost)
        if cost < best_cost:
            best_cost = cost
            best_path = list(path)

        print("  Route (OR-Tools):", path)
        print(f"  Weighted cost:   {cost:.4f}")
        print(f"  --- Iteration {it + 1}: {time.time() - t0:.3f} seconds ---")

    res_arr = np.array(res)
    print("\n===== Phase 2 summary =====")
    print(f"Mean weighted cost: {res_arr.mean():.4f}")
    print(f"Std  weighted cost: {res_arr.std():.4f}")
    print(f"Best weighted cost: {best_cost:.4f}")

    # Bung permutation thanh chuoi o thuc te
    cells = grid.getPath(best_path)
    length = grid.pathLength(cells)
    risk = grid.pathRisk(cells)
    print("\n===== Path metrics (thuc te tren grid) =====")
    print(f"length(P) = {length:.4f}  (cong thuc 3)")
    if risk is not None:
        print(f"risk(P)   = {risk:.4f}   (cong thuc 5)")

    if draw:
        grid.drawPath(cells)
        grid.drawFMComponent()
        grid.drawDijkstraWave()
        try:
            grid.drawSafety()
            grid.drawFCost()
        except AttributeError:
            pass

    return best_path, best_cost


# =========================================================
# Full pipeline wrapper
# =========================================================
def run_wpfmf_pipeline(grid,
                      w1=1,
                      C1=0.5,
                      ntest=1,
                      distance_scale=1000,
                      time_limit_sec=5,
                      first_solution_strategy=None,
                      local_search_metaheuristic=None,
                      draw=True):
    """
    Pipeline day du:
      Pha 1: WP-FMF   -> grid.dijk (ma tran chi phi giua cac checkpoint)
      Pha 2: OR-Tools -> thu tu tham toi uu
    w1: trong so giua length (w1=1) va risk (w1=0)
    C1: trong so N_obs (C1=1) vs d_min (C1=0) trong S(c)
    """
    print(f"===== WP-FMF Pipeline (w1={w1}, C1={C1}) =====")
    print("[Phase 1] Building WP-FMF cost matrix ...")
    timeEval(grid, w1=w1, C1=C1)

    print("\n[Phase 2] OR-Tools TSP on cost matrix ...")
    return evaluation_wpfmf(
        grid,
        ntest=ntest,
        distance_scale=distance_scale,
        time_limit_sec=time_limit_sec,
        first_solution_strategy=first_solution_strategy,
        local_search_metaheuristic=local_search_metaheuristic,
        draw=draw,
    )


# Giu ten ham cu cho tuong thich
def ADR_main(grid,
             ntest=1,
             distance_scale=1000,
             time_limit_sec=5,
             first_solution_strategy=None,
             local_search_metaheuristic=None,
             w1=0.7,
             C1=0.5):
    return run_wpfmf_pipeline(
        grid,
        w1=w1,
        C1=C1,
        ntest=ntest,
        distance_scale=distance_scale,
        time_limit_sec=time_limit_sec,
        first_solution_strategy=first_solution_strategy,
        local_search_metaheuristic=local_search_metaheuristic,
    )


def ADF(grid):
    return ADR_main(grid)


# =========================================================
# Main
# =========================================================
def main():
    grid = My_grid.GridMap(mapSize=20)

    # Chinh duong dan map cho phu hop may cua ban.
    # Cac file map nam trong thu muc FMF/ cua repo.

    map_path = r"E:\last_dance\LastDance\FMF\square400.txt"
    # candidates = [
    #     "warehouse2.txt",
    #     "./FMF/warehouse2.txt",
    #     r"E:\last_dance\LastDance\FMF\warehouse2.txt",
    # ]
    # map_path = None
    # for p in candidates:
    #     if os.path.exists(p):
    #         map_path = p
    #         break
    # if map_path is None:
    #     map_path = candidates[0]
    #     print(f"Khong tim thay map. Su dung {map_path} (co the bao loi).")

    grid.get_grid_from_file(map_path)
    print(f"Loaded {map_path}: {grid.mapSize}x{grid.mapSize}, {grid.npos} checkpoints")
    for pos in grid.deslist:
        print(" ", pos)
    print()

    # Chinh w1, C1 o day de thay nghiem thay doi
    run_wpfmf_pipeline(
        grid,
        w1=0.7,      # 0.7 length + 0.3 safety (giam w1 -> an toan hon, dai hon)
        C1=0.5,      # can bang N_obs va d_min
        ntest=1,
        distance_scale=1000,
        time_limit_sec=5,
        first_solution_strategy=(
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
            if routing_enums_pb2 is not None else None),
        local_search_metaheuristic=(
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
            if routing_enums_pb2 is not None else None),
    )


if __name__ == "__main__":
    main()
