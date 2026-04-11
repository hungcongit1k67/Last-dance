from numpy.core.fromnumeric import shape
import My_grid as My_grid
import GA
import numpy as np
import time

try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2
except ImportError:  # pragma: no cover
    pywrapcp = None
    routing_enums_pb2 = None


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
):
    """
    Pha 2 TSP bang OR-Tools thay cho ACO.
    grid.dijk la ma tran chi phi ngan nhat giua cac checkpoint tren grid map.
    """
    _require_ortools()

    resORT = []
    bestpath = None
    bestcost = float("inf")

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

        # getPath cua grid se tu dong them lai dinh dau o cuoi,
        # nen path o day chi can moi dinh xuat hien 1 lan.
        resORT.append(cost)

        if cost < bestcost:
            bestcost = cost
            bestpath = path.copy()

        print("Route (OR-Tools):", path)
        print("Cost (float):", cost)
        print(f"--- Iteration {iter+1}: {time.time() - a} seconds ---")

    resORT = np.array(resORT)
    print("Mean OR-Tools cost:", resORT.mean(), "Std OR-Tools:", resORT.std())
    print("Best cost:", bestcost)

    points = grid.getPath(bestpath)
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
    )


# Giu ten cu de code cu van chay, nhung mac dinh goi OR-Tools.
def ADF(grid):
    return ADR_main(grid)



def main():
    mapSize = 20
    grid = My_grid.GridMap(mapSize)
    if 0 == 1:
        npos = 4
        grid.create_grid_map(npos)
    else:
        # grid.get_grid_from_file("mixed2002.txt")
        # grid.get_grid_from_file("square400.txt")
        # grid.get_grid_from_file("triangle300.txt")
        # grid.get_grid_from_file("obstacle80.txt")
        # grid.get_grid_from_file("example1.txt")
        # grid.get_grid_from_file("warehouse4.txt")
        # grid.get_grid_from_file("map20_4.txt")
        # grid.get_grid_from_file("map20_7.txt")
        # grid.get_grid_from_file("map35_11.txt")
        # grid.get_grid_from_file("map35_12.txt")
        # grid.get_grid_from_file("map35_13.txt")
        grid.get_grid_from_file("E:\\last_dance\\LastDance\FMF\\mixed200.txt")
        # grid.get_grid_from_file("map40_2.txt")
        # grid.get_grid_from_file("map40_12.txt")
        # grid.get_grid_from_file("map40_15.txt")
        # grid.get_grid_from_file("map80_1.txt")
        # grid.get_grid_from_file("map80_2.txt")
        # grid.get_grid_from_file("map101_00.txt")
        # grid.get_grid_from_file("map200_00.txt")
        # grid.get_grid_from_file("maptest1.txt")
        # grid.get_grid_from_file("maptest2.txt")

    for pos in grid.deslist:
        print(pos)
    print("\n\n\n")

    ADR_main(
        grid,
        ntest=1,
        distance_scale=1000,
        time_limit_sec=5,
        first_solution_strategy=(
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
            if routing_enums_pb2 is not None else None
        ),
        local_search_metaheuristic=(
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
            if routing_enums_pb2 is not None else None
        ),
    )


if __name__ == "__main__":
    main()
