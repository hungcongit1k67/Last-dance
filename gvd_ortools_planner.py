
from __future__ import annotations

import math
import subprocess
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from gvd_pathfinding import (
    INF,
    Point,
    euclid,
    precompute_base_gvd_index,
    shortest_path_via_prebuilt_gvd,
)


def _ensure_ortools():
    try:
        from ortools.constraint_solver import pywrapcp, routing_enums_pb2  # type: ignore
        return pywrapcp, routing_enums_pb2
    except ImportError:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "ortools"])
        from ortools.constraint_solver import pywrapcp, routing_enums_pb2  # type: ignore
        return pywrapcp, routing_enums_pb2


DISTANCE_SCALE_DEFAULT = 1000
BIG_M_DEFAULT = 10**12


def extract_goals_from_grid(grid_map: np.ndarray, goal_value: int = 2) -> List[Point]:
    """Extract goal cells from a grid map in row-major order."""
    goals = np.argwhere(np.asarray(grid_map) == goal_value)
    return [tuple(map(int, p)) for p in goals]


def reverse_path(path: Sequence[Point]) -> List[Point]:
    return list(reversed(path)) if path else []


def merge_segments(segments: Sequence[Sequence[Point]]) -> List[Point]:
    total_path: List[Point] = []
    for i, segment in enumerate(segments):
        segment = list(segment)
        if not segment:
            return []
        if i == 0:
            total_path.extend(segment)
        else:
            total_path.extend(segment[1:])
    return total_path


def build_pairwise_shortest_paths_gvd(
    gvd_index: dict,
    points: Sequence[Point],
    max_connections: int = 8,
    verbose: bool = True,
) -> Dict[Tuple[Point, Point], Tuple[List[Point], float]]:
    """
    Compute shortest paths between every unordered pair in `points`
    using Dijkstra on the prebuilt GVD graph.
    """
    points = list(dict.fromkeys(points))
    pairwise_cache: Dict[Tuple[Point, Point], Tuple[List[Point], float]] = {}

    for i in range(len(points)):
        a = points[i]
        pairwise_cache[(a, a)] = ([a], 0.0)
        for j in range(i + 1, len(points)):
            b = points[j]
            res = shortest_path_via_prebuilt_gvd(
                gvd_index,
                a,
                b,
                max_connections=max_connections,
            )
            path_ab = res["path_nodes"]
            dist_ab = float(res["distance"])

            pairwise_cache[(a, b)] = (path_ab, dist_ab)
            if path_ab:
                pairwise_cache[(b, a)] = (reverse_path(path_ab), dist_ab)
            else:
                pairwise_cache[(b, a)] = ([], INF)

            if verbose:
                if dist_ab >= INF / 2:
                    print(f"GVD-Dijkstra({a} -> {b}) = INF (no path)")
                else:
                    print(f"GVD-Dijkstra({a} -> {b}) = {dist_ab:.3f}")

    return pairwise_cache


def build_cost_matrix(points: Sequence[Point], pairwise_cache: dict) -> List[List[float]]:
    n_points = len(points)
    matrix: List[List[float]] = [[INF for _ in range(n_points)] for _ in range(n_points)]
    for i, a in enumerate(points):
        for j, b in enumerate(points):
            if i == j:
                matrix[i][j] = 0.0
            else:
                d = float(pairwise_cache[(a, b)][1])
                matrix[i][j] = d if d < INF / 2 else INF
    return matrix


def build_integer_cost_matrix(
    points: Sequence[Point],
    pairwise_cache: dict,
    distance_scale: int = DISTANCE_SCALE_DEFAULT,
    big_m: int = BIG_M_DEFAULT,
) -> List[List[int]]:
    float_matrix = build_cost_matrix(points, pairwise_cache)
    int_matrix: List[List[int]] = []
    for row in float_matrix:
        int_row: List[int] = []
        for d in row:
            if d >= INF / 2:
                int_row.append(big_m)
            else:
                int_row.append(int(round(d * distance_scale)))
        int_matrix.append(int_row)
    return int_matrix


def solve_cycle_tsp_with_ortools(
    int_cost_matrix: Sequence[Sequence[int]],
    start_idx: int = 0,
    time_limit_sec: int = 10,
):
    pywrapcp, routing_enums_pb2 = _ensure_ortools()

    n_points = len(int_cost_matrix)
    manager = pywrapcp.RoutingIndexManager(n_points, 1, start_idx)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(int_cost_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = int(time_limit_sec)

    solution = routing.SolveWithParameters(search_parameters)
    if solution is None:
        return None, None

    index = routing.Start(0)
    visit_order: List[int] = []
    scaled_cost = 0

    while not routing.IsEnd(index):
        visit_order.append(manager.IndexToNode(index))
        next_index = solution.Value(routing.NextVar(index))
        scaled_cost += routing.GetArcCostForVehicle(index, next_index, 0)
        index = next_index

    return visit_order, scaled_cost


def solve_path_tsp_with_ortools(
    int_cost_matrix: Sequence[Sequence[int]],
    start_idx: int,
    end_idx: int,
    time_limit_sec: int = 10,
):
    pywrapcp, routing_enums_pb2 = _ensure_ortools()

    n_points = len(int_cost_matrix)
    manager = pywrapcp.RoutingIndexManager(n_points, 1, [start_idx], [end_idx])
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(int_cost_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = int(time_limit_sec)

    solution = routing.SolveWithParameters(search_parameters)
    if solution is None:
        return None, None

    index = routing.Start(0)
    visit_order: List[int] = []
    scaled_cost = 0

    while True:
        visit_order.append(manager.IndexToNode(index))
        if routing.IsEnd(index):
            break
        next_index = solution.Value(routing.NextVar(index))
        scaled_cost += routing.GetArcCostForVehicle(index, next_index, 0)
        index = next_index

    return visit_order, scaled_cost


def reconstruct_route_from_order(
    order_points: Sequence[Point],
    pairwise_cache: dict,
    close_cycle: bool = False,
) -> Tuple[List[Point], float]:
    if not order_points:
        return [], INF

    route_points = list(order_points)
    if close_cycle and len(route_points) >= 1:
        route_points = route_points + [route_points[0]]

    segments: List[List[Point]] = []
    total_length = 0.0

    for a, b in zip(route_points[:-1], route_points[1:]):
        seg_path, seg_cost = pairwise_cache[(a, b)]
        if seg_cost >= INF / 2 or not seg_path:
            return [], INF
        segments.append(seg_path)
        total_length += float(seg_cost)

    return merge_segments(segments), total_length


def build_goal_distance_matrix_via_gvd(
    grid_map: np.ndarray,
    goals: Optional[Sequence[Point]] = None,
    max_connections: int = 8,
    distance_scale: int = DISTANCE_SCALE_DEFAULT,
    verbose: bool = True,
):
    """
    Build all-pairs goal-to-goal distances by running Dijkstra on a prebuilt GVD graph.
    """
    if goals is None:
        goals = extract_goals_from_grid(grid_map)
    goals = list(dict.fromkeys(goals))
    if not goals:
        raise ValueError("No goals provided and no goal cells with value 2 found in grid_map.")

    gvd_index = precompute_base_gvd_index(grid_map)
    pairwise_cache = build_pairwise_shortest_paths_gvd(
        gvd_index,
        goals,
        max_connections=max_connections,
        verbose=verbose,
    )
    float_matrix = build_cost_matrix(goals, pairwise_cache)
    int_matrix = build_integer_cost_matrix(
        goals,
        pairwise_cache,
        distance_scale=distance_scale,
    )

    return {
        "goals": goals,
        "gvd_index": gvd_index,
        "pairwise_cache": pairwise_cache,
        "float_matrix": float_matrix,
        "int_matrix": int_matrix,
        "distance_scale": distance_scale,
    }


def find_shortest_cycle_with_goals_gvd_ortools(
    grid_map: np.ndarray,
    goals: Optional[Sequence[Point]] = None,
    start_node: Optional[Point] = None,
    max_connections: int = 8,
    distance_scale: int = DISTANCE_SCALE_DEFAULT,
    time_limit_sec: int = 10,
    verbose: bool = True,
):
    """
    Phase 1:
        Build the all-pairs distance matrix between goals using Dijkstra on a GVD graph.
    Phase 2:
        Use OR-Tools to find the best visiting cycle.
    Phase 3:
        Reconstruct the full cycle path by concatenating the Dijkstra-on-GVD paths
        between consecutive goals in the OR-Tools order.
    """
    prep = build_goal_distance_matrix_via_gvd(
        grid_map=grid_map,
        goals=goals,
        max_connections=max_connections,
        distance_scale=distance_scale,
        verbose=verbose,
    )
    goals = prep["goals"]
    pairwise_cache = prep["pairwise_cache"]
    int_matrix = prep["int_matrix"]

    if start_node is None:
        start_node = goals[0]
    if start_node not in goals:
        raise ValueError("start_node must belong to goals.")

    start_idx = goals.index(start_node)
    best_idx_order, scaled_objective = solve_cycle_tsp_with_ortools(
        int_cost_matrix=int_matrix,
        start_idx=start_idx,
        time_limit_sec=time_limit_sec,
    )

    if not best_idx_order:
        return {
            "full_path": [],
            "total_length": INF,
            "best_order": [],
            "goals": goals,
            "pairwise_cache": pairwise_cache,
            "float_matrix": prep["float_matrix"],
            "int_matrix": int_matrix,
            "scaled_objective": None,
            "solver": "OR-Tools",
            "gvd_index": prep["gvd_index"],
        }

    ordered_cycle_points = [goals[i] for i in best_idx_order]
    # keep the same style as your notebook: best_order excludes the repeated start
    best_order = ordered_cycle_points[1:]
    full_path, total_length = reconstruct_route_from_order(
        [start_node] + best_order,
        pairwise_cache,
        close_cycle=True,
    )

    return {
        "full_path": full_path,
        "total_length": total_length,
        "best_order": best_order,
        "ordered_cycle_points": ordered_cycle_points,
        "goals": goals,
        "pairwise_cache": pairwise_cache,
        "float_matrix": prep["float_matrix"],
        "int_matrix": int_matrix,
        "scaled_objective": scaled_objective,
        "solver": "OR-Tools",
        "gvd_index": prep["gvd_index"],
        "distance_scale": distance_scale,
        "time_limit_sec": time_limit_sec,
    }


def find_shortest_path_with_goals_gvd_ortools(
    grid_map: np.ndarray,
    start_node: Point,
    end_node: Point,
    mandatory_waypoints: Sequence[Point],
    max_connections: int = 8,
    distance_scale: int = DISTANCE_SCALE_DEFAULT,
    time_limit_sec: int = 10,
    verbose: bool = True,
):
    """
    Open-path variant:
    start_node != end_node allowed.

    Pairwise costs are still computed by Dijkstra on the prebuilt GVD graph.
    Then OR-Tools solves the visiting order, and finally the full route is
    reconstructed from the pairwise Dijkstra paths on the GVD graph.
    """
    mandatory_waypoints = list(dict.fromkeys(mandatory_waypoints))
    inner_waypoints = [wp for wp in mandatory_waypoints if wp != start_node and wp != end_node]
    inner_waypoints = list(dict.fromkeys(inner_waypoints))

    if start_node == end_node:
        return find_shortest_cycle_with_goals_gvd_ortools(
            grid_map=grid_map,
            goals=[start_node] + inner_waypoints,
            start_node=start_node,
            max_connections=max_connections,
            distance_scale=distance_scale,
            time_limit_sec=time_limit_sec,
            verbose=verbose,
        )

    points = [start_node] + inner_waypoints + [end_node]
    prep = build_goal_distance_matrix_via_gvd(
        grid_map=grid_map,
        goals=points,
        max_connections=max_connections,
        distance_scale=distance_scale,
        verbose=verbose,
    )
    pairwise_cache = prep["pairwise_cache"]
    int_matrix = prep["int_matrix"]

    best_idx_order, scaled_objective = solve_path_tsp_with_ortools(
        int_cost_matrix=int_matrix,
        start_idx=0,
        end_idx=len(points) - 1,
        time_limit_sec=time_limit_sec,
    )

    if not best_idx_order:
        return {
            "full_path": [],
            "total_length": INF,
            "best_order": [],
            "points": points,
            "pairwise_cache": pairwise_cache,
            "float_matrix": prep["float_matrix"],
            "int_matrix": int_matrix,
            "scaled_objective": None,
            "solver": "OR-Tools",
            "gvd_index": prep["gvd_index"],
        }

    ordered_points = [points[i] for i in best_idx_order]
    best_order = ordered_points[1:-1]
    full_path, total_length = reconstruct_route_from_order(
        [start_node] + best_order + [end_node],
        pairwise_cache,
        close_cycle=False,
    )

    return {
        "full_path": full_path,
        "total_length": total_length,
        "best_order": best_order,
        "ordered_points": ordered_points,
        "points": points,
        "pairwise_cache": pairwise_cache,
        "float_matrix": prep["float_matrix"],
        "int_matrix": int_matrix,
        "scaled_objective": scaled_objective,
        "solver": "OR-Tools",
        "gvd_index": prep["gvd_index"],
        "distance_scale": distance_scale,
        "time_limit_sec": time_limit_sec,
    }
