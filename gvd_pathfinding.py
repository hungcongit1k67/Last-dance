from __future__ import annotations

import heapq
import math
from collections import defaultdict
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import convolve
from skimage.morphology import medial_axis

Point = Tuple[int, int]
Graph = Dict[Point, Dict[Point, float]]
MOVES8 = [(-1, -1), (-1, 0), (-1, 1),
          (0, -1),           (0, 1),
          (1, -1),  (1, 0),  (1, 1)]
INF = 1e18


def euclid(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def in_bounds(p: Point, n_rows: int, n_cols: int) -> bool:
    return 0 <= p[0] < n_rows and 0 <= p[1] < n_cols


def _prepare_grid(grid_map: np.ndarray) -> np.ndarray:
    work = np.array(grid_map, copy=True)
    work[work == 2] = 0
    return work


def build_gvd_from_grid(grid_map: np.ndarray):
    """
    Build a raster approximation of the Voronoi Boundary (VB) using the medial axis.

    Parameters
    ----------
    grid_map : np.ndarray
        0 = free, 1 = obstacle, 2 = goal(optional, treated as free)

    Returns
    -------
    dict
        free_mask, gvd_mask, distance, neighbor_count, branch_mask, end_mask
    """
    work = _prepare_grid(grid_map)
    free_mask = (work == 0)
    gvd_mask, distance = medial_axis(free_mask, return_distance=True)

    neighbor_count = convolve(
        gvd_mask.astype(np.uint8),
        np.ones((3, 3), dtype=np.uint8),
        mode="constant",
        cval=0,
    ) - gvd_mask.astype(np.uint8)

    branch_mask = gvd_mask & (neighbor_count >= 3)
    end_mask = gvd_mask & (neighbor_count == 1)

    return {
        "free_mask": free_mask,
        "gvd_mask": gvd_mask,
        "distance": distance,
        "neighbor_count": neighbor_count,
        "branch_mask": branch_mask,
        "end_mask": end_mask,
    }


def build_gvd_pixel_graph(gvd_mask: np.ndarray) -> Graph:
    """
    Build an 8-neighbor weighted graph whose vertices are GVD pixels.

    This is simpler and more robust than immediately compressing the GVD into a
    branch-point graph. For your current goal (Dijkstra on GVD instead of on the
    full grid), this representation is already much smaller than the grid graph.
    """
    pts = np.argwhere(gvd_mask)
    graph: Graph = {}
    if len(pts) == 0:
        return graph

    point_set = {tuple(map(int, p)) for p in pts}
    for p in point_set:
        graph[p] = {}
        for dr, dc in MOVES8:
            q = (p[0] + dr, p[1] + dc)
            if q in point_set:
                graph[p][q] = math.hypot(dr, dc)
    return graph


def _supercover_line(a: Point, b: Point) -> List[Point]:
    """Return grid cells touched by segment a->b using dense sampling."""
    r0, c0 = a
    r1, c1 = b
    dr = r1 - r0
    dc = c1 - c0
    steps = int(max(abs(dr), abs(dc)) * 4) + 1
    cells = []
    seen = set()
    for t in np.linspace(0.0, 1.0, steps):
        r = int(round(r0 + dr * t))
        c = int(round(c0 + dc * t))
        cell = (r, c)
        if cell not in seen:
            seen.add(cell)
            cells.append(cell)
    if cells[-1] != b:
        cells.append(b)
    return cells


def collision_free_segment(grid_map: np.ndarray, a: Point, b: Point, forbid_corner_cut: bool = True) -> bool:
    """
    Check whether straight segment a->b stays in free cells.

    If forbid_corner_cut is True, diagonal jumps between two obstacle corners are
    also rejected.
    """
    work = _prepare_grid(grid_map)
    n_rows, n_cols = work.shape
    cells = _supercover_line(a, b)

    for p in cells:
        if not in_bounds(p, n_rows, n_cols) or work[p] == 1:
            return False

    if forbid_corner_cut:
        for p, q in zip(cells[:-1], cells[1:]):
            dr = q[0] - p[0]
            dc = q[1] - p[1]
            if abs(dr) == 1 and abs(dc) == 1:
                side1 = (p[0], q[1])
                side2 = (q[0], p[1])
                if in_bounds(side1, n_rows, n_cols) and in_bounds(side2, n_rows, n_cols):
                    if work[side1] == 1 and work[side2] == 1:
                        return False
    return True


def _select_connection_pixels(
    grid_map: np.ndarray,
    gvd_mask: np.ndarray,
    distance_to_obstacle: np.ndarray,
    point: Point,
    max_connections: int = 8,
    tol_start: float = 1.5,
    tol_step: float = 0.75,
    tol_max: float = 4.5,
) -> List[Point]:
    """
    Approximate Algorithm 1 from the paper on a raster map.

    After adding start/goal as obstacle pixels, points on the modified VB around the
    inserted point satisfy approximately
        d(v, point) ~= d(v, obstacle_region).
    We therefore search GVD pixels satisfying this equality (within tolerance) and
    with a collision-free straight connection to the query point.
    """
    gvd_points = [tuple(map(int, p)) for p in np.argwhere(gvd_mask)]
    if not gvd_points:
        return []

    candidates: List[Tuple[float, float, Point]] = []
    tol = tol_start
    while tol <= tol_max and not candidates:
        for v in gvd_points:
            dv = euclid(point, v)
            clearance = float(distance_to_obstacle[v])
            if abs(dv - clearance) <= tol and collision_free_segment(grid_map, point, v):
                candidates.append((dv, abs(dv - clearance), v))
        tol += tol_step

    if not candidates:
        # Fallback: just use visible closest GVD pixels.
        for v in gvd_points:
            if collision_free_segment(grid_map, point, v):
                candidates.append((euclid(point, v), 0.0, v))

    candidates.sort(key=lambda x: (x[0], x[1]))
    selected: List[Point] = []
    used = set()
    for _, _, v in candidates:
        if v not in used:
            used.add(v)
            selected.append(v)
        if len(selected) >= max_connections:
            break
    return selected


def connect_start_goal_algorithm1(
    grid_map: np.ndarray,
    start_point: Point,
    goal_point: Point,
    max_connections: int = 8,
):
    """
    Raster adaptation of Algorithm 1 (Connection of the Start and Goal Point).

    Steps:
    1. Add start and goal as temporary obstacle pixels.
    2. Recompute the modified GVD.
    3. Find visible GVD pixels around start and goal satisfying the paper's circle
       condition approximately.
    4. Add START/GOAL connections to the GVD graph.

    Returns a dict containing the modified GVD and the augmented graph.
    """
    work = _prepare_grid(grid_map)
    n_rows, n_cols = work.shape
    for p, name in [(start_point, "start_point"), (goal_point, "goal_point")]:
        if not in_bounds(p, n_rows, n_cols):
            raise ValueError(f"{name}={p} is outside the grid")
        if work[p] == 1:
            raise ValueError(f"{name}={p} lies on an obstacle")

    modified = work.copy()
    modified[start_point] = 1
    modified[goal_point] = 1

    gvd = build_gvd_from_grid(modified)
    graph = build_gvd_pixel_graph(gvd["gvd_mask"])

    start_links = _select_connection_pixels(
        work,
        gvd["gvd_mask"],
        gvd["distance"],
        start_point,
        max_connections=max_connections,
    )
    goal_links = _select_connection_pixels(
        work,
        gvd["gvd_mask"],
        gvd["distance"],
        goal_point,
        max_connections=max_connections,
    )

    graph[start_point] = {}
    graph[goal_point] = {}

    for v in start_links:
        w = euclid(start_point, v)
        graph[start_point][v] = w
        graph.setdefault(v, {})[start_point] = w

    for v in goal_links:
        w = euclid(goal_point, v)
        graph[goal_point][v] = w
        graph.setdefault(v, {})[goal_point] = w

    # Optional direct connection if visible
    if collision_free_segment(work, start_point, goal_point):
        w = euclid(start_point, goal_point)
        graph[start_point][goal_point] = w
        graph[goal_point][start_point] = w

    return {
        "modified_grid": modified,
        "gvd": gvd,
        "graph": graph,
        "start_links": start_links,
        "goal_links": goal_links,
    }


def dijkstra_on_graph(graph: Graph, start: Point, goal: Point) -> Tuple[List[Point], float]:
    if start not in graph or goal not in graph:
        return [], INF

    dist: Dict[Point, float] = {node: INF for node in graph}
    prev: Dict[Point, Optional[Point]] = {node: None for node in graph}
    dist[start] = 0.0
    pq: List[Tuple[float, Point]] = [(0.0, start)]

    while pq:
        cur_dist, u = heapq.heappop(pq)
        if cur_dist > dist[u]:
            continue
        if u == goal:
            break
        for v, w in graph[u].items():
            nd = cur_dist + w
            if nd < dist.get(v, INF):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if dist.get(goal, INF) >= INF / 2:
        return [], INF

    path: List[Point] = []
    cur: Optional[Point] = goal
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path, dist[goal]


def shortest_path_via_gvd(
    grid_map: np.ndarray,
    start_point: Point,
    goal_point: Point,
    max_connections: int = 8,
):
    """
    Main convenience wrapper.

    Returns
    -------
    dict with keys:
        path_nodes: node path on the augmented GVD graph
        distance: path cost on that graph
        connection_info: output of connect_start_goal_algorithm1
    """
    conn = connect_start_goal_algorithm1(
        grid_map,
        start_point,
        goal_point,
        max_connections=max_connections,
    )
    path_nodes, dist = dijkstra_on_graph(conn["graph"], start_point, goal_point)
    return {
        "path_nodes": path_nodes,
        "distance": dist,
        "connection_info": conn,
    }


def polyline_length(path: Sequence[Point]) -> float:
    if len(path) < 2:
        return 0.0
    return sum(euclid(a, b) for a, b in zip(path[:-1], path[1:]))


def path_to_grid_overlay(grid_map: np.ndarray, path: Sequence[Point]) -> np.ndarray:
    """Return a copy of the grid with the path painted as value 3."""
    overlay = np.array(grid_map, copy=True)
    for p in path:
        if in_bounds(p, *overlay.shape):
            overlay[p] = 3
    return overlay



def precompute_base_gvd_index(grid_map: np.ndarray):
    """
    Precompute the original-map GVD and its pixel graph once.

    This is the practical choice when you need many shortest-path queries on the
    same map (for example all-pairs between many goals). It is not a strict
    implementation of Algorithm 1, but it avoids recomputing the medial axis for
    every pair.
    """
    work = _prepare_grid(grid_map)
    gvd = build_gvd_from_grid(work)
    graph = build_gvd_pixel_graph(gvd["gvd_mask"])
    return {
        "grid_map": work,
        "gvd": gvd,
        "graph": graph,
    }


def connect_point_to_prebuilt_gvd(index: dict, point: Point, max_connections: int = 8) -> List[Point]:
    work = index["grid_map"]
    gvd = index["gvd"]
    return _select_connection_pixels(
        work,
        gvd["gvd_mask"],
        gvd["distance"],
        point,
        max_connections=max_connections,
    )


def shortest_path_via_prebuilt_gvd(index: dict, start_point: Point, goal_point: Point, max_connections: int = 8):
    """
    Fast reusable mode:
    - reuse a precomputed GVD graph on the original map,
    - connect start and goal directly to that GVD,
    - run Dijkstra on the augmented sparse graph.

    This is usually the better mode for multi-goal planning because you pay the
    GVD construction cost only once.
    """
    base_graph: Graph = index["graph"]
    graph: Graph = {u: dict(vs) for u, vs in base_graph.items()}

    start_links = connect_point_to_prebuilt_gvd(index, start_point, max_connections=max_connections)
    goal_links = connect_point_to_prebuilt_gvd(index, goal_point, max_connections=max_connections)

    graph[start_point] = {}
    graph[goal_point] = {}

    for v in start_links:
        w = euclid(start_point, v)
        graph[start_point][v] = w
        graph.setdefault(v, {})[start_point] = w

    for v in goal_links:
        w = euclid(goal_point, v)
        graph[goal_point][v] = w
        graph.setdefault(v, {})[goal_point] = w

    if collision_free_segment(index["grid_map"], start_point, goal_point):
        w = euclid(start_point, goal_point)
        graph[start_point][goal_point] = w
        graph[goal_point][start_point] = w

    path_nodes, dist = dijkstra_on_graph(graph, start_point, goal_point)
    return {
        "path_nodes": path_nodes,
        "distance": dist,
        "start_links": start_links,
        "goal_links": goal_links,
        "graph": graph,
    }
