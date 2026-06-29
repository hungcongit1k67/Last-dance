"""
aco_ga.py — Bộ giải TSP bằng ACO-GA (bi-level) và ACO thuần cho pipeline WP-FMF.
=================================================================================
Port lại thuật toán ACO-GA-A* (Zhang et al., 2025) từ project aco_ga_astar_mtipp:
    Pha A: GA  -> tìm route tốt cho TSP khép kín (start cố định)
    Pha B: khởi tạo pheromone từ route của GA (cộng bonus lên các cạnh của GA)
    Pha C: ACO  -> tinh chỉnh trên pheromone đã khởi tạo

Tham số MẶC ĐỊNH lấy từ configs/scenario5.yaml của project gốc — chỉnh trực tiếp
trong dict CONFIG bên dưới.

Định dạng trả về GIỐNG hệt solve_tsp_ortools() trong ADR_main_ortools.py:
    (route, real_cost_float)
  - route: permutation 0..n-1 bắt đầu từ start_index, KHÔNG lặp lại node đầu ở cuối
           (đúng định dạng grid.getPath() mong đợi).
  - real_cost_float: chi phí tour KHÉP KÍN tính trên dist_matrix (float).
"""
from __future__ import annotations

import random
from typing import List, Sequence, Tuple

import numpy as np

# =========================================================
# CONFIG — Chỉnh sửa tham số ACO-GA tại đây (mặc định = scenario5)
# =========================================================
ACO_GA_CONFIG = {
    "random_seed": 42,          # None -> không cố định seed (mỗi lần chạy khác nhau)

    "GA": {                     # Table 7, scenario 5-7
        "population_size": 100, # np
        "max_generation": 100,  # Gmax
        "crossover_rate": 0.9,  # pc
        "mutation_rate": 0.05,  # pm
        "gap": 0.9,             # gap
        "elite_size": 3,        # (project-only)
    },

    "ACO": {                    # Table 7, scenario 5-7
        "num_ants": 50,        # Nmax
        "max_iteration": 100,   # Imax
        "alpha": 1.0,           # alpha
        "beta": 5.0,            # beta
        "rho": 0.1,             # rho (tốc độ bay hơi pheromone)
        "q": 1.0,               # Q
        "q0": 0.85,             # luật pseudo-random q0
        "initial_pheromone": 0.5,    # tau0
        "ga_pheromone_bonus": 0.5,   # C của công thức (10)
        "min_pheromone": 1.0e-06,
        "max_pheromone": 12.0,
    },
}

# CONFIG cho ACO thuần (dùng class ACO/Graph trong aco.py đã đẩy vào folder)
# LƯU Ý: aco.py dùng `rho` là HỆ SỐ GIỮ LẠI pheromone (residual = 1 - bay hơi),
#         nên đặt 0.9 = 1 - 0.1 (rho bay hơi của scenario5).
ACO_CONFIG = {
    "random_seed": 42,
    "ant_count": 50,    # = num_ants scenario5
    "generations": 100,  # = max_iteration scenario5
    "alpha": 1.0,
    "beta": 5.0,
    "rho": 0.5,          # residual = 1 - 0.1
    "q": 1.0,
    "strategy": 0,       # 0 ant-cycle, 1 ant-quality, 2 ant-density
}

Route = List[int]

# Bộ đếm gọi nội bộ: cho phép tái lập được nhưng vẫn khác nhau qua các lần chạy (ntest).
_call_count = 0


def _resolve_seed(base_seed):
    """Trả về seed cho lần gọi hiện tại (base_seed + số lần đã gọi) rồi tăng bộ đếm.
    Nhờ vậy chạy lại chương trình cho kết quả tái lập, nhưng các iteration trong cùng
    một lần chạy (ntest) vẫn khác nhau."""
    global _call_count
    if base_seed is None:
        return None
    s = int(base_seed) + _call_count
    _call_count += 1
    return s


# =========================================================
# Hàm chi phí tour
# =========================================================
def route_cost(route: Sequence[int], cost_matrix: np.ndarray) -> float:
    """Tổng chi phí dọc theo route (route đã bao gồm điểm đóng tour ở cuối)."""
    if len(route) < 2:
        return 0.0
    total = 0.0
    for i in range(len(route) - 1):
        c = float(cost_matrix[route[i], route[i + 1]])
        if not np.isfinite(c):
            return float("inf")
        total += c
    return total


def _closed_tour_cost(perm: Sequence[int], cost_matrix: np.ndarray) -> float:
    """Chi phí tour khép kín cho permutation (tự cộng cạnh quay về node đầu)."""
    n = len(perm)
    if n < 2:
        return 0.0
    total = 0.0
    for i in range(n):
        c = float(cost_matrix[perm[i], perm[(i + 1) % n]])
        if not np.isfinite(c):
            return float("inf")
        total += c
    return total


# =========================================================
# Pha A — Genetic Algorithm cho TSP (start cố định)
# =========================================================
class GeneticAlgorithmTSP:
    def __init__(self, cost_matrix, start_index=0, population_size=80, max_generation=150,
                 crossover_rate=0.9, mutation_rate=0.08, elite_size=2, gap=0.9):
        self.cost_matrix = cost_matrix
        self.n = cost_matrix.shape[0]
        self.start_index = start_index
        self.population_size = max(4, int(population_size))
        self.max_generation = int(max_generation)
        self.crossover_rate = float(crossover_rate)
        self.mutation_rate = float(mutation_rate)
        self.elite_size = max(1, int(elite_size))
        self.gap = float(gap)
        self.middle_nodes = [i for i in range(self.n) if i != self.start_index]

    def _make_route(self, chromosome: Sequence[int]) -> Route:
        return [self.start_index] + list(chromosome) + [self.start_index]

    def _random_chromosome(self) -> Route:
        chrom = self.middle_nodes[:]
        random.shuffle(chrom)
        return chrom

    def _cost(self, chromosome: Sequence[int]) -> float:
        return route_cost(self._make_route(chromosome), self.cost_matrix)

    def _tournament_select(self, population: List[Route], k: int = 3) -> Route:
        candidates = random.sample(population, k=min(k, len(population)))
        candidates.sort(key=self._cost)
        return candidates[0][:]

    @staticmethod
    def _ordered_crossover(p1: Route, p2: Route) -> Tuple[Route, Route]:
        size = len(p1)
        if size < 2:
            return p1[:], p2[:]
        a, b = sorted(random.sample(range(size), 2))

        def ox(parent_a: Route, parent_b: Route) -> Route:
            child = [None] * size
            child[a:b + 1] = parent_a[a:b + 1]
            fill = [x for x in parent_b if x not in child]
            idx = 0
            for i in list(range(0, a)) + list(range(b + 1, size)):
                child[i] = fill[idx]
                idx += 1
            return [int(x) for x in child]

        return ox(p1, p2), ox(p2, p1)

    @staticmethod
    def _swap_mutation(chromosome: Route) -> Route:
        if len(chromosome) < 2:
            return chromosome[:]
        child = chromosome[:]
        a, b = random.sample(range(len(child)), 2)
        child[a], child[b] = child[b], child[a]
        return child

    def run(self) -> Tuple[Route, float]:
        """Trả về (best_route_khep_kin, best_cost)."""
        if self.n <= 1:
            return [self.start_index, self.start_index], 0.0

        population = [self._random_chromosome() for _ in range(self.population_size)]
        for _ in range(self.max_generation):
            population.sort(key=self._cost)
            next_population = [ch[:] for ch in population[: self.elite_size]]
            offspring_target = max(self.population_size - self.elite_size,
                                   int(self.population_size * self.gap))
            while len(next_population) < self.elite_size + offspring_target:
                p1 = self._tournament_select(population)
                p2 = self._tournament_select(population)
                if random.random() < self.crossover_rate:
                    c1, c2 = self._ordered_crossover(p1, p2)
                else:
                    c1, c2 = p1[:], p2[:]
                if random.random() < self.mutation_rate:
                    c1 = self._swap_mutation(c1)
                if random.random() < self.mutation_rate:
                    c2 = self._swap_mutation(c2)
                next_population.extend([c1, c2])
            population = next_population[: self.population_size]

        population.sort(key=self._cost)
        best_route = self._make_route(population[0])
        return best_route, route_cost(best_route, self.cost_matrix)


# =========================================================
# Pha B — Khởi tạo pheromone từ route của GA (công thức 10)
# =========================================================
def initialize_pheromone_from_ga(n_targets, ga_route, base=0.5, bonus=0.5) -> np.ndarray:
    pheromone = np.full((n_targets, n_targets), float(base), dtype=float)
    np.fill_diagonal(pheromone, 0.0)
    for a, b in zip(ga_route[:-1], ga_route[1:]):
        pheromone[a, b] = base + bonus
        pheromone[b, a] = base + bonus
    return pheromone


# =========================================================
# Pha C — Ant Colony Optimization cho TSP (start cố định)
# =========================================================
def _adaptive_xi(current_iteration: int, max_iteration: int) -> float:
    if max_iteration <= 0:
        return 1.0
    if current_iteration >= max_iteration:
        return 1.0 / max_iteration
    return (max_iteration - current_iteration) / max_iteration


def _heuristic_value(cost_matrix, i, j, current_iteration, max_iteration) -> float:
    cost = float(cost_matrix[i, j])
    if not np.isfinite(cost) or cost <= 0:
        return 0.0
    return _adaptive_xi(current_iteration, max_iteration) / cost


def _update_pheromone_elite(pheromone, best_route, best_cost, worst_route, worst_cost,
                            rho, q, min_pheromone=1e-6, max_pheromone=10.0) -> np.ndarray:
    new_pheromone = (1.0 - rho) * pheromone
    if np.isfinite(best_cost) and best_cost > 0:
        delta_best = rho * q / best_cost
        for a, b in zip(best_route[:-1], best_route[1:]):
            new_pheromone[a, b] += delta_best
            new_pheromone[b, a] += delta_best
    if worst_route is not None and worst_cost is not None and np.isfinite(worst_cost) and worst_cost > 0:
        delta_worst = rho * q / worst_cost
        for a, b in zip(worst_route[:-1], worst_route[1:]):
            new_pheromone[a, b] -= 0.25 * delta_worst
            new_pheromone[b, a] -= 0.25 * delta_worst
    np.fill_diagonal(new_pheromone, 0.0)
    return np.clip(new_pheromone, min_pheromone, max_pheromone)


class AntColonyTSP:
    def __init__(self, cost_matrix, initial_pheromone, start_index=0, num_ants=50,
                 max_iteration=180, alpha=1.0, beta=5.0, rho=0.15, q=100.0, q0=0.85,
                 min_pheromone=1e-6, max_pheromone=10.0):
        self.cost_matrix = cost_matrix
        self.pheromone = initial_pheromone.astype(float).copy()
        self.n = cost_matrix.shape[0]
        self.start_index = start_index
        self.num_ants = int(num_ants)
        self.max_iteration = int(max_iteration)
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.rho = float(rho)
        self.q = float(q)
        self.q0 = float(q0)
        self.min_pheromone = float(min_pheromone)
        self.max_pheromone = float(max_pheromone)

    def _transition_scores(self, current, unvisited, iteration):
        scores = []
        for j in unvisited:
            if not np.isfinite(self.cost_matrix[current, j]):
                score = 0.0
            else:
                tau = max(self.pheromone[current, j], self.min_pheromone)
                eta = _heuristic_value(self.cost_matrix, current, j, iteration, self.max_iteration)
                score = (tau ** self.alpha) * (eta ** self.beta)
            scores.append((j, float(score)))
        return scores

    def _choose_next(self, current, unvisited, iteration):
        scores = self._transition_scores(current, unvisited, iteration)
        positive = [(j, s) for j, s in scores if s > 0 and np.isfinite(s)]
        if not positive:
            return random.choice(list(unvisited))
        if random.random() <= self.q0:
            return max(positive, key=lambda item: item[1])[0]
        total = sum(s for _, s in positive)
        r = random.random() * total
        acc = 0.0
        for j, s in positive:
            acc += s
            if acc >= r:
                return j
        return positive[-1][0]

    def _construct_route(self, iteration):
        route = [self.start_index]
        unvisited = set(range(self.n))
        unvisited.remove(self.start_index)
        current = self.start_index
        while unvisited:
            nxt = self._choose_next(current, unvisited, iteration)
            route.append(nxt)
            unvisited.remove(nxt)
            current = nxt
        route.append(self.start_index)
        return route

    def run(self) -> Tuple[Route, float]:
        """Trả về (best_route_khep_kin, best_cost)."""
        if self.n <= 1:
            return [self.start_index, self.start_index], 0.0

        global_best_route = None
        global_best_cost = float("inf")

        for iteration in range(1, self.max_iteration + 1):
            routes = [self._construct_route(iteration) for _ in range(self.num_ants)]
            costs = [route_cost(route, self.cost_matrix) for route in routes]

            best_idx = int(np.argmin(costs))
            worst_idx = int(np.argmax(costs))
            iter_best_route = routes[best_idx]
            iter_best_cost = float(costs[best_idx])

            if iter_best_cost < global_best_cost:
                global_best_cost = iter_best_cost
                global_best_route = iter_best_route[:]

            self.pheromone = _update_pheromone_elite(
                self.pheromone,
                best_route=iter_best_route,
                best_cost=iter_best_cost,
                worst_route=routes[worst_idx],
                worst_cost=float(costs[worst_idx]),
                rho=self.rho,
                q=self.q,
                min_pheromone=self.min_pheromone,
                max_pheromone=self.max_pheromone,
            )

        if global_best_route is None:
            raise RuntimeError("ACO thất bại: mọi route đều có chi phí vô hạn. "
                               "Kiểm tra ma trận chi phí giữa các checkpoint.")
        return global_best_route, float(global_best_cost)


# =========================================================
# API public — tương thích solve_tsp_ortools()
# =========================================================
def _as_matrix(dist_matrix) -> np.ndarray:
    m = np.asarray(dist_matrix, dtype=float)
    if m.ndim != 2 or m.shape[0] != m.shape[1]:
        raise ValueError("dist_matrix phải là ma trận vuông NxN")
    return m


def solve_tsp_aco_ga(dist_matrix, start_index=0, config=None):
    """Giải TSP khép kín bằng ACO-GA. Trả về (route_permutation, real_cost_float)."""
    cfg = config if config is not None else ACO_GA_CONFIG
    matrix = _as_matrix(dist_matrix)
    n = matrix.shape[0]

    seed = _resolve_seed(cfg.get("random_seed"))
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed % (2 ** 32))

    if n <= 1:
        return [start_index], 0.0

    ga_cfg = cfg.get("GA", {})
    aco_cfg = cfg.get("ACO", {})

    # Pha A: GA
    ga = GeneticAlgorithmTSP(
        matrix,
        start_index=start_index,
        population_size=int(ga_cfg.get("population_size", 80)),
        max_generation=int(ga_cfg.get("max_generation", 150)),
        crossover_rate=float(ga_cfg.get("crossover_rate", 0.9)),
        mutation_rate=float(ga_cfg.get("mutation_rate", 0.08)),
        elite_size=int(ga_cfg.get("elite_size", 2)),
        gap=float(ga_cfg.get("gap", 0.9)),
    )
    ga_route, _ = ga.run()

    # Pha B: khởi tạo pheromone từ GA
    pheromone = initialize_pheromone_from_ga(
        n_targets=n,
        ga_route=ga_route,
        base=float(aco_cfg.get("initial_pheromone", 0.5)),
        bonus=float(aco_cfg.get("ga_pheromone_bonus", 0.5)),
    )

    # Pha C: ACO
    aco = AntColonyTSP(
        matrix,
        initial_pheromone=pheromone,
        start_index=start_index,
        num_ants=int(aco_cfg.get("num_ants", 50)),
        max_iteration=int(aco_cfg.get("max_iteration", 180)),
        alpha=float(aco_cfg.get("alpha", 1.0)),
        beta=float(aco_cfg.get("beta", 5.0)),
        rho=float(aco_cfg.get("rho", 0.15)),
        q=float(aco_cfg.get("q", 100.0)),
        q0=float(aco_cfg.get("q0", 0.85)),
        min_pheromone=float(aco_cfg.get("min_pheromone", 1e-6)),
        max_pheromone=float(aco_cfg.get("max_pheromone", 10.0)),
    )
    best_route, best_cost = aco.run()

    # Bỏ node đóng tour ở cuối -> permutation đúng định dạng getPath()
    route = best_route[:-1]
    return route, float(best_cost)


def solve_tsp_aco(dist_matrix, start_index=0, config=None):
    """Giải TSP khép kín bằng ACO thuần (dùng class ACO/Graph trong aco.py).
    Trả về (route_permutation, real_cost_float)."""
    from aco import ACO, Graph  # aco.py đã được đẩy vào cùng folder

    cfg = config if config is not None else ACO_CONFIG
    matrix = _as_matrix(dist_matrix)
    n = matrix.shape[0]

    seed = _resolve_seed(cfg.get("random_seed"))
    if seed is not None:
        random.seed(seed)

    if n <= 1:
        return [start_index], 0.0

    matrix_list = matrix.tolist()
    graph = Graph(matrix_list, n)
    aco = ACO(
        ant_count=int(cfg.get("ant_count", 50)),
        generations=int(cfg.get("generations", 100)),
        alpha=float(cfg.get("alpha", 1.0)),
        beta=float(cfg.get("beta", 5.0)),
        rho=float(cfg.get("rho", 0.9)),
        q=int(cfg.get("q", 1)),
        strategy=int(cfg.get("strategy", 0)),
    )
    best_solution, _ = aco.solve(graph)  # best_solution: permutation (tabu list)

    # Xoay route để bắt đầu từ start_index (tour khép kín nên thứ tự vòng không đổi).
    route = list(best_solution)
    if start_index in route:
        k = route.index(start_index)
        route = route[k:] + route[:k]

    cost = _closed_tour_cost(route, matrix)
    return route, float(cost)
