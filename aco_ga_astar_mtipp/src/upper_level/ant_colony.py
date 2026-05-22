from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from src.upper_level.genetic_algorithm import route_cost
from src.upper_level.heuristic_function import heuristic_value
from src.upper_level.pheromone_update import update_pheromone_elite

Route = List[int]


@dataclass
class ACOResult:
    route: Route
    cost: float
    history: List[float]


class AntColonyTSP:
    """ACO for a closed TSP route with a fixed start node."""

    def __init__(
        self,
        cost_matrix: np.ndarray,
        initial_pheromone: np.ndarray,
        start_index: int = 0,
        num_ants: int = 50,
        max_iteration: int = 180,
        alpha: float = 1.0,
        beta: float = 5.0,
        rho: float = 0.15,
        q: float = 100.0,
        q0: float = 0.85,
        min_pheromone: float = 1e-6,
        max_pheromone: float = 10.0,
    ) -> None:
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

    def _transition_scores(self, current: int, unvisited: set[int], iteration: int) -> list[tuple[int, float]]:
        scores = []
        for j in unvisited:
            if not np.isfinite(self.cost_matrix[current, j]):
                score = 0.0
            else:
                tau = max(self.pheromone[current, j], self.min_pheromone)
                eta = heuristic_value(self.cost_matrix, current, j, iteration, self.max_iteration)
                score = (tau ** self.alpha) * (eta ** self.beta)
            scores.append((j, float(score)))
        return scores

    def _choose_next(self, current: int, unvisited: set[int], iteration: int) -> int:
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

    def _construct_route(self, iteration: int) -> Route:
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

    def run(self) -> ACOResult:
        if self.n <= 1:
            return ACOResult([self.start_index, self.start_index], 0.0, [0.0])

        global_best_route: Route | None = None
        global_best_cost = float("inf")
        history: List[float] = []

        for iteration in range(1, self.max_iteration + 1):
            routes = [self._construct_route(iteration) for _ in range(self.num_ants)]
            costs = [route_cost(route, self.cost_matrix) for route in routes]

            best_idx = int(np.argmin(costs))
            worst_idx = int(np.argmax(costs))
            iter_best_route = routes[best_idx]
            iter_best_cost = float(costs[best_idx])
            iter_worst_route = routes[worst_idx]
            iter_worst_cost = float(costs[worst_idx])

            if iter_best_cost < global_best_cost:
                global_best_cost = iter_best_cost
                global_best_route = iter_best_route[:]

            self.pheromone = update_pheromone_elite(
                self.pheromone,
                best_route=iter_best_route,
                best_cost=iter_best_cost,
                worst_route=iter_worst_route,
                worst_cost=iter_worst_cost,
                rho=self.rho,
                q=self.q,
                min_pheromone=self.min_pheromone,
                max_pheromone=self.max_pheromone,
            )
            history.append(float(global_best_cost))

        if global_best_route is None:
            raise RuntimeError(
                "ACO failed: all routes have infinite cost. "
                "Check that every target has at least one finite path to every other target."
            )
        return ACOResult(route=global_best_route, cost=float(global_best_cost), history=history)
