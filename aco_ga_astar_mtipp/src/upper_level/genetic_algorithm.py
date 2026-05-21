from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

Route = List[int]


def route_cost(route: Sequence[int], cost_matrix: np.ndarray) -> float:
    if len(route) < 2:
        return 0.0
    total = 0.0
    for i in range(len(route) - 1):
        c = float(cost_matrix[route[i], route[i + 1]])
        if not np.isfinite(c):
            return float("inf")
        total += c
    return total


@dataclass
class GAResult:
    route: Route
    cost: float
    history: List[float]


class GeneticAlgorithmTSP:
    """GA for a closed TSP route with a fixed start node.

    A chromosome stores only the middle nodes. The final route is
    [start] + chromosome + [start].
    """

    def __init__(
        self,
        cost_matrix: np.ndarray,
        start_index: int = 0,
        population_size: int = 80,
        max_generation: int = 150,
        crossover_rate: float = 0.9,
        mutation_rate: float = 0.08,
        elite_size: int = 2,
        gap: float = 0.9,
    ) -> None:
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

    def _fitness(self, chromosome: Sequence[int]) -> float:
        cost = route_cost(self._make_route(chromosome), self.cost_matrix)
        if not np.isfinite(cost):
            return 0.0
        return 1.0 / (cost + 1e-12)

    def _population_costs(self, population: List[Route]) -> List[float]:
        return [route_cost(self._make_route(ch), self.cost_matrix) for ch in population]

    def _tournament_select(self, population: List[Route], k: int = 3) -> Route:
        candidates = random.sample(population, k=min(k, len(population)))
        candidates.sort(key=lambda ch: route_cost(self._make_route(ch), self.cost_matrix))
        return candidates[0][:]

    @staticmethod
    def _ordered_crossover(p1: Route, p2: Route) -> Tuple[Route, Route]:
        size = len(p1)
        if size < 2:
            return p1[:], p2[:]
        a, b = sorted(random.sample(range(size), 2))

        def ox(parent_a: Route, parent_b: Route) -> Route:
            child = [None] * size  # type: ignore[list-item]
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

    def run(self) -> GAResult:
        if self.n <= 1:
            return GAResult([self.start_index, self.start_index], 0.0, [0.0])

        population = [self._random_chromosome() for _ in range(self.population_size)]
        history: List[float] = []

        for _ in range(self.max_generation):
            population.sort(key=lambda ch: route_cost(self._make_route(ch), self.cost_matrix))
            best_cost = route_cost(self._make_route(population[0]), self.cost_matrix)
            history.append(float(best_cost))

            next_population = [ch[:] for ch in population[: self.elite_size]]
            offspring_target = max(self.population_size - self.elite_size, int(self.population_size * self.gap))

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

        population.sort(key=lambda ch: route_cost(self._make_route(ch), self.cost_matrix))
        best_route = self._make_route(population[0])
        best_cost = route_cost(best_route, self.cost_matrix)
        history.append(float(best_cost))
        return GAResult(route=best_route, cost=float(best_cost), history=history)
