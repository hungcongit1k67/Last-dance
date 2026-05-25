import random


class Graph(object):
    def __init__(self, cost_matrix: list, rank: int):
        """
        :param cost_matrix:
        :param rank: rank of the cost matrix
        """
        self.matrix = cost_matrix
        # rank = len(cost_matrix)
        self.rank = rank
        # noinspection PyUnusedLocal
        self.pheromone = [[1 / (rank * rank) for j in range(rank)] for i in range(rank)]


class ACO(object):
    def __init__(self, ant_count: int, generations: int, alpha: float, beta: float, rho: float, q: int,
                 strategy: int):
        """
        :param ant_count:
        :param generations:
        :param alpha: relative importance of pheromone
        :param beta: relative importance of heuristic information
        :param rho: pheromone residual coefficient
        :param q: pheromone intensity
        :param strategy: pheromone update strategy. 0 - ant-cycle, 1 - ant-quality, 2 - ant-density
        """
        self.Q = q
        self.rho = rho
        self.beta = beta
        self.alpha = alpha
        self.ant_count = ant_count
        self.generations = generations
        self.update_strategy = strategy

    def _update_pheromone(self, graph: Graph, ants: list):
        for i, row in enumerate(graph.pheromone):
            for j, col in enumerate(row):
                graph.pheromone[i][j] *= self.rho
                for ant in ants:
                    graph.pheromone[i][j] += ant.pheromone_delta[i][j]

    # noinspection PyProtectedMember
    def solve(self, graph: Graph):
        """
        :param graph:
        """
        best_cost = float('inf')
        best_solution = []
        for gen in range(self.generations):
            # noinspection PyUnusedLocal
            ants = [_Ant(self, graph) for i in range(self.ant_count)]
            for ant in ants:
                for i in range(graph.rank - 1):
                    ant._select_next()
                ant.total_cost += graph.matrix[ant.tabu[-1]][ant.tabu[0]]
                if ant.total_cost < best_cost:
                    best_cost = ant.total_cost
                    best_solution = [] + ant.tabu
                # update pheromone
                ant._update_pheromone_delta()
            self._update_pheromone(graph, ants)
            # print('generation #{}, best cost: {}, path: {}'.format(gen, best_cost, best_solution))
        return best_solution, best_cost


class _Ant(object):
    def __init__(self, aco: ACO, graph: Graph):
        self.colony = aco
        self.graph = graph
        self.total_cost = 0.0
        self.tabu = []  # tabu list
        self.pheromone_delta = []  # the local increase of pheromone
        self.allowed = [i for i in range(graph.rank)]  # nodes which are allowed for the next selection
        self.eta = [[0 if i == j else 1 / graph.matrix[i][j] for j in range(graph.rank)] for i in
                    range(graph.rank)]  # heuristic information
        start = random.randint(0, graph.rank - 1)  # start from any node
        self.tabu.append(start)
        self.current = start
        self.allowed.remove(start)

    def _select_next(self):
        denominator = 0
        for i in self.allowed:
            denominator += self.graph.pheromone[self.current][i] ** self.colony.alpha * self.eta[self.current][
                                                                                            i] ** self.colony.beta
        # noinspection PyUnusedLocal
        probabilities = [0 for i in range(self.graph.rank)]  # probabilities for moving to a node in the next step
        for i in range(self.graph.rank):
            try:
                self.allowed.index(i)  # test if allowed list contains i
                probabilities[i] = self.graph.pheromone[self.current][i] ** self.colony.alpha * \
                    self.eta[self.current][i] ** self.colony.beta / denominator
            except ValueError:
                pass  # do nothing
        # select next node by probability roulette
        selected = 0
        rand = random.random()
        for i, probability in enumerate(probabilities):
            rand -= probability
            if rand <= 0:
                selected = i
                break
        self.allowed.remove(selected)
        self.tabu.append(selected)
        self.total_cost += self.graph.matrix[self.current][selected]
        self.current = selected

    # noinspection PyUnusedLocal
    def _update_pheromone_delta(self):
        self.pheromone_delta = [[0 for j in range(self.graph.rank)] for i in range(self.graph.rank)]
        for _ in range(1, len(self.tabu)):
            i = self.tabu[_ - 1]
            j = self.tabu[_]
            if self.colony.update_strategy == 1:  # ant-quality system
                self.pheromone_delta[i][j] = self.colony.Q
            elif self.colony.update_strategy == 2:  # ant-density system
                # noinspection PyTypeChecker
                self.pheromone_delta[i][j] = self.colony.Q / self.graph.matrix[i][j]
            else:  # ant-cycle system
                self.pheromone_delta[i][j] = self.colony.Q / self.total_cost


# lst = [[0, 34,50756334578302, 22.854323573670655, 33.133881472831305, 32.44642041709688, 32.29077171568983, 41.32745363200192, 34.60589997533138, 33.47327581167178, 33.36414353639994],
#            [34.50756334578302, 0.0, 38.78526567957931, 27.14678426375078, 24.976164104481533, 39.88816151428006, 35.75919464567039, 35.344578366931536, 40.08687419857077, 32.961942356847985],
#            [22.854323573670655, 38.78526567957931, 0.0, 45.84955456018835, 24.58090178664591, 35.772531974959925, 41.319011454574905, 27.190815290383252, 31.940434534984426, 39.56307056885795],
#            [33.133881472831305, 27.14678426375078, 45.84955456018835, 0.0, 32.88685893405129, 32.09656022348861, 33.96111295709401, 47.0656728596857, 29.831942693779794, 31.277746354553134],
#            [32.44642041709688, 24.976164104481533, 24.58090178664591, 32.88685893405129, 0.0, 38.457613174525484, 35.93499801770572, 31.71573745206073, 31.490184659670156, 36.00604303147982],
#            [32.29077171568983, 39.88816151428006, 35.772531974959925, 32.09656022348861, 38.457613174525484, 0.0, 30.396645871185164, 35.84222349439915, 36.55583525828927, 25.88385368553901],
#            [41.32745363200192, 35.75919464567039, 41.319011454574905, 33.96111295709401, 35.93499801770572, 30.396645871185164, 0.0, 32.12173391538704, 33.46664896531891, 34.70521173290715],
#            [34.60589997533138, 35.344578366931536, 27.190815290383252, 47.0656728596857, 31.71573745206073, 35.84222349439915, 32.12173391538704, 0.0, 37.16273254233167, 25.19904764705158],
#            [33.47327581167178, 40.08687419857077, 31.940434534984426, 29.831942693779794, 31.490184659670156, 36.55583525828927, 33.46664896531891, 37.16273254233167, 0.0, 23.16907986026424],
#            [33.36414353639994, 32.961942356847985, 39.56307056885795, 31.277746354553134, 36.00604303147982, 25.88385368553901, 34.70521173290715, 25.19904764705158, 23.16907986026424, 0.0]]

# aco = ACO(100, 200, 1.0, 10.0, 0.5, 10, 2)
# graph = Graph(lst, len(lst))
# aco_value = aco.solve(graph)
# print(aco_value)