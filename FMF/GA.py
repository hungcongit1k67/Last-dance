import numpy as np
import math
import random as rd

class GA:

    grid = None


    def __init__(self, grid):
        self.grid = grid
        self.n = grid.npos

    # Order Crossover
    def evolution(self, p1, p2):
        n = self.grid.npos
        first = rd.randint(0, n - 1)
        second = rd.randint(first + 1, n)
        c1 = np.zeros(n)
        c2 = np.zeros(n)
        c1[first:second] = p1[first:second]
        c2[first:second] = p2[first:second]

        it = 0
        for i in range(n):
            if (it==first):
                it = second
            if (it>=n): break

            x = p2[i]
            if x in c1: continue

            c1[it] = x
            it+=1

        it = 0
        for i in range(n):
            if (it==first):
                it = second
            if (it>=n): break

            x = p1[i]
            if x in c2: continue

            c2[it] = x
            it+=1

        return c1, c2


    def c_cost(self, sol):
        cost = 0
        pre_city = sol[-1]
        for city in sol:
            cost += self.grid.dijk[int(pre_city)][int(city)]
            pre_city = city
        return cost


    def solve(self,tot=300,numevo=300):
        swamp = []
        for i in range(tot):
            cur = list(range(self.grid.npos))
            rd.shuffle(cur)
            swamp.append(cur)

        swamp.sort(key=self.c_cost)

        layer1 = int((tot/5) * 2)
        layer2 = int((tot/5) * 4)
        #print(layer1)
        #print(layer2)

        for evotime in range(numevo):            
            for i in range(layer1,layer2,2):
                p1 = rd.randint(0,layer1-2)
                p2 = rd.randint(p1+1,layer1-1)
                child1, child2 = self.evolution(swamp[p1], swamp[p2])
                swamp[i] = child1
                swamp[i+1] = child2
            
            for i in range(layer2,tot):
                cur = list(range(self.grid.npos))
                rd.shuffle(cur)
                swamp[i] = cur

            swamp.sort(key=self.c_cost)

        return swamp[0]

    # See PyCharm help at https://www.jetbrains.com/help/pycharm/