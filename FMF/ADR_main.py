 
from aco import ACO, Graph
from numpy.core.fromnumeric import shape
import My_grid as My_grid
import GA 
import numpy as np

import time
def timeEval(grid):
    start_time = time.time()
    grid.buildGraphAdvanced()
    print("--- %s seconds ---" % (time.time() - start_time))
    


def evaluation1(grid,ntest=10): # <----- iteration
    resGA = []
    resACO = []
    bestpath = None
    bestcost = grid.mapSize*grid.mapSize*grid.npos
    for iter in range(ntest):
        print("Iteration ",iter+1,"/",ntest)
        a = time.time()
        ga = GA.GA(grid)
        #mybest = ga.solve(100,100)
        #resGA.append(ga.c_cost(mybest))

        #aco = ACO(30, 400, 1.0, 10.0, 0.5, 10, 2)  
        #graph = Graph(grid.dijk, grid.npos)
        #path, cost = aco.solve(graph)
        #print('cost: {}, path: {}'.format(cost, path))
        mypath = ga.solve(500,600)
        mycost = ga.c_cost(mypath)
        resACO.append(mycost)
        if (bestcost > mycost):
            bestcost = mycost
            bestpath = mypath
        print(f"--- Iteration {iter+1}: {time.time() - a} seconds ---")

    #resGA = np.array(resGA)
    resACO = np.array(resACO)

    #sumGA = resGA.mean()
    #stdGA = resGA.std()
    #print("Mean GA cost: ",sumGA," Std GA std",stdGA)


    sumACO = resACO.mean()
    stdACO = resACO.std()
    print("Mean ACO cost: ",sumACO," Std ACO std",stdACO)
    print("Best cost:",bestcost)

    points = grid.getPath(bestpath)
    #print(points)
    grid.drawPath(points)
    grid.drawFMComponent(rmv=[0])
    grid.drawDijkstraWave(rmv=[])


def evaluation2(grid, ntest=10):
    resACO = []
    bestpath = None
    bestcost = float("inf")

    for iter in range(ntest):
        print("Iteration ", iter + 1, "/", ntest)
        a = time.time()

        aco = ACO(30, 400, 1.0, 10.0, 0.5, 10, 2)
        graph = Graph(grid.dijk, grid.npos)

        path, cost = aco.solve(graph)

        # Nếu ACO trả path có lặp lại điểm đầu ở cuối thì bỏ nó đi
        if len(path) == grid.npos + 1 and path[0] == path[-1]:
            path = path[:-1]

        resACO.append(cost)

        if cost < bestcost:
            bestcost = cost
            bestpath = path
        
        print(f"--- Iteration {iter+1}: {time.time() - a} seconds ---")

    resACO = np.array(resACO)
    print("Mean ACO cost:", resACO.mean(), "Std ACO:", resACO.std())
    print("Best cost:", bestcost)

    points = grid.getPath(bestpath)
    grid.drawPath(points)
    grid.drawFMComponent(rmv=[0])
    grid.drawDijkstraWave(rmv=[])

def ADF(grid):

    timeEval(grid)

    #grid.buildForest()
    #grid.buildGraphNormal()
    #print(grid.deslist)
    #grid.buildGraphAdvanced()
    evaluation1(grid)
    

    #for i in range(10):
    #    ga = GA.GA(grid)
    #    mybest = ga.solve()
    #    print(ga.c_cost(mybest))

    #print("Best path found:")
    #points = list(grid.getPath(mybest)).copy()
    #print(points)
#
    #print("Drawing...")



def main():

    mapSize = 20    
    grid = My_grid.GridMap(mapSize)
    if (0==1):
        npos = 4
        #npos = int(input())
        grid.create_grid_map(npos)
    else:
        #grid.get_grid_from_file("mixed2002.txt")
        #grid.get_grid_from_file("square400.txt")
        #grid.get_grid_from_file("triangle300.txt")
        #grid.get_grid_from_file("obstacle80.txt")
        #grid.get_grid_from_file("example1.txt")
        #grid.get_grid_from_file("warehouse4.txt")
        #grid.get_grid_from_file("map20_4.txt")  
        #grid.get_grid_from_file("map20_7.txt")    
        #grid.get_grid_from_file("map35_11.txt")    
        #grid.get_grid_from_file("map35_12.txt")    
        #grid.get_grid_from_file("map35_13.txt")    
        grid.get_grid_from_file("E:\\last_dance\\Code\\allcode\\FMF\\mixed200.txt")    
        #grid.get_grid_from_file("map40_2.txt")   
        #grid.get_grid_from_file("map40_12.txt")   
        #grid.get_grid_from_file("map40_15.txt")    
        #grid.get_grid_from_file("map80_1.txt")    
        #grid.get_grid_from_file("map80_2.txt")    
        #grid.get_grid_from_file("map101_00.txt")    
        #grid.get_grid_from_file("map200_00.txt")   
        #grid.get_grid_from_file("maptest1.txt")    
        #grid.get_grid_from_file("maptest2.txt")    
        

    for pos in grid.deslist:
        print(pos)
    print("\n\n\n")
    ADF(grid)

 
if __name__ == "__main__":
    main()




