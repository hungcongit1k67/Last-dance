from cgi import print_form
import random
#from numpy.lib.shape_base import tile
import pygame
import numpy as np
import math
import matplotlib.pyplot as plt
import queue 
from pygame import color

class GridMap():

    mapSize = None
    gridMap = None
    npos = None
    deslist = None
    dista = None
    trace = None
    owner = None
    hold = None
    adj = None
    inters = None
    mksz = 10
    DFType = "Fast Marching"
    colorHold = ['blue','green','gold','tan','maroon','orange','cyan','violet','salmon','lime','darkslateblue']
    #colorWave = ['lightseagreen','aquamarine','aqua','mediumspringgreen','springgreen','blueviolet','violet','slateblue','darkslateblue','plum']
    colorWave = ['linen','peachpuff','papayawhip','oldlace','floralwhite','seashell','mintcream','azure','ghostwhite','lightcyan','aliceblue',]
    

    tx = [-1,0,1,0,-1,1,1,-1]
    ty = [0,1,0,-1,1,1,-1,-1]

    # gird[i][j] = 2 ~ checkpoint
    # grid[i][j] = 1 ~ block

    def __init__(self, mapSize, square_width = 20, square_height = 20, margin = 1):
        self.mapSize = mapSize
        self.square_width = square_width
        self.square_height = square_height
        self.margin = margin
        self.window_size = [self.mapSize*square_width+(self.mapSize+1)*self.margin,
                            self.mapSize*square_height+(self.mapSize+1)*self.margin]
        self.gridMap = []
        self.checked = np.zeros((mapSize,mapSize))



    def create_grid_map(self,npos):
        self.npos = npos
        self.deslist = np.zeros((npos,2))
        WIDTH = self.square_width
        HEIGHT = self.square_height
        MARGIN = self.margin
        grid = []
        for row in range(self.mapSize):
            grid.append([])
            for column in range(self.mapSize):
                grid[row].append(0)

        pygame.init()
        window_size = self.window_size
        scr = pygame.display.set_mode(window_size)
        pygame.display.set_caption("Grid Map")
        done = False
        clock = pygame.time.Clock()

        i = 0

        while not done:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    column = pos[0] // (WIDTH + MARGIN)
                    row = pos[1] // (HEIGHT + MARGIN)
                    if (i < npos):
                        grid[row][column] = 2
                        self.deslist[i] = np.array([row,column])
                        i = i+1
                        print("Click ", pos, "Grid coordinates: ", row, column)
                    else:
                        grid[row][column] = 1
                        print("Click ", pos, "Grid coordinates: ", row, column)
            scr.fill((0,0,0))
            for row in range(self.mapSize):
                for column in range(self.mapSize):
                    color = (255,255,255)
                    if grid[row][column] == 1:
                        color = (0,0,0)
                    pygame.draw.rect(scr,
                                     color,
                                     [(MARGIN + WIDTH) * column + MARGIN,
                                      (MARGIN + HEIGHT) * row + MARGIN,
                                      WIDTH,
                                      HEIGHT])

                    color = (255,255,255)
                    if (grid[row][column] == 2):
                        color = (0,255,0)
                        pygame.draw.rect(scr,
                                         color,
                                         [(MARGIN + WIDTH) * column + MARGIN,
                                          (MARGIN + HEIGHT) * row + MARGIN,
                                          WIDTH,
                                          HEIGHT])

            clock.tick(50)
            pygame.display.flip()

        pygame.quit()
        self.gridMap = grid
        return grid

        
    def showGridMap(self):

        WIDTH = self.square_width
        HEIGHT = self.square_height
        MARGIN = self.margin
        grid = self.gridMap

        pygame.init()
        window_size = self.window_size
        scr = pygame.display.set_mode(window_size)
        pygame.display.set_caption("Grid")
        done = False
        clock = pygame.time.Clock()
        

        while not done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
            scr.fill((0,0,0))
            for row in range(self.mapSize):
                for column in range(self.mapSize):
                    color = (255,255,255)
                    if grid[row][column] == 1:
                        color = (0,0,0)
                    pygame.draw.rect(scr,color,[(MARGIN + WIDTH) * column + MARGIN,(MARGIN + HEIGHT) * row + MARGIN,WIDTH,HEIGHT])

                    color = (255,255,255)
                    if (grid[row][column] == 2):
                        color = (0,255,0)
                        pygame.draw.rect(scr,color,[(MARGIN + WIDTH) * column + MARGIN,(MARGIN + HEIGHT) * row + MARGIN,WIDTH,HEIGHT])

            clock.tick(50)
            pygame.display.flip()
        return 



    def goStraight(self,p1,p2):
        if self.gridMap[p1[0]][p2[1]] == 1: return 0
        if self.gridMap[p2[0]][p1[1]] == 1: return 0
        return 1
        


    def validpos(self,u,v):
        # (u,v) is in the map and not obstacle
        if (u<0) or (u>self.mapSize-1) or (v<0) or (v>self.mapSize-1) or (self.gridMap[int(u)][int(v)]==1):
            return 0
        return 1



    def distant(self,x1,y1,x2,y2):
        dx = x1-x2
        dy = y1-y2
        re = math.sqrt(dx*dx + dy*dy)
        return re


    def buildSumBlock(self):
        self.sumBlock = [ [0 for i in range(self.mapSize+1)] for i in range(self.mapSize+1)]
        for i in range(self.mapSize):
            for j in range(self.mapSize):
                self.sumBlock[i+1][j+1] = self.sumBlock[i+1][j] + self.sumBlock[i][j+1] - self.sumBlock[i][j]
                if (self.gridMap[i][j]==1):
                    self.sumBlock[i+1][j+1] += 1
                    


    def countBlock(self,x1,y1,x2,y2):
        x1 += 1
        y1 += 1
        x2 += 1
        y2 += 1
        if (x1>x2):
            x1,x2 = x2,x1
        if (y1>y2):
            y1,y2 = y2,y1

        re = self.sumBlock[x2][y2] - self.sumBlock[x1-1][y2] - self.sumBlock[x2][y1-1] + self.sumBlock[x1-1][y1-1]
        return re



    def connectable(self,x1,y1,x2,y2):
        vec = 0
        if (x1>x2):
            x1,x2 = x2,x1
            vec = vec^1
        if (y1>y2):
            y1,y2 = y2,y1
            vec = vec^1

        tot = self.countBlock(x1,y1,x2,y2)
        dx = (x2-x1+1)//2 - 1
        dy = (y2-y1+1)//2 - 1
        t1 = 0
        t2 = 0
        if dx>0 and dy>0:
            if (vec==1):
                t1 = self.countBlock(x1,y1,x1+dx-1,y1+dy-1)
                t2 = self.countBlock(x2-dx+1,y2-dy+1,x2,y2)
            else:
                t1 = self.countBlock(x2-dx+1,y1,x2,y1+dy-1)
                t2 = self.countBlock(x1,y2-dy+1,x1+dx-1,y2)
        cnt = tot - t1 - t2
        return cnt


    def add_edge(self, posA, posB):

        u1 = self.owner[posA[0]][posA[1]][posA[2]]
        u2 = self.owner[posB[0]][posB[1]][posB[2]]
        if u1 == u2 :return
        if (u1==-1 or u2==-1): return

        totdis = self.dista[posA[1]][posA[2]] + self.dista[posB[1]][posB[2]]

        if self.adj[u1][u2] > totdis:
            self.adj[u1][u2] = totdis
            self.adj[u2][u1] = totdis
            self.inters[u1][u2] = (posA,posB)
            self.inters[u2][u1] = (posB,posA)



    def spread(self,pos,dir):

        re = []
        for h in range(0,dir): # 4 direction
            u2 = pos[0] + self.tx[h]
            v2 = pos[1] + self.ty[h]
            if (self.validpos(u2,v2)==0): continue
            if self.goStraight(pos,(u2,v2)) == 0: continue

            re.append( ((u2,v2),h) )

        return re
            


    def buildForest(self):

        self.dista = [ [self.mapSize*self.mapSize for i in range(self.mapSize)] for i in range(self.mapSize)]
        self.trace = [ [ [ (-1,-1,-1) for i in range(self.mapSize)] for i in range(self.mapSize)] for i in range(2)]
        self.owner = [ [ [-1 for i in range(self.mapSize)] for i in range(self.mapSize)] for i in range(2)]
        self.adj = [ [self.mapSize*self.mapSize for i in range(self.npos)] for i in range(self.npos)]
        self.inters = [ [ ((-1,-1,-1),(-1,-1,-1)) for i in range(self.npos) ] for i in range(self.npos)]
        self.froze = [ [ 0 for i in range(self.mapSize) ] for i in range(self.mapSize)]

        pq = queue.PriorityQueue()
        for i in range(self.npos): 
            pos = self.deslist[i]
            pq.put((0,pos))
            self.dista[pos[0]][pos[1]] = 0
            self.trace[0][pos[0]][pos[1]] = (-1,-1,-1)
            self.owner[0][pos[0]][pos[1]] = i

        while not pq.empty():
            item = pq.get()
            nxt = self.spread(item[1],4) # 4 direction
            if self.froze[item[1][0]][item[1][1]] == 1: continue
            self.froze[item[1][0]][item[1][1]] = 1

            for (u2,v2),h in nxt:

                if self.froze[u2][v2] == 0:
                    dx = self.mapSize*self.mapSize
                    if (self.validpos(u2-1,v2)!=0):
                        if (self.gridMap[u2-1][v2]!=1):
                            dx = min(dx,self.dista[u2-1][v2])
                    if (self.validpos(u2+1,v2)!=0):
                        if (self.gridMap[u2+1][v2]!=1):
                            dx = min(dx,self.dista[u2+1][v2])

                    dy = self.mapSize*self.mapSize
                    if (self.validpos(u2,v2-1)!=0):
                        if (self.gridMap[u2][v2-1]!=1):
                            dy = min(dy,self.dista[u2][v2-1])
                    if (self.validpos(u2,v2+1)!=0):
                        if (self.gridMap[u2][v2+1]!=1):
                            dy = min(dy,self.dista[u2][v2+1])

                    dis = 0
                    T = self.dista[u2][v2]
                    if (T>dx and T>dy):
                        dis = (math.sqrt(-(dx*dx)+2*dx*dy-(dy*dy)+2) +dx +dy)/2
                    elif (T>dx and T<=dy):
                        dis = dx + 1
                    elif (T<=dx and T>dy):
                        dis = dy + 1

                    if (self.dista[u2][v2] > dis):
                        self.dista[u2][v2] = dis
                        pq.put( ( dis , (u2,v2)) )
                        nxt2 = self.spread((u2,v2),4) # 8-direction
                        nho1 = self.mapSize*self.mapSize
                        nho2 = (-1,-1)
                        for (u3,v3),h in nxt2:
                            if (self.owner[0][u3][v3]!=-1 and self.dista[u3][v3]<nho1):
                                nho1 = self.dista[u3][v3]
                                nho2 = (u3,v3)
                        self.trace[0][u2][v2] = (0,nho2[0],nho2[1])
                        self.owner[0][u2][v2] = self.owner[0][nho2[0]][nho2[1]]
                        continue

        #add_edge         
        for u in range(self.mapSize):
            for v in range(self.mapSize):
                if self.owner[0][u][v] == 0: continue
                nxt = self.spread((u,v),4)
                for (u2,v2),h in nxt:
                    self.add_edge((0,u,v),(0,u2,v2))


    def buildForestAdvanced(self): # require buildForest before

        self.allow = [ [0 for i in range(self.mapSize)] for i in range(self.mapSize)]
        self.mirrow = [ [0 for i in range(self.mapSize)] for i in range(self.mapSize)]
        self.hold = [ [] for i in range(self.npos) ]

        for i in range(self.mapSize):
            for j in range(self.mapSize):
                own = self.owner[0][i][j]
                if (own!=-1):
                    self.hold[own].append((i,j))

        for comp in range(self.npos):
            pq = queue.PriorityQueue()
            for pos in self.hold[comp]:
                self.allow[pos[0]][pos[1]] = 1
                self.mirrow[pos[0]][pos[1]] = self.dista[pos[0]][pos[1]]
                self.dista[pos[0]][pos[1]] = self.mapSize*self.mapSize
                self.froze[pos[0]][pos[1]] = 0

                nxt = self.spread(pos,8)
                for (u2,v2),h in nxt:
                    if self.owner[0][u2][v2] != comp:
                        pq.put( (self.dista[u2][v2],(0,u2,v2)) )
                        self.froze[u2][v2] = 0
            
            while not pq.empty():
                item = pq.get()
                nxt = self.spread((item[1][1],item[1][2]),4)
                if self.froze[item[1][1]][item[1][2]] == 1: continue
                self.froze[item[1][1]][item[1][2]] = 1

                for (u2,v2),h in nxt:
                    if self.allow[u2][v2] == 0: continue

                    if self.froze[u2][v2] == 0:
                        dx = self.mapSize*self.mapSize
                        if (self.validpos(u2-1,v2)!=0):
                            if (self.gridMap[u2-1][v2]!=1):
                                dx = min(dx,self.dista[u2-1][v2])
                        if (self.validpos(u2+1,v2)!=0):
                            if (self.gridMap[u2+1][v2]!=1):
                                dx = min(dx,self.dista[u2+1][v2])

                        dy = self.mapSize*self.mapSize
                        if (self.validpos(u2,v2-1)!=0):
                            if (self.gridMap[u2][v2-1]!=1):
                                dy = min(dy,self.dista[u2][v2-1])
                        if (self.validpos(u2,v2+1)!=0):
                            if (self.gridMap[u2][v2+1]!=1):
                                dy = min(dy,self.dista[u2][v2+1])  

                        dis = 0
                        T = self.dista[u2][v2]
                        #print(dx," ",dy)
                        if (T>dx and T>dy):
                            val = -(dx*dx)+2*dx*dy-(dy*dy)+2
                            if (val>=0):
                                dis = (math.sqrt(val) +dx +dy)/2
                            else:
                                dis = min(dx,dy)+1
                        elif (T>dx and T<=dy):
                            dis = dx + 1
                        elif (T<=dx and T>dy):
                            dis = dy + 1

                        if (self.dista[u2][v2] > dis):
                            self.dista[u2][v2] = dis
                            pq.put( ( dis , (1,u2,v2)) )
                            nxt2 = self.spread((u2,v2),8) # 8-direction
                            nho1 = self.mapSize*self.mapSize
                            nho2 = (-1,-1,-1)
                            for (u3,v3),h in nxt2:
                                if (self.owner[0][u3][v3]!=-1 and self.allow[u3][v3]==0 and self.dista[u3][v3]<nho1):
                                    nho1 = self.dista[u3][v3]
                                    nho2 = (0,u3,v3)
                                if (self.owner[1][u3][v3]!=-1 and self.allow[u3][v3]==1 and self.dista[u3][v3]<nho1):         
                                    nho1 = self.dista[u3][v3]
                                    nho2 = (1,u3,v3)
                            self.trace[1][u2][v2] = nho2
                            self.owner[1][u2][v2] = self.owner[nho2[0]][nho2[1]][nho2[2]]
                            continue

                    #if (self.owner[1][u2][v2]!=-1):
                    #    self.add_edge((item[1]),(1,u2,v2),dis)
                    #if (self.owner[0][u2][v2]!=-1):
                    #    self.add_edge((item[1]),(0,u2,v2),dis)

                #add_edge         
            for pos in self.hold[comp]:
                nxt = self.spread(pos,4)
                for (u2,v2),h in nxt:
                    if (self.owner[0][u2][v2]!=-1 and self.allow[u2][v2]==0):
                        self.add_edge((1,pos[0],pos[1]),(0,u2,v2))
                    if (self.owner[1][u2][v2]!=-1 and self.allow[u2][v2]==1):
                        self.add_edge((1,pos[0],pos[1]),(1,u2,v2))

            for pos in self.hold[comp]:
                self.allow[pos[0]][pos[1]] = 0
                self.dista[pos[0]][pos[1]] = self.mirrow[pos[0]][pos[1]]
        
            


    def getAdj(self,u1,u2): # path from u1 to u2
        re = []
        p1,p2 = self.inters[u1][u2]
        p1 = list(p1)
        p2 = list(p2)
        #print(p1," ",p2)
        #print(self.owner[p1[0]][p1[1]]," ",self.owner[p2[0]][p2[1]])
        #print(self.trace[p1[0]][p1[1]]," ",self.trace[p2[0]][p2[1]])
        tra = self.trace[p1[0]][p1[1]][p1[2]]
        re.append(p1[1:3].copy())
        while (tra[0] != -1):
            re.append(p1[1:3].copy())
            p1 = list(tra)
            tra = self.trace[p1[0]][p1[1]][p1[2]]
        re.append(p1[1:3].copy())
        re.reverse()

        tra = self.trace[p2[0]][p2[1]][p2[2]]
        re.append(p2[1:3].copy())
        while (tra[0] != -1):
            re.append(p2[1:3].copy())
            p2 = list(tra)
            tra = self.trace[p2[0]][p2[1]][p2[2]]  
        re.append(p2[1:3].copy())
        return re



    def getTrace(self,pos):

        re = []
        dis = 0
        cur = list(pos[0])
        for i in range(len(pos)):
            if self.connectable(cur[0],cur[1],pos[i][0],pos[i][1]) !=0 :
                dis += self.distant(cur[0],cur[1],pos[i-1][0],pos[i-1][1])
                re.append(cur.copy())
                cur = list(pos[i-1])
        
        dis += self.distant(cur[0],cur[1],pos[-1][0],pos[-1][1])
        re.append(cur.copy())
        re.append(list(pos[-1]).copy())

        return re,dis



    def twoPointTracing(self):
        self.pathTrace = [ [ [0] for i in range(self.npos)] for j in range(self.npos)]
        self.buildSumBlock()

        n = self.npos
        for u in range(n):
            for v in range(n):
                if (self.inters[u][v][0][0]==-1): continue

                t1 = self.getAdj(u,v)
                t2,dis = self.getTrace(t1)

                #print("Get trace fron ",u," to ",v)
                #print("Path :",t1)
                #print("Short:",t2)

                self.pathTrace[u][v] = t2.copy()
                self.adj[u][v] = dis

        for u in range(n-1):
            for v in range(u,n):
                if (self.inters[u][v][0][0]==-1): continue

                if (self.adj[u][v]>self.adj[v][u]):
                    self.adj[u][v] = self.adj[v][u]
                    self.pathTrace[u][v] = self.pathTrace[v][u].copy()
                    self.pathTrace[u][v].reverse()
                    continue
                if (self.adj[u][v]<self.adj[v][u]):
                    self.adj[v][u] = self.adj[u][v]
                    self.pathTrace[v][u] = self.pathTrace[u][v].copy()
                    self.pathTrace[v][u].reverse()
                    continue


    def dijkstra(self):

        self.next = [ [-1 for i in range(self.npos)] for i in range(self.npos)]
        self.dijk = [ [self.mapSize*self.mapSize for i in range(self.npos)] for i in range(self.npos)]
        self.dtra = [ [-1 for i in range(self.npos)] for i in range(self.npos)]
        for root in range(self.npos):
            self.dijk[root][root] = 0
            pq = queue.PriorityQueue()
            pq.put((0,root))

            while not pq.empty():
                item = pq.get()
                u = item[1]
                for v in range(self.npos):
                    if (self.dijk[root][v] > item[0] + self.adj[u][v]):
                        self.dijk[root][v] = item[0] + self.adj[u][v]
                        self.dtra[root][v] = u
                        pq.put((self.dijk[root][v],v))


    def buildGraphNormal(self):
        self.buildForest()
        self.twoPointTracing()
        self.dijkstra()
        self.DFType = "Fast marching"
        

    def buildGraphAdvanced(self):
        self.buildForest()
        self.buildForestAdvanced()
        self.twoPointTracing()
        self.dijkstra()
        self.DFType = "Fast marching firework"



    def getPath(self,sol):
        re = []
        sol = np.append(sol,sol[0])
        pre = int(sol[0])
        for i in range(1,self.npos+1):
            cur = int(sol[i])
            #print("From ",pre," to ",cur)
            #print("From ",self.deslist[pre]," to ",self.deslist[cur])
            p1 = int(pre)
            p2 = int(cur)
            t1 = []
            while(p1!=p2):
                k = self.dtra[p1][p2]
                t2 = list(self.pathTrace[p2][k]).copy() # t2 = p2 -> k
                #print("pass ",k," ... ",p2)
                t1.extend(t2.copy())
                p2 = k

            #t1 = p2 -> p1

            t1.reverse()
            re.extend(t1)
            pre = cur

            #print(t1,'\n')

        return re


    def get_grid_from_file(self,file_path):

        f = open(file_path, "r")
        info = f.read().split('\n')

        msz = int(info[0])
        npos = 0
        points = []

        gr = info[1:]
        for i in range(msz):
            gr[i] = gr[i].split(' ')
            for j in range(msz):
                gr[i][j] = int(gr[i][j])
                if (gr[i][j]==2):
                    points.append((i,j))
                    npos += 1
        
        self.mapSize = msz
        self.npos = npos
        self.deslist = points
        self.gridMap = gr
        self.mksz = int(20*20/msz +1)
           


    def drawPath(self,points):
        sz = self.mapSize
        plt.figure(figsize=(8, 8), dpi=80)
        ##plt.axis([ -sz, sz, -sz, sz]) 
        plt.axis([ -1, sz, -sz, 1]) 
        plt.title(self.DFType,fontsize=18)
        mksz = mksz = self.mksz

        points = list(points).copy()
        for i in range(len(points)):
            points[i][0] *= -1
        ys, xs = zip(*points) #create lists of x and y values
        plt.plot(xs,ys,color='blue',linewidth = 4) 
        
        blx = []
        bly = []
        for i in range(self.mapSize):
            for j in range(self.mapSize):
                if (self.gridMap[i][j]==1):
                    blx.append(j)
                    bly.append(-i)
        plt.plot(blx,bly,'ks',markersize = mksz)

        dx = []
        dy = []
        for i in range(self.mapSize):
            for j in range(self.mapSize):
                if (self.gridMap[i][j]==2):
                    dx.append(j)
                    dy.append(-i)
        plt.plot(dx,dy,'s',color='red',markersize = mksz + 3)

        conner = [[-0.5,0.5], [sz-0.5,0.5], [sz-0.5,-(sz-0.5)], [-0.5,-(sz-0.5)]]
        conner.append(conner[0])
        cnx, cny = zip(*conner) #create lists of x and y values
        plt.plot(cnx,cny,color="black")
        plt.xticks([])
        plt.yticks([])
        plt.show()




    def drawFMComponent(self,rmv=[]):
        sz = self.mapSize
        plt.figure(figsize=(8, 8), dpi=80)
        ##plt.axis([ -sz, sz, -sz, sz]) 
        plt.axis([ -1, sz, -sz, 1]) 
        plt.title(self.DFType,fontsize=18)
        mksz = self.mksz

        for cmp in rmv:
            for pos in self.hold[cmp]:
                self.owner[0][pos[0]][pos[1]] = self.owner[1][pos[0]][pos[1]]

        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1: continue
                if self.owner[0][i][j] == -1: continue
                col = self.owner[0][i][j]
                col = col % len(self.colorHold)
                #print(xs[0],' ',ys[0],' to ',xs[1],' ',ys[1])
                plt.plot(j,-i,'s',color=self.colorHold[col],markersize = mksz)

        blx = []
        bly = []
        for i in range(self.mapSize):
            for j in range(self.mapSize):
                if (self.gridMap[i][j]==1):
                    blx.append(j)
                    bly.append(-i)
        plt.plot(blx,bly,'ks',markersize = mksz)

        for i in range(self.npos-1):
            for j in range(i,self.npos):
                if (self.inters[i][j][0][0]==-1): continue
                points = list(self.pathTrace[i][j]).copy()
                #print("From ",i," to ",j)
                for k in range(len(points)):
                    if (points[k][0] > 0):
                        points[k][0] *= -1
                #print(points,"\n")
                ys, xs = zip(*points) #create lists of x and y values
                plt.plot(xs,ys,color='crimson',linewidth = 3) 
    

        dx = []
        dy = []
        for i in range(self.mapSize):
            for j in range(self.mapSize):
                if (self.gridMap[i][j]==2):
                    dx.append(j)
                    dy.append(-i)
        plt.plot(dx,dy,'s',color='red',markersize = mksz+4)

        conner = [[-0.5,0.5], [sz-0.5,0.5], [sz-0.5,-(sz-0.5)], [-0.5,-(sz-0.5)]]
        conner.append(conner[0])
        cnx, cny = zip(*conner) #create lists of x and y values
        plt.plot(cnx,cny,color="black")
        plt.xticks([])
        plt.yticks([])
        plt.show()


    def drawDijkstraWave(self,rmv=[]):
        sz = self.mapSize
        plt.figure(figsize=(8, 8), dpi=80)
        ##plt.axis([ -sz, sz, -sz, sz]) 
        plt.axis([ -1, sz, -sz, 1]) 
        plt.title(self.DFType,fontsize=18)
        mksz = self.mksz
        
        blx = []
        bly = []
        for i in range(self.mapSize):
            for j in range(self.mapSize):
                if (self.gridMap[i][j]==1):
                    blx.append(j)
                    bly.append(-i)
        plt.plot(blx,bly,'ks',markersize = mksz)

        if (0==1):
            for i in range(self.mapSize):
                for j in range(self.mapSize):
                    if (self.gridMap[i][j]==0 and self.owner[0][i][j]!=-1):
                        #print("wave")
                        val = (self.dista[i][j]*2+self.mapSize/20)/self.mapSize
                        val = int(val*255)
                        if (val>255): val = 255
                        he = hex(val)
                        he = he[2:]
                        if (len(he)==1): he = '0' + he
                        col = '#' + he + he + 'DF' 
                        plt.plot(j,-i,'s',color=col,markersize = mksz)
        else:
            #for i in range(self.mapSize):
            #    for j in range(self.mapSize):
            #        if (self.gridMap[i][j]==0 and self.owner[0][i][j]!=-1):
            #            val = int(self.dista[i][j]*2)
            #            val = val % len(self.colorWave)
            #            plt.plot(j,-i,'s',color=self.colorWave[val],markersize = mksz)
            x2 = []
            y2 = []
            z2 = []
            for i in range(self.mapSize):
                for j in range(self.mapSize):
                    if (self.gridMap[i][j]==0 and self.owner[0][i][j]!=-1):
                        if self.owner[0][i][j] in rmv: continue
                        x2.append(j)
                        y2.append(-i)
                        z2.append(-self.dista[i][j])
            #print(len(x2))
            #print(len(y2))
            #print(len(z2))
            plt.scatter(x2, y2, c=z2, cmap="jet",marker='s',s = mksz*mksz)

        dx = []
        dy = []
        for i in range(self.mapSize):
            for j in range(self.mapSize):
                if (self.gridMap[i][j]==2):
                    dx.append(j)
                    dy.append(-i)
        plt.plot(dx,dy,'s',color='red',markersize = mksz)

        conner = [[-0.5,0.5], [sz-0.5,0.5], [sz-0.5,-(sz-0.5)], [-0.5,-(sz-0.5)]]
        conner.append(conner[0])
        cnx, cny = zip(*conner) #create lists of x and y values
        plt.plot(cnx,cny,color="black")
        plt.xticks([])
        plt.yticks([])
        plt.show()





























    def inBound(self,pos):
        pos = np.round(pos,0)
        if (pos[0]<0):
            pos[0] = 0
        if (pos[0]>self.mapSize-1):
            pos[0] = self.mapSize-1

        if (pos[1]<0):
            pos[1] = 0
        if (pos[1]>self.mapSize-1):
            pos[1] = self.mapSize-1

        pos = np.array((int(pos[0]),int(pos[1])))
        return pos



    def nearestFree(self,pos):
        pos = self.inBound(pos)
        d = np.zeros((self.mapSize,self.mapSize))
        hx = [0,1,0,-1,1,1,-1,-1]
        hy = [-1,0,1,0,-1,1,-1,1]


        head = 0
        tail = 0
        Q = np.array([[pos[0],pos[1]]])
        while (head<=tail):
            
            cur = Q[head]
            head+=1

            if (self.gridMap[int(cur[0])][int(cur[1])] != 1):
                return self.inBound(cur)

            rag = np.array((0,1,2,3,4,5,6,7))

            for i in rag:
                x = cur[0] + hx[int(i)]
                y = cur[1] + hy[int(i)]
                if (x<0) or (x>self.mapSize-1) or (y<0) or (y>self.mapSize-1) or (d[int(x)][int(y)] != 0):
                    continue
                    
                d[int(x)][int(y)] = 0
                Q = np.append(Q,np.array([[x,y]]),0)
                tail+=1
        

    def getDis(self, start, end, returnPath = 0):

        start = self.inBound(start)
        end = self.inBound(end)

        if self.checked[start[0]][start[1]] == 0:
            
            self.checked[start[0]][start[1]] = 1
            d = np.zeros((self.mapSize,self.mapSize))
            trace = np.zeros((self.mapSize,self.mapSize,2))

            hx = [0,1,0,-1,1,1,-1,-1]
            hy = [-1,0,1,0,-1,1,-1,1]


            for i in range(0,self.mapSize):
                for j in range(0,self.mapSize):

                    if self.gridMap[i][j] == 1:
                        d[i][j] = -1

            d[int(start[0])][int(start[1])] = 1
            head = 0
            tail = 0
            Q = np.array([[start[0],start[1]]])
            while (head<=tail):
                
                cur = Q[head]
                head+=1

                #di song song
                for i in range(4):
                    cur2 = np.zeros(2)
                    cur2[0] = cur[0] + hx[i]
                    cur2[1] = cur[1] + hy[i]

                    if self.validPos(cur2):
                        if d[int(cur2[0])][int(cur2[1])]==0:
                            d[int(cur2[0])][int(cur2[1])] = d[int(cur[0])][int(cur[1])] + 1
                            trace[int(cur2[0])][int(cur2[1])] = cur
                            Q = np.append(Q,np.array([[cur2[0],cur2[1]]]),0)
                            tail+=1
                #di cheo
                for i in range(4,8):
                    cur2 = np.zeros(2)
                    cur2[0] = cur[0] + hx[i]
                    cur2[1] = cur[1] + hy[i]
                    cur3 = np.zeros(2)
                    cur3[0] = cur[0]
                    cur3[1] = cur2[1]
                    cur4 = np.zeros(2)
                    cur4[0] = cur2[0]
                    cur4[1] = cur[1]

                    if (self.validPos(cur2) and self.validPos(cur3) and self.validPos(cur4)):
                        if d[int(cur2[0])][int(cur2[1])]==0:
                            d[int(cur2[0])][int(cur2[1])] = d[int(cur[0])][int(cur[1])] + math.sqrt(2)
                            trace[int(cur2[0])][int(cur2[1])] = cur
                            Q = np.append(Q,np.array([[cur2[0],cur2[1]]]),0)
                            tail+=1

            self.Gbfs[start[0]][start[1]] = np.copy(d)
            self.Gtrace[start[0]][start[1]] = np.copy(trace)

        d = self.Gbfs[start[0]][start[1]]
        trace = self.Gtrace[start[0]][start[1]]

        if returnPath == 0:
            return d[int(end[0])][int(end[1])] - 1

        cur = np.copy(end)
        re = np.array([[cur[0],cur[1]]])
        while ((cur==start).all() == False):
            cur = np.copy(trace[int(cur[0])][int(cur[1])])
            re = np.append(re,np.array([[cur[0],cur[1]]]),0)

        return re


    def getFullPath(self,posList,posSize):
        posList = np.append( posList,np.array([[posList[0][0],posList[0][1]]]),0)
        posSize += 1
        trace = np.array([[posList[0][0],posList[0][1]]])

        for i in range(1,posSize):

            path = self.getDis(posList[i],posList[i-1],1)
            for pos in path:
                trace = np.append(trace,np.array([[int(pos[0]),int(pos[1])]]),0)

        return trace


    def getRandomPath(self):
        pos = self.npos
        re = np.zeros((pos,2))
        re[0] = self.deslist[0]
        re[pos-1] = self.deslist[pos-1]
        for i in range (1,pos-1):
            newPos = np.zeros(2)
            newPos[0] = np.random.randint(0, self.mapSize)
            newPos[1] = np.random.randint(0, self.mapSize)

            newPos = self.nearestFree(newPos)
            re[i] = newPos

        re = np.copy(self.deslist)
        np.random.shuffle(re)
        return re
        



