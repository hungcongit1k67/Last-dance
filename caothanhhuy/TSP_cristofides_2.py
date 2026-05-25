from heapq import heapify, heappop, heappush
import numpy as np
import random
import time
from FFT_TSP import *
from Map_Grid import *


def kruskal(edges):
    # Hàm tạo cây MST từ tập các cạnh và trọng số
    # edges: [(weight, (vertex1, vertex2)), ...]
    # Trả về danh sách các cạnh trong cây MST

    parent = {}  # Lưu cha của mỗi đỉnh

    def find(v):
        # Hàm tìm cha của đỉnh v
        if parent[v] != v:
            parent[v] = find(parent[v])
        return parent[v]

    def union(v1, v2):
        # Hàm kết hợp hai tập hợp chứa v1 và v2 thành một tập hợp duy nhất
        root1 = find(v1)
        root2 = find(v2)

        if root1 != root2:
            parent[root2] = root1

    # Sắp xếp các cạnh theo trọng số tăng dần
    # edges.sort(key=lambda x: x[0])

    mst = []  # Danh sách các cạnh trong cây MST

    for edge in edges:
        weight, (v1, v2) = edge
        if v1 not in parent:
            parent[v1] = v1
        if v2 not in parent:
            parent[v2] = v2

        if find(v1) != find(v2):
            union(v1, v2)
            mst.append(edge)

    # print("Cây khung: ", mst)
    return mst


class CRIST:

    def __init__(self, dist: list[list[int]]):
        self.dist = dist
        self.n = len(dist)

    def ans(self, edges):
        # mst = self.prim()  # minimum spanning tree
        # print(mst)
        mst = kruskal(edges)
        d = self.degree(mst)  # degree of each node
        # d_new = self.merge_edges(mst, edges)
        # d = self.degree(d_new)
        sub = self.sub_graph(d)  # graph of odd-degree-nodes
        matching = self.match(sub, d)  # minimum matching
        euler_graph = self.unite(mst, matching)  # sub + matching has euler circuit
        euler_circuit = self.circuit(euler_graph)  # make circuit
        hamilton = self.skip(euler_circuit)  # make trail

        # print("Đồ thị Euler: ", euler_circuit)
        # print('route: ' + str(hamilton[1]))
        # print('weight: ' + str(hamilton[0]))
        return hamilton[1]

    def ans2(self):
        mst = self.prim()
        tmp = mst
        for i in range(len(tmp)):
            mst.append((tmp[i][0], (tmp[i][1][1], tmp[i][1][0])))

        circuit = self.circuit(mst)
        hamilton = self.skip(circuit)

        print('route: ' + str(hamilton[1]))
        print('weight: ' + str(hamilton[0]))

    def prim(self):
        tree = []  # answer
        heap = []  # heap
        marked = [False for _ in range(self.n)]  # marked?
        new_node = 0  # new node marked lately
        marked[new_node] = True  # marked first node
        count = 1  # the number of marked node
        weight = 0  # total weight
        while count < self.n:
            for i in range(self.n):
                if marked[i] == True:  # skip
                    continue
                heappush(heap, (self.dist[new_node][i], (new_node, i)))  # push all neighbor of current tree

            for i in heap:
                line = heappop(heap)  # check whether addable to tree
                if marked[line[1][1]] == False:
                    break

            # new_vertex
            weight += line[0]
            marked[line[1][1]] = True
            count += 1
            new_node = line[1][1]
            tree.append(line)

        tree.sort()
        # print(tree)
        return tree

    def merge_edges(self, mst, edges):
        lst = []
        last_point = mst[-1]
        n = 0
        for i in range(len(edges)):
            if edges[i] == last_point:
                n = i
                break
        lst = edges[:n]
        # print(mst)
        # print(edges)
        return lst

    def degree(self, mst):
        deg = [0 for _ in range(self.n)]
        for i in range(len(mst)):
            deg[mst[i][1][0]] += 1
            deg[mst[i][1][1]] += 1

        # print(deg)
        return deg

    def sub_graph(self, deg):
        sub = []
        for i in range(self.n):
            for j in range(i, self.n):
                if deg[i] % 2 == 0 or deg[j] % 2 == 0 or i == j:
                    continue
                sub.append((self.dist[i][j], (i, j)))

        sub.sort()
        return sub

    def match(self, sub, deg):  # greedy
        matching = []
        for e in sub:
            if deg[e[1][0]] % 2 == 1 and deg[e[1][1]] % 2 == 1:
                matching.append(e)
                # heappush(matching, e)
                deg[e[1][0]] += 1
                deg[e[1][1]] += 1
            # if deg[e[1][0]] % 2 == 1 and deg[e[1][1]] % 2 == 1 and e[0] > 9000:
            #     path = fast_marching(draw(), grid, e[1][0], e[0][1])

        return matching

    def unite(self, mst, matching):
        euler = mst
        euler.extend(matching)
        euler.sort()

        return euler

    def circuit(self, euler_graph):
        deg = [0 for _ in range(self.n)]
        for i in euler_graph:
            deg[i[1][0]] += 1
            deg[i[1][1]] += 1

        ans = []
        stack = []
        list = euler_graph
        current_node = list[0][1][0]

        while deg[current_node] > 0 or stack != []:
            if deg[current_node] == 0:
                ans.append(current_node)
                current_node = stack.pop()
            else:
                stack.append(current_node)
                for neighbor in list:
                    if neighbor[1][0] == current_node:
                        deg[current_node] -= 1
                        deg[neighbor[1][1]] -= 1
                        current_node = neighbor[1][1]
                        list.remove(neighbor)
                        break
                    elif neighbor[1][1] == current_node:
                        deg[current_node] -= 1
                        deg[neighbor[1][0]] -= 1
                        current_node = neighbor[1][0]
                        list.remove(neighbor)
                        break
        # print("Đồ thị Euler: ", ans)
        return ans

    def skip(self, circuit):
        weight = 0
        order = []
        cir = circuit
        visited = [0 for _ in range(self.n)]
        c = cir.pop(0)
        first = c
        while len(cir) >= 0:
            visited[c] = True
            if cir == []:
                order.append(c)
                order.append(first)
                weight += self.dist[c][first]
                break
            elif visited[cir[0]] == True:
                cir.pop(0)
            else:
                order.append(c)
                cc = cir.pop(0)
                weight += self.dist[c][cc]
                # w.append(self.dist[c][cc])
                c = cc
        # print("Đồ thị Hamilton: ", order)
        return (weight, order)


def main():
    # random sample data
    num = 1000
    start_time = time.time()
    data = [[random.randint(0, 1000) for _ in range(num)] for _ in range(num)]
    end_time = time.time()
    time_running = end_time - start_time
    print(time_running)

    c = CRIST(data)
    c.ans()


if __name__ == "__main__":
    main()