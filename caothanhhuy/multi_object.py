import math
import time
from itertools import permutations

import pygame.draw_py
from Robot import *
from Map_Grid import *
import numpy as np
from FFT_TSP import *
from TSP_cristofides_2 import *
from fixmap import gen_random_map
# from GWO_TSP import *   # file không tồn tại trong project
# from GWO_TSP2 import *  # file không tồn tại trong project
from ACO_TSP import *
import matplotlib.pyplot as plt
from Voronoi import *

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Path Finding Algorithm")


def reconstruct_path_safe(lst, grid, g_score, min_safe):
    path = []
    n = len(lst)

    path.append(lst[0])
    for i in range(1, n-1):
        current = lst[i]
        for neighbor in get_neighbor(lst[i-1], grid, 200):
            if neighbor in get_neighbor(lst[i+1], grid, 200):
                if current.safe < min_safe and neighbor.safe > current.safe and g_score[neighbor] <= g_score[current] + 0.5:
                    current = neighbor
                    lst[i] = neighbor

        path.append(current)
    return path


def reconstruct_path_smooth(path, grid):
    for i in range(1, len(path) - 1):
        if path[i-1].get_row() == path[i+1].get_row():
            if path[i].get_row() != path[i-1].get_row() and not grid[path[i-1].get_row()][path[i].get_col()].is_barrier():
                path[i] = grid[path[i-1].get_row()][path[i].get_col()]
                #print("aaa")
        elif path[i-1].get_col() == path[i+1].get_col():
            if path[i].get_col() != path[i-1].get_col() and not grid[path[i].get_row()][path[i-1].get_col()].is_barrier():
                path[i] = grid[path[i].get_row()][path[i-1].get_col()]
                #print("bbb")
    return path


def caculate_path(path, path_value, ds, lst_start, grid):
    n = len(ds)
    path_final = []
    path_length = 0
    count = 1
    while count < n:
        lst = reconstruct_path_smooth(path[ds[count - 1]][ds[count]], grid)
        path_final = path_final + lst
        path_length += path_value[ds[count - 1]][ds[count]]
        count += 1
    return path_length


def draw_path_final(path, path_value, ds, lst_start, grid, draw):
    # print(ds)
    n = len(ds)

    path_final = []
    path_length = 0
    count = 1
    #print(ds[count-1])
    while count < n:
        lst = reconstruct_path_smooth(path[ds[count-1]][ds[count]], grid)
        # lst = path[ds[count - 1]][ds[count]]
        # lst = smooth_path(path[ds[count - 1]][ds[count]])
        path_final = path_final + lst
        path_length += path_value[ds[count-1]][ds[count]]
        # print(ds[count - 1], ds[count])
        count += 1
    result = []
    for item in path_final:
        if item not in result:
            result.append(item)
    path_final = result
    for points in path_final:
        # lst = points.update_neighbors(grid)
        # for neigh in lst:
        #     neigh.make_path()
        points.make_path()
        # draw()
        time.sleep(0)
    # robot = Robot((lst_start[0].get_x() + 4, lst_start[0].get_y() + 4), "Robot.png", 4)
    # robot.pathRb = path_final
    # while True:
    #     for points in path_final:
    #         robot.draw(WIN)
    #         robot.trail((points.get_x() + 2, points.get_y() + 2), WIN, ORANGE)
    #         time.sleep(0.01)
    #         pygame.display.update()
    #     # time.sleep(30)
    #     # robot.move()
    #     break
    lst_start[0].make_start()
    lst_lst_start = lst_start[0].update_neighbors(grid)
    for points in lst_lst_start:
        points.make_start()
    for i in range(1, len(lst_start)):
        lst_start[i].make_end()
        lst_lst_end = lst_start[i].update_neighbors(grid)
        for points in lst_lst_end:
            points.make_end()
    draw()
    # print("Tổng độ dài quãng đường FFT là: "+ str(round(path_length, 2)))
    return round(path_length, 2)


def draw_path(lst_start, path, draw):
    n = len(path[0])
    for i in range(n):
        for j in range(n):
            lst = path[i][j]
            for points in lst:
                points.make_path()
    lst_start[0].make_start()
    for i in range(1, len(lst_start)):
        lst_start[i].make_end()
    draw()


def algorithm(draw, grid, lst_start, T_step, T_max, a_dgree):
    edges = []
    rows = len(grid)
    n = len(lst_start)
    lst_check = [0 for i in range(n)]
    lst_count = [0 for i in range(n)]
    lst_open_set = [[] for i in range(n)]
    lst_came_from = [{} for i in range(n)]
    lst_g_score = [{} for i in range(n)]
    lst_f_score = [{} for i in range(n)]
    lst_open_set_hash = [{} for i in range(n)]
    path = [[[] for j in range(n)] for i in range(n)]
    path_value = [[0 for j in range(n)] for i in range(n)]
    check_path = [[0 for j in range(n)] for i in range(n)]
    max_count_path = 0
    check_grid = {lst_start[0]}
    for i in range(n):
        lst_count[i] = 0
        lst_open_set[i] = PriorityQueue()
        lst_open_set[i].put((0, lst_count[i], lst_start[i]))
        lst_came_from[i] = {}
        lst_g_score[i] = {spot: float("inf") for row in grid for spot in row}
        lst_g_score[i][lst_start[i]] = 0
        lst_f_score[i] = {spot: float("inf") for row in grid for spot in row}
        lst_f_score[i][lst_start[i]] = lst_g_score[i][lst_start[i]]
        lst_open_set_hash[i] = {lst_start[i]}
        check_grid.add(lst_start[i])
    run = True
    # min_neighbors = 5 #round(math.sqrt(n) * math.log1p(n))
    # min_max_count_path = n*min_neighbors - (min_neighbors * min_neighbors - (n%min_neighbors*(n%min_neighbors)-1))
    while run:
        # print(max_count_path)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        lst_current = [None for i in range(n)]
        for i in range(n):
            lst_current[i] = lst_open_set[i].get()[2]
            for j in range(n):

                if lst_count[0] >= T_max:
                    # print(lst_count[0])
                    if check_path_valid(fix_path_map(path, path_value) + [edges]):
                        sum_edge = 0
                        for p in lst_check:
                            sum_edge += p
                        # print(lst_check)
                        # print(sum_edge / ((n - 1) * n))
                        return fix_path_map(path, path_value) + [edges]
                    else:
                        T_max += T_step
                if i != j and (lst_current[i] in lst_open_set_hash[j]) and check_path[i][j] == 0:
                    lst_check[i] += 1
                    lst_check[j] += 1
                    path_temp1 = [lst_current[i]] + reconstruct_path(lst_came_from[i], lst_current[i])
                    path_temp2 = [lst_current[i]] + reconstruct_path(lst_came_from[j], lst_current[i])
                    path_safe1 = reconstruct_path_safe(path_temp1, grid, lst_g_score[i], min_safe=1.71)
                    path_safe2 = reconstruct_path_safe(path_temp2, grid, lst_g_score[j], min_safe=1.71)
                    path_safe1.reverse()
                    path[i][j] = path_safe1 + path_safe2
                    temp_path = path[i][j].copy()
                    temp_path.reverse()
                    path[j][i] = temp_path
                    check_path[i][j] = check_path[j][i] = 1
                    path_value[i][j] = path_value[j][i] = lst_g_score[i][lst_current[i]] + lst_g_score[j][lst_current[i]]
                    edges.append((path_value[i][j], (i, j)))
                    max_count_path += 2
                    if max_count_path >= n*(n-1):
                        sum_edge = 0
                        for p in lst_check:
                            sum_edge += p
                        # print(lst_check)
                        # print(sum_edge/((n-1)*n))
                        return fix_path_map(path, path_value) + [edges]
            for neighbor in lst_current[i].neighbors:
                if neighbor.safe >= 1.71 and lst_g_score[i][lst_current[i]] < lst_g_score[i][neighbor] and neighbor not in lst_open_set_hash[i]:
                    lst_came_from[i][neighbor] = lst_current[i]
                    if neighbor.get_row() + 1 < rows and neighbor.get_row() - 1 > 0:
                        dx = min(lst_g_score[i][grid[neighbor.get_row() + 1][neighbor.get_col()]],
                                 lst_g_score[i][grid[neighbor.get_row() - 1][neighbor.get_col()]])
                    if neighbor.get_col() + 1 < rows and neighbor.get_col() - 1 > 0:
                        dy = min(lst_g_score[i][grid[neighbor.get_row()][neighbor.get_col() + 1]],
                                 lst_g_score[i][grid[neighbor.get_row()][neighbor.get_col() - 1]])
                    if neighbor.get_row() + 1 == rows:
                        dx = lst_g_score[i][grid[neighbor.get_row() - 1][neighbor.get_col()]]
                    if neighbor.get_row() - 1 == 0:
                        dx = lst_g_score[i][grid[neighbor.get_row() + 1][neighbor.get_col()]]
                    if neighbor.get_col() + 1 == rows:
                        dy = lst_g_score[i][grid[neighbor.get_row()][neighbor.get_col() - 1]]
                    if neighbor.get_col() - 1 == 0:
                        dy = lst_g_score[i][grid[neighbor.get_row()][neighbor.get_col() + 1]]

                    deltaxy = 2 - (dx - dy) * (dx - dy)
                    if deltaxy >= 0:
                        lst_g_score[i][neighbor] = (dx + dy + math.sqrt(deltaxy)) / 2
                    else:
                        lst_g_score[i][neighbor] = min(dx + 1, dy + 1)
                # temp_g_score = lst_g_score[i][lst_current[i]] + delta(lst_current[i].get_pos(), neighbor.get_pos())

                    lst_f_score[i][neighbor] = a_dgree * lst_g_score[i][neighbor] + lst_current[i].get_safe() * (-1 + a_dgree)

                    lst_count[i] += 1
                    lst_open_set[i].put((lst_f_score[i][neighbor], lst_count[i], neighbor))
                    neighbor.make_open(i)
                    lst_open_set_hash[i].add(neighbor)
                    # check_grid.add(neighbor)
                else:
                    neighbor.make_open(i)

        # if lst_count[0] % 10 == 0:
        #     draw()
        for i in range(n):
            if lst_current[i] not in lst_start:
                lst_current[i].make_closed(i)
    return [path, path_value]


def algorithm2(draw, grid, lst_start, T_step, T_max, a_dgree):
    edges = []
    n = len(lst_start)
    lst_check = [0 for i in range(n)]
    lst_count = [0 for i in range(n)]
    lst_open_set = [[] for i in range(n)]
    lst_came_from = [{} for i in range(n)]
    lst_g_score = [{} for i in range(n)]
    lst_f_score = [{} for i in range(n)]
    lst_open_set_hash = [{} for i in range(n)]
    path = [[[] for j in range(n)] for i in range(n)]
    path_value = [[0 for j in range(n)] for i in range(n)]
    check_path = [[0 for j in range(n)] for i in range(n)]
    max_count_path = 0
    check_grid = {lst_start[0]}
    for i in range(n):
        lst_count[i] = 0
        lst_open_set[i] = PriorityQueue()
        lst_open_set[i].put((0, lst_count[i], lst_start[i]))
        lst_came_from[i] = {}
        lst_g_score[i] = {spot: float("inf") for row in grid for spot in row}
        lst_g_score[i][lst_start[i]] = 0
        lst_f_score[i] = {spot: float("inf") for row in grid for spot in row}
        lst_f_score[i][lst_start[i]] = lst_g_score[i][lst_start[i]]
        lst_open_set_hash[i] = {lst_start[i]}
        check_grid.add(lst_start[i])
    run = True
    # min_neighbors = 5 #round(math.sqrt(n) * math.log1p(n))
    # min_max_count_path = n*min_neighbors - (min_neighbors * min_neighbors - (n%min_neighbors*(n%min_neighbors)-1))
    while run:
        # print(max_count_path)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        lst_current = [None for i in range(n)]
        for i in range(n):
            lst_current[i] = lst_open_set[i].get()[2]
            for j in range(n):

                if lst_count[0] >= T_max:
                    # print(lst_count[0])
                    if check_path_valid(fix_path_map(path, path_value) + [edges]):
                        sum_edge = 0
                        for p in lst_check:
                            sum_edge += p
                        # print(lst_check)
                        # print(sum_edge / ((n - 1) * n))
                        return fix_path_map(path, path_value) + [edges]
                    else:
                        T_max += T_step
                if i != j and (lst_current[i] in lst_open_set_hash[j]) and check_path[i][j] == 0:
                    lst_check[i] += 1
                    lst_check[j] += 1
                    path_temp1 = [lst_current[i]] + reconstruct_path(lst_came_from[i], lst_current[i])
                    path_temp2 = [lst_current[i]] + reconstruct_path(lst_came_from[j], lst_current[i])
                    path_safe1 = reconstruct_path_safe(path_temp1, grid, lst_g_score[i], min_safe=1.71)
                    # path_safe1 = reconstruct_path_safe(path_safe1, grid, lst_g_score[i], min_safe=2.5)
                    path_safe2 = reconstruct_path_safe(path_temp2, grid, lst_g_score[j], min_safe=1.71)
                    # path_safe2 = reconstruct_path_safe(path_safe2, grid, lst_g_score[j], min_safe=2.5)
                    path_safe1.reverse()
                    path[i][j] = path_safe1 + path_safe2
                    temp_path = path[i][j].copy()
                    temp_path.reverse()
                    path[j][i] = temp_path
                    check_path[i][j] = check_path[j][i] = 1
                    path_value[i][j] = path_value[j][i] = lst_g_score[i][lst_current[i]] + lst_g_score[j][lst_current[i]]
                    edges.append((path_value[i][j], (i, j)))
                    max_count_path += 2
                    if max_count_path >= n*(n-1):
                        sum_edge = 0
                        for p in lst_check:
                            sum_edge += p
                        # print(lst_check)
                        print(sum_edge/((n-1)*n))
                        return fix_path_map(path, path_value) + [edges]
            for neighbor in lst_current[i].neighbors:
                temp_g_score = lst_g_score[i][lst_current[i]] + delta(lst_current[i].get_pos(), neighbor.get_pos())

                if temp_g_score < lst_g_score[i][neighbor]:
                    lst_came_from[i][neighbor] = lst_current[i]
                    lst_g_score[i][neighbor] = temp_g_score
                    lst_f_score[i][neighbor] = a_dgree * lst_g_score[i][neighbor] + lst_current[i].get_safe() * (-1 + a_dgree)
                    if neighbor.safe >= 1.71 and neighbor not in lst_open_set_hash[i]:
                        lst_count[i] += 1
                        lst_open_set[i].put((lst_f_score[i][neighbor], lst_count[i], neighbor))
                        neighbor.make_open(i)
                        lst_open_set_hash[i].add(neighbor)
                        # check_grid.add(neighbor)
                    else:
                        neighbor.make_open(i)
        # if lst_count[0] % 20 == 0:
        #     draw()
        for i in range(n):
            if lst_current[i] not in lst_start:
                lst_current[i].make_closed(i)
    return [path, path_value]


def main(win, width):
    choose = 2
    rows = 200
    count_test = 0
    perimeter_map = 0
    safe = float("inf")
    safe_medium = 0

    # filemap = "map_IN2D/IN2D_13.txt"
    # filemap = "map_health_care/HC_05.txt"
    # filemap = "map_DVRP/DVRP_04.txt"
    # filemap = "map_unknow/map_vd01.txt"
    # filemap = "map_NVIDIA/map_04_new.txt"
    filemap = "map_random/map_factory_200.txt"
    # filemap = "map_random/map_evenly_distributed_06.txt"
    num_goal = 2
    lst_goal = [None] * num_goal
    grid = make_grid(rows, width, safe)
    start = None
    end = None
    # for i in range(num_goal):
    #     lst_goal[i] = None
    goal_count = 0

    lst_start = []
    run = True
    draw(win, grid, rows, width)
    while run:
        # draw(win, grid, rows, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:  # LEFT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, rows, width)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()
                    lst_start.append(start)

                elif not end and spot != start:
                    end = spot
                    end.make_end()
                    lst_start.append(end)

                elif goal_count < num_goal and spot not in lst_start:
                    # print(goal_count)
                    lst_goal[goal_count] = spot
                    lst_goal[goal_count].make_end()
                    lst_start.append(lst_goal[goal_count])
                    goal_count += 1

                elif spot != end and spot != start and spot not in lst_goal:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, rows, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None
                if spot != start and spot != end and spot in lst_goal:
                    lst_goal[lst_goal.index(spot)] = None
                    goal_count -= 1

            # run
            if event.type == pygame.KEYDOWN:
                # if event.key == pygame.K_e:
                #     lst_start = read_map_from_file2(grid, rows, "square402.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.59)
                #     T_max = 160000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_r:
                #     lst_start = read_map_from_file2(grid, rows, "square401.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.59)
                #     T_max = 160000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_t:
                #     lst_start = read_map_from_file2(grid, rows, "square400.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.59)
                #     T_max = 160000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                if event.key == pygame.K_1:
                    choose = 0
                if event.key == pygame.K_2:
                    filemap = "map_NVIDIA/map_01.txt"
                if event.key == pygame.K_3:
                    filemap = "map_NVIDIA/map_02.txt"
                if event.key == pygame.K_4:
                    filemap = "map_NVIDIA/map_03.txt"
                if event.key == pygame.K_5:
                    filemap = "map_NVIDIA/map_04.txt"
                if event.key == pygame.K_8:
                    gen_random_map(filemap, 50)
                if event.key == pygame.K_7:
                    gen_random_map(filemap, 30)
                if event.key == pygame.K_6:
                    gen_random_map(filemap, 20)
                if event.key == pygame.K_9:
                    # filemap = "map_IN2D/IN2D_01.txt"
                    # gen_random_map(filemap, 58)
                    data = read_map_from_file2(grid, rows, filemap)
                    lst_start = data[0]
                    start = lst_start[0]
                    end = lst_start[1]
                    print(data[1])
                    draw(win, grid, rows, width)



                    # T_step = 200
                    # print(n, len(lst_start), T_max)
                # if event.key == pygame.K_w:
                #     lst_start = read_map_from_file2(grid, rows, "map200_RD2.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.45)
                #     T_max = 40000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_p:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/map200mix.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log2(len(lst_start)+1)) * (1 - 0.25)
                #     T_max = 40000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_u:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/obstacle95.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.92)
                #     T_max = 40000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_i:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/obstacle85.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.8)
                #     T_max = 40000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_o:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/warehouse4.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.36)
                #     T_max = 40000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                #     # print(len(lst_start))
                # if event.key == pygame.K_a:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/warehouse3.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.28)
                #     T_max = 40000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_s:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/mixed2002.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.251)
                #     T_max = 40000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_d:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/mixed200.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.25)
                #     T_max = 40000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_y:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/map200_00.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.17)
                #     T_max = 40000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_q:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/map200_RD.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.22)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_g:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/obstacle90.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.82)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_h:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/obstacle80.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.87)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_j:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/warehouse1.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.4)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start)) + 1
                # if event.key == pygame.K_k:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/warehouse2.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.62)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_v:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/map104_00.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.62)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_b:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/map103_00.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.62)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_n:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/map102_00.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.36)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_m:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/map101_00.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.41)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                # if event.key == pygame.K_l:
                #     lst_start = read_map_from_file2(grid, rows, "map_unknow/map100_00.txt")
                #     start = lst_start[0]
                #     end = lst_start[1]
                #     n = math.sqrt(len(lst_start)) * (math.log10(len(lst_start)+1)) * (1 - 0.17)
                #     T_max = 10000 / n
                #     min_neighbors = math.sqrt(len(lst_start))
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    if choose == 0:

                        count_test += 1
                        # print("new test: " + str(count_test))
                        # print_map(grid, rows, lst_goal)
                        T_max = 10000
                        T_step = 200
                        start_time = time.time()
                        temp_path = algorithm(lambda: draw(win, grid, rows, width), grid, lst_start, T_step, T_max, a_dgree)
                        path = temp_path[0]
                        path_value = temp_path[1]
                        edges = temp_path[2]
                        path_value = np.array(path_value)
                        # print(path_value)
                        # start_time = time.time()
                        c = CRIST(path_value)
                        ds = c.ans(edges)
                        # print(ds)
                        ds = sort_ds(ds)
                        # print("Số điểm đi là: ", len(ds) - 1)
                        end_time = time.time()
                        time_running = end_time - start_time
                        # print("Thời gian chạy là: ", time_running)
                        print(round(time_running, 4))
                        path_lenght = draw_path_final(path, path_value, ds, lst_start, grid, lambda: draw(win, grid, rows, width))
                        safety = calculate_safety(path, ds)
                        print(path_lenght)
                        # print("Độ an toàn trung bình FFT là: ", safety)
                        print(round(safety, 2))
                        print("F = ", safety * 1/math.log1p(2 - a_dgree) + perimeter_map/path_lenght * 1/math.log1p(1+a_dgree))
                        print("F2 = ", safety * 1/(a_dgree+1) + perimeter_map / path_lenght * 1 / (2 - a_dgree))

                        # start_time3 = time.time()
                        # GWO_value2 = run_GWO(path_value)
                        # end_time3 = time.time()
                        # time_running3 = end_time3 - start_time3

                        # start_time4 = time.time()
                        # aco = ACO(50, 200, 1.0, 20.0, 0.5, 10, 2)
                        # graph = Graph(path_value, len(path_value))
                        # aco_value = aco.solve(graph)
                        # end_time4 = time.time()
                        # time_running4 = end_time4 - start_time4

                        # aco2 = ACO(50, 100, 1.0, 20.0, 0.5, 10, 2)
                        # graph2 = Graph(path_value, len(path_value))
                        # aco_value2 = aco2.solve(graph2)

                        # print("Thời gian chạy ACO là: ", time_running4)
                        #print(aco_value[0])
                        # print("Độ dài quãng đường ACO là: ", aco_value[1])

                        # print("Độ dài quãng đường ACO2 là: ", aco_value2[1])


                        # print("Thời gian chạy GWO2 là: ", time_running3)
                        # # print(GWO_value[0])
                        # print("Độ dài quãng đường GWO2 là: ", GWO_value2[1])

                        # safety = calculate_safety(GWO_value[0])
                        # print("Độ an toàn trung bình GWO là: ", safety)
                        # print(path_value)

                    if choose == 1:
                        T_max = 40000
                        T_step = 0
                        start_time = time.time()
                        temp_path = algorithm2(lambda: draw(win, grid, rows, width), grid, lst_start, T_step, T_max)
                        path = temp_path[0]
                        path_value = temp_path[1]
                        edges = temp_path[2]
                        path_value = np.array(path_value)
                        c = CRIST(path_value)
                        ds = c.ans(edges)
                        print(ds)
                        ds = sort_ds(ds)
                        end_time = time.time()
                        time_running = end_time - start_time
                        # print("Thời gian chạy là: ", time_running)
                        print(round(time_running, 2))
                        draw_path_final(path, path_value, ds, lst_start, grid, lambda: draw(win, grid, rows, width))
                        safety = calculate_safety(path, ds)
                        # print("Độ an toàn trung bình FFT là: ", safety)
                        print(round(safety, 2))
                        break

                    if choose == 2:
                        T_max = 1500
                        T_step = 200
                        a_dgree = 0.65
                        delta_a_degree = 0
                        max_n = 1
                        dsF = []
                        dsF2 = []
                        ds_max = PriorityQueue()
                        ds_max2 = PriorityQueue()
                        lst_D = []
                        lst_S = []
                        lst_runtime = []
                        lst_F = []
                        for i in range(max_n):
                            print("cout =", i+1)
                            # gen_random_map(filemap, 30)
                            data = read_map_from_file2(grid, rows, filemap)
                            lst_start = data[0]
                            start = lst_start[0]
                            end = lst_start[1]
                            safe_medium = safe_zone(lambda: draw(win, grid, rows, width), rows, grid)
                            perimeter_map = cal_perimeter(filemap, rows)
                            a_dgree += delta_a_degree
                            start_time = time.time()
                            temp_path = algorithm(lambda: draw(win, grid, rows, width), grid, lst_start, T_step, T_max, a_dgree)
                            path = temp_path[0]
                            path_value = temp_path[1]
                            edges = temp_path[2]
                            path_value = np.array(path_value)
                            c = CRIST(path_value)
                            ds = c.ans(edges)
                            ds = sort_ds(ds)
                            end_time = time.time()
                            time_running = end_time - start_time
                            path_lenght = draw_path_final(path, path_value, ds, lst_start, grid, lambda: draw(win, grid, rows, width))
                            safety = calculate_safety(path, ds)
                            # safety = min_safety(path, ds)
                            if path_lenght < 200:
                                max_n += 1
                                continue

                            lst_D.append(path_lenght)
                            lst_S.append(safety)
                            lst_runtime.append(time_running)
                            F = safety/safe_medium * 1 / math.log1p(1 - a_dgree) + perimeter_map / path_lenght * 1 / math.log1p(a_dgree)
                            lst_F.append(F)


                            # F = safety/safe_medium * 1 / math.log1p(2 - a_dgree) + perimeter_map / path_lenght * 1 / math.log1p(1 + a_dgree)
                            # F2 = 1.8 * safety * 1 / (2 - a_dgree) + perimeter_map / path_lenght * 1 / (1 + a_dgree)
                            # dsF.append(F)
                            # dsF2.append(F2)
                            # ds_max.put(([1/math.exp(F)], [a_dgree, F, time_running, path_lenght, safety]))
                            # ds_max2.put(([1/math.exp(F2)], [a_dgree, F2, time_running, path_lenght, safety]))

                        # tt_result = ds_max.get()
                        # tt_result2 = ds_max2.get()
                        # print(tt_result)
                        # print(tt_result2)
                        # plt.plot(dsF, label='mảng F', marker='o', linestyle='-')
                        # plt.plot(dsF2, label='mảng F2', marker='x', linestyle='-')
                        # plt.title('Biểu đồ đường của mảng F và F2')
                        # plt.xlabel('Chỉ số')
                        # plt.ylabel('Giá trị')
                        # plt.show()

                        D1, D2 = mean_avg(lst_D)
                        S1, S2 = mean_avg(lst_S)
                        T1, T2 = mean_avg(lst_runtime)
                        F1, F2 = mean_avg(lst_F)
                        print(round(D1,3), round(D2,3))
                        print(round(S1,3), round(S2,3))
                        print(round(T1,3), round(T2,3))
                        print(round(F1,3), round(F2,3))
                        # write_file_resault("resault/scenario_3.txt", lst_D, "D = ")
                        # write_file_resault("resault/scenario_3.txt", lst_S, "S = ")
                        # write_file_resault("resault/scenario_3.txt", lst_runtime, "T = ")
                        # write_file_resault("resault/scenario_3.txt", lst_F, "F = ")



                if event.key == pygame.K_c:
                    lst_start = []
                    lst_goal = [None] * num_goal
                    goal_count = 0
                    grid = make_grid(rows, width, safe)

                if event.key == pygame.K_f:
                    safe_medium = safe_zone(lambda: draw(win, grid, rows, width), rows, grid)
                    perimeter_map = cal_perimeter(filemap, rows)
                    print(safe_medium)
                    print(perimeter_map)
                if event.key == pygame.K_d:
                    regions, segments = partition_environment(grid)
                    segments = [(p1, p2) for p1, p2 in segments if
                                line_of_sight((int(p1[0] / 4), int(p1[1] / 4)), (int(p2[0] / 4), int(p2[1] / 4)), grid)]
                    new_segments = fix_segments(segments, grid)
                    for p1, p2 in new_segments:
                        pygame.draw.line(WIN, (255, 0, 0), p1, p2, 3)
                    pygame.display.update()
                    print("updated")

            # end

    pygame.quit()


main(WIN, WIDTH)
