from Map_Grid import *
from collections import defaultdict
from scipy.spatial import Voronoi, voronoi_plot_2d
from xlwt import Utils


def partition_environment(grid):
    # Create a list of obstacle points from the grid
    obstacle_points = []
    for row in grid:
        for spot in row:
            if spot.is_barrier():
                obstacle_points.append((spot.get_x(), spot.get_y()))

    # Compute Voronoi diagram
    # print(obstacle_points)
    map_voronoi = Voronoi(obstacle_points, incremental=True)

    # Create a list of line segments representing the Voronoi diagram
    segments = []
    for i, j in map_voronoi.ridge_vertices:
        if i >= 0 and j >= 0:
            p1 = (map_voronoi.vertices[i][0], map_voronoi.vertices[i][1])
            p2 = (map_voronoi.vertices[j][0], map_voronoi.vertices[j][1])
            segments.append((p1, p2))
    # print(segments)

    # Partition the environment into regions based on the MAP.voronoi diagram
    regions = []
    for i, region in enumerate(map_voronoi.regions):
        if -1 not in region:
            polygon = [map_voronoi.vertices[i] for i in region]
            regions.append(polygon)

    # Convert point in segments to integer
    for i, segment in enumerate(segments):
        segments[i] = ((int(segment[0][0]), int(segment[0][1])), (int(segment[1][0]), int(segment[1][1])))

    return regions, segments


def line_of_sight(p1, p2, grid):
    # print(p1,p2)
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    f = 0

    if dy < 0:
        dy = -dy
        sy = -1
    else:
        sy = 1
    if dx < 0:
        dx = -dx
        sx = -1
    else:
        sx = 1

    # print(x1 + ((sx - 1) // 2))
    # print(y1+1)
    if dx >= dy:
        while x1 != x2:
            f = f + dy
            if f >= dx:
                if grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier():
                    return False
                y1 = y1 + sy
                f = f - dx

            if f != 0 and grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier():
                return False

            if dy == 0:
                if grid[x1 + ((sx - 1) // 2)][y1].is_barrier() or grid[x1 + ((sx - 1) // 2)][y1 - 1].is_barrier(): # or grid[x1 + ((sx - 1) // 2)][y1 + 1].is_barrier():
                    return False

            x1 = x1 + sx
    else:
        while y1 != y2:
            f = f + dx
            if f >= dy:
                if grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier():
                    return False
                x1 = x1 + sx
                f = f - dy

            if f != 0 and grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier():
                return False

            if dx == 0:
                if grid[x1][y1 + ((sy - 1) // 2)].is_barrier() or grid[x1 - 1][y1 + ((sy - 1) // 2)].is_barrier(): # or grid[x1 + 1][y1 + ((sy - 1) // 2)].is_barrier():
                    return False

            y1 = y1 + sy

    return True


def fix_segments(segments, grid):
    lst = []
    for p1, p2 in segments:
        if grid[int(p1[0]/4)][int(p1[1]/4)].safe >= 2 and grid[int(p2[0]/4)][int(p2[1]/4)].safe >= 2:
        # if len(grid[int(p1[0]/4)][int(p1[1]/4)].update_neighbors(grid)) == 8 and len(grid[int(p2[0]/4)][int(p2[1]/4)].update_neighbors(grid)) == 8:
            lst.append((p1, p2))
    return lst


def nearst_to_voronoi(segments, lst_start, grid):
    pass


def TSP_voronoi(segments, lst_start, grid):
    start = lst_start[0]
    n = len(lst_start)
    lst_came_from = [[] for i in range(n)]
    lst_open_set = [[] for i in range(n)]
    lst_open_set_hash = [{} for i in range(n)]
    lst_count = [0 for i in range(n)]
    lst_g_score = [{} for i in range(n)]
    lst_f_score = [{} for i in range(n)]

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

    run = True
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        lst_current = [None for i in range(n)]
        for i in range(n):
            lst_current[i] = lst_open_set[i].get()[2]

            if lst_current[i] in segments:
                pass

    return True