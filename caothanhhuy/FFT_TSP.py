from TSP_cristofides_2 import *
import math


# nhánh cận chay
def tsp(distance_matrix):
    num_cities = len(distance_matrix)
    visited = [False] * num_cities  # Danh sách kiểm tra các thành phố đã được thăm
    path = [0]  # Đường đi bắt đầu từ điểm 0
    min_cost = math.inf  # Giá trị lớn nhất để so sánh và cập nhật chi phí nhỏ nhất
    best_path = []  # Danh sách các điểm đi qua trong chu trình ngắn nhất

    def calculate_cost():
        # Tính toán chi phí của đường đi hiện tại
        cost = 0
        for i in range(len(path) - 1):
            cost += distance_matrix[path[i]][path[i+1]]
        cost += distance_matrix[path[-1]][path[0]]  # Cộng chi phí trở lại điểm ban đầu
        return cost

    def backtrack(curr_city, cost):
        nonlocal min_cost
        nonlocal best_path

        if len(path) == num_cities:  # Nếu đã đi qua tất cả các thành phố
            if distance_matrix[curr_city][0] != 0:  # Kiểm tra có thể quay trở lại điểm ban đầu
                cost += distance_matrix[curr_city][0]  # Cộng chi phí trở lại điểm ban đầu
                if cost < min_cost:  # Nếu chi phí nhỏ hơn chi phí nhỏ nhất hiện tại
                    min_cost = cost
                    best_path = path.copy()
            return

        for next_city in range(num_cities):
            if not visited[next_city] and distance_matrix[curr_city][next_city] != 0:
                # Thăm thành phố tiếp theo nếu chưa được thăm và có đường đi hợp lệ
                visited[next_city] = True
                path.append(next_city)
                backtrack(next_city, cost + distance_matrix[curr_city][next_city])
                # Quay lui và thử các thành phố khác
                visited[next_city] = False
                path.pop()

    visited[0] = True  # Đánh dấu điểm đầu tiên đã được thăm
    backtrack(0, 0)  # Bắt đầu từ điểm đầu tiên
    best_path.append(0)  # Thêm điểm ban đầu vào cuối danh sách để tạo chu trình
    return best_path


# nhánh cận hiệu quả
def efficient_branching(distance_matrix):
    num_cities = distance_matrix.shape[0]
    visited = [False] * num_cities
    visited[0] = True
    current_path = [0]
    min_cost = np.inf
    optimal_path = None

    def branch_and_bound(current_city, current_cost, depth):
        nonlocal min_cost, optimal_path

        if depth == num_cities - 1:
            total_cost = current_cost + distance_matrix[current_city, 0]
            if total_cost < min_cost:
                min_cost = total_cost
                optimal_path = current_path[:]
                optimal_path.append(0)
            return

        for next_city in range(1, num_cities):
            if not visited[next_city] and distance_matrix[current_city, next_city] < 200:
                current_cost += distance_matrix[current_city, next_city]
                if current_cost < min_cost:
                    visited[next_city] = True
                    current_path.append(next_city)
                    branch_and_bound(next_city, current_cost, depth + 1)
                    current_path.pop()
                    visited[next_city] = False
                current_cost -= distance_matrix[current_city, next_city]

    branch_and_bound(0, 0, 0)

    return optimal_path


def check_fast_marching_tsp(lst_check, T, T_max, min_neighbors):
    n = len(lst_check)
    for i in range(n):
        if lst_check[i] < min_neighbors and T >= T_max:
            T_max += 40000/(len(lst_check)**2)
            # T_max += 100
            return False
    return True


def check_path_valid(paths):
    path_value = paths[1]
    edges = paths[2]
    c = CRIST(path_value)
    ds = c.ans(edges)

    # Tính toán độ dài đường đi
    n = len(ds)
    path_length = 0
    count = 1
    while count < n:
        path_length += path_value[ds[count - 1]][ds[count]]
        count += 1
    # Kiểm tra đường đi hợp lệ
    if path_length < 10000:
        return True
    else:
        # print(path_length)
        return False


def reconstruct_path(came_from, current):
    lst = []
    while current in came_from:
        current = came_from[current]
        lst.append(current)
    return lst


def find_hamiltonian_tour(eulerian_tour):
    visited = set()
    hamiltonian_tour = []

    for vertex in eulerian_tour:
        if vertex not in visited:
            visited.add(vertex)
            hamiltonian_tour.append(vertex)

    hamiltonian_tour.append(hamiltonian_tour[0])  # Go back to the starting city
    return hamiltonian_tour


# Fast marching đơn lẻ
def fast_marching(grid, start, end):
    lst = []
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = g_score[start]

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        current = open_set.get()[2]
        if current == end:
            lst = reconstruct_path(came_from, end, draw)
            start.make_start()
            end.make_end()
            return lst

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + delta(current.get_pos(), neighbor.get_pos())

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = g_score[neighbor]
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    neighbor.make_open()
                    open_set_hash.add(neighbor)
                else:
                    neighbor.make_open()

        # draw()

        if current != start:
            current.make_closed()

    return lst


def penalty_function(x, y):
    f_score = 0
    return f_score


def calculate_safety(path, ds):
    num = 0
    safety = 0
    min_safety = 3
    n = len(ds)
    count = 1
    while count < n:
        lst = path[ds[count-1]][ds[count]]
        for point in lst:
            safety += point.get_safe()
            # print(point.get_safe())
            # if point.get_safe() > 1000:
                # safety += 0
            # else:
            # if min_safety < point.get_safe():
            #     continue
            # else:
            #     safety += -1/min_safety + 1/point.get_safe()
            num += 1
        count += 1

    if num != 0:
        safety = safety/num
    # safety = 0.5/math.exp(safety/10)
    return safety


def min_safety(path, ds):
    min_sf = 999
    n = len(ds)
    count = 1
    while count < n:
        lst = path[ds[count - 1]][ds[count]]
        for point in lst:
            if point.get_safe() < min_sf:
                min_sf = point.get_safe()
        count += 1
    return min_sf


def mean_avg(lst):
    m = 0
    n = 0
    for i in range(len(lst)):
        m += lst[i]
    m = m/len(lst)
    for i in range(len(lst)):
        n = n + (m-lst[i])*(m-lst[i])
    n = math.sqrt(n/len(lst))

    return m, n