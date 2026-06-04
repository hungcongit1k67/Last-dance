"""
My_grid.py - WP-FMF (Weighted Potential Fast Marching Firework)
=================================================================
Triển khai thuật toán WP-FMF theo paper:
    "WP-FMF: Phương pháp Weighted Potential Fast Marching Firework
     cho quy hoạch đường đi đa đích đến của robot di động
     trong môi trường nhà máy hạt nhân"

Cải tiến so với FMF cổ điển:
- Chỉ số an toàn S(c) trên lân cận 11×11 (eq. 4)
- Hàm chi phí cục bộ f(x) = w1 + w2·R̄_norm(x) + w3·(1−S(x)) (eq. 11)
- Bản đồ phóng xạ R̄(x) tích hợp trực tiếp vào lan truyền pháo hoa
- Cập nhật Eikonal dạng sai phân: T(y) = T(x) + d(x,y)·f(y) (eq. 13a)
- Bridge cost: d(x,y)·(f(x)+f(y))/2 (eq. 14a)
- Pipeline 2 pha: Expanding (sơ cấp) + Intersecting (thứ cấp)
"""

import heapq
import math
import os
import queue
import random
import numpy as np
import matplotlib.pyplot as plt

try:
    import pygame
except ImportError:
    pygame = None

try:
    from supercover import supercover_cells
except ImportError:  # fallback khi cwd khác thư mục module
    import sys as _sys
    _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from supercover import supercover_cells


class GridMap:
    # ---------- Color palettes cho visualization ----------
    colorHold = ['blue', 'green', 'gold', 'tan', 'maroon', 'orange',
                 'cyan', 'violet', 'salmon', 'lime', 'darkslateblue']

    # ---------- 8 hướng lân cận ----------
    DIRS8 = [(-1, 0), (0, 1), (1, 0), (0, -1),
             (-1, 1), (1, 1), (1, -1), (-1, -1)]

    # ---------- Giá trị grid: 0 = trống, 1 = vật cản, 2 = checkpoint ----------

    def __init__(self, mapSize, square_width=20, square_height=20, margin=1):
        self.mapSize = mapSize
        self.square_width = square_width
        self.square_height = square_height
        self.margin = margin
        self.gridMap = []
        self.npos = 0
        self.deslist = []
        self.mksz = 10

        # Trọng số WP-FMF — chỉnh qua config()
        self.w1 = 0.6    # trọng số chiều dài đường đi
        self.w2 = 0.2    # trọng số độ rủi ro phóng xạ R(P)
        self.w3 = 0.2    # trọng số độ rủi ro va chạm risk(P)
        self.C1 = 0.5    # cân bằng N_obs (→1) vs d_min (→0) trong S(c)
        self.a  = 1.0    # kích thước ô lưới (m) — dùng trong công thức R(P)
        self.v  = 1.0    # vận tốc robot (m/s)   — dùng trong công thức R(P)
        self.safety_radius       = 5    # bán kính vùng lân cận tính S(c)
        self.safety_max_distance = 7.0  # khoảng cách chuẩn hóa d_min trong S(c)

        # Hàm mục tiêu mà TSP solver tối thiểu hóa:
        #   True  → ma trận TSP = Total cost (7a) w1·length+w2·R+w3·risk của từng đoạn
        #           ⇒ solver minimize đúng  min w1·length(P)+w2·R(P)+w3·Risk(P)  (TSP cost == Total cost).
        #   False → ma trận TSP = thế năng WP-FMF có trọng số f (hành vi gốc).
        self.Solver_minimize = True

        # supercover=False: risk(P) & R(P) dùng công thức gốc (trung bình 2 đầu mút mỗi đoạn).
        # supercover=True : đoạn KHÔNG kề nhau (path smooth/turning points) tính trung bình
        #   (1−S)/R̄ trên toàn bộ ô mà đoạn thẳng cắt qua (supercover line); đoạn KỀ nhau
        #   (cell-by-cell) vẫn dùng công thức gốc.
        self.supercover = False

        # Bản đồ phóng xạ
        self.radiation_map  = None   # giá trị thô (μSv/h hoặc đơn vị tương đương)
        self.radiation_norm = None   # chuẩn hóa về [0, 1]

        self.window_size = [mapSize * square_width + (mapSize + 1) * margin,
                            mapSize * square_height + (mapSize + 1) * margin]
        self.DFType = "WP-FMF"

    # =========================================================
    # Cấu hình tham số
    # =========================================================
    def config(self, w1=None, w2=None, w3=None, C1=None, a=None, v=None,
               safety_radius=None, safety_max_distance=None, Solver_minimize=None,
               supercover=None):
        """Cấu hình tham số thuật toán.

        Trọng số (w1 + w2 + w3 = 1):
          w1 – chiều dài đường đi length(P)
          w2 – độ rủi ro phóng xạ R(P)
          w3 – độ rủi ro va chạm risk(P)

        An toàn va chạm (công thức 4):
          C1                – cân bằng N_obs (C1→1) vs d_min (C1→0)
          safety_radius     – bán kính vùng lân cận tính S(c)
          safety_max_distance – khoảng cách chuẩn hóa d_min

        Vật lý (công thức 6):
          a  – kích thước ô lưới (m)
          v  – vận tốc robot (m/s)

        Solver_minimize – True: ma trận TSP = Total cost (7a) → solver minimize đúng
          w1·length(P)+w2·R(P)+w3·Risk(P) (TSP cost == Total cost);
          False: dùng thế năng WP-FMF có trọng số f (gốc).
        """
        w1_ = float(w1) if w1 is not None else self.w1
        w2_ = float(w2) if w2 is not None else self.w2
        w3_ = float(w3) if w3 is not None else self.w3
        if abs(w1_ + w2_ + w3_ - 1.0) > 1e-6:
            raise ValueError(
                f"w1+w2+w3 phải bằng 1.0. "
                f"Hiện tại: {w1_:.4f}+{w2_:.4f}+{w3_:.4f} = {w1_+w2_+w3_:.4f}"
            )
        self.w1, self.w2, self.w3 = w1_, w2_, w3_
        if C1 is not None:
            self.C1 = float(C1)
        if a is not None:
            self.a = float(a)
        if v is not None:
            self.v = float(v)
        if safety_radius is not None:
            self.safety_radius = int(safety_radius)
        if safety_max_distance is not None:
            self.safety_max_distance = float(safety_max_distance)
        if Solver_minimize is not None:
            self.Solver_minimize = bool(Solver_minimize)
        if supercover is not None:
            self.supercover = bool(supercover)

    def setWeights(self, w1=0.7, C1=0.5):
        """Giữ lại cho tương thích ngược. Khuyến nghị dùng config() thay thế."""
        self.w1 = float(w1)
        self.C1 = float(C1)

    # =========================================================
    # I/O bản đồ
    # =========================================================
    def load_radiation_map(self, file_path):
        """Nạp bản đồ phóng xạ từ file.

        Định dạng: mỗi dòng là một hàng lưới, các giá trị cách nhau bởi khoảng trắng.
        """
        with open(file_path, 'r') as f:
            lines = [line.strip() for line in f if line.strip()]
        self.radiation_map = [[float(x) for x in line.split()] for line in lines]
        nrows = len(self.radiation_map)
        ncols = len(self.radiation_map[0]) if nrows > 0 else 0
        print(f"  Radiation map: {nrows}×{ncols}  [{os.path.basename(file_path)}]")

    def get_grid_from_file(self, file_path):
        with open(file_path, 'r') as f:
            info = f.read().split('\n')
        msz = int(info[0])
        npos = 0
        points = []
        gr = info[1:]
        for i in range(msz):
            gr[i] = gr[i].split(' ')
            for j in range(msz):
                gr[i][j] = int(gr[i][j])
                if gr[i][j] == 2:
                    points.append((i, j))
                    npos += 1
        self.mapSize = msz
        self.npos = npos
        self.deslist = points
        self.gridMap = gr
        self.mksz = int(20 * 20 / msz + 1)

        # Tự động nạp bản đồ phóng xạ nếu tồn tại cùng thư mục
        rad_path = os.path.join(
            os.path.dirname(os.path.abspath(file_path)), 'radiation_grid.txt'
        )
        if os.path.exists(rad_path):
            self.load_radiation_map(rad_path)

    def create_grid_map(self, npos):
        """Tạo bản đồ tương tác qua pygame (giữ lại từ bản gốc)."""
        if pygame is None:
            raise RuntimeError("pygame chưa được cài đặt.")
        self.npos = npos
        self.deslist = [(0, 0)] * npos
        WIDTH, HEIGHT, MARGIN = self.square_width, self.square_height, self.margin
        grid = [[0] * self.mapSize for _ in range(self.mapSize)]

        pygame.init()
        scr = pygame.display.set_mode(self.window_size)
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
                    col = pos[0] // (WIDTH + MARGIN)
                    row = pos[1] // (HEIGHT + MARGIN)
                    if i < npos:
                        grid[row][col] = 2
                        self.deslist[i] = (row, col)
                        i += 1
                    else:
                        grid[row][col] = 1
            scr.fill((0, 0, 0))
            for row in range(self.mapSize):
                for col in range(self.mapSize):
                    color = (255, 255, 255)
                    if grid[row][col] == 1:
                        color = (0, 0, 0)
                    elif grid[row][col] == 2:
                        color = (0, 255, 0)
                    pygame.draw.rect(scr, color,
                                     [(MARGIN + WIDTH) * col + MARGIN,
                                      (MARGIN + HEIGHT) * row + MARGIN,
                                      WIDTH, HEIGHT])
            clock.tick(50)
            pygame.display.flip()
        pygame.quit()
        self.gridMap = grid
        return grid

    # =========================================================
    # Tiện ích hình học
    # =========================================================
    def validpos(self, u, v):
        u, v = int(u), int(v)
        if u < 0 or u >= self.mapSize or v < 0 or v >= self.mapSize:
            return False
        return self.gridMap[u][v] != 1

    def distant(self, x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)

    def goStraight(self, p1, p2):
        if self.gridMap[p1[0]][p2[1]] == 1:
            return 0
        if self.gridMap[p2[0]][p1[1]] == 1:
            return 0
        return 1

    def buildSumBlock(self):
        """Prefix-sum 2D để đếm nhanh số vật cản trong hình chữ nhật."""
        sz = self.mapSize
        self.sumBlock = [[0] * (sz + 1) for _ in range(sz + 1)]
        for i in range(sz):
            for j in range(sz):
                self.sumBlock[i + 1][j + 1] = (self.sumBlock[i + 1][j]
                                               + self.sumBlock[i][j + 1]
                                               - self.sumBlock[i][j])
                if self.gridMap[i][j] == 1:
                    self.sumBlock[i + 1][j + 1] += 1

    def countBlock(self, x1, y1, x2, y2):
        x1 += 1; y1 += 1; x2 += 1; y2 += 1
        if x1 > x2: x1, x2 = x2, x1
        if y1 > y2: y1, y2 = y2, y1
        return (self.sumBlock[x2][y2] - self.sumBlock[x1 - 1][y2]
                - self.sumBlock[x2][y1 - 1] + self.sumBlock[x1 - 1][y1 - 1])

    def connectable(self, x1, y1, x2, y2):
        """Kiểm tra "line of sight" giữa 2 ô (dùng prefix-sum)."""
        vec = 0
        if x1 > x2:
            x1, x2 = x2, x1
            vec ^= 1
        if y1 > y2:
            y1, y2 = y2, y1
            vec ^= 1
        tot = self.countBlock(x1, y1, x2, y2)
        dx = (x2 - x1 + 1) // 2 - 1
        dy = (y2 - y1 + 1) // 2 - 1
        t1 = t2 = 0
        if dx > 0 and dy > 0:
            if vec == 1:
                t1 = self.countBlock(x1, y1, x1 + dx - 1, y1 + dy - 1)
                t2 = self.countBlock(x2 - dx + 1, y2 - dy + 1, x2, y2)
            else:
                t1 = self.countBlock(x2 - dx + 1, y1, x2, y1 + dy - 1)
                t2 = self.countBlock(x1, y2 - dy + 1, x1 + dx - 1, y2)
        return tot - t1 - t2

    # =========================================================
    # WP-FMF bước 1: tính S(c), chuẩn hoá phóng xạ, rồi f(x)
    # =========================================================
    def computeSafety(self):
        """Công thức (4): S(c) = C1·(N_max−N_obs)/N_max + (1−C1)·d_min/safety_max_distance

        Bán kính lân cận và khoảng cách chuẩn hóa chỉnh qua config().
        """
        sz = self.mapSize
        r = self.safety_radius
        n_max = float((2 * r + 1) ** 2 - 1)  # số ô trong vùng lân cận (trừ tâm)
        d_max = self.safety_max_distance
        self.safety = [[0.0] * sz for _ in range(sz)]
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    self.safety[i][j] = 0.0
                    continue
                n_obs = 0
                d_min = d_max
                for di in range(-r, r + 1):
                    for dj in range(-r, r + 1):
                        if di == 0 and dj == 0:
                            continue
                        ni, nj = i + di, j + dj
                        if 0 <= ni < sz and 0 <= nj < sz:
                            if self.gridMap[ni][nj] == 1:
                                n_obs += 1
                                d = math.hypot(di, dj)
                                if d < d_min:
                                    d_min = d
                self.safety[i][j] = (self.C1 * (n_max - n_obs) / n_max
                                     + (1.0 - self.C1) * d_min / d_max)

    def _normalize_radiation(self):
        """Chuẩn hóa R̄(x) về [0, 1] dựa trên max tại các ô không phải vật cản."""
        if self.radiation_map is None:
            self.radiation_norm = None
            return
        sz = self.mapSize
        nrows = len(self.radiation_map)
        ncols = len(self.radiation_map[0]) if nrows > 0 else 0
        rmax = 0.0
        for i in range(min(sz, nrows)):
            for j in range(min(sz, ncols)):
                if self.gridMap[i][j] != 1:
                    val = self.radiation_map[i][j]
                    if val > rmax:
                        rmax = val
        if rmax == 0.0:
            rmax = 1.0
        self.radiation_norm = [[0.0] * sz for _ in range(sz)]
        for i in range(sz):
            for j in range(sz):
                if i < nrows and j < ncols:
                    self.radiation_norm[i][j] = self.radiation_map[i][j] / rmax

    def computeFCost(self):
        """Công thức (11): f(x) = w1 + w2·R̄_norm(x) + w3·(1−S(x))."""
        sz = self.mapSize
        INF = float('inf')
        self.f_cost = [[INF] * sz for _ in range(sz)]
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    self.f_cost[i][j] = INF
                else:
                    r_norm = (self.radiation_norm[i][j]
                              if self.radiation_norm is not None else 0.0)
                    self.f_cost[i][j] = (self.w1
                                         + self.w2 * r_norm
                                         + self.w3 * (1.0 - self.safety[i][j]))

    # =========================================================
    # Algorithm 2: AddEdge (cải tiến bridge cost)
    # =========================================================
    def _add_edge_wp(self, x, y, stage):
        """
        cost_bridge = d(x,y) · (f(x)+f(y))/2
        D[u,v]      = T[x] + T[y] + cost_bridge   (eq. 14a)
        """
        fx = self.F_label[x[0]][x[1]]
        fy = self.F_label[y[0]][y[1]]
        if fx == fy or fx == -1 or fy == -1:
            return
        d_xy = math.hypot(x[0] - y[0], x[1] - y[1])
        fxv = self.f_cost[x[0]][x[1]]
        fyv = self.f_cost[y[0]][y[1]]
        cost_bridge = d_xy * (fxv + fyv) / 2.0
        total = self.T[x[0]][x[1]] + self.T[y[0]][y[1]] + cost_bridge
        if total < self.adj[fx][fy]:
            self.adj[fx][fy] = total
            self.adj[fy][fx] = total
            self.inters[fx][fy] = (tuple(x), tuple(y))
            self.inters[fy][fx] = (tuple(y), tuple(x))
            self.M[fx][fy] = stage
            self.M[fy][fx] = stage

    # =========================================================
    # Algorithm 1 pha 1: Expanding stage (multi-source Dijkstra có trọng số)
    # =========================================================
    def _firework_primary(self):
        sz = self.mapSize
        INF = float('inf')
        n = self.npos

        self.T = [[INF] * sz for _ in range(sz)]
        self.F_label = [[-1] * sz for _ in range(sz)]
        self.trace_p = [[(-1, -1)] * sz for _ in range(sz)]

        self.adj = [[INF] * n for _ in range(n)]
        self.inters = [[((-1, -1), (-1, -1))] * n for _ in range(n)]
        self.M = [[0] * n for _ in range(n)]

        pq = []
        for i, pos in enumerate(self.deslist):
            r, c = pos
            self.T[r][c] = 0.0
            self.F_label[r][c] = i
            heapq.heappush(pq, (0.0, r, c))

        while pq:
            t, r, c = heapq.heappop(pq)
            if t > self.T[r][c] + 1e-12:
                continue
            for dr, dc in self.DIRS8:
                nr, nc = r + dr, c + dc
                if nr < 0 or nr >= sz or nc < 0 or nc >= sz:
                    continue
                if self.gridMap[nr][nc] == 1:
                    continue
                d_step = math.hypot(dr, dc)
                cost_step = d_step * self.f_cost[nr][nc]
                new_t = self.T[r][c] + cost_step
                if new_t + 1e-12 < self.T[nr][nc]:
                    self.T[nr][nc] = new_t
                    self.F_label[nr][nc] = self.F_label[r][c]
                    self.trace_p[nr][nc] = (r, c)
                    heapq.heappush(pq, (new_t, nr, nc))

        self.hold = [[] for _ in range(n)]
        for i in range(sz):
            for j in range(sz):
                if self.F_label[i][j] != -1:
                    self.hold[self.F_label[i][j]].append((i, j))

        for i in range(sz):
            for j in range(sz):
                if self.F_label[i][j] == -1:
                    continue
                for dr, dc in self.DIRS8:
                    ni, nj = i + dr, j + dc
                    if ni < 0 or ni >= sz or nj < 0 or nj >= sz:
                        continue
                    if self.F_label[ni][nj] == -1:
                        continue
                    if self.F_label[ni][nj] != self.F_label[i][j]:
                        self._add_edge_wp((i, j), (ni, nj), stage=1)

    # =========================================================
    # Algorithm 1 pha 2: Intersecting stage (kết nối thứ cấp)
    # =========================================================
    def _firework_secondary(self):
        sz = self.mapSize
        INF = float('inf')

        self.trace_s = [[(-1, -1)] * sz for _ in range(sz)]

        for i in range(self.npos):
            hold_cells = list(self.hold[i])
            if not hold_cells:
                continue

            saved = {}
            hold_set = set(hold_cells)
            for (r, c) in hold_cells:
                saved[(r, c)] = (self.T[r][c], self.F_label[r][c])
                self.T[r][c] = INF
                self.F_label[r][c] = -1

            pq = []
            seeded = set()
            for (r, c) in hold_cells:
                for dr, dc in self.DIRS8:
                    nr, nc = r + dr, c + dc
                    if nr < 0 or nr >= sz or nc < 0 or nc >= sz:
                        continue
                    if self.F_label[nr][nc] == -1:
                        continue
                    if (nr, nc) in seeded:
                        continue
                    seeded.add((nr, nc))
                    heapq.heappush(pq, (self.T[nr][nc], nr, nc))

            while pq:
                t, r, c = heapq.heappop(pq)
                if t > self.T[r][c] + 1e-12:
                    continue
                for dr, dc in self.DIRS8:
                    nr, nc = r + dr, c + dc
                    if nr < 0 or nr >= sz or nc < 0 or nc >= sz:
                        continue
                    if self.gridMap[nr][nc] == 1:
                        continue
                    if (nr, nc) not in hold_set:
                        continue
                    d_step = math.hypot(dr, dc)
                    cost_step = d_step * self.f_cost[nr][nc]
                    new_t = self.T[r][c] + cost_step
                    if new_t + 1e-12 < self.T[nr][nc]:
                        self.T[nr][nc] = new_t
                        self.F_label[nr][nc] = self.F_label[r][c]
                        self.trace_s[nr][nc] = (r, c)
                        heapq.heappush(pq, (new_t, nr, nc))

            for (r, c) in hold_cells:
                if self.F_label[r][c] == -1:
                    continue
                for dr, dc in self.DIRS8:
                    nr, nc = r + dr, c + dc
                    if nr < 0 or nr >= sz or nc < 0 or nc >= sz:
                        continue
                    if self.F_label[nr][nc] == -1:
                        continue
                    if self.F_label[nr][nc] != self.F_label[r][c]:
                        self._add_edge_wp((r, c), (nr, nc), stage=2)

            for (r, c), (t0, f0) in saved.items():
                self.T[r][c] = t0
                self.F_label[r][c] = f0

    # =========================================================
    # Dựng lại path ô-by-ô giữa mọi cặp source
    # =========================================================
    def _backtrack_to_source(self, start_cell, target_source):
        sr, sc = self.deslist[target_source]
        target = (sr, sc)
        cur = tuple(start_cell)
        path = [cur]
        visited = {cur}
        max_iter = self.mapSize * self.mapSize + 5
        for _ in range(max_iter):
            if cur == target:
                break
            pl = self.F_label[cur[0]][cur[1]]
            if pl == target_source:
                parent = self.trace_p[cur[0]][cur[1]]
            else:
                parent = self.trace_s[cur[0]][cur[1]]
                if parent == (-1, -1):
                    parent = self.trace_p[cur[0]][cur[1]]
            if parent == (-1, -1) or parent in visited:
                break
            cur = parent
            path.append(cur)
            visited.add(cur)
        return path

    def _smooth_path(self, cells):
        """Làm mịn đường đi bằng line-of-sight check."""
        if len(cells) <= 2:
            return [list(c) for c in cells]
        out = [list(cells[0])]
        anchor = cells[0]
        for i in range(1, len(cells)):
            if self.connectable(anchor[0], anchor[1], cells[i][0], cells[i][1]) != 0:
                out.append(list(cells[i - 1]))
                anchor = cells[i - 1]
        out.append(list(cells[-1]))
        return out

    def twoPointTracing(self, smooth=True):
        """Dựng lại pathTrace[u][v] cho mọi cặp source có kết nối.

        smooth=True  (mặc định): rút gọn bằng line-of-sight → turning points.
        smooth=False            : giữ toàn bộ ô cell-by-cell.
        """
        self.buildSumBlock()
        n = self.npos
        self.pathTrace = [[[] for _ in range(n)] for _ in range(n)]
        for u in range(n):
            for v in range(n):
                if u == v:
                    continue
                if self.inters[u][v][0][0] == -1:
                    continue
                x, y = self.inters[u][v]
                p_u = self._backtrack_to_source(x, u)
                p_v = self._backtrack_to_source(y, v)
                cells = list(reversed(p_u)) + list(p_v)
                dedup = [cells[0]]
                for c in cells[1:]:
                    if c != dedup[-1]:
                        dedup.append(c)
                if smooth:
                    self.pathTrace[u][v] = self._smooth_path(dedup)
                else:
                    self.pathTrace[u][v] = [list(c) for c in dedup]
                if self.Solver_minimize:
                    # Cạnh = Total cost (7a) của đúng đoạn pathTrace mà getPath sẽ ghép
                    # ⇒ Σ cạnh dọc tour = Total cost toàn đường (TSP cost == Total cost).
                    total, _, _, _ = self.pathTotalCost(self.pathTrace[u][v])
                    self.adj[u][v] = total

        # Hòa giải đối xứng: _smooth_path phụ thuộc chiều nên pathTrace[u][v] và
        # pathTrace[v][u] (cùng adj) có thể lệch nhau. Chọn chiều rẻ hơn và đồng bộ
        # pathTrace để dijkstra/getPath nhất quán → TSP cost == Total cost tuyệt đối.
        if self.Solver_minimize:
            for u in range(n - 1):
                for v in range(u + 1, n):
                    if self.inters[u][v][0][0] == -1:
                        continue
                    if self.adj[u][v] > self.adj[v][u]:
                        self.adj[u][v] = self.adj[v][u]
                        self.pathTrace[u][v] = [list(c) for c in reversed(self.pathTrace[v][u])]
                    elif self.adj[u][v] < self.adj[v][u]:
                        self.adj[v][u] = self.adj[u][v]
                        self.pathTrace[v][u] = [list(c) for c in reversed(self.pathTrace[u][v])]

    # =========================================================
    # Dijkstra trên đồ thị checkpoint (pha 1 cuối cùng)
    # =========================================================
    def dijkstra(self):
        """All-pairs shortest path trên đồ thị checkpoint."""
        n = self.npos
        INF = float('inf')
        self.dijk = [[INF] * n for _ in range(n)]
        self.dtra = [[-1] * n for _ in range(n)]
        for root in range(n):
            self.dijk[root][root] = 0.0
            pq = [(0.0, root)]
            while pq:
                d, u = heapq.heappop(pq)
                if d > self.dijk[root][u] + 1e-12:
                    continue
                for v in range(n):
                    if v == u:
                        continue
                    w = self.adj[u][v]
                    if w == INF:
                        continue
                    nd = d + w
                    if nd + 1e-12 < self.dijk[root][v]:
                        self.dijk[root][v] = nd
                        self.dtra[root][v] = u
                        heapq.heappush(pq, (nd, v))

    # =========================================================
    # Pipeline tổng hợp
    # =========================================================
    def buildGraphAdvanced(self, w1=None, w2=None, w3=None, C1=None):
        """Pipeline pha 1 hoàn chỉnh.

        Các tham số truyền vào ghi đè cài đặt hiện tại.
        Khuyến nghị: gọi config() trước rồi gọi buildGraphAdvanced() không tham số.
        """
        if w1 is not None:
            self.w1 = float(w1)
        if w2 is not None:
            self.w2 = float(w2)
        if w3 is not None:
            self.w3 = float(w3)
        if C1 is not None:
            self.C1 = float(C1)
        self.DFType = (f"WP-FMF (w1={self.w1:.2f}, w2={self.w2:.2f}, "
                       f"w3={self.w3:.2f}, C1={self.C1:.2f})")

        self.computeSafety()
        self._normalize_radiation()
        self.computeFCost()
        self._firework_primary()
        self._firework_secondary()
        self.twoPointTracing()
        self.dijkstra()

        # Alias giữ tương thích với code visualization cũ
        self.owner = [self.F_label]
        self.dista = self.T

    def buildGraphNormal(self):
        self.buildGraphAdvanced()

    # =========================================================
    # getPath: mở rộng TSP permutation thành chuỗi ô
    # =========================================================
    def getPath(self, sol):
        """sol: permutation 0..npos-1. Trả về list [r, c] tour khép kín."""
        sol = list(sol) + [sol[0]]
        cells = []
        for i in range(len(sol) - 1):
            u, v = int(sol[i]), int(sol[i + 1])
            p = v
            t1 = []
            while p != u:
                k = self.dtra[u][p]
                if k == -1:
                    t1.append(list(self.deslist[p]))
                    t1.append(list(self.deslist[u]))
                    break
                seg = list(self.pathTrace[p][k])
                t1.extend(seg)
                p = k
            t1.reverse()
            cells.extend(t1)
        return cells

    # =========================================================
    # Metrics đường đi
    # =========================================================
    def pathLength(self, cells):
        """length(P) = Σ d(p_i, p_{i+1})  — công thức (3)."""
        total = 0.0
        for i in range(len(cells) - 1):
            total += math.hypot(cells[i + 1][0] - cells[i][0],
                                cells[i + 1][1] - cells[i][1])
        return total

    def _segment_avg(self, a, b, value_at):
        """Giá trị trung bình của value_at(·) trên một đoạn a→b.

        - supercover=True và a,b KHÔNG kề nhau (đoạn smooth/turning points)
          → trung bình trên mọi ô supercover line mà đoạn cắt qua (gồm 2 đầu mút).
        - ngược lại (cell-by-cell, hoặc supercover=False)
          → trung bình hai đầu mút (công thức gốc)."""
        adjacent = (abs(int(a[0]) - int(b[0])) <= 1
                    and abs(int(a[1]) - int(b[1])) <= 1)
        if self.supercover and not adjacent:
            scl = supercover_cells(a, b)
            return sum(value_at(p) for p in scl) / len(scl)
        return (value_at(a) + value_at(b)) / 2.0

    def pathRisk(self, cells):
        """risk(P) = Σ d(p_n,p_{n+1}) · avg(1−S) trên từng đoạn.

        avg = trung bình hai đầu mút (cell-by-cell, công thức 5) hoặc trung bình
        trên supercover line khi supercover=True và đoạn không kề nhau (công thức 6)."""
        if not hasattr(self, 'safety'):
            return None

        def risk_at(p):
            r, c = int(p[0]), int(p[1])
            if 0 <= r < self.mapSize and 0 <= c < self.mapSize:
                return 1.0 - self.safety[r][c]
            return 0.0

        total = 0.0
        for i in range(len(cells) - 1):
            a, b = cells[i], cells[i + 1]
            d = self.distant(a[0], a[1], b[0], b[1])
            total += d * self._segment_avg(a, b, risk_at)
        return total

    def pathRadiation(self, cells):
        """R(P) = Σ d(p_n,p_{n+1}) · avg(R̄) · (a/v) trên từng đoạn.

        avg = trung bình hai đầu mút (cell-by-cell, công thức 7) hoặc trung bình
        trên supercover line khi supercover=True và đoạn không kề nhau (công thức 8)."""
        if self.radiation_map is None:
            return None
        nrows = len(self.radiation_map)
        ncols = len(self.radiation_map[0]) if nrows > 0 else 0

        def rad_at(p):
            r, c = int(p[0]), int(p[1])
            if 0 <= r < nrows and 0 <= c < ncols:
                return self.radiation_map[r][c]
            return 0.0

        total = 0.0
        for i in range(len(cells) - 1):
            a, b = cells[i], cells[i + 1]
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            total += (d * self._segment_avg(a, b, rad_at)
                      * (self.a / self.v)) / 3600.0  # Chuyển μSv·m/s thành mSv·h
        return total

    def pathTotalCost(self, cells):
        """Total cost = w1·length(P) + w2·R(P) + w3·risk(P)  — công thức (7a).

        Trả về (total, length, radiation, risk).
        """
        L    = self.pathLength(cells)
        risk = self.pathRisk(cells)
        R    = self.pathRadiation(cells)
        risk = risk if risk is not None else 0.0
        R    = R    if R    is not None else 0.0
        total = self.w1 * L + self.w2 * R + self.w3 * risk
        return total, L, R, risk

    # =========================================================
    # Visualization
    # =========================================================
    def drawPath(self, points):
        sz = self.mapSize
        plt.figure(figsize=(8, 8), dpi=80)
        plt.axis([-1, sz, -sz, 1])
        plt.title(self.DFType, fontsize=14)
        mksz = self.mksz

        pts = [list(p) for p in points]
        for p in pts:
            p[0] *= -1
        if pts:
            ys, xs = zip(*pts)
            plt.plot(xs, ys, color='blue', linewidth=4)

        blx, bly = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    blx.append(j); bly.append(-i)
        plt.plot(blx, bly, 'ks', markersize=mksz)

        dx, dy = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 2:
                    dx.append(j); dy.append(-i)
        plt.plot(dx, dy, 's', color='red', markersize=mksz + 3)

        corner = [[-0.5, 0.5], [sz - 0.5, 0.5],
                  [sz - 0.5, -(sz - 0.5)], [-0.5, -(sz - 0.5)], [-0.5, 0.5]]
        cnx, cny = zip(*corner)
        plt.plot(cnx, cny, color='black')
        plt.xticks([]); plt.yticks([])
        plt.show()

    def drawFMComponent(self, rmv=None):
        rmv = rmv or []
        sz = self.mapSize
        plt.figure(figsize=(8, 8), dpi=80)
        plt.axis([-1, sz, -sz, 1])
        plt.title(self.DFType, fontsize=14)
        mksz = self.mksz

        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    continue
                lbl = self.F_label[i][j]
                if lbl == -1 or lbl in rmv:
                    continue
                col = self.colorHold[lbl % len(self.colorHold)]
                plt.plot(j, -i, 's', color=col, markersize=mksz)

        blx, bly = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    blx.append(j); bly.append(-i)
        plt.plot(blx, bly, 'ks', markersize=mksz)

        for i in range(self.npos - 1):
            for j in range(i + 1, self.npos):
                pts = self.pathTrace[i][j]
                if not pts:
                    continue
                pts = [list(p) for p in pts]
                for p in pts:
                    if p[0] > 0:
                        p[0] *= -1
                ys, xs = zip(*pts)
                plt.plot(xs, ys, color='crimson', linewidth=3)

        dx, dy = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 2:
                    dx.append(j); dy.append(-i)
        plt.plot(dx, dy, 's', color='red', markersize=mksz + 4)

        corner = [[-0.5, 0.5], [sz - 0.5, 0.5],
                  [sz - 0.5, -(sz - 0.5)], [-0.5, -(sz - 0.5)], [-0.5, 0.5]]
        cnx, cny = zip(*corner)
        plt.plot(cnx, cny, color='black')
        plt.xticks([]); plt.yticks([])
        plt.show()

    def drawDijkstraWave(self, rmv=None):
        rmv = rmv or []
        sz = self.mapSize
        plt.figure(figsize=(8, 8), dpi=80)
        plt.axis([-1, sz, -sz, 1])
        plt.title(self.DFType + " - Cumulative cost T(x)", fontsize=14)
        mksz = self.mksz

        blx, bly = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    blx.append(j); bly.append(-i)
        plt.plot(blx, bly, 'ks', markersize=mksz)

        x2, y2, z2 = [], [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 0 and self.F_label[i][j] != -1:
                    if self.F_label[i][j] in rmv:
                        continue
                    x2.append(j); y2.append(-i); z2.append(-self.T[i][j])
        if x2:
            plt.scatter(x2, y2, c=z2, cmap='jet', marker='s', s=mksz * mksz)

        dx, dy = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 2:
                    dx.append(j); dy.append(-i)
        plt.plot(dx, dy, 's', color='red', markersize=mksz)

        corner = [[-0.5, 0.5], [sz - 0.5, 0.5],
                  [sz - 0.5, -(sz - 0.5)], [-0.5, -(sz - 0.5)], [-0.5, 0.5]]
        cnx, cny = zip(*corner)
        plt.plot(cnx, cny, color='black')
        plt.xticks([]); plt.yticks([])
        plt.show()

    def drawSafety(self):
        """Trường chỉ số an toàn S(x)."""
        sz = self.mapSize
        if not hasattr(self, 'safety'):
            print("Chưa tính safety. Hãy gọi buildGraphAdvanced() trước.")
            return
        plt.figure(figsize=(8, 8), dpi=80)
        plt.axis([-1, sz, -sz, 1])
        plt.title("Safety field S(x)", fontsize=14)
        mksz = self.mksz

        x2, y2, z2 = [], [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] != 1:
                    x2.append(j); y2.append(-i); z2.append(self.safety[i][j])
        plt.scatter(x2, y2, c=z2, cmap='RdYlGn', marker='s',
                    s=mksz * mksz, vmin=0, vmax=1)
        plt.colorbar(label='S(x)')

        blx, bly = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    blx.append(j); bly.append(-i)
        plt.plot(blx, bly, 'ks', markersize=mksz)

        dx, dy = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 2:
                    dx.append(j); dy.append(-i)
        plt.plot(dx, dy, 's', color='white',
                 markeredgecolor='red', markersize=mksz, markeredgewidth=2)

        plt.xticks([]); plt.yticks([])
        plt.show()

    def drawFCost(self):
        """Trường chi phí cục bộ f(x)."""
        sz = self.mapSize
        if not hasattr(self, 'f_cost'):
            print("Chưa tính f_cost. Hãy gọi buildGraphAdvanced() trước.")
            return
        plt.figure(figsize=(8, 8), dpi=80)
        plt.axis([-1, sz, -sz, 1])
        plt.title(f"Local cost f(x)  [w1={self.w1:.2f}, w2={self.w2:.2f}, w3={self.w3:.2f}]",
                  fontsize=14)
        mksz = self.mksz

        x2, y2, z2 = [], [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] != 1:
                    fv = self.f_cost[i][j]
                    if fv < 1e9:
                        x2.append(j); y2.append(-i); z2.append(fv)
        if x2:
            plt.scatter(x2, y2, c=z2, cmap='viridis', marker='s', s=mksz * mksz)
            plt.colorbar(label='f(x)')

        blx, bly = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    blx.append(j); bly.append(-i)
        plt.plot(blx, bly, 'ks', markersize=mksz)

        dx, dy = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 2:
                    dx.append(j); dy.append(-i)
        plt.plot(dx, dy, 's', color='white',
                 markeredgecolor='red', markersize=mksz, markeredgewidth=2)

        plt.xticks([]); plt.yticks([])
        plt.show()

    def drawRadiation(self):
        """Heatmap bản đồ phóng xạ R̄(x)."""
        if self.radiation_map is None:
            print("Chưa nạp radiation_map.")
            return
        sz = self.mapSize
        nrows = len(self.radiation_map)
        ncols = len(self.radiation_map[0]) if nrows > 0 else 0
        plt.figure(figsize=(8, 8), dpi=80)
        plt.axis([-1, sz, -sz, 1])
        plt.title("Radiation map R̄(x)", fontsize=14)
        mksz = self.mksz

        x2, y2, z2 = [], [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] != 1 and i < nrows and j < ncols:
                    x2.append(j); y2.append(-i)
                    z2.append(self.radiation_map[i][j])
        if x2:
            plt.scatter(x2, y2, c=z2, cmap='hot_r', marker='s', s=mksz * mksz)
            plt.colorbar(label='Radiation dose rate')

        blx, bly = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    blx.append(j); bly.append(-i)
        plt.plot(blx, bly, 'ks', markersize=mksz)

        dx, dy = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 2:
                    dx.append(j); dy.append(-i)
        plt.plot(dx, dy, 's', color='cyan',
                 markeredgecolor='blue', markersize=mksz, markeredgewidth=2)

        plt.xticks([]); plt.yticks([])
        plt.show()
