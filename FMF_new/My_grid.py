"""
My_grid.py - WP-FMF (Weighted Potential Fast Marching Firework)
=================================================================
Triển khai thuật toán WP-FMF theo paper:
    "WP-FMF: Phương pháp Weighted Potential Fast Marching Firework
     cho quy hoạch đường đi đa đích đến của robot di động"

Các thay đổi cốt lõi so với FMF cổ điển:
- Chỉ số an toàn S(c) trên lân cận 5x5 (eq. 4 trong paper)
- Hàm chi phí cục bộ f(x) = w1 + (1-w1)(1-S(x)) (eq. 11a)
- Cập nhật Eikonal dạng sai phân: T(y) = T(x) + d(x,y)·f(y) (eq. 13a)
- Bridge cost dùng f trung bình: d(x,y)·(f(x)+f(y))/2 (eq. 14a)
- Pipeline 2 pha: Expanding (sơ cấp) + Intersecting (thứ cấp)
"""

import heapq
import math
import queue
import random
import numpy as np
import matplotlib.pyplot as plt

try:
    import pygame
except ImportError:
    pygame = None


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

        # Trọng số WP-FMF (có thể chỉnh qua setWeights)
        self.w1 = 0.7    # trade-off length vs safety
        self.C1 = 0.5    # trade-off N_obs vs d_min trong S(c)

        self.window_size = [mapSize * square_width + (mapSize + 1) * margin,
                            mapSize * square_height + (mapSize + 1) * margin]

        self.DFType = "WP-FMF"

    # =========================================================
    # Chỉnh trọng số
    # =========================================================
    def setWeights(self, w1=0.7, C1=0.5):
        """w1 -> 1: ưu tiên đường ngắn; w1 -> 0: ưu tiên đường an toàn.
           C1 -> 1: nhấn số vật cản; C1 -> 0: nhấn khoảng cách tới vật cản gần."""
        self.w1 = float(w1)
        self.C1 = float(C1)

    # =========================================================
    # I/O bản đồ
    # =========================================================
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
        """Kiểm tra "line of sight" giữa 2 ô (dùng prefix-sum, giữ nguyên từ bản gốc)."""
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
    # WP-FMF bước 1: tính S(c) và f(x)
    # =========================================================
    def computeSafety(self):
        """Công thức (4) trong paper:
            S(c) = C1 · (24 - N_obs)/24 + (1-C1) · d_min/3
        với N_obs, d_min tính trên vùng lân cận 5x5 (không kể chính c).
        """
        sz = self.mapSize
        self.safety = [[0.0] * sz for _ in range(sz)]
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    # Vật cản: an toàn = 0 (không dùng ô này để đi qua)
                    self.safety[i][j] = 0.0
                    continue
                n_obs = 0
                d_min = 5.0  # mặc định khi không có vật cản trong 11x11
                for di in range(-5, 6):
                    for dj in range(-5, 6):
                        if di == 0 and dj == 0:
                            continue
                        ni, nj = i + di, j + dj
                        if 0 <= ni < sz and 0 <= nj < sz:
                            if self.gridMap[ni][nj] == 1:
                                n_obs += 1
                                d = math.hypot(di, dj)
                                if d < d_min:
                                    d_min = d
                self.safety[i][j] = (self.C1 * (120.0 - n_obs) / 120.0
                                    + (1.0 - self.C1) * d_min / 5.0)

    def computeFCost(self):
        """Công thức (11a): f(x) = w1 + (1 - w1) * (1 - S(x))."""
        sz = self.mapSize
        INF = float('inf')
        self.f_cost = [[INF] * sz for _ in range(sz)]
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    self.f_cost[i][j] = INF
                else:
                    self.f_cost[i][j] = self.w1 + (1.0 - self.w1) * (1.0 - self.safety[i][j])

    # =========================================================
    # Algorithm 2: AddEdge (đã cải tiến theo bridge cost)
    # =========================================================
    def _add_edge_wp(self, x, y, stage):
        """
        x, y là 2 ô thuộc 2 vùng pháo hoa khác nhau.
            cost_bridge = d(x,y) * (f(x) + f(y)) / 2
            new_D       = T[x] + T[y] + cost_bridge
        Nếu new_D < D[u,v] hiện tại thì cập nhật. M[u,v] lưu stage (1 hoặc 2).
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

        # State của pháo hoa sơ cấp
        self.T = [[INF] * sz for _ in range(sz)]
        self.F_label = [[-1] * sz for _ in range(sz)]       # F(x) trong paper
        self.trace_p = [[(-1, -1)] * sz for _ in range(sz)]  # parent sơ cấp

        # Ma trận D' (adjacency giữa các source)
        self.adj = [[INF] * n for _ in range(n)]
        self.inters = [[((-1, -1), (-1, -1))] * n for _ in range(n)]
        self.M = [[0] * n for _ in range(n)]

        # Khởi tạo: mỗi source đẩy vào heap với T = 0
        pq = []
        for i, pos in enumerate(self.deslist):
            r, c = pos
            self.T[r][c] = 0.0
            self.F_label[r][c] = i
            heapq.heappush(pq, (0.0, r, c))

        # Weighted Dijkstra đa nguồn (8-connected)
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

        # Phân hoạch Hold[i]
        self.hold = [[] for _ in range(n)]
        for i in range(sz):
            for j in range(sz):
                if self.F_label[i][j] != -1:
                    self.hold[self.F_label[i][j]].append((i, j))

        # Kết nối sơ cấp: cặp ô kề nhau thuộc 2 vùng khác nhau
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
        """
        Với mỗi source i:
          1. Tạm xoá Hold[i] (T = INF, F = -1)
          2. Lan truyền từ các ô biên (thuộc vùng khác) vào vùng bị xoá
          3. Khi hai sóng từ 2 source khác gặp nhau trong Hold[i] -> AddEdge stage 2
          4. Khôi phục Hold[i]
        """
        sz = self.mapSize
        INF = float('inf')

        # trace_s lưu parent cho sóng thứ cấp (chỉ set cho ô bị re-label)
        self.trace_s = [[(-1, -1)] * sz for _ in range(sz)]

        for i in range(self.npos):
            hold_cells = list(self.hold[i])
            if not hold_cells:
                continue

            # Sao lưu và tạm xoá Hold[i]
            saved = {}
            hold_set = set(hold_cells)
            for (r, c) in hold_cells:
                saved[(r, c)] = (self.T[r][c], self.F_label[r][c])
                self.T[r][c] = INF
                self.F_label[r][c] = -1

            # Khởi tạo Border: các ô lân cận Hold[i] nhưng thuộc vùng khác
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

            # Lan truyền: chỉ đi vào các ô đang bị xoá (thuộc Hold[i])
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
                    # Chỉ đi vào vùng tạm bị xoá
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

            # Kết nối thứ cấp: trong Hold[i], cặp ô kề mà secondary label khác nhau
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
                        # f() và T() hiện tại đều là secondary → bridge cost đúng
                        self._add_edge_wp((r, c), (nr, nc), stage=2)

            # Khôi phục Hold[i] về trạng thái sơ cấp
            for (r, c), (t0, f0) in saved.items():
                self.T[r][c] = t0
                self.F_label[r][c] = f0

    # =========================================================
    # Dựng lại path ô-by-ô giữa mọi cặp source
    # =========================================================
    def _backtrack_to_source(self, start_cell, target_source):
        """
        Đi từ start_cell về source target_source dựa trên trace.
        - Khi ở vùng primary == target_source: dùng trace_p (đi về nguồn gốc)
        - Khi ở vùng primary khác: dùng trace_s trước (sóng thứ cấp đi ra khỏi Hold),
          nếu không có trace_s thì dùng trace_p.
        """
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
        return path  # start → ... → source

    def _smooth_path(self, cells):
        """Làm mịn đường đi bằng line-of-sight check (dùng prefix-sum)."""
        if len(cells) <= 2:
            return [list(c) for c in cells]
        out = [list(cells[0])]
        anchor = cells[0]
        for i in range(1, len(cells)):
            if self.connectable(anchor[0], anchor[1], cells[i][0], cells[i][1]) != 0:
                # Line-of-sight bị phá: commit ô trước đó
                out.append(list(cells[i - 1]))
                anchor = cells[i - 1]
        out.append(list(cells[-1]))
        return out

    def twoPointTracing(self):
        """Dựng lại pathTrace[u][v] cho mọi cặp source có kết nối."""
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
                p_u = self._backtrack_to_source(x, u)   # x → ... → s_u
                p_v = self._backtrack_to_source(y, v)   # y → ... → s_v
                # Ghép: s_u → ... → x → y → ... → s_v
                cells = list(reversed(p_u)) + list(p_v)
                # Khử cell trùng liên tiếp
                dedup = [cells[0]]
                for c in cells[1:]:
                    if c != dedup[-1]:
                        dedup.append(c)
                self.pathTrace[u][v] = self._smooth_path(dedup)

    # =========================================================
    # Dijkstra trên đồ thị checkpoint (pha 1 cuối cùng)
    # =========================================================
    def dijkstra(self):
        """All-pairs shortest path trên đồ thị checkpoint (kích thước npos)."""
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
    # Pipeline
    # =========================================================
    def buildGraphAdvanced(self, w1=None, C1=None):
        """Pipeline pha 1 hoàn chỉnh theo paper WP-FMF."""
        if w1 is not None:
            self.w1 = float(w1)
        if C1 is not None:
            self.C1 = float(C1)
        self.DFType = f"WP-FMF (w1={self.w1}, C1={self.C1})"

        self.computeSafety()
        self.computeFCost()
        self._firework_primary()
        self._firework_secondary()
        self.twoPointTracing()
        self.dijkstra()

        # Alias giữ tương thích với code visualization cũ
        self.owner = [self.F_label]   # drawFMComponent dùng owner[0]
        self.dista = self.T           # drawDijkstraWave dùng dista

    # Giữ tên cũ cho khả năng tương thích
    def buildGraphNormal(self):
        self.buildGraphAdvanced()

    # =========================================================
    # getPath: mở rộng TSP permutation thành chuỗi ô
    # =========================================================
    def getPath(self, sol):
        """
        sol: permutation của 0..npos-1 (TSP solution, không kèm depot cuối).
        Trả về list các [r, c] mà robot đi qua trên lưới (tour khép kín).
        """
        sol = list(sol) + [sol[0]]
        cells = []
        for i in range(len(sol) - 1):
            u, v = int(sol[i]), int(sol[i + 1])
            p = v
            t1 = []
            # Lần ngược từ v về u qua dtra
            while p != u:
                k = self.dtra[u][p]
                if k == -1:
                    # Không có path -> fallback đường thẳng
                    t1.append(list(self.deslist[p]))
                    t1.append(list(self.deslist[u]))
                    break
                seg = list(self.pathTrace[p][k])
                t1.extend(seg)
                p = k
            t1.reverse()  # u → ... → v
            cells.extend(t1)
        return cells

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

        # Vật cản
        blx, bly = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    blx.append(j); bly.append(-i)
        plt.plot(blx, bly, 'ks', markersize=mksz)

        # Checkpoint
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

        # Tô vùng Voronoi-like
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    continue
                lbl = self.F_label[i][j]
                if lbl == -1 or lbl in rmv:
                    continue
                col = self.colorHold[lbl % len(self.colorHold)]
                plt.plot(j, -i, 's', color=col, markersize=mksz)

        # Vật cản
        blx, bly = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    blx.append(j); bly.append(-i)
        plt.plot(blx, bly, 'ks', markersize=mksz)

        # Đường nối giữa các source (crimson)
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

        # Checkpoint
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

        # Vật cản
        blx, bly = [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 1:
                    blx.append(j); bly.append(-i)
        plt.plot(blx, bly, 'ks', markersize=mksz)

        # Heatmap T
        x2, y2, z2 = [], [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] == 0 and self.F_label[i][j] != -1:
                    if self.F_label[i][j] in rmv:
                        continue
                    x2.append(j); y2.append(-i); z2.append(-self.T[i][j])
        if x2:
            plt.scatter(x2, y2, c=z2, cmap='jet', marker='s', s=mksz * mksz)

        # Checkpoint
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
        """Trường chỉ số an toàn S(x) - mới trong WP-FMF."""
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
        """Trường chi phí cục bộ f(x) - mới trong WP-FMF."""
        sz = self.mapSize
        if not hasattr(self, 'f_cost'):
            print("Chưa tính f_cost. Hãy gọi buildGraphAdvanced() trước.")
            return
        plt.figure(figsize=(8, 8), dpi=80)
        plt.axis([-1, sz, -sz, 1])
        plt.title(f"Local cost f(x), w1={self.w1}", fontsize=14)
        mksz = self.mksz

        x2, y2, z2 = [], [], []
        for i in range(sz):
            for j in range(sz):
                if self.gridMap[i][j] != 1:
                    x2.append(j); y2.append(-i); z2.append(self.f_cost[i][j])
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

    # =========================================================
    # Tính risk thực của một đường đi (để đánh giá)
    # =========================================================
    def pathRisk(self, cells):
        """risk(P) = sum over p in P of (1 - S(p))  - công thức (5)."""
        if not hasattr(self, 'safety'):
            return None
        total = 0.0
        for p in cells:
            r, c = int(p[0]), int(p[1])
            if 0 <= r < self.mapSize and 0 <= c < self.mapSize:
                total += (1.0 - self.safety[r][c])
        return total

    def pathLength(self, cells):
        """length(P) = sum of Euclidean distance giữa các waypoint liên tiếp."""
        total = 0.0
        for i in range(len(cells) - 1):
            total += math.hypot(cells[i + 1][0] - cells[i][0],
                                cells[i + 1][1] - cells[i][1])
        return total
