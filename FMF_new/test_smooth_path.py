# -*- coding: utf-8 -*-
"""
test_smooth_path.py — Minh hoạ quá trình làm mượt đường đi (line-of-sight string-pulling)
=========================================================================================
Tái hiện ĐÚNG thuật toán làm mượt đang dùng trong My_grid.py:
    buildSumBlock  -> tổng tiền tố 2D đếm vật cản
    countBlock     -> đếm vật cản trong hình chữ nhật (O(1))
    connectable    -> xấp xỉ kiểm tra "tầm nhìn thẳng" theo dải chéo
    smooth_path    -> rút gọn tham lam (giữ điểm ngoặt / turning points)

Đầu vào:
    - obstacle_grid : ma trận vật cản (1 = vật cản, 0 = ô tự do)
    - cell_path     : đường đi cell-by-cell, danh sách [row, col] các ô kề nhau

Đầu ra:
    - Lưu ảnh so sánh path GỐC (cell-by-cell) và path ĐÃ LÀM MƯỢT (turning points).

Chạy:
    python test_smooth_path.py
"""
import sys
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass
import os
import math
import heapq

import numpy as np
import matplotlib
matplotlib.use("Agg")  # backend không cần màn hình, chỉ lưu file
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


# =========================================================
# Phần lõi: sao chép trung thực thuật toán từ My_grid.py
# =========================================================
class PathSmoother:
    """Đóng gói thuật toán làm mượt theo tầm nhìn thẳng, dựa trên tổng tiền tố 2D."""

    def __init__(self, obstacle_grid):
        self.gridMap = [list(row) for row in obstacle_grid]
        self.mapSize = len(self.gridMap)
        self.buildSumBlock()

    # ---- Tổng tiền tố 2D để đếm nhanh số vật cản trong hình chữ nhật ----
    def buildSumBlock(self):
        sz = self.mapSize
        self.sumBlock = [[0] * (sz + 1) for _ in range(sz + 1)]
        for i in range(sz):
            for j in range(sz):
                self.sumBlock[i + 1][j + 1] = (self.sumBlock[i + 1][j]
                                               + self.sumBlock[i][j + 1]
                                               - self.sumBlock[i][j])
                if self.gridMap[i][j] == 1:
                    self.sumBlock[i + 1][j + 1] += 1

    # ---- Số vật cản trong hình chữ nhật [x1..x2] x [y1..y2] (bao hàm-loại trừ) ----
    def countBlock(self, x1, y1, x2, y2):
        x1 += 1; y1 += 1; x2 += 1; y2 += 1
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1
        return (self.sumBlock[x2][y2] - self.sumBlock[x1 - 1][y2]
                - self.sumBlock[x2][y1 - 1] + self.sumBlock[x1 - 1][y1 - 1])

    # ---- Xấp xỉ "line of sight": trả về số vật cản trên dải chéo (0 = nhìn thẳng được) ----
    def connectable(self, x1, y1, x2, y2):
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

    # ---- Kiểm tra tầm nhìn CHÍNH XÁC bằng đường supercover (Bresenham, 1965) ----
    def connectable_bresenham(self, x1, y1, x2, y2):
        """Trả về số ô vật cản mà đoạn thẳng nối tâm ô (x1,y1)->(x2,y2) đi qua
        (0 => nhìn thẳng được).

        Khác connectable() (xấp xỉ "dải chéo" bằng tổng tiền tố): hàm này duyệt ĐÚNG
        TẤT CẢ các ô mà đoạn thẳng chạm vào (đường *supercover* — biến thể đầy đủ của
        vạch đường Bresenham 1965), nên không bỏ sót ô bị đường thẳng quệt qua góc.
        Đây là phép tầm nhìn chính xác trên mô hình tâm-ô, tương ứng tinh thần
        LineOfSight trong Theta* (Daniel et al., 2010). Chỉ dùng phép toán số nguyên.

        Khi đoạn đi đúng qua một điểm góc lưới (bước chéo), nếu CẢ HAI ô kề bên tại góc
        đó đều là vật cản thì coi như bị chặn (không cho "lách khe" giữa hai vật cản
        chạm góc nhau).
        """
        sz = self.mapSize

        def is_block(r, c):
            return 0 <= r < sz and 0 <= c < sz and self.gridMap[r][c] == 1

        x0, y0 = int(x1), int(y1)
        xe, ye = int(x2), int(y2)
        nx = abs(xe - x0)
        ny = abs(ye - y0)
        sx = 1 if xe >= x0 else -1
        sy = 1 if ye >= y0 else -1

        count = 1 if is_block(x0, y0) else 0  # ô xuất phát
        x, y = x0, y0
        ix = iy = 0
        while ix < nx or iy < ny:
            # so sánh (1+2ix)*ny  vs  (1+2iy)*nx  — toàn số nguyên
            decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
            if decision == 0:          # đi đúng qua góc lưới -> bước chéo
                if is_block(x + sx, y) and is_block(x, y + sy):
                    count += 1         # bị chặn ở khe chéo
                x += sx; y += sy
                ix += 1; iy += 1
            elif decision < 0:         # bước ngang
                x += sx; ix += 1
            else:                      # bước dọc
                y += sy; iy += 1
            if is_block(x, y):
                count += 1
        return count

    # ---- Làm mượt tham lam (string-pulling): giữ các điểm ngoặt ----
    def smooth_path(self, cells, method="band"):
        """method="band"      -> dùng connectable() (xấp xỉ dải chéo, O(1)/truy vấn).
           method="bresenham"  -> dùng connectable_bresenham() (LOS chính xác kiểu Theta*).
        """
        los = self.connectable_bresenham if method == "bresenham" else self.connectable
        if len(cells) <= 2:
            return [list(c) for c in cells]
        out = [list(cells[0])]
        anchor = cells[0]
        for i in range(1, len(cells)):
            if los(anchor[0], anchor[1], cells[i][0], cells[i][1]) != 0:
                out.append(list(cells[i - 1]))
                anchor = cells[i - 1]
        out.append(list(cells[-1]))
        return out


# =========================================================
# Tiện ích sinh path cell-by-cell mẫu (A* 8-hướng)
# =========================================================
def astar_8(grid, start, goal):
    """A* trên lưới 8-liên thông; trả về đường đi cell-by-cell [row, col]."""
    sz = len(grid)
    DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)]

    def h(a, b):  # heuristic octile
        dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
        return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)

    start, goal = tuple(start), tuple(goal)
    openpq = [(h(start, goal), 0.0, start)]
    came = {start: None}
    gscore = {start: 0.0}
    while openpq:
        _, g, cur = heapq.heappop(openpq)
        if cur == goal:
            path = []
            while cur is not None:
                path.append([cur[0], cur[1]])
                cur = came[cur]
            return list(reversed(path))
        for dr, dc in DIRS:
            nr, nc = cur[0] + dr, cur[1] + dc
            if nr < 0 or nr >= sz or nc < 0 or nc >= sz:
                continue
            if grid[nr][nc] == 1:
                continue
            # chặn cắt chéo qua khe vật cản
            if dr != 0 and dc != 0 and (grid[cur[0]][nc] == 1 and grid[nr][cur[1]] == 1):
                continue
            step = math.hypot(dr, dc)
            ng = g + step
            nxt = (nr, nc)
            if ng < gscore.get(nxt, float("inf")):
                gscore[nxt] = ng
                came[nxt] = cur
                heapq.heappush(openpq, (ng + h(nxt, goal), ng, nxt))
    return None  # không có đường


def path_length(cells):
    return sum(math.hypot(cells[i + 1][0] - cells[i][0],
                          cells[i + 1][1] - cells[i][1])
               for i in range(len(cells) - 1))


# =========================================================
# Vẽ và lưu ảnh so sánh
# =========================================================
def plot_compare(grid, raw_path, smooth_path, out_png,
                 smooth_label="Đường đi sau khi làm mượt (turning points)",
                 smooth_color="#D7263D"):
    grid = np.array(grid)
    fig, ax = plt.subplots(figsize=(8, 8))

    # Vật cản: đen, ô tự do: trắng. imshow dùng (row, col) -> trục x = col, y = row.
    ax.imshow(grid, cmap="Greys", origin="upper", vmin=0, vmax=1,
              extent=[-0.5, grid.shape[1] - 0.5, grid.shape[0] - 0.5, -0.5])

    # Lưới mảnh cho dễ đọc
    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
    ax.grid(which="minor", color="0.85", linewidth=0.5)
    ax.tick_params(which="minor", length=0)

    # Path gốc (cell-by-cell): xanh nhạt, chấm nhỏ ở mỗi ô
    rx = [c[1] for c in raw_path]   # col -> x
    ry = [c[0] for c in raw_path]   # row -> y
    ax.plot(rx, ry, "-o", color="#4C9BE8", markersize=3, linewidth=1.2,
            alpha=0.8, label=f"Đường đi gốc (cell-by-cell)")

    # Path đã làm mượt: đỏ đậm, marker vuông ở các điểm ngoặt
    sx = [c[1] for c in smooth_path]
    sy = [c[0] for c in smooth_path]
    ax.plot(sx, sy, "-", color=smooth_color, linewidth=2.4,
            label=f"{smooth_label}")
    ax.plot(sx, sy, "s", color=smooth_color, markersize=7,
            markerfacecolor="white", markeredgewidth=1.8)

    # Điểm đầu / cuối
    ax.plot(rx[0], ry[0], "P", color="green", markersize=14, label="Start")
    ax.plot(rx[-1], ry[-1], "X", color="black", markersize=14, label="Goal")

    raw_len = path_length(raw_path)
    sm_len = path_length(smooth_path)
    # ax.set_title("Làm mượt đường đi theo tầm nhìn thẳng (line-of-sight string-pulling)\n"
    #              f"Chiều dài: gốc = {raw_len:.2f}  →  mượt = {sm_len:.2f}  "
    #              f"(giảm {100 * (raw_len - sm_len) / raw_len:.1f}%)",
    #              fontsize=11)
    # ax.set_xlabel("cột (col)")
    # ax.set_ylabel("hàng (row)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08),
              ncol=2, fontsize=9, frameon=False)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Đã lưu ảnh -> {out_png}")


def plot_los_compare(grid, raw_path, smooth_band, smooth_bres, out_png):
    """So sánh hai phép kiểm tra tầm nhìn: band (dải chéo, xấp xỉ) vs Bresenham (chính xác)."""
    grid = np.array(grid)
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.imshow(grid, cmap="Greys", origin="upper", vmin=0, vmax=1,
              extent=[-0.5, grid.shape[1] - 0.5, grid.shape[0] - 0.5, -0.5])
    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
    ax.grid(which="minor", color="0.85", linewidth=0.5)
    ax.tick_params(which="minor", length=0)

    # Path gốc mờ làm nền
    rx = [c[1] for c in raw_path]
    ry = [c[0] for c in raw_path]
    ax.plot(rx, ry, "-o", color="#9bbfe0", markersize=2, linewidth=1.0,
            alpha=0.6, label=f"Path gốc (cell-by-cell): {len(raw_path)} ô")

    # band: cam, nét đứt
    bx = [c[1] for c in smooth_band]
    by = [c[0] for c in smooth_band]
    ax.plot(bx, by, "--s", color="#F18F01", linewidth=2.2, markersize=8,
            markerfacecolor="white", markeredgewidth=1.6,
            label=f"connectable (dải chéo, xấp xỉ): {len(smooth_band)} điểm")

    # bresenham: đỏ, nét liền
    ex = [c[1] for c in smooth_bres]
    ey = [c[0] for c in smooth_bres]
    ax.plot(ex, ey, "-o", color="#D7263D", linewidth=2.2, markersize=5,
            label=f"connectable_bresenham (chính xác): {len(smooth_bres)} điểm")

    ax.plot(rx[0], ry[0], "P", color="green", markersize=14, label="Start")
    ax.plot(rx[-1], ry[-1], "X", color="black", markersize=14, label="Goal")

    lb = path_length(smooth_band)
    le = path_length(smooth_bres)
    ax.set_title("So sánh phép kiểm tra tầm nhìn khi làm mượt\n"
                 f"band (dải chéo) dài {lb:.2f} / {len(smooth_band)} điểm   vs   "
                 f"Bresenham dài {le:.2f} / {len(smooth_bres)} điểm",
                 fontsize=11)
    ax.set_xlabel("cột (col)")
    ax.set_ylabel("hàng (row)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08),
              ncol=2, fontsize=9, frameon=False)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Đã lưu ảnh -> {out_png}")


# =========================================================
# Demo
# =========================================================
def make_demo_grid(sz=24):
    """Tạo một bản đồ vật cản mẫu để A* phải đi vòng -> làm mượt thấy rõ tác dụng."""
    grid = [[0] * sz for _ in range(sz)]

    def block(r0, r1, c0, c1):
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                if 0 <= r < sz and 0 <= c < sz:
                    grid[r][c] = 1

    block(4, 14, 6, 8)     # tường dọc 1
    block(10, 12, 8, 16)   # tường ngang
    block(16, 20, 12, 14)  # tường dọc 2
    block(2, 6, 14, 18)    # khối góc trên phải
    return grid


def main():
    # ---- Đầu vào: ma trận vật cản + path cell-by-cell ----
    # Bạn có thể THAY phần này bằng obstacle_grid và cell_path của riêng mình.
    # grid = make_demo_grid(sz=24)
    grid = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    # start, goal = [22, 1], [1, 22]

    # raw_path = astar_8(grid, start, goal)

    raw_path = [[22, 1], [21, 2], [20, 3], [19, 4], [18, 5], [17, 6], [16, 7], [15, 8], [14, 9], [13, 10], [13, 11], 
                [13, 12], [13, 13], [13, 14], [13, 15], [13, 16], [12, 17], [11, 17], [10, 17], [9, 17], [8, 17], 
                [7, 18], [6, 19], [5, 19], [4, 19], [3, 20], [2, 21], [1, 22]]
    
    if raw_path is None:
        raise RuntimeError("Không tìm được đường đi cell-by-cell giữa start và goal.")

    # ---- Làm mượt (đúng thuật toán trong My_grid.py) ----
    smoother = PathSmoother(grid)
    smooth_band = smoother.smooth_path(raw_path, method="band")        # xấp xỉ dải chéo
    smooth_bres = smoother.smooth_path(raw_path, method="bresenham")   # LOS chính xác (Bresenham)

    print(f"Path gốc                  : {len(raw_path)} ô")
    print(f"Làm mượt (band/dải chéo)  : {len(smooth_band)} điểm ngoặt -> "
          f"{[tuple(p) for p in smooth_band]}")
    print(f"Làm mượt (Bresenham chính xác): {len(smooth_bres)} điểm ngoặt -> "
          f"{[tuple(p) for p in smooth_bres]}")
    same = [tuple(p) for p in smooth_band] == [tuple(p) for p in smooth_bres]
    print("Hai phương pháp cho kết quả " + ("GIỐNG nhau." if same else "KHÁC nhau."))

    # ---- Lưu ảnh ----
    out_dir = os.path.dirname(os.path.abspath(__file__))
    # (1) ảnh gốc: path cell-by-cell vs path mượt (dùng band như mặc định trong code)
    plot_compare(grid, raw_path, smooth_band,
                 os.path.join(out_dir, "smooth_path_demo.png"))
    # (2) ảnh so sánh hai phép kiểm tra tầm nhìn: band vs Bresenham
    plot_los_compare(grid, raw_path, smooth_band, smooth_bres,
                     os.path.join(out_dir, "smooth_los_compare.png"))
    # (3) ảnh so sánh: path cell-by-cell vs path mượt dùng Bresenham (LOS chính xác)
    plot_compare(grid, raw_path, smooth_bres,
                 os.path.join(out_dir, "smooth_path_bresenham_demo.png"),
                 smooth_label="Đường đi sau khi làm mượt")


if __name__ == "__main__":
    main()
