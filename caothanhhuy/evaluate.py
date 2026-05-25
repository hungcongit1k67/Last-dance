# -*- coding: utf-8 -*-
"""
evaluate.py ─ Đánh giá các tiêu chí thuật toán lập lộ trình đa mục tiêu
========================================================================
Chạy headless (không cần cửa sổ Pygame), so sánh 3 bộ giải TSP:
  • Christofides     — xấp xỉ ≤ 1.5× tối ưu, nhanh
  • ACO              — Ant Colony Optimization, heuristic
  • FFT Backtracking — chính xác, chỉ dùng khi n ≤ 12

Tiêu chí đánh giá (khớp với FMF_new):
  path_length  — tổng Euclidean distance dọc đường đi
  safety_mean  — an toàn trung bình (cao hơn = tốt hơn)
  safety_min   — an toàn thấp nhất (điểm nguy hiểm nhất trên đường)
  risk         — rủi ro va chạm tổng hợp  Σ max(0, 1 − S(p)/S_avg)
  risk_mean    — rủi ro trung bình mỗi bước
  total_cost   — w1·path_length + w2·risk
  F_score      — hàm mục tiêu gốc của dự án
  time_*_sec   — thời gian từng pha và tổng

Kết quả lưu vào:  results/<map_name>_eval.json

Cách chạy:
  python evaluate.py
  python evaluate.py --map map_NVIDIA/map_01.txt --w1 0.4 --w2 0.6
  python evaluate.py --map map_health_care/HC_01.txt --T_max 2000
"""

import os, sys, math, time, json, datetime
from queue import PriorityQueue
import numpy as np

# Fix encoding UTF-8 cho Windows console
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ──────────────────────────────────────────────────────────────────
# 1. Khởi động Pygame ở chế độ headless TRƯỚC mọi import khác
# ──────────────────────────────────────────────────────────────────
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import pygame
pygame.init()
pygame.display.set_mode((1, 1))   # cửa sổ ẩn — bắt buộc để Sprite/Event hoạt động

# Thêm thư mục caothanhhuy vào sys.path để import các module nội bộ
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from Map_Grid import (
    make_grid, read_map_from_file2, cal_perimeter,
    sort_ds, fix_path_map, delta, get_neighbor,
)
from FFT_TSP import (
    calculate_safety, min_safety, mean_avg,
    reconstruct_path, check_path_valid, efficient_branching,
)
from TSP_cristofides_2 import CRIST
from ACO_TSP import ACO, Graph


# ──────────────────────────────────────────────────────────────────
# 2. Cấu hình — chỉnh tại đây hoặc truyền qua CLI
# ──────────────────────────────────────────────────────────────────
CONFIG = {
    # Trọng số hàm mục tiêu (w1 + w2 = 1.0)
    "w1": 0.5,          # Độ dài đường đi
    "w2": 0.5,          # Rủi ro va chạm

    # Tham số FMM đa nguồn
    "T_max":    1500,   # Ngưỡng dừng tối đa
    "T_step":    200,   # Bước mở rộng khi chưa đủ cạnh
    "a_dgree":  0.65,   # Cân bằng an toàn vs khoảng cách (0 < a < 1)

    # Số lần lặp (dùng cho tương lai khi chạy ntest > 1)
    "ntest": 1,

    # ACO
    "aco_ants":      50,
    "aco_gen":      200,
    "aco_alpha":    1.0,
    "aco_beta":    20.0,
    "aco_rho":      0.5,
    "aco_q":         10,
    "aco_strategy":   2,   # 0=ant-cycle  1=ant-quality  2=ant-density

    # Đường dẫn bản đồ mặc định
    "map_path": os.path.join(_HERE, "map_random", "map_factory_200.txt"),
}


# ──────────────────────────────────────────────────────────────────
# 3. safe_zone headless — giống Map_Grid.safe_zone nhưng không cần
#    pygame.event (event loop chạy headless không sinh event)
# ──────────────────────────────────────────────────────────────────
def _safe_zone_headless(grid, row_max):
    """
    Tính khoảng cách FMM từ mỗi ô đến vật cản gần nhất (headless).
    Trả về safe_medium — trung bình giá trị safe của toàn bản đồ.
    """
    count = 0
    open_set = PriorityQueue()
    open_set_hash = set()

    for row in grid:
        for spot in row:
            if spot.is_barrier():
                spot.safe = 0
                open_set.put((0, count, spot))
                open_set_hash.add(spot)
                count += 1

    while not open_set.empty():
        current = open_set.get()[2]
        for neighbor in current.update_neighbors(grid)[:4]:
            if not neighbor.is_closed_safe():
                if current.safe < neighbor.safe and neighbor not in open_set_hash:
                    r, c = neighbor.get_row(), neighbor.get_col()
                    if r + 1 < row_max and r - 1 > 0:
                        dx = min(grid[r+1][c].safe, grid[r-1][c].safe)
                    elif r + 1 == row_max:
                        dx = grid[r-1][c].safe
                    else:
                        dx = grid[r+1][c].safe

                    if c + 1 < row_max and c - 1 > 0:
                        dy = min(grid[r][c+1].safe, grid[r][c-1].safe)
                    elif c + 1 == row_max:
                        dy = grid[r][c-1].safe
                    else:
                        dy = grid[r][c+1].safe

                    disc = 2.0 - (dx - dy) ** 2
                    neighbor.safe = (
                        (dx + dy + math.sqrt(disc)) / 2 if disc >= 0
                        else min(dx, dy) + 1
                    )
                    count += 1
                    open_set_hash.add(neighbor)
                    open_set.put((neighbor.safe, count, neighbor))
                    neighbor.make_open_safe()

            if not current.is_barrier():
                current.make_closed_safe(current.safe)

    total, cnt = 0.0, 0
    for row in grid:
        for spot in row:
            if not spot.is_barrier() and spot.safe >= 1:
                total += spot.safe
                cnt += 1
    return round(total / cnt, 4) if cnt else 0.0


# ──────────────────────────────────────────────────────────────────
# 4. Hàm nội bộ reconstruct_path_safe (từ multi_object.py)
# ──────────────────────────────────────────────────────────────────
def _reconstruct_path_safe(lst, grid, g_score, min_safe=1.71):
    path = [lst[0]]
    for i in range(1, len(lst) - 1):
        current = lst[i]
        for neighbor in get_neighbor(lst[i - 1], grid, 200):
            if neighbor in get_neighbor(lst[i + 1], grid, 200):
                if (current.safe < min_safe
                        and neighbor.safe > current.safe
                        and g_score[neighbor] <= g_score[current] + 0.5):
                    current = neighbor
                    lst[i] = neighbor
        path.append(current)
    return path


# ──────────────────────────────────────────────────────────────────
# 5. FMM đa nguồn headless (từ multi_object.algorithm, bỏ draw/event)
# ──────────────────────────────────────────────────────────────────
def _algorithm_headless(grid, lst_start, T_step, T_max, a_dgree):
    """
    FMM đa nguồn — xây dựng ma trận đường đi và chi phí giữa
    tất cả cặp checkpoint.
    Trả về [path_matrix, path_value_matrix, edges].
    """
    rows = len(grid)
    n    = len(lst_start)

    lst_check        = [0] * n
    lst_count        = [0] * n
    lst_open_set     = [PriorityQueue()                                          for _ in range(n)]
    lst_came_from    = [{}                                                        for _ in range(n)]
    lst_g_score      = [{sp: float("inf") for row in grid for sp in row}         for _ in range(n)]
    lst_f_score      = [{sp: float("inf") for row in grid for sp in row}         for _ in range(n)]
    lst_open_set_hash = [set()                                                   for _ in range(n)]

    path        = [[[] for _ in range(n)] for _ in range(n)]
    path_value  = [[0]  * n               for _ in range(n)]
    check_path  = [[0]  * n               for _ in range(n)]
    max_count_path = 0
    edges = []

    for i in range(n):
        lst_open_set[i].put((0, lst_count[i], lst_start[i]))
        lst_g_score[i][lst_start[i]] = 0
        lst_f_score[i][lst_start[i]] = 0
        lst_open_set_hash[i].add(lst_start[i])

    while True:
        if any(lst_open_set[i].empty() for i in range(n)):
            break

        lst_current = [lst_open_set[i].get()[2] for i in range(n)]

        for i in range(n):
            for j in range(n):
                # Kiểm tra điều kiện dừng
                if lst_count[0] >= T_max:
                    result = fix_path_map(path, path_value) + [edges]
                    if check_path_valid(result):
                        return result
                    T_max += T_step

                # Phát hiện giao cắt giữa hai wavefront i và j
                if (i != j
                        and lst_current[i] in lst_open_set_hash[j]
                        and check_path[i][j] == 0):
                    lst_check[i] += 1
                    lst_check[j] += 1

                    pt1 = [lst_current[i]] + reconstruct_path(lst_came_from[i], lst_current[i])
                    pt2 = [lst_current[i]] + reconstruct_path(lst_came_from[j], lst_current[i])
                    ps1 = _reconstruct_path_safe(pt1, grid, lst_g_score[i])
                    ps2 = _reconstruct_path_safe(pt2, grid, lst_g_score[j])
                    ps1.reverse()

                    path[i][j] = ps1 + ps2
                    path[j][i] = list(reversed(path[i][j]))
                    check_path[i][j] = check_path[j][i] = 1

                    cost_ij = lst_g_score[i][lst_current[i]] + lst_g_score[j][lst_current[i]]
                    path_value[i][j] = path_value[j][i] = cost_ij
                    edges.append((cost_ij, (i, j)))

                    max_count_path += 2
                    if max_count_path >= n * (n - 1):
                        return fix_path_map(path, path_value) + [edges]

        # Mở rộng wavefront
        for i in range(n):
            for neighbor in lst_current[i].neighbors:
                if (neighbor.safe >= 1.71
                        and lst_g_score[i][lst_current[i]] < lst_g_score[i][neighbor]
                        and neighbor not in lst_open_set_hash[i]):

                    lst_came_from[i][neighbor] = lst_current[i]
                    r, c = neighbor.get_row(), neighbor.get_col()

                    if r + 1 < rows and r - 1 > 0:
                        dx = min(lst_g_score[i][grid[r+1][c]], lst_g_score[i][grid[r-1][c]])
                    elif r + 1 == rows:
                        dx = lst_g_score[i][grid[r-1][c]]
                    else:
                        dx = lst_g_score[i][grid[r+1][c]]

                    if c + 1 < rows and c - 1 > 0:
                        dy = min(lst_g_score[i][grid[r][c+1]], lst_g_score[i][grid[r][c-1]])
                    elif c + 1 == rows:
                        dy = lst_g_score[i][grid[r][c-1]]
                    else:
                        dy = lst_g_score[i][grid[r][c+1]]

                    disc = 2.0 - (dx - dy) ** 2
                    lst_g_score[i][neighbor] = (
                        (dx + dy + math.sqrt(disc)) / 2 if disc >= 0
                        else min(dx, dy) + 1
                    )
                    lst_f_score[i][neighbor] = (
                        a_dgree * lst_g_score[i][neighbor]
                        + lst_current[i].get_safe() * (-1 + a_dgree)
                    )
                    lst_count[i] += 1
                    lst_open_set[i].put((lst_f_score[i][neighbor], lst_count[i], neighbor))
                    lst_open_set_hash[i].add(neighbor)

        for i in range(n):
            if lst_current[i] not in lst_start:
                lst_current[i].make_closed(i)

    return fix_path_map(path, path_value) + [edges]


# ──────────────────────────────────────────────────────────────────
# 6. Mở rộng permutation TSP → danh sách ô thực tế trên lưới
# ──────────────────────────────────────────────────────────────────
def _expand_path(path_matrix, ds):
    """
    Nối các sub-path theo thứ tự ds thành 1 danh sách ô.
    Loại bỏ trùng lặp giữa các đoạn (giống draw_path_final).
    """
    cells = []
    seen  = set()
    n = len(ds)
    for k in range(1, n):
        seg = path_matrix[ds[k - 1]][ds[k]]
        for spot in seg:
            if id(spot) not in seen:
                seen.add(id(spot))
                cells.append(spot)
    return cells


# ──────────────────────────────────────────────────────────────────
# 7a. Chuyển danh sách Spot → list tuple toạ độ (row, col)
# ──────────────────────────────────────────────────────────────────
def _cells_to_coords(cells):
    """Trả về list [(row, col), ...] từ danh sách Spot."""
    return [(sp.get_row(), sp.get_col()) for sp in cells]


# ──────────────────────────────────────────────────────────────────
# 7b. Tính toán tất cả tiêu chí từ đường đi đã mở rộng
# ──────────────────────────────────────────────────────────────────
def _compute_metrics(cells, path_matrix, ds,
                     safe_medium, perimeter_map, a_dgree, w1, w2):
    """
    Trả về dict tiêu chí:
      path_length  — tổng Euclidean distance
      safety_mean  — an toàn trung bình (từ Spot.safe)
      safety_min   — an toàn thấp nhất dọc đường
      risk         — Σ max(0, 1 − S(p)/safe_medium)  (tổng rủi ro)
      risk_mean    — risk / số bước
      total_cost   — w1·path_length + w2·risk
      F_score      — hàm mục tiêu gốc dự án
    """
    if not cells:
        return {"error": "Đường đi rỗng"}

    # Độ dài đường đi
    path_length = sum(
        delta(cells[k - 1].get_pos(), cells[k].get_pos())
        for k in range(1, len(cells))
    )

    # An toàn (dùng hàm gốc từ FFT_TSP.py)
    safety_mean = calculate_safety(path_matrix, ds)
    safety_min  = min_safety(path_matrix, ds)

    # Rủi ro va chạm — chuẩn hoá theo safe_medium
    s_ref = safe_medium if safe_medium > 0 else 1.0
    risk  = sum(max(0.0, 1.0 - sp.safe / s_ref) for sp in cells)
    risk_mean = risk / len(cells)

    # Total cost (tương đương FMF_new: w1·L + w2·R)
    total_cost = w1 * path_length + w2 * risk

    # F_score — hàm mục tiêu gốc của multi_object.py
    try:
        f_score = (
            safety_mean / s_ref / math.log1p(1.0 - a_dgree)
            + (perimeter_map / path_length) / math.log1p(a_dgree)
        )
    except (ZeroDivisionError, ValueError):
        f_score = None

    return {
        "path_length":  round(path_length,  4),
        "safety_mean":  round(safety_mean,  4),
        "safety_min":   round(safety_min,   4),
        "risk":         round(risk,         4),
        "risk_mean":    round(risk_mean,    6),
        "total_cost":   round(total_cost,   4),
        "F_score": (round(f_score, 6) if f_score is not None and math.isfinite(f_score) else None),
    }


# ──────────────────────────────────────────────────────────────────
# 8. Lưu / cập nhật file JSON kết quả
# ──────────────────────────────────────────────────────────────────
def _save_results(map_path, config_snapshot, solver_results):
    results_dir = os.path.join(_HERE, "results")
    os.makedirs(results_dir, exist_ok=True)

    map_name  = os.path.splitext(os.path.basename(map_path))[0]
    json_path = os.path.join(results_dir, f"{map_name}_eval.json")

    if os.path.exists(json_path):
        with open(json_path, "r", encoding="utf-8") as f:
            record = json.load(f)
    else:
        record = {"map": map_name, "map_path": map_path, "runs": []}

    run_entry = {
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        "config":    config_snapshot,
        "solvers":   solver_results,
    }
    record["runs"].append(run_entry)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(record, f, ensure_ascii=False, indent=2)

    print(f"\nKết quả đã lưu → {json_path}  (tổng số lần chạy: {len(record['runs'])})")
    return json_path


# ──────────────────────────────────────────────────────────────────
# 9. In kết quả một bộ giải
# ──────────────────────────────────────────────────────────────────
def _print_solver(name, m):
    print(f"\n  ── {name} ──────────────────────────")
    if "error" in m:
        print(f"  LỖI: {m['error']}")
        return
    print(f"  path_length  = {m.get('path_length')}")
    print(f"  safety_mean  = {m.get('safety_mean')}   (min: {m.get('safety_min')})")
    print(f"  risk         = {m.get('risk')}   (trung bình/bước: {m.get('risk_mean')})")
    print(f"  total_cost   = {m.get('total_cost')}")
    print(f"  F_score      = {m.get('F_score')}")
    print(f"  time_fmm     = {m.get('time_fmm_sec')}s")
    print(f"  time_tsp     = {m.get('time_tsp_sec')}s")
    print(f"  time_total   = {m.get('time_total_sec')}s")
    coords = m.get("path_coords")
    if coords:
        preview = coords[:5]
        suffix  = f" ... ({len(coords)} bước)" if len(coords) > 5 else f" ({len(coords)} bước)"
        print(f"  path_coords  = {preview}{suffix}")


# ──────────────────────────────────────────────────────────────────
# 10. In bảng so sánh tổng hợp
# ──────────────────────────────────────────────────────────────────
def _print_comparison(solver_results):
    if not solver_results:
        return
    header = f"  {'Solver':<22} {'Length':>10} {'Safety':>10} {'Risk':>10} {'TotalCost':>12} {'F_score':>10} {'Time(s)':>9}"
    sep    = "  " + "-" * (len(header) - 2)
    print("\n" + "=" * len(header))
    print("  SO SÁNH CÁC BỘ GIẢI TSP")
    print(header)
    print(sep)
    for name, m in solver_results.items():
        if "error" in m:
            print(f"  {name:<22}  (không có kết quả: {m['error']})")
            continue
        print(
            f"  {name:<22}"
            f" {str(m.get('path_length', '-')):>10}"
            f" {str(m.get('safety_mean', '-')):>10}"
            f" {str(m.get('risk',        '-')):>10}"
            f" {str(m.get('total_cost',  '-')):>12}"
            f" {str(m.get('F_score',     '-')):>10}"
            f" {str(m.get('time_total_sec', '-')):>9}"
        )
    print("=" * len(header))

    # Tìm bộ giải tốt nhất theo từng tiêu chí
    valid = {k: v for k, v in solver_results.items() if "error" not in v}
    if len(valid) >= 2:
        print()
        for metric, label, lower_better in [
            ("path_length", "Đường đi ngắn nhất", True),
            ("safety_mean", "An toàn nhất",        False),
            ("risk",        "Rủi ro thấp nhất",    True),
            ("total_cost",  "Chi phí thấp nhất",   True),
        ]:
            vals = {k: v[metric] for k, v in valid.items() if metric in v and v[metric] is not None}
            if vals:
                best = min(vals, key=vals.__getitem__) if lower_better else max(vals, key=vals.__getitem__)
                print(f"  ✓ {label:<22}: {best}  ({vals[best]})")


# ──────────────────────────────────────────────────────────────────
# 11. Pipeline chính
# ──────────────────────────────────────────────────────────────────
def evaluate(map_path=None, config=None):
    """
    Pipeline đánh giá đầy đủ:
      Pha 1 — FMM đa nguồn → ma trận chi phí giữa các checkpoint
      Pha 2A — Christofides
      Pha 2B — ACO
      Pha 2C — FFT Backtracking (chỉ khi n ≤ 12)

    Tham số:
      map_path  — đường dẫn file .txt (mặc định: CONFIG["map_path"])
      config    — dict ghi đè CONFIG (tuỳ chọn)

    Trả về: dict {solver_name: metrics_dict}
    """
    cfg      = {**CONFIG, **(config or {})}
    map_path = map_path or cfg["map_path"]

    ROWS  = 200
    WIDTH = 800
    w1       = cfg["w1"]
    w2       = cfg["w2"]
    a_dgree  = cfg["a_dgree"]
    T_max    = cfg["T_max"]
    T_step   = cfg["T_step"]

    print("=" * 62)
    print(f"  Bản đồ  : {map_path}")
    print(f"  Trọng số: w1={w1}, w2={w2}")
    print(f"  FMM     : T_max={T_max}, T_step={T_step}, a_dgree={a_dgree}")
    print("=" * 62)

    # ── Khởi tạo lưới ─────────────────────────────────────────────
    grid = make_grid(ROWS, WIDTH, safe=float("inf"))

    # ── Đọc bản đồ ────────────────────────────────────────────────
    if not os.path.isfile(map_path):
        # Thử relative path từ thư mục caothanhhuy
        map_path = os.path.join(_HERE, map_path)
    if not os.path.isfile(map_path):
        raise FileNotFoundError(f"Không tìm thấy bản đồ: {map_path}")

    data = read_map_from_file2(grid, ROWS, map_path)
    lst_start, obstacle_count = data[0], data[1]
    n = len(lst_start)
    print(f"\n  Checkpoints : {n}")
    print(f"  Ô vật cản   : {obstacle_count}")

    if n < 2:
        raise ValueError("Bản đồ cần ít nhất 2 điểm (start + 1 goal).")

    # ── Tính vùng an toàn ─────────────────────────────────────────
    print("\n[Bước 1/3] Tính safe_zone ...")
    t0 = time.time()
    safe_medium = _safe_zone_headless(grid, ROWS)
    t_safe = time.time() - t0
    print(f"  safe_medium = {safe_medium}  ({t_safe:.2f}s)")

    # ── Chu vi bản đồ (dùng trong F_score) ───────────────────────
    perimeter_map = cal_perimeter(map_path, ROWS)

    # ── Cập nhật láng giềng (bắt buộc trước khi chạy FMM) ────────
    for row in grid:
        for spot in row:
            spot.update_neighbors(grid)

    # ── FMM đa nguồn ──────────────────────────────────────────────
    print(f"\n[Bước 2/3] FMM đa nguồn (n={n} checkpoints) ...")
    t0 = time.time()
    raw = _algorithm_headless(grid, lst_start, T_step, T_max, a_dgree)
    fmm_time    = time.time() - t0
    path_matrix = raw[0]
    path_value  = np.array(raw[1])
    edges       = raw[2]
    print(f"  Hoàn thành ({fmm_time:.2f}s)")

    # ── Giải TSP ──────────────────────────────────────────────────
    print("\n[Bước 3/3] Giải TSP ...")
    solver_results = {}

    # ── 2A: Christofides ─────────────────────────────────────────
    print("  → Christofides ...")
    t0 = time.time()
    try:
        c  = CRIST(path_value)
        ds = sort_ds(c.ans(edges))
        t_tsp = time.time() - t0
        cells = _expand_path(path_matrix, ds)
        m = _compute_metrics(cells, path_matrix, ds,
                             safe_medium, perimeter_map, a_dgree, w1, w2)
        m.update({
            "time_fmm_sec":   round(fmm_time,          4),
            "time_tsp_sec":   round(t_tsp,             4),
            "time_total_sec": round(fmm_time + t_tsp,  4),
            "route":          ds,
            "cells_count":    len(cells),
            "path_coords":    _cells_to_coords(cells),
        })
    except Exception as e:
        m = {"error": str(e)}
    _print_solver("Christofides", m)
    solver_results["Christofides"] = m

    # ── 2B: ACO ──────────────────────────────────────────────────
    print("\n  → ACO ...")
    t0 = time.time()
    try:
        aco       = ACO(cfg["aco_ants"], cfg["aco_gen"],
                        cfg["aco_alpha"], cfg["aco_beta"],
                        cfg["aco_rho"],   cfg["aco_q"],
                        cfg["aco_strategy"])
        graph     = Graph(path_value.tolist(), n)
        aco_route, aco_cost = aco.solve(graph)
        ds_aco    = sort_ds(aco_route + [aco_route[0]])  # khép vòng
        t_tsp     = time.time() - t0
        cells_aco = _expand_path(path_matrix, ds_aco)
        m = _compute_metrics(cells_aco, path_matrix, ds_aco,
                             safe_medium, perimeter_map, a_dgree, w1, w2)
        m.update({
            "aco_raw_cost":   round(float(aco_cost), 4),
            "time_fmm_sec":   round(fmm_time,         4),
            "time_tsp_sec":   round(t_tsp,            4),
            "time_total_sec": round(fmm_time + t_tsp, 4),
            "route":          ds_aco,
            "cells_count":    len(cells_aco),
            "path_coords":    _cells_to_coords(cells_aco),
        })
    except Exception as e:
        m = {"error": str(e)}
    _print_solver("ACO", m)
    solver_results["ACO"] = m

    # ── 2C: FFT Backtracking (chỉ khi n ≤ 12) ────────────────────
    if n <= 12:
        print(f"\n  → FFT Backtracking (n={n}) ...")
        t0 = time.time()
        try:
            ds_fft = efficient_branching(path_value)
            t_tsp  = time.time() - t0
            if ds_fft:
                ds_fft    = sort_ds(ds_fft)
                cells_fft = _expand_path(path_matrix, ds_fft)
                m = _compute_metrics(cells_fft, path_matrix, ds_fft,
                                     safe_medium, perimeter_map, a_dgree, w1, w2)
                m.update({
                    "time_fmm_sec":   round(fmm_time,         4),
                    "time_tsp_sec":   round(t_tsp,            4),
                    "time_total_sec": round(fmm_time + t_tsp, 4),
                    "route":          ds_fft,
                    "cells_count":    len(cells_fft),
                    "path_coords":    _cells_to_coords(cells_fft),
                })
            else:
                m = {"error": "FFT Backtracking không tìm được nghiệm"}
        except Exception as e:
            m = {"error": str(e)}
        _print_solver("FFT_Backtracking", m)
        solver_results["FFT_Backtracking"] = m
    else:
        print(f"\n  → FFT Backtracking bỏ qua (n={n} > 12, chi phí tổ hợp quá lớn).")

    # ── So sánh & lưu ─────────────────────────────────────────────
    _print_comparison(solver_results)

    config_snapshot = {
        "w1": w1, "w2": w2, "a_dgree": a_dgree,
        "T_max": T_max, "T_step": T_step,
        "aco_ants":    cfg["aco_ants"],
        "aco_gen":     cfg["aco_gen"],
        "aco_alpha":   cfg["aco_alpha"],
        "aco_beta":    cfg["aco_beta"],
        "aco_rho":     cfg["aco_rho"],
        "aco_q":       cfg["aco_q"],
        "aco_strategy":cfg["aco_strategy"],
        "map_size":        ROWS,
        "checkpoints":     n,
        "safe_medium":     safe_medium,
        "perimeter_map":   perimeter_map,
        "obstacle_count":  obstacle_count,
    }
    _save_results(map_path, config_snapshot, solver_results)

    return solver_results


# ──────────────────────────────────────────────────────────────────
# 12. CLI
# ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Đánh giá thuật toán lập lộ trình đa mục tiêu (caothanhhuy)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--map", default=None,
        help="Đường dẫn file bản đồ .txt\n"
             "Ví dụ: map_NVIDIA/map_01.txt  hoặc  đường dẫn tuyệt đối",
    )
    parser.add_argument("--w1",     type=float, default=None, metavar="0.5",
                        help="Trọng số độ dài đường đi     (mặc định 0.5)")
    parser.add_argument("--w2",     type=float, default=None, metavar="0.5",
                        help="Trọng số rủi ro va chạm      (mặc định 0.5)")
    parser.add_argument("--T_max",  type=int,   default=None, metavar="1500",
                        help="T_max FMM — ngưỡng dừng      (mặc định 1500)")
    parser.add_argument("--T_step", type=int,   default=None, metavar="200",
                        help="T_step FMM — bước mở rộng    (mặc định 200)")
    parser.add_argument("--a",      type=float, default=None, metavar="0.65",
                        dest="a_dgree",
                        help="a_dgree — cân bằng an toàn   (mặc định 0.65)")
    parser.add_argument("--aco_ants", type=int, default=None, metavar="50",
                        help="Số kiến ACO                  (mặc định 50)")
    parser.add_argument("--aco_gen",  type=int, default=None, metavar="200",
                        help="Số thế hệ ACO                (mặc định 200)")

    args   = parser.parse_args()
    override = {k: v for k, v in vars(args).items()
                if v is not None and k != "map"}

    evaluate(map_path=args.map, config=override if override else None)
