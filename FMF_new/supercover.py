"""
supercover.py — Supercover line trên lưới (dùng cho hàm mục tiêu khi smooth=True).

Tách riêng phần lõi `supercover_cells` từ supercover_line.py để My_grid /
ADR_main gọi khi cần liệt kê các ô mà đoạn thẳng nối hai waypoint cắt qua
(SCL — supercover line), phục vụ công thức risk(P) và R(P) cho path không
phải cell-to-cell.

Lưu ý: danh sách trả về BAO GỒM cả hai ô đầu mút (start và end).
Toạ độ theo (row, col).
"""


def _append_unique(cells, seen, cell):
    if cell not in seen:
        cells.append(cell)
        seen.add(cell)


def supercover_cells(start, end):
    """Trả về mọi ô lưới mà đoạn thẳng nối tâm hai ô start→end đi qua.

    start/end là toạ độ ô (row, col). Khi đoạn thẳng cắt qua một góc lưới,
    cả hai ô kề cạnh góc đó đều được thêm vào trước khi bước chéo — đây là
    supercover bảo toàn (conservative). Danh sách KÈM cả hai đầu mút.
    """
    r, c = int(start[0]), int(start[1])
    r1, c1 = int(end[0]), int(end[1])

    cells = []
    seen = set()
    _append_unique(cells, seen, (r, c))

    dc = c1 - c # chiều lệch theo cột --> ngang
    dr = r1 - r # chiều lệch theo hàng --> dọc
    step_c = 1 if dc > 0 else -1 if dc < 0 else 0
    step_r = 1 if dr > 0 else -1 if dr < 0 else 0

    if dc == 0 and dr == 0:
        return cells

    inf = float("inf")

    # Tính t_delta cho mỗi trục: khoảng cách t giữa các lần cắt qua ranh giới ô theo trục đó.
    t_delta_c = abs(1.0 / dc) if dc != 0 else inf
    t_delta_r = abs(1.0 / dr) if dr != 0 else inf

    # Từ tâm ô, biên đầu tiên theo mỗi trục cách nửa ô. t chạy từ 0.0 (start)
    # đến 1.0 (end).
    t_max_c = 0.5 * t_delta_c if dc != 0 else inf
    t_max_r = 0.5 * t_delta_r if dr != 0 else inf

    while (r, c) != (r1, c1):
        if t_max_c < t_max_r:
            c += step_c
            t_max_c += t_delta_c
            _append_unique(cells, seen, (r, c))
        elif t_max_r < t_max_c:
            r += step_r
            t_max_r += t_delta_r
            _append_unique(cells, seen, (r, c))
        else:
            # Cắt qua góc: thêm cả hai ô chia sẻ góc đó, rồi tới ô chéo.
            side_c = (r, c + step_c)
            side_r = (r + step_r, c)
            r += step_r
            c += step_c
            t_max_c += t_delta_c
            t_max_r += t_delta_r
            _append_unique(cells, seen, side_c)
            _append_unique(cells, seen, side_r)
            _append_unique(cells, seen, (r, c))

    return cells
