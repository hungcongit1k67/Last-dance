"""
Supercover line on a grid.

Input: a matrix containing exactly two cells with value 2.
Output: a copied matrix where every cell touched by the straight segment
between the two marked cell centers is marked with "x" except the endpoints.

Coordinates are handled as (row, col). The traversal uses the grid-cell
boundaries, not DDA rounding, so it is conservative enough for collision,
risk, and radiation evaluation.
"""

import sys
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


def find_twos(matrix):
    """Return the two (row, col) cells whose value is 2."""
    points = []
    for r, row in enumerate(matrix):
        for c, value in enumerate(row):
            if value == 2:
                points.append((r, c))

    if len(points) != 2:
        raise ValueError(
            "Matrix must contain exactly two cells with value 2; found %d."
            % len(points)
        )
    return points[0], points[1]


# Hàm phụ để thêm cell vào danh sách cells nếu nó chưa được thấy trước đó
def _append_unique(cells, seen, cell):
    if cell not in seen:
        cells.append(cell)
        seen.add(cell)


def supercover_cells(start, end):
    """Return all grid cells touched by the segment from start to end.

    start/end are (row, col) integer cell coordinates. The line connects cell
    centers. When the line crosses a grid corner, both side-neighbor cells are
    included before the diagonal step; this is the conservative supercover.
    """
    r, c = int(start[0]), int(start[1])
    r1, c1 = int(end[0]), int(end[1])

    cells = [] # danh sách các cell được chạm bởi đoạn thẳng
    seen = set()
    _append_unique(cells, seen, (r, c)) 

    # Tính độ lệch giữa hai điểm đầu và cuối
    dc = c1 - c # độ lệch theo cột --> ngang
    dr = r1 - r # độ lệch theo hàng --> dọc

    # Xác định hướng di chuyển trên lưới
    step_c = 1 if dc > 0 else -1 if dc < 0 else 0 # Nếu dc > 0 thì di chuyển sang phải, nếu dc < 0 thì di chuyển sang trái, nếu dc = 0 thì không di chuyển theo cột
    step_r = 1 if dr > 0 else -1 if dr < 0 else 0 # Nếu dr > 0 thì di chuyển lên trên, nếu dr < 0 thì di chuyển xuống dưới, nếu dr = 0 thì không di chuyển theo hàng

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


def supercover_line(matrix):
    """Mark the supercover line connecting the two cells with value 2."""
    if not matrix or not matrix[0]:
        raise ValueError("Matrix must not be empty.")

    cols = len(matrix[0])
    if any(len(row) != cols for row in matrix):
        raise ValueError("Matrix must be rectangular.")

    start, end = find_twos(matrix)
    rows = len(matrix)
    result = [row[:] for row in matrix]

    for r, c in supercover_cells(start, end):
        if 0 <= r < rows and 0 <= c < cols and result[r][c] != 2:
            result[r][c] = "x"

    return result


def print_matrix(matrix):
    for row in matrix:
        print(" ".join(str(value) for value in row))


def draw_endpoint_rectangle(ax, start, end, color="red", linewidth=3):
    """Draw the outline of the grid subrectangle bounded by start and end."""
    from matplotlib.patches import Rectangle

    r0, c0 = start
    r1, c1 = end
    min_r, max_r = sorted((r0, r1))
    min_c, max_c = sorted((c0, c1))

    rectangle = Rectangle(
        (min_c - 0.5, min_r - 0.5),
        max_c - min_c + 1,
        max_r - min_r + 1,
        fill=False,
        edgecolor=color,
        linewidth=linewidth,
        zorder=5,
    )
    ax.add_patch(rectangle)
    return rectangle


def draw_grid(matrix, output_path="E:\\last_dance\\LastDance\\FMF_new\\supercover_grid.png", show=False):
    """Draw a grid matrix and the segment connecting the two value-2 cells.

    Cells marked with "x" are highlighted. The line connects the centers of
    the two cells whose value is 2.
    """
    try:
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
    except ImportError as exc:
        raise ImportError(
            "draw_grid requires matplotlib. Install it with: pip install matplotlib"
        ) from exc

    if not matrix or not matrix[0]:
        raise ValueError("Matrix must not be empty.")

    cols = len(matrix[0])
    if any(len(row) != cols for row in matrix):
        raise ValueError("Matrix must be rectangular.")

    rows = len(matrix)
    start, end = find_twos(matrix)

    color_index = []
    for row in matrix:
        drawn_row = []
        for value in row:
            if value == "x" or value == 2: # Nếu là "x" hoặc 2 thì tô màu vàng, nếu là 1 thì tô màu đen, còn lại tô màu xanh da trời
                drawn_row.append(1)
            elif value == 1:
                drawn_row.append(2)
            else:
                drawn_row.append(0)
        color_index.append(drawn_row)

    cmap = ListedColormap(["#87CEEB", "#FFFF66", "#222222"])
    fig_size = (max(5, cols * 0.55), max(5, rows * 0.55))
    fig, ax = plt.subplots(figsize=fig_size)
    ax.imshow(color_index, cmap=cmap, origin="upper", interpolation="none", vmin=0, vmax=2)

    ax.set_xticks([c - 0.5 for c in range(cols + 1)], minor=True)
    ax.set_yticks([r - 0.5 for r in range(rows + 1)], minor=True)
    ax.grid(which="minor", color="black", linewidth=1.6)
    ax.tick_params(which="both", bottom=False, left=False, labelbottom=False, labelleft=False)

    (r0, c0), (r1, c1) = start, end
    ax.plot([c0, c1], [r0, r1], color="red", linewidth=3, zorder=3)
    ax.scatter([c0, c1], [r0, r1], color="red", edgecolors="black", s=170, zorder=4)
    draw_endpoint_rectangle(ax, start, end)

    # vẽ 2 ngôi sao ở hai điểm đầu cuối của đoạn thẳng
    # for r, c in (start, end):
    #     ax.scatter(c, r, marker="*", color="blue", s=900, zorder=5)

    #     # Số "2" được vẽ ở góc trên bên phải của ô
    #     ax.text(
    #         c + 0.12,
    #         r + 0.12,
    #         "*",
    #         color="blue",
    #         fontsize=20,
    #         fontweight="bold",
    #         ha="left",
    #         va="center",
    #         zorder=6,
    #     )
    
    ax.text(
            c0 + 0.12,
            r0 + 0.12,
            "s",
            color="blue",
            fontsize=20,
            fontweight="bold",
            ha="left",
            va="center",
            zorder=6,
        )
    
    ax.text(
            c1 + 0.12,
            r1 + 0.12,
            "e",
            color="blue",
            fontsize=20,
            fontweight="bold",
            ha="left",
            va="center",
            zorder=6,
        )

    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)
    ax.set_aspect("equal")

    output_path = Path(output_path)
    fig.tight_layout(pad=0)
    fig.savefig(output_path, dpi=160, bbox_inches="tight", pad_inches=0.02)

    if show:
        plt.show()
    else:
        plt.close(fig)

    return output_path


if __name__ == "__main__":
    # m = [[0 for _ in range(10)] for _ in range(10)]
    # m[0][2] = 2
    # m[1][5] = 2

    # m = [[0, 2, 0, 0, 0], 
    #      [0, 0, 0, 0, 0], 
    #      [0, 0, 0, 0, 0],
    #      [0, 0, 0, 0, 0], 
    #      [0, 0, 0, 2, 0]]
    
    # ma trận 10x10
    m = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 2, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 2, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0]]
    print("Input matrix:")
    print_matrix(m)

    print("\nSupercover line:")
    marked_m = supercover_line(m)
    print_matrix(marked_m)

    print("\nCells:")
    print(supercover_cells(*find_twos(m)))

    output_file = draw_grid(marked_m)
    print("\nSaved figure:")
    print(output_file)
