"""
Bresenham line on a grid.

Input: a matrix containing exactly two cells with value 2.
Output: a copied matrix where every cell on the Bresenham line between the two
marked cell centers is marked with "x" except the endpoints.

Coordinates are handled as (row, col). Unlike the supercover variant, the
classic Bresenham line visits exactly one cell per major-axis step (a thin
staircase): when the line crosses a grid corner it steps diagonally without
adding the two side-neighbor cells. This is the standard integer line-drawing
algorithm (Bresenham, 1965).
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


def bresenham_cells(start, end):
    """Return the grid cells on the Bresenham line from start to end.

    Transcription trung thực của thuật toán LineOfSight (Algorithm 6) trong
    Theta* (Daniel, Nash, Koenig & Felner, 2010) — vạch đường bằng thuật toán
    Bresenham (Bresenham, 1965) chỉ dùng phép toán số nguyên (biến tích lũy f).

    Quy ước trong bài báo dùng (x, y) với x là trục ngang, y là trục dọc; ở đây
    ánh xạ x = col, y = row, và mỗi ô lưới ghi lại theo (row, col). Vòng lặp đi
    theo trục chính (trục có độ lệch lớn hơn) một bước mỗi vòng, bước theo trục
    phụ khi f vượt ngưỡng -> đúng một ô trên mỗi bước trục chính (đường mảnh).
    """
    # x = col, y = row  (theo ký hiệu của Algorithm 6)
    x0, y0 = int(start[1]), int(start[0])   # 107-108: x0:=s.x; y0:=s.y
    x1, y1 = int(end[1]), int(end[0])       # 109-110: x1:=s'.x; y1:=s'.y

    dy = y1 - y0                            # 111
    dx = x1 - x0                            # 112
    f = 0                                   # 113

    if dy < 0:                              # 114-116
        dy = -dy
        sy = -1
    else:                                   # 117-118
        sy = 1

    if dx < 0:                              # 119-121
        dx = -dx
        sx = -1
    else:                                   # 122-123
        sx = 1

    cells = []  # các ô (row, col) nằm trên đường nhìn thẳng
    seen = set()
    _append_unique(cells, seen, (y0, x0))

    if dx >= dy:                            # 124: trục chính là x (col)
        while x0 != x1:                     # 125
            f += dy                         # 126
            if f >= dx:                     # 127
                y0 += sy                    # 130
                f -= dx                     # 131
            x0 += sx                        # 136
            _append_unique(cells, seen, (y0, x0))
    else:                                   # 137: trục chính là y (row)
        while y0 != y1:                     # 138
            f += dx                         # 139
            if f >= dy:                     # 140
                x0 += sx                    # 143
                f -= dy                     # 144
            y0 += sy                        # 149
            _append_unique(cells, seen, (y0, x0))

    return cells


def bresenham_line(matrix):
    """Mark the Bresenham line connecting the two cells with value 2."""
    if not matrix or not matrix[0]:
        raise ValueError("Matrix must not be empty.")

    cols = len(matrix[0])
    if any(len(row) != cols for row in matrix):
        raise ValueError("Matrix must be rectangular.")

    start, end = find_twos(matrix)
    rows = len(matrix)
    result = [row[:] for row in matrix]

    for r, c in bresenham_cells(start, end):
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


def draw_grid(matrix, output_path="E:\\last_dance\\LastDance\\FMF_new\\bresenham_grid_paper.png", show=False):
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
            if value == "x" or value == 2:  # Nếu là "x" hoặc 2 thì tô màu vàng, nếu là 1 thì tô màu đen, còn lại tô màu xanh da trời
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
    # ma trận 10x10
    m = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 2, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    print("Input matrix:")
    print_matrix(m)

    print("\nBresenham line:")
    marked_m = bresenham_line(m)
    print_matrix(marked_m)

    print("\nCells:")
    print(bresenham_cells(*find_twos(m)))

    output_file = draw_grid(marked_m)
    print("\nSaved figure:")
    print(output_file)
