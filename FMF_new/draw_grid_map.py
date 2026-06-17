"""
Draw a grid map from a matrix containing cells marked with value 2.

- Nếu có đúng 1 điểm "2": vẽ 8 mũi tên từ ô đó sang 8 ô lân cận và tô màu
  8 ô lân cận đó, rồi lưu ảnh.
- Nếu có đúng 2 điểm "2": vẽ grid map và một đường thẳng dạng mũi tên nối
  tâm hai ô "2".

Coordinates are handled as (row, col).
"""

import sys
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


def find_all_twos(matrix):
    """Trả về danh sách tất cả các ô (row, col) có giá trị 2."""
    points = []
    for r, row in enumerate(matrix):
        for c, value in enumerate(row):
            if value == 2:
                points.append((r, c))
    return points


def _setup_grid_axes(ax, color_index, rows, cols):
    """Vẽ nền màu + đường lưới chung cho cả hai trường hợp."""
    from matplotlib.colors import ListedColormap

    cmap = ListedColormap(["#87CEEB", "#FFFF66", "#222222"])
    ax.imshow(color_index, cmap=cmap, origin="upper", interpolation="none", vmin=0, vmax=2)

    ax.set_xticks([c - 0.5 for c in range(cols + 1)], minor=True)
    ax.set_yticks([r - 0.5 for r in range(rows + 1)], minor=True)
    ax.grid(which="minor", color="black", linewidth=1.6)
    ax.tick_params(which="both", bottom=False, left=False, labelbottom=False, labelleft=False)

    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)
    ax.set_aspect("equal")


def draw_grid_map(matrix, output_path="E:\\last_dance\\LastDance\\FMF_new\\grid_map.png", show=False):
    """Vẽ grid map tùy theo số điểm có giá trị 2.

    - Nếu có đúng 1 điểm "2": vẽ 8 mũi tên từ ô đó sang 8 ô lân cận và tô màu
      8 ô lân cận đó.
    - Nếu có đúng 2 điểm "2": vẽ grid map và một đường thẳng dạng mũi tên nối
      tâm hai ô "2".
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise ImportError(
            "draw_grid_map requires matplotlib. Install it with: pip install matplotlib"
        ) from exc

    if not matrix or not matrix[0]:
        raise ValueError("Matrix must not be empty.")

    cols = len(matrix[0])
    if any(len(row) != cols for row in matrix):
        raise ValueError("Matrix must be rectangular.")
    rows = len(matrix)

    points = find_all_twos(matrix)
    if len(points) not in (1, 2):
        raise ValueError(
            "Matrix must contain exactly one or two cells with value 2; found %d."
            % len(points)
        )

    # 8 ô lân cận sẽ được tô màu khi chỉ có 1 điểm "2".
    neighbor_cells = set()
    if len(points) == 1:
        (r0, c0) = points[0]
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r0 + dr, c0 + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    neighbor_cells.add((nr, nc))

    # Bảng màu: 0 = xanh da trời (trống), 1 = vàng (ô "2"/lân cận/"x"), 2 = đen (vật cản).
    color_index = []
    for r, row in enumerate(matrix):
        drawn_row = []
        for c, value in enumerate(row):
            if value == "x" or value == 2 or (r, c) in neighbor_cells:
                drawn_row.append(1)
            elif value == 1:
                drawn_row.append(2)
            else:
                drawn_row.append(0)
        color_index.append(drawn_row)

    fig_size = (max(5, cols * 0.55), max(5, rows * 0.55))
    fig, ax = plt.subplots(figsize=fig_size)
    _setup_grid_axes(ax, color_index, rows, cols)

    if len(points) == 1:
        (r0, c0) = points[0]
        # Tô đậm ô trung tâm.
        ax.scatter(c0, r0, color="red", edgecolors="black", s=200, zorder=4)
        # ax.text(
        #     c0 + 0.12, r0 + 0.12, "2",
        #     color="blue", fontsize=18, fontweight="bold",
        #     ha="left", va="center", zorder=6,
        # )
        # Vẽ 8 mũi tên từ tâm ô ra 8 ô lân cận.
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r0 + dr, c0 + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                ax.annotate(
                    "",
                    xy=(nc, nr), xytext=(c0, r0),
                    arrowprops=dict(arrowstyle="->", color="red", lw=2.2),
                    zorder=5,
                )
    else:
        (r0, c0), (r1, c1) = points
        # Đường thẳng dạng mũi tên nối tâm hai ô "2".
        ax.annotate(
            "",
            xy=(c1, r1), xytext=(c0, r0),
            arrowprops=dict(arrowstyle="->", color="red", lw=2.2, mutation_scale=30),
            zorder=3,
        )
        ax.scatter([c0, c1], [r0, r1], color="red", edgecolors="black", s=170, zorder=4)
        ax.text(
            c0 + 0.12, r0 + 0.12, "p1",
            color="blue", fontsize=20, fontweight="bold",
            ha="left", va="center", zorder=6,
        )
        ax.text(
            c1 + 0.12, r1 + 0.12, "p2",
            color="blue", fontsize=20, fontweight="bold",
            ha="left", va="center", zorder=6,
        )

    output_path = Path(output_path)
    fig.tight_layout(pad=0)
    fig.savefig(output_path, dpi=160, bbox_inches="tight", pad_inches=0.02)

    if show:
        plt.show()
    else:
        plt.close(fig)

    return output_path


if __name__ == "__main__":
    # Ma trận tự nhập (ví dụ có 2 điểm "2").
    m = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 2, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 2, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    output_file = draw_grid_map(m)
    print("Saved figure:")
    print(output_file)
