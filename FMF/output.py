import numpy as np

def read_map_txt(path):
    with open(path, "r", encoding="utf-8") as f:
        lines = [line.strip() for line in f if line.strip()]

    # Dòng đầu là kích thước map
    mapsize = int(lines[0])

    # Kiểm tra số dòng dữ liệu map
    if len(lines) != mapsize + 1:
        raise ValueError(
            f"File không hợp lệ: mapsize = {mapsize} nhưng có {len(lines)-1} dòng map"
        )

    gr = []
    points = []

    # Đọc ma trận map
    for i in range(mapsize):
        row = list(map(int, lines[i + 1].split()))

        if len(row) != mapsize:
            raise ValueError(
                f"Dòng {i+2} không đủ {mapsize} phần tử, hiện có {len(row)}"
            )

        gr.append(row)

        # Tìm các goal (ô có giá trị 2)
        for j, val in enumerate(row):
            if val == 2:
                points.append([i, j])

    npos = len(points)

    # Nếu muốn dùng numpy array thì mở dòng dưới
    gr = np.array(gr, dtype=int)

    return mapsize, npos, points, gr


# =========================
# Cách dùng
# =========================
path = r"E:\last_dance\Code\allcode\FMF\map20_2.txt"

mapsize, npos, points, gr = read_map_txt(path)

print("mapsize =", mapsize)
print("npos =", npos)
print("points =", points)
print("grid =")
print(gr)