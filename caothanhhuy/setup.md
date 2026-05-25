# Hướng dẫn cài đặt và chạy dự án

## Giới thiệu

Dự án của **Cao Thành Huy** (MSSV: 20204656) là hệ thống **lập lộ trình đa mục tiêu cho robot** trong môi trường có chướng ngại vật, kết hợp nhiều thuật toán tìm đường và giải bài toán TSP (Traveling Salesman Problem), được trực quan hóa bằng Pygame.

### Các thuật toán tích hợp
- **Tìm đường**: Fast Marching Method, A*, BFS, DFS, Dijkstra
- **Giải TSP**: Ant Colony Optimization (ACO), Christofides (~1.5x tối ưu), Backtracking, Branch & Bound
- **Phân vùng môi trường**: Voronoi Diagram
- **Robot**: Động học vi sai, tránh chướng ngại vật bằng KDTree

### Cấu trúc thư mục

```
caothanhhuy/
├── multi_object.py         # Ứng dụng chính (controller)
├── Fast_marching_method.py # Thuật toán Fast Marching Method
├── ACO_TSP.py              # Ant Colony Optimization
├── TSP_cristofides_2.py    # Thuật toán Christofides
├── FFT_TSP.py              # Backtracking + Branch & Bound
├── Voronoi.py              # Phân vùng Voronoi
├── Robot.py                # Mô hình động học robot
├── Map_Grid.py             # Quản lý lưới bản đồ
├── Spot.py                 # Lớp ô lưới (cell)
├── fixmap.py               # Tiền xử lý bản đồ
├── Test_docfile.py         # Công cụ chuyển ảnh thành lưới
├── map_DVRP/               # Bản đồ giao hàng (5 file)
├── map_health_care/        # Bản đồ y tế (5 file)
├── map_IN2D/               # Bản đồ 2D (17 file)
├── map_NVIDIA/             # Bản đồ NVIDIA (8 file)
├── map_random/             # Bản đồ ngẫu nhiên (23 file)
├── map_unknow/             # Bản đồ khác (26 file)
└── resault/                # Kết quả mẫu (4 file)
```

---

## Yêu cầu hệ thống

- **Python**: 3.9 trở lên
- **OS**: Windows / Linux / macOS

---

## Cài đặt môi trường

### Bước 1 — Tạo virtual environment

```bash
# Di chuyển vào thư mục dự án
cd path/to/caothanhhuy

# Tạo .venv
python -m venv .venv
```

### Bước 2 — Kích hoạt .venv

**Windows (PowerShell):**
```powershell
.venv\Scripts\Activate.ps1
```

**Windows (CMD):**
```cmd
.venv\Scripts\activate.bat
```

**Linux / macOS:**
```bash
source .venv/bin/activate
```

> Sau khi kích hoạt thành công, dấu nhắc terminal sẽ hiển thị `(.venv)` ở đầu dòng.

### Bước 3 — Cài đặt thư viện

```bash
pip install -r requirements.txt
```

---

## Chạy chương trình

### Khởi động

```bash
python multi_object.py
```

> **Cửa sổ mở ra trắng tinh là bình thường** — lưới chưa có bản đồ, phải load thủ công bằng phím tắt bên dưới.

---

### Luồng sử dụng cơ bản

```
Bước 1  →  (Tuỳ chọn) Chọn bản đồ bằng phím 2/3/4/5
Bước 2  →  Nhấn 9 để load bản đồ lên màn hình
Bước 3  →  Nhấn Space để chạy thuật toán tìm đường
```

#### Ví dụ nhanh — load bản đồ mặc định và chạy

1. Mở cửa sổ (trắng)
2. Nhấn **`9`** → bản đồ `map_factory_200` hiện ra với tường đen, điểm đỏ (start), điểm xanh (goal)
3. Nhấn **`Space`** → thuật toán chạy, đường đi tím hiện ra

---

### Bảng phím tắt đầy đủ

#### Chọn bản đồ (nhấn trước khi load)

| Phím | Bản đồ được chọn |
|---|---|
| *(mặc định)* | `map_random/map_factory_200.txt` |
| **`2`** | `map_NVIDIA/map_01.txt` |
| **`3`** | `map_NVIDIA/map_02.txt` |
| **`4`** | `map_NVIDIA/map_03.txt` |
| **`5`** | `map_NVIDIA/map_04.txt` |

#### Load & hiển thị

| Phím | Hành động |
|---|---|
| **`9`** | **Load bản đồ đang chọn lên lưới** (bắt buộc trước khi chạy) |
| **`f`** | Tính toán và tô màu vùng an toàn xung quanh chướng ngại vật |
| **`d`** | Vẽ phân vùng Voronoi lên lưới |

#### Tạo waypoint ngẫu nhiên

| Phím | Hành động |
|---|---|
| **`6`** | Sinh 20 waypoint ngẫu nhiên trên bản đồ hiện tại |
| **`7`** | Sinh 30 waypoint ngẫu nhiên |
| **`8`** | Sinh 50 waypoint ngẫu nhiên |

> Các phím 6/7/8 ghi đè file bản đồ gốc — dùng cẩn thận.

#### Chạy thuật toán

| Phím | Hành động |
|---|---|
| **`Space`** | **Chạy thuật toán tìm đường** (cần đã load bản đồ có điểm start + end) |
| **`1`** | Chuyển sang chế độ thuật toán thay thế (choose = 0) |

#### Tương tác chuột

| Thao tác | Hành động |
|---|---|
| **Chuột trái** | Click vào ô trống → đặt điểm **start** (lần 1), **end** (lần 2), waypoint (lần 3+), hoặc tường |
| **Chuột phải** | Xoá ô bất kỳ (tường / start / end / waypoint) |

#### Tiện ích

| Phím | Hành động |
|---|---|
| **`c`** | Xoá toàn bộ lưới, reset về trạng thái trắng ban đầu |
| **`Esc` / đóng cửa sổ** | Thoát chương trình |

---

### Định dạng file bản đồ

Các bản đồ lưu trong `map_*/` là file `.txt`, mỗi dòng là một hàng ô lưới, giá trị cách nhau bằng dấu cách:

| Giá trị | Ý nghĩa | Màu hiển thị |
|---|---|---|
| `0` | Ô trống | Trắng |
| `1` | Chướng ngại vật | Đen |
| `2` | Waypoint / điểm đích | Xanh lam |
| `3` | Điểm xuất phát (start) | Đỏ |

Đường đi tìm được sẽ tô màu **tím** trên lưới.

---

## Lưu ý

> **Cảnh báo:** File `multi_object.py` import hai module `GWO_TSP` và `GWO_TSP2` (Grey Wolf Optimizer) không có trong thư mục này. Nếu gặp lỗi `ModuleNotFoundError`, hãy liên hệ tác giả để lấy hai file còn thiếu, hoặc comment tạm hai dòng import đó nếu không cần dùng đến GWO.

```python
# multi_object.py — dòng 12-13, comment tạm nếu thiếu file:
# from GWO_TSP import *
# from GWO_TSP2 import *
```

---

## Thư viện sử dụng

| Thư viện | Mục đích |
|---|---|
| `pygame` | Giao diện đồ họa, vẽ lưới và animation |
| `numpy` | Tính toán ma trận, xử lý lưới |
| `scipy` | KDTree (tránh vật cản), Voronoi, convolution, interpolation |
| `matplotlib` | Vẽ biểu đồ kết quả |
| `Pillow` | Đọc ảnh PNG để chuyển thành bản đồ lưới |
| `xlwt` | Xuất dữ liệu ra file Excel |
