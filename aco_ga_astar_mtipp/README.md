# ACO-GA-A* MTIPP

Bản cài đặt Python tham khảo cho thuật toán **ACO-GA-A\*** giải bài toán **Multi-Target Inspection Path Planning (MTIPP)** của robot di động trong môi trường trong nhà có vật cản và phóng xạ.

> Đây là bản code lại từ mô tả thuật toán, công thức và mã giả trong bài báo. Không phải source code chính thức của tác giả.

## Ý tưởng chính

Thuật toán được chia thành 2 tầng:

1. **Lower-level: Modified A\***
   - Tìm đường đi tối ưu giữa từng cặp target.
   - Tránh vật cản.
   - Tránh vùng phóng xạ vượt ngưỡng `ri_max`.
   - Tính 3 loại chi phí: chiều dài đường đi, rủi ro phóng xạ, năng lượng khi rẽ.
   - Sinh các ma trận chi phí: `length_matrix`, `risk_matrix`, `energy_matrix`, `total_cost_matrix`.

2. **Upper-level: GA + ACO**
   - GA tìm thứ tự target gần tối ưu ban đầu.
   - Kết quả GA dùng để khởi tạo pheromone không đồng đều cho ACO.
   - ACO tối ưu thứ tự thăm tất cả target rồi quay về điểm đầu.

## Dữ liệu bản đồ

Mỗi kịch bản dùng 2 file `.txt`:

- `obstacle_grid.txt`: bản đồ vật cản và target
  - `0`: ô đi được
  - `1`: vật cản
  - `2`: target phải thăm
- `radiation_grid.txt`: bản đồ suất liều/phóng xạ, cùng kích thước với obstacle map

Ví dụ:

```txt
0 0 1 0 0 0 0 0 0 0
0 0 1 0 2 0 0 0 0 0
0 0 1 0 0 0 1 1 0 0
```

## Cài đặt

```bash
cd aco_ga_astar_mtipp
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
pip install -r requirements.txt
```

## Chạy nhanh

```bash
python run.py --config configs/default.yaml
```

Chạy kịch bản khác:

```bash
python run.py --config configs/scenario_medium.yaml
python run.py --config configs/scenario_large.yaml
python run.py --config configs/scenario_custom_200x200.yaml
python run.py --config configs/scenario_triangle_300_lowrisk.yaml
python run.py --config configs/scenario_square_400_lowrisk.yaml
python run.py --config configs/scenario5.yaml
```

## Kết quả đầu ra

Sau khi chạy, kết quả được lưu tại:

```txt
results/
├── paths/route_summary.json
├── cost_matrices/*.txt
└── figures/path_result.png
```

## Cấu trúc thư mục

```txt
src/environment/   # đọc và biểu diễn obstacle layer, radiation layer, two-layer map
src/costs/         # tính length/risk/energy/total cost
src/lower_level/   # Modified A*, pairwise path, cost matrix builder
src/upper_level/   # GA, ACO, pheromone, heuristic
src/planner/       # ghép lower-level và upper-level
src/visualization/ # vẽ bản đồ, đường đi, hội tụ
```

## Chạy eval.py
```
# Chạy 10 lần với seed tăng dần (42, 43, 44, ...)
python eval.py --config configs/default.yaml --n 10

# Chạy 20 lần với seed ngẫu nhiên hoàn toàn độc lập (đảm bảo không trùng seed nào --> nên dùng khi muốn đánh giá tổng thể, không cần so sánh seed cụ thể)
python eval.py --config configs/default.yaml --n 20 --independent
python eval.py --config configs/scenario_custom_200x200.yaml --n 3 --independent

python eval.py --config configs/factory400_30.yaml --n 5 --independent

python eval.py --config configs/scenario4.yaml --n 2 --independent

python eval.py --config configs/scenario_triangle_300_lowrisk.yaml --n 2 --independent

python eval.py --config configs/scenario_square_400_lowrisk.yaml --n 2 --independent

python eval.py --config configs/mixed500.yaml --n 2 --independent

# Dùng config khác
python eval.py --config configs/scenario_small.yaml --n 15

# Chạy nhiều kịch bản cùng lúc
python eval.py --config configs/scenario7.yaml --n 3 --independent
python eval.py --config configs/scenario_custom_200x200.yaml --n 3 --independent
python eval.py --config configs/scenario_triangle_300_lowrisk.yaml --n 3 --independent
python eval.py --config configs/scenario_square_400_lowrisk.yaml --n 3 --independent
python eval.py --config configs/mixed500.yaml --n 3 --independent

python eval.py --config configs/scenario4.yaml --n 3 --independent
python eval.py --config configs/scenario5.yaml --n 3 --independent
python eval.py --config configs/scenario6.yaml --n 3 --independent
python eval.py --config configs/scenario7.yaml --n 3 --independent

python eval.py --config configs/warehouse3.yaml --n 3 --independent
python eval.py --config configs/warehouse4.yaml --n 3 --independent
```


## Ghi chú quan trọng

- `ri_max` càng nhỏ thì robot càng né nhiều vùng phóng xạ cao.
- `omega_length`, `omega_risk`, `omega_energy` điều chỉnh ưu tiên giữa đường ngắn, an toàn phóng xạ, và ít rẽ.
- Nếu một cặp target không có đường đi hợp lệ, chi phí cặp đó sẽ là `inf`, route chứa cạnh đó sẽ bị loại.
