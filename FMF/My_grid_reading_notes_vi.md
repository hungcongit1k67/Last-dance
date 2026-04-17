# Ghi chú đọc `My_grid.py`

Mình đã tạo một bản có comment chi tiết trong file `My_grid_commented_vi.py`.

## Cách đọc nhanh cấu trúc file

- `GridMap`: lớp trung tâm chứa toàn bộ map, goal, obstacle, wavefront, graph và các hàm vẽ.
- `buildForest()`: pha Fast Marching cơ bản, lan từ nhiều goal cùng lúc.
- `buildForestAdvanced()`: pha “firework” nâng cao, cho vùng của từng goal lan tiếp để tạo thêm điểm chạm/cạnh.
- `twoPointTracing()`: dựng đường đi chi tiết giữa từng cặp goal từ các điểm giao.
- `dijkstra()`: chạy shortest path trên đồ thị goal-goal.
- `getPath(sol)`: ghép các đoạn đường theo thứ tự thăm goal trong lời giải TSP.

## Những chỗ nên lưu ý khi đọc

1. File này có dấu hiệu là code nghiên cứu/experimental, chưa dọn sạch hoàn toàn.
2. Ở phần cuối có các hàm kiểu cũ như `getDis()` dùng `validPos`, `Gbfs`, `Gtrace`, nhưng trong file hiện tại:
   - không thấy định nghĩa `validPos()` (chỉ thấy `validpos()`),
   - không thấy khởi tạo `Gbfs`, `Gtrace`.
3. `nearestFree()` có dòng `d[int(x)][int(y)] = 0`, nhiều khả năng đáng ra phải là đánh dấu đã thăm bằng `1`.
4. Một số comment gốc trong code không khớp hoàn toàn với lời gọi hiện tại, ví dụ có chỗ ghi `# 8-direction` nhưng hàm đang truyền `4`.

## Tệp đã tạo

- `My_grid_commented_vi.py`: bản code có chèn comment tiếng Việt rất dày, bám theo gần như từng dòng có ý nghĩa.
