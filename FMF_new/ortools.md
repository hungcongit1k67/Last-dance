# OR-Tools giải bài toán TSP — Công thức toán học & thuật toán

> Tài liệu này mô tả **chính xác cách thư viện OR-Tools (phiên bản 9.14.6206, đường dẫn
> `C:\Users\PC\anaconda3\Lib\site-packages\ortools`) được dùng trong file
> [`ADR_main_ortools.py`](ADR_main_ortools.py) để giải bài toán Người bán hàng du lịch (TSP)**
> trên ma trận chi phí `grid.dijk` sinh ra từ pha WP-FMF.
>
> Cấu hình thực tế trong code (hàm `solve_tsp_ortools`):
> - **First solution strategy:** `FirstSolutionStrategy.PATH_CHEAPEST_ARC`
> - **Local search metaheuristic:** `LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH`
> - **1 phương tiện, 1 depot (node 0), chu trình kín (về điểm xuất phát)**

---

## 1. Tổng quan kiến trúc: OR-Tools KHÔNG giải TSP bằng một công thức đóng

Điều quan trọng cần hiểu trước tiên: TSP là bài toán **NP-hard**, nên OR-Tools **không** dùng
một thuật toán tối ưu chính xác (exact) cho trường hợp tổng quát. Thay vào đó, module
`ortools.constraint_solver` (gọi qua `pywrapcp`) mô hình hóa TSP như một **bài toán định tuyến
phương tiện (Vehicle Routing Problem - VRP)** rồi giải bằng quy trình **2 giai đoạn**:

```
┌─────────────────────────────────────────────────────────────────┐
│  GIAI ĐOẠN A — Mô hình hóa (Constraint Programming)              │
│     RoutingModel + RoutingIndexManager                            │
│     → biến quyết định, ràng buộc, hàm mục tiêu                   │
├─────────────────────────────────────────────────────────────────┤
│  GIAI ĐOẠN B — Tìm kiếm nghiệm                                   │
│     B1. First Solution Heuristic  (PATH_CHEAPEST_ARC)            │
│         → dựng 1 lời giải khả thi ban đầu (nhanh, tham lam)       │
│     B2. Local Search + Metaheuristic (GUIDED_LOCAL_SEARCH)       │
│         → cải thiện lặp đi lặp lại, thoát cực tiểu địa phương     │
└─────────────────────────────────────────────────────────────────┘
```

TSP chỉ là trường hợp đặc biệt của VRP với **1 phương tiện** — đúng như code khai báo:
```python
manager = pywrapcp.RoutingIndexManager(n, 1, 0)   # n node, 1 vehicle, depot = node 0
routing = pywrapcp.RoutingModel(manager)
```

---

## 2. Mô hình toán học của TSP trong OR-Tools

### 2.1. Dữ liệu đầu vào

- Tập đỉnh (checkpoint): $V = \{0, 1, 2, \dots, n-1\}$, với node $0$ là depot (điểm xuất phát/kết thúc).
- Ma trận chi phí $C = [c_{ij}]$, trong đó $c_{ij}$ là chi phí đi từ node $i$ sang node $j$.
  Trong dự án này $c_{ij}$ **không phải khoảng cách Euclid** mà là chi phí tổng hợp có trọng số
  do pha WP-FMF tính ra (`grid.dijk`):

$$
c_{ij} = w_1 \cdot \text{length}(P_{ij}) + w_2 \cdot R(P_{ij}) + w_3 \cdot \text{risk}(P_{ij})
$$

  với $w_1 + w_2 + w_3 = 1$ (chiều dài / phóng xạ / rủi ro va chạm).

### 2.2. Biến quyết định — mô hình "successor" (NextVar)

OR-Tools **không** dùng biến nhị phân $x_{ij}$ như công thức MILP cổ điển. Thay vào đó nó dùng
biến **người kế tiếp** (successor variable), một cho mỗi node:

$$
\text{next}(i) = j \quad \Longleftrightarrow \quad \text{trên lộ trình, ngay sau node } i \text{ là node } j
$$

Trong code, biến này chính là `routing.NextVar(index)` được duyệt khi trích xuất lộ trình:
```python
index = routing.Start(0)
while not routing.IsEnd(index):
    route.append(manager.IndexToNode(index))
    index = solution.Value(routing.NextVar(index))   # đọc next(i)
```

Miền giá trị: $\text{next}(i) \in V \setminus \{i\}$.

### 2.3. Hàm mục tiêu

Tối thiểu hóa tổng chi phí của tất cả các cung được chọn (chu trình kín đi qua tất cả node đúng 1 lần):

$$
\min \sum_{i \in V} c_{\,i,\ \text{next}(i)}
$$

Trong code, chi phí mỗi cung được nạp qua **arc cost evaluator**:
```python
def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node   = manager.IndexToNode(to_index)
    return int_matrix[from_node][to_node]

transit_cb_idx = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_cb_idx)   # gán c_ij cho hàm mục tiêu
```

### 2.4. Ràng buộc

| Ràng buộc | Ý nghĩa | Cách OR-Tools thực thi |
|---|---|---|
| **Mỗi node đúng 1 successor** | `next(i)` là một phép gán hàm (mỗi node ra đúng 1 cung) | Biến `NextVar(i)` nhận đúng 1 giá trị |
| **AllDifferent** | Không 2 node nào cùng trỏ tới 1 successor (mỗi node có đúng 1 cung vào) | Ràng buộc `AllDifferent` trên tập `next(·)` |
| **No-subtour / liên thông** | Phải là **một** chu trình duy nhất, không tách thành nhiều vòng con | Ràng buộc cấu trúc đường đi (path/circuit) của Routing layer — tự động loại bỏ subtour bằng cách duy trì cây đường đi từ `Start` tới `End`, **thay cho** ràng buộc MTZ hay cắt subtour của MILP |
| **Bắt đầu/kết thúc tại depot** | Lộ trình xuất phát và quay về node 0 | `routing.Start(0)` … `routing.IsEnd(index)` |

> **So sánh với công thức MILP cổ điển (Miller–Tucker–Zemlin):** mô hình MTZ thêm biến phụ
> $u_i$ và ràng buộc $u_i - u_j + n \cdot x_{ij} \le n-1$ để chống subtour. OR-Tools **không
> dùng** cách này; nó dùng mô hình ràng buộc đường đi (CP routing) nên cấu trúc "một chu trình
> liên thông" được bảo đảm bởi chính cách biểu diễn `next(·)` + lan truyền ràng buộc, không cần
> biến thứ tự.

### 2.5. Chuẩn hóa chi phí về số nguyên

OR-Tools routing solver **yêu cầu chi phí là số nguyên**. Code nhân ma trận float với
`distance_scale = 1000` rồi làm tròn (hàm `_build_int_distance_matrix`):

$$
\tilde{c}_{ij} = \big\lfloor c_{ij} \cdot \text{scale} + 0.5 \big\rfloor, \qquad \text{scale} = 1000
$$

Ví dụ: $1.0 \to 1000$, $\sqrt{2} \approx 1.41421 \to 1414$. Giá trị $\infty$ hoặc `NaN` được thay bằng $10^9$.
Việc nhân 1000 giúp **giữ lại ~3 chữ số thập phân** độ chính xác sau khi làm tròn.

---

## 3. Giai đoạn B1 — Lời giải ban đầu: `PATH_CHEAPEST_ARC`

Đây là một **heuristic dựng đường tham lam (path-addition / nearest-neighbour mở rộng)**.
Trích nguyên văn mô tả trong file `routing_enums_pb2.pyi` của thư viện:

> *"Starting from a route 'start' node, connect it to the node which produces the cheapest
> route segment, then extend the route by iterating on the last node added to the route."*

### Thuật toán (giả mã)

```
1.  route ← [depot 0];   current ← 0;   visited ← {0}
2.  while còn node chưa thăm:
3.      j* ← argmin_{ j ∉ visited }  c[current][j]      # cung rẻ nhất từ node hiện tại
4.      route.append(j*)
5.      visited.add(j*)
6.      current ← j*
7.  đóng chu trình: nối current → depot 0
```

Công thức chọn node kế tiếp tại mỗi bước:

$$
j^{*} = \arg\min_{\,j \notin \text{visited}} \; \tilde{c}_{\,\text{current},\,j}
$$

- **Độ phức tạp:** $O(n^2)$ — rất nhanh.
- **Vai trò:** chỉ tạo *một điểm xuất phát khả thi*, KHÔNG tối ưu. Chất lượng lời giải này
  thường còn cách tối ưu 15–25%, sẽ được giai đoạn B2 mài giũa.

> OR-Tools còn hỗ trợ nhiều first-solution khác (cùng có trong thư viện): `SAVINGS`
> (Clarke–Wright), `CHRISTOFIDES` (xấp xỉ 3/2 cho TSP metric), `PARALLEL_CHEAPEST_INSERTION`,
> `SWEEP`… nhưng code này cố định dùng `PATH_CHEAPEST_ARC`.

---

## 4. Giai đoạn B2 — Tìm kiếm cục bộ (Local Search)

Sau khi có lời giải ban đầu $s_0$, solver lặp lại việc **biến đổi nhẹ lộ trình** (di chuyển sang
"hàng xóm" $s' \in N(s)$) để giảm chi phí. Tập hàng xóm $N(s)$ được sinh bởi các **toán tử lân cận**
(neighborhood operators). Trong OR-Tools các toán tử áp dụng cho TSP gồm (xác nhận từ
`routing_parameters_pb2.pyi`):

| Toán tử | Phép biến đổi lộ trình |
|---|---|
| **2-opt** (`use_two_opt`) | Đảo ngược một đoạn con của lộ trình — gỡ 2 cung chéo nhau, nối lại cho ngắn hơn |
| **Or-opt** (`use_or_opt`) | Dời một chuỗi 1–3 node liên tiếp sang vị trí khác |
| **Relocate** (`use_relocate`) | Dời 1 node sang vị trí khác trên lộ trình |
| **Exchange** (`use_exchange`) | Hoán đổi vị trí 2 node |
| **Cross / Cross-exchange** (`use_cross`) | Trao đổi đoạn đuôi giữa 2 phần lộ trình |
| **Lin–Kernighan** (`use_lin_kernighan`) | Chuỗi k-opt biến thiên (mạnh, kinh điển cho TSP) |
| **TSP-opt** (`use_tsp_opt`) | Tối ưu lại trọn vẹn một đoạn con bằng giải TSP nhỏ trên DAG |

### 4.1. 2-opt — toán tử quan trọng nhất cho TSP

Cho lộ trình $\dots \to a \to b \to \dots \to c \to d \to \dots$, phép 2-opt đảo ngược đoạn
$b \dots c$, biến hai cung $(a,b)$ và $(c,d)$ thành $(a,c)$ và $(b,d)$. **Độ giảm chi phí** (delta):

$$
\Delta = \big[c_{ac} + c_{bd}\big] - \big[c_{ab} + c_{cd}\big]
$$

Chỉ chấp nhận khi $\Delta < 0$ (làm rẻ hơn). Lặp đến khi không còn cặp cung nào cho $\Delta<0$
⇒ đạt **cực tiểu địa phương 2-opt**.

### 4.2. Cơ chế greedy descent

Bước cơ bản của local search (chính là `GREEDY_DESCENT`):

$$
s_{t+1} = \arg\min_{\,s' \in N(s_t)} \; f(s') \quad\text{chỉ nhận nếu } f(s_{t+1}) < f(s_t)
$$

Nhược điểm: dừng lại ngay tại cực tiểu **địa phương** đầu tiên. Để thoát ra cần metaheuristic
ở mục 5.

---

## 5. Metaheuristic: `GUIDED_LOCAL_SEARCH` (GLS)

Đây là phần "thông minh" giúp thoát cực tiểu địa phương. Mô tả của thư viện:

> *"Uses guided local search to escape local minima; this is generally the most efficient
> metaheuristic for vehicle routing."*

### 5.1. Ý tưởng

Khi local search bị kẹt tại cực tiểu địa phương, GLS **không** nhảy ngẫu nhiên (như Simulated
Annealing) mà **sửa lại hàm mục tiêu**: nó "phạt" những cung (đặc trưng) xuất hiện trong nghiệm
kẹt và có chi phí cao, khiến chúng kém hấp dẫn ở vòng tìm kiếm sau ⇒ đẩy tìm kiếm sang vùng khác.

### 5.2. Định nghĩa đặc trưng và hàm mục tiêu tăng cường

- **Đặc trưng (feature):** mỗi cung $(i,j)$ có thể nằm trong lộ trình. Chỉ thị:

$$
I_{ij}(s) = \begin{cases} 1 & \text{nếu cung } (i,j) \in s \\ 0 & \text{ngược lại}\end{cases}
$$

- **Chi phí đặc trưng:** $c_{ij}$ (chính là chi phí cung).
- **Số đếm phạt:** $p_{ij}$ — số lần cung $(i,j)$ đã bị phạt (khởi tạo $=0$).

GLS tối thiểu hóa **hàm mục tiêu tăng cường** thay vì $f$ gốc:

$$
\boxed{\;h(s) = f(s) + \lambda \sum_{(i,j)} p_{ij}\, I_{ij}(s)\;}
$$

trong đó:
- $f(s) = \sum_i c_{i,\text{next}(i)}$ — chi phí thực của lộ trình,
- $\lambda > 0$ — hệ số phạt, chính là tham số
  `guided_local_search_lambda_coefficient` trong thư viện (mô tả: *"Lambda coefficient used to
  penalize arc costs when GUIDED_LOCAL_SEARCH is used. Must be positive."*).
  OR-Tools tự động đặt $\lambda = \alpha \cdot \dfrac{f(s_{\text{local opt}})}{\text{số cung}}$
  với $\alpha$ = hệ số cấu hình, để mức phạt cân xứng với quy mô chi phí.

### 5.3. Quy tắc chọn cung để phạt — tiện ích (utility)

Tại mỗi cực tiểu địa phương, GLS phạt cung có **tiện ích phạt** lớn nhất:

$$
\text{util}(i,j) = I_{ij}(s) \cdot \frac{c_{ij}}{1 + p_{ij}}
$$

$$
(i^*, j^*) = \arg\max_{(i,j)} \; \text{util}(i,j), \qquad p_{i^*j^*} \mathrel{+}= 1
$$

Diễn giải: ưu tiên phạt cung **đắt** ($c_{ij}$ lớn) nhưng **chưa bị phạt nhiều** (mẫu số
$1+p_{ij}$ chống phạt mãi một cung). Sau khi tăng $p_{i^*j^*}$, local search lại chạy trên $h(s)$
mới ⇒ thoát khỏi vùng kẹt.

### 5.4. Vòng lặp GLS (giả mã)

```
s ← lời giải ban đầu (PATH_CHEAPEST_ARC)
s* ← s                                  # nghiệm tốt nhất từng thấy
p[i][j] ← 0 với mọi cung
while chưa hết time_limit:
    s ← local_search(s, h)              # greedy descent trên hàm tăng cường h (2-opt, Or-opt,…)
    if f(s) < f(s*):  s* ← s            # cập nhật nghiệm tốt nhất theo CHI PHÍ THỰC f
    # tại cực tiểu địa phương của h: phạt cung có util lớn nhất
    chọn (i*,j*) = argmax util(i,j);  p[i*][j*] += 1
return s*
```

Lưu ý: GLS luôn ghi nhớ nghiệm tốt nhất theo **chi phí thực $f$**, dù quá trình tìm kiếm
được dẫn dắt bởi $h$. (Thư viện còn có tùy chọn
`guided_local_search_reset_penalties_on_new_best_solution` để reset $p_{ij}$ về 0 mỗi khi tìm
được nghiệm tốt hơn — khởi động lại một pha greedy descent.)

### 5.5. Điều kiện dừng

Trong code, điều kiện dừng là **giới hạn thời gian**:
```python
params.time_limit.seconds = time_limit_sec   # mặc định CONFIG = 10 giây
```
Khi hết thời gian, solver trả về $s^*$ — lời giải tốt nhất tìm được (không bảo đảm tối ưu tuyệt
đối, nhưng với TSP cỡ vài chục–vài trăm node thường rất gần tối ưu).

---

## 6. Tóm tắt toàn bộ pipeline trong `solve_tsp_ortools`

```
              grid.dijk (float, n×n)
                      │
                      ▼  _build_int_distance_matrix  (×1000, làm tròn)
        ┌──────────────────────────────────────────┐
        │  Ma trận chi phí nguyên  c̃_ij             │
        └──────────────────────────────────────────┘
                      │
                      ▼   Mô hình CP-Routing
   RoutingIndexManager(n, 1, 0) + RoutingModel
   biến next(i),  AllDifferent,  no-subtour,  mục tiêu Σ c̃[i,next(i)]
                      │
        ┌─────────────┴──────────────┐
        ▼                            ▼
   B1. PATH_CHEAPEST_ARC        B2. GUIDED_LOCAL_SEARCH
   (dựng nghiệm tham lam)       (2-opt/Or-opt/relocate… + phạt cung)
        └─────────────┬──────────────┘
                      ▼   (dừng khi hết time_limit_sec)
        route = [0, …]  +  real_cost = Σ c_ij (float, KHÔNG scale)
```

Chi phí trả về cho người dùng được tính lại trên ma trận float gốc (hàm `_route_cost_float`),
tức **bỏ scale 1000**, để báo cáo đúng đơn vị chi phí WP-FMF:

$$
\text{real\_cost} = \sum_{k=0}^{m-1} c_{\,r_k,\ r_{(k+1)\bmod m}}
$$

với $r = [r_0, r_1, \dots, r_{m-1}]$ là thứ tự thăm và $r_{(k+1)\bmod m}$ đảm bảo cộng cả cung
đóng chu trình quay về điểm đầu.

---

## 7. Bảng đối chiếu công thức ↔ dòng code

| Khái niệm toán học | Vị trí trong `ADR_main_ortools.py` |
|---|---|
| $\tilde c_{ij} = \lfloor c_{ij}\cdot 1000 + 0.5\rfloor$ | `_build_int_distance_matrix`, dòng 111–129 |
| Mô hình 1 phương tiện, depot 0 | `RoutingIndexManager(n, 1, 0)`, dòng 154 |
| Arc cost evaluator $c_{i,\text{next}(i)}$ | `distance_callback` + `SetArcCostEvaluatorOfAllVehicles`, dòng 157–163 |
| First solution `PATH_CHEAPEST_ARC` | dòng 167 |
| Metaheuristic `GUIDED_LOCAL_SEARCH` | dòng 169 |
| Điều kiện dừng theo thời gian | `params.time_limit.seconds`, dòng 172 |
| Trích lộ trình qua `next(i)` | vòng `while not routing.IsEnd(index)`, dòng 179–182 |
| $\text{real\_cost}=\sum c_{r_k,r_{k+1}}$ (chu trình) | `_route_cost_float`, dòng 132–140 |

---

## 8. Ghi chú quan trọng

1. **Không có công thức nghiệm đóng.** OR-Tools là solver **heuristic/metaheuristic** —
   nó trả về nghiệm *rất tốt* trong giới hạn thời gian, không phải nghiệm tối ưu được
   chứng minh (trừ khi gặp `ROUTING_OPTIMAL`). Với cấu hình `time_limit_sec=10` và
   GLS, nghiệm thường nằm trong vài % so với tối ưu.

2. **Phần lõi thuật toán nằm trong C++.** File Python (`pywrapcp.py`,
   `routing_enums_pb2.py`) chỉ là lớp bọc (SWIG wrapper) gọi xuống thư viện biên dịch
   `_pywrapcp.pyd`. Các công thức 2-opt/GLS ở trên là mô tả thuật toán mà phần C++ thực thi,
   được suy ra từ tài liệu và docstring trong các file `.pyi` của chính bản cài 9.14.6206.

3. **Chi phí bất đối xứng được hỗ trợ.** Vì $c_{ij}$ đến từ WP-FMF (đường đi qua chướng ngại,
   phóng xạ…) nên có thể $c_{ij} \ne c_{ji}$. Mô hình routing của OR-Tools xử lý tốt TSP
   **bất đối xứng (ATSP)** — không cần giả thiết đối xứng hay bất đẳng thức tam giác.

---

*Tài liệu sinh từ phân tích trực tiếp source `ADR_main_ortools.py` và bản cài OR-Tools
9.14.6206 tại `C:\Users\PC\anaconda3\Lib\site-packages\ortools\constraint_solver`.*