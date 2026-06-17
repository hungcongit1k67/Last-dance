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

Đây là phần khác biệt cốt lõi. OR-Tools không có biến $x_{ij}$, không có $u_i$, không có cả hai họ ràng buộc trên. Nó dùng biến successor $\text{next}(i) \in V$ và đặt một ràng buộc CP toàn cục:

$$
\textsf{AllDifferent}\big(\text{next}(0), \text{next}(1), \dots, \text{next}(n-1)\big)
$$

$$
\textsf{NoSubtour / Circuit}\big(\text{next}(\cdot)\big) = \text{true}
$$

Phát biểu toán học của ràng buộc circuit (còn gọi là Hamiltonian-circuit constraint, Caseau–Laburthe): với hàm successor $\sigma(i) = \text{next}(i)$, yêu cầu $\sigma$ là một hoán vị có đúng một quỹ đạo (single cycle) phủ toàn bộ $V$:

$$
\{i,\sigma(i),\sigma^2(i),\dots,\sigma^{n-1}(i)\} = V,\qquad \forall i \in V
$$

Tức là: xuất phát từ bất kỳ node nào, áp dụng $\sigma$ liên tiếp phải duyệt hết $n$ node trước khi quay về điểm đầu. Nếu tồn tại subtour độ dài $k < n$ thì $\sigma^k(i) = i$ với $k<n$ — vi phạm.

Cách thực thi (propagation, không phải bất đẳng thức tuyến tính): solver duy trì các chuỗi đường đi (path chains) từ Start tới End. Mỗi khi gán $\text{next}(i)=j$, nó cập nhật hai đầu mút chuỗi và áp luật cấm: với một chuỗi đang nối từ đầu $h$ đến đuôi $t$ (chưa phủ hết node), cấm gán cung đóng vòng sớm

$$
\text{next}(t) \neq h \quad \text{khi chuỗi } h \rightsquigarrow t \text{ chưa chứa đủ } n \text{ node}
$$

Đây chính là cơ chế "duy trì cây/đường đi từ Start tới End" mà dòng 103 mô tả. Nó tương đương về mặt loại nghiệm với DFJ nhưng được cài bằng lan truyền ràng buộc (constraint propagation) trong CP, nên:

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
(neighborhood operators). Tập hàng xóm là **hợp** của mọi toán tử đang bật,
$N(s)=\bigcup_{o}N_o(s)$, và mỗi bước local search chọn move tốt nhất trên toàn bộ hợp này — tức
**dùng đồng thời cả bộ**, không phải chỉ 2-opt. Mặc định OR-Tools bật cả bộ phù hợp với TSP (các toán
tử cho node tùy chọn / pickup-delivery không kích hoạt vì bài toán không có). Các toán tử áp dụng cho
TSP gồm (xác nhận từ `routing_parameters_pb2.pyi`):

| Toán tử | Phép biến đổi lộ trình |
|---|---|
| **2-opt** (`use_two_opt`) | Đảo ngược một đoạn con của lộ trình — gỡ 2 cung chéo nhau, nối lại cho ngắn hơn |
| **Or-opt** (`use_or_opt`) | Dời một chuỗi 1–3 node liên tiếp sang vị trí khác |
| **Relocate** (`use_relocate`) | Dời 1 node sang vị trí khác trên lộ trình |
| **Exchange** (`use_exchange`) | Hoán đổi vị trí 2 node |
| **Cross / Cross-exchange** (`use_cross`) | Trao đổi đoạn đuôi giữa 2 phần lộ trình |
| **Lin–Kernighan** (`use_lin_kernighan`) | Chuỗi k-opt biến thiên (mạnh, kinh điển cho TSP) |
| **TSP-opt** (`use_tsp_opt`) | Tối ưu lại trọn vẹn một đoạn con bằng giải TSP nhỏ trên DAG |

### 4.0. Nguyên lý chung: mọi toán tử = "gỡ vài cung, nối vài cung"

Tất cả các toán tử dưới đây đều là **move cục bộ**: chúng chỉ thay đổi một số ít cung của lộ trình
và giữ nguyên phần còn lại. Gọi $R$ là **tập cung bị gỡ** và $A$ là **tập cung được nối thêm**.
Vì mọi node bên trong các đoạn không đổi vẫn nối với nhau như cũ, độ thay đổi chi phí (delta) **chỉ
phụ thuộc các cung ở hai biên**:

$$
\boxed{\;\Delta \;=\; \sum_{(i,j)\in A} c_{ij} \;-\; \sum_{(i,j)\in R} c_{ij}\;}
$$

Một move được **chấp nhận khi $\Delta < 0$** (rẻ hơn). Lưu ý hai điều kiện hợp lệ kèm theo:
1. **Bảo toàn chu trình Hamilton** — sau khi nối lại vẫn phải là **một** vòng kín duy nhất phủ hết
   $V$ (chính ràng buộc no-subtour ở mục 2.4). Toán tử nào đảo chiều một đoạn thì trên TSP **bất đối
   xứng** ($c_{ij}\neq c_{ji}$) còn phải cộng thêm chênh lệch chi phí của các cung **bên trong** đoạn
   bị đảo (xem ghi chú ở 4.1).
2. Số cung gỡ = số cung nối, để bậc vào/ra của mỗi node vẫn bằng 1.

Quy ước ký hiệu dùng chung: trên lộ trình, với một node $v$ ta gọi $p=\text{pred}(v)$ là node liền
trước và $q=\text{succ}(v)$ là node liền sau (tức đang có $p\to v\to q$). Các chữ $a,b,c,d,\dots$ ký
hiệu các node tại điểm chèn/cắt.

### 4.1. 2-opt — toán tử quan trọng nhất cho TSP

Cho lộ trình $\dots \to a \to b \to \dots \to c \to d \to \dots$, phép 2-opt đảo ngược đoạn
$b \dots c$, biến hai cung $(a,b)$ và $(c,d)$ thành $(a,c)$ và $(b,d)$. **Độ giảm chi phí** (delta):

$$
\Delta = \big[c_{ac} + c_{bd}\big] - \big[c_{ab} + c_{cd}\big]
$$

Chỉ chấp nhận khi $\Delta < 0$ (làm rẻ hơn). Lặp đến khi không còn cặp cung nào cho $\Delta<0$
⇒ đạt **cực tiểu địa phương 2-opt**.

> **Bất đối xứng (ATSP):** 2-opt đảo ngược chiều đoạn $b\dots c$, nên mọi cung bên trong đoạn cũng
> bị **đảo chiều**. Với chi phí đối xứng ($c_{ij}=c_{ji}$) phần bên trong không đổi nên công thức
> trên đủ dùng. Nhưng `grid.dijk` của bạn **bất đối xứng**, nên delta đầy đủ phải cộng thêm chênh
> lệch chiều của các cung trong đoạn:
> $$
> \Delta = \big[c_{ac}+c_{bd}\big]-\big[c_{ab}+c_{cd}\big] \;+\; \sum_{(u,v)\,\in\, b\dots c}\big(c_{vu}-c_{uv}\big)
> $$
> Đây là lý do trên ATSP các toán tử **không đảo chiều** (Relocate, Or-opt, Exchange) thường rẻ hơn
> để đánh giá so với 2-opt.

### 4.2. Relocate (`use_relocate`) — dời 1 node

Gỡ node $v$ ra khỏi vị trí hiện tại ($p\to v\to q$) rồi chèn vào giữa một cung $(a,b)$ ở chỗ khác
($a\to b$ trở thành $a\to v\to b$).

- **Gỡ:** $R=\{(p,v),\,(v,q),\,(a,b)\}$
- **Nối:** $A=\{(p,q),\,(a,v),\,(v,b)\}$

$$
\Delta = \big[c_{pq}+c_{av}+c_{vb}\big] - \big[c_{pv}+c_{vq}+c_{ab}\big]
$$

Không đảo chiều đoạn nào ⇒ công thức đúng nguyên vẹn cả với chi phí bất đối xứng.

### 4.3. Or-opt (`use_or_opt`) — dời một chuỗi 1–3 node

Tổng quát hóa Relocate: dời cả một **chuỗi liên tiếp** $\langle v_1,\dots,v_L\rangle$ ($L\in\{1,2,3\}$)
đang nằm giữa $p$ và $q$ ($p\to v_1\to\cdots\to v_L\to q$), chèn vào giữa cung $(a,b)$.

- **Gỡ:** $R=\{(p,v_1),\,(v_L,q),\,(a,b)\}$
- **Nối (giữ nguyên chiều chuỗi):** $A=\{(p,q),\,(a,v_1),\,(v_L,b)\}$

$$
\Delta = \big[c_{pq}+c_{a,v_1}+c_{v_L,b}\big] - \big[c_{p,v_1}+c_{v_L,q}+c_{ab}\big]
$$

Các cung **bên trong** chuỗi ($v_1\!\to\!v_2\!\to\!\cdots\!\to\!v_L$) không đổi nên không xuất hiện trong
$\Delta$. Khi $L=1$ thì Or-opt **trùng** Relocate. (OR-Tools còn thử cả phương án chèn chuỗi **đảo
chiều** $\langle v_L,\dots,v_1\rangle$; khi đó thêm số hạng đảo chiều như ở 4.1.)

### 4.4. Exchange (`use_exchange`) — hoán đổi 2 node

Tráo vị trí hai node $v$ (đang ở $p\to v\to q$) và $w$ (đang ở $r\to w\to s$), giả thiết **không kề
nhau**. Sau move: $p\to w\to q$ và $r\to v\to s$.

- **Gỡ:** $R=\{(p,v),(v,q),(r,w),(w,s)\}$
- **Nối:** $A=\{(p,w),(w,q),(r,v),(v,s)\}$

$$
\Delta = \big[c_{pw}+c_{wq}+c_{rv}+c_{vs}\big]-\big[c_{pv}+c_{vq}+c_{rw}+c_{ws}\big]
$$

(Trường hợp $v,w$ **kề nhau**, ví dụ $q=w$, một số cung trùng nhau và phải rút gọn lại — đó là một
case biên solver xử lý riêng.)

### 4.5. Cross-exchange (`use_cross` / `use_cross_exchange`) — trao đổi 2 đoạn

Cắt hai **đoạn con** rồi tráo chỗ cho nhau. Cho đoạn $S_1=\langle b\dots e\rangle$ nằm sau $a$ và
trước $f$ ($a\to[b\dots e]\to f$), và đoạn $S_2=\langle g\dots h\rangle$ nằm sau $c$ và trước $i$
($c\to[g\dots h]\to i$). Sau move: $a\to[g\dots h]\to f$ và $c\to[b\dots e]\to i$.

- **Gỡ:** $R=\{(a,b),(e,f),(c,g),(h,i)\}$
- **Nối:** $A=\{(a,g),(h,f),(c,b),(e,i)\}$

$$
\Delta = \big[c_{ag}+c_{hf}+c_{cb}+c_{ei}\big]-\big[c_{ab}+c_{ef}+c_{cg}+c_{hi}\big]
$$

Bên trong mỗi đoạn giữ nguyên (không đảo) nên không vào $\Delta$. Đây là toán tử "đa cung" — đụng tới
4 cung biên cùng lúc, mạnh hơn Relocate/Exchange nhưng không gian hàng xóm lớn hơn.

### 4.6. Lin–Kernighan (`use_lin_kernighan`) — k-opt độ sâu biến thiên

Không cố định $k$. LK xây một **dãy trao đổi cung tuần tự**: lần lượt gỡ cung $x_1,x_2,\dots$ và nối
cung $y_1,y_2,\dots$ xen kẽ, theo dõi **độ lợi tích lũy**

$$
G_k=\sum_{m=1}^{k}\big(c_{x_m}-c_{y_m}\big)
$$

LK tiếp tục kéo dài chuỗi chừng nào **độ lợi riêng phần dương** ($g_m=c_{x_m}-c_{y_m}$, tổng cộng
dồn $>0$), và tại mỗi bước thử "đóng" chuỗi thành một tour hợp lệ. Move cuối cùng được chọn là độ sâu
$k^\*$ cho độ lợi lớn nhất:

$$
k^\*=\arg\max_{k}\;G_k,\qquad \text{nhận nếu } G_{k^\*}>0\;(\Leftrightarrow \Delta=-G_{k^\*}<0)
$$

Bản chất LK = tổng quát của 2-opt ($k=2$) và 3-opt ($k=3$) với $k$ tự thích nghi ⇒ thoát được nhiều
cực tiểu địa phương mà 2-opt một mình bị kẹt.

### 4.7. TSP-opt (`use_tsp_opt`) — tối ưu lại trọn một đoạn

Chọn một đoạn con với **hai đầu mút cố định** $a$ (đầu) và $z$ (cuối), tập node bên trong $U$. Toán tử
giải **chính xác** bài toán đường Hamilton ngắn nhất đi từ $a$ qua tất cả node của $U$ tới $z$ (một TSP
con nhỏ, cài bằng quy hoạch động trên DAG — Held–Karp):

$$
\pi^\*=\arg\min_{\pi\in\Pi(U)}\Big[c_{a,\pi_1}+\sum_{t=1}^{|U|-1}c_{\pi_t,\pi_{t+1}}+c_{\pi_{|U|},z}\Big]
$$

với $\Pi(U)$ là tập mọi hoán vị của $U$. Delta là chênh lệch giữa thứ tự tối ưu $\pi^\*$ và thứ tự hiện
tại của đoạn:

$$
\Delta = c(\pi^\*) - c(\text{đoạn hiện tại})\;\le\;0
$$

Vì là tối ưu cục bộ **chính xác** trên đoạn, $\Delta$ luôn $\le 0$. Chỉ áp dụng cho đoạn ngắn (độ
phức tạp Held–Karp $O(2^{|U|}|U|^2)$ tăng theo hàm mũ theo độ dài đoạn).

### 4.8. Cơ chế greedy descent

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