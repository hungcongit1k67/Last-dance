# Ví dụ chi tiết: OR-Tools giải TSP với ma trận chi phí 5×5

> Tài liệu này chạy tay **từng bước** thuật toán mà
> [`ADR_main_ortools.py`](ADR_main_ortools.py) dùng để giải TSP:
> `PATH_CHEAPEST_ARC` (lời giải ban đầu) → **2-opt local search** → `GUIDED_LOCAL_SEARCH`
> (phạt cung để thoát cực tiểu địa phương).
>
> Xem phần lý thuyết & công thức tổng quát ở [`ortools.md`](ortools.md). File này tập trung vào
> một **ví dụ số cụ thể** để thấy rõ cơ chế.

---

## 0. Bài toán: 5 checkpoint

Giả sử pha WP-FMF trả về ma trận chi phí $C = [c_{ij}]$ giữa **5 checkpoint** (node `0`–`4`),
trong đó node `0` là depot (điểm xuất phát/kết thúc). Ma trận đối xứng ($c_{ij}=c_{ji}$) để dễ
theo dõi — *(OR-Tools cũng xử lý được ma trận bất đối xứng, xem ghi chú cuối bài)*:

| $c_{ij}$ | **0** | **1** | **2** | **3** | **4** |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **0** | 0  | 10 | 15 | 20 | 12 |
| **1** | 10 | 0  | 35 | 25 | 18 |
| **2** | 15 | 35 | 0  | 30 | 22 |
| **3** | 20 | 25 | 30 | 0  | 16 |
| **4** | 12 | 18 | 22 | 16 | 0  |

**Mục tiêu:** tìm chu trình kín xuất phát từ `0`, đi qua cả 5 node đúng một lần, quay về `0`,
với tổng chi phí nhỏ nhất:

$$
\min_{\text{hoán vị } r} \;\; \sum_{k=0}^{4} c_{\,r_k,\ r_{(k+1)\bmod 5}}, \qquad r_0 = 0
$$

> **Tham chiếu trước (để kiểm chứng):** duyệt toàn bộ $(5-1)!/2 = 12$ chu trình khác nhau, nghiệm
> tối ưu là **`0→1→3→4→2→0` với chi phí 88**. Ta sẽ xem OR-Tools tự tìm ra nó như thế nào.

---

## 1. Bước chuẩn bị — Scale ma trận về số nguyên

OR-Tools routing yêu cầu chi phí nguyên. Hàm `_build_int_distance_matrix` nhân
`distance_scale = 1000` rồi làm tròn:

$$
\tilde{c}_{ij} = \big\lfloor c_{ij}\cdot 1000 + 0.5\big\rfloor
$$

Ở đây các giá trị đã là số nguyên nên chỉ đơn giản $\tilde c_{ij} = 1000\cdot c_{ij}$
(ví dụ $10 \to 10000$, $35 \to 35000$). **Tỉ lệ giữa các cung không đổi**, nên để dễ đọc, từ đây
ta làm việc trực tiếp trên giá trị gốc (chia lại 1000); thứ tự so sánh hoàn toàn tương đương.

---

## 2. Mô hình hóa (Constraint Programming)

```python
manager = pywrapcp.RoutingIndexManager(5, 1, 0)   # 5 node, 1 vehicle, depot = 0
routing  = pywrapcp.RoutingModel(manager)
```

- Biến quyết định: `next(i)` = node đi ngay sau `i`, với $i=0,\dots,4$.
- Ràng buộc: mỗi node 1 successor + `AllDifferent(next)` + một chu trình liên thông (chống subtour).
- Hàm mục tiêu: $\sum_i c_{i,\text{next}(i)}$ (nạp qua `distance_callback`).

---

## 3. GIAI ĐOẠN B1 — `PATH_CHEAPEST_ARC` (dựng lời giải ban đầu)

Quy tắc: bắt đầu ở depot, mỗi bước **nối node hiện tại tới node chưa thăm rẻ nhất**:

$$
j^{*} = \arg\min_{\,j \notin \text{visited}} \; c_{\,\text{current},\,j}
$$

### Diễn tiến từng bước

| Bước | `current` | Các cung tới node chưa thăm | Chọn $j^*$ (rẻ nhất) | Lộ trình tạm |
|:---:|:---:|---|:---:|---|
| 1 | **0** | 0→1=**10**, 0→2=15, 0→3=20, 0→4=12 | **1** (10) | `0→1` |
| 2 | **1** | 1→2=35, 1→3=25, 1→4=**18** | **4** (18) | `0→1→4` |
| 3 | **4** | 4→2=22, 4→3=**16** | **3** (16) | `0→1→4→3` |
| 4 | **3** | 3→2=**30** (chỉ còn 2) | **2** (30) | `0→1→4→3→2` |
| 5 | — | đóng chu trình: 2→0 = 15 | — | `0→1→4→3→2→0` |

### Lời giải ban đầu

$$
s_0 = (0 \to 1 \to 4 \to 3 \to 2 \to 0)
$$

$$
f(s_0) = c_{01}+c_{14}+c_{43}+c_{32}+c_{20} = 10+18+16+30+15 = \mathbf{89}
$$

> Lưu ý: heuristic tham lam này **chưa tối ưu** (89 > 88). Cung `3→2 = 30` rất đắt bị "kẹt" vào
> cuối vì lúc đó không còn lựa chọn. Giai đoạn local search sẽ sửa.

---

## 4. GIAI ĐOẠN B2a — Local Search bằng 2-opt

Toán tử **2-opt** gỡ 2 cung rồi nối lại bằng cách đảo ngược đoạn giữa. Với hai cung
$(a,b)$ và $(c,d)$ xuất hiện theo thứ tự `…a→b…c→d…`, thay bằng $(a,c)$ và $(b,d)$:

$$
\Delta = \big[c_{ac} + c_{bd}\big] - \big[c_{ab} + c_{cd}\big], \qquad \text{nhận nếu } \Delta < 0
$$

Lộ trình hiện tại $s_0 = 0\!\to\!1\!\to\!4\!\to\!3\!\to\!2\!\to\!0$ có 5 cung:
$(0,1),(1,4),(4,3),(3,2),(2,0)$. Xét mọi cặp cung **không kề nhau**:

| Cặp cung gỡ | $(a,c)+(b,d)$ thêm vào | $\Delta = (c_{ac}+c_{bd})-(c_{ab}+c_{cd})$ | Kết quả |
|---|---|---|:---:|
| $(0,1)$ & $(4,3)$ | $(0,4)+(1,3)$ | $(12+25)-(10+16)=37-26$ | $+11$ ✗ |
| $(0,1)$ & $(3,2)$ | $(0,3)+(1,2)$ | $(20+35)-(10+30)=55-40$ | $+15$ ✗ |
| $(1,4)$ & $(3,2)$ | $(1,3)+(4,2)$ | $(25+22)-(18+30)=47-48$ | $\mathbf{-1}$ ✓ |
| $(1,4)$ & $(2,0)$ | $(1,2)+(4,0)$ | $(35+12)-(18+15)=47-33$ | $+14$ ✗ |
| $(4,3)$ & $(2,0)$ | $(4,2)+(3,0)$ | $(22+20)-(16+15)=42-31$ | $+11$ ✗ |

**Có một move cải thiện:** gỡ $(1,4)$ và $(3,2)$, đảo ngược đoạn `4→3` thành `3→4`:

$$
0\to 1\to \underbrace{4\to 3}_{\text{đảo}}\to 2\to 0 \quad\Longrightarrow\quad 0\to 1\to 3\to 4\to 2\to 0
$$

$$
f(s_1) = 89 + (-1) = \mathbf{88}
$$

Kiểm chứng trực tiếp: $c_{01}+c_{13}+c_{34}+c_{42}+c_{20} = 10+25+16+22+15 = 88$. ✓

---

## 5. Kiểm tra cực tiểu địa phương 2-opt

Chạy lại 2-opt trên $s_1 = 0\!\to\!1\!\to\!3\!\to\!4\!\to\!2\!\to\!0$, cung
$(0,1),(1,3),(3,4),(4,2),(2,0)$:

| Cặp cung gỡ | Thêm vào | $\Delta$ | |
|---|---|:---:|:---:|
| $(0,1)$ & $(3,4)$ | $(0,3)+(1,4)$ | $(20+18)-(10+16)=+12$ | ✗ |
| $(0,1)$ & $(4,2)$ | $(0,4)+(1,2)$ | $(12+35)-(10+22)=+15$ | ✗ |
| $(1,3)$ & $(4,2)$ | $(1,4)+(3,2)$ | $(18+30)-(25+22)=+1$ | ✗ |
| $(1,3)$ & $(2,0)$ | $(1,2)+(3,0)$ | $(35+20)-(25+15)=+15$ | ✗ |
| $(3,4)$ & $(2,0)$ | $(3,2)+(4,0)$ | $(30+12)-(16+15)=+11$ | ✗ |

**Không còn move nào $\Delta < 0$** ⇒ $s_1$ là **cực tiểu địa phương 2-opt**, chi phí **88**.
(Tình cờ ở ví dụ nhỏ này nó cũng chính là tối ưu toàn cục — với bài toán lớn hơn 2-opt thường
kẹt ở cực tiểu cục bộ *cao hơn* tối ưu, và đó là lúc cần GLS ở bước sau.)

---

## 6. GIAI ĐOẠN B2b — `GUIDED_LOCAL_SEARCH` (thoát cực tiểu địa phương)

GLS không nhảy ngẫu nhiên; nó **phạt cung** để bóp méo hàm mục tiêu, buộc local search rời khỏi
vùng đang kẹt. Hàm mục tiêu tăng cường:

$$
h(s) = f(s) + \lambda \sum_{(i,j)} p_{ij}\, I_{ij}(s)
$$

với $p_{ij}$ = số lần cung $(i,j)$ bị phạt (khởi tạo 0), $I_{ij}(s)=1$ nếu $s$ dùng cung đó.

### 6.1. Chọn cung để phạt (utility lớn nhất)

Tại cực tiểu $s_1$ (chi phí 88), với các cung đang dùng và $p_{ij}=0$:

$$
\text{util}(i,j) = I_{ij}(s_1)\cdot\frac{c_{ij}}{1+p_{ij}}
$$

| Cung trong $s_1$ | $c_{ij}$ | $p_{ij}$ | $\text{util} = c_{ij}/(1+p_{ij})$ |
|:---:|:---:|:---:|:---:|
| (0,1) | 10 | 0 | 10 |
| **(1,3)** | **25** | 0 | **25** ← lớn nhất |
| (3,4) | 16 | 0 | 16 |
| (4,2) | 22 | 0 | 22 |
| (2,0) | 15 | 0 | 15 |

⇒ Phạt cung **(1,3)**: đặt $p_{13} \mathrel{+}= 1 \Rightarrow p_{13}=1$.

### 6.2. Hàm mục tiêu tăng cường sau khi phạt

Lấy $\lambda = 5$ (minh họa; OR-Tools tự đặt $\lambda \approx \alpha\cdot f/\text{số cung}$).
Mọi lộ trình **chứa cung (1,3)** giờ bị cộng thêm $\lambda\cdot p_{13}=5$ vào $h$:

| Lộ trình | $f(s)$ | Có cung (1,3)? | $h(s)=f+5\cdot[\text{có}(1,3)]$ |
|---|:---:|:---:|:---:|
| $0\to1\to3\to4\to2\to0$ (cực tiểu cũ) | 88 | ✔ | **93** |
| $0\to1\to4\to3\to2\to0$ | 89 | không | **89** |

→ Trên thước đo $h$, lộ trình cũ (93) giờ **đắt hơn** lộ trình hàng xóm (89). Greedy descent vì
thế **rời khỏi cực tiểu cũ**, đi sang $0\to1\to4\to3\to2\to0$ — đúng mục đích của GLS: thoát ra
để khám phá vùng khác của không gian nghiệm.

### 6.3. Ghi nhớ nghiệm tốt nhất theo chi phí THỰC

Quan trọng: GLS luôn so sánh nghiệm tốt nhất bằng **chi phí thực $f$**, không phải $h$. Khi nó
lang thang sang các lộ trình khác rồi quay lại, nghiệm tốt nhất đã lưu vẫn là:

$$
s^{*} = 0\to1\to3\to4\to2\to0, \qquad f(s^{*}) = \mathbf{88}
$$

Các vòng phạt tiếp theo sẽ tiếp tục thử cung khác (ví dụ kế đến phạt (4,2) vì util=22), nhưng
trong ví dụ 5 node này 88 đã là tối ưu toàn cục nên $s^*$ không đổi cho đến khi **hết
`time_limit_sec`** ⇒ trả về `88`.

> Với bài toán lớn, chính vòng "đi lạc rồi quay lại" này giúp GLS tìm được nghiệm **tốt hơn**
> cực tiểu 2-opt ban đầu — đó là lý do nó được mô tả trong thư viện là *"generally the most
> efficient metaheuristic for vehicle routing"*.

---

## 7. Trích xuất lộ trình từ biến `next(i)`

Khi solver dừng, lời giải $s^*$ tương ứng các biến successor:

$$
\text{next}(0)=1,\;\; \text{next}(1)=3,\;\; \text{next}(3)=4,\;\; \text{next}(4)=2,\;\; \text{next}(2)=0
$$

Code duyệt chuỗi này để dựng lại `route`:

```python
route = []
index = routing.Start(0)          # bắt đầu ở depot 0
while not routing.IsEnd(index):
    route.append(manager.IndexToNode(index))
    index = solution.Value(routing.NextVar(index))
# route = [0, 1, 3, 4, 2]
```

Rồi `_route_cost_float` cộng chi phí **trên ma trận float gốc** (bỏ scale 1000), cộng cả cung
đóng chu trình `2→0`:

$$
\text{real\_cost} = c_{01}+c_{13}+c_{34}+c_{42}+c_{20} = 10+25+16+22+15 = \mathbf{88.0}
$$

---

## 8. Tổng kết các bước

| Giai đoạn | Thao tác | Lộ trình | Chi phí $f$ |
|---|---|---|:---:|
| Input | Ma trận 5×5 (WP-FMF) | — | — |
| Scale | $\times 1000$, làm tròn int | — | — |
| **B1** `PATH_CHEAPEST_ARC` | dựng tham lam từ depot | `0→1→4→3→2→0` | 89 |
| **B2a** 2-opt | gỡ (1,4)&(3,2), đảo đoạn | `0→1→3→4→2→0` | **88** |
| **B2a** kiểm tra | không còn move cải thiện | (cực tiểu cục bộ 2-opt) | 88 |
| **B2b** GLS | phạt cung (1,3), thoát ra, không tìm được nghiệm tốt hơn | giữ $s^*$ | **88** |
| Output | trích `next(i)` + tính real_cost | `[0,1,3,4,2]` | **88.0** |

**Kết quả cuối:** OR-Tools trả về thứ tự thăm `0 → 1 → 3 → 4 → 2 → 0` với chi phí **88** —
cải thiện so với lời giải tham lam ban đầu (89) và trùng nghiệm tối ưu toàn cục.

---

## 9. Ghi chú

1. **Tính tất định.** Với cùng ma trận và tham số, mọi bước trên đều xác định duy nhất (argmin/
   argmax không có ngẫu nhiên) ⇒ chạy lại `ntest` lần luôn ra `88`, **std = 0**. Đây là lý do
   bạn thấy phương sai bằng 0 — xem giải thích trong [`ortools.md`](ortools.md).

2. **Ma trận bất đối xứng (ATSP).** Nếu $c_{ij}\ne c_{ji}$ (thường gặp với chi phí WP-FMF qua
   chướng ngại/phóng xạ), công thức và quy trình **không đổi**, chỉ khác là khi 2-opt đảo ngược
   một đoạn thì các cung *bên trong* đoạn cũng đổi chiều nên phải tính lại chi phí đoạn đó. Mô
   hình routing của OR-Tools xử lý việc này tự động.

3. **Vì sao ví dụ nhỏ chưa thấy GLS "tỏa sáng".** Với 5 node, 2-opt đã chạm tối ưu nên GLS chỉ
   xác nhận lại. Lợi ích thật của GLS xuất hiện khi $n$ lớn: 2-opt kẹt ở cực tiểu cục bộ cao hơn
   tối ưu, và cơ chế phạt cung mới đẩy được tìm kiếm tới nghiệm tốt hơn.