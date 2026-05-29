Mỗi segment từ w_k → w_{k+1}, độ dài $d_k = |w_{k+1} - w_k|$:

$$R(\text{seg}k) = d_k \cdot \frac{\bar{R}(w_k) + \bar{R}(w{k+1})}{2} \cdot \frac{a}{v}$$

$$\text{risk}(\text{seg}k) = d_k \cdot \frac{(1 - S(w_k)) + (1 - S(w{k+1}))}{2}$$

---

Thêm điểm giữa $m_k = \frac{w_k + w_{k+1}}{2}$:

$$R(\text{seg}k) = \frac{d_k}{6} \cdot \left[\bar{R}(w_k) + 4\bar{R}(m_k) + \bar{R}(w{k+1})\right] \cdot \frac{a}{v}$$

$$\text{risk}(\text{seg}k) = \frac{d_k}{6} \cdot \left[(1 - S(w_k)) + 4(1 - S(m_k)) + (1 - S(w{k+1}))\right]$$

---

Rasterize segment thành các ô lưới ${c_0, c_1, \ldots, c_m}$ bằng thuật toán Bresenham, mỗi ô đóng góp theo độ dài thực tế đoạn đi qua nó $\Delta d_i = |c_{i+1} - c_i|$:

$$R(\text{seg}k) = \sum{i=0}^{m-1} \Delta d_i \cdot \frac{\bar{R}(c_i) + \bar{R}(c_{i+1})}{2} \cdot \frac{a}{v}$$

$$\text{risk}(\text{seg}k) = \sum{i=0}^{m} \Delta d_i \cdot (1 - S(c_i))$$

Với $\Delta d_0$ của $c_0$ tính bằng nửa khoảng cách đến $c_1$ (hoặc dùng trapezoidal hoàn toàn).