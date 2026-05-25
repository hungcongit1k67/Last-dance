$$\text{risk} = \sum_{i=1}^{N} \max!\left(0,; 1 - \frac{S(p_i)}{\bar{S}}\right)$$

Với mỗi ô: nếu ô đó kém an toàn hơn mức trung bình ($S(p) < \bar{S}$), cộng thêm phần thiếu hụt. Nếu $S(p) \geq \bar{S}$ thì đóng góp $= 0$ (kẹp về 0).

$$\text{Ví dụ: } S(p)=3.0,; \bar{S}=10.37 ;\Rightarrow; \max(0,; 1-\frac{3.0}{10.37}) = 0.711 \text{ (rủi ro cao)}$$
$$S(p)=15.0,; \bar{S}=10.37 ;\Rightarrow; \max(0,; 1-\frac{15}{10.37}) = 0 \text{ (an toàn, không cộng)}$$

$$\text{SafetyMean} = \frac{1}{N} \sum_{i=1}^{N} S(p_i)$$

Trung bình cộng giá trị $S(p)$ của tất cả ô trên đường. Cao hơn = xa vật cản hơn = tốt hơn.