# PID 控制器参数整定推导与比较

## 1. 基础 PID 结构 (2009 Chang 模型)

PID 控制器形式为：

$$
u = K \left( e + T_D \dot{e} + T_I^{-1} \int e \, dt \right)
$$

展开得：

$$
u = K e + K T_I^{-1} \int e \, dt + K T_D \dot{e}
$$

与标准 PID 形式比较：

$$
K_p = K, \quad K_i = K T_I^{-1}, \quad K_D = K T_D
$$

---

## 2. PPI 控制器结构

PPI 控制形式为：

$$
u = K_{pp} e_v + K_{vi} \int e_v \, dt
$$

其中：

$$
e_v = K_{pp} e + \dot{e}
$$

代入得：

$$
\begin{aligned}
u &= K_{pp}(K_{pp}e + \dot{e}) + K_{vi} \int (K_{pp}e + \dot{e}) \, dt \\
  &= K_{pp}^2 e + K_{pp} \dot{e} + K_{vi} K_{pp} \int e \, dt + K_{vi} \int \dot{e} \, dt
\end{aligned}
$$

注意 $\int \dot{e} \, dt = e$，合并项：

$$
u = (K_{pp}^2 + K_{vi}) e + K_{vi} K_{pp} \int e \, dt + K_{pp} \dot{e}
$$

---

## 3. 比较 PID 与 PPI 参数

PID 形式：

$$
u = K_p e + K_i \int e \, dt + K_D \dot{e}
$$

对比得：

$$
\begin{aligned}
K_p &= K_{pp}^2 + K_{vi} \\
K_i &= K_{vi} K_{pp} \\
K_D &= K_{pp}
\end{aligned}
$$

---

## 4. 参数关系推导 (2009 Chang)

已知：

$$
\begin{aligned}
K &= K_{vp} K_{pp} + K_{vi} \\
K T_I^{-1} &= K_{vi} K_{pp} \\
K T_D &= K_{vp}
\end{aligned}
$$

---

## 5. 代入系统参数

由 2009 Chang 模型：

$$
\begin{aligned}
K &= \frac{\bar{g}^{-1} \cdot 2S \omega_n}{L} \\
T_D &= \frac{1}{2S \omega_n} \\
T_I &= \frac{2S}{\omega_n}
\end{aligned}
$$

代入得：

$$
\begin{aligned}
\frac{\bar{g}^{-1} \cdot 2S \omega_n}{L} &= K_{vp} K_{pp} + K_{vi} \\
\frac{\bar{g}^{-1} \cdot 2S \omega_n}{L} \cdot \frac{\omega_n}{2S \omega_n} &= K_{vi} K_{pp} = \frac{\bar{g}^{-1} \cdot \omega_n^2}{L} \\
\frac{\bar{g}^{-1} \cdot 2S \omega_n}{L} \cdot \frac{1}{2S \omega_n} &= K_{vp} = \frac{\bar{g}^{-1}}{L}
\end{aligned}
$$

---

## 6. 最终参数表达式

由上述推导可得：

$$
\begin{aligned}
K_{vp} &= \frac{\bar{g}^{-1}}{L} \\
K_{vi} &= \frac{\bar{g}^{-1}}{L} (\xi \omega_n + \omega_n \sqrt{\xi^2 - 1}) \\
K_{pp} &= \xi \omega_n + \omega_n \sqrt{\xi^2 - 1}
\end{aligned}
$$

---

## 7. 特殊情况：$\xi = 1$

当阻尼比 $\xi = 1$ 时：

$$
\begin{aligned}
K_{vp} &= \frac{\bar{g}^{-1}}{L} \\
K_{vi} &= \frac{\bar{g}^{-1}}{L} \cdot \omega_n \\
K_{pp} &= \omega_n \quad \text{(需注意系数关系)}
\end{aligned}
$$

---

## 8. 参数整定顺序

建议整定顺序为：

1. $K_{vp}$
2. $K_{vi}$
3. $K_{pp}$

---

## 注

- 所有符号含义遵循 2009 Chang 文献中的定义。
- 推导中假设系统为二阶标准形式，阻尼比 $\xi$ 和自然频率 $\omega_n$ 已知。
- 若选择负根形式，$K_{vi}$ 可改为：

$$
K_{vi} = \frac{\bar{g}^{-1}}{L} (\xi \omega_n - \omega_n \sqrt{\xi^2 - 1})
$$
