# 并联踝关节逆运动学算法文档

## 1 概述

并联踝关节机构是一种两自由度（2-DOF）空间并联机构，通过两个独立的线性执行器共同控制踝关节的横滚（Roll）和俯仰（Pitch）姿态。本算法采用解析法求解逆运动学，根据目标姿态角计算各执行器的电机输出角度。

## 2 几何模型

### 2.1 执行器结构

每个执行器由以下组件构成：

| 组件 | 说明 |
|------|------|
| A点 | 电机输出端固定点（基坐标系下坐标已知） |
| C点 | 关节连接点（随姿态变化而旋转） |
| 法兰杆（$l_{bar}$） | 电机输出杆，一端连接A点，另一端连接C点 |
| 连杆（$l_{rod}$） | 连接杆，一端连接C点，另一端连接末端执行器 |

### 2.2 几何参数定义

对于第 $i$ 个执行器（$i = 1, 2$）：

$$
\vec{r}_{A_i} = \begin{pmatrix} x_{A_i} \\ y_{A_i} \\ z_{A_i} \end{pmatrix} \quad \text{A点在基坐标系下的位置}
$$

$$
\vec{r}_{C_i}^{(0)} = \begin{pmatrix} x_{C_i}^{(0)} \\ y_{C_i}^{(0)} \\ z_{C_i}^{(0)} \end{pmatrix} \quad \text{C点初始位置（零姿态时）}
$$

### 2.3 几何参数值

#### 执行器1

| 参数 | 值 |
|------|-----|
| A点坐标 | (-46, 0, 257.94) mm |
| C点初始坐标 | (-46, 59, 0) mm |
| 法兰杆长度 $l_{bar}$ | 60 mm |
| 连杆长度 $l_{rod}$ | 268.354 mm |
| 零位偏移 $\alpha_{offset}$ | -0.1745 rad (-10°) |

#### 执行器2

| 参数 | 值 |
|------|-----|
| A点坐标 | (-46, 0, 177.94) mm |
| C点初始坐标 | (-46, -59, 0) mm |
| 法兰杆长度 $l_{bar}$ | 60 mm |
| 连杆长度 $l_{rod}$ | 188.355 mm |
| 零位偏移 $\alpha_{offset}$ | -0.1745 rad (-10°) |

## 3 坐标变换

### 3.1 姿态旋转矩阵

给定目标横滚角 $\phi$（Roll）和俯仰角 $\theta$（Pitch），基坐标系到末端坐标系的旋转矩阵为：

$$
\mathbf{X}_{rot} = \mathbf{R}_y(\theta) \cdot \mathbf{R}_x(\phi)
$$

其中：

$$
\mathbf{R}_x(\phi) = \begin{pmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{pmatrix}
$$

$$
\mathbf{R}_y(\theta) = \begin{pmatrix}
\cos\theta & 0 & \sin\theta \\
0 & 1 & 0 \\
-\sin\theta & 0 & \cos\theta
\end{pmatrix}
$$

### 3.2 C点位置计算

旋转后C点在基坐标系下的位置：

$$
\vec{r}_{C_i} = \mathbf{X}_{rot} \cdot \vec{r}_{C_i}^{(0)}
$$

## 4 逆运动学求解

### 4.1 向量关系

定义从A点到C点的向量：

$$
\vec{r} = \vec{r}_{C_i} - \vec{r}_{A_i} = \begin{pmatrix} r_x \\ r_y \\ r_z \end{pmatrix}
$$

代码中的分量对应关系：

- $a = r_y$（Y方向分量）
- $b = -r_z$（负Z方向分量）

向量模长：

$$
d = \|\vec{r}\| = \sqrt{r_x^2 + r_y^2 + r_z^2} = \sqrt{r_x^2 + a^2 + b^2}
$$

### 4.2 中间变量 $c$ 的几何意义

将向量 $\vec{r}$ 分解为**沿法兰杆方向**和**垂直于法兰杆方向**的分量：

- **投影长度 $c$**：$\vec{r}$ 在法兰杆方向上的投影长度

$$
c = \frac{l_{rod}^2 - l_{bar}^2 - d^2}{2 \cdot l_{bar}}
$$

其中 $d = \sqrt{r_x^2 + a^2 + b^2}$。

**垂直分量**：

$$
\sqrt{d^2 - c^2} = \sqrt{r_x^2 + a^2 + b^2 - c^2}
$$

### 4.3 角度分解与三角恒等变换

将 $\theta_i - \alpha_{offset}$ 分解为两个角度的和：

$$
\theta_i - \alpha_{offset} = \phi + \gamma
$$

其中：
- $\phi = \arctan\left(\frac{b}{a}\right)$：向量 $\vec{r}$ 在Y-Z平面内相对于Y轴的方向角
- $\gamma = \arccos\left(\frac{c}{d}\right)$：法兰杆与向量 $\vec{r}$ 之间的夹角

由三角恒等变换：

$$
\sin(\phi + \gamma) = \sin\phi \cos\gamma + \cos\phi \sin\gamma
$$

代入各分量的表达式：

- $\sin\phi = \frac{b}{d}$（Z方向分量/d）
- $\cos\phi = \frac{a}{d}$（Y方向分量/d）
- $\cos\gamma = \frac{c}{d}$（杆向投影/d）
- $\sin\gamma = \frac{\sqrt{d^2 - c^2}}{d}$（垂直分量/d）


### 4.4 角度求解

最终电机角度为：

$$
\theta_i = \arcsin\left( \frac{b \cdot c + |a| \cdot \sqrt{a^2 + b^2 - c^2}}{a^2 + b^2} \right) + \alpha_{offset}
$$

## 5 构型可行性条件

为确保机构可达，需满足以下约束：

### 5.1 平方根项约束

$$
a^2 + b^2 - c^2 \geq 0
$$

若此项为负，则表明目标构型超出机构工作空间。

### 5.2 正弦值约束

arcsin的参数必须位于合法区间：

$$
\left| \frac{b \cdot c + |a| \cdot \sqrt{a^2 + b^2 - c^2}}{a^2 + b^2} \right| \leq 1
$$

### 5.3 分母约束

$$
a^2 + b^2 \neq 0
$$


## 6 两执行器联合求解

由于两个执行器共享同一踝关节基座，输入相同的Roll和Pitch角度，但使用不同的几何参数独立计算：

$$
\begin{pmatrix} \theta_1 \\ \theta_2 \end{pmatrix} = \begin{pmatrix} f(\phi, \theta, \text{params}_1) \\ f(\phi, \theta, \text{params}_2) \end{pmatrix}
$$

## 7 雅可比矩阵

### 7.1 问题定义

雅可比矩阵描述了关节空间速度到电机空间速度的映射：

$$
\mathbf{J} = \frac{\partial \boldsymbol{\theta}}{\partial \mathbf{q}} = \begin{pmatrix} \frac{\partial \theta_1}{\partial q_{roll}} & \frac{\partial \theta_1}{\partial q_{pitch}} \\ \frac{\partial \theta_2}{\partial q_{roll}} & \frac{\partial \theta_2}{\partial q_{pitch}} \end{pmatrix}
$$

其中 $\boldsymbol{\theta} = (\theta_1, \theta_2)^T$，$\mathbf{q} = (q_{roll}, q_{pitch})^T$。

### 7.2 数值雅可比计算

由于逆运动学函数 $\theta_i = f(q_{roll}, q_{pitch})$ 是非线性函数，其解析雅可比推导复杂。实际应用中采用**有限差分法**计算数值雅可比：

对于单执行器：

$$\frac{\partial \theta_i}{\partial q_j} \approx \frac{f(q + \Delta q_j) - f(q)}{\Delta q_j}$$

其中 $q_j \in \{q_{roll}, q_{pitch}\}$，$\Delta q_j$ 为微小扰动量。

### 7.3 雅可比矩阵的用途

| 用途 | 说明 |
|------|------|
| 电机速度求解 | $\dot{\boldsymbol{\theta}} = \mathbf{J} \cdot \dot{\mathbf{q}}$ |
| 力映射 | $\boldsymbol{\tau}_{motor} = \mathbf{J}^T \cdot \boldsymbol{\tau}_{ankle}$ |
| 奇异性分析 | 当 $\det(\mathbf{J}) = 0$ 时机构处于奇异构型 |
| 数值优化 | 逆运动学数值迭代求解 |

