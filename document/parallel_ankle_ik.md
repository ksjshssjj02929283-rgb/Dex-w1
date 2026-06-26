# 并联踝关节逆运动学算法文档

## 1 概述

并联踝关节机构是一种两自由度（2-DOF）空间并联机构，通过两个独立的线性执行器共同控制踝关节的横滚（Roll）和俯仰（Pitch）姿态。本算法采用解析法求解逆运动学，根据目标姿态角计算各执行器的电机输出角度，并使用数值法计算雅可比矩阵。

## 2 几何模型

### 2.1 执行器结构

每个执行器由以下组件构成：
原点O为脚踝Roll方向旋转中心
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

#### 左踝 - 执行器1

| 参数 | 值 |
|------|-----|
| A1点坐标 | (-46, 0, 257.94) mm |
| C1点初始坐标 | (-46, -59, 0) mm |
| 法兰杆长度 $l_{bar}$ | 60 mm |
| 连杆长度 $l_{rod}$ | 268.354 mm |
| 零位偏移 $\alpha_{offset}$ | 0.1745 rad (10°) |

#### 左踝 - 执行器2

| 参数 | 值 |
|------|-----|
| A2点坐标 | (-46, 0, 177.94) mm |
| C2点初始坐标 | (-46, 59, 0) mm |
| 法兰杆长度 $l_{bar}$ | 60 mm |
| 连杆长度 $l_{rod}$ | 188.355 mm |
| 零位偏移 $\alpha_{offset}$ | -0.1745 rad (-10°) |

#### 右踝 - 执行器1

| 参数 | 值 |
|------|-----|
| A1点坐标 | (-46, 0, 257.94) mm |
| C1点初始坐标 | (-46, 59, 0) mm |
| 法兰杆长度 $l_{bar}$ | 60 mm |
| 连杆长度 $l_{rod}$ | 268.354 mm |
| 零位偏移 $\alpha_{offset}$ | -0.1745 rad (-10°) |

#### 右踝 - 执行器2

| 参数 | 值 |
|------|-----|
| A2点坐标 | (-46, 0, 177.94) mm |
| C2点初始坐标 | (-46, -59, 0) mm |
| 法兰杆长度 $l_{bar}$ | 60 mm |
| 连杆长度 $l_{rod}$ | 188.355 mm |
| 零位偏移 $\alpha_{offset}$ | 0.1745 rad (10°) |

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
- $b = z_{A_i} - z_{C_i}$（负Z方向分量）

向量模长：

$$
d = \|\vec{r}\| = \sqrt{r_x^2 + r_y^2 + r_z^2}
$$

### 4.2 中间变量 $c$ 的几何意义

将向量 $\vec{r}$ 分解为**沿法兰杆方向**和**垂直于法兰杆方向**的分量：

- **投影长度 $c$**：$\vec{r}$ 在法兰杆方向上的投影长度

$$
c = \frac{l_{rod}^2 - l_{bar}^2 - d^2}{2 \cdot l_{bar}}
$$

### 4.3 角度求解

中间几何角度（尚未加入零位偏移与方向修正）：

$$
\theta_i = \arcsin\left( \frac{b \cdot c + |a| \cdot \sqrt{a^2 + b^2 - c^2}}{a^2 + b^2} \right)
$$

### 4.4 电机方向修正

由于左右踝关节的实际电机转向可能与理论方向相反，需要对计算角度进行符号修正。当踝关节类型（左右）与执行器编号（是否为第一个）相同时取反（即 `is_left == is_first_actuator`，XNOR 关系）：

$$
\theta_i = \begin{cases}
-\theta_i & \text{if } is\_left == is\_first\_actuator \\
\theta_i & \text{otherwise}
\end{cases}
$$

随后叠加零位偏移，得到最终电机角度：

$$
\theta_i \mathrel{+}= \alpha_{offset}
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

## 7 正运动学求解

正运动学给定电机角度 $(\theta_1, \theta_2)$，求解对应的俯仰角和横滚角。

采用**Newton-Raphson迭代法**求解残差方程：

$$
\mathbf{f}(\mathbf{q}) = \begin{pmatrix} \theta_1(\mathbf{q}) - \theta_{1,desired} \\ \theta_2(\mathbf{q}) - \theta_{2,desired} \end{pmatrix} = \mathbf{0}
$$

迭代公式：

$$
\mathbf{q}_{n+1} = \mathbf{q}_n - \mathbf{J}^{-1} \cdot \mathbf{f}(\mathbf{q}_n)
$$

其中 $\mathbf{J}$ 为雅可比矩阵。

## 8 雅可比矩阵

### 8.1 问题定义

雅可比矩阵描述了关节空间速度到电机空间速度的映射：

$$
\mathbf{J} = \frac{\partial \boldsymbol{\theta}}{\partial \mathbf{q}} = \begin{pmatrix} 
\frac{\partial \theta_1}{\partial q_{pitch}} & \frac{\partial \theta_1}{\partial q_{roll}} \\ 
\frac{\partial \theta_2}{\partial q_{pitch}} & \frac{\partial \theta_2}{\partial q_{roll}} 
\end{pmatrix}
$$

其中 $\boldsymbol{\theta} = (\theta_1, \theta_2)^T$，$\mathbf{q} = (q_{pitch}, q_{roll})^T$。

### 8.2 数值雅可比计算

由于逆运动学函数 $\theta_i = f(q_{pitch}, q_{roll})$ 是非线性函数，其解析雅可比推导复杂。实际应用中采用**有限差分法**计算数值雅可比：

对于单执行器：

$$\frac{\partial \theta_i}{\partial q_j} \approx \frac{f(q + \Delta q_j) - f(q)}{\Delta q_j}$$

其中 $q_j \in \{q_{pitch}, q_{roll}\}$，$\Delta q_j$ 为微小扰动量（通常取 $\Delta q_j = 1 \times 10^{-6}$）。

### 8.3 雅可比矩阵的用途

| 用途 | 说明 |
|------|------|
| 电机速度求解 | $\dot{\boldsymbol{\theta}} = \mathbf{J} \cdot \dot{\mathbf{q}}$ |
| 关节速度求解 | $\dot{\mathbf{q}} = \mathbf{J}^{-1} \cdot \dot{\boldsymbol{\theta}}$ |
| 力映射 | $\boldsymbol{\tau}_{motor} = \mathbf{J}^{-T} \cdot \boldsymbol{\tau}_{task}$ |
| 刚度映射 | $\mathbf{K}_{motor} = \mathbf{J}^{-T} \cdot \mathbf{K}_{task} \cdot \mathbf{J}^{-1}$ |
| 阻尼映射 | $\mathbf{D}_{motor} = \mathbf{J}^{-T} \cdot \mathbf{D}_{task} \cdot \mathbf{J}^{-1}$ |
| 奇异性分析 | 当 $\det(\mathbf{J}) = 0$ 时机构处于奇异构型 |
| 数值优化 | 逆运动学数值迭代求解 |

## 9 空间转换原理

### 9.1 速度转换

电机空间与关节空间之间的速度关系通过雅可比矩阵建立：

$$
\dot{\boldsymbol{\theta}} = \mathbf{J} \cdot \dot{\mathbf{q}}
$$

其中：
- $\dot{\boldsymbol{\theta}} = (\dot{\theta}_1, \dot{\theta}_2)^T$ 为电机角速度向量
- $\dot{\mathbf{q}} = (\dot{q}_{pitch}, \dot{q}_{roll})^T$ 为关节角速度向量
- $\mathbf{J}$ 为雅可比矩阵

反之，从电机速度转换为关节速度：

$$
\dot{\mathbf{q}} = \mathbf{J}^{-1} \cdot \dot{\boldsymbol{\theta}}
$$

### 9.2 力矩转换

力矩从关节空间映射到电机空间需要使用雅可比矩阵的逆 transpose：

$$
\boldsymbol{\tau}_{motor} = \mathbf{J}^{-T} \cdot \boldsymbol{\tau}_{task}
$$

展开为标量形式：

$$
\begin{pmatrix} \tau_1 \\ \tau_2 \end{pmatrix} = \begin{pmatrix} J_{11}^{-1} & J_{21}^{-1} \\ J_{12}^{-1} & J_{22}^{-1} \end{pmatrix} \begin{pmatrix} \tau_{pitch} \\ \tau_{roll} \end{pmatrix}
$$

其中 $\mathbf{J}^{-T} = (\mathbf{J}^{-1})^T$。

**物理意义**：电机需要产生的力矩不仅与任务空间力矩成正比，还与机构的几何构型有关。当机构处于奇异位置时，雅可比矩阵行列式接近零，电机需要产生极大的力矩来维持很小的任务空间力矩。

### 9.3 刚度转换

刚度矩阵描述了关节位置对力的响应特性。任务空间刚度映射到电机空间刚度的公式为：

$$
\mathbf{K}_{motor} = \mathbf{J}^{-T} \cdot \mathbf{K}_{task} \cdot \mathbf{J}^{-1}
$$

其中 $\mathbf{K}_{task}$ 为任务空间刚度对角矩阵：

$$
\mathbf{K}_{task} = \begin{pmatrix} k_{pitch} & 0 \\ 0 & k_{roll} \end{pmatrix}
$$

记 $\mathbf{M} = \mathbf{J}^{-1}$（即雅可比矩阵的逆，$\mathbf{M}_{ij}$ 为其元素），$\mathbf{K}_{motor}$ 一般为满阵：

$$
\mathbf{K}_{motor} = \begin{pmatrix}
k_{pitch} M_{11}^2 + k_{roll} M_{21}^2 & k_{pitch} M_{11} M_{12} + k_{roll} M_{21} M_{22} \\
k_{pitch} M_{11} M_{12} + k_{roll} M_{21} M_{22} & k_{pitch} M_{12}^2 + k_{roll} M_{22}^2
\end{pmatrix}
$$

**物理意义**：电机刚度通常大于任务空间刚度，因为电机位于传动链的更前端。当机构处于某些特定构型时，一个方向的刚度可能会被"放大"到电机空间。

### 9.4 阻尼转换

阻尼矩阵描述了速度对力的阻尼特性。任务空间阻尼映射到电机空间阻尼的公式与刚度转换形式相同：

$$
\mathbf{D}_{motor} = \mathbf{J}^{-T} \cdot \mathbf{D}_{task} \cdot \mathbf{J}^{-1}
$$

其中 $\mathbf{D}_{task}$ 为任务空间阻尼对角矩阵：

$$
\mathbf{D}_{task} = \begin{pmatrix} d_{pitch} & 0 \\ 0 & d_{roll} \end{pmatrix}
$$

记 $\mathbf{M} = \mathbf{J}^{-1}$（雅可比矩阵的逆，$\mathbf{M}_{ij}$ 为其元素），$\mathbf{D}_{motor}$ 一般为满阵：

$$
\mathbf{D}_{motor} = \begin{pmatrix}
d_{pitch} M_{11}^2 + d_{roll} M_{21}^2 & d_{pitch} M_{11} M_{12} + d_{roll} M_{21} M_{22} \\
d_{pitch} M_{11} M_{12} + d_{roll} M_{21} M_{22} & d_{pitch} M_{12}^2 + d_{roll} M_{22}^2
\end{pmatrix}
$$
