> 返回总目录：[paper_spec.md](../../paper_spec.md) | 正文对应：[第 1 章 §1.2.1](../01_浮基运动学.md#121-足端单点运动学位置雅可比与导数)

# 附录 A. 雅可比矩阵列权重推导

> 本附录给出 [第 1 章 §1.2.1](../01_浮基运动学.md#121-足端单点运动学位置雅可比与导数)（图 1.2）中 \(J_{\text{tip}}\) 与 \(J_{\text{toe}}\) 各列解析式的几何来源与代数展开（极限特例验算见 [第 8 章 §8.2.1](../08_硬件设计验证与里程碑映射.md#821-step-1-雅可比极限特例验算)）。

$$
\underbrace{\dot{\mathbf{p}}_{\text{tip}}}_{\text{爪尖速度}}
= \underbrace{\sum_{j=1}^{6}\textcolor{#16a34a}{\frac{\partial\mathbf{p}_{\text{tip}}}{\partial q_j}}\,\dot{q}_j}_{\text{6 个坐标速度贡献之和（链式法则）}}
= \underbrace{\begin{bmatrix}
\textcolor{#16a34a}{\dfrac{\partial\mathbf{p}_{\text{tip}}}{\partial q_1}} & \cdots & \textcolor{#16a34a}{\dfrac{\partial\mathbf{p}_{\text{tip}}}{\partial q_6}}
\end{bmatrix}}_{\textcolor{#16a34a}{J_{\text{tip}}(\mathbf{q})}\in\mathbb{R}^{2\times 6}}\dot{\mathbf{q}}
$$

![图 A.1 雅可比矩阵的物理直觉：平动平移、转动切向与矢量叠加](../assets/fig_a_1_jacobian_intuition.svg?v=16)

---

## A.1 列权重的几何求法与逐列展开

### 1. 转动列通式（杆臂逆时针转 90°）

- **平动列**（\(q_1, q_2\)）：仅沿 \(+X / +Y\) 单位刚体平移，前两列直接为 \([1,0]^T\) 与 \([0,1]^T\)（即单位阵 \(\mathbf{I}_2\)）。
- **转动列**（\(q_3 \sim q_6\)）：锁死其余坐标、仅绕面外轴 \(\mathbf{e}_z = [0, 0, 1]^T\) 以单位角速度 \(\dot{q}_j = 1\) 转动时，爪尖线速度为 \(\mathbf{e}_z \times \mathbf{r}_j\)。取平面 \(X\text{-}Y\) 分量，恰好等于将杆臂 \(\textcolor{#0284c7}{\mathbf{r}_j} = \mathbf{p}_{\text{tip}} - \mathbf{p}_{c(j)}\) 逆时针旋转 \(90^\circ\)：

$$
\textcolor{#16a34a}{\frac{\partial\mathbf{p}_{\text{tip}}}{\partial q_j}}
= \mathbf{e}_z \times \textcolor{#0284c7}{\mathbf{r}_j}
= \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} r_{j,x} \\ r_{j,y} \\ 0 \end{bmatrix}
\xrightarrow{\text{取 }X\text{-}Y}
\begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}\textcolor{#0284c7}{\begin{bmatrix} r_{j,x} \\ r_{j,y} \end{bmatrix}}
= R\!\left(\tfrac{\pi}{2}\right)\textcolor{#0284c7}{\big(\mathbf{p}_{\text{tip}}-\mathbf{p}_{c(j)}\big)}
= \textcolor{#16a34a}{\begin{bmatrix} -(y_{\text{tip}}-y_{c(j)}) \\ x_{\text{tip}}-x_{c(j)} \end{bmatrix}},
\qquad j=3,4,5,6
$$

![图 A.2 各转动列对应的有效杆臂与 6 列权重速查](../assets/fig_a_2_lever_arms.svg?v=1)

### 2. 单列推导示例（以第 5 列 \(q_5\) 为例）

以上图中橙色杆臂 \(\mathbf{r}_5\)（绕踝铰点 \(J_2\)，跨 \(l_2 \sim l_4\)）为例，将其世界系投影代入通式 \((r_x,\ r_y) \mapsto (-r_y,\ r_x)\)：

$$
\textcolor{#0284c7}{\mathbf{r}_5} = \mathbf{p}_{\text{tip}} - \mathbf{p}_2 =
\begin{bmatrix} l_2\cos\alpha_2 + l_3\cos\alpha_3 + l_4\cos\alpha_4 \\ l_2\sin\alpha_2 + l_3\sin\alpha_3 + l_4\sin\alpha_4 \end{bmatrix}
\;\xrightarrow{\;R(\pi/2)\;}\;
\textcolor{#16a34a}{\begin{bmatrix} J_{15} \\ J_{25} \end{bmatrix}} =
\begin{bmatrix} -(l_2\sin\alpha_2 + l_3\sin\alpha_3 + l_4\sin\alpha_4) \\ l_2\cos\alpha_2 + l_3\cos\alpha_3 + l_4\cos\alpha_4 \end{bmatrix}
$$

其余转动列（第 3、4 列代入蓝杆臂 \(\mathbf{r}_3=\mathbf{r}_4\)，第 6 列代入绿杆臂 \(\mathbf{r}_6\)）按完全相同的步骤代入，即得 [第 1 章图 1.2](../01_浮基运动学.md#121-足端单点运动学位置雅可比与导数) 中的全部闭式；趾关节 \(J_{\text{toe}}\) 只需令 \(l_4 = 0\)，第 6 列自然为 \(\mathbf{0}\)。

### 3. 几何法与代数求导对照验证

![几何法（杆臂逆转 90°）与代数法（正运动学求导归并）对照](../assets/fig_a_3_geo_vs_alg.svg?v=5)
