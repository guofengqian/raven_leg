> 返回总目录：[paper_spec.md](../../paper_spec.md) | 正文对应：[第 1 章 §1.2.3](../01_浮基构型与足地接触运动学.md)

# 附录 A. 爪尖与趾关节的雅可比矩阵（由线速度中广义坐标的权重定义）

## 目录

- [A.1 定义：线速度 = 权重矩阵 \(\times\) 广义速度](#a1-定义线速度-权重矩阵-times-广义速度)
- [A.2 各列权重的几何求法（杆臂转 \(\pi/2\)）](#a2-各列权重的几何求法杆臂转-pi2)
- [A.3 权重展开成矩阵](#a3-权重展开成矩阵)
- [A.4 极限工况（检验权重列）](#a4-极限工况检验权重列)

---

**结论.** 雅可比的主语是接触点位置：爪尖 \(\mathbf{p}_{\text{tip}}\) 与趾关节 \(\mathbf{p}_{\text{toe}}\)。二者线速度对广义速度 \(\dot{\mathbf{q}}\) 都是线性的；各 \(\dot{q}_j\) 前面的**权重矩阵就是雅可比**：

$$
\dot{\mathbf{p}}_{\text{tip}} = J_{\text{tip}}(\mathbf{q})\,\dot{\mathbf{q}}, \qquad
\dot{\mathbf{p}}_{\text{toe}} = J_{\text{toe}}(\mathbf{q})\,\dot{\mathbf{q}}
$$

下文以爪尖为主线写列权重；趾关节同一套规则，末端改为 \(\mathbf{p}_{\text{toe}}\)（在 \(J_4\) 截断）。实现仍以 §1.1.4 成品为准。前置：§1.1.2 的 \(\alpha_i,\dot{\alpha}_i\)，§1.1.3 的 \(\mathbf{p}_{\text{tip}},\mathbf{p}_{\text{toe}},\mathbf{p}_i\)。支撑 `tests/test_step1.py`。

## A.1 定义：线速度 = 权重矩阵 \(\times\) 广义速度

**结论.** 爪尖位置 \(\mathbf{p}_{\text{tip}}(\mathbf{q})\) 只依赖构型。对时间求导：

$$
\dot{\mathbf{p}}_{\text{tip}}
= \sum_{j=1}^{6}\frac{\partial\mathbf{p}_{\text{tip}}}{\partial q_j}\,\dot{q}_j
= J_{\text{tip}}(\mathbf{q})\,\dot{\mathbf{q}}
$$

其中权重矩阵（雅可比）按列拼成：

$$
J_{\text{tip}}(\mathbf{q})
= \begin{bmatrix}
\dfrac{\partial\mathbf{p}_{\text{tip}}}{\partial q_1} &
\dfrac{\partial\mathbf{p}_{\text{tip}}}{\partial q_2} &
\dfrac{\partial\mathbf{p}_{\text{tip}}}{\partial q_3} &
\dfrac{\partial\mathbf{p}_{\text{tip}}}{\partial q_4} &
\dfrac{\partial\mathbf{p}_{\text{tip}}}{\partial q_5} &
\dfrac{\partial\mathbf{p}_{\text{tip}}}{\partial q_6}
\end{bmatrix}
\in\mathbb{R}^{2\times 6}
$$

第 \(j\) 列就是 \(\dot{q}_j\) 的权重：只动第 \(j\) 个广义坐标、速率为 1 时，爪尖得到的线速度。同理 \(\dot{\mathbf{p}}_{\text{toe}}=J_{\text{toe}}\dot{\mathbf{q}}\)。

**推导.**

1. 正运动学已给出 \(\mathbf{p}_{\text{tip}}=\mathbf{p}_{\text{tip}}(\mathbf{q})\)（§1.1.3）。速度是位置对时间的导数，链式法则自动变成「对各 \(q_j\) 的偏导 × \(\dot{q}_j\)」再求和。
2. 把六个偏导并成一行块矩阵，左乘 \(\dot{\mathbf{q}}\)，即 \(\dot{\mathbf{p}}_{\text{tip}}=J\dot{\mathbf{q}}\)。因此 **J 不是另起炉灶的对象**，而是线速度表达式里 \(\dot{\mathbf{q}}\) 的系数（权重）矩阵。
3. 余下只需求出每一列权重（见 A.2），再代入杆臂坐标展开成 §1.1.4 的数值公式（见 A.3）。

## A.2 各列权重的几何求法（杆臂转 \(\pi/2\)）

**结论.** 平动广义坐标的权重是单位阵；转动广义坐标的权重等于「**转动中心 → 爪尖**」杆臂左乘 \(R_Z(\pi/2)\)：

$$
\frac{\partial\mathbf{p}_{\text{tip}}}{\partial q_j}
= R_Z\!\left(\tfrac{\pi}{2}\right)\big(\mathbf{p}_{\text{tip}}-\mathbf{p}_{c(j)}\big)
= \begin{bmatrix} -(y_{\text{tip}}-y_{c(j)}) \\ x_{\text{tip}}-x_{c(j)} \end{bmatrix},
\qquad j=3,4,5,6
$$

| 列（\(\dot{q}_j\) 的权重） | \(q_j\) | 转动中心 \(\mathbf{p}_{c(j)}\) | 杆臂 \(\mathbf{r}_j=\mathbf{p}_{\text{tip}}-\mathbf{p}_{c(j)}\) |
|---|---|---|---|
| 1–2 | \(q_1,q_2\) | —（纯平移） | 权重 \(=\mathbf{I}_2\) |
| 3 | \(q_3\) | \(\mathbf{p}_1\) | 全链 \(l_1\sim l_4\) |
| 4 | \(q_4\) | \(\mathbf{p}_1\)（与 \(q_3\) 同） | 同第 3 列 |
| 5 | \(q_5\) | \(\mathbf{p}_2\) | \(l_2\sim l_4\) |
| 6 | \(q_6\) | \(\mathbf{p}_4\) | 仅 \(l_4\) |

\(J_{\text{toe}}\)：同一「权重 = 杆臂转 \(\pi/2\)」规则，末端改为 \(\mathbf{p}_{\text{toe}}\)（在 \(J_4\) 截断、无 \(l_4\)），故第 6 列权重恒 \(\mathbf{0}\)。

**推导.**

1. **平移权重**：\(q_1,q_2\) 把整条腿刚体平移，爪尖速度 \(=[\dot{q}_1,\dot{q}_2]^T\)，故前两列权重为 \(\mathbf{I}_2\)。
2. **转动权重来自** \(\boldsymbol{\omega}\times\mathbf{r}\)：绕面外轴以角速度 \(\dot{q}_j\) 转时，爪尖线速度 \(\dot{q}_j\,\mathbf{e}_z\times\mathbf{r}_j\)。平面内

$$
\mathbf{e}_z\times\mathbf{r} = \begin{bmatrix} -r_y \\ r_x \end{bmatrix} = R_Z\!\left(\tfrac{\pi}{2}\right)\mathbf{r}
$$

因此该 \(\dot{q}_j\) 的权重向量 = 杆臂转 \(\pi/2\)（垂直于「中心→爪尖」，逆时针切向），而不是杆臂本身。
3. **中心怎么选**：\(q_j\) 只带动下游杆段，最近不动铰点即 \(\mathbf{p}_{c(j)}\)。\(q_3\) 与 \(q_4\) 都使 \(\alpha_1\sim\alpha_4\) 同增，中心同为 \(J_1\)，两列权重相同。\(\theta_3\) 不是广义坐标（固定安装角），无对应列。
4. **与串联叠加一致**：四段以 \(\dot{\alpha}_i\) 绕近端铰转动时

$$
\dot{\mathbf{p}}_{\text{tip}}
= \begin{bmatrix} \dot{q}_1 \\ \dot{q}_2 \end{bmatrix}
+ \sum_{i=1}^{4} \dot{\alpha}_i\,(\mathbf{e}_z\times\mathbf{r}_i),
\qquad \mathbf{r}_i=\mathbf{p}_{\text{tip}}-\mathbf{p}_i
$$

把 §1.1.2 的 \(\dot{\alpha}_i(\dot{\mathbf{q}})\) 代入，按 \(\dot{q}_j\) 归并系数，得到的正是上表各列权重——与「逐列选中心、杆臂转 \(\pi/2\)」同一结果。

## A.3 权重展开成矩阵

**结论.** 把 §1.1.3 的杆臂差向量代入 A.2 的通式，逐列即得 §1.1.4 的 \(J_{\text{tip}}\)；\(J_{\text{toe}}\) 只需在所有杆臂中截去 \(l_4\)，第 6 列自然为 0。

**推导.** 三个转动中心对应杆臂 \(\mathbf{p}_{\text{tip}}-\mathbf{p}_1\)、\(\mathbf{p}_{\text{tip}}-\mathbf{p}_2\)、\(\mathbf{p}_{\text{tip}}-\mathbf{p}_4\)，其坐标由 §1.1.3 直接给出；左乘 \(R_Z(\pi/2)\)，即 \((r_x,r_y)\mapsto(-r_y,\ r_x)\)，再按列排列。成品矩阵只在 §1.1.4 定稿，此处不重复。

## A.4 极限工况（检验权重列）

**结论.** 下列极限必须与 A.1–A.3 / §1.1.4 一致（单测）：

1. 仅 \(\dot{q}_1,\dot{q}_2\)：\(\dot{\mathbf{p}}_{\text{tip}}=[\dot{q}_1,\dot{q}_2]^T\)，前两列权重 \(=\mathbf{I}_2\)。
2. 仅 \(\dot{q}_6\)：\(\lVert\dot{\mathbf{p}}_{\text{tip}}\rVert=l_4\lvert\dot{q}_6\rvert\)，方向垂直脚趾（第 6 列 = 趾杆臂转 \(\pi/2\)）。
3. \(\dot{q}_4=-\dot{q}_3\) 其余零：\(\dot{\alpha}_i\equiv 0\Rightarrow\dot{\mathbf{p}}_{\text{tip}}=\mathbf{0}\)，第 3、4 列权重相同故抵消。
4. \(\alpha_i=0\)：转动列权重 \(=[0,\sum l_i]^T\)，杆臂沿 \(+X\)，转 \(\pi/2\) 后切向纯竖直。

**推导.** 把特殊 \(\dot{\mathbf{q}}\) 或 \(\alpha_i\) 代入 \(\dot{\mathbf{p}}_{\text{tip}}=J\dot{\mathbf{q}}\)（A.2 / §1.1.4），化简即得。
