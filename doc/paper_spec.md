# EPFL RAVEN 动力学仿真数学规范 — 矢状面 6-DOF 完整建模体系

> 本规范严格对齐论文 *Fast ground-to-air transition with avian-inspired multifunctional legs* 的 **Methods — Dynamics modeling and simulation（pp. 13–16，式 (2)–(23)）**，并与 `plan/milestone.md`（Step 1 → Step 5）及 `config/robot_paras.yaml` 保持符号与量纲完全一致。

## 1. 广义坐标与坐标系定义 (System Coordinates)

### 1.1 坐标系建立_step1

1. **世界惯性坐标系** \(O\text{-}XZ\) **(World Frame)**：
   * **原点** \(O\)：固定于水平地面，取初始构型下尾巴末端在地面上的投影点（依据论文 Extended Data Fig. 5a Base Frame）。
   * \(+X\) **轴**：沿机器人前进方向水平指向正前方。
   * \(+Z\) **轴**：竖直向上。
   * \(+Y\) **轴**：垂直于矢状面向外，与 \(X, Z\) 构成右手直角坐标系。所有绕 \(Y\) 轴的转角以逆时针（抬头/后倾）为正。
2. **机身本体坐标系** \(B\text{-}x_B z_B\) **(Body Frame)**：
   * **原点** \(O_B\)：**左右两个髋关节连线的中点**（即两髋在矢状面内的公共投影点）。**注意**：\(O_B\) **不是机身质心**。
   * \(+x_B\) **轴**：沿机身纵轴指向头部，固连于机身刚体。
   * \(+z_B\) **轴**：在矢状面内垂直于 \(+x_B\) 轴向上（由 \(+x_B\) 绕 \(+Y\) 逆时针旋转 \(90^\circ\) 得到）。
   * **姿态变换**：机身姿态完全由俯仰角 \(q_3\) 确定，本体系到世界系的二维旋转矩阵为：

$$
R(q_3) = \begin{bmatrix} \cos q_3 & -\sin q_3 \\ \sin q_3 & \cos q_3 \end{bmatrix}
$$

### 1.2 六个广义坐标向量 \(\mathbf{q} \in \mathbb{R}^6\)

系统采用浮基串联腿链矢状面模型，包含 **6 个广义坐标**，与 `config/robot_paras.yaml` 中的符号定义完全一致：

$$
\mathbf{q} = \begin{bmatrix} q_1 & q_2 & q_3 & q_4 & q_5 & q_6 \end{bmatrix}^T \in \mathbb{R}^6
$$

| 广义坐标 | 符号说明 | 物理含义与驱动属性 |
|---|---|---|
| \(q_1\) | \(x_{\text{hip}}\) | 髋关节连线中点在世界系下的水平位置（浮基平动，无驱动） |
| \(q_2\) | \(z_{\text{hip}}\) | 髋关节连线中点在世界系下的垂直位置（浮基平动，无驱动） |
| \(q_3\) | \(\theta_{\text{body}}\) | 机身纵轴相对世界系 \(+X\) 轴的俯仰角（浮基转动，抬头为正，无驱动） |
| \(q_4\) | \(q_{\text{hip}}\) | 髋关节相对转角（大腿相对机身纵轴夹角，电机驱动） |
| \(q_5\) | \(q_{\text{ankle}}\) | 踝关节相对转角（下肢相对大腿夹角，电机驱动 + 并行储能扭簧） |
| \(q_6\) | \(q_{\text{toe}}\) | 脚趾关节相对偏转角（脚趾相对偏移杆夹角，被动弹性扭簧） |

### 1.3 机构几何常数与连杆定义

腿链由 4 根刚性连杆串联构成，几何参数从 `config/robot_paras.yaml` 读取：

* \(l_1\)：上肢（大腿）长度，自髋关节 \(J_1\) 至踝关节 \(J_2\)。
* \(l_2\)：下肢（跗跖骨）长度，自踝关节 \(J_2\) 至下肢末端基点 \(J_3\)。
* \(l_3\)：趾关节偏移杆长度，自下肢末端 \(J_3\) 至趾关节 \(J_4\)。
* \(l_4\)：脚趾连杆长度，自趾关节 \(J_4\) 至足端触地点 \(\mathbf{p}_{\text{tip}}\)。
* \(\theta_3\)：**趾关节固定偏移角**（常数参数，非广义坐标）。量度下肢 \(l_2\) 轴线与偏移杆 \(l_3\) 之间的固定安装夹角（`theta3_deg = 45°`），无相对自由度。
* \((d_x, d_z)\)：**机身质心** \(C\) **在机身本体坐标系** \(B\) **下的位置偏移**（原点为两髋连线中点，轴向约定见 §1.1）。论文未给出直接测量数值，但由正文稳定性条件与 Extended Data Table 2 气动参数可推导其**严格可行域（取值区间）**：
  $$
  d_x \in (-0.02,\ 0)\ \text{m}, \qquad d_z \in (0,\ 0.052)\ \text{m}
  $$
  1. **水平区间** \(d_x \in (-0.02, 0)\ \text{m}\) **（髋后 0 至 2 cm 之间）**：
     * **上界** \(d_x < 0\)：质心位于髋关节后方。论文 p13 指出起跳初始构型中质心投影在趾关节前方微小邻域（以驱动身体前倾起跳），而由于两髋关节处于足端更靠前的位置，因此髋关节坐标原点在质心前方；
     * **下界** \(d_x > -0.02\ \text{m}\)：论文 p4 明确指出质心必须位于全机气动合力中心（中立点 Neutral Point）之前（*"in front of ... the center of aerodynamic forces"*）以确保滑翔飞行被动稳定性；Extended Data Table 2 给出的主升力面（机翼）气动压心位置为 \(P_{w,x} = -0.02\ \text{m}\)，因此质心纵向位置必须位于机翼压心之前，即 \(d_x > -0.02\ \text{m}\)。
  2. **垂直区间** \(d_z \in (0, 0.052)\ \text{m}\) **（髋上 0 至 5.2 cm 之间）**：
     * **下界** \(d_z > 0\)：论文 p4 指明髋关节安装于机翼前缘下方及附近（*"placed the hips below and near the wing leading edge"*），机身大部分刚体质量（机身、电子设备、机翼等）均位于髋部上方，故质心在髋关节上方；
     * **上界** \(d_z < 0.052\ \text{m}\)：论文 p4 指明质心必须位于气动合力中心下方（*"below the center of aerodynamic forces"*）；结合 Extended Data Table 2 中机翼（面积 \(0.18\ \text{m}^2\)，高度 \(P_{w,z} = 0.042\ \text{m}\)）与尾翼（面积 \(0.0864\ \text{m}^2\)，高度 \(P_{t,z} = 0.074\ \text{m}\)）按面积加权估算全机气动中立点高度：
     \(z_{\text{NP}} \approx \frac{0.18 \times 0.042 + 0.0864 \times 0.074}{0.18 + 0.0864} \approx 0.0524\ \text{m}\)
     因此垂直位置必须满足 \(d_z < 0.052\ \text{m}\)。
  3. **地面静态与起步支撑多边形约束**：论文 p4、p13 及 Extended Data Fig. 3d 指出，静态站立时质心地面投影落在趾关节与后爪（hallux）构成的足底支撑区间内，以实现被动平衡；起跳构型下质心微偏趾关节前侧（支持向前蹬伸）。
  4. **工程落地原则**：根据项目规范，此区间仅作为 Step 3 动力学方程参数的可行域约束，当前阶段**暂不填入未经验证的经验常数**（详见 `doc/missing_params_log.md`）。

---

## 2. 串联运动链与连杆绝对倾角 (Kinematic Chain)

### 2.1 局部坐标系与连杆向量

定义 4 个固连于各连杆近端铰点的局部坐标系 \(B_1 \sim B_4\)，连杆向量沿自身局部系 \(+x\) 轴：

* 上肢 \(l_1\) 在髋关节系 \(B_1\) 下：\(\mathbf{v}_1 = [l_1, 0]^T\)
* 下肢 \(l_2\) 在踝关节系 \(B_2\) 下：\(\mathbf{v}_2 = [l_2, 0]^T\)
* 偏移杆 \(l_3\) 在偏移基系 \(B_3\) 下：\(\mathbf{v}_3 = [l_3, 0]^T\)
* 脚趾 \(l_4\) 在趾关节系 \(B_4\) 下：\(\mathbf{v}_4 = [l_4, 0]^T\)

### 2.2 连杆绝对倾角 \(\alpha_i\)

定义各连杆相对惯性世界坐标系 \(+X\) 轴（水平向前）的逆时针绝对转角 \(\alpha_i\)。由串联链逐级叠加展开：

$$
\begin{aligned}
\alpha_1 &= q_3 + q_4 \\
\alpha_2 &= \alpha_1 + q_5 = q_3 + q_4 + q_5 \\
\alpha_3 &= \alpha_2 + \theta_3 = q_3 + q_4 + q_5 + \theta_3 \\
\alpha_4 &= \alpha_3 + q_6 = q_3 + q_4 + q_5 + \theta_3 + q_6
\end{aligned}
$$

各式含义：

* \(\alpha_1\)：上肢 \(l_1\) 的绝对倾角
* \(\alpha_2\)：下肢 \(l_2\) 的绝对倾角
* \(\alpha_3\)：偏移杆 \(l_3\) 的绝对倾角（含固定角 \(\theta_3\)）
* \(\alpha_4\)：脚趾 \(l_4\) 的绝对倾角

### 2.3 绝对角速度传递关系

对绝对倾角求时间导数，因 \(\theta_3\) 为几何固定常数（\(\dot{\theta}_3 = 0\)），得到角速度传递关系：

$$
\begin{aligned}
\dot{\alpha}_1 &= \dot{q}_3 + \dot{q}_4 \\
\dot{\alpha}_2 &= \dot{q}_3 + \dot{q}_4 + \dot{q}_5 \\
\dot{\alpha}_3 &= \dot{\alpha}_2 = \dot{q}_3 + \dot{q}_4 + \dot{q}_5 \\
\dot{\alpha}_4 &= \dot{\alpha}_3 + \dot{q}_6 = \dot{q}_3 + \dot{q}_4 + \dot{q}_5 + \dot{q}_6
\end{aligned}
$$

---

## 3. 正运动学解析方程 (Forward Kinematics，论文式 (6)–(8))

### 3.1 关键铰点世界系坐标

沿运动链从机身本体坐标系原点（髋关节 \(J_1\)）逐级向远端递推：

1. **髋关节位置** \(\mathbf{p}_{\text{hip}}\) (\(J_1\))：

$$
\mathbf{p}_1 = \begin{bmatrix} q_1 \\ q_2 \end{bmatrix}
$$

2. **踝关节位置** \(\mathbf{p}_{\text{ankle}}\) (\(J_2\))：

$$
\mathbf{p}_2 = \mathbf{p}_1 + R(\alpha_1)\begin{bmatrix} l_1 \\ 0 \end{bmatrix} = \begin{bmatrix} q_1 + l_1 \cos\alpha_1 \\ q_2 + l_1 \sin\alpha_1 \end{bmatrix}
$$

3. **下肢末端基点位置** \(\mathbf{p}_{\text{base}}\) (\(J_3\))：

$$
\mathbf{p}_3 = \mathbf{p}_2 + R(\alpha_2)\begin{bmatrix} l_2 \\ 0 \end{bmatrix} = \begin{bmatrix} x_{\text{ankle}} + l_2 \cos\alpha_2 \\ z_{\text{ankle}} + l_2 \sin\alpha_2 \end{bmatrix}
$$

4. **趾关节位置** \(\mathbf{p}_{\text{toe}}\) (\(J_4\)，**对应论文 p14 式 (6)**)：

$$
\mathbf{p}_{\text{toe}}(\mathbf{q}) \equiv \mathbf{p}_4 = \mathbf{p}_3 + R(\alpha_3)\begin{bmatrix} l_3 \\ 0 \end{bmatrix} = \begin{bmatrix} q_1 + l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 \\ q_2 + l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3 \end{bmatrix}
$$

### 3.2 足端爪尖触地点世界系坐标 \(\mathbf{p}_{\text{tip}}(\mathbf{q})\)（对应论文 p14 式 (7)）

足端爪尖（论文记为 \(\mathbf{p}_{\text{claw}}\)，代码接口统称 \(\mathbf{p}_{\text{tip}}\)）位于脚趾连杆 \(l_4\) 的末端，解析位置向量为：

$$
\mathbf{p}_{\text{tip}}(\mathbf{q}) \equiv \mathbf{p}_{\text{claw}}(\mathbf{q}) = \mathbf{p}_{\text{toe}}(\mathbf{q}) + R(\alpha_4)\begin{bmatrix} l_4 \\ 0 \end{bmatrix} = \begin{bmatrix} x_{\text{tip}}(\mathbf{q}) \\ z_{\text{tip}}(\mathbf{q}) \end{bmatrix}
$$

显式展开为绝对倾角求和形式：

$$
\begin{aligned}
x_{\text{tip}}(\mathbf{q}) &= q_1 + l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 + l_4 \cos\alpha_4 \\
z_{\text{tip}}(\mathbf{q}) &= q_2 + l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3 + l_4 \sin\alpha_4
\end{aligned}
$$

### 3.3 各构件质心（CoM）位置方程

用于后续 Step 3 动能与重力势能推导：

1. **机身质心** \(\mathbf{p}_C\)（本体偏移 \((d_x, d_z)\) 满足可行域 \(d_x \in (-0.02, 0)\ \text{m}, d_z \in (0, 0.052)\ \text{m}\)，详见 §1.3）：

$$
\mathbf{p}_C(\mathbf{q}) = \begin{bmatrix} q_1 \\ q_2 \end{bmatrix} + R(q_3)\begin{bmatrix} d_x \\ d_z \end{bmatrix} = \begin{bmatrix} q_1 + d_x \cos q_3 - d_z \sin q_3 \\ q_2 + d_x \sin q_3 + d_z \cos q_3 \end{bmatrix}
$$

2. **各腿杆质心** \(\mathbf{p}_{ci}\)（设质心距各自近端铰点为 \(c_i\)）：

$$
\begin{aligned}
\mathbf{p}_{c1} &= \mathbf{p}_1 + R(\alpha_1)[c_1, 0]^T = \begin{bmatrix} q_1 + c_1 \cos\alpha_1 \\ q_2 + c_1 \sin\alpha_1 \end{bmatrix} \\
\mathbf{p}_{c2} &= \mathbf{p}_2 + R(\alpha_2)[c_2, 0]^T = \begin{bmatrix} x_{\text{ankle}} + c_2 \cos\alpha_2 \\ z_{\text{ankle}} + c_2 \sin\alpha_2 \end{bmatrix} \\
\mathbf{p}_{c3} &= \mathbf{p}_3 + R(\alpha_3)[c_3, 0]^T = \begin{bmatrix} x_{\text{base}} + c_3 \cos\alpha_3 \\ z_{\text{base}} + c_3 \sin\alpha_3 \end{bmatrix} \\
\mathbf{p}_{c4} &= \mathbf{p}_4 + R(\alpha_4)[c_4, 0]^T = \begin{bmatrix} x_{\text{toe}} + c_4 \cos\alpha_4 \\ z_{\text{toe}} + c_4 \sin\alpha_4 \end{bmatrix}
\end{aligned}
$$

---

## 4. 解析雅可比矩阵 \(J(\mathbf{q})\) 与时间导数 \(\dot{J}(\mathbf{q}, \dot{\mathbf{q}})\)（论文式 (2)–(4)）

### 4.1 足端爪尖雅可比矩阵 \(J_{\text{tip}}(\mathbf{q}) \in \mathbb{R}^{2 \times 6}\)

根据多元复合函数对时间的链式求导法则，足端在惯性系下的瞬时线速度通过遍历全部 6 个广义坐标求和得到（对应论文式 (2)、(4)）：

$$
\dot{\mathbf{p}}_{\text{tip}} = \frac{d\mathbf{p}_{\text{tip}}}{dt} = \sum_{j=1}^6 \frac{\partial \mathbf{p}_{\text{tip}}}{\partial q_j}\dot{q}_j = \begin{bmatrix} \sum_{j=1}^6 J_{1j}\dot{q}_j \\ \sum_{j=1}^6 J_{2j}\dot{q}_j \end{bmatrix} = J_{\text{tip}}(\mathbf{q})\,\dot{\mathbf{q}}
$$

其中雅可比矩阵定义为 \(J_{\text{tip}}(\mathbf{q}) = \frac{\partial \mathbf{p}_{\text{tip}}}{\partial \mathbf{q}} \in \mathbb{R}^{2 \times 6}\)。

### 4.2 解析矩阵结构与各列分量

由于浮基平动坐标 \((q_1, q_2)\) 为线性平移项，其对应的雅可比前两列恒为单位阵：

$$
J_{\text{tip}}(\mathbf{q}) = \begin{bmatrix} 1 & 0 & J_{13} & J_{14} & J_{15} & J_{16} \\ 0 & 1 & J_{23} & J_{24} & J_{25} & J_{26} \end{bmatrix}
$$

各矩阵元素的偏导解析通式如下（包含浮基坐标与 4 段连杆连加项）：

$$
J_{1j} = \frac{\partial x_{\text{tip}}}{\partial q_j} = \frac{\partial q_1}{\partial q_j} + \sum_{i=1}^4 \frac{\partial(l_i \cos\alpha_i)}{\partial \alpha_i}\frac{\partial \alpha_i}{\partial q_j} = \delta_{1j} + \sum_{i=1}^4 (-l_i \sin\alpha_i)\frac{\partial \alpha_i}{\partial q_j} \quad (j = 1, \dots, 6)
$$

$$
J_{2j} = \frac{\partial z_{\text{tip}}}{\partial q_j} = \frac{\partial q_2}{\partial q_j} + \sum_{i=1}^4 \frac{\partial(l_i \sin\alpha_i)}{\partial \alpha_i}\frac{\partial \alpha_i}{\partial q_j} = \delta_{2j} + \sum_{i=1}^4 (l_i \cos\alpha_i)\frac{\partial \alpha_i}{\partial q_j} \quad (j = 1, \dots, 6)
$$

具体展开到各列分量如下：

* **第 1、2 列**（\(q_1, q_2\) **浮基平动**）：
  $$
  \begin{bmatrix} J_{11} & J_{12} \\ J_{21} & J_{22} \end{bmatrix} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}
  $$
* **第 3、4 列**（\(q_3\) **俯仰角，与** \(q_4\) **髋关节角等效**）：
  $$
  \begin{aligned}
  J_{13} = J_{14} &= -(l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3 + l_4 \sin\alpha_4) \\
  J_{23} = J_{24} &= l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 + l_4 \cos\alpha_4
  \end{aligned}
  $$
* **第 5 列**（\(q_5\) **踝关节角**）：
  $$
  \begin{aligned}
  J_{15} &= -(l_2 \sin\alpha_2 + l_3 \sin\alpha_3 + l_4 \sin\alpha_4) \\
  J_{25} &= l_2 \cos\alpha_2 + l_3 \cos\alpha_3 + l_4 \cos\alpha_4
  \end{aligned}
  $$
* **第 6 列**（\(q_6\) **趾关节角**）：
  $$
  \begin{aligned}
  J_{16} &= -l_4 \sin\alpha_4 \\
  J_{26} &= l_4 \cos\alpha_4
  \end{aligned}
  $$

### 4.3 趾关节雅可比矩阵 \(J_{\text{toe}}(\mathbf{q}) \in \mathbb{R}^{2 \times 6}\)

趾关节 \(\mathbf{p}_{\text{toe}}\) 不受脚趾偏转角 \(q_6\) 影响，故其第 6 列恒为 0，前 5 列为 \(J_{\text{tip}}\) 中去除 \(l_4\) 项的结果：

$$
J_{\text{toe}}(\mathbf{q}) = \begin{bmatrix}
1 & 0 & -(l_1 s_1 + l_2 s_2 + l_3 s_3) & -(l_1 s_1 + l_2 s_2 + l_3 s_3) & -(l_2 s_2 + l_3 s_3) & 0 \\
0 & 1 & l_1 c_1 + l_2 c_2 + l_3 c_3 & l_1 c_1 + l_2 c_2 + l_3 c_3 & l_2 c_2 + l_3 c_3 & 0
\end{bmatrix}
$$

（注：上式中简记 \(s_i = \sin\alpha_i,\ c_i = \cos\alpha_i\)）。

### 4.4 雅可比矩阵时间导数 \(\dot{J}_{\text{tip}}(\mathbf{q}, \dot{\mathbf{q}}) \in \mathbb{R}^{2 \times 6}\)

足端在惯性系下的瞬时加速度映射（对应论文式 (3)）：

$$
\ddot{\mathbf{p}}_{\text{tip}} = J_{\text{tip}}(\mathbf{q})\,\ddot{\mathbf{q}} + \dot{J}_{\text{tip}}(\mathbf{q}, \dot{\mathbf{q}})\,\dot{\mathbf{q}}
$$

其中 \(\dot{J}_{\text{tip}} = \frac{d}{dt} J_{\text{tip}}(\mathbf{q})\) 包含各连杆由于转动带来的向心加速度与科氏加速度分量。

### 4.5 时间导数解析显式表达式

对 \(J_{\text{tip}}\) 每一项按时间 \(t\) 运用全微分（利用 \(\frac{d}{dt}\sin\alpha_i = \cos\alpha_i \dot{\alpha}_i\)、\(\frac{d}{dt}\cos\alpha_i = -\sin\alpha_i \dot{\alpha}_i\)）：

$$
\dot{J}_{\text{tip}}(\mathbf{q}, \dot{\mathbf{q}}) = \begin{bmatrix} 0 & 0 & \dot{J}_{13} & \dot{J}_{14} & \dot{J}_{15} & \dot{J}_{16} \\ 0 & 0 & \dot{J}_{23} & \dot{J}_{24} & \dot{J}_{25} & \dot{J}_{26} \end{bmatrix}
$$

前两列因常数求导严格为 0，其余非零元素解析式如下：

* **第 3、4 列**（\(\dot{J}_{13} = \dot{J}_{14}, \dot{J}_{23} = \dot{J}_{24}\)）：
  $$
  \begin{aligned}
  \dot{J}_{13} &= -l_1 \cos\alpha_1 \dot{\alpha}_1 - l_2 \cos\alpha_2 \dot{\alpha}_2 - l_3 \cos\alpha_3 \dot{\alpha}_3 - l_4 \cos\alpha_4 \dot{\alpha}_4 \\
  \dot{J}_{23} &= -l_1 \sin\alpha_1 \dot{\alpha}_1 - l_2 \sin\alpha_2 \dot{\alpha}_2 - l_3 \sin\alpha_3 \dot{\alpha}_3 - l_4 \sin\alpha_4 \dot{\alpha}_4
  \end{aligned}
  $$
* **第 5 列**：
  $$
  \begin{aligned}
  \dot{J}_{15} &= -l_2 \cos\alpha_2 \dot{\alpha}_2 - l_3 \cos\alpha_3 \dot{\alpha}_3 - l_4 \cos\alpha_4 \dot{\alpha}_4 \\
  \dot{J}_{25} &= -l_2 \sin\alpha_2 \dot{\alpha}_2 - l_3 \sin\alpha_3 \dot{\alpha}_3 - l_4 \sin\alpha_4 \dot{\alpha}_4
  \end{aligned}
  $$
* **第 6 列**：
  $$
  \begin{aligned}
  \dot{J}_{16} &= -l_4 \cos\alpha_4 \dot{\alpha}_4 \\
  \dot{J}_{26} &= -l_4 \sin\alpha_4 \dot{\alpha}_4
  \end{aligned}
  $$

（同理，\(\dot{J}_{\text{toe}}\) 为 \(\dot{J}_{\text{tip}}\) 中令 \(l_4 = 0\) 的结果。）

---

## 5. 分阶段无滑移地面接触约束（论文式 (2)–(5), (9)）

在触地起跳阶段，假设足部与地面无滑移（Non-slipping condition），速度与加速度满足完整约束方程（论文式 (2)、(3)）：

$$
\dot{\mathbf{p}}_c = \mathbf{J}_c(\mathbf{q})\,\dot{\mathbf{q}} = \mathbf{0}, \qquad \ddot{\mathbf{p}}_c = \mathbf{J}_c(\mathbf{q})\,\ddot{\mathbf{q}} + \dot{\mathbf{J}}_c(\mathbf{q}, \dot{\mathbf{q}})\,\dot{\mathbf{q}} = \mathbf{0}
$$

根据脚趾偏转状态，接触约束位置 \(\mathbf{p}_c\) 与约束雅可比 \(\mathbf{J}_c\) 分为两个子阶段：

1. **全足接触阶段（趾关节未达最大偏转角，论文式 (5)）**：

   趾关节与爪尖同时接触地面：

$$
\mathbf{p}_c = \begin{bmatrix} \mathbf{p}_{\text{toe}} \\ \mathbf{p}_{\text{claw}} \end{bmatrix} \in \mathbb{R}^4, \qquad \mathbf{J}_c(\mathbf{q}) = \begin{bmatrix} J_{\text{toe}}(\mathbf{q}) \\ J_{\text{tip}}(\mathbf{q}) \end{bmatrix} \in \mathbb{R}^{4 \times 6}
$$

2. **绕趾尖翻转阶段（趾关节偏转达最大行程后，论文式 (9)）**：

   趾关节抬离地面，机体绕爪尖 \(\mathbf{p}_{\text{claw}}\) 旋转：

$$
\mathbf{p}_c = \mathbf{p}_{\text{claw}} \equiv \mathbf{p}_{\text{tip}} \in \mathbb{R}^2, \qquad \mathbf{J}_c(\mathbf{q}) = J_{\text{tip}}(\mathbf{q}) \in \mathbb{R}^{2 \times 6}
$$

---

## 6. 驱动器、弹性肌腱与广义外力模型（Step 2，论文式 (11)）

作用于系统的非接触广义外力向量 \(\mathbf{f}_{\text{ext}} \in \mathbb{R}^6\) 由五部分叠加构成（论文式 (11)）：

$$
\mathbf{f}_{\text{ext}} = \mathbf{f}_p + \mathbf{f}_w + \mathbf{f}_t + \boldsymbol{\tau}_a + \boldsymbol{\tau}_t
$$

1. **主动电机驱动映射** \(\mathbf{S}^T \boldsymbol{\tau}\)：
   * 髋关节（\(q_4\)）与踝关节（\(q_5\)）由电机经减速比 \(n_{\text{gear}} = 19.13\) 驱动，选择矩阵 \(\mathbf{S}^T = [\mathbf{0}_{2\times 3},\ \mathbf{I}_{2\times 2},\ \mathbf{0}_{2\times 1}]^T \in \mathbb{R}^{6 \times 2}\)，电机输出受最大堵转扭矩 \(\tau_{\max}\) 与最大空载转速 \(\omega_{\max}\) 线性边界约束。
2. **关节扭转弹簧广义力矩** \(\boldsymbol{\tau}_a, \boldsymbol{\tau}_t\)：
   * **踝关节单向并联扭簧** \(\boldsymbol{\tau}_a\)：仅在压缩区间（\(q_5 < q_{5,0}\)）输出恢复力矩，超过自由角后完全解耦（输出为 \(0\ \text{N}\cdot\text{m}\)），作用于第 5 个广义坐标；
   * **趾关节被动扭簧** \(\boldsymbol{\tau}_t\)：与趾关节偏转角 \(q_6\) 成线性恢复关系，作用于第 6 个广义坐标。
3. **螺旋桨推力与机翼/尾翼气动外力** \(\mathbf{f}_p, \mathbf{f}_w, \mathbf{f}_t\)：
   * 机翼压心 \(\mathbf{P}_w = (P_{w,x}, P_{w,z})\) 与尾翼压心 \(\mathbf{P}_t = (P_{t,x}, P_{t,z})\) 固连于机身本体系，世界系位置为 \(\mathbf{p}_{w/t}(\mathbf{q}) = [q_1, q_2]^T + R(q_3)\mathbf{P}_{w/t}\)，通过对应点雅可比转置 \(\mathbf{J}_{w/t}^T(\mathbf{q})\) 将气动升力与阻力投影至广义坐标空间。

---

## 7. 两阶段拉格朗日动力学与任务空间控制（Step 3 & Step 4，论文式 (10), (12)–(23)）

### 7.1 触地起跳阶段受约束动力学（论文式 (10)）

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) + \mathbf{J}_c^T(\mathbf{q})\mathbf{f}_c = \mathbf{S}^T\boldsymbol{\tau} + \mathbf{f}_{\text{ext}}
$$

联立加速度级接触约束 \(\mathbf{J}_c \ddot{\mathbf{q}} = -\dot{\mathbf{J}}_c \dot{\mathbf{q}}\)，构成标准 **KKT 鞍点线性方程组**求解广义加速度 \(\ddot{\mathbf{q}}\) 与地面反作用力 \(\mathbf{f}_c\)：

$$
\begin{bmatrix} \mathbf{M}(\mathbf{q}) & \mathbf{J}_c^T(\mathbf{q}) \\ \mathbf{J}_c(\mathbf{q}) & \mathbf{0} \end{bmatrix} \begin{bmatrix} \ddot{\mathbf{q}} \\ \mathbf{f}_c \end{bmatrix} = \begin{bmatrix} \mathbf{S}^T\boldsymbol{\tau} + \mathbf{f}_{\text{ext}} - \mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}} - \mathbf{g}(\mathbf{q}) \\ -\dot{\mathbf{J}}_c(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}} \end{bmatrix}
$$

### 7.2 离地腾空阶段无约束动力学（论文式 (23)）

当足端法向地面反作用力过零（\(F_z \le 0\)）触发离地事件后，接触约束消失（\(\mathbf{f}_c = \mathbf{0}\)）：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{S}^T\boldsymbol{\tau} + \mathbf{f}_{\text{ext}}
$$

### 7.3 零空间优先级任务控制与 QR 分解力矩反解（论文式 (12)–(22)）

1. **三级任务优先级排序**：① 机身俯仰角 \(q_3\) \(\to\) ② 水平位置 \(q_1\) \(\to\) ③ 垂直位置 \(q_2\)。
2. **PD 期望任务加速度（式 (12)–(15)）**：

$$
\ddot{x}_{i,d} = k_p(x_{i,d} - x_i) + k_d(\dot{x}_{i,d} - \dot{x}_i), \qquad a_i = \ddot{x}_{i,d} - \dot{\mathbf{J}}_i\dot{\mathbf{q}}
$$

3. **零空间投影递推求** \(\ddot{\mathbf{q}}_d\) **（式 (16)–(19)）**：

   定义增广雅可比 \(\mathbf{J}_{A,i} = [\mathbf{J}_1^T, \dots, \mathbf{J}_i^T]^T\)，采用阻尼最小二乘伪逆 \(\mathbf{A}^+ = \mathbf{A}^T(\lambda \mathbf{I} + \mathbf{A}\mathbf{A}^T)^{-1}\) 计算零空间投影矩阵 \(\mathbf{N}_i = \mathbf{I} - \mathbf{J}_{A,i}^+ \mathbf{J}_{A,i}\)（\(\mathbf{N}_0 = \mathbf{I}\)），递推求解：

$$
\ddot{\mathbf{q}}_i = \ddot{\mathbf{q}}_{i-1} + \mathbf{N}_i(\mathbf{J}_i \mathbf{N}_{i-1})^+(a_i - \mathbf{J}_i \ddot{\mathbf{q}}_{i-1}), \qquad \ddot{\mathbf{q}}_0 = \mathbf{0} \implies \ddot{\mathbf{q}}_d = \ddot{\mathbf{q}}_3
$$

4. **QR 分解无约束空间反解关节力矩（式 (20)–(22)）**：

   对接触雅可比转置进行 QR 分解 \(\mathbf{J}_c^T = \mathbf{Q}[\mathbf{R}^T,\ \mathbf{0}]^T\)，通过无约束子空间选择矩阵 \(\mathbf{S}_u\) 消去接触反力 \(\mathbf{f}_c\)，求得期望电机扭矩：

$$
\boldsymbol{\tau} = (\mathbf{S}_u \mathbf{Q}^T \mathbf{S}^T)^+ \mathbf{S}_u \mathbf{Q}^T (\mathbf{M}\ddot{\mathbf{q}}_d + \mathbf{C}\dot{\mathbf{q}} + \mathbf{g})
$$

---

## 8. 论文建模步骤与项目代码模块映射（Step 1 → Step 5）

| 里程碑 | 对应论文模块与公式 | 核心源码文件 | 必选接口与验收基准（详见 `plan/milestone.md`） |
|---|---|---|---|
| **Step 1** | 坐标系、正运动学、解析雅可比及导数（式 (2)–(9)） | `src/kinematics.py` | `forward_kinematics_tip`（误差 \(<10^{-12}\ \text{m}\)）、`jacobian_tip`（误差 \(<10^{-6}\)）、`jacobian_dot_tip`（误差 \(<10^{-5}\)） |
| **Step 2** | 踝/趾关节扭簧与电机外特性（式 (11) 驱动项） | `src/actuators.py` | 单向扭簧释放过零截断（\(>q_0\) 严格为 \(0\)）、电机堵转与空载转速边界 |
| **Step 3** | 拉格朗日动力学 \(\mathbf{M}, \mathbf{C}, \mathbf{g}\) 与 KKT 接触求解（式 (10), (23)） | `src/dynamics.py` | \(\mathbf{M}(\mathbf{q})\) 对称正定、KKT 接触加速度残差 \(<10^{-10}\ \text{m/s}^2\)、无外力机械能守恒误差 \(<0.1\%\) |
| **Step 4** | 优先级控制、QR 扭矩反解与状态机切换（式 (12)–(22)） | `src/simulator.py` | `solve_ivp` 事件触发离地检测（\(\lvert F_z \rvert < 10^{-4}\ \text{N}\)）、状态连续性 |
| **Step 5** | 跳跃起飞全流程仿真与论文基准对比 | `main_benchmark.py` | 起飞时间、起飞速度（约 \(2.4\sim 2.5\ \text{m/s}\)）、质心轨迹与关节力矩曲线复现 |
