# EPFL RAVEN 动力学仿真数学规范 — 矢状面 6-DOF 运动学模型 (Step 1)

## 1. 坐标系与 6 个广义坐标定义

建立惯性世界坐标系 $O\text{-}XZ$（$X$ 轴水平向前为正，$Z$ 轴垂直向上为正，逆时针旋转为正）。
系统为**浮基四连杆矢状面模型**（机身 + 大腿杆 $l_1$ + 跗跖骨/小腿杆 $l_2$ + 脚趾杆 $l_3$），共包含 **6 个广义坐标**，符号与 `config/robot_paras.yaml` 中 `simulation.generalized_coords` 的约定（`q1 水平 / q2 竖直 / q3 俯仰 / q4 上肢角 / q5 下肢角 / q6 脚趾偏转角`）完全一致：

$$\mathbf{q} = \begin{bmatrix} q_1 & q_2 & q_3 & q_4 & q_5 & q_6 \end{bmatrix}^T \in \mathbb{R}^6$$

* $q_1$：机身本体坐标系原点（髋关节铰点）在世界坐标系下的水平位置（原符号 $x_b$）。注意：不是机身质心位置。
* $q_2$：机身本体坐标系原点（髋关节铰点）在世界坐标系下的垂直位置（原符号 $z_b$）。注意：不是机身质心位置。
* $q_3$：机身俯仰角（原符号 $\theta_b$，抬头为正）。
* $q_4$：髋关节相对转角（Hip，电机驱动）。
* $q_5$：踝关节相对转角（Ankle，电机驱动 + 并行扭簧）。
* $q_6$：脚趾关节相对转角（Toe，被动扭簧）。

> **$(q_1, q_2)$ 的物理位置**：论文 p14 式 (6) 把足端位置写为
> $\mathbf{p}_{tip} = [q_1\ q_2]^T + R(q_{4,abs})[l_1\ 0]^T + R(q_{5,abs})[l_2\ 0]^T + R(\theta_3)[l_3\ 0]^T$，
> 即 $(q_1, q_2)$ 是腿链的起点（髋关节铰点），Extended Data Fig. 5a 中 "Body frame" 的坐标十字也画在髋关节处。
> 因此机身质心**不**在该点，而在本体系 $+x_B$（机头）方向的前方，偏移量记作 $(d_x, d_z)$，见 §1.2。

> 符号一致性说明：本文档的广义坐标统一采用 $q_1\!\sim\! q_6$，与 `config/robot_paras.yaml` 的 `initial`（`pitch_deg`→$q_3$、`hip_deg`→$q_4$、`ankle_deg`→$q_5$、`toe_deflection_deg`→$q_6$）一一对应。

---

### 1.1 世界（惯性）坐标系 $O\text{-}XZ$

* **原点 $O$**：固定于地面（惯性系），不随机身运动；取初始时刻足端触地点（或任意固定参考点）为原点。
* **$X$ 轴**：沿机器人前进方向水平指向正前方。
* **$Z$ 轴**：竖直向上。
* **$Y$ 轴**：垂直于矢状面（纸面）向外，与 $X, Z$ 构成右手系。
* **正方向约定**：绕 $Y$ 轴的转角以逆时针为正（即抬头为正）。

该坐标系下描述量全部加下标 $W$ 或直接用绝对量表示：本体坐标系原点位置 $(q_1, q_2)$、机身俯仰角 $q_3$、各连杆绝对倾角 $\alpha_i$ 均相对于此坐标系度量。

### 1.2 机身本体坐标系 $B\text{-}x_B z_B$（Body Frame）

* **原点 $O_B$**：**髋关节铰点**（左右腿髋关节在矢状面内的公共投影点），即广义坐标 $(q_1, q_2)$ 所描述的点。**它不是机身质心**。
* **$x_B$ 轴**：沿机身纵轴指向头部（机头方向），固连于机身。
* **$z_B$ 轴**：在矢状面内垂直于 $x_B$，由 $x_B$ 逆时针旋转 $90^\circ$ 得到；$z_B$ 与 $x_B$、$Y$ 轴（垂直纸面向外）构成右手系，即 $Y = z_B \times x_B$。
* **姿态唯一由 $q_3$ 决定**：机身无独立横滚/偏航自由度，因此本体系相对世界系只差一个绕 $Y$ 轴的旋转 $q_3$。
* **机身质心 $C$ 的位置**：质心位于本体系内 $\mathbf{p}^{B}_{C} = (d_x, d_z)$，其中 $d_x > 0$ 表示质心在原点前方（机头方向）。论文未给出该偏移的数值，符号与量纲已确定、数值待确认（见 `doc/missing_params_log.md`）。

### 1.3 坐标变换关系

**本体系 → 世界系**（设某点在机身本体系下的坐标为 $\mathbf{p}^{B} = (p_x^{B}, p_z^{B})$）：

$$\mathbf{p}^{W} = \underbrace{\begin{bmatrix} q_1 \\ q_2 \end{bmatrix}}_{\text{本体坐标系原点（髋关节）}} + R(q_3)\,\mathbf{p}^{B}, \qquad R(q_3) = \begin{bmatrix} \cos q_3 & -\sin q_3 \\ \sin q_3 & \cos q_3 \end{bmatrix}$$

**世界系 → 本体系**（利用正交性 $R^{-1} = R^{T}$）：

$$\mathbf{p}^{B} = R^{T}(q_3)\left(\mathbf{p}^{W} - \begin{bmatrix} q_1 \\ q_2 \end{bmatrix}\right)$$

本体系坐标轴在世界系下的单位矢量即 $R(q_3)$ 的两列：

$$\mathbf{e}_{x_B}^{W} = \begin{bmatrix} \cos q_3 \\ \sin q_3 \end{bmatrix}, \qquad \mathbf{e}_{z_B}^{W} = \begin{bmatrix} -\sin q_3 \\ \cos q_3 \end{bmatrix}$$

**机身质心 $C$ 的世界系位置**（本体系中为常值 $\mathbf{p}^{B}_{C} = (d_x, d_z)$）：

$$\mathbf{p}_{C}(\mathbf{q}) = \begin{bmatrix} q_1 \\ q_2 \end{bmatrix} + R(q_3)\begin{bmatrix} d_x \\ d_z \end{bmatrix} = \begin{bmatrix} q_1 + d_x \cos q_3 - d_z \sin q_3 \\ q_2 + d_x \sin q_3 + d_z \cos q_3 \end{bmatrix}$$

**髋关节铰点**就是本体系原点，故其世界系位置直接是 $\mathbf{p}_{\text{hip}} = (q_1, q_2)$，不含任何偏移项——这与论文式 (6) 的写法一致。

> 参数状态：$(d_x, d_z)$ 的**数值**论文未给出（Extended Data Table 2 只给质量，未给质心位置），已登记为待确认项；该偏移量会在 Step 3 质量矩阵中引入"质心偏置"的耦合项，未确定前不能开始 Step 3。

### 1.4 三个坐标/角度基准的区分（易混点）

| 量 | 参考基准 | 说明 | 对应符号 |
|---|---|---|---|
| 机身质心位置 | 世界系 $O$ | 绝对位置 | $q_1, q_2$ |
| 机身俯仰角 | 世界系 $+X$ | 机身纵轴 $x_B$ 相对 $+X$ 的转角 | $q_3$ |
| 关节相对角 | 相邻连杆 | 子连杆相对父连杆的转角 | $q_4, q_5, q_6$ |
| 连杆绝对倾角 | 世界系 $-Z$（竖直向下） | 各连杆相对竖直向下方向的转角 | $\alpha_1, \alpha_2, \alpha_3$ |

> 注意：$q_3$ 的基准是 $+X$（水平向前），而 $\alpha_i$ 的基准是 $-Z$（竖直向下），两者相差 $90^\circ$；由 §2 中的 $\alpha_1 = q_3 + q_4$ 可知，$q_4$ 是关节相对角而非绝对角。

### 几何常数（均从 `config/robot_paras.yaml` 读取）

* $(d_x, d_z)$：髋关节铰点在机身本体坐标系下的固定偏移量。
* $l_1$：大腿连杆长度（髋关节到踝关节）。
* $l_2$：小腿/跗跖骨连杆长度（踝关节到脚趾关节）。
* $l_3$：脚趾连杆长度（脚趾关节到脚趾尖端触地点）。

---

## 2. 连杆绝对倾角定义（中间辅助变量）

定义各连杆相对于惯性世界系垂直向下方向（$-Z$ 轴方向）的绝对转角（逆时针为正）：

$$\begin{cases} \alpha_1 = q_3 + q_4 \\ \alpha_2 = q_3 + q_4 + q_5 \\ \alpha_3 = q_3 + q_4 + q_5 + q_6 \end{cases}$$

其中：
* $\alpha_1$ 为大腿杆绝对倾角
* $\alpha_2$ 为小腿/跗跖骨绝对倾角
* $\alpha_3$ 为脚趾杆绝对倾角

---

## 3. 正运动学方程（Forward Kinematics）

### 3.1 髋关节位置 $\mathbf{p}_{\text{hip}}$

$$\mathbf{p}_{\text{hip}}(\mathbf{q}) = \begin{bmatrix} x_{\text{hip}} \\ z_{\text{hip}} \end{bmatrix} = \begin{bmatrix} q_1 + d_x \cos q_3 - d_z \sin q_3 \\ q_2 + d_x \sin q_3 + d_z \cos q_3 \end{bmatrix}$$

### 3.2 踝关节位置 $\mathbf{p}_{\text{ankle}}$

$$\mathbf{p}_{\text{ankle}}(\mathbf{q}) = \begin{bmatrix} x_{\text{ankle}} \\ z_{\text{ankle}} \end{bmatrix} = \begin{bmatrix} x_{\text{hip}} + l_1 \sin\alpha_1 \\ z_{\text{hip}} - l_1 \cos\alpha_1 \end{bmatrix}$$

### 3.3 脚趾关节位置 $\mathbf{p}_{\text{toe}}$

$$\mathbf{p}_{\text{toe}}(\mathbf{q}) = \begin{bmatrix} x_{\text{toe}} \\ z_{\text{toe}} \end{bmatrix} = \begin{bmatrix} x_{\text{ankle}} + l_2 \sin\alpha_2 \\ z_{\text{ankle}} - l_2 \cos\alpha_2 \end{bmatrix}$$

### 3.4 脚趾尖端触地点位置 $\mathbf{p}_{\text{tip}}$

$$\mathbf{p}_{\text{tip}}(\mathbf{q}) = \begin{bmatrix} x_{\text{tip}} \\ z_{\text{tip}} \end{bmatrix} = \begin{bmatrix} x_{\text{hip}} + l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3 \\ z_{\text{hip}} - l_1 \cos\alpha_1 - l_2 \cos\alpha_2 - l_3 \cos\alpha_3 \end{bmatrix}$$

---

## 4. 足端解析雅可比矩阵 $J_{\text{tip}}(\mathbf{q}) \in \mathbb{R}^{2 \times 6}$

足端尖端线速度与广义速度的关系为 $\dot{\mathbf{p}}_{\text{tip}} = J_{\text{tip}}(\mathbf{q}) \dot{\mathbf{q}}$：

$$J_{\text{tip}}(\mathbf{q}) = \begin{bmatrix} 1 & 0 & J_{13} & J_{14} & J_{15} & J_{16} \\ 0 & 1 & J_{23} & J_{24} & J_{25} & J_{26} \end{bmatrix}$$

各元素具体解析表达式如下：

* **第 1 列（对 $q_1$ 求导）**：

$$J_{11} = 1, \quad J_{21} = 0$$

* **第 2 列（对 $q_2$ 求导）**：

$$J_{12} = 0, \quad J_{22} = 1$$

* **第 3 列（对 $q_3$ 求导）**：

$$\begin{cases} J_{13} = -d_x \sin q_3 - d_z \cos q_3 + l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 \\ J_{23} = d_x \cos q_3 - d_z \sin q_3 + l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3 \end{cases}$$

* **第 4 列（对 $q_4$ 求导）**：

$$\begin{cases} J_{14} = l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 \\ J_{24} = l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3 \end{cases}$$

* **第 5 列（对 $q_5$ 求导）**：

$$\begin{cases} J_{15} = l_2 \cos\alpha_2 + l_3 \cos\alpha_3 \\ J_{25} = l_2 \sin\alpha_2 + l_3 \sin\alpha_3 \end{cases}$$

* **第 6 列（对 $q_6$ 求导）**：

$$\begin{cases} J_{16} = l_3 \cos\alpha_3 \\ J_{26} = l_3 \sin\alpha_3 \end{cases}$$

---

## 5. 雅可比矩阵时间导数 $\dot{J}_{\text{tip}}(\mathbf{q}, \dot{\mathbf{q}}) \in \mathbb{R}^{2 \times 6}$

定义绝对角速度中间变量：

$$\dot{\alpha}_1 = \dot{q}_3 + \dot{q}_4, \quad \dot{\alpha}_2 = \dot{q}_3 + \dot{q}_4 + \dot{q}_5, \quad \dot{\alpha}_3 = \dot{q}_3 + \dot{q}_4 + \dot{q}_5 + \dot{q}_6$$

对 $J_{\text{tip}}(\mathbf{q})$ 关于时间 $t$ 求全导数：

$$\dot{J}_{\text{tip}}(\mathbf{q}, \dot{\mathbf{q}}) = \begin{bmatrix} 0 & 0 & \dot{J}_{13} & \dot{J}_{14} & \dot{J}_{15} & \dot{J}_{16} \\ 0 & 0 & \dot{J}_{23} & \dot{J}_{24} & \dot{J}_{25} & \dot{J}_{26} \end{bmatrix}$$

非零元素解析式如下：

* **第 3 列**：

$$\begin{cases} \dot{J}_{13} = (-d_x \cos q_3 + d_z \sin q_3)\dot{q}_3 - l_1 \sin\alpha_1 \dot{\alpha}_1 - l_2 \sin\alpha_2 \dot{\alpha}_2 - l_3 \sin\alpha_3 \dot{\alpha}_3 \\ \dot{J}_{23} = (-d_x \sin q_3 - d_z \cos q_3)\dot{q}_3 + l_1 \cos\alpha_1 \dot{\alpha}_1 + l_2 \cos\alpha_2 \dot{\alpha}_2 + l_3 \cos\alpha_3 \dot{\alpha}_3 \end{cases}$$

* **第 4 列**：

$$\begin{cases} \dot{J}_{14} = -l_1 \sin\alpha_1 \dot{\alpha}_1 - l_2 \sin\alpha_2 \dot{\alpha}_2 - l_3 \sin\alpha_3 \dot{\alpha}_3 \\ \dot{J}_{24} = l_1 \cos\alpha_1 \dot{\alpha}_1 + l_2 \cos\alpha_2 \dot{\alpha}_2 + l_3 \cos\alpha_3 \dot{\alpha}_3 \end{cases}$$

* **第 5 列**：

$$\begin{cases} \dot{J}_{15} = -l_2 \sin\alpha_2 \dot{\alpha}_2 - l_3 \sin\alpha_3 \dot{\alpha}_3 \\ \dot{J}_{25} = l_2 \cos\alpha_2 \dot{\alpha}_2 + l_3 \cos\alpha_3 \dot{\alpha}_3 \end{cases}$$

* **第 6 列**：

$$\begin{cases} \dot{J}_{16} = -l_3 \sin\alpha_3 \dot{\alpha}_3 \\ \dot{J}_{26} = l_3 \cos\alpha_3 \dot{\alpha}_3 \end{cases}$$

---

## 6. 各连杆质心（CoM）正运动学（供 Step 3 质量矩阵使用）

设三段腿杆质心分别距离各自近端关节为 $c_1, c_2, c_3$：

* **大腿质心**：

$$\mathbf{p}_{c1} = \begin{bmatrix} x_{\text{hip}} + c_1 \sin\alpha_1 \\ z_{\text{hip}} - c_1 \cos\alpha_1 \end{bmatrix}$$

* **小腿/跗跖骨质心**：

$$\mathbf{p}_{c2} = \begin{bmatrix} x_{\text{ankle}} + c_2 \sin\alpha_2 \\ z_{\text{ankle}} - c_2 \cos\alpha_2 \end{bmatrix}$$

* **脚趾质心**：

$$\mathbf{p}_{c3} = \begin{bmatrix} x_{\text{toe}} + c_3 \sin\alpha_3 \\ z_{\text{toe}} - c_3 \cos\alpha_3 \end{bmatrix}$$
