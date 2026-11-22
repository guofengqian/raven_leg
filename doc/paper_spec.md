# EPFL RAVEN 动力学仿真数学规范 — 矢状面 6-DOF 完整建模体系

> 本规范严格对齐论文 *Fast ground-to-air transition with avian-inspired multifunctional legs* 的 **Methods — Dynamics modeling and simulation（pp. 13–16，式 (2)–(23)）**，并与 `plan/milestone.md`（Step 1 → Step 5）保持一致。
>
> * **单一事实来源**：物理量的数值与单位只写在 `config/robot_paras.yaml`；本文只给符号、公式与键名映射。验收阈值只写在 `plan/milestone.md`。
> * **符号优先**：符号含义以 §0.2 为准；同一公式只在一处定稿，别处写「见 §x.y」。
> * **历史命名**：YAML 中 `P_wz_m` / `P_tz_m` 和 `d_z` 的 `z` 表示**竖直方向**（旧约定）；本文竖直轴为 \(+Y\)，故写作 \(P_{w,y}\)、\(d_y\)，数值与量纲不变，映射见 §0.5。`plan/milestone.md` 中的 \(F_z\) 同样指竖直地面反力，对应本文 \(F_y\)。
> * **待确认项**：论文未公开的参数一律登记在 `doc/missing_params_log.md`，本文不填任何经验值。

## 0. 读法与符号速查

### 0.1 读法

主文给实现接口公式；推导与参数论证放附录。

| 章节 | 角色 | 完成度 |
|---|---|---|
| §1–§4 | 坐标系、旋转、串联拓扑、倾角 | Step 1 定稿 |
| §5 | 正运动学位置 / 质心 | Step 1 定稿 |
| §6 | \(J\)、\(\dot{J}\) 成品公式（实现主对照） | Step 1 定稿 |
| §7 | 两阶段接触约束（7.1–7.3） | Step 1 接口已定 |
| §8 | 驱动器、弹性肌腱与广义外力（8.1–8.3） | Step 2 符号级 |
| §9 | 动力学与任务控制（9.1–9.4） | Step 3–4 符号级 |
| §10 | 里程碑 ↔ 源码 ↔ 验收 | 工程映射（阈值见 milestone） |
| 附录 A | 雅可比推导增量 + 极限工况 | 旁路；支撑 `tests/test_step1.py` |
| 附录 B | \((d_x,d_y)\) 可行域论证 | 参数旁路 |

* **Step 1**：§0.2 → §0.3 → §0.4 → §1 → §2 → §3 → §4 → §5 → §6 → §7 → §10；\((d_x,d_y)\) 见附录 B；推导见附录 A。
* **Step 2+**：§8–§9 + `plan/milestone.md`。

### 0.2 符号速查

| 类别 | 符号 | 含义 |
|---|---|---|
| 坐标系 | \(O\) | 世界系原点：水平地面上、初始构型尾巴末端投影点（Extended Data Fig. 5a Base Frame） |
| 坐标系 | \(+X,+Y,+Z\) | 世界系：前进水平、竖直向上、矢状面向外；绕 \(Z\) 逆时针（抬头）为正 |
| 坐标系 | \(O_B\) | 本体系原点：左右髋关节连线中点（矢状面投影）；**不是**机身质心 |
| 坐标系 | \(+x_B,+y_B,+z_B\) | 本体系：沿机身纵轴朝头；矢状面内由 \(+x_B\) 绕 \(+Z\) 逆时针 \(90^\circ\)；与世界 \(+Z\) 平行 |
| 坐标系 | \(B_1\sim B_4\) | 固连于各连杆近端铰点的局部系；轴向约定同本体系（\(+x\) 沿杆指向远端） |
| 坐标系 | \(J_1\sim J_4\) | 髋、踝、偏移基点、趾关节铰点；\(J_1\equiv O_B\) |
| 旋转矩阵 | \(R_Z(\theta)\in SO(3)\) | 绕面外轴 \(Z\) 转 \(\theta\) 的三维旋转矩阵 |
| 旋转矩阵 | \(R(\theta)\) | \(R_Z(\theta)\) 在 \(XY\) 平面的 \(2\times 2\) 抽取矩阵 |
| 旋转矩阵 | \({}^{W}R_{B}\) | 本体系到世界系的旋转矩阵，等于 \(R_Z(q_3)\) |
| 广义坐标 | \(\mathbf{q}\in\mathbb{R}^6\) | \((q_1,\ldots,q_6)^T\)，与 `config/robot_paras.yaml` 一致 |
| 广义坐标 | \(q_1\equiv x_{\text{hip}}\) | 髋点世界系水平位置；浮基平动，无驱动 |
| 广义坐标 | \(q_2\equiv y_{\text{hip}}\) | 髋点世界系竖直位置；浮基平动，无驱动 |
| 广义坐标 | \(q_3\equiv \theta_{\text{body}}\) | 机身纵轴相对世界 \(+X\) 的俯仰角；抬头为正；浮基转动，无驱动 |
| 广义坐标 | \(q_4\equiv q_{\text{hip}}\) | 髋相对转角（大腿相对机身纵轴）；电机驱动 |
| 广义坐标 | \(q_5\equiv q_{\text{ankle}}\) | 踝相对转角（下肢相对大腿）；电机驱动 + 并行储能扭簧 |
| 广义坐标 | \(q_6\equiv q_{\text{toe}}\) | 趾相对偏转角（脚趾相对偏移杆）；被动弹性扭簧 |
| 几何 | \(l_1\) | 上肢（大腿）长：\(J_1\to J_2\) |
| 几何 | \(l_2\) | 下肢（跗跖骨）长：\(J_2\to J_3\) |
| 几何 | \(l_3\) | 偏移杆长：\(J_3\to J_4\) |
| 几何 | \(l_4\) | 脚趾杆长：\(J_4\to\mathbf{p}_{\text{tip}}\) |
| 几何 | \(\theta_3\) | 趾关节固定安装角（\(l_2\) 与 \(l_3\) 夹角，`theta3_deg=45^\circ`）；非自由度，\(\dot{\theta}_3=0\) |
| 几何 | \((d_x,d_y)\) | 机身质心 \(C\) 在本体系 \(B\) 下的偏移；可行域与论证见附录 B |
| 几何 | \(\mathbf{v}_i\) | 第 \(i\) 段连杆在局部系下的向量 \([l_i,0,0]^T\) |
| 几何 | \(c_i\) | 第 \(i\) 段腿杆质心距其近端铰点的距离（待定，见 `doc/missing_params_log.md`） |
| 倾角 | \(\alpha_i\) | 第 \(i\) 段连杆相对世界 \(+X\) 的逆时针绝对倾角（\(i=1..4\) 对应大腿、下肢、偏移杆、脚趾） |
| 倾角 | \(\dot{\alpha}_i\) | \(\alpha_i\) 对时间的导数（绝对角速度） |
| 位置 | \(\mathbf{p}^{3\text{D}},\ \mathbf{p}\) | 三维位置 \([x,y,0]^T\)；平面工作向量 \([x,y]^T\)（\(z\equiv 0\)） |
| 位置 | \(\mathbf{p}_1\equiv\mathbf{p}_{\text{hip}}\) | 髋关节世界系位置（\(J_1\)） |
| 位置 | \(\mathbf{p}_2\equiv\mathbf{p}_{\text{ankle}}\) | 踝关节世界系位置（\(J_2\)） |
| 位置 | \(\mathbf{p}_3\equiv\mathbf{p}_{\text{base}}\) | 偏移杆基点世界系位置（\(J_3\)） |
| 位置 | \(\mathbf{p}_4\equiv\mathbf{p}_{\text{toe}}\) | 趾关节世界系位置（\(J_4\)；论文式 (6)） |
| 位置 | \(\mathbf{p}_{\text{tip}}\equiv\mathbf{p}_{\text{claw}}\) | 足端爪尖世界系位置（代码 `tip`，论文 `claw`；式 (7)） |
| 位置 | \(\mathbf{p}_C\) | 机身质心世界系位置 |
| 位置 | \(\mathbf{p}_{ci}\) | 第 \(i\) 段腿杆质心世界系位置 |
| 位置 | \(\mathbf{P}_w,\mathbf{P}_t\) | 机翼 / 尾翼压心在本体系下的坐标 \((P_{*,x},P_{*,y})\) |
| 位置 | \(\mathbf{p}_{w/t}(\mathbf{q})\) | 机翼 / 尾翼压心的世界系位置 |
| 位置 | \(y_{\text{NP}}\) | 全机气动中立点高度（面积加权估算，见附录 B） |
| 雅可比 | \(J_{\text{tip}}(\mathbf{q})\) | 爪尖雅可比 \(\partial\mathbf{p}_{\text{tip}}/\partial\mathbf{q}\in\mathbb{R}^{2\times 6}\) |
| 雅可比 | \(J_{\text{toe}}(\mathbf{q})\) | 趾关节雅可比；第 6 列恒 0 |
| 雅可比 | \(\dot{J}_{\text{tip}},\dot{J}_{\text{toe}}\) | 对应雅可比对时间的全导数 |
| 雅可比 | \(J_{ij},\ \dot{J}_{ij}\) | \(J_{\text{tip}}\) / \(\dot{J}_{\text{tip}}\) 的第 \((i,j)\) 元素 |
| 接触 | \(\mathbf{p}_c\) | 接触约束位置（全足阶段堆叠趾+爪，或绕尖阶段仅爪尖） |
| 接触 | \(\mathbf{J}_c\) | 接触约束雅可比（全足 \(4\times 6\) 或绕尖 \(2\times 6\)） |
| 接触 | \(\mathbf{f}_c\) | 接触约束反力向量（全足 \(\mathbb{R}^4\)、绕尖 \(\mathbb{R}^2\)），含法向与切向分量 |
| 接触 | \(F_x,F_y\) | 地面对足端的切向 / 法向反力；\(+Y\) 竖直向上 |
| 接触 | \(q_{6,\max}\) | 趾关节最大偏转（全足 → 绕尖切换判据） |
| 接触 | \(\mu\) | 足地静摩擦系数（Step 4 摩擦锥校验） |
| 外力 | \(\mathbf{f}_{\text{ext}}\) | 非接触广义外力 \(\in\mathbb{R}^6\) |
| 外力 | \(\mathbf{f}_p,\mathbf{f}_w,\mathbf{f}_t\) | 螺旋桨推力、机翼气动力、尾翼气动力（投影至广义坐标） |
| 外力 | \(\boldsymbol{\tau}_a\) | 踝关节单向并联扭簧广义力矩（作用于 \(q_5\)；仅 \(q_5<q_{5,0}\) 非零） |
| 外力 | \(\boldsymbol{\tau}_t\) | 趾关节被动扭簧广义力矩（作用于 \(q_6\)） |
| 外力 | \(\mathbf{F}_{w/t}\) | 机翼 / 尾翼气动力（压心处的二维力向量） |
| 气动 | \(\rho,S_w,S_t\) | 空气密度、机翼 / 尾翼面积 |
| 外力 | \(q_{5,0},q_{6,0}\) | 踝 / 趾扭簧自由角；踝簧在 \(q_5\ge q_{5,0}\) 时输出截断为 0 |
| 驱动 | \(\boldsymbol{\tau}\) | 电机输出力矩（髋、踝两通道） |
| 驱动 | \(\mathbf{S}^T\) | 驱动选择矩阵 \(\in\mathbb{R}^{6\times 2}\)，把 \(\boldsymbol{\tau}\) 映到 \(q_4,q_5\) |
| 驱动 | \(n_{\text{gear}}\) | 减速比（数值见 YAML `actuators.gearbox.ratio`） |
| 驱动 | \(\tau_{\max},\omega_{\max}\) | 电机最大堵转扭矩、最大空载转速（YAML 无直接键，Step 2 前需登记推导来源） |
| 驱动 | \(\mathbf{J}_{w/t}\) | 机翼 / 尾翼压心处的点雅可比 |
| 动力学 | \(m_B,m_i\) | 机身 / 第 \(i\) 段腿杆质量（\(i=1..4\)，数值见 YAML） |
| 动力学 | \(I_B,I_i\) | 机身 / 第 \(i\) 段腿杆绕自身质心的转动惯量（待定） |
| 动力学 | \(\mathbf{M}(\mathbf{q})\) | 质量矩阵 |
| 动力学 | \(\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\) | 科氏 / 离心项 |
| 动力学 | \(\mathbf{g}(\mathbf{q})\) | 重力广义力 |
| 控制 | \(\ddot{x}_{i,d},a_i\) | 第 \(i\) 级任务期望加速度及补偿后的任务加速度 |
| 控制 | \(k_p,k_d\) | 任务空间 PD 增益 |
| 控制 | \(\mathbf{J}_i,\mathbf{J}_{A,i}\) | 第 \(i\) 级任务雅可比及其增广堆叠 |
| 控制 | \(\mathbf{N}_i\) | 第 \(i\) 级零空间投影矩阵（\(\mathbf{N}_0=\mathbf{I}\)） |
| 控制 | \(\ddot{\mathbf{q}}_d\) | 期望广义加速度（三级任务递推结果） |
| 控制 | \(\mathbf{Q},\mathbf{R}\) | 对 \(\mathbf{J}_c^T\) 作 QR 分解得到的因子 |
| 控制 | \(\mathbf{S}_u\) | 无约束子空间选择矩阵（消去 \(\mathbf{f}_c\)） |
| 控制 | \(\lambda\) | 阻尼最小二乘伪逆的阻尼系数 |

### 0.3 论文式号 ↔ 章节索引

| 论文式号 | 内容 | 本文位置 |
|---|---|---|
| (2) | 接触点速度约束 \(\dot{\mathbf{p}}_c=\mathbf{J}_c\dot{\mathbf{q}}=\mathbf{0}\) | §7 |
| (3) | 接触点加速度约束 | §7 |
| (4) | 足端速度与雅可比定义 | §6.1 |
| (5) | 全足接触约束 | §7.1 |
| (6) | 趾关节位置 \(\mathbf{p}_{\text{toe}}\) | §5.1 |
| (7) | 爪尖位置 \(\mathbf{p}_{\text{tip}}\) | §5.2 |
| (8) | 各构件质心位置 | §5.3 |
| (9) | 绕趾尖翻转约束 | §7.2 |
| (10) | 触地受约束动力学 / KKT | §9.2 |
| (11) | 广义外力 \(\mathbf{f}_{\text{ext}}\) | §8 |
| (12)–(15) | 任务空间 PD 期望加速度 | §9.4 |
| (16)–(19) | 零空间优先级递推 | §9.4 |
| (20)–(22) | QR 分解力矩反解 | §9.4 |
| (23) | 腾空无约束动力学 | §9.3 |

### 0.4 建模假设与适用范围

1. **刚体假设**：机身与 4 段腿杆共 5 个刚体；铰接视为理想转动副，无摩擦、无间隙、无柔性。
2. **平面假设**：全部运动限制在矢状面内，\(z\equiv0\)；所有关节转轴平行于 \(Z\)，侧向、滚转与偏航运动不在模型范围内。`plan/milestone.md` 习惯称该平面为 \(x\text{-}z\) 平面，其 \(z\) 指竖直方向，对应本文 \(Y\)。
3. **左右腿合并**：平面模型把左右腿等效为一条腿，质量按 YAML 的 `simulation.links` 简化模型取值。
4. **足地接触**：地面刚性且水平；约束层面按无滑移处理，法向单向 \(F_y\ge0\)，切向静摩擦锥 \(\lvert F_x\rvert\le\mu F_y\) 由 Step 4 事后校验（约束求解本身不显式含摩擦项）。
5. **弹簧与电机**：线性扭转弹簧、无阻尼，踝簧单向；电机视为理想力矩源，仅受转速—力矩线性外特性限制，不含电气动态。
6. **气动**：准定常模型，压心固连机身；升阻力系数与安装角取自论文 / YAML，不做非定常、地面效应与柔性翼建模。
7. **适用范围**：只覆盖蹬地起跳与起飞过渡的矢状面动力学，不包含着陆缓冲、空中机动与多腿步态。

### 0.5 参数符号 ↔ YAML 键名

数值与单位以 `config/robot_paras.yaml` 为准；本表只登记键名与换算，不复制数值。

| 符号 | YAML 键 | 单位 | 备注 |
|---|---|---|---|
| \(l_1\sim l_4\) | `simulation.geometry.l1_m` … `l4_m` | m | |
| \(\theta_3\) | `simulation.geometry.theta3_deg` | deg | 使用前转 rad |
| \(m_B\) | `simulation.links.body_mass_kg` | kg | 简化刚体 |
| \(m_1\sim m_4\) | `simulation.links.upper_limb_mass_kg`、`lower_limb_mass_kg`、`palm_mass_kg`、`toe_mass_kg` | kg | 顺序对应 \(l_1\sim l_4\) |
| \(k_a\) | `simulation.spring.ankle_constant_Nmm_per_deg` | N·mm/deg | 换算 N·m/rad 时乘 \((180/\pi)\times10^{-3}\) |
| \(k_t\) | `simulation.spring.toe_constant_Nmm_per_deg` | N·mm/deg | 同上 |
| \(n_{\text{gear}}\) | `actuators.gearbox.ratio` | — | 电机轴 → 关节侧 |
| \(\rho,S_w,S_t\) | `simulation.aerodynamics.air_density_kg_per_m3`、`wing_area_m2`、`tail_area_m2` | kg/m³, m² | |
| \(\mathbf{P}_w,\mathbf{P}_t\) | `simulation.reference_points.P_wx_m`、`P_wz_m`、`P_tx_m`、`P_tz_m` | m | 键名 `_z` 表示竖直，对应本文 \(P_{w,y}\) |
| \(g\) | `simulation.gravity_m_per_s2` | m/s² | 重力向量 \([0,-g]^T\) |
| \(\mathbf{q}_0\) | `simulation.initial.pitch_deg`、`hip_deg`、`ankle_deg`、`toe_deflection_deg` | deg | 使用前转 rad |
| \((d_x,d_y)\) | 无数值键（注释中写作 `d_x` / `d_z`） | m | 待定，见 `doc/missing_params_log.md` |
| \(c_i,\ I_B,\ I_i\) | 无键 | — | 待定，见 `doc/missing_params_log.md` |
| \(q_{5,0},q_{6,0}\) | 无键 | rad | 待定，实现前登记 |
| \(\tau_{\max},\omega_{\max}\) | 无直接键 | — | 由 `actuators.leg_motor.*` 推导，Step 2 前登记 |
| \(\mu\) | 无键 | — | 待定，Step 4 前登记 |

---

## 1. 坐标系 (Coordinate Frames)

符号见 §0.2。本节只固定「有哪些架、原点落在哪类几何点」；轴向细节见表，旋转公式见 §2。

* **世界系** \(O\text{-}XYZ\)：原点 \(O\) 在地面（初始尾巴末端投影）；\(+X\) 前进、\(+Y\) 竖直向上、\(+Z\) 出矢状面。
* **本体系** \(B\)：原点 \(O_B\) 为两髋连线中点（非质心）；轴固连机身。
* **连杆局部系** \(B_1\sim B_4\)：原点分别在 \(J_1\sim J_4\)（近端铰点）；\(B_1\) 与 \(O_B\) 重合。

| 局部系 | 固连连杆 | 原点 |
|---|---|---|
| \(B_1\) | 上肢 \(l_1\) | \(J_1\) |
| \(B_2\) | 下肢 \(l_2\) | \(J_2\) |
| \(B_3\) | 偏移杆 \(l_3\) | \(J_3\) |
| \(B_4\) | 脚趾 \(l_4\) | \(J_4\) |

---

## 2. 旋转约定与平面约化

所有关节转轴平行于 \(Z\)；姿态为绕 \(Z\) 的三维旋转（非平面内二维旋转）。符号见 §0.2。

$$
R_Z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix} \in SO(3)
$$

\(R_Z(\theta)\) 把 \(\mathbf{e}_{x}=[1,0,0]^T\) 映到 \([\cos\theta,\ \sin\theta,\ 0]^T\)。本体系取 \(\theta=q_3\)，故 \({}^{W}R_{B}=R_Z(q_3)\)；第 \(i\) 段连杆局部系取 \(\theta=\alpha_i\)。

仿真约束：\(z\equiv 0\)，速度 / 加速度的 \(Z\) 分量恒为零。平面抽取：

$$
\begin{bmatrix} x \\ y \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix} R_Z(\theta)\begin{bmatrix} x_B \\ y_B \\ 0 \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}\begin{bmatrix} x_B \\ y_B \end{bmatrix}
$$

后文 \(R(\theta)\) 均指该 \(2\times 2\) 抽取；底层仍是 \(R_Z(\theta)\)。

---

## 3. 串联运动链拓扑

\(\mathbf{q}\)、\(l_i\)、\(\theta_3\)、\(\mathbf{v}_i\) 等含义见 §0.2；数值见 §0.5 / YAML；\((d_x,d_y)\) 可行域见附录 B。

$$
O_B \equiv J_1 \xrightarrow{l_1} J_2 \xrightarrow{l_2} J_3 \xrightarrow{l_3} J_4 \xrightarrow{l_4} \mathbf{p}_{\text{tip}}
$$

| 段 | 近端 | 连杆 | 远端 | 相对前一段转角 |
|---|---|---|---|---|
| 1 | \(J_1\) | \(l_1\) | \(J_2\) | \(q_4\) |
| 2 | \(J_2\) | \(l_2\) | \(J_3\) | \(q_5\) |
| 3 | \(J_3\) | \(l_3\) | \(J_4\) | \(\theta_3\)（固定） |
| 4 | \(J_4\) | \(l_4\) | \(\mathbf{p}_{\text{tip}}\) | \(q_6\) |

\(q_3\) 为整链世界系基准朝向。§4 写 \(\alpha_i\)；§5 沿上表累加正运动学。

---

## 4. 连杆绝对倾角

按 §3 拓扑叠加：

$$
\begin{aligned}
\alpha_1 &= q_3 + q_4 \\
\alpha_2 &= \alpha_1 + q_5 = q_3 + q_4 + q_5 \\
\alpha_3 &= \alpha_2 + \theta_3 = q_3 + q_4 + q_5 + \theta_3 \\
\alpha_4 &= \alpha_3 + q_6 = q_3 + q_4 + q_5 + \theta_3 + q_6
\end{aligned}
$$

$$
\mathbf{v}_i^{W} = R_Z(\alpha_i)\,\mathbf{v}_i = [l_i\cos\alpha_i,\ l_i\sin\alpha_i,\ 0]^T
$$

平面分量：\(R(\alpha_i)[l_i,\ 0]^T\)。因 \(\dot{\theta}_3=0\)：

$$
\begin{aligned}
\dot{\alpha}_1 &= \dot{q}_3 + \dot{q}_4 \\
\dot{\alpha}_2 &= \dot{q}_3 + \dot{q}_4 + \dot{q}_5 \\
\dot{\alpha}_3 &= \dot{\alpha}_2 = \dot{q}_3 + \dot{q}_4 + \dot{q}_5 \\
\dot{\alpha}_4 &= \dot{\alpha}_3 + \dot{q}_6 = \dot{q}_3 + \dot{q}_4 + \dot{q}_5 + \dot{q}_6
\end{aligned}
$$

---

## 5. 正运动学（论文式 (6)–(8)）

位置与质心方程。雅可比成品见 §6；推导增量见附录 A。

### 5.1 铰点位置

递推 \(\mathbf{p}_{i+1}^{3\text{D}}=\mathbf{p}_i^{3\text{D}}+R_Z(\alpha_i)\mathbf{v}_i\)；下文写平面分量。

$$
\mathbf{p}_1 = \begin{bmatrix} q_1 \\ q_2 \end{bmatrix}
$$

$$
\mathbf{p}_2 = \mathbf{p}_1 + R(\alpha_1)\begin{bmatrix} l_1 \\ 0 \end{bmatrix} = \begin{bmatrix} q_1 + l_1 \cos\alpha_1 \\ q_2 + l_1 \sin\alpha_1 \end{bmatrix}
$$

$$
\mathbf{p}_3 = \mathbf{p}_2 + R(\alpha_2)\begin{bmatrix} l_2 \\ 0 \end{bmatrix} = \begin{bmatrix} q_1 + l_1 \cos\alpha_1 + l_2 \cos\alpha_2 \\ q_2 + l_1 \sin\alpha_1 + l_2 \sin\alpha_2 \end{bmatrix}
$$

$$
\mathbf{p}_{\text{toe}}(\mathbf{q}) \equiv \mathbf{p}_4 = \mathbf{p}_3 + R(\alpha_3)\begin{bmatrix} l_3 \\ 0 \end{bmatrix} = \begin{bmatrix} q_1 + l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 \\ q_2 + l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3 \end{bmatrix}
$$

（论文 p14 式 (6)。）

### 5.2 足端爪尖 \(\mathbf{p}_{\text{tip}}\)（论文式 (7)）

$$
\mathbf{p}_{\text{tip}}(\mathbf{q}) \equiv \mathbf{p}_{\text{claw}}(\mathbf{q}) = \mathbf{p}_{\text{toe}}(\mathbf{q}) + R(\alpha_4)\begin{bmatrix} l_4 \\ 0 \end{bmatrix}
$$

$$
\begin{aligned}
x_{\text{tip}}(\mathbf{q}) &= q_1 + l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 + l_4 \cos\alpha_4 \\
y_{\text{tip}}(\mathbf{q}) &= q_2 + l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3 + l_4 \sin\alpha_4
\end{aligned}
$$

### 5.3 质心位置

用于 Step 3 动能 / 重力势能。\((d_x,d_y)\) 可行域见附录 B。

$$
\mathbf{p}_C(\mathbf{q}) = \begin{bmatrix} q_1 \\ q_2 \end{bmatrix} + R(q_3)\begin{bmatrix} d_x \\ d_y \end{bmatrix} = \begin{bmatrix} q_1 + d_x \cos q_3 - d_y \sin q_3 \\ q_2 + d_x \sin q_3 + d_y \cos q_3 \end{bmatrix}
$$

$$
\begin{aligned}
\mathbf{p}_{c1} &= \mathbf{p}_1 + R(\alpha_1)[c_1, 0]^T \\
\mathbf{p}_{c2} &= \mathbf{p}_2 + R(\alpha_2)[c_2, 0]^T \\
\mathbf{p}_{c3} &= \mathbf{p}_3 + R(\alpha_3)[c_3, 0]^T \\
\mathbf{p}_{c4} &= \mathbf{p}_4 + R(\alpha_4)[c_4, 0]^T
\end{aligned}
$$

---

## 6. 解析雅可比与 \(\dot{J}\)（论文式 (2)–(4)）

> **实现主对照**；列几何与极限检验见附录 A。成品矩阵只在本节给出。

### 6.1 \(J_{\text{tip}}\)（式 (2)、(4)）

$$
\dot{\mathbf{p}}_{\text{tip}} = J_{\text{tip}}(\mathbf{q})\,\dot{\mathbf{q}}, \qquad
J_{\text{tip}}(\mathbf{q}) = \begin{bmatrix} 1 & 0 & J_{13} & J_{14} & J_{15} & J_{16} \\ 0 & 1 & J_{23} & J_{24} & J_{25} & J_{26} \end{bmatrix}
$$

通式：

$$
J_{1j} = \delta_{1j} + \sum_{i=1}^4 (-l_i \sin\alpha_i)\frac{\partial \alpha_i}{\partial q_j}, \qquad
J_{2j} = \delta_{2j} + \sum_{i=1}^4 (l_i \cos\alpha_i)\frac{\partial \alpha_i}{\partial q_j}
$$

各列：

$$
\begin{bmatrix} J_{11} & J_{12} \\ J_{21} & J_{22} \end{bmatrix} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}
$$

$$
\begin{aligned}
J_{13} = J_{14} &= -(l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3 + l_4 \sin\alpha_4) \\
J_{23} = J_{24} &= l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 + l_4 \cos\alpha_4
\end{aligned}
$$

$$
\begin{aligned}
J_{15} &= -(l_2 \sin\alpha_2 + l_3 \sin\alpha_3 + l_4 \sin\alpha_4) \\
J_{25} &= l_2 \cos\alpha_2 + l_3 \cos\alpha_3 + l_4 \cos\alpha_4
\end{aligned}
$$

$$
\begin{aligned}
J_{16} &= -l_4 \sin\alpha_4 \\
J_{26} &= l_4 \cos\alpha_4
\end{aligned}
$$

### 6.2 \(J_{\text{toe}}\)

第 6 列恒 0；前 5 列为 \(J_{\text{tip}}\) 去掉 \(l_4\)：

$$
J_{\text{toe}}(\mathbf{q}) = \begin{bmatrix}
1 & 0 & -(l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3) & -(l_1 \sin\alpha_1 + l_2 \sin\alpha_2 + l_3 \sin\alpha_3) & -(l_2 \sin\alpha_2 + l_3 \sin\alpha_3) & 0 \\
0 & 1 & l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 & l_1 \cos\alpha_1 + l_2 \cos\alpha_2 + l_3 \cos\alpha_3 & l_2 \cos\alpha_2 + l_3 \cos\alpha_3 & 0
\end{bmatrix}
$$

### 6.3 \(\dot{J}_{\text{tip}}\) 与加速度映射（式 (3)）

$$
\ddot{\mathbf{p}}_{\text{tip}} = J_{\text{tip}}(\mathbf{q})\,\ddot{\mathbf{q}} + \dot{J}_{\text{tip}}(\mathbf{q}, \dot{\mathbf{q}})\,\dot{\mathbf{q}}
$$

$$
\dot{J}_{\text{tip}}(\mathbf{q}, \dot{\mathbf{q}}) = \begin{bmatrix} 0 & 0 & \dot{J}_{13} & \dot{J}_{14} & \dot{J}_{15} & \dot{J}_{16} \\ 0 & 0 & \dot{J}_{23} & \dot{J}_{24} & \dot{J}_{25} & \dot{J}_{26} \end{bmatrix}
$$

$$
\begin{aligned}
\dot{J}_{13} = \dot{J}_{14} &= -l_1 \cos\alpha_1 \dot{\alpha}_1 - l_2 \cos\alpha_2 \dot{\alpha}_2 - l_3 \cos\alpha_3 \dot{\alpha}_3 - l_4 \cos\alpha_4 \dot{\alpha}_4 \\
\dot{J}_{23} = \dot{J}_{24} &= -l_1 \sin\alpha_1 \dot{\alpha}_1 - l_2 \sin\alpha_2 \dot{\alpha}_2 - l_3 \sin\alpha_3 \dot{\alpha}_3 - l_4 \sin\alpha_4 \dot{\alpha}_4
\end{aligned}
$$

$$
\begin{aligned}
\dot{J}_{15} &= -l_2 \cos\alpha_2 \dot{\alpha}_2 - l_3 \cos\alpha_3 \dot{\alpha}_3 - l_4 \cos\alpha_4 \dot{\alpha}_4 \\
\dot{J}_{25} &= -l_2 \sin\alpha_2 \dot{\alpha}_2 - l_3 \sin\alpha_3 \dot{\alpha}_3 - l_4 \sin\alpha_4 \dot{\alpha}_4
\end{aligned}
$$

$$
\begin{aligned}
\dot{J}_{16} &= -l_4 \cos\alpha_4 \dot{\alpha}_4 \\
\dot{J}_{26} &= -l_4 \sin\alpha_4 \dot{\alpha}_4
\end{aligned}
$$

\(\dot{J}_{\text{toe}}\)：在 \(\dot{J}_{\text{tip}}\) 中令 \(l_4=0\)。

---

## 7. 分阶段无滑移地面接触约束（论文式 (2)–(5), (9)）

$$
\dot{\mathbf{p}}_c = \mathbf{J}_c(\mathbf{q})\,\dot{\mathbf{q}} = \mathbf{0}, \qquad \ddot{\mathbf{p}}_c = \mathbf{J}_c(\mathbf{q})\,\ddot{\mathbf{q}} + \dot{\mathbf{J}}_c(\mathbf{q}, \dot{\mathbf{q}})\,\dot{\mathbf{q}} = \mathbf{0}
$$

### 7.1 全足接触（趾未达最大偏转，式 (5)）

趾关节与爪尖同时接触地面：

$$
\mathbf{p}_c = \begin{bmatrix} \mathbf{p}_{\text{toe}} \\ \mathbf{p}_{\text{claw}} \end{bmatrix} \in \mathbb{R}^4, \qquad \mathbf{J}_c(\mathbf{q}) = \begin{bmatrix} J_{\text{toe}}(\mathbf{q}) \\ J_{\text{tip}}(\mathbf{q}) \end{bmatrix} \in \mathbb{R}^{4 \times 6}
$$

### 7.2 绕趾尖翻转（趾达最大行程，式 (9)）

趾关节抬离地面，机体绕爪尖 \(\mathbf{p}_{\text{claw}}\) 旋转：

$$
\mathbf{p}_c = \mathbf{p}_{\text{claw}} \equiv \mathbf{p}_{\text{tip}} \in \mathbb{R}^2, \qquad \mathbf{J}_c(\mathbf{q}) = J_{\text{tip}}(\mathbf{q}) \in \mathbb{R}^{2 \times 6}
$$

### 7.3 状态切换判据

* **全足 → 绕尖**：\(q_6\) 达到上限 \(q_{6,\max}\)，且 \(\dot{q}_6\) 仍指向偏转增大方向；
* **绕尖 → 离地**：法向反力过零，\(F_y \le 0\)；
* **离地 → 着陆**：足端高度 \(y_{\text{tip}} \le 0\)（地面为 \(y=0\)）。

切换在 `solve_ivp` 中由 `events` 实现；事件符号与触发方向以 `plan/milestone.md` 为准。

---

## 8. 驱动器、弹性肌腱与广义外力（Step 2 符号级，式 (11)）

$$
\mathbf{f}_{\text{ext}} = \mathbf{f}_p + \mathbf{f}_w + \mathbf{f}_t + \boldsymbol{\tau}_a + \boldsymbol{\tau}_t
$$

### 8.1 电机与驱动映射

关节力矩 \(\boldsymbol{\tau}=[\tau_4,\tau_5]^T\) 经选择矩阵映射到广义坐标：

$$
\mathbf{S}^T = [\mathbf{0}_{2\times 3},\ \mathbf{I}_{2\times 2},\ \mathbf{0}_{2\times 1}]^T \in \mathbb{R}^{6 \times 2}
$$

电机外特性为转速—力矩线性边界（堵转 \(\tau_{\max}\)、空载 \(\omega_{\max}\)）：

$$
\lvert\tau_j\rvert \le \tau_{\max}\max\left(0,\ 1-\frac{\lvert\dot{q}_j\rvert}{\omega_{\max}}\right), \qquad j=4,5
$$

减速比 \(n_{\text{gear}}\) 把电机轴量换算到关节侧；换算顺序与限幅位置在 Step 2 定稿。

### 8.2 关节扭簧

踝关节单向并联扭簧只在压缩区输出恢复力矩：

$$
\tau_a(q_5)=\begin{cases} k_a\,(q_{5,0}-q_5), & q_5 < q_{5,0} \\ 0, & q_5 \ge q_{5,0}\end{cases}
$$

趾关节被动扭簧：

$$
\tau_t(q_6) = -k_t\,(q_6-q_{6,0})
$$

两式均与弹性势能 \(V=\tfrac12 k(\cdot)^2\) 的梯度一致；Step 2 用能量一致性测试校验符号。二者分别只作用于广义坐标 \(q_5\)、\(q_6\)。

### 8.3 气动力与推力

压心世界系位置与点雅可比：

$$
\mathbf{p}_{w/t}(\mathbf{q}) = \begin{bmatrix} q_1 \\ q_2 \end{bmatrix} + R(q_3)\,\mathbf{P}_{w/t}, \qquad \mathbf{f}_{w/t} = \mathbf{J}_{w/t}^T(\mathbf{q})\,\mathbf{F}_{w/t}
$$

压心只随 \(q_1,q_2,q_3\) 移动，故点雅可比只有前 3 列非零：

$$
\mathbf{J}_{w/t}(\mathbf{q}) = \begin{bmatrix} 1 & 0 & -(y_{w/t}-q_2) & 0 & 0 & 0 \\ 0 & 1 & (x_{w/t}-q_1) & 0 & 0 & 0 \end{bmatrix}
$$

螺旋桨推力作用点与方向由 `simulation.aerodynamics.thrust_angle_offset_deg` 等参数给出，Step 2 定稿其广义力表达。

---

## 9. 拉格朗日动力学与任务空间控制（Step 3–4 符号级，式 (10), (12)–(23)）

### 9.1 能量与 M/C/g 的组装路线

Step 3 的 \(\mathbf{M},\mathbf{C},\mathbf{g}\) 统一由 §5.3 的质心位置组装，不在别处重复定义符号。

$$
T = \tfrac12 m_B\,\dot{\mathbf{p}}_C^T\dot{\mathbf{p}}_C + \tfrac12 I_B\,\dot{q}_3^2 + \sum_{i=1}^4\left(\tfrac12 m_i\,\dot{\mathbf{p}}_{ci}^T\dot{\mathbf{p}}_{ci} + \tfrac12 I_i\,\dot{\alpha}_i^2\right)
$$

$$
V = g\left(m_B y_C + \sum_{i=1}^4 m_i y_{ci}\right) + V_a(q_5) + V_t(q_6)
$$

$$
\mathbf{M}(\mathbf{q}) = \sum_k\left(m_k\,\mathbf{J}_{vk}^T\mathbf{J}_{vk} + I_k\,\mathbf{J}_{\omega k}^T\mathbf{J}_{\omega k}\right), \qquad \mathbf{g}(\mathbf{q}) = \frac{\partial V}{\partial \mathbf{q}}
$$

其中 \(\mathbf{J}_{vk}=\partial\mathbf{p}_k/\partial\mathbf{q}\) 是各刚体质心的平动雅可比，\(\mathbf{J}_{\omega k}\) 是对应角速度映射（机身取 \(q_3\)，腿杆取 \(\alpha_i\)）；\(\mathbf{C}\) 由 Christoffel 符号构造。\(c_i\)、\(I_B\)、\(I_i\) 尚无定值（\(m_B,m_i\) 见 YAML），本式只固定结构、不产生数值；进入 Step 3 前必须补齐（见 `doc/missing_params_log.md`）。

### 9.2 触地受约束动力学（式 (10)）

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) + \mathbf{J}_c^T(\mathbf{q})\mathbf{f}_c = \mathbf{S}^T\boldsymbol{\tau} + \mathbf{f}_{\text{ext}}
$$

$$
\begin{bmatrix} \mathbf{M}(\mathbf{q}) & \mathbf{J}_c^T(\mathbf{q}) \\ \mathbf{J}_c(\mathbf{q}) & \mathbf{0} \end{bmatrix} \begin{bmatrix} \ddot{\mathbf{q}} \\ \mathbf{f}_c \end{bmatrix} = \begin{bmatrix} \mathbf{S}^T\boldsymbol{\tau} + \mathbf{f}_{\text{ext}} - \mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}} - \mathbf{g}(\mathbf{q}) \\ -\dot{\mathbf{J}}_c(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}} \end{bmatrix}
$$

### 9.3 腾空无约束动力学（式 (23)）

\(F_y\le 0\) 离地后 \(\mathbf{f}_c=\mathbf{0}\)：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{S}^T\boldsymbol{\tau} + \mathbf{f}_{\text{ext}}
$$

### 9.4 零空间优先级控制与 QR 力矩反解（式 (12)–(22)）

优先级：① \(q_3\) → ② \(q_1\) → ③ \(q_2\)。三级任务对应的雅可比行：

$$
\mathbf{J}_1=[0,0,1,0,0,0], \qquad \mathbf{J}_2=[1,0,0,0,0,0], \qquad \mathbf{J}_3=[0,1,0,0,0,0]
$$

$$
\ddot{x}_{i,d} = k_p(x_{i,d} - x_i) + k_d(\dot{x}_{i,d} - \dot{x}_i), \qquad a_i = \ddot{x}_{i,d} - \dot{\mathbf{J}}_i\dot{\mathbf{q}}
$$

\(\mathbf{J}_{A,i}=[\mathbf{J}_1^T,\ldots,\mathbf{J}_i^T]^T\)，\(\mathbf{A}^+=\mathbf{A}^T(\lambda\mathbf{I}+\mathbf{A}\mathbf{A}^T)^{-1}\)，\(\mathbf{N}_i=\mathbf{I}-\mathbf{J}_{A,i}^+\mathbf{J}_{A,i}\)（\(\mathbf{N}_0=\mathbf{I}\)）：

$$
\ddot{\mathbf{q}}_i = \ddot{\mathbf{q}}_{i-1} + \mathbf{N}_i(\mathbf{J}_i \mathbf{N}_{i-1})^+(a_i - \mathbf{J}_i \ddot{\mathbf{q}}_{i-1}), \qquad \ddot{\mathbf{q}}_0 = \mathbf{0} \implies \ddot{\mathbf{q}}_d = \ddot{\mathbf{q}}_3
$$

把 \(\mathbf{J}_c^T=\mathbf{Q}[\mathbf{R}^T,\ \mathbf{0}]^T\) 的 \(\mathbf{Q}\) 按受约束 / 无约束列分块，取 \(\mathbf{S}_u=[\mathbf{0}_{(6-r)\times r},\ \mathbf{I}_{6-r}]\)（\(r\) 为接触约束维数：全足 4、绕尖 2）：

$$
\boldsymbol{\tau} = (\mathbf{S}_u \mathbf{Q}^T \mathbf{S}^T)^+ \mathbf{S}_u \mathbf{Q}^T (\mathbf{M}\ddot{\mathbf{q}}_d + \mathbf{C}\dot{\mathbf{q}} + \mathbf{g})
$$

---

## 10. 里程碑与代码模块映射（Step 1 → Step 5，阈值见 `plan/milestone.md`）

| 里程碑 | 对应论文公式 | 核心源码 | 接口 / 职责 | 验收 |
|---|---|---|---|---|
| **Step 1** | (2)–(9) | `src/kinematics.py` | `forward_kinematics_tip(q)`、`jacobian_tip(q)`、`jacobian_dot_tip(q, dq)` | 见 `plan/milestone.md` §Step 1 |
| **Step 2** | (11) 驱动项 | `src/actuators.py` | 踝 / 趾扭簧力矩、电机转速—力矩边界、选择矩阵映射 | 见 milestone §Step 2 |
| **Step 3** | (10), (23) | `src/dynamics.py` | \(\mathbf{M}(\mathbf{q})\)、\(\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\)、\(\mathbf{g}(\mathbf{q})\)、KKT 求解 \((\ddot{\mathbf{q}},\mathbf{f}_c)\) | 见 milestone §Step 3 |
| **Step 4** | (12)–(22) | `src/simulator.py` | 优先级任务加速度、QR 力矩反解、`solve_ivp` 事件状态机 | 见 milestone §Step 4 |
| **Step 5** | 全流程 | `main_benchmark.py` | 完整工况仿真与论文基准对比 | 见 milestone §Step 5 |

---

## 附录 A. 雅可比推导增量与极限工况

不重复 §5.2 的 \(\mathbf{p}_{\text{tip}}\)、§4 的 \(\dot{\alpha}_i\)、§6 的成品列公式。本节只补推导增量，支撑 `tests/test_step1.py`。

### A.1 由位置到速度

对 §5.2 求导，并代入 §4 的 \(\dot{\alpha}_i\)：

$$
\begin{aligned}
\dot{x}_{\text{tip}} &= \dot{q}_1 - \sum_{i=1}^4 l_i \sin\alpha_i\,\dot{\alpha}_i \\
\dot{y}_{\text{tip}} &= \dot{q}_2 + \sum_{i=1}^4 l_i \cos\alpha_i\,\dot{\alpha}_i
\end{aligned}
$$

按 \(\dot{\mathbf{q}}\) 提取系数即得 \(J_{\text{tip}}\)（成品见 §6.1）。

### A.2 列向量的 \(R_Z(\pi/2)\) 杆臂分解

单位 \(\dot{q}_j=1\) 的切向映射为左乘 \(R_Z(\pi/2)\)：

$$
R_Z\left(\tfrac{\pi}{2}\right)\begin{bmatrix} r_x \\ r_y \\ 0 \end{bmatrix} = \begin{bmatrix} -r_y \\ r_x \\ 0 \end{bmatrix}, \qquad
\frac{\partial \mathbf{p}_{\text{tip}}^{3\text{D}}}{\partial q_j} = R_Z\left(\tfrac{\pi}{2}\right)\big(\mathbf{p}_{\text{tip}}^{3\text{D}} - \mathbf{p}_{j\text{-center}}^{3\text{D}}\big)
$$

几何对照（元素展开见 §6.1，此处不重写）：

| \(q_j\) | 转动中心 | \(\mathbf{r}_j=\mathbf{p}_{\text{tip}}-\mathbf{p}_{j\text{-center}}\) |
|---|---|---|
| \(q_3,\ q_4\) | \(\mathbf{p}_1\) | 全链 \(l_1\sim l_4\) |
| \(q_5\) | \(\mathbf{p}_2\) | \(l_2\sim l_4\) |
| \(q_6\) | \(\mathbf{p}_4\) | \(l_4\) |
| \(q_1,\ q_2\) | — | 纯平移，列为 \(\mathbf{I}_2\) |

### A.3 极限工况检验

**检验 1**（仅 \(\dot{q}_1,\dot{q}_2\)）：\(\dot{\mathbf{p}}_{\text{tip}}=[\dot{q}_1,\dot{q}_2]^T\)，前两列 \(\mathbf{I}_2\)。

**检验 2**（仅 \(\dot{q}_6\)）：\(\lVert\dot{\mathbf{p}}_{\text{tip}}\rVert=l_4\lvert\dot{q}_6\rvert\)，方向垂直脚趾。

**检验 3**（\(\dot{q}_4=-\dot{q}_3\)，其余零）：\(\dot{\alpha}_i\equiv 0\Rightarrow\dot{\mathbf{p}}_{\text{tip}}=\mathbf{0}\)，第 3、4 列同号一致。

**检验 4**（\(\alpha_i=0\)）：转动列 \(=[0,\sum l_i]^T\)，切向纯竖直。

加速度映射见 §6.3。

---

## 附录 B. 机身质心偏移 \((d_x,d_y)\) 可行域论证

> 本节数值是写作时的推导快照；参数值以 `config/robot_paras.yaml` 为准，YAML 更新后需重算本节。

可行域（本文定稿）：

$$
d_x \in (-0.02,\ 0)\ \text{m}, \qquad d_y \in (0,\ 0.052)\ \text{m}
$$

暂不填未验证经验常数（`doc/missing_params_log.md`）。依据如下。

1. **水平** \(d_x \in (-0.02, 0)\ \text{m}\)：
   * \(d_x < 0\)：质心在髋后。起跳初始构型质心投影在趾前微小邻域，髋比足端更靠前，故 \(O_B\) 在质心前方（论文 p13）。
   * \(d_x > -0.02\ \text{m}\)：质心须在气动中立点之前（论文 p4）；机翼压心 \(P_{w,x}=-0.02\ \text{m}\)。
2. **垂直** \(d_y \in (0, 0.052)\ \text{m}\)：
   * \(d_y > 0\)：髋在机翼前缘下方附近，机身质量多在髋上（论文 p4）。
   * \(d_y < 0.052\ \text{m}\)：质心须在气动合力中心下方；机翼面积 \(0.18\ \text{m}^2\)、\(P_{w,y}=0.042\ \text{m}\)，尾翼面积 \(0.0864\ \text{m}^2\)、\(P_{t,y}=0.074\ \text{m}\)：

$$
y_{\text{NP}} \approx \frac{0.18 \times 0.042 + 0.0864 \times 0.074}{0.18 + 0.0864} \approx 0.0524\ \text{m}
$$

3. **支撑多边形**：静态站立时质心地面投影落在趾与后爪（hallux）支撑区间；起跳微偏趾前（论文 p4、p13，Extended Data Fig. 3d）。
4. **工程原则**：区间仅作 Step 3 参数可行域；暂不填未验证经验常数（`doc/missing_params_log.md`）。
