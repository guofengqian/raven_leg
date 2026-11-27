# X-Y 矢状面跳跃起跳的动力学建模与仿真

> 对齐论文 *Fast ground-to-air transition with avian-inspired multifunctional legs*（Methods — Dynamics modeling and simulation，pp. 13–16，式 (2)–(23)）与 `plan/milestone.md`（Step 1 → Step 5）。核心目标：**验证机构物理可行性**、**反解减速比上限**（\(i \le 22.8 \to 19.13\)）与**生成真机离线轨迹**。

$$
\underbrace{\mathbf{x}=\begin{bmatrix}\textcolor{#0284c7}{\mathbf{q}}\\ \dot{\mathbf{q}}\end{bmatrix}}_{\text{状态（阶段 1 定义 }\mathbf{q}\text{）}},\;
\underbrace{\textcolor{#ea580c}{\boldsymbol{\tau},\,\mathbf{f}_{\text{ext}}}}_{\text{输入（阶段 2 / 4）}},\;
\sigma
\;\xrightarrow{\;\text{主方程 + 加速度约束（阶段 3）}\;}
\underbrace{\textcolor{#7c3aed}{\ddot{\mathbf{q}}}}_{\text{待求}}
\bigl(\text{触地兼求 }\textcolor{#7c3aed}{\mathbf{f}_c}\bigr)
\;\xrightarrow{\;\text{积分（阶段 5）}\;}
(\mathbf{q},\dot{\mathbf{q}})
$$

$$
\underbrace{\textcolor{#7c3aed}{\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})}}_{\text{阶段 3：多刚体惯性与重力}} + \underbrace{\textcolor{#16a34a}{\mathbf{J}_c^T(\mathbf{q})}\textcolor{#7c3aed}{\mathbf{f}_c}}_{\text{阶段 1 接口 / 阶段 3 求解}} = \underbrace{\textcolor{#ea580c}{\mathbf{S}^T\boldsymbol{\tau}}}_{\text{阶段 2 限幅 / 阶段 4 反解}} + \underbrace{\textcolor{#ea580c}{\mathbf{f}_{\text{ext}}}}_{\text{阶段 2：扭簧+气动+推力}}
$$

$$
\underbrace{\textcolor{#16a34a}{\mathbf{J}_c(\mathbf{q})\dot{\mathbf{q}}=\mathbf{0}}}_{\text{阶段 1：速度级硬约束（式 2）}},\qquad
\underbrace{\textcolor{#16a34a}{\mathbf{J}_c(\mathbf{q})}\textcolor{#7c3aed}{\ddot{\mathbf{q}}}=-\textcolor{#16a34a}{\dot{\mathbf{J}}_c(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}}}}_{\text{阶段 1：加速度级硬约束（式 3）}}
$$

![仿生跳跃动力学建模、控制与设计闭环架构](spec/assets/stages_architecture.svg?v=2)

---

## 目录

| 阶段 | 章节 | 核心定位 | 里程碑 |
|---|---|---|---|
| **前置** | [第 0 章：读法与符号速查](spec/00_读法与符号速查.md) | 符号表、式号索引与 YAML 映射 | 全程 |
| **阶段 1** | [第 1 章：浮基运动学](spec/01_浮基运动学.md) | 广义坐标 \(\mathbf{q}\)、足端雅可比与接触约束 | Step 1 |
| **阶段 2** | [第 2 章：执行机构与非接触外力建模](spec/02_执行机构与非接触外力建模.md) | 电机包络、扭簧与气动/推力右端项 | Step 2 |
| **阶段 3** | [第 3 章：多刚体惯性与重力及接触约束建模](spec/03_多刚体惯性与重力及接触约束建模.md) | 拉格朗日 \(\mathbf{M},\mathbf{C},\mathbf{g}\) 与触地 KKT 求解 | Step 3 |
| **阶段 4** | [第 4 章：多任务优先级控制与力矩逆解](spec/04_多任务优先级控制与力矩逆解.md) | 零空间 3 级投影与 QR 力矩反解 | Step 4 |
| **阶段 3** | [第 5 章：接触模态切换与碰撞重置](spec/05_接触模态切换与碰撞重置.md) | 模态状态机、碰撞重置与摩擦锥 | Step 4 |
| **阶段 5** | [第 6 章：闭环数值求解与仿真规范](spec/06_闭环数值求解与仿真规范.md) | Baumgarte 稳定化与 ODE 事件积分 | Step 3–4 |
| **阶段 5** | [第 7 章：起跳仿真与减速比反解](spec/07_起跳仿真与减速比反解.md) | 极限角速度提取与减速比上限反解 | Step 4–5 |
| **阶段 5** | [第 8 章：硬件设计验证与里程碑映射](spec/08_硬件设计验证与里程碑映射.md) | 代码模块映射、V&V 清单与实机轨迹 | Step 1–5 |

- **附录 A**：[雅可比矩阵权重推导](spec/appendices/附录A_雅可比矩阵权重推导.md)（列权重几何与代数推导）
- **附录 B**：[机身质心偏移可行域](spec/appendices/附录B_机身质心偏移可行域.md)（质心偏移 \((d_x,d_y)\) 可行域论证）
- **附录 D**：[运动方程直观解释](spec/appendices/附录D_运动方程直观解释.md)（直觉入门与量纲核对，对应阶段 3）

---

> 单一事实来源：参数 `config/robot_paras.yaml` · 验收与口径 `plan/milestone.md` · 符号与历史命名 [第 0 章](spec/00_读法与符号速查.md) · 待确认参数 [missing_params_log.md](missing_params_log.md)
