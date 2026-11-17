# EPFL RAVEN 跳跃起飞纯数学动力学仿真 (Jumping Takeoff Dynamics Simulation)

本项目旨在通过纯 Python 科学计算栈，数学复现洛桑联邦理工学院（EPFL）在论文reference/Fast ground-to-air transition with avian-inspired multifunctional legs.pdf里的**蹬地跳跃（Jumping）及跳跃辅助起飞（Jumping Takeoff）**动力学全过程。

---

## 1. 核心复现目标与物理边界

* **物理建模范畴**：矢状面（x-z 平面）浮基多连杆刚体动力学 + 弹簧储能释放 + 被动弹性脚趾 + 触地/腾空混合动力切换（Hybrid Dynamical System）+ 螺旋桨推力与固定翼准定常气动模型。
* **严禁引入外部物理引擎**：本项目定位为**纯数学微分-代数方程（DAE）求解**，**严禁**引入 MuJoCo、PyBullet、Gazebo 或 Isaac Sim 等重型物理引擎。
* **核心论文基准指标（Benchmarks）**：
  * **整机总质量**：`0.62 kg`（翼展 `1.0 m`，机身长 `0.5 m`）
  * **工况 A（纯腿部垂直跳跃，关闭螺旋桨）**：最大垂直跳跃高度约 **`0.26 m`**
  * **工况 B（跳跃辅助起飞，开启螺旋桨与气动）**：离地瞬时高度约 **`0.50 m`**，前飞水平速度达到 **`2.20 m/s`**

---

## 2. 技术栈锁定（Tech Stack）

* **编程语言**：Python 3.10+
* **核心数值计算**：`numpy`（矩阵与雅可比运算）、`scipy`（统一使用 `scipy.integrate.solve_ivp` 配合 `events` 事件检测进行状态机切换）
* **配置管理**：`pyyaml`（读取 `config/robot_params.yaml`）
* **可视化与测试**：`matplotlib`（静态曲线与火柴人频闪动画）、`pytest`（物理守恒与精度断言测试）

---

## 3. 项目目录结构与文件职责

```text
raven-epfl-replication/
├── .trae/
│   └── rules/
│       └── raven_simulation_rules.md  # Trae 强制开发与防脑补铁律
├── milestones.md                      # 【验收标准主文件】Step 1 ~ Step 5 硬性精度门槛
├── README.md                          # 项目全局上下文、技术栈与运行指南（本文件）
├── config/
│   └── robot_params.yaml              # 【唯一参数源】所有质量、尺寸、刚度与气动常数
├── docs/
│   ├── paper_spec.md                  # 从 Nature 正文与补充材料提取的数学公式与状态机
│   └── missing_params_log.md          # 论文未公开参数的待确认记录表
├── src/                               # 核心纯数学动力学源码
│   ├── kinematics.py                  # Step 1：正运动学、质心与解析雅可比矩阵
│   ├── actuators.py                   # Step 2：电机外特性、单向储能弹簧与被动脚趾力矩
│   ├── dynamics.py                    # Step 3：质量矩阵、KKT 触地约束求解与气动外力
│   └── simulator.py                   # Step 4：混合动力状态机与 solve_ivp 积分器
├── tests/                             # 自动化物理校验测试（对应 milestones.md）
├── output/                            # 自动生成的校验图表与 final_report/ 最终动画
└── main_benchmark.py                  # Step 5：一键运行论文基准对照实验
```

---

## 4. 环境安装与运行方式

```bash
# 1. 安装依赖
pip install numpy scipy pyyaml matplotlib pytest

# 2. 运行分步物理验收测试（以 Step 1 为例）
pytest tests/test_step1.py -v

# 3. 运行最终基准仿真与图表生成（完成 Step 5 后）
python main_benchmark.py
```

---

## 5. AI Agent 协作协议（必读）

* **验收标准索引**：所有开发进度与代码通过条件一律以根目录的 **`milestones.md`**（验收标准主文件）为准。
* **参数引用规范**：编写任何物理方程时，必须通过 `yaml.safe_load` 读取 `config/robot_params.yaml`，严禁硬编码常数。
