# EPFL RAVEN 跳跃起飞纯数学动力学仿真 (Jumping Takeoff Dynamics Simulation)

本项目旨在通过纯 Python 科学计算栈，复现 EPFL 论文 `reference/Fast ground-to-air transition with avian-inspired multifunctional legs.pdf` 中的蹬地跳跃及跳跃辅助起飞动力学。论文原始证据以 PDF 为准，确认后的参数以 `config/robot_paras.yaml` 为准，公式与符号以 `doc/paper_spec.md` 为准，验收口径以 `plan/milestone.md` 为准；本 README 只作导航。

## 当前进度

- 已有实现：`src/kinematics.py`。
- 已有验证：`tests/test_step1.py`。
- 尚未实现：执行器、完整动力学、混合状态机和基准运行入口。仓库目录树中的这些文件是计划交付物，不代表已经完成。
- Step 3 动力学实现受缺失参数阻塞；具体项目、状态及处理规则见 [`doc/missing_params_log.md`](doc/missing_params_log.md)。
- 下一步：继续按 [`plan/milestone.md`](plan/milestone.md) 的顺序推进当前未完成步骤；不得将未确认参数作为默认值使用。

---

## 1. 核心复现目标与物理边界

* **物理建模范畴**：世界系 (X)-(Y) 矢状面浮基多连杆刚体动力学；(+Y) 为竖直向上，(Z) 为面外方向。旧材料中的 `z` / `Fz` 在注明的兼容语境下表示竖直量，规范符号统一用 `y` / `Fy`，详见 [`doc/spec/00_读法与符号速查.md`](doc/spec/00_读法与符号速查.md)。
* **严禁引入外部物理引擎**：本项目定位为**纯数学微分-代数方程（DAE）求解**，**严禁**引入 MuJoCo、PyBullet、Gazebo 或 Isaac Sim 等重型物理引擎。
* **质量与论文指标**：实机称量质量、正文近似质量和各仿真工况指标分开记录。具体质量来源见 YAML；基准值、测量定义及容差只以 [`plan/milestone.md`](plan/milestone.md) 为准。

---

## 2. 技术栈锁定（Tech Stack）

* **编程语言**：Python 3.10+
* **核心数值计算**：`numpy`（矩阵与雅可比运算）、`scipy`（统一使用 `scipy.integrate.solve_ivp` 配合 `events` 事件检测进行状态机切换）
* **配置管理**：`pyyaml`（读取 `config/robot_paras.yaml`）
* **可视化与测试**：`matplotlib`（静态曲线与火柴人频闪动画）、`pytest`（物理守恒与精度断言测试）

---

## 3. 项目目录结构与文件职责

```text
raven_leg/
├── .trae/
│   └── rules/
│       └── raven_rules.md                  # Trae 强制开发与防脑补铁律
├── plan/
│   └── milestone.md                        # 【验收标准主文件】Step 1 ~ Step 5 硬性精度门槛
├── README.md                               # 项目全局上下文、技术栈与运行指南（本文件）
├── reference/
│   └── Fast ground-to-air transition with avian-inspired multifunctional legs.pdf
├── config/
│   └── robot_paras.yaml                    # 【唯一参数源】所有质量、尺寸、刚度与气动常数
├── doc/
│   ├── paper_spec.md                       # 从 Science Robotics 正文与补充材料提取的数学公式与状态机
│   └── missing_params_log.md               # 论文未公开参数的待确认记录表
├── src/                                    # 核心纯数学动力学源码
│   ├── kinematics.py                       # Step 1：正运动学、质心与解析雅可比矩阵
│   ├── actuators.py                        # Step 2：电机外特性、单向储能弹簧与被动脚趾力矩
│   ├── dynamics.py                         # Step 3：质量矩阵、KKT 触地约束求解与气动外力
│   └── simulator.py                        # Step 4：混合动力状态机与 solve_ivp 积分器
├── tests/                                  # 自动化物理校验测试（对应 plan/milestone.md）
├── output/                                 # 自动生成的校验图表与 final_report/ 最终动画
└── main_benchmark.py                       # Step 5：一键运行论文基准对照实验
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

* **验收标准索引**：所有开发进度与代码通过条件一律以 **`plan/milestone.md`**（验收标准主文件）为准。
* **参数引用规范**：编写任何物理方程时，必须通过 `yaml.safe_load` 读取 `config/robot_paras.yaml`，严禁硬编码常数。
