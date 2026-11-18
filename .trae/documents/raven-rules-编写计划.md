# RAVEN 项目 rules 文件编写计划

## Summary

重写 `.trae/rules/raven_rules.md`，并同步修正 `README.md`、`plan/milestone.md` 中的旧路径、旧文件名和论文出处，使项目文档与实际结构一致。规则将明确各类事实来源的边界，保留纯数学仿真路线、Step 1 → Step 5 单步开发、测试先行和人工验收门禁。不得借本次修改调整任何物理参数、数学公式或验收数值。

## Current State Analysis

1. 当前规则 `.trae/rules/raven_rules.md` 已包含参数溯源、纯 Python 架构、五步里程碑和人工门禁，但存在以下问题：
   - frontmatter 的 description 错写为 `plan/milestones.md`，实际文件是 `plan/milestone.md`。
   - 正文错写 `config/robot_params.yaml`，实际唯一参数文件是 `config/robot_paras.yaml`。
   - 正文错写 `docs/paper_spec.md`、`docs/missing_params_log.md`，实际目录是 `doc/`。
   - 开头声称“五项铁律”，正文实际只有四个一级章节，结构不一致。
   - 参数文件、公式规范、验收标准、缺失参数日志之间的权威边界没有明确，发生冲突时缺少处理规则。
   - 验收阈值在 rules 与 milestone 中重复书写，后续容易产生版本漂移。
2. `plan/milestone.md` 已完整规定 Step 1～5 的交付物与硬性阈值，应继续作为唯一验收标准；本次只修正其自引用文件名、参数/日志路径和论文出处，不改任何验收内容。
3. `doc/paper_spec.md` 已定义广义坐标、世界系/本体系、髋关节与质心的区别，应作为数学符号和公式规范来源。
4. `doc/missing_params_log.md` 已记录质心、连杆质心距和转动惯量等未公开参数；当前用户决定是只锁可行域、暂不定值/估算。规则必须防止把“待确认”参数伪装成论文值或填入占位数值。
5. `README.md` 和 `plan/milestone.md` 含有旧路径；此外 README 的目录说明和 milestone 的 Step 5 目标把论文错写为 *Nature*，实际来源是 *Science Robotics*。按用户最新要求，这些文件本次一并同步。

## Proposed Changes

### 修改文件 1：`.trae/rules/raven_rules.md`

采用整体重写，保留原有强制约束的实质内容，按以下结构组织。

#### 1. Frontmatter 与规则定位

- 保留 `alwaysApply: true`，使规则适用于整个项目，而不只限于 `src/*.py` 和 `tests/*.py`。
- 移除会误导适用范围的狭窄 `globs`；description 改为“EPFL RAVEN 纯数学动力学仿真开发、参数溯源与分步强制验收规范，绑定 plan/milestone.md 为唯一验收标准”。
- 开篇明确角色、目标和适用行为：阅读论文、修改文档/配置、编写代码、运行命令、生成图表和汇报验收结果均受约束。

#### 2. 权威文件与冲突处理

建立清晰的职责分工，不再笼统地把多个文件都称为“唯一来源”：

| 文件 | 唯一负责的内容 |
|---|---|
| `reference/Fast ground-to-air transition with avian-inspired multifunctional legs.pdf` | 论文原始证据与实验基准出处 |
| `config/robot_paras.yaml` | 已确认物理参数的数值与单位 |
| `doc/paper_spec.md` | 坐标系、广义坐标、符号约定和数学公式 |
| `plan/milestone.md` | Step 顺序、交付物、测试阈值和通过条件 |
| `doc/missing_params_log.md` | 未公开参数、检索依据、可行域、估算状态和用户决定 |
| `.trae/rules/raven_rules.md` | 开发流程与协作约束，不复制物理参数或验收阈值 |

补充冲突处理规则：发现论文、YAML、paper_spec 或 milestone 互相矛盾时，暂停受影响模块，列出冲突位置和证据并请求确认；不得自行挑选一个版本继续实现。README 仅作项目导航，不得覆盖上述权威文件的实际路径与定义。

#### 3. 参数溯源与“零脑补”

- 所有质量、长度、质心、惯量、刚度、电机、气动、接触和摩擦等物理参数必须从 `config/robot_paras.yaml` 读取；禁止在 `src/`、`tests/` 中硬编码物理常数。
- 论文值、由论文值推导的量、用户确认的估算值必须明确区分；推导值需保留公式和来源，估算值必须经用户明确确认后才能进入 YAML，并在 YAML 与 missing log 中标注“估算值，非论文原值”。
- 论文未公开且模块必需的参数，必须先登记到 `doc/missing_params_log.md`，至少包含名称、符号、量纲、影响模块、论文检索范围、可行域/候选值、来源和确认状态。
- 用户选择“暂不定值”时，不得在 YAML 或代码中填默认值、经验值、占位值，也不得继续实现依赖该参数的模块；不相关的只读分析或文档整理可继续。
- 区分物理参数与数值算法设置：有限差分步长、随机种子、求解器容差等不是物理参数，但必须使用有意义的命名并说明用途；验收阈值只能以 milestone 为准。

#### 4. 数学模型与符号一致性

- 实现前必须核对 `doc/paper_spec.md` 与 YAML，统一使用 `q1～q6`、世界系/本体系和连杆角度约定。
- 明确 `(q1,q2)` 是本体坐标系原点（髋关节铰点），不是机身质心；质心偏移及气动力作用点不得混用。
- 修改任何运动学/动力学定义时，必须同步检查受影响的公式、参数注释和测试；发现 `l3/l4/theta3` 等定义冲突时按“冲突处理”停下确认，不得自行换定义。
- rules 不重复书写具体推导公式，避免与 paper_spec 形成第二份数学规范。

#### 5. 技术栈、模块边界与数值稳定性

- 锁定 Python 3.10+、NumPy、SciPy、Matplotlib、PyYAML、pytest；禁止 MuJoCo、PyBullet、Gazebo、Isaac Sim 等外部物理引擎。
- 主积分器使用 `scipy.integrate.solve_ivp`，触地/腾空切换使用 `events`。
- 保留现有模块职责：`src/kinematics.py`、`src/actuators.py`、`src/dynamics.py`、`src/simulator.py`、对应 `tests/test_stepX.py`、`output/` 和 Step 5 的 `main_benchmark.py`。
- 单步开发期间不得提前编写后续 Step 的实现或空壳函数。
- 线性方程组优先 `np.linalg.solve`；确需奇异/病态处理时使用有依据的 `lstsq` 或伪逆保护；禁止用 `np.linalg.inv` 显式求逆完成动力学/KKT 求解。

#### 6. Step 1～5 与人工门禁

- 严格按 `plan/milestone.md` 的 Step 1 → Step 5 顺序，每次只实现一个 Step。
- 用一张不含重复数值的表列出各 Step 的名称、核心模块、测试脚本和主要图表；所有误差阈值、样本数量和基准指标均直接引用 milestone，不在 rules 再复制一份。
- 禁止放宽阈值、删除失败断言、用 `skip`/`xfail` 规避验收，或把绘图失败视为可忽略警告。
- 当前 Step 的测试全部通过且要求的图表已生成后，才能勾选 milestone 对应项；随后展示核心误差并立即停止，不得开始下一 Step。
- 只有用户明确回复“通过，进入下一步”后，才允许开始下一 Step。

#### 7. 测试、运行证据与汇报格式

- 每个 Step 的源码完成后立即编写对应 `tests/test_stepX.py`，并运行 `pytest tests/test_stepX.py -v`。
- 随机校验使用固定且可报告的随机种子；样本数量与容差严格按 milestone。
- 测试失败时先定位物理公式、符号、单位或数值实现问题，不得通过调大容差求得通过。
- 完成汇报必须包含：实际运行命令、通过/失败数量、核心误差数值及单位、生成文件路径、尚未解决的阻塞项；没有终端证据不得声称“已通过”。
- 本次仅修改规则与导航/验收文档中的同步信息，不新增测试文件，也不运行物理仿真测试。

#### 8. 最小改动与协作边界

- 只完成当前请求，不做无关重构，不主动创建额外文档或目录。
- 未经用户明确要求，不执行 git commit/push，不删除或恢复用户文件。
- 使用简明中文说明；涉及物理量时写清符号、单位和来源，避免把推断表述成论文原文。
- 修正开头“五项铁律”与实际章节数不一致的问题：不再写数量，统一称为“以下铁律”。

### 修改文件 2：`README.md`

只做导航与元信息同步，不重写项目目标：

- 技术栈和 AI 协作协议中的 `config/robot_params.yaml` 全部改为 `config/robot_paras.yaml`。
- 目录树根名改为 `raven_leg/`；规则文件改为 `.trae/rules/raven_rules.md`；根目录的 `milestones.md` 改为实际的 `plan/milestone.md`；`docs/` 改为 `doc/`。
- 目录树补入 `reference/Fast ground-to-air transition with avian-inspired multifunctional legs.pdf`，并保留现有 `src/`、`tests/`、`output/`、`main_benchmark.py` 的规划说明，不因此提前创建这些文件。
- 测试目录注释和“验收标准索引”改为引用 `plan/milestone.md`。
- 两处 *Nature* 表述改为 *Science Robotics*；论文路径统一指向 `reference/` 下的实际 PDF。
- 保留现有技术栈、安装命令、物理边界和基准数值，不修改任何物理定义。

### 修改文件 3：`plan/milestone.md`

仅做以下四处同步：

1. 文件自引用 `milestones.md` 改为 `plan/milestone.md`。
2. 全局前置约束中的 `config/robot_params.yaml` 改为 `config/robot_paras.yaml`。
3. 全局前置约束中的 `docs/missing_params_log.md` 改为 `doc/missing_params_log.md`。
4. Step 5 目标中的 EPFL *Nature* 改为 EPFL *Science Robotics*。

除上述路径/出处外，Step 1～5 的复选框状态、交付物、样本数量、所有数值阈值、误差范围、图表文件名和人工门禁均保持原文。

### 明确不修改的文件

`config/robot_paras.yaml`、`doc/paper_spec.md`、`doc/missing_params_log.md`、论文 PDF 以及所有 `src/`、`tests/`、`output/` 内容均保持不变。`paper_spec.md` 中已知的 `l3/l4/theta3` 定义问题不属于本次路径同步范围，仍按规则另行确认。

## Assumptions & Decisions

- 用户最新决定：rules、README 和 milestone 中的不同步信息一并修正。
- 现有 Step 1～5 顺序、全部验收阈值、测试样本数量和人工确认门禁完全保留。
- rules 是全局流程规范，不作为物理参数、公式或验收阈值的第二份数据源。
- 本次不提交、不推送 git；如需要同步云端，由用户另行明确请求。

## Verification

1. 重新读取三个目标文件，检查 frontmatter、章节编号、目录树和实际文件路径。
2. 在 `.trae/rules/raven_rules.md`、`README.md`、`plan/milestone.md` 中检索，确认不再出现 `config/robot_params.yaml`、`docs/`、`milestones.md`、`raven_simulation_rules.md` 和错误的 *Nature* 出处。
3. 用只读 shell 检查文档引用的论文、YAML、paper_spec、milestone、missing log 均真实存在。
4. 查看 `git diff -- plan/milestone.md`，确认只有计划的四处路径/出处变化；查看 README diff，确认基准指标与运行命令未被改变；对照 milestone 确认 rules 没有另设验收数值。
5. 运行 `git diff --check` 检查格式问题，并用 `git status --short` 确认本次内容修改仅涉及 `.trae/rules/raven_rules.md`、`README.md`、`plan/milestone.md`（计划文件除外）。
6. 最终回复列出三个文件的同步内容，并明确说明本次是文档修改、未运行物理测试、未提交或推送。
