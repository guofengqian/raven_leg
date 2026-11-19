"""
=============================================================================
EPFL RAVEN 机器人矢状面 6-DOF 运动学与雅可比矩阵模块 (Step 1)
=============================================================================

【小白入门导读】：
很多初学者看到“正运动学”、“雅可比矩阵”会觉得高深，其实物理本质非常直观：

1. 什么是正运动学 (Forward Kinematics, FK)？
   - 通俗说就是：“知道了各个关节转了多少度，算出脚尖在世界上的具体坐标 (X, Z)”。
   - 就像折线画火柴人：从髋关节出发，知道每根骨头的长度和朝向角度，一截一截把向量
     加起来，最终就能精确算出脚尖、脚趾在哪里。

2. 什么是雅可比矩阵 (Jacobian Matrix, J)？
   - 通俗说就是：“关节转动的速度，怎样放大或缩小成脚尖移动的线速度”。
   - 它是末端位置对关节角度的偏导数矩阵（即多维的传动比）：
       脚尖速度 V_tip = J * 关节角速度 q_dot
   - 雅可比的每一列代表：如果对应的那个关节以 1 rad/s 转动，脚尖会在 X 和 Z 方向以多大速度移动。

3. 什么是雅可比时间导数 (Jacobian Time Derivative, J_dot)？
   - 机械臂在旋转时，不仅姿态在变，各关节到末端的“杠杆臂长度”也在动态变化。
   - 对速度公式再次求导：
       脚尖加速度 a_tip = J * 关节角加速度 q_ddot + J_dot * 关节角速度 q_dot
   - 这里的 J_dot * q_dot 项，正好对应物理学中旋转产生的【向心加速度】和【科氏加速度】！

4. 系统的 6 个广义坐标 q = [q1, q2, q3, q4, q5, q6]^T 分别是什么？
   - q1: 髋关节连线中点的水平位置 X (米，向前为正) [浮基平动，未驱动]
   - q2: 髋关节连线中点的垂直位置 Z (米，向上为正) [浮基平动，未驱动]
   - q3: 整个机身的俯仰角 theta_b (弧度，抬头为正) [浮基转动，未驱动]
   - q4: 髋关节转角 (弧度，大腿相对机身纵轴的相对夹角) [电机驱动]
   - q5: 踝关节转角 (弧度，下肢相对大腿的相对夹角) [电机驱动 + 并行储能弹簧]
   - q6: 趾关节偏转角 (弧度，脚趾相对偏移杆的相对夹角) [被动弹簧]

权威规范依据：doc/paper_spec.md
物理参数来源：config/robot_paras.yaml
=============================================================================
"""

import os  # 操作系统路径工具：用于获取当前文件位置并拼接配置文件路径
from typing import Dict, Any, Tuple  # 类型注解：Dict 字典、Any 任意类型、Tuple 元组
import yaml  # YAML 解析库：读取 config/robot_paras.yaml 配置文件
import numpy as np  # 数值计算库：向量、矩阵运算与三角函数


def load_kinematics_params(config_path: str = None) -> Dict[str, float]:
    """
    【功能】：从 YAML 配置文件中加载机器人机构的几何尺寸参数。

    【输入】：
    - config_path: YAML 配置文件路径；为 None 时自动使用项目内 config/robot_paras.yaml

    【输出】：
    - params: 包含各杆长（l1~l4，米）、theta3（弧度）和 body_length（米）的字典
    """
    if config_path is None:
        # curr_dir: 当前代码文件 (kinematics.py) 所在的本地绝对目录 (字符串)
        curr_dir: str = os.path.dirname(os.path.abspath(__file__))
        # config_path: 拼接出 config/robot_paras.yaml 配置文件的绝对路径 (字符串)
        config_path: str = os.path.abspath(os.path.join(curr_dir, "..", "config", "robot_paras.yaml"))

    # 以只读 UTF-8 编码安全读取 yaml 文件
    with open(config_path, "r", encoding="utf-8") as f:
        # cfg: 通过 YAML 解析器读入的完整嵌套字典，存储了所有硬件与仿真参数
        cfg: dict = yaml.safe_load(f)

    # geom: 从配置中提取出的仿真几何参数子字典 (包含杆长、角度等)
    geom: dict = cfg["simulation"]["geometry"]

    # params: 规范化整理后的物理几何参数字典，全部转换为标准 SI 单位（米、弧度）
    params: Dict[str, float] = {
        "l1": float(geom["l1_m"]),                       # l1: 上肢（大腿）长度，单位：米 (0.12 m)
        "l2": float(geom["l2_m"]),                       # l2: 下肢（小腿/跗跖骨）长度，单位：米 (0.12 m)
        "l3": float(geom["l3_m"]),                       # l3: 趾关节偏移杆长度，单位：米 (0.023 m)
        "l4": float(geom["l4_m"]),                       # l4: 脚趾连杆长度，单位：米 (0.06 m)
        "theta3": float(np.deg2rad(geom["theta3_deg"])), # theta3: 趾关节结构固定安装角，45° 转换为弧度 (0.7854 rad)
        "body_length": float(geom["body_length_m"]),     # body_length: 机身全长参考值，单位：米 (0.5 m)
    }
    return params


def compute_absolute_angles(q: np.ndarray, theta3: float) -> Tuple[float, float, float, float]:
    """
    【功能】：计算腿部 4 根连杆相对世界坐标系 +X 轴（水平正前方）的绝对姿态角 alpha_1 ~ alpha_4。

    【输入变量说明】：
    - q: 6 维广义坐标向量，类型为 numpy 数组，包含 [q1, q2, q3, q4, q5, q6]
    - theta3: 固定的趾关节偏移角（单位：弧度）
    """
    # 从广义坐标数组 q 中提取出各转动自由度：
    q3: float = q[2]  # q3: 机身纵轴相对世界水平 +X 轴的俯仰角 (弧度，抬头为正)
    q4: float = q[3]  # q4: 髋关节相对角，即大腿连杆相对机身纵轴的转角 (弧度)
    q5: float = q[4]  # q5: 踝关节相对角，即小腿连杆相对大腿连杆的转角 (弧度)
    q6: float = q[5]  # q6: 趾关节相对角，即脚趾连杆相对偏移杆的偏转角 (弧度)

    # alpha1: 上肢（大腿 l1）相对世界水平 +X 轴的绝对倾角 (弧度)
    # 机身抬头 q3，大腿又相对机身转了 q4，因此两角直接相加
    alpha1: float = q3 + q4

    # alpha2: 下肢（小腿 l2）相对世界水平 +X 轴的绝对倾角 (弧度)
    # 在大腿绝对角 alpha1 的基础上，膝/踝关节相对弯折了 q5
    alpha2: float = alpha1 + q5

    # alpha3: 趾关节偏移杆（l3）相对世界水平 +X 轴的绝对倾角 (弧度)
    # 在小腿绝对角 alpha2 的基础上，叠加机械固定的安装偏角 theta3 (45°)
    alpha3: float = alpha2 + theta3

    # alpha4: 脚趾连杆（l4）相对世界水平 +X 轴的绝对倾角 (弧度)
    # 在偏移杆绝对角 alpha3 的基础上，叠加脚趾弹簧的偏转角 q6
    alpha4: float = alpha3 + q6

    return alpha1, alpha2, alpha3, alpha4


def compute_absolute_angle_rates(q_dot: np.ndarray) -> Tuple[float, float, float, float]:
    """
    【功能】：计算 4 根连杆绝对倾角的时间变化率（即绝对角速度 d_alpha / dt）。

    【输入变量说明】：
    - q_dot: 6 维广义速度向量 [q1_dot, q2_dot, q3_dot, q4_dot, q5_dot, q6_dot]

    【输出】：
    - (alpha1_dot, alpha2_dot, alpha3_dot, alpha4_dot): 4 个绝对角速度 (rad/s)
    """
    # 从广义速度数组 q_dot 中提取出各转动自由度的一阶时间导数（角速度）：
    q3_dot: float = q_dot[2]  # q3_dot: 机身俯仰角速度 (rad/s)
    q4_dot: float = q_dot[3]  # q4_dot: 髋关节电机旋转角速度 (rad/s)
    q5_dot: float = q_dot[4]  # q5_dot: 踝关节电机旋转角速度 (rad/s)
    q6_dot: float = q_dot[5]  # q6_dot: 脚趾关节偏转角速度 (rad/s)

    # alpha1_dot: 上肢（大腿 l1）绕垂直纸面轴的绝对角速度 (rad/s)
    alpha1_dot: float = q3_dot + q4_dot

    # alpha2_dot: 下肢（小腿 l2）绕垂直纸面轴的绝对角速度 (rad/s)
    alpha2_dot: float = alpha1_dot + q5_dot

    # alpha3_dot: 趾关节偏移杆（l3）的绝对角速度 (rad/s)
    # ★重要：因为固定角 theta3 是常数 (d(theta3)/dt = 0)，所以偏移杆角速度等于小腿角速度
    alpha3_dot: float = alpha2_dot

    # alpha4_dot: 脚趾连杆（l4）的绝对角速度 (rad/s)
    alpha4_dot: float = alpha3_dot + q6_dot

    return alpha1_dot, alpha2_dot, alpha3_dot, alpha4_dot


def forward_kinematics_tip(q: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：正向运动学函数 —— 计算足端爪尖 (p_claw / p_tip) 在世界坐标系下的绝对坐标。

    【输入】：
    - q: 6 维广义坐标 [q1, q2, q3, q4, q5, q6]
    - params: 包含 l1~l4 与 theta3 的参数字典

    【输出】：
    - 长度为 2 的数组 [X_tip, Z_tip]，单位：米
    """
    # q1, q2: 左右两髋关节连线中点在世界惯性系下的绝对坐标 (X_hip, Z_hip)，单位：米
    q1: float = q[0]
    q2: float = q[1]

    # 从参数字典中解包各个连杆长度与固定偏移角：
    l1: float = params["l1"]          # 大腿连杆长度 (米)
    l2: float = params["l2"]          # 小腿连杆长度 (米)
    l3: float = params["l3"]          # 偏移杆长度 (米)
    l4: float = params["l4"]          # 脚趾连杆长度 (米)
    theta3: float = params["theta3"]  # 趾关节固定偏角 (弧度)

    # a1, a2, a3, a4: 4 根刚性连杆当前的绝对朝向角 (弧度)
    a1, a2, a3, a4 = compute_absolute_angles(q, theta3)

    # x_tip: 足端爪尖在世界坐标系下的水平位置 X (米)
    # 物理意义：基点水平位置 q1 + 各连杆水平投影 (l * cos(angle)) 依次累加
    x_tip: float = q1 + l1 * np.cos(a1) + l2 * np.cos(a2) + l3 * np.cos(a3) + l4 * np.cos(a4)

    # z_tip: 足端爪尖在世界坐标系下的垂直高度 Z (米)
    # 物理意义：基点垂直高度 q2 + 各连杆垂直投影 (l * sin(angle)) 依次累加
    z_tip: float = q2 + l1 * np.sin(a1) + l2 * np.sin(a2) + l3 * np.sin(a3) + l4 * np.sin(a4)

    # 返回足端二维空间位置向量 [X_tip, Z_tip]^T
    return np.array([x_tip, z_tip], dtype=np.float64)


def forward_kinematics_toe(q: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：正向运动学函数 —— 计算趾关节旋转中心 (p_toe / J4) 在世界坐标系下的绝对坐标。

    【输入】：
    - q: 6 维广义坐标 [q1, q2, q3, q4, q5, q6]
    - params: 包含 l1~l3 与 theta3 的参数字典（趾关节不依赖脚趾长 l4）

    【输出】：
    - 长度为 2 的数组 [X_toe, Z_toe]，单位：米
    """
    q1: float = q[0]  # 髋关节连线中点水平坐标 X (米)
    q2: float = q[1]  # 髋关节连线中点垂直坐标 Z (米)

    l1: float = params["l1"]          # 大腿长 (米)
    l2: float = params["l2"]          # 小腿长 (米)
    l3: float = params["l3"]          # 偏移杆长 (米)
    theta3: float = params["theta3"]  # 趾关节固定偏角 (弧度)

    # 计算前 3 截连杆的绝对角（趾关节尚未经过脚趾连杆 l4，故不需要 a4）
    a1, a2, a3, _ = compute_absolute_angles(q, theta3)

    # x_toe: 趾关节在世界坐标系下的水平坐标 X (米)
    x_toe: float = q1 + l1 * np.cos(a1) + l2 * np.cos(a2) + l3 * np.cos(a3)

    # z_toe: 趾关节在世界坐标系下的垂直坐标 Z (米)
    z_toe: float = q2 + l1 * np.sin(a1) + l2 * np.sin(a2) + l3 * np.sin(a3)

    return np.array([x_toe, z_toe], dtype=np.float64)


def forward_kinematics_all_joints(q: np.ndarray, params: Dict[str, float],
                                  dx: float = None, dz: float = None) -> Dict[str, np.ndarray]:
    """
    【功能】：正向运动学调试与绘图利器 —— 计算机器人身体所有关键骨架点的坐标。

    【输入】：
    - q: 6 维广义坐标 [q1, q2, q3, q4, q5, q6]
    - params: 包含 l1~l4、theta3 与可选 body_length 的参数字典
    - dx, dz: 可选，机身质心相对髋关节在机身本体系下的偏移（米）；提供时才计算 com_body

    【输出】：
    - 字典，键为关节名（hip、ankle、base、toe、tip、nose、tail，可选 com_body），值为世界坐标 [X, Z]（米）
    """
    q1: float = q[0]  # 髋关节中点 X 位置 (米)
    q2: float = q[1]  # 髋关节中点 Z 位置 (米)
    q3: float = q[2]  # 机身俯仰角 (弧度)

    l1: float = params["l1"]                     # 大腿长 (米)
    l2: float = params["l2"]                     # 小腿长 (米)
    l3: float = params["l3"]                     # 偏移杆长 (米)
    l4: float = params["l4"]                     # 脚趾长 (米)
    body_length: float = params.get("body_length", 0.5)  # 机身基准长度 (米)
    theta3: float = params["theta3"]             # 趾关节固定角 (弧度)

    # a1~a4: 腿部 4 根连杆的绝对倾角 (弧度)
    a1, a2, a3, a4 = compute_absolute_angles(q, theta3)

    # p_hip: J1 髋关节世界坐标 [X, Z]，是腿部连杆链条的根原点
    p_hip: np.ndarray = np.array([q1, q2], dtype=np.float64)

    # p_ankle: J2 踝关节世界坐标，位于大腿 l1 远端
    p_ankle: np.ndarray = p_hip + np.array([l1 * np.cos(a1), l1 * np.sin(a1)])

    # p_base: J3 小腿末端基点世界坐标，位于小腿 l2 远端
    p_base: np.ndarray = p_ankle + np.array([l2 * np.cos(a2), l2 * np.sin(a2)])

    # p_toe: J4 趾关节世界坐标，位于偏移杆 l3 远端，是脚趾偏转轴心
    p_toe: np.ndarray = p_base + np.array([l3 * np.cos(a3), l3 * np.sin(a3)])

    # p_tip: 足端爪尖触地点世界坐标，位于脚趾 l4 末端
    p_tip: np.ndarray = p_toe + np.array([l4 * np.cos(a4), l4 * np.sin(a4)])

    # cos3, sin3: 机身俯仰角的三角函数值
    cos3: float = np.cos(q3)
    sin3: float = np.sin(q3)

    # R_b: 机身本体系到世界惯性坐标系的二维旋转变换矩阵 (2x2)
    R_b: np.ndarray = np.array([[cos3, -sin3], [sin3, cos3]])
    
    # p_nose: 示意机头在世界系中的坐标 (假设从髋关节沿机身中轴向前 30% 机身长)
    p_nose: np.ndarray = p_hip + R_b @ np.array([body_length * 0.3, 0.0])

    # p_tail: 示意机尾在世界系中的坐标 (假设从髋关节沿机身中轴向后 70% 机身长)
    p_tail: np.ndarray = p_hip + R_b @ np.array([-body_length * 0.7, 0.0])

    # joints: 汇总存储所有关键骨架关节位置的字典
    joints: Dict[str, np.ndarray] = {
        "hip": p_hip,
        "ankle": p_ankle,
        "base": p_base,
        "toe": p_toe,
        "tip": p_tip,
        "nose": p_nose,
        "tail": p_tail,
    }

    # 如果调用方传入了机身质心相对髋关节的本体系偏移 (dx, dz):
    if dx is not None and dz is not None:
        # p_com_body: 机身质心在世界坐标系下的绝对空间位置 (米)
        p_com_body: np.ndarray = p_hip + R_b @ np.array([dx, dz])
        joints["com_body"] = p_com_body

    return joints


def jacobian_tip(q: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：解析计算足端爪尖的雅可比矩阵 J_tip(q) (2x6 矩阵)。

    【输入】：
    - q: 6 维广义坐标 [q1, q2, q3, q4, q5, q6]
    - params: 包含 l1~l4 与 theta3 的参数字典

    【输出】：
    - J: 形状 (2, 6) 的雅可比矩阵，满足 V_tip = J @ q_dot
    """
    l1: float = params["l1"]          # 大腿长 (米)
    l2: float = params["l2"]          # 小腿长 (米)
    l3: float = params["l3"]          # 偏移杆长 (米)
    l4: float = params["l4"]          # 脚趾长 (米)
    theta3: float = params["theta3"]  # 固定角 (弧度)

    # 4 根连杆绝对角度
    a1, a2, a3, a4 = compute_absolute_angles(q, theta3)

    # s1~s4, c1~c4: 各连杆倾角的正弦值与余弦值
    s1: float = np.sin(a1)  # 大腿倾角 a1 的正弦
    c1: float = np.cos(a1)  # 大腿倾角 a1 的余弦
    s2: float = np.sin(a2)  # 小腿倾角 a2 的正弦
    c2: float = np.cos(a2)  # 小腿倾角 a2 的余弦
    s3: float = np.sin(a3)  # 偏移杆倾角 a3 的正弦
    c3: float = np.cos(a3)  # 偏移杆倾角 a3 的余弦
    s4: float = np.sin(a4)  # 脚趾连杆倾角 a4 的正弦
    c4: float = np.cos(a4)  # 脚趾连杆倾角 a4 的余弦

    # J: 初始化 2 行 6 列的全零双精度雅可比矩阵
    # 行 0 代表末端水平位置 X_tip，行 1 代表末端垂直位置 Z_tip
    # 列 0~5 分别对应广义坐标 q1, q2, q3, q4, q5, q6
    J: np.ndarray = np.zeros((2, 6), dtype=np.float64)

    # 第 1 列、第 2 列：机身整体平动对足端的影响
    # 机器人整体向前平移 1 米，脚尖也跟着向前平移 1 米：∂X_tip/∂q1 = 1.0, ∂Z_tip/∂q1 = 0.0
    J[0, 0] = 1.0
    # 机器人整体向上平移 1 米，脚尖也跟着向上平移 1 米：∂X_tip/∂q2 = 0.0, ∂Z_tip/∂q2 = 1.0
    J[1, 1] = 1.0

    # J_13, J_23: 机身俯仰角 q3 与 髋关节角 q4 对足端坐标的偏导数
    # J_13: 旋转角度变化 1 弧度引起末端 X 方向的变化率 (米/弧度)
    J_13: float = -(l1 * s1 + l2 * s2 + l3 * s3 + l4 * s4)
    # J_23: 旋转角度变化 1 弧度引起末端 Z 方向的变化率 (米/弧度)
    J_23: float = l1 * c1 + l2 * c2 + l3 * c3 + l4 * c4

    # 写入第 3 列 (q3 俯仰角) 与第 4 列 (q4 髋关节角)
    J[0, 2] = J_13
    J[1, 2] = J_23
    J[0, 3] = J_13
    J[1, 3] = J_23

    # 写入第 5 列 (q5 踝关节角)：大腿 l1 位于踝关节近端上方，不受其影响，仅有小腿、偏移杆、脚趾项
    J[0, 4] = -(l2 * s2 + l3 * s3 + l4 * s4)
    J[1, 4] = l2 * c2 + l3 * c3 + l4 * c4

    # 写入第 6 列 (q6 趾关节偏转角)：仅影响肢体末梢的脚趾连杆 l4
    J[0, 5] = -l4 * s4
    J[1, 5] = l4 * c4

    return J


def jacobian_toe(q: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：解析计算趾关节旋转中心 (p_toe) 的雅可比矩阵 J_toe(q) (2x6 矩阵)。

    【输入】：
    - q: 6 维广义坐标 [q1, q2, q3, q4, q5, q6]
    - params: 包含 l1~l3 与 theta3 的参数字典

    【输出】：
    - J: 形状 (2, 6) 的雅可比矩阵，满足 V_toe = J @ q_dot
    """
    l1: float = params["l1"]          # 大腿长 (米)
    l2: float = params["l2"]          # 小腿长 (米)
    l3: float = params["l3"]          # 偏移杆长 (米)
    theta3: float = params["theta3"]  # 固定角 (弧度)

    a1, a2, a3, _ = compute_absolute_angles(q, theta3)  # 趾关节不经过 l4，故忽略 a4

    # s1~s3, c1~c3: 大腿、小腿、偏移杆绝对倾角的正弦与余弦值
    s1: float = np.sin(a1)  # 大腿倾角 a1 的正弦
    c1: float = np.cos(a1)  # 大腿倾角 a1 的余弦
    s2: float = np.sin(a2)  # 小腿倾角 a2 的正弦
    c2: float = np.cos(a2)  # 小腿倾角 a2 的余弦
    s3: float = np.sin(a3)  # 偏移杆倾角 a3 的正弦
    c3: float = np.cos(a3)  # 偏移杆倾角 a3 的余弦

    # J: 趾关节雅可比矩阵，形状 2x6
    J: np.ndarray = np.zeros((2, 6), dtype=np.float64)

    # 浮基平动项
    J[0, 0] = 1.0
    J[1, 1] = 1.0

    # J_13, J_23: 俯仰角与髋关节角对趾关节中心位置的偏导数
    J_13: float = -(l1 * s1 + l2 * s2 + l3 * s3)
    J_23: float = l1 * c1 + l2 * c2 + l3 * c3
    J[0, 2] = J_13
    J[1, 2] = J_23
    J[0, 3] = J_13
    J[1, 3] = J_23

    # 踝关节角对趾关节的偏导数 (只含 l2, l3 项)
    J[0, 4] = -(l2 * s2 + l3 * s3)
    J[1, 4] = l2 * c2 + l3 * c3

    # 第 6 列：脚趾本身的弯折角 q6 绝不会改变趾关节铰链自身的位置，因此偏导严格全为 0.0
    J[0, 5] = 0.0
    J[1, 5] = 0.0

    return J


def jacobian_dot_tip(q: np.ndarray, q_dot: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：解析计算爪尖雅可比矩阵对时间的全微分矩阵 d(J_tip)/dt (2x6 矩阵)。

    【输入】：
    - q: 6 维广义坐标 [q1, q2, q3, q4, q5, q6]
    - q_dot: 6 维广义速度 [q1_dot, ..., q6_dot]
    - params: 包含 l1~l4 与 theta3 的参数字典

    【输出】：
    - J_dot: 形状 (2, 6) 的矩阵，用于加速度计算 a_tip = J @ q_ddot + J_dot @ q_dot
    """
    l1: float = params["l1"]          # 大腿长 (米)
    l2: float = params["l2"]          # 小腿长 (米)
    l3: float = params["l3"]          # 偏移杆长 (米)
    l4: float = params["l4"]          # 脚趾长 (米)
    theta3: float = params["theta3"]  # 固定角 (弧度)

    # a1~a4: 连杆绝对角度 (弧度)
    a1, a2, a3, a4 = compute_absolute_angles(q, theta3)
    # da1~da4: 连杆绝对角速度 (弧度/秒)
    da1, da2, da3, da4 = compute_absolute_angle_rates(q_dot)

    # s1~s4, c1~c4: 各连杆绝对倾角的正弦值与余弦值（用于后续导数公式）
    s1: float = np.sin(a1)  # 大腿连杆倾角 a1 的正弦
    c1: float = np.cos(a1)  # 大腿连杆倾角 a1 的余弦
    s2: float = np.sin(a2)  # 小腿连杆倾角 a2 的正弦
    c2: float = np.cos(a2)  # 小腿连杆倾角 a2 的余弦
    s3: float = np.sin(a3)  # 偏移杆倾角 a3 的正弦
    c3: float = np.cos(a3)  # 偏移杆倾角 a3 的余弦
    s4: float = np.sin(a4)  # 脚趾连杆倾角 a4 的正弦
    c4: float = np.cos(a4)  # 脚趾连杆倾角 a4 的余弦

    # J_dot: 雅可比时间导数矩阵，形状 2x6，单位：米/(弧度·秒)
    J_dot: np.ndarray = np.zeros((2, 6), dtype=np.float64)

    # 第 1 列、第 2 列因常数导数为零，自动保留 0.0

    # jdot_13: J_13 元素对时间的全导数 d(J_13)/dt
    jdot_13: float = -(l1 * c1 * da1 + l2 * c2 * da2 + l3 * c3 * da3 + l4 * c4 * da4)
    # jdot_23: J_23 元素对时间的全导数 d(J_23)/dt
    jdot_23: float = -(l1 * s1 * da1 + l2 * s2 * da2 + l3 * s3 * da3 + l4 * s4 * da4)

    # 填入第 3 列 (俯仰) 与第 4 列 (髋关节)
    J_dot[0, 2] = jdot_13
    J_dot[1, 2] = jdot_23
    J_dot[0, 3] = jdot_13
    J_dot[1, 3] = jdot_23

    # 填入第 5 列 (踝关节对时间的全导数)
    J_dot[0, 4] = -(l2 * c2 * da2 + l3 * c3 * da3 + l4 * c4 * da4)
    J_dot[1, 4] = -(l2 * s2 * da2 + l3 * s3 * da3 + l4 * s4 * da4)

    # 填入第 6 列 (趾关节对时间的全导数)
    J_dot[0, 5] = -l4 * c4 * da4
    J_dot[1, 5] = -l4 * s4 * da4

    return J_dot


def jacobian_dot_toe(q: np.ndarray, q_dot: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：解析计算趾关节雅可比矩阵对时间的全微分矩阵 d(J_toe)/dt (2x6 矩阵)。

    【输入】：
    - q: 6 维广义坐标 [q1, q2, q3, q4, q5, q6]
    - q_dot: 6 维广义速度 [q1_dot, ..., q6_dot]
    - params: 包含 l1~l3 与 theta3 的参数字典

    【输出】：
    - J_dot: 形状 (2, 6) 的矩阵，用于趾关节加速度计算
    """
    l1: float = params["l1"]          # 大腿长 (米)
    l2: float = params["l2"]          # 小腿长 (米)
    l3: float = params["l3"]          # 偏移杆长 (米)
    theta3: float = params["theta3"]  # 固定角 (弧度)

    a1, a2, a3, _ = compute_absolute_angles(q, theta3)
    da1, da2, da3, _ = compute_absolute_angle_rates(q_dot)  # 前三段连杆的绝对角速度

    # s1~s3, c1~c3: 大腿、小腿、偏移杆绝对倾角的正弦与余弦值
    s1: float = np.sin(a1)  # 大腿倾角 a1 的正弦
    c1: float = np.cos(a1)  # 大腿倾角 a1 的余弦
    s2: float = np.sin(a2)  # 小腿倾角 a2 的正弦
    c2: float = np.cos(a2)  # 小腿倾角 a2 的余弦
    s3: float = np.sin(a3)  # 偏移杆倾角 a3 的正弦
    c3: float = np.cos(a3)  # 偏移杆倾角 a3 的余弦

    # J_dot: 趾关节雅可比时间导数矩阵，形状 2x6
    J_dot: np.ndarray = np.zeros((2, 6), dtype=np.float64)

    # 俯仰与髋关节项
    jdot_13: float = -(l1 * c1 * da1 + l2 * c2 * da2 + l3 * c3 * da3)
    jdot_23: float = -(l1 * s1 * da1 + l2 * s2 * da2 + l3 * s3 * da3)
    J_dot[0, 2] = jdot_13
    J_dot[1, 2] = jdot_23
    J_dot[0, 3] = jdot_13
    J_dot[1, 3] = jdot_23

    # 踝关节项
    J_dot[0, 4] = -(l2 * c2 * da2 + l3 * c3 * da3)
    J_dot[1, 4] = -(l2 * s2 * da2 + l3 * s3 * da3)

    # 趾关节列严格为 0
    J_dot[0, 5] = 0.0
    J_dot[1, 5] = 0.0

    return J_dot
