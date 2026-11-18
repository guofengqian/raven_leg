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

import os
from typing import Dict, Any, Tuple
import yaml
import numpy as np


def load_kinematics_params(config_path: str = None) -> Dict[str, float]:
    """
    【功能】：从 YAML 配置文件中加载机器人机构的几何尺寸参数。
    
    【设计原则 - 零脑补与单一事实来源】：
    在专业工程与科研中，严禁在算法代码里硬编码诸如 0.12, 0.06 这样的数字（称为 Magic Numbers）。
    所有物理常数必须统一从 config/robot_paras.yaml 集中读取。如果将来硬件改版调整了连杆长度，
    只需改 YAML 文件，这里的所有运动学方程自动保持正确。

    【参数解析】：
    - config_path: yaml 文件路径。如果为空，自动寻找项目标准路径 config/robot_paras.yaml。

    【返回值】：
    包含各杆件长度与角度的字典：
    - l1: 上肢（大腿）长度，单位：米
    - l2: 下肢（小腿/跗跖骨）长度，单位：米
    - l3: 趾关节偏移杆长度，单位：米
    - l4: 脚趾长度，单位：米
    - theta3: 趾关节机械固定偏移角度，已转换为【弧度制 (rad)】
    - body_length: 机身参考长度，单位：米
    """
    if config_path is None:
        # 自动获取当前脚本所在的绝对目录，并回退一级找到 config 目录
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.abspath(os.path.join(curr_dir, "..", "config", "robot_paras.yaml"))

    # 以只读 UTF-8 编码安全读取 yaml 文件
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    # 提取 simulation -> geometry 节点下的几何数据
    geom = cfg["simulation"]["geometry"]
    params = {
        "l1": float(geom["l1_m"]),                       # 论文 Table 2: 0.12 m
        "l2": float(geom["l2_m"]),                       # 论文 Table 2: 0.12 m
        "l3": float(geom["l3_m"]),                       # 论文 Table 2: 0.023 m
        "l4": float(geom["l4_m"]),                       # 论文 Table 2: 0.06 m
        "theta3": float(np.deg2rad(geom["theta3_deg"])), # 论文 Table 2: 45° -> 转换成弧度 π/4
        "body_length": float(geom["body_length_m"]),     # 论文 Table 2: 0.5 m
    }
    return params


def compute_absolute_angles(q: np.ndarray, theta3: float) -> Tuple[float, float, float, float]:
    """
    【功能】：计算腿部 4 根连杆相对世界坐标系 +X 轴（水平正前方）的绝对姿态角 alpha_1 ~ alpha_4。

    【小白数学原理解析】：
    - 相对角 vs 绝对角：
      * 传感器或电机编码器测出的是“关节相对角”（比如大腿相对于机身转了多少度）；
      * 但要算出脚尖在地球（惯性系）上的空间位置，必须知道每根杆件相对于地面水平轴的“绝对倾角”！
    - 级联求和原理（串联机械臂的逐级姿态叠加）：
      * 连杆 1（大腿 l1）：自身转了 q4，但基座（机身）已经抬头转了 q3，
        所以绝对倾角 alpha1 = q3 + q4。
      * 连杆 2（小腿 l2）：在上肢倾角 alpha1 的基础上，膝/踝关节相对弯折了 q5，
        所以绝对倾角 alpha2 = alpha1 + q5 = q3 + q4 + q5。
      * 连杆 3（趾关节偏移杆 l3）：小腿末端装有一个固定 45° (theta3) 的刚性偏转座（注意这是结构常数，不是活动关节），
        所以绝对倾角 alpha3 = alpha2 + theta3 = q3 + q4 + q5 + theta3。
      * 连杆 4（脚趾 l4）：脚趾在趾关节处受被动弹簧约束偏转了 q6，
        所以绝对倾角 alpha4 = alpha3 + q6 = q3 + q4 + q5 + theta3 + q6。

    【输入】：
    - q: 6 维广义坐标向量 [q1, q2, q3, q4, q5, q6]^T
    - theta3: 固定的趾关节偏移角（弧度）

    【输出】：
    - (alpha1, alpha2, alpha3, alpha4): 四根连杆的绝对倾角（弧度）
    """
    q3, q4, q5, q6 = q[2], q[3], q[4], q[5]

    alpha1 = q3 + q4
    alpha2 = alpha1 + q5
    alpha3 = alpha2 + theta3
    alpha4 = alpha3 + q6

    return alpha1, alpha2, alpha3, alpha4


def compute_absolute_angle_rates(q_dot: np.ndarray) -> Tuple[float, float, float, float]:
    """
    【功能】：计算 4 根连杆绝对倾角的时间变化率（即绝对角速度 d_alpha / dt）。

    【小白微积分原理解析】：
    对上面的绝对角求时间导数 d/dt：
    - d(alpha1)/dt = q3_dot + q4_dot
    - d(alpha2)/dt = q3_dot + q4_dot + q5_dot
    - d(alpha3)/dt = d(alpha2 + theta3)/dt = alpha2_dot + 0 = alpha2_dot
      （★特别注意：因为 theta3 是机械加工固定的常数角 45°，它的时间导数严格为 0！）
    - d(alpha4)/dt = alpha3_dot + q6_dot = q3_dot + q4_dot + q5_dot + q6_dot

    【输入】：
    - q_dot: 6 维广义速度向量 [q1_dot, q2_dot, q3_dot, q4_dot, q5_dot, q6_dot]^T

    【输出】：
    - (alpha1_dot, alpha2_dot, alpha3_dot, alpha4_dot): 4 个绝对角速度 (rad/s)
    """
    q3_dot, q4_dot, q5_dot, q6_dot = q_dot[2], q_dot[3], q_dot[4], q_dot[5]

    alpha1_dot = q3_dot + q4_dot
    alpha2_dot = alpha1_dot + q5_dot
    alpha3_dot = alpha2_dot               # 固定角无角速度贡献
    alpha4_dot = alpha3_dot + q6_dot

    return alpha1_dot, alpha2_dot, alpha3_dot, alpha4_dot


def forward_kinematics_tip(q: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：正向运动学函数 —— 计算足端爪尖 (p_claw / p_tip) 在世界坐标系下的绝对坐标。
    
    【论文对应公式】：
    对应论文 Methods 第 14 页式 (7):
    p_claw = p_toe + R(alpha4) * [l4, 0]^T

    【几何直观推导】：
    脚尖的最终坐标，就是从髋关节起点出发，沿每一截连杆的投影向量一路“头尾相接累加”：
    - X 方向总位置 = 髋关节水平起点 q1 + 各连杆沿水平方向的余弦投影累加：
      X_tip = q1 + l1*cos(alpha1) + l2*cos(alpha2) + l3*cos(alpha3) + l4*cos(alpha4)
    - Z 方向总位置 = 髋关节垂直起点 q2 + 各连杆沿竖直方向的正弦投影累加：
      Z_tip = q2 + l1*sin(alpha1) + l2*sin(alpha2) + l3*sin(alpha3) + l4*sin(alpha4)

    【输入】：
    - q: 6 维广义坐标 [q1, q2, q3, q4, q5, q6]^T
    - params: 包含 l1~l4 与 theta3 的参数字典

    【输出】：
    - 一维数组 [x_tip, z_tip]，单位：米
    """
    q1, q2 = q[0], q[1]
    l1, l2, l3, l4 = params["l1"], params["l2"], params["l3"], params["l4"]
    theta3 = params["theta3"]

    # 1. 算出 4 根杆件当前的绝对朝向角
    a1, a2, a3, a4 = compute_absolute_angles(q, theta3)

    # 2. 几何投影逐段累加求和
    x_tip = q1 + l1 * np.cos(a1) + l2 * np.cos(a2) + l3 * np.cos(a3) + l4 * np.cos(a4)
    z_tip = q2 + l1 * np.sin(a1) + l2 * np.sin(a2) + l3 * np.sin(a3) + l4 * np.sin(a4)

    return np.array([x_tip, z_tip], dtype=np.float64)


def forward_kinematics_toe(q: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：正向运动学函数 —— 计算趾关节旋转中心 (p_toe / J4) 在世界坐标系下的绝对坐标。

    【论文对应公式】：
    对应论文 Methods 第 14 页式 (6):
    p_toe = [q1, q2]^T + R(alpha1)*[l1, 0]^T + R(alpha2)*[l2, 0]^T + R(alpha3)*[l3, 0]^T

    【为什么单独需要这个函数？】：
    在全足触地蹬伸阶段（论文式 5），趾关节也是贴地的接触点。为了施加全足约束方程，
    必须单独提取出趾关节的坐标与雅可比矩阵。
    它与 foot_tip 的区别在于：它只累加到偏移杆 l3，尚未计入末端脚趾连杆 l4。
    """
    q1, q2 = q[0], q[1]
    l1, l2, l3 = params["l1"], params["l2"], params["l3"]
    theta3 = params["theta3"]

    a1, a2, a3, _ = compute_absolute_angles(q, theta3)

    x_toe = q1 + l1 * np.cos(a1) + l2 * np.cos(a2) + l3 * np.cos(a3)
    z_toe = q2 + l1 * np.sin(a1) + l2 * np.sin(a2) + l3 * np.sin(a3)

    return np.array([x_toe, z_toe], dtype=np.float64)


def forward_kinematics_all_joints(q: np.ndarray, params: Dict[str, float],
                                  dx: float = None, dz: float = None) -> Dict[str, np.ndarray]:
    """
    【功能】：正向运动学调试与绘图利器 —— 计算机器人身体所有关键骨架点的坐标。
    
    【输出字典包含各个关键点】：
    - "hip": 髋关节 J1 [q1, q2]，它是整个腿部链条的根原点
    - "ankle": 踝关节 J2，位于上肢 l1 末端
    - "base": 跗跖骨末端 J3，位于下肢 l2 末端、偏移杆根部
    - "toe": 趾关节 J4，位于偏移杆 l3 末端，脚趾旋转轴
    - "tip": 足端爪尖，脚趾 l4 末端，与地面相互作用的最前端
    - "nose": 示意机头位置
    - "tail": 示意机尾位置
    - "com_body": 机身质心位置（如果传入了质心相对髋关节的偏移 dx, dz）
    """
    q1, q2 = q[0], q[1]
    q3 = q[2]
    l1, l2, l3, l4 = params["l1"], params["l2"], params["l3"], params["l4"]
    body_length = params.get("body_length", 0.5)
    theta3 = params["theta3"]

    a1, a2, a3, a4 = compute_absolute_angles(q, theta3)

    # 1. 腿链上的骨架连接点
    p_hip = np.array([q1, q2], dtype=np.float64)
    p_ankle = p_hip + np.array([l1 * np.cos(a1), l1 * np.sin(a1)])
    p_base = p_ankle + np.array([l2 * np.cos(a2), l2 * np.sin(a2)])
    p_toe = p_base + np.array([l3 * np.cos(a3), l3 * np.sin(a3)])
    p_tip = p_toe + np.array([l4 * np.cos(a4), l4 * np.sin(a4)])

    # 2. 机身轮廓线估算（以髋关节为基准，按机身俯仰角 q3 进行平面旋转变换）
    # R(q3) 是 2x2 旋转矩阵
    cos3, sin3 = np.cos(q3), np.sin(q3)
    R_b = np.array([[cos3, -sin3], [sin3, cos3]])
    
    # 假设机头向前延伸 30% 长度，尾巴向后延伸 70% 长度
    p_nose = p_hip + R_b @ np.array([body_length * 0.3, 0.0])
    p_tail = p_hip + R_b @ np.array([-body_length * 0.7, 0.0])

    joints = {
        "hip": p_hip,
        "ankle": p_ankle,
        "base": p_base,
        "toe": p_toe,
        "tip": p_tip,
        "nose": p_nose,
        "tail": p_tail,
    }

    # 3. 机身质心计算：机身质心坐标 = 髋关节坐标 + R(q3) * 质心在本体系的偏移量 [dx, dz]^T
    if dx is not None and dz is not None:
        p_com_body = p_hip + R_b @ np.array([dx, dz])
        joints["com_body"] = p_com_body

    return joints


def jacobian_tip(q: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：解析计算足端爪尖的雅可比矩阵 J_tip(q) (2x6 矩阵)。
    
    【小白通俗微积分原理】：
    雅可比矩阵是末端位移向量 p_tip 对 6 个广义坐标向量 q 的偏导数矩阵：
        J_tip = [ ∂p_tip/∂q1,  ∂p_tip/∂q2,  ∂p_tip/∂q3,  ∂p_tip/∂q4,  ∂p_tip/∂q5,  ∂p_tip/∂q6 ]
    
    我们逐列看看为什么公式是这样写出来的（链式求导）：
    - 第 1 列 (∂/∂q1)：
      x_tip = q1 + ...，所以 ∂x_tip/∂q1 = 1；z_tip 里没有 q1，所以 ∂z_tip/∂q1 = 0。
      物理含义：机器人水平整体平移 1 米，脚尖也跟着水平平移 1 米。第一列恒为 [1, 0]^T。
    - 第 2 列 (∂/∂q2)：
      同理，竖直方向平移对脚尖的偏导恒为 [0, 1]^T。
    - 第 3 列 (∂/∂q3 机身俯仰角) 与 第 4 列 (∂/∂q4 髋关节角)：
      由于 alpha1 = q3 + q4, alpha2 = q3 + q4 + q5 ...
      对 q3 和 q4 求偏导，链式因子 ∂alpha_i / ∂q3 和 ∂alpha_i / ∂q4 全部都等于 1！
      因为 d(cos(a))/da = -sin(a)，d(sin(a))/da = cos(a)：
      所以偏导数为：
        J_13 = J_14 = -(l1*sin(a1) + l2*sin(a2) + l3*sin(a3) + l4*sin(a4))
        J_23 = J_24 =   l1*cos(a1) + l2*cos(a2) + l3*cos(a3) + l4*cos(a4)
      物理含义：整个机身昂头 1 度，和大腿转动 1 度，对脚尖空间位移的瞬间影响是完全等价的！
    - 第 5 列 (∂/∂q5 踝关节角)：
      踝关节位于大腿末端，它的转动完全影响不到大腿 l1（因为 l1 在它上面），
      所以这一列里只含有后面的 l2, l3, l4 项，没有 l1 项！
    - 第 6 列 (∂/∂q6 趾关节角)：
      脚趾是肢体最末节，它的偏转完全不会带动大腿和小腿，所以这一列只含有 l4 项！

    【返回】：
    np.ndarray (2, 6) 的双精度浮点数矩阵。
    """
    l1, l2, l3, l4 = params["l1"], params["l2"], params["l3"], params["l4"]
    theta3 = params["theta3"]

    # 计算各连杆绝对角
    a1, a2, a3, a4 = compute_absolute_angles(q, theta3)

    # 预先计算正弦与余弦值，避免重复计算提升效率
    s1, c1 = np.sin(a1), np.cos(a1)
    s2, c2 = np.sin(a2), np.cos(a2)
    s3, c3 = np.sin(a3), np.cos(a3)
    s4, c4 = np.sin(a4), np.cos(a4)

    J = np.zeros((2, 6), dtype=np.float64)

    # 第 1 列、第 2 列：浮基平动项
    J[0, 0] = 1.0
    J[1, 1] = 1.0

    # 第 3 列 (q3: 俯仰角) 与 第 4 列 (q4: 髋关节角)
    J_13 = -(l1 * s1 + l2 * s2 + l3 * s3 + l4 * s4)
    J_23 = l1 * c1 + l2 * c2 + l3 * c3 + l4 * c4
    J[0, 2] = J_13
    J[1, 2] = J_23
    J[0, 3] = J_13
    J[1, 3] = J_23

    # 第 5 列 (q5: 踝关节角，影响 l2, l3, l4)
    J[0, 4] = -(l2 * s2 + l3 * s3 + l4 * s4)
    J[1, 4] = l2 * c2 + l3 * c3 + l4 * c4

    # 第 6 列 (q6: 趾关节角，仅影响脚趾连杆 l4)
    J[0, 5] = -l4 * s4
    J[1, 5] = l4 * c4

    return J


def jacobian_toe(q: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：解析计算趾关节旋转中心 (p_toe) 的雅可比矩阵 J_toe(q) (2x6 矩阵)。
    
    【小白理解点】：
    趾关节位于脚趾连杆 l4 的根部，因此脚趾本身的偏转角 q6 不可能影响到趾关节自己的位置！
    所以：J_toe 的第 6 列（最后一列）必然严格全为 0.0。
    前 5 列的计算方式与 jacobian_tip 完全一致，只是去掉了连杆 l4 的贡献。
    """
    l1, l2, l3 = params["l1"], params["l2"], params["l3"]
    theta3 = params["theta3"]

    a1, a2, a3, _ = compute_absolute_angles(q, theta3)

    s1, c1 = np.sin(a1), np.cos(a1)
    s2, c2 = np.sin(a2), np.cos(a2)
    s3, c3 = np.sin(a3), np.cos(a3)

    J = np.zeros((2, 6), dtype=np.float64)

    # 浮基平动列
    J[0, 0] = 1.0
    J[1, 1] = 1.0

    # 俯仰角与髋关节列
    J_13 = -(l1 * s1 + l2 * s2 + l3 * s3)
    J_23 = l1 * c1 + l2 * c2 + l3 * c3
    J[0, 2] = J_13
    J[1, 2] = J_23
    J[0, 3] = J_13
    J[1, 3] = J_23

    # 踝关节列
    J[0, 4] = -(l2 * s2 + l3 * s3)
    J[1, 4] = l2 * c2 + l3 * c3

    # 趾关节列（趾关节位置不受自身转角影响，导数为 0）
    J[0, 5] = 0.0
    J[1, 5] = 0.0

    return J


def jacobian_dot_tip(q: np.ndarray, q_dot: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：解析计算爪尖雅可比矩阵对时间的全微分矩阵 d(J_tip)/dt (2x6 矩阵)。

    【小白微积分推导过程】：
    我们看 J 矩阵中的某一项，比如 J_16 = -l4 * sin(a4)。
    当机器人运动时，角度 a4 随时间在变。根据微积分链式求导法则：
        d/dt [-l4 * sin(a4)] = -l4 * cos(a4) * (d(a4)/dt) = -l4 * cos(a4) * a4_dot
    
    同理：
        d/dt [l4 * cos(a4)] = -l4 * sin(a4) * a4_dot

    通过把 J_tip 的每一个非零元素对时间 t 运用全微分，就能求出精确的 J_dot 矩阵！
    注意：因为 J 的前两列是纯常数 [1, 0]^T 和 [0, 1]^T，常数对时间的导数严格为 0，
    所以 J_dot 的前两列严格全为 0。

    【输入】：
    - q: 6 维广义坐标向量
    - q_dot: 6 维广义角速度向量
    - params: 几何参数字典

    【输出】：
    - np.ndarray (2, 6) 的时间导数矩阵
    """
    l1, l2, l3, l4 = params["l1"], params["l2"], params["l3"], params["l4"]
    theta3 = params["theta3"]

    # 1. 获得绝对倾角 a1~a4 以及绝对角速度 da1~da4
    a1, a2, a3, a4 = compute_absolute_angles(q, theta3)
    da1, da2, da3, da4 = compute_absolute_angle_rates(q_dot)

    s1, c1 = np.sin(a1), np.cos(a1)
    s2, c2 = np.sin(a2), np.cos(a2)
    s3, c3 = np.sin(a3), np.cos(a3)
    s4, c4 = np.sin(a4), np.cos(a4)

    J_dot = np.zeros((2, 6), dtype=np.float64)

    # 前两列（常数的导数 = 0）自动为 0.0

    # 第 3、4 列对时间的全导数
    # d/dt [-(l1*s1 + l2*s2 + ...)] = -(l1*c1*da1 + l2*c2*da2 + ...)
    # d/dt [ l1*c1 + l2*c2 + ...]  = -(l1*s1*da1 + l2*s2*da2 + ...)
    jdot_13 = -(l1 * c1 * da1 + l2 * c2 * da2 + l3 * c3 * da3 + l4 * c4 * da4)
    jdot_23 = -(l1 * s1 * da1 + l2 * s2 * da2 + l3 * s3 * da3 + l4 * s4 * da4)
    J_dot[0, 2] = jdot_13
    J_dot[1, 2] = jdot_23
    J_dot[0, 3] = jdot_13
    J_dot[1, 3] = jdot_23

    # 第 5 列对时间的全导数
    J_dot[0, 4] = -(l2 * c2 * da2 + l3 * c3 * da3 + l4 * c4 * da4)
    J_dot[1, 4] = -(l2 * s2 * da2 + l3 * s3 * da3 + l4 * s4 * da4)

    # 第 6 列对时间的全导数
    J_dot[0, 5] = -l4 * c4 * da4
    J_dot[1, 5] = -l4 * s4 * da4

    return J_dot


def jacobian_dot_toe(q: np.ndarray, q_dot: np.ndarray, params: Dict[str, float]) -> np.ndarray:
    """
    【功能】：解析计算趾关节雅可比矩阵对时间的全微分矩阵 d(J_toe)/dt (2x6 矩阵)。
    
    【小白理解点】：
    与 jacobian_dot_tip 的区别同样是去掉了末端 l4 脚趾项；
    前两列（平动）与最后一列（脚趾偏转）的导数均严格为 0。
    """
    l1, l2, l3 = params["l1"], params["l2"], params["l3"]
    theta3 = params["theta3"]

    a1, a2, a3, _ = compute_absolute_angles(q, theta3)
    da1, da2, da3, _ = compute_absolute_angle_rates(q_dot)

    s1, c1 = np.sin(a1), np.cos(a1)
    s2, c2 = np.sin(a2), np.cos(a2)
    s3, c3 = np.sin(a3), np.cos(a3)

    J_dot = np.zeros((2, 6), dtype=np.float64)

    jdot_13 = -(l1 * c1 * da1 + l2 * c2 * da2 + l3 * c3 * da3)
    jdot_23 = -(l1 * s1 * da1 + l2 * s2 * da2 + l3 * s3 * da3)
    J_dot[0, 2] = jdot_13
    J_dot[1, 2] = jdot_23
    J_dot[0, 3] = jdot_13
    J_dot[1, 3] = jdot_23

    J_dot[0, 4] = -(l2 * c2 * da2 + l3 * c3 * da3)
    J_dot[1, 4] = -(l2 * s2 * da2 + l3 * s3 * da3)

    J_dot[0, 5] = 0.0
    J_dot[1, 5] = 0.0

    return J_dot
