"""
Step 1 运动学与雅可比矩阵物理验收测试 (tests/test_step1.py)

严格对齐 plan/milestone.md 验收标准：
1. 静态几何解析校验：特殊姿态误差 < 1e-12 m
2. 雅可比数值微分自检：50 组随机姿态，有限差分误差 < 1e-6
3. 时间导数 J_dot 校验：50 组随机状态，有限差分误差 < 1e-5
4. 可视化几何核验：生成 output/step1_kinematics_check.png 并确认连杆构型合理
"""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pytest
import numpy as np
import matplotlib
matplotlib.use("Agg")  # 无头模式绘图
import matplotlib.pyplot as plt

from src.kinematics import (
    load_kinematics_params,
    compute_absolute_angles,
    forward_kinematics_tip,
    forward_kinematics_toe,
    forward_kinematics_all_joints,
    jacobian_tip,
    jacobian_toe,
    jacobian_dot_tip,
    jacobian_dot_toe,
)


@pytest.fixture(scope="module")
def params():
    """加载 YAML 参数基准"""
    return load_kinematics_params()


def test_static_geometry_analytical(params):
    """
    验收标准 1：静态几何解析校验
    在特殊已知构型下，对比解析解与数值输出，误差必须严格 < 1e-12 m。
    """
    l1, l2, l3, l4 = params["l1"], params["l2"], params["l3"], params["l4"]
    theta3 = params["theta3"]

    # 构型 A: 全部平动为 0，连杆水平伸展特例（临时设 theta3=0 进行纯一维展开验证）
    params_straight = params.copy()
    params_straight["theta3"] = 0.0
    q_zero = np.zeros(6, dtype=np.float64)
    p_tip_zero = forward_kinematics_tip(q_zero, params_straight)
    expected_x = l1 + l2 + l3 + l4
    expected_y = 0.0
    err_zero = np.linalg.norm(p_tip_zero - np.array([expected_x, expected_y]))
    assert err_zero < 1e-12, f"特例平展构型误差超标: {err_zero:.2e} m"

    # 构型 B: 真实 theta3 参数下，各关节纯直角弯折测试
    # q = [1.0, 2.0, 0, pi/2, -pi/2, 0]
    # alpha1 = 0 + pi/2 = pi/2
    # alpha2 = pi/2 - pi/2 = 0
    # alpha3 = 0 + theta3 = theta3
    # alpha4 = theta3 + 0 = theta3
    q_ortho = np.array([1.0, 2.0, 0.0, np.pi / 2, -np.pi / 2, 0.0], dtype=np.float64)
    p_tip_ortho = forward_kinematics_tip(q_ortho, params)

    expected_ortho_x = 1.0 + l1 * np.cos(np.pi / 2) + l2 * np.cos(0.0) + (l3 + l4) * np.cos(theta3)
    expected_ortho_y = 2.0 + l1 * np.sin(np.pi / 2) + l2 * np.sin(0.0) + (l3 + l4) * np.sin(theta3)
    err_ortho = np.linalg.norm(p_tip_ortho - np.array([expected_ortho_x, expected_ortho_y]))
    assert err_ortho < 1e-12, f"直角折叠解析校验误差超标: {err_ortho:.2e} m"

    # 构型 C: 趾关节单独校验
    p_toe_ortho = forward_kinematics_toe(q_ortho, params)
    expected_toe_x = 1.0 + l1 * np.cos(np.pi / 2) + l2 * np.cos(0.0) + l3 * np.cos(theta3)
    expected_toe_y = 2.0 + l1 * np.sin(np.pi / 2) + l2 * np.sin(0.0) + l3 * np.sin(theta3)
    err_toe = np.linalg.norm(p_toe_ortho - np.array([expected_toe_x, expected_toe_y]))
    assert err_toe < 1e-12, f"趾关节解析校验误差超标: {err_toe:.2e} m"


def test_jacobian_finite_difference(params):
    """
    验收标准 2：雅可比数值微分自检
    在 50 组随机关节姿态 q 下，中心有限差分 (delta_q = 1e-7) 与解析雅可比最大绝对误差 < 1e-6。
    """
    np.random.seed(42)  # 固定随机种子，保证可复现
    num_samples = 50
    delta_q = 1e-7

    max_err_tip = 0.0
    max_err_toe = 0.0

    for _ in range(num_samples):
        # 随机采样广义坐标 (位置与角度合理区间)
        q = np.array([
            np.random.uniform(-1.0, 1.0),
            np.random.uniform(0.1, 1.0),
            np.random.uniform(-np.pi, np.pi),
            np.random.uniform(-np.pi, np.pi),
            np.random.uniform(-np.pi, np.pi),
            np.random.uniform(-np.pi / 2, np.pi / 2),
        ], dtype=np.float64)

        # 解析雅可比
        J_tip_analytical = jacobian_tip(q, params)
        J_toe_analytical = jacobian_toe(q, params)

        # 中心有限差分计算
        J_tip_fd = np.zeros((2, 6), dtype=np.float64)
        J_toe_fd = np.zeros((2, 6), dtype=np.float64)

        for j in range(6):
            dq = np.zeros(6, dtype=np.float64)
            dq[j] = delta_q

            p_tip_plus = forward_kinematics_tip(q + dq, params)
            p_tip_minus = forward_kinematics_tip(q - dq, params)
            J_tip_fd[:, j] = (p_tip_plus - p_tip_minus) / (2.0 * delta_q)

            p_toe_plus = forward_kinematics_toe(q + dq, params)
            p_toe_minus = forward_kinematics_toe(q - dq, params)
            J_toe_fd[:, j] = (p_toe_plus - p_toe_minus) / (2.0 * delta_q)

        err_tip = np.max(np.abs(J_tip_analytical - J_tip_fd))
        err_toe = np.max(np.abs(J_toe_analytical - J_toe_fd))

        if err_tip > max_err_tip:
            max_err_tip = err_tip
        if err_toe > max_err_toe:
            max_err_toe = err_toe

    print(f"\n[Step 1 验收] 爪尖雅可比 50 组样本最大有限差分误差: {max_err_tip:.3e}")
    print(f"[Step 1 验收] 趾关节雅可比 50 组样本最大有限差分误差: {max_err_toe:.3e}")

    assert max_err_tip < 1e-6, f"爪尖雅可比误差超标: {max_err_tip:.2e} >= 1e-6"
    assert max_err_toe < 1e-6, f"趾关节雅可比误差超标: {max_err_toe:.2e} >= 1e-6"


def test_jacobian_time_derivative(params):
    """
    验收标准 3：雅可比时间导数 J_dot 校验
    在 50 组随机状态 (q, q_dot) 下，时间中心有限差分 (delta_t = 1e-6) 与解析 J_dot 误差 < 1e-5。
    """
    np.random.seed(1024)
    num_samples = 50
    delta_t = 1e-6

    max_err_jdot_tip = 0.0
    max_err_jdot_toe = 0.0

    for _ in range(num_samples):
        q = np.random.uniform(-np.pi, np.pi, size=6)
        q[0] = np.random.uniform(-1.0, 1.0)
        q[1] = np.random.uniform(0.1, 1.0)
        q_dot = np.random.uniform(-5.0, 5.0, size=6)

        # 解析时间导数
        J_dot_tip_ana = jacobian_dot_tip(q, q_dot, params)
        J_dot_toe_ana = jacobian_dot_toe(q, q_dot, params)

        # 时间有限差分 J_dot ≈ (J(q + q_dot*dt) - J(q - q_dot*dt)) / (2*dt)
        q_next = q + q_dot * delta_t
        q_prev = q - q_dot * delta_t

        J_tip_next = jacobian_tip(q_next, params)
        J_tip_prev = jacobian_tip(q_prev, params)
        J_dot_tip_fd = (J_tip_next - J_tip_prev) / (2.0 * delta_t)

        J_toe_next = jacobian_toe(q_next, params)
        J_toe_prev = jacobian_toe(q_prev, params)
        J_dot_toe_fd = (J_toe_next - J_toe_prev) / (2.0 * delta_t)

        err_tip = np.max(np.abs(J_dot_tip_ana - J_dot_tip_fd))
        err_toe = np.max(np.abs(J_dot_toe_ana - J_dot_toe_fd))

        if err_tip > max_err_jdot_tip:
            max_err_jdot_tip = err_tip
        if err_toe > max_err_jdot_toe:
            max_err_jdot_toe = err_toe

    print(f"\n[Step 1 验收] 爪尖雅可比时间导数最大差分误差: {max_err_jdot_tip:.3e}")
    print(f"[Step 1 验收] 趾关节雅可比时间导数最大差分误差: {max_err_jdot_toe:.3e}")

    assert max_err_jdot_tip < 1e-5, f"J_dot_tip 误差超标: {max_err_jdot_tip:.2e} >= 1e-5"
    assert max_err_jdot_toe < 1e-5, f"J_dot_toe 误差超标: {max_err_jdot_toe:.2e} >= 1e-5"


def test_visual_geometry_plot(params):
    """
    验收标准 4：可视化几何核验图生成
    生成 output/step1_kinematics_check.png，展示初始跳跃构型下的机器人骨架与关节中心。
    """
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "output"))
    os.makedirs(output_dir, exist_ok=True)
    png_path = os.path.join(output_dir, "step1_kinematics_check.png")

    # 从 YAML 加载论文初始角度配置 (仿真初始跳跃下蹲姿态)
    # pitch = 10 deg, hip = 135 deg, ankle = 145 deg, toe_deflection = 25 deg
    q_init = np.array([
        0.0,
        0.25,  # 假定髋关节初始高度
        np.deg2rad(10.0),
        np.deg2rad(135.0),
        np.deg2rad(145.0),
        np.deg2rad(25.0),
    ], dtype=np.float64)

    # 质心偏移可行域中位候选值 dx=-0.01m, dy=0.02m (标注为候选示意)
    dx_est = -0.01
    dy_est = 0.02

    joints = forward_kinematics_all_joints(q_init, params, dx=dx_est, dy=dy_est)

    fig, ax = plt.subplots(figsize=(9, 7), dpi=150)

    # 绘制地面
    ax.axhline(0, color="gray", linestyle="--", linewidth=1.2, label="Ground (Y=0)")

    # 绘制机身基准线 (尾巴到机头)
    nose = joints["nose"]
    tail = joints["tail"]
    ax.plot([tail[0], nose[0]], [tail[1], nose[1]], color="#2b5c8f", linewidth=4.0, label="Body Trunk (x_B axis)")

    # 绘制腿部串联连杆链: Hip -> Ankle -> Base -> Toe -> Tip
    leg_x = [joints["hip"][0], joints["ankle"][0], joints["base"][0], joints["toe"][0], joints["tip"][0]]
    leg_y = [joints["hip"][1], joints["ankle"][1], joints["base"][1], joints["toe"][1], joints["tip"][1]]
    ax.plot(leg_x, leg_y, color="#e65100", linewidth=3.0, marker="o", markersize=6, label="Leg Kinematic Chain")

    # 关键点标注与标记
    ax.plot(joints["hip"][0], joints["hip"][1], "ro", markersize=9, label="J1: Hip (q1, q2)")
    ax.plot(joints["ankle"][0], joints["ankle"][1], "go", markersize=8, label="J2: Ankle")
    ax.plot(joints["base"][0], joints["base"][1], "co", markersize=6, label="J3: Lower Limb Base")
    ax.plot(joints["toe"][0], joints["toe"][1], "mo", markersize=8, label="J4: Toe Joint (p_toe)")
    ax.plot(joints["tip"][0], joints["tip"][1], "ks", markersize=9, label="Contact Tip (p_claw)")

    # 质心标注
    if "com_body" in joints:
        com = joints["com_body"]
        ax.plot(com[0], com[1], "y*", markersize=12, label=f"CoM (dx={dx_est}m, dy={dy_est}m)")

    # 文字标注
    ax.text(joints["hip"][0] + 0.01, joints["hip"][1] + 0.015, "Hip (q1,q2)", fontsize=10, fontweight="bold")
    ax.text(joints["ankle"][0] + 0.01, joints["ankle"][1], "Ankle", fontsize=9)
    ax.text(joints["toe"][0] + 0.01, joints["toe"][1], "Toe Joint", fontsize=9)
    ax.text(joints["tip"][0] + 0.01, joints["tip"][1] - 0.02, "Tip Contact", fontsize=9, fontweight="bold")

    ax.set_title("EPFL RAVEN - Step 1 Kinematics & Linkage Check\n(Initial Takeoff Configuration)", fontsize=13, fontweight="bold")
    ax.set_xlabel("X [m] (Forward)", fontsize=11)
    ax.set_ylabel("Y [m] (Vertical)", fontsize=11)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper left", framealpha=0.9, fontsize=9)
    ax.set_aspect("equal", "datalim")

    plt.tight_layout()
    plt.savefig(png_path)
    plt.close()

    assert os.path.exists(png_path), f"校验图表未生成: {png_path}"
    assert os.path.getsize(png_path) > 1000, "生成图表文件过小，可能绘制异常"
    print(f"\n[Step 1 验收] 几何核验图成功生成: {png_path}")
