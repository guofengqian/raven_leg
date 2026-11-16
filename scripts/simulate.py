"""
加载 V1 基础腿模型，在 MuJoCo viewer 中观看重力下的自然摆动。

用法（在项目根目录执行）:
    python3 scripts/simulate.py
"""

import math
import os
import select
import shutil
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


def _ensure_mjpython_on_macos() -> None:
    """macOS 上 GLFW 窗口必须跑在 mjpython 里；若当前不是，则自动切换过去。"""
    if sys.platform != "darwin":
        return
    # mjpython 启动后会带上这个环境变量
    if "MJPYTHON_BIN" in os.environ:
        return

    candidates = [
        shutil.which("mjpython"),
        "/Library/Frameworks/Python.framework/Versions/3.12/bin/mjpython",
        str(Path(sys.executable).with_name("mjpython")),
    ]
    mjpython = next((p for p in candidates if p and Path(p).exists()), None)
    if mjpython is None:
        print(
            "找不到 mjpython。请确认已安装 mujoco，或手动运行完整路径，例如:\n"
            "  /Library/Frameworks/Python.framework/Versions/3.12/bin/mjpython "
            "scripts/simulate.py"
        )
        sys.exit(1)

    print(f"正在用 mjpython 重新启动: {mjpython}")
    os.execv(mjpython, [mjpython, *sys.argv])


def _apply_start_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """回到倾斜起始姿势，速度清零，准备再次摆动。"""
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)


def _enter_pressed() -> bool:
    """非阻塞检测：终端里是否按了回车。"""
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


# ===== 电机 PD 控制参数（V2）=====
# 顺序与 data.ctrl 一致：[hip, ankle]。
# kp：比例增益，越大越“有劲”把腿拉向目标；kd：微分增益，抑制来回振荡。
MOTOR_KP = [30.0, 15.0]
MOTOR_KD = [3.0, 1.5]

# 预设目标姿势列表（弧度），每条 = [hip 目标角, ankle 目标角]。
# 终端按回车会循环切换，观察电机主动把腿运动到不同角度。
MOTOR_TARGETS = [
    [0.7854, -0.30],   # ① 起始：大腿 +45°
    [0.0000, -0.30],   # ② 竖直向下
    [-0.7854, -0.30],  # ③ 另一侧：大腿 -45°
    [0.7854, -0.30],   # ④ 回到起始
]


def _pd_control(model: mujoco.MjModel, data: mujoco.MjData, target) -> None:
    """对 hip / ankle 做 PD 位置伺服，把算出的力矩写进 data.ctrl。
    torque = kp * (目标角 - 当前角) - kd * 当前角速度，
    再限幅到该电机的 ctrlrange（XML 里定义的最大力矩）。"""
    for i in range(model.nu):
        q = float(data.qpos[i])
        qv = float(data.qvel[i])
        torque = MOTOR_KP[i] * (target[i] - q) - MOTOR_KD[i] * qv
        lo, hi = model.actuator_ctrlrange[i]
        data.ctrl[i] = max(lo, min(hi, torque))


# ===== 脚触地检测（V3）=====
def _foot_contacts(model: mujoco.MjModel, data: mujoco.MjData, foot_id: int):
    """返回脚掌参与的所有接触：[(接触点世界坐标, 法向力大小 N), ...]。
    法向力用 mj_contactForce 取接触坐标系的第 0 分量（沿接触法线方向）。
    注意：mj_contactForce 的 result 必须是长度为 6 的 numpy 数组，普通 list 会报错。"""
    out = []
    for i in range(data.ncon):
        con = data.contact[i]
        if con.geom1 != foot_id and con.geom2 != foot_id:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        out.append((con.pos.copy(), abs(force[0])))
    return out


def main() -> None:
    _ensure_mjpython_on_macos()

    # 定位 XML：以本脚本所在目录为基准，找到 ../models/v1_basic.xml
    project_root = Path(__file__).resolve().parent.parent
    model_path = project_root / "models" / "v1_basic.xml"

    if not model_path.exists():
        raise FileNotFoundError(f"找不到模型文件: {model_path}")

    print(f"正在加载模型: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    _apply_start_pose(model, data)

    # --no-motor：关闭所有电机，只看被动动力学（重力 + 阻尼 + 踝弹簧）。
    # 不传该参数 → 电机开启（PD 主动控制）。两种模式方便对比弹簧的作用。
    motor_enabled = "--no-motor" not in sys.argv

    print("打开 viewer：关闭窗口即结束仿真。")
    if motor_enabled:
        print("电机已开启：PD 控制会主动把腿稳住到目标角度（能抗住重力不下垂）。")
        print("在 viewer 窗口里按键（先点一下窗口获得焦点）：")
        print("  1=髋+2°, 2=踝+2°, 3=髋-2°, 4=踝-2°（每次 2 度）")
        print("回到这个终端窗口按回车 → 切换下一个预设姿势。")
    else:
        print("电机已关闭（--no-motor）：只看被动效果——重力、关节阻尼、以及踝关节弹簧。")
    print("（不要按 viewer 里的数字键 0，在被动模式下容易黑屏。）")

    # 当前目标姿势在 MOTOR_TARGETS 列表里的索引（仅电机开启时使用）
    target_idx = 0
    # 当前目标角度（弧度），[hip, ankle]。
    # 终端回车切换预设 → 整体覆盖；viewer 窗口按键 1~4 → 对某一项微调 2°。
    current_target = list(MOTOR_TARGETS[0])
    KEY_STEP = math.radians(2.0)  # 每次按键变化 2 度

    # viewer 窗口按键回调：1=髋+2°, 2=踝+2°, 3=髋-2°, 4=踝-2°（GLFW 按键码）
    def on_key(key: int) -> None:
        if not motor_enabled:
            return
        mapping = {
            49: (0, +1),   # 键 1 → hip  +2°
            50: (1, +1),   # 键 2 → ankle +2°
            51: (0, -1),   # 键 3 → hip  -2°
            52: (1, -1),   # 键 4 → ankle -2°
        }
        if key not in mapping:
            return
        idx, sign = mapping[key]
        current_target[idx] += sign * KEY_STEP
        current_target[idx] = max(-math.pi, min(math.pi, current_target[idx]))  # 限幅 ±180°
        print(f"按键 → hip={math.degrees(current_target[0]):+.1f}°, "
              f"ankle={math.degrees(current_target[1]):+.1f}°")

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        # ===== 坐标轴显示 =====
        # 关闭 viewer 自带的世界坐标轴（它画在原点、较粗，会挡住杆子）：
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_NONE
        # 开启 3D 标签：显示所有 site 的名字。
        # 模型里只有 3 个 site（X / Y / Z，在 XML 自绘细坐标轴的末端），
        # 所以画面上正好只标出 X / Y / Z 三个轴名，不会多出别的标签。
        viewer.opt.label = mujoco.mjtLabel.mjLABEL_SITE
        # 颜色约定（XML 里自绘的细轴）：红 = X 轴，绿 = Y 轴，蓝 = Z 轴
        # 本模型方向：gravity="0 0 -9.81" → +Z 朝上、重力沿 -Z 朝下；
        #             +X 朝右，+Y 朝前（从屏幕里指向观察者）。

        # 脚掌 geom 的 id，用于识别"脚"参与的接触（V3）
        foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_geom")

        # 上次打印接触状态的时间，避免每帧刷屏（每 0.5 秒报一次）
        last_report = -1.0

        while viewer.is_running():
            step_start = time.time()

            # 终端回车 → 切换到下一个预设姿势（仅电机开启时有效）
            if motor_enabled and _enter_pressed():
                target_idx = (target_idx + 1) % len(MOTOR_TARGETS)
                current_target[:] = MOTOR_TARGETS[target_idx]  # 覆盖为该预设
                print(f"切换预设：hip={math.degrees(current_target[0]):+.0f}°, "
                      f"ankle={math.degrees(current_target[1]):+.0f}°")

            # 电机 PD 控制：步进前写好力矩（data.ctrl），mj_step 时电机就会发力。
            # 电机关闭时跳过 → ctrl 保持 0，只靠重力/阻尼/踝弹簧被动运动。
            if motor_enabled:
                _pd_control(model, data, current_target)

            mujoco.mj_step(model, data)

            # ===== 脚触地状态：终端汇报 + viewer 里画接触点（V3）=====
            contacts = _foot_contacts(model, data, foot_id)
            total_fn = sum(f for _, f in contacts)
            if data.time - last_report >= 0.5:
                last_report = data.time
                state = "着地" if contacts else "悬空"
                print(f"[触地] {state}  脚底法向力={total_fn:6.2f}N  "
                      f"hip={math.degrees(data.qpos[0]):+6.1f}°  "
                      f"ankle={math.degrees(data.qpos[1]):+6.1f}°")

            viewer.sync()

            elapsed = time.time() - step_start
            remaining = model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
