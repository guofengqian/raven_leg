"""
加载 V1 基础腿模型，在 MuJoCo viewer 中观看重力下的自然摆动。

用法（在项目根目录执行）:
    python3 scripts/simulate.py
"""

import os
import select
import shutil
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer


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


def _almost_stopped(data: mujoco.MjData, speed_eps: float = 0.05) -> bool:
    """关节角速度都很小 → 认为已经摆停。"""
    return float(max(abs(v) for v in data.qvel)) < speed_eps


def _enter_pressed() -> bool:
    """非阻塞检测：终端里是否按了回车。"""
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


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

    print("打开 viewer：关闭窗口即结束仿真。")
    print("摆停后会自动再摆一次。")
    print("想立刻重摆：回到这个终端窗口，按一下回车。")
    print("（不要按 viewer 里的数字键 0，在被动模式下容易黑屏。）")

    # 连续静止多久后自动重摆（秒）
    still_need = 0.8
    still_time = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # 终端回车 → 立刻重摆
            if _enter_pressed():
                with viewer.lock():
                    _apply_start_pose(model, data)
                still_time = 0.0
                print("已重置，重新摆动。")

            mujoco.mj_step(model, data)

            # 摆停一小段时间 → 自动重摆（避免一直挂着不动）
            if _almost_stopped(data):
                still_time += model.opt.timestep
                if still_time >= still_need:
                    with viewer.lock():
                        _apply_start_pose(model, data)
                    still_time = 0.0
                    print("检测到已停住，自动重新摆动。")
            else:
                still_time = 0.0

            viewer.sync()

            elapsed = time.time() - step_start
            remaining = model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
