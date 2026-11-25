# -*- coding: utf-8 -*-
"""
生成踝关节单向储能扭簧与脚趾双向顺应扭簧特性曲线图
"""
import os
import numpy as np
import matplotlib.pyplot as plt

os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib'
os.makedirs('doc/spec/assets', exist_ok=True)

plt.rcParams['font.sans-serif'] = ['PingFang SC', 'Arial Unicode MS', 'Heiti TC', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

# =============================================================================
# 1. 图 2.2 踝关节单向储能扭簧特性曲线 (ankle_spring_profile.png)
# =============================================================================
q5_0_deg = 145.0
k_a_deg = 3.207 * 1e-3  # N·m/deg

q5_deg = np.linspace(95, 175, 500)
tau_a = np.where(q5_deg < q5_0_deg, k_a_deg * (q5_0_deg - q5_deg), 0.0)
V_a = np.where(q5_deg < q5_0_deg, 0.5 * k_a_deg * (np.radians(q5_0_deg - q5_deg))**2 * (180.0 / np.pi), 0.0)
# 势能精确 SI 单位: 0.5 * k_a_rad * (delta_q_rad)^2
k_a_rad = k_a_deg * 180.0 / np.pi
V_a_J = np.where(q5_deg < q5_0_deg, 0.5 * k_a_rad * (np.radians(q5_0_deg - q5_deg))**2, 0.0)

fig, ax1 = plt.subplots(figsize=(8.8, 5.2), dpi=300)

# 区域填充
ax1.axvspan(95, q5_0_deg, color='#0284c7', alpha=0.12, label='屈曲深蹲压缩区 (机体下压蓄能)')
ax1.axvspan(q5_0_deg, 175, color='#64748b', alpha=0.10, label='伸展脱嵌自由区 (机械脱开无阻力)')

# 力矩曲线（左 Y 轴）
line_tau, = ax1.plot(q5_deg, tau_a, color='#0369a1', lw=3.0, label=r'单向恢复力矩 $\tau_a(q_5)$')

# 势能曲线（右 Y 轴）
ax2 = ax1.twinx()
line_v, = ax2.plot(q5_deg, V_a_J, color='#ea580c', lw=2.5, ls='--', label=r'弹性势能 $V_a(q_5)$')

# 关键状态点标注
ax1.axvline(q5_0_deg, color='#0f172a', ls=':', lw=1.5)
ax1.scatter([q5_0_deg], [0], color='#0369a1', s=70, zorder=5)
ax1.annotate(f'单向脱嵌临界点 $q_{{5,0}} = {int(q5_0_deg)}^\\circ$\n力矩与势能降为零',
             xy=(q5_0_deg, 0), xytext=(q5_0_deg + 3.0, 0.06),
             arrowprops=dict(arrowstyle='->', color='#0f172a', lw=1.3),
             fontsize=10, fontweight='bold', color='#0f172a',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#f8fafc', edgecolor='#94a3b8', alpha=0.95))

# 物理过程指引
ax1.annotate('【起跳初期：深蹲压紧】\n关节角度减小 (屈曲)\n弹性势能抛物线上升蓄力',
             xy=(115, 0.096), xytext=(98, 0.12),
             arrowprops=dict(arrowstyle='->', color='#0284c7', lw=1.3),
             fontsize=9.5, color='#075985',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#e0f2fe', edgecolor='#38bdf8', alpha=0.95))

ax1.annotate('【起跳末期：爆发蹬伸】\n向自由角伸展，势能瞬间释放\n补偿高速下电机力矩衰减 (削峰)',
             xy=(135, 0.032), xytext=(122, 0.075),
             arrowprops=dict(arrowstyle='->', color='#ea580c', lw=1.3),
             fontsize=9.5, color='#9a3412',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#fff7ed', edgecolor='#fb923c', alpha=0.95))

# 坐标轴设置
ax1.set_xlim(95, 175)
ax1.set_ylim(-0.01, 0.18)
ax2.set_ylim(-0.005, 0.09)

ax1.set_xlabel(r'踝关节广义坐标角度 $q_5\ (^\circ)$', fontsize=11, fontweight='bold')
ax1.set_ylabel(r'踝关节扭簧力矩 $\tau_a\ (\mathrm{N\cdot m})$', fontsize=11, fontweight='bold', color='#0369a1')
ax2.set_ylabel(r'踝关节弹性势能 $V_a\ (\mathrm{J})$', fontsize=11, fontweight='bold', color='#ea580c')

ax1.tick_params(axis='y', labelcolor='#0369a1')
ax2.tick_params(axis='y', labelcolor='#ea580c')
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.set_title('图 2.2 踝关节单向储能扭簧分段力矩与势能特性曲线', fontsize=12.5, fontweight='bold', pad=15)

lines = [line_tau, line_v]
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, loc='upper right', framealpha=0.95, fontsize=9.2)

plt.tight_layout()
plt.savefig('doc/spec/assets/ankle_spring_profile.png', dpi=300)
plt.close()
print('Ankle spring profile saved!')

# =============================================================================
# 2. 图 2.3 脚趾双向顺应扭簧特性曲线 (toe_spring_profile.png)
# =============================================================================
q6_0_deg = 25.0
k_t_deg = 6.249 * 1e-3  # N·m/deg
k_t_rad = k_t_deg * 180.0 / np.pi

q6_deg = np.linspace(-15, 65, 500)
delta_q_deg = q6_deg - q6_0_deg
tau_t = -k_t_deg * delta_q_deg
V_t_J = 0.5 * k_t_rad * (np.radians(delta_q_deg))**2

fig, ax1 = plt.subplots(figsize=(8.8, 5.2), dpi=300)

# 对称零点参考
ax1.axvline(q6_0_deg, color='#0f172a', ls=':', lw=1.5)
ax1.axhline(0, color='#64748b', ls='--', lw=1.0)

# 区域填充
ax1.axvspan(-15, q6_0_deg, color='#059669', alpha=0.10, label='受压偏转区 (地表支撑反力加载)')
ax1.axvspan(q6_0_deg, 65, color='#7c3aed', alpha=0.08, label='反向复位区 (离地腾空自由伸展)')

# 力矩曲线（左 Y 轴）
line_tau_t, = ax1.plot(q6_deg, tau_t, color='#047857', lw=3.0, label=r'双向恢复力矩 $\tau_t(q_6) = -k_t(q_6 - q_{6,0})$')

# 势能曲线（右 Y 轴）
ax2 = ax1.twinx()
line_v_t, = ax2.plot(q6_deg, V_t_J, color='#7c3aed', lw=2.5, ls='--', label=r'弹性势能 $V_t(q_6) = \frac{1}{2}k_t(q_6 - q_{6,0})^2$')

# 关键状态点标注
ax1.scatter([q6_0_deg], [0], color='#047857', s=70, zorder=5)
ax1.annotate(f'自然平衡自由角 $q_{{6,0}} = {int(q6_0_deg)}^\\circ$\n力矩为零，势能极小',
             xy=(q6_0_deg, 0), xytext=(q6_0_deg + 3.0, 0.05),
             arrowprops=dict(arrowstyle='->', color='#0f172a', lw=1.3),
             fontsize=10, fontweight='bold', color='#0f172a',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#f8fafc', edgecolor='#94a3b8', alpha=0.95))

# 物理功能指引
ax1.annotate('【触地加载：顺应缓冲】\n地面反力迫使脚趾偏折\n产生正向弹力恢复力矩并蓄能',
             xy=(5, 0.125), xytext=(-12, 0.18),
             arrowprops=dict(arrowstyle='->', color='#047857', lw=1.3),
             fontsize=9.5, color='#065f46',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#ecfdf5', edgecolor='#34d399', alpha=0.95))

ax1.annotate('【离地腾空：自动复位】\n外力撤销后依靠弹力回弹\n无脱开断点，连续对称U型势能',
             xy=(45, -0.125), xytext=(32, -0.22),
             arrowprops=dict(arrowstyle='->', color='#7c3aed', lw=1.3),
             fontsize=9.5, color='#5b21b6',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#f5f3ff', edgecolor='#a78bfa', alpha=0.95))

# 坐标轴设置
ax1.set_xlim(-15, 65)
ax1.set_ylim(-0.28, 0.28)
ax2.set_ylim(-0.005, 0.09)

ax1.set_xlabel(r'脚趾广义坐标角度 $q_6\ (^\circ)$', fontsize=11, fontweight='bold')
ax1.set_ylabel(r'脚趾扭簧恢复力矩 $\tau_t\ (\mathrm{N\cdot m})$', fontsize=11, fontweight='bold', color='#047857')
ax2.set_ylabel(r'脚趾弹性势能 $V_t\ (\mathrm{J})$', fontsize=11, fontweight='bold', color='#7c3aed')

ax1.tick_params(axis='y', labelcolor='#047857')
ax2.tick_params(axis='y', labelcolor='#7c3aed')
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.set_title('图 2.3 脚趾双向顺应扭簧对称力矩与势能特性曲线', fontsize=12.5, fontweight='bold', pad=15)

lines = [line_tau_t, line_v_t]
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, loc='upper right', framealpha=0.95, fontsize=9.2)

plt.tight_layout()
plt.savefig('doc/spec/assets/toe_spring_profile.png', dpi=300)
plt.close()
print('Toe spring profile saved!')
