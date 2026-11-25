# -*- coding: utf-8 -*-
"""
生成关节执行机构转速—力矩线性外特性包络与功率曲线
"""
import os
import numpy as np
import matplotlib.pyplot as plt

os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib'
os.makedirs('doc/spec/assets', exist_ok=True)

plt.rcParams['font.sans-serif'] = ['PingFang SC', 'Arial Unicode MS', 'Heiti TC', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

# 归一化自变量
w_norm = np.linspace(0, 1.15, 600)
tau_norm = np.maximum(0, 1.0 - w_norm)
tau_norm[w_norm > 1.0] = 0.0

p_norm = 4.0 * w_norm * (1.0 - w_norm)
p_norm[w_norm > 1.0] = 0.0

fig, ax1 = plt.subplots(figsize=(9.2, 5.8), dpi=300)

# 区域填充
ax1.fill_between(w_norm[w_norm <= 1.0], 0, tau_norm[w_norm <= 1.0], color='#0284c7', alpha=0.15, label='电机可行运行区 (Operating Region)')
ax1.fill_between(w_norm, tau_norm, 1.25, color='#ef4444', alpha=0.08, label='不可行/饱和超限区 (Infeasible / Saturated)')

# 力矩包络线
line_tau, = ax1.plot(w_norm[w_norm <= 1.0], tau_norm[w_norm <= 1.0], color='#0369a1', lw=3.0, label=r'力矩极限包络 $|\tau_j| \leq \tau_{\mathrm{max}}(1 - |\dot{q}_j|/\omega_{\mathrm{max}})$')
ax1.plot([1.0, 1.15], [0, 0], color='#0369a1', lw=2.5, ls='--')

# 次坐标轴：输出功率
ax2 = ax1.twinx()
line_p, = ax2.plot(w_norm[w_norm <= 1.0], p_norm[w_norm <= 1.0], color='#d97706', lw=2.4, ls='-.', label=r'瞬时输出机械功率 $P_j = \tau_j \dot{q}_j$ (峰值在 $0.5\omega_{\mathrm{max}}$)')

# 关键特征点标注
ax1.scatter([0], [1.0], color='#0369a1', s=80, zorder=5)
ax1.annotate(r'堵转点 $(0, \tau_{\mathrm{max}})$' + '\n' + r'$\tau_{\mathrm{max}} = n_{\mathrm{gear}} \eta_{\mathrm{trans}} \tau_{\mathrm{stall}}$',
             xy=(0, 1.0), xytext=(0.04, 1.08),
             arrowprops=dict(arrowstyle='->', color='#0369a1', lw=1.5),
             fontsize=10, fontweight='bold', color='#0c4a6e',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#e0f2fe', edgecolor='#38bdf8', alpha=0.9))

ax1.scatter([1.0], [0.0], color='#0369a1', s=80, zorder=5)
ax1.annotate(r'空载极速点 $(\omega_{\mathrm{max}}, 0)$' + '\n' + r'$\omega_{\mathrm{max}} = \omega_{\mathrm{motor},\mathrm{max}} / n_{\mathrm{gear}}$',
             xy=(1.0, 0.0), xytext=(0.70, 0.22),
             arrowprops=dict(arrowstyle='->', color='#0369a1', lw=1.5),
             fontsize=10, fontweight='bold', color='#0c4a6e',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#e0f2fe', edgecolor='#38bdf8', alpha=0.9))

ax2.scatter([0.5], [1.0], color='#d97706', s=80, zorder=5)
ax2.annotate(r'最大输出功率点 $(0.5, P_{\mathrm{max}})$' + '\n' + r'$P_{\mathrm{max}} = \frac{1}{4} \tau_{\mathrm{max}} \omega_{\mathrm{max}}$',
             xy=(0.5, 1.0), xytext=(0.38, 1.10),
             arrowprops=dict(arrowstyle='->', color='#d97706', lw=1.5),
             fontsize=10, fontweight='bold', color='#78350f',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#fef3c7', edgecolor='#f59e0b', alpha=0.9))

# 起跳阶段物理状态批注
ax1.annotate('【起跳初期：深蹲起蹬】\n低转速、大载荷\n电机为主输出大推力加速',
             xy=(0.15, 0.85), xytext=(0.18, 0.65),
             arrowprops=dict(arrowstyle='->', color='#047857', lw=1.3),
             fontsize=9.5, color='#065f46',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#ecfdf5', edgecolor='#34d399', alpha=0.95))

ax1.annotate('【起跳末期：爆发蹬离】\n关节角速度急剧攀升\n电机电磁力矩断崖式衰减\n由踝关节并联单向弹簧补充爆发',
             xy=(0.85, 0.15), xytext=(0.50, 0.45),
             arrowprops=dict(arrowstyle='->', color='#b91c1c', lw=1.3),
             fontsize=9.5, color='#991b1b',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#fef2f2', edgecolor='#f87171', alpha=0.95))

# 坐标轴设置
ax1.set_xlim(-0.03, 1.16)
ax1.set_ylim(-0.06, 1.26)
ax2.set_ylim(-0.06, 1.26)

ax1.set_xlabel(r'归一化关节角速度 $|\dot{q}_j| / \omega_{\mathrm{max}}$', fontsize=11, fontweight='bold')
ax1.set_ylabel(r'归一化关节输出力矩 $|\tau_j| / \tau_{\mathrm{max}}$', fontsize=11, fontweight='bold', color='#0369a1')
ax2.set_ylabel(r'归一化瞬时输出功率 $P_j / P_{\mathrm{max}}$', fontsize=11, fontweight='bold', color='#d97706')

ax1.tick_params(axis='y', labelcolor='#0369a1')
ax2.tick_params(axis='y', labelcolor='#d97706')

ax1.grid(True, linestyle=':', alpha=0.6)
ax1.set_title('图 2.1 关节执行机构转速—力矩线性外特性包络与功率曲线', fontsize=12.5, fontweight='bold', pad=15)

# 合并图例
lines = [line_tau, line_p]
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, loc='upper right', framealpha=0.95, fontsize=9.2)

plt.tight_layout()
output_path = 'doc/spec/assets/torque_speed_envelope.png'
plt.savefig(output_path, dpi=300)
plt.close()
print(f'Saved {output_path} successfully!')
