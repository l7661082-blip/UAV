#!/usr/bin/env python3
"""
Visual Servo Debug Data Analyzer
分析视觉伺服的 CSV 日志数据，生成可视化图表和诊断报告
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from pathlib import Path

class VisualServoAnalyzer:
    def __init__(self, csv_path):
        self.csv_path = csv_path
        self.df = pd.read_csv(csv_path)
        self.output_dir = Path(csv_path).parent
        self.basename = Path(csv_path).stem
        
        # 状态映射
        self.state_names = {0: 'ALIGNING', 1: 'APPROACHING', 2: 'SPRINTING'}
        
    def analyze(self):
        """执行完整分析"""
        print(f"分析文件: {self.csv_path}")
        print(f"数据行数: {len(self.df)}")
        print(f"时间范围: {self.df['timestamp'].iloc[0]:.2f} - {self.df['timestamp'].iloc[-1]:.2f} 秒")
        print()
        
        # 生成图表
        self.plot_errors()
        self.plot_velocities()
        self.plot_area_ratio()
        self.plot_pd_control()
        
        # 统计分析
        self.print_statistics()
        
        # 失败诊断
        self.diagnose_failures()
        
        print(f"\n图表保存到: {self.output_dir}")
        
    def plot_errors(self):
        """绘制误差曲线"""
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # X方向误差
        axes[0].plot(self.df['timestamp'], self.df['err_x'], label='err_x (raw)', alpha=0.6)
        axes[0].plot(self.df['timestamp'], self.df['err_x_dz'], label='err_x (deadzone)', linewidth=2)
        axes[0].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[0].set_ylabel('X Error (pixels)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('Position Errors Over Time')
        
        # Y方向误差
        axes[1].plot(self.df['timestamp'], self.df['err_y'], label='err_y (raw)', alpha=0.6)
        axes[1].plot(self.df['timestamp'], self.df['err_y_dz'], label='err_y (deadzone)', linewidth=2)
        axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[1].set_ylabel('Y Error (pixels)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        # 状态
        axes[2].plot(self.df['timestamp'], self.df['state'], drawstyle='steps-post')
        axes[2].set_ylabel('State')
        axes[2].set_yticks([0, 1, 2])
        axes[2].set_yticklabels(['ALIGNING', 'APPROACHING', 'SPRINTING'])
        axes[2].grid(True, alpha=0.3)
        axes[2].set_xlabel('Time (s)')
        
        plt.tight_layout()
        plt.savefig(self.output_dir / f'{self.basename}_errors.png', dpi=150)
        plt.close()
        
    def plot_velocities(self):
        """绘制速度曲线"""
        fig, axes = plt.subplots(4, 1, figsize=(12, 12))
        
        axes[0].plot(self.df['timestamp'], self.df['v_body_x'], label='vx (forward)')
        axes[0].set_ylabel('Vx (m/s)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('Body Velocities Over Time')
        
        axes[1].plot(self.df['timestamp'], self.df['v_body_y'], label='vy (lateral)')
        axes[1].set_ylabel('Vy (m/s)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        axes[2].plot(self.df['timestamp'], self.df['v_body_z'], label='vz (vertical)')
        axes[2].set_ylabel('Vz (m/s)')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)
        
        axes[3].plot(self.df['timestamp'], self.df['velocity_mag'], label='velocity magnitude')
        axes[3].axhline(0.15, color='r', linestyle='--', label='stability threshold')
        axes[3].set_ylabel('Velocity (m/s)')
        axes[3].set_xlabel('Time (s)')
        axes[3].legend()
        axes[3].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / f'{self.basename}_velocities.png', dpi=150)
        plt.close()
        
    def plot_area_ratio(self):
        """绘制面积比例和冲刺准备"""
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        axes[0].plot(self.df['timestamp'], self.df['area_ratio'], label='current ratio')
        axes[0].plot(self.df['timestamp'], self.df['max_area_ratio'], label='max ratio', linestyle='--')
        axes[0].axhline(0.2, color='r', linestyle='--', alpha=0.5, label='threshold')
        axes[0].set_ylabel('Area Ratio')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('Target Area Ratio and Sprint Readiness')
        
        axes[1].plot(self.df['timestamp'], self.df['sprint_ready_frames'])
        axes[1].axhline(15, color='r', linestyle='--', label='trigger threshold')
        axes[1].set_ylabel('Sprint Ready Frames')
        axes[1].set_xlabel('Time (s)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / f'{self.basename}_area.png', dpi=150)
        plt.close()
        
    def plot_pd_control(self):
        """绘制PD控制项"""
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # X方向
        axes[0].plot(self.df['timestamp'], self.df['p_term_x'], label='P term', alpha=0.7)
        axes[0].plot(self.df['timestamp'], self.df['d_term_x'], label='D term', alpha=0.7)
        axes[0].set_ylabel('X Control Terms')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('PD Controller Terms')
        
        # Y方向
        axes[1].plot(self.df['timestamp'], self.df['p_term_y'], label='P term', alpha=0.7)
        axes[1].plot(self.df['timestamp'], self.df['d_term_y'], label='D term', alpha=0.7)
        axes[1].set_ylabel('Y Control Terms')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        # Yaw
        axes[2].plot(self.df['timestamp'], self.df['p_term_yaw'], label='P term', alpha=0.7)
        axes[2].plot(self.df['timestamp'], self.df['d_term_yaw'], label='D term', alpha=0.7)
        axes[2].set_ylabel('Yaw Control Terms')
        axes[2].set_xlabel('Time (s)')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / f'{self.basename}_pd_control.png', dpi=150)
        plt.close()
        
    def print_statistics(self):
        """打印统计信息"""
        print("=" * 60)
        print("统计分析")
        print("=" * 60)
        
        for state_id, state_name in self.state_names.items():
            state_data = self.df[self.df['state'] == state_id]
            if len(state_data) == 0:
                continue
                
            print(f"\n{state_name} 阶段:")
            print(f"  持续时间: {len(state_data) * 0.05:.2f}s ({len(state_data)} 帧)")
            print(f"  平均误差: X={state_data['err_x'].mean():.1f} Y={state_data['err_y'].mean():.1f} px")
            print(f"  误差标准差: X={state_data['err_x'].std():.1f} Y={state_data['err_y'].std():.1f} px")
            print(f"  平均速度: vx={state_data['v_body_x'].mean():.2f} vy={state_data['v_body_y'].mean():.2f} m/s")
            
            if state_id == 1:  # APPROACHING
                print(f"  速度缩放: 平均={state_data['speed_scale'].mean():.2f} 最小={state_data['speed_scale'].min():.2f}")
                print(f"  最大冲刺帧数: {state_data['sprint_ready_frames'].max()}")
                
        print(f"\n最大面积比例: {self.df['max_area_ratio'].max():.3f}")
        print(f"控制频率: {1.0 / self.df['dt'].mean():.1f} Hz")
        
    def diagnose_failures(self):
        """诊断失败模式"""
        print("\n" + "=" * 60)
        print("失败诊断")
        print("=" * 60)
        
        issues = []
        
        # 检查对齐阶段振荡
        aligning_data = self.df[self.df['state'] == 0]
        if len(aligning_data) > 10:
            err_x_signs = np.sign(aligning_data['err_x'].values)
            sign_changes = np.sum(np.diff(err_x_signs) != 0)
            osc_freq = sign_changes / (len(aligning_data) * 0.05)
            if osc_freq > 3:
                issues.append(f"⚠ ALIGNING阶段振荡: {osc_freq:.1f} 次/秒 (建议增加D增益或死区)")
                
        # 检查接近阶段偏离
        approaching_data = self.df[self.df['state'] == 1]
        if len(approaching_data) > 10:
            err_growth = approaching_data['err_x'].abs().diff().mean()
            if err_growth > 0.5:
                issues.append(f"⚠ APPROACHING阶段误差增长: {err_growth:.2f} px/帧 (检查P增益)")
                
            # 检查速度过小
            avg_vx = approaching_data['v_body_x'].mean()
            if avg_vx < 0.1:
                issues.append(f"⚠ APPROACHING速度过慢: {avg_vx:.2f} m/s (考虑增加forward_velocity)")
                
        # 检查冲刺触发
        max_sprint_frames = self.df['sprint_ready_frames'].max()
        sprinted = (self.df['state'] == 2).any()
        
        if max_sprint_frames >= 10 and not sprinted:
            issues.append(f"⚠ 冲刺条件差点满足但未触发 (最大{max_sprint_frames}/15帧)")
            
        max_ratio = self.df['max_area_ratio'].max()
        if max_ratio < 0.15:
            issues.append(f"⚠ 未充分接近目标: 最大面积比例={max_ratio:.3f} (目标>0.2)")
            
        # 输出诊断结果
        if issues:
            print("\n发现以下问题:")
            for issue in issues:
                print(f"  {issue}")
        else:
            print("\n✓ 未发现明显问题")
            
        # 参数建议
        print("\n参数调优建议:")
        if len(aligning_data) > 0:
            avg_err_x = aligning_data['err_x'].abs().mean()
            if avg_err_x > 30:
                print("  - 增加 k_p_yaw (当前对齐响应较慢)")
        if len(approaching_data) > 0:
            avg_lateral_err = approaching_data['err_y'].abs().mean()
            if avg_lateral_err > 30:
                print("  - 增加 k_p_y (纵向修正不足)")

def main():
    if len(sys.argv) < 2:
        print("用法: python3 analyze_visual_servo.py <csv_file>")
        print("示例: python3 analyze_visual_servo.py ~/28com_ws/visual_servo_logs/servo_debug_20260123_093500.csv")
        sys.exit(1)
        
    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f"错误: 文件不存在: {csv_path}")
        sys.exit(1)
        
    analyzer = VisualServoAnalyzer(csv_path)
    analyzer.analyze()
    
    print("\n分析完成!")

if __name__ == "__main__":
    main()
