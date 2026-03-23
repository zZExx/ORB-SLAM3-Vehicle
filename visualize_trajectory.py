#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增强版轨迹可视化脚本
用于可视化SLAM系统输出的轨迹数据，包含更多分析功能
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
import os

def setup_chinese_font():
    """设置中文字体和全局字体大小"""
    try:
        # 尝试设置中文字体
        plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'WenQuanYi Micro Hei']
        plt.rcParams['axes.unicode_minus'] = False
    except:
        # 如果中文字体不可用，使用英文
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
        print("Warning: Chinese fonts not available, using English labels")
    
    # 设置全局字体大小
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['axes.labelsize'] = 12
    plt.rcParams['xtick.labelsize'] = 10
    plt.rcParams['ytick.labelsize'] = 10
    plt.rcParams['legend.fontsize'] = 10

def create_output_directories():
    """创建输出目录"""
    output_dir = "trajectory_visualization_output"
    plots_dir = os.path.join(output_dir, "plots")
    data_dir = os.path.join(output_dir, "data")
    
    os.makedirs(plots_dir, exist_ok=True)
    os.makedirs(data_dir, exist_ok=True)
    
    return output_dir, plots_dir, data_dir

def load_trajectory(filename):
    """
    加载轨迹文件
    
    Args:
        filename: 轨迹文件路径
        
    Returns:
        timestamps: 时间戳数组
        positions: 位置坐标数组 (N, 3)
        quaternions: 四元数数组 (N, 4)
    """
    if not os.path.exists(filename):
        raise FileNotFoundError(f"File {filename} not found")
    
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]  # x, y, z
    quaternions = data[:, 4:8]  # qx, qy, qz, qw
    
    return timestamps, positions, quaternions

def calculate_trajectory_stats(positions):
    """计算轨迹统计信息"""
    if len(positions) < 2:
        return {
            'length': 0,
            'avg_speed': 0,
            'max_speed': 0,
            'total_displacement': 0
        }
    
    # 计算轨迹长度
    diffs = np.diff(positions, axis=0)
    distances = np.sqrt(np.sum(diffs**2, axis=1))
    total_length = np.sum(distances)
    
    # 计算总位移（起点到终点的直线距离）
    total_displacement = np.linalg.norm(positions[-1] - positions[0])
    
    # 计算速度统计（假设时间间隔相等）
    speeds = distances  # 这里简化处理，实际应该除以时间间隔
    avg_speed = np.mean(speeds)
    max_speed = np.max(speeds)
    
    return {
        'length': total_length,
        'avg_speed': avg_speed,
        'max_speed': max_speed,
        'total_displacement': total_displacement,
        'distances': distances
    }

def plot_trajectory_2d_enhanced(positions_f, positions_kf, title="Trajectory Comparison (2D Views)"):
    """
    绘制增强版2D轨迹图
    """
    fig, axes = plt.subplots(2, 2, figsize=(16, 12)) # 设置 2行2列, 16*12英寸
    fig.suptitle(title, fontsize=18, fontweight='bold', y=1.05) # 设置整个图形的标题，字体大小为18，字体加粗，y=1.05表示标题位置往上移1.05倍的高度
    
    # 计算统计信息
    stats_f = calculate_trajectory_stats(positions_f)
    stats_kf = calculate_trajectory_stats(positions_kf)
    
    # XY平面 (俯视图)
    axes[0, 0].plot(positions_f[:, 0], positions_f[:, 1], 'b-', linewidth=1, alpha=0.7, label='Full Trajectory')
    axes[0, 0].plot(positions_kf[:, 0], positions_kf[:, 1], 'ro-', markersize=4, linewidth=2, label='Keyframes')
    # 标记起点和终点
    axes[0, 0].plot(positions_f[0, 0], positions_f[0, 1], 'gs', markersize=8, label='Start')
    axes[0, 0].plot(positions_f[-1, 0], positions_f[-1, 1], 'r^', markersize=8, label='End')
    axes[0, 0].set_xlabel('X (m)', fontsize=14)
    axes[0, 0].set_ylabel('Y (m)', fontsize=14)
    axes[0, 0].set_title('XY Plane (Top View)', fontsize=16)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(fontsize=12)
    axes[0, 0].axis('equal')
    
    # XZ平面 (侧视图)
    axes[0, 1].plot(positions_f[:, 0], positions_f[:, 2], 'b-', linewidth=1, alpha=0.7, label='Full Trajectory')
    axes[0, 1].plot(positions_kf[:, 0], positions_kf[:, 2], 'ro-', markersize=4, linewidth=2, label='Keyframes')
    axes[0, 1].plot(positions_f[0, 0], positions_f[0, 2], 'gs', markersize=8, label='Start')
    axes[0, 1].plot(positions_f[-1, 0], positions_f[-1, 2], 'r^', markersize=8, label='End')
    axes[0, 1].set_xlabel('X (m)', fontsize=14)
    axes[0, 1].set_ylabel('Z (m)', fontsize=14)
    axes[0, 1].set_title('XZ Plane (Side View)', fontsize=16)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend(fontsize=12)
    
    # YZ平面 (正视图)
    axes[1, 0].plot(positions_f[:, 1], positions_f[:, 2], 'b-', linewidth=1, alpha=0.7, label='Full Trajectory')
    axes[1, 0].plot(positions_kf[:, 1], positions_kf[:, 2], 'ro-', markersize=4, linewidth=2, label='Keyframes')
    axes[1, 0].plot(positions_f[0, 1], positions_f[0, 2], 'gs', markersize=8, label='Start')
    axes[1, 0].plot(positions_f[-1, 1], positions_f[-1, 2], 'r^', markersize=8, label='End')
    axes[1, 0].set_xlabel('Y (m)', fontsize=14)
    axes[1, 0].set_ylabel('Z (m)', fontsize=14)
    axes[1, 0].set_title('YZ Plane (Front View)', fontsize=16)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend(fontsize=12)
    
    # 统计信息
    axes[1, 1].axis('off')
    
    stats_text = f"""Trajectory Statistics:

Full Trajectory:
  - Data points: {len(positions_f)}
  - Path length: {stats_f['length']:.3f} m
  - Displacement: {stats_f['total_displacement']:.3f} m
  - Avg step size: {stats_f['avg_speed']:.6f} m
  - Max step size: {stats_f['max_speed']:.6f} m
  - X range: [{positions_f[:, 0].min():.3f}, {positions_f[:, 0].max():.3f}] m
  - Y range: [{positions_f[:, 1].min():.3f}, {positions_f[:, 1].max():.3f}] m
  - Z range: [{positions_f[:, 2].min():.3f}, {positions_f[:, 2].max():.3f}] m

Keyframe Trajectory:
  - Data points: {len(positions_kf)}
  - Path length: {stats_kf['length']:.3f} m
  - Displacement: {stats_kf['total_displacement']:.3f} m
  - Avg step size: {stats_kf['avg_speed']:.6f} m
  - Max step size: {stats_kf['max_speed']:.6f} m
  - X range: [{positions_kf[:, 0].min():.3f}, {positions_kf[:, 0].max():.3f}] m
  - Y range: [{positions_kf[:, 1].min():.3f}, {positions_kf[:, 1].max():.3f}] m
  - Z range: [{positions_kf[:, 2].min():.3f}, {positions_kf[:, 2].max():.3f}] m"""
    
    axes[1, 1].text(0.05, 0.95, stats_text, transform=axes[1, 1].transAxes, 
                    fontsize=12, verticalalignment='top', fontfamily='monospace',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))
    
    plt.tight_layout()
    return fig

def plot_trajectory_3d_enhanced(positions_f, positions_kf, title="3D Trajectory Comparison"):
    """
    绘制增强版3D轨迹图
    """
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(title, fontsize=18, fontweight='bold', y=1.05)
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制完整轨迹，使用颜色映射显示时间进展
    n_points = len(positions_f)
    colors = plt.cm.viridis(np.linspace(0, 1, n_points))
    
    # 绘制轨迹线
    ax.plot(positions_f[:, 0], positions_f[:, 1], positions_f[:, 2], 
            'b-', linewidth=1, alpha=0.7, label='Full Trajectory')
    
    # 绘制关键帧轨迹
    ax.plot(positions_kf[:, 0], positions_kf[:, 1], positions_kf[:, 2], 
            'ro-', markersize=5, linewidth=2, label='Keyframes')
    
    # 标记起点和终点
    ax.scatter(positions_f[0, 0], positions_f[0, 1], positions_f[0, 2], 
               c='green', s=150, marker='s', label='Start', edgecolors='black')
    ax.scatter(positions_f[-1, 0], positions_f[-1, 1], positions_f[-1, 2], 
               c='red', s=150, marker='^', label='End', edgecolors='black')
    
    # 添加时间进展的颜色映射点
    scatter = ax.scatter(positions_f[::10, 0], positions_f[::10, 1], positions_f[::10, 2], 
                        c=np.arange(0, len(positions_f), 10), cmap='viridis', 
                        s=20, alpha=0.6, label='Time progression')
    
    ax.set_xlabel('X (m)', fontsize=14)
    ax.set_ylabel('Y (m)', fontsize=14)
    ax.set_zlabel('Z (m)', fontsize=14)
    ax.set_title(title, fontsize=16, fontweight='bold')
    ax.legend(fontsize=12)
    
    # 添加颜色条
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.5, aspect=20)
    cbar.set_label('Time progression', fontsize=12)
    
    # 设置相等的坐标轴比例
    max_range = np.array([positions_f[:, 0].max()-positions_f[:, 0].min(),
                         positions_f[:, 1].max()-positions_f[:, 1].min(),
                         positions_f[:, 2].max()-positions_f[:, 2].min()]).max() / 2.0
    
    mid_x = (positions_f[:, 0].max()+positions_f[:, 0].min()) * 0.5
    mid_y = (positions_f[:, 1].max()+positions_f[:, 1].min()) * 0.5
    mid_z = (positions_f[:, 2].max()+positions_f[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    return fig

def plot_motion_analysis(timestamps_f, positions_f, timestamps_kf, positions_kf):
    """
    绘制运动分析图表
    """
    # 将时间戳转换为相对时间（秒）
    time_f = (timestamps_f - timestamps_f[0]) / 1e9  # 纳秒转秒
    time_kf = (timestamps_kf - timestamps_f[0]) / 1e9
    
    # 计算速度和加速度
    stats_f = calculate_trajectory_stats(positions_f)
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Motion Analysis', fontsize=18, fontweight='bold', y=1.05)
    
    # 坐标随时间变化
    coordinates = ['X', 'Y', 'Z']
    colors = ['red', 'green', 'blue']
    
    for i in range(3):
        axes[0, 0].plot(time_f, positions_f[:, i], color=colors[i], linewidth=1, 
                       alpha=0.7, label=f'{coordinates[i]} - Full')
        axes[0, 0].plot(time_kf, positions_kf[:, i], 'o', color=colors[i], 
                       markersize=3, label=f'{coordinates[i]} - KF')
    
    axes[0, 0].set_xlabel('Time (s)', fontsize=14)
    axes[0, 0].set_ylabel('Position (m)', fontsize=14)
    axes[0, 0].set_title('Position vs Time', fontsize=16)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(fontsize=12)
    
    # 步长分析
    if len(stats_f['distances']) > 0:
        axes[0, 1].plot(stats_f['distances'], 'b-', linewidth=1, alpha=0.7)
        axes[0, 1].set_xlabel('Step Number', fontsize=14)
        axes[0, 1].set_ylabel('Step Size (m)', fontsize=14)
        axes[0, 1].set_title('Step Size Analysis', fontsize=16)
        axes[0, 1].grid(True, alpha=0.3)
        
        # 添加统计线
        axes[0, 1].axhline(y=np.mean(stats_f['distances']), color='r', linestyle='--', 
                          label=f'Mean: {np.mean(stats_f["distances"]):.6f}')
        axes[0, 1].axhline(y=np.median(stats_f['distances']), color='g', linestyle='--', 
                          label=f'Median: {np.median(stats_f["distances"]):.6f}')
        axes[0, 1].legend(fontsize=12)
    
    # 轨迹密度热图 (XY平面)
    try:
        axes[1, 0].hist2d(positions_f[:, 0], positions_f[:, 1], bins=30, cmap='Blues', alpha=0.7)
        axes[1, 0].plot(positions_f[:, 0], positions_f[:, 1], 'r-', linewidth=1, alpha=0.5)
        axes[1, 0].set_xlabel('X (m)', fontsize=14)
        axes[1, 0].set_ylabel('Y (m)', fontsize=14)
        axes[1, 0].set_title('Trajectory Density (XY Plane)', fontsize=16)
    except:
        axes[1, 0].plot(positions_f[:, 0], positions_f[:, 1], 'b-', linewidth=1)
        axes[1, 0].set_xlabel('X (m)', fontsize=14)
        axes[1, 0].set_ylabel('Y (m)', fontsize=14)
        axes[1, 0].set_title('Trajectory (XY Plane)', fontsize=16)
    
    # 高度变化分析
    axes[1, 1].plot(time_f, positions_f[:, 2], 'b-', linewidth=1, alpha=0.7, label='Full trajectory')
    axes[1, 1].plot(time_kf, positions_kf[:, 2], 'ro', markersize=4, label='Keyframes')
    axes[1, 1].set_xlabel('Time (s)', fontsize=14)
    axes[1, 1].set_ylabel('Z (m)', fontsize=14)
    axes[1, 1].set_title('Height vs Time', fontsize=16)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(fontsize=12)
    
    plt.tight_layout()
    return fig

def main():
    """主函数"""
    # 设置字体
    setup_chinese_font()
    
    # 创建输出目录
    output_dir, plots_dir, data_dir = create_output_directories()
    
    # 文件路径
    f_file = "/mnt/ssd/dataset/V1_01_easy/KeyFrameTrajectory.txt"
    kf_file = "/mnt/ssd/dataset/V1_01_easy/KeyFrameTrajectory.txt"
    
    try:
        # 加载数据
        print("Loading trajectory data...")
        timestamps_f, positions_f, quaternions_f = load_trajectory(f_file)
        timestamps_kf, positions_kf, quaternions_kf = load_trajectory(kf_file)
        
        print(f"Full trajectory data points: {len(positions_f)}")
        print(f"Keyframe data points: {len(positions_kf)}")
        
        # 复制轨迹文件到数据目录
        import shutil
        shutil.copy2(f_file, os.path.join(data_dir, f_file))
        shutil.copy2(kf_file, os.path.join(data_dir, kf_file))
        print(f"Trajectory files copied to: {data_dir}")
        
        # 创建可视化图表
        print("Generating enhanced 2D trajectory plots...")
        fig1 = plot_trajectory_2d_enhanced(positions_f, positions_kf)
        
        print("Generating enhanced 3D trajectory plot...")
        fig2 = plot_trajectory_3d_enhanced(positions_f, positions_kf)
        
        print("Generating motion analysis plots...")
        fig3 = plot_motion_analysis(timestamps_f, positions_f, timestamps_kf, positions_kf)
        
        # 保存图片到plots目录
        print("Saving plots...")
        fig1.savefig(os.path.join(plots_dir, 'trajectory_2d_enhanced.png'), dpi=800, bbox_inches='tight')
        fig2.savefig(os.path.join(plots_dir, 'trajectory_3d_enhanced.png'), dpi=800, bbox_inches='tight')
        fig3.savefig(os.path.join(plots_dir, 'motion_analysis.png'), dpi=800, bbox_inches='tight')
        
        print("Plots saved to:")
        print(f"- {plots_dir}/trajectory_2d.png:  2D trajectory comparison")
        print(f"- {plots_dir}/trajectory_3d.png:  3D trajectory with time progression")
        print(f"- {plots_dir}/motion_analysis.png: Motion analysis plots")
        
        print(f"\nAll outputs saved in: {output_dir}/")
        
        # 显示图表
        plt.show()
        
    except FileNotFoundError as e:
        print(f"Error: File not found - {e}")
        print("Please ensure the following files exist in the current directory:")
        print(f"- {f_file}")
        print(f"- {kf_file}")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 
