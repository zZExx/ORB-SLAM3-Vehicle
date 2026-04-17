#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹数据信息查看器
快速查看SLAM轨迹数据的基本统计信息
"""

import numpy as np
import os

def load_and_analyze_trajectory(filename):
    """加载并分析轨迹文件"""
    if not os.path.exists(filename):
        print(f"错误: 文件 {filename} 不存在")
        return None
    
    try:
        data = np.loadtxt(filename)
        timestamps = data[:, 0]
        positions = data[:, 1:4]  # x, y, z
        quaternions = data[:, 4:8]  # qx, qy, qz, qw
        
        # 计算基本统计信息
        n_points = len(positions)
        
        # 时间信息
        duration = (timestamps[-1] - timestamps[0]) / 1e9  # 转换为秒
        avg_freq = n_points / duration if duration > 0 else 0
        
        # 位置统计
        pos_min = positions.min(axis=0)
        pos_max = positions.max(axis=0)
        pos_range = pos_max - pos_min
        pos_mean = positions.mean(axis=0)
        pos_std = positions.std(axis=0)
        
        # 轨迹长度
        if n_points > 1:
            diffs = np.diff(positions, axis=0)
            distances = np.sqrt(np.sum(diffs**2, axis=1))
            total_length = np.sum(distances)
            avg_step = np.mean(distances)
            max_step = np.max(distances)
            min_step = np.min(distances)
        else:
            total_length = avg_step = max_step = min_step = 0
        
        # 总位移
        total_displacement = np.linalg.norm(positions[-1] - positions[0]) if n_points > 1 else 0
        
        return {
            'filename': filename,
            'n_points': n_points,
            'duration': duration,
            'avg_freq': avg_freq,
            'pos_min': pos_min,
            'pos_max': pos_max,
            'pos_range': pos_range,
            'pos_mean': pos_mean,
            'pos_std': pos_std,
            'total_length': total_length,
            'total_displacement': total_displacement,
            'avg_step': avg_step,
            'max_step': max_step,
            'min_step': min_step,
            'start_pos': positions[0] if n_points > 0 else None,
            'end_pos': positions[-1] if n_points > 0 else None
        }
        
    except Exception as e:
        print(f"错误: 无法读取文件 {filename}: {e}")
        return None

def print_trajectory_info(info):
    """打印轨迹信息"""
    if info is None:
        return
    
    print(f"\n{'='*60}")
    print(f"文件: {info['filename']}")
    print(f"{'='*60}")
    
    print(f"\n📊 基本信息:")
    print(f"  数据点数量: {info['n_points']}")
    print(f"  持续时间: {info['duration']:.2f} 秒")
    print(f"  平均频率: {info['avg_freq']:.2f} Hz")
    
    print(f"\n📍 位置统计 (米):")
    print(f"  起始位置: [{info['start_pos'][0]:.6f}, {info['start_pos'][1]:.6f}, {info['start_pos'][2]:.6f}]")
    print(f"  结束位置: [{info['end_pos'][0]:.6f}, {info['end_pos'][1]:.6f}, {info['end_pos'][2]:.6f}]")
    print(f"  平均位置: [{info['pos_mean'][0]:.6f}, {info['pos_mean'][1]:.6f}, {info['pos_mean'][2]:.6f}]")
    
    print(f"\n📏 坐标范围 (米):")
    print(f"  X: [{info['pos_min'][0]:.6f}, {info['pos_max'][0]:.6f}] (范围: {info['pos_range'][0]:.6f})")
    print(f"  Y: [{info['pos_min'][1]:.6f}, {info['pos_max'][1]:.6f}] (范围: {info['pos_range'][1]:.6f})")
    print(f"  Z: [{info['pos_min'][2]:.6f}, {info['pos_max'][2]:.6f}] (范围: {info['pos_range'][2]:.6f})")
    
    print(f"\n📐 运动统计:")
    print(f"  轨迹总长度: {info['total_length']:.6f} 米")
    print(f"  总位移: {info['total_displacement']:.6f} 米")
    print(f"  平均步长: {info['avg_step']:.6f} 米")
    print(f"  最大步长: {info['max_step']:.6f} 米")
    print(f"  最小步长: {info['min_step']:.6f} 米")
    
    print(f"\n📊 位置标准差 (米):")
    print(f"  X: {info['pos_std'][0]:.6f}")
    print(f"  Y: {info['pos_std'][1]:.6f}")
    print(f"  Z: {info['pos_std'][2]:.6f}")

def compare_trajectories(info1, info2):
    """比较两个轨迹"""
    if info1 is None or info2 is None:
        return
    
    print(f"\n{'='*60}")
    print(f"轨迹对比")
    print(f"{'='*60}")
    
    print(f"\n📊 数据量对比:")
    print(f"  {info1['filename']}: {info1['n_points']} 点")
    print(f"  {info2['filename']}: {info2['n_points']} 点")
    print(f"  比例: {info1['n_points']/info2['n_points']:.2f}:1")
    
    print(f"\n📏 轨迹长度对比:")
    print(f"  {info1['filename']}: {info1['total_length']:.6f} 米")
    print(f"  {info2['filename']}: {info2['total_length']:.6f} 米")
    print(f"  差异: {abs(info1['total_length'] - info2['total_length']):.6f} 米")
    
    print(f"\n📐 位移对比:")
    print(f"  {info1['filename']}: {info1['total_displacement']:.6f} 米")
    print(f"  {info2['filename']}: {info2['total_displacement']:.6f} 米")
    print(f"  差异: {abs(info1['total_displacement'] - info2['total_displacement']):.6f} 米")
    
    print(f"\n⏱️ 时间对比:")
    print(f"  {info1['filename']}: {info1['duration']:.2f} 秒 ({info1['avg_freq']:.2f} Hz)")
    print(f"  {info2['filename']}: {info2['duration']:.2f} 秒 ({info2['avg_freq']:.2f} Hz)")

def main():
    """主函数"""
    print("SLAM轨迹数据分析器")
    print("="*60)
    
    #! 文件路径
    f_file = "f_dataset-Basler_mono_final_23.txt"
    kf_file = "kf_dataset-Basler_mono_final_23.txt"
    
    # 分析完整轨迹
    print("正在分析完整轨迹数据...")
    info_f = load_and_analyze_trajectory(f_file)
    if info_f:
        print_trajectory_info(info_f)
    
    # 分析关键帧轨迹
    print("\n正在分析关键帧轨迹数据...")
    info_kf = load_and_analyze_trajectory(kf_file)
    if info_kf:
        print_trajectory_info(info_kf)
    
    # 对比分析
    if info_f and info_kf:
        compare_trajectories(info_f, info_kf)
    
    print(f"\n{'='*60}")
    print("分析完成！")
    print("提示: 运行 'python3 visualize_trajectory_enhanced.py' 来生成可视化图表")
    print(f"{'='*60}")

if __name__ == "__main__":
    main() 