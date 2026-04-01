# 项目目标
1. 需要实现完整的SLAM框架，包括建图，定位
2. 需要有重定位功能，此处的重定位指的是设备到达已建好图的地方直接定位自己位置
# 硬件背景
1. 低速车载（类AGV）单目相机 + IMU + RDKS100P
2. 相机为GPIO硬触发，帧率约20HZ，固定曝光时间6ms，相机时间戳打在成像中心时刻
3. IMU为Free Run，帧率约200HZ，IMU性能参数较好

# 软件背景
1. 算法框架是基于ORB-SLAM3，后续可能会加速superpoint或其他深度学习方法替换掉如ORB提取等组件

# 注意
1. 如果硬件设备确实无法达成目标，需要给出详细解释及证据
2. 如果ORB-SLAM3框架无法达成目标，需要给出详细解释及证据

## 工程路径
1.  ORB-SLAM3 算法逻辑 /home/sunrise/ORB_SLAM3/ORB_SLAM3
2. ROS2封装层 /home/sunrise/ORB_SLAM3/ORB_SLAM3_ROS2