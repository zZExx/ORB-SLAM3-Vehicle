#!/bin/bash


ros2 bag play "/mnt/ssd/ros2_bag/vins_bag_upgrade/bag5/bag5_0.db3" \
    --topics /camera/image_raw /imu/data \
    -r 1.0
