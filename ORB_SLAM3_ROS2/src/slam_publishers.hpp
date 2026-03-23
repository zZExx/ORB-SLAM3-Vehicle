#pragma once
// slam_publishers.hpp — shared pose/pointcloud/marker publishers for ORB-SLAM3 ROS2 nodes
// Included by both monocular and mono-inertial nodes.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "System.h"
#include "Atlas.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include <sophus/se3.hpp>
#include <vector>
#include <memory>

struct SlamPublishers
{
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             kf_path;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frustum;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_bc;
    nav_msgs::msg::Path                                           path_msg;
    double                                                        last_kf_path_time_ = -1.0;

    void reset()
    {
        path_msg.poses.clear();
    }

    void init(rclcpp::Node* node)
    {
        pose    = node->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_pose",    10);
        path    = node->create_publisher<nav_msgs::msg::Path>("/orb_path",                10);
        kf_path = node->create_publisher<nav_msgs::msg::Path>("/orb_kf_path",            1);
        cloud   = node->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_points",    1);
        frustum = node->create_publisher<visualization_msgs::msg::Marker>("/orb_frustum", 10);
        tf_bc   = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
        path_msg.header.frame_id = "world";
    }

    // Tcw = world-to-camera transform returned by TrackMonocular() (T_{cw}: p_c = Tcw * p_w)
    void publish(ORB_SLAM3::System* slam, const Sophus::SE3f& Tcw, double t_sec)
    {
        int state = slam->GetTrackingState();
        if (state != 2 && state != 3) return;  // OK or RECENTLY_LOST only

        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f    tv = Twc.translation();
        Eigen::Quaternionf qv = Twc.unit_quaternion();

        rclcpp::Time stamp(static_cast<int64_t>(t_sec * 1e9), RCL_ROS_TIME);

        // --- pose ---
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp    = stamp;
        ps.header.frame_id = "world";
        ps.pose.position.x = tv.x();
        ps.pose.position.y = tv.y();
        ps.pose.position.z = tv.z();
        ps.pose.orientation.x = qv.x();
        ps.pose.orientation.y = qv.y();
        ps.pose.orientation.z = qv.z();
        ps.pose.orientation.w = qv.w();
        pose->publish(ps);

        path_msg.header.stamp = stamp;
        path_msg.poses.push_back(ps);
        path->publish(path_msg);

        // --- TF ---
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = stamp;
        tf.header.frame_id = "world";
        tf.child_frame_id  = "camera_pose";
        tf.transform.translation.x = tv.x();
        tf.transform.translation.y = tv.y();
        tf.transform.translation.z = tv.z();
        tf.transform.rotation.x = qv.x();
        tf.transform.rotation.y = qv.y();
        tf.transform.rotation.z = qv.z();
        tf.transform.rotation.w = qv.w();
        tf_bc->sendTransform(tf);

        // --- map points (PointCloud2) ---
        auto mpts = slam->GetTrackedMapPoints();
        std::vector<Eigen::Vector3f> valid_pts;
        valid_pts.reserve(mpts.size());
        for (auto* mp : mpts)
            if (mp && !mp->isBad())
                valid_pts.push_back(mp->GetWorldPos());

        sensor_msgs::msg::PointCloud2 pc;
        pc.header.stamp    = stamp;
        pc.header.frame_id = "world";
        pc.height = 1;
        pc.width  = static_cast<uint32_t>(valid_pts.size());
        sensor_msgs::PointCloud2Modifier mod(pc);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(valid_pts.size());
        sensor_msgs::PointCloud2Iterator<float> it_x(pc, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(pc, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(pc, "z");
        for (const auto& p : valid_pts) {
            *it_x = p.x(); *it_y = p.y(); *it_z = p.z();
            ++it_x; ++it_y; ++it_z;
        }
        cloud->publish(pc);

        // --- camera frustum (LINE_LIST marker) ---
        visualization_msgs::msg::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = "world";
        m.ns = "orb_frustum"; m.id = 0;
        m.type   = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.01;
        m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;
        m.pose.orientation.w = 1.0;

        // frustum corners in camera frame (scale 0.15m, 4:3 aspect)
        float s = 0.15f;
        float ar = 4.0f / 3.0f;
        std::vector<Eigen::Vector3f> cam_pts = {
            {0,0,0}, { s*ar, s,s}, {0,0,0}, {-s*ar, s,s},
            {0,0,0}, {-s*ar,-s,s}, {0,0,0}, { s*ar,-s,s},
            { s*ar, s,s}, {-s*ar, s,s}, {-s*ar, s,s}, {-s*ar,-s,s},
            {-s*ar,-s,s}, { s*ar,-s,s}, { s*ar,-s,s}, { s*ar, s,s}
        };
        Eigen::Matrix3f R = Twc.rotationMatrix();
        for (const auto& cp : cam_pts) {
            Eigen::Vector3f wp = R * cp + tv;
            geometry_msgs::msg::Point gp;
            gp.x = wp.x(); gp.y = wp.y(); gp.z = wp.z();
            m.points.push_back(gp);
        }
        frustum->publish(m);

        // --- keyframe path from Atlas (1 Hz) — matches KeyFrameTrajectory.txt source ---
        if (t_sec - last_kf_path_time_ > 1.0) {
            last_kf_path_time_ = t_sec;
            ORB_SLAM3::Atlas* atlas = slam->GetAtlas();
            if (atlas) {
                std::vector<ORB_SLAM3::KeyFrame*> vpKFs = atlas->GetAllKeyFrames();
                std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);
                nav_msgs::msg::Path kf_path_msg;
                kf_path_msg.header.stamp    = stamp;
                kf_path_msg.header.frame_id = "world";
                for (ORB_SLAM3::KeyFrame* pKF : vpKFs) {
                    if (!pKF || pKF->isBad()) continue;
                    Sophus::SE3f Twc_kf = pKF->GetPoseInverse();
                    Eigen::Vector3f  t_kf = Twc_kf.translation();
                    Eigen::Quaternionf q_kf = Twc_kf.unit_quaternion();
                    geometry_msgs::msg::PoseStamped kf_ps;
                    kf_ps.header.stamp    = rclcpp::Time(
                        static_cast<int64_t>(pKF->mTimeStamp * 1e9), RCL_ROS_TIME);
                    kf_ps.header.frame_id = "world";
                    kf_ps.pose.position.x    = t_kf.x();
                    kf_ps.pose.position.y    = t_kf.y();
                    kf_ps.pose.position.z    = t_kf.z();
                    kf_ps.pose.orientation.x = q_kf.x();
                    kf_ps.pose.orientation.y = q_kf.y();
                    kf_ps.pose.orientation.z = q_kf.z();
                    kf_ps.pose.orientation.w = q_kf.w();
                    kf_path_msg.poses.push_back(kf_ps);
                }
                kf_path->publish(kf_path_msg);
            }
        }
    }
};
