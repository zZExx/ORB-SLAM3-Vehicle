#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include <std_srvs/srv/set_bool.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    this->declare_parameter<bool>("localization_only", false);
    this->get_parameter("localization_only", localizationOnly_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        sensor_qos,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    localization_service_ = this->create_service<std_srvs::srv::SetBool>(
        "localization_mode",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            if (request->data) {
                m_SLAM->ActivateLocalizationMode();
                response->message = "Localization mode enabled";
            } else {
                m_SLAM->DeactivateLocalizationMode();
                response->message = "Localization mode disabled";
            }
            response->success = true;
        });

    if (localizationOnly_) {
        m_SLAM->ActivateLocalizationMode();
        RCLCPP_INFO(this->get_logger(), "Localization mode enabled at startup");
    }

    pubs_.init(this);
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Save trajectory before Shutdown so we get KeyFrameTrajectory.txt even if Shutdown() segfaults
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryEuRoC("FrameTrajectory.txt");
    m_SLAM->Shutdown();
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    double t = Utility::StampToSec(msg->header.stamp);
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, t);
    pubs_.publish(m_SLAM, Tcw, t);
}
