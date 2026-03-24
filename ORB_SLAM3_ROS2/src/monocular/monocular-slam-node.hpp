#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"
#include "../slam_publishers.hpp"
#include <atomic>
#include <thread>

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    enum class DataSourceMode
    {
        SUBSCRIBE = 0,
        DB = 1
    };

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void ProcessImage(const ImageMsg::SharedPtr msg, bool fromDb);
    void DbReadLoop();
    void SwitchDataSource(DataSourceMode mode);
    bool IsDbMode() const;

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr localization_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr source_switch_service_;
    std::thread* dbThread_{nullptr};
    std::atomic<bool> stopDbThread_{false};
    std::atomic<bool> dbReaderFinished_{true};
    std::string dataSource_{"subscribe"};
    std::string dbBagPath_;
    std::string dbCameraTopic_{"/camera/image_raw"};
    double dbPlayRate_{1.0};
    std::atomic<int> activeDataSource_{static_cast<int>(DataSourceMode::SUBSCRIBE)};
    bool localizationOnly_{false};

    SlamPublishers pubs_;
};

#endif
