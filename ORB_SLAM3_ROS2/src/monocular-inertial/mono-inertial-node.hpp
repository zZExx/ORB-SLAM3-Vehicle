#ifndef __MONO_INERTIAL_NODE_HPP__
#define __MONO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"
#include "../slam_publishers.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class MonoInertialNode : public rclcpp::Node
{
public:
    MonoInertialNode(ORB_SLAM3::System* pSLAM, const std::string &strDoEqual);
    ~MonoInertialNode();

private:
    enum class DataSourceMode
    {
        SUBSCRIBE = 0,
        DB = 1
    };

    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const ImageMsg::SharedPtr msg);
    void PushImu(const ImuMsg::SharedPtr msg, bool fromDb);
    void PushImage(const ImageMsg::SharedPtr msg, bool fromDb);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();
    void DbReadLoop();
    void SwitchDataSource(DataSourceMode mode);
    bool IsDbMode() const;

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr localization_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr source_switch_service_;

    SlamPublishers pubs_;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;
    std::thread *dbThread_{nullptr};
    std::atomic<bool> stopDbThread_{false};
    std::atomic<bool> dbReaderFinished_{true};

    // IMU
    std::queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    std::queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex bufMutexImg_;

    double lastImageTs_{-1.0};
    double lastImuTs_{-1.0};
    std::vector<double> imuFreqSamples_;
    std::vector<double> imageFreqSamples_;
    double imuLastLogTime_{-1.0};
    double imageLastLogTime_{-1.0};
    double obsLastLogTime_{-1.0};
    double cameraTimeOffset_{0.0};
    double imuTimeOffset_{0.0};
    std::string dataSource_{"subscribe"};
    std::string dbBagPath_;
    std::string dbCameraTopic_{"/camera/image_raw"};
    std::string dbImuTopic_{"/imu/data"};
    double dbPlayRate_{1.0};
    std::atomic<int> activeDataSource_{static_cast<int>(DataSourceMode::SUBSCRIBE)};
    bool localizationOnly_{false};
    bool initMonoLogged_{false};
    bool imuInitStartLogged_{false};
    bool imuReadyLogged_{false};
    bool lastLostState_{false};
    std::atomic<int> emptyImuWindows_{0};
    std::atomic<int> droppedOutOfOrderImu_{0};
    std::atomic<int> droppedOutOfOrderImage_{0};
    std::atomic<int> imageOverwriteCount_{0};
    std::atomic<int> waitingForImuCount_{0};

    bool doEqual_;
    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
