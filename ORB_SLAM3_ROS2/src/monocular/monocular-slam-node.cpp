#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    this->declare_parameter<bool>("localization_only", false);
    this->declare_parameter<std::string>("data_source", "subscribe");
    this->declare_parameter<std::string>("db_bag_path", "");
    this->declare_parameter<std::string>("db_camera_topic", "/camera/image_raw");
    this->declare_parameter<double>("db_play_rate", 1.0);
    this->get_parameter("localization_only", localizationOnly_);
    this->get_parameter("data_source", dataSource_);
    this->get_parameter("db_bag_path", dbBagPath_);
    this->get_parameter("db_camera_topic", dbCameraTopic_);
    this->get_parameter("db_play_rate", dbPlayRate_);

    if (dataSource_ == "db")
    {
        activeDataSource_.store(static_cast<int>(DataSourceMode::DB));
    }
    else
    {
        activeDataSource_.store(static_cast<int>(DataSourceMode::SUBSCRIBE));
    }

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

    source_switch_service_ = this->create_service<std_srvs::srv::SetBool>(
        "data_source_mode",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            DataSourceMode mode = request->data ? DataSourceMode::DB : DataSourceMode::SUBSCRIBE;
            if (mode == DataSourceMode::DB && dbBagPath_.empty())
            {
                response->success = false;
                response->message = "db_bag_path is empty";
                return;
            }
            SwitchDataSource(mode);
            response->success = true;
            response->message = request->data ? "Data source switched to DB" : "Data source switched to subscribe";
        });

    if (localizationOnly_) {
        m_SLAM->ActivateLocalizationMode();
        RCLCPP_INFO(this->get_logger(), "Localization mode enabled at startup");
    }

    pubs_.init(this);

    if (IsDbMode())
    {
        dbThread_ = new std::thread(&MonocularSlamNode::DbReadLoop, this);
    }
}

MonocularSlamNode::~MonocularSlamNode()
{
    stopDbThread_.store(true);
    if (dbThread_ != nullptr)
    {
        dbThread_->join();
        delete dbThread_;
        dbThread_ = nullptr;
    }

    // Save trajectory before Shutdown so we get KeyFrameTrajectory.txt even if Shutdown() segfaults
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryEuRoC("FrameTrajectory.txt");
    m_SLAM->Shutdown();
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    ProcessImage(msg, false);
}

bool MonocularSlamNode::IsDbMode() const
{
    return activeDataSource_.load() == static_cast<int>(DataSourceMode::DB);
}

void MonocularSlamNode::SwitchDataSource(DataSourceMode mode)
{
    int target = static_cast<int>(mode);
    if (activeDataSource_.load() == target)
    {
        return;
    }

    activeDataSource_.store(target);
    if (mode == DataSourceMode::DB)
    {
        if (dbReaderFinished_.load() && dbThread_ != nullptr)
        {
            if (dbThread_->joinable())
            {
                dbThread_->join();
            }
            delete dbThread_;
            dbThread_ = nullptr;
        }
        if (dbThread_ == nullptr)
        {
            stopDbThread_.store(false);
            dbThread_ = new std::thread(&MonocularSlamNode::DbReadLoop, this);
        }
        RCLCPP_INFO(this->get_logger(), "Data source switched to DB");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Data source switched to subscribe");
    }
}

void MonocularSlamNode::ProcessImage(const ImageMsg::SharedPtr msg, bool fromDb)
{
    if (!fromDb && IsDbMode())
    {
        return;
    }

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

void MonocularSlamNode::DbReadLoop()
{
    dbReaderFinished_.store(false);
    if (dbBagPath_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "DB mode enabled but db_bag_path is empty");
        dbReaderFinished_.store(true);
        return;
    }

    rosbag2_storage::StorageOptions storageOptions;
    storageOptions.uri = dbBagPath_;
    storageOptions.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converterOptions;
    converterOptions.input_serialization_format = "cdr";
    converterOptions.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader;
    try
    {
        reader.open(storageOptions, converterOptions);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open db bag: %s", e.what());
        dbReaderFinished_.store(true);
        return;
    }

    rclcpp::Serialization<ImageMsg> imageSerialization;
    double playRate = dbPlayRate_ > 0.0 ? dbPlayRate_ : 1.0;
    RCLCPP_INFO(this->get_logger(), "DB reader started: %s (db_play_rate=%.3f)", dbBagPath_.c_str(), playRate);
    bool playbackClockInit = false;
    std::chrono::steady_clock::time_point playbackT0Wall;
    double playbackT0Sensor = 0.0;
    double lastImageTs = -1.0;

    while (rclcpp::ok() && !stopDbThread_.load())
    {
        if (!IsDbMode())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (!reader.has_next())
        {
            RCLCPP_INFO(this->get_logger(), "DB reader reached end of bag");
            break;
        }

        std::shared_ptr<rosbag2_storage::SerializedBagMessage> serializedBagMessage = reader.read_next();
        if (!serializedBagMessage || serializedBagMessage->topic_name != dbCameraTopic_)
        {
            continue;
        }

        ImageMsg imageMsg;
        rclcpp::SerializedMessage serializedMsg(*serializedBagMessage->serialized_data);
        imageSerialization.deserialize_message(&serializedMsg, &imageMsg);

        double tIm = Utility::StampToSec(imageMsg.header.stamp);
        if (lastImageTs >= 0.0 && tIm <= lastImageTs)
        {
            continue;
        }

        if (!playbackClockInit)
        {
            playbackT0Wall = std::chrono::steady_clock::now();
            playbackT0Sensor = tIm;
            playbackClockInit = true;
        }
        else
        {
            const double sensorDelta = tIm - playbackT0Sensor;
            const std::chrono::duration<double> targetElapsed(sensorDelta / playRate);
            const std::chrono::steady_clock::time_point deadline =
                playbackT0Wall + std::chrono::duration_cast<std::chrono::steady_clock::duration>(targetElapsed);
            const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            if (deadline > now)
            {
                std::this_thread::sleep_until(deadline);
            }
        }

        ProcessImage(std::make_shared<ImageMsg>(imageMsg), true);
        lastImageTs = tIm;
    }

    RCLCPP_INFO(this->get_logger(), "DB reader stopped");
    dbReaderFinished_.store(true);
}
