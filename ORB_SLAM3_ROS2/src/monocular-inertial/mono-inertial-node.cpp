#include "mono-inertial-node.hpp"


#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_srvs/srv/set_bool.hpp>

using std::placeholders::_1;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System* pSLAM, const std::string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(pSLAM)
{
    std::stringstream ss_eq(strDoEqual);
    ss_eq >> std::boolalpha >> doEqual_;
    bClahe_ = doEqual_;

    this->declare_parameter<double>("camera_time_offset", 0.0);
    this->declare_parameter<double>("imu_time_offset", 0.0);
    this->declare_parameter<bool>("localization_only", false);
    this->get_parameter("camera_time_offset", cameraTimeOffset_);
    this->get_parameter("imu_time_offset", imuTimeOffset_);
    this->get_parameter("localization_only", localizationOnly_);

    std::cout << "Equal: " << doEqual_ << std::endl;
    std::cout << "Time offset: camera=" << cameraTimeOffset_ << " imu=" << imuTimeOffset_ << std::endl;

    auto sensor_qos = rclcpp::SensorDataQoS();
    subImu_ = this->create_subscription<ImuMsg>("imu", sensor_qos, std::bind(&MonoInertialNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<ImageMsg>("camera", sensor_qos, std::bind(&MonoInertialNode::GrabImage, this, _1));

    localization_service_ = this->create_service<std_srvs::srv::SetBool>(
        "localization_mode",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            if (request->data) {
                SLAM_->ActivateLocalizationMode();
                response->message = "Localization mode enabled";
            } else {
                SLAM_->DeactivateLocalizationMode();
                response->message = "Localization mode disabled";
            }
            response->success = true;
        });

    if (localizationOnly_) {
        SLAM_->ActivateLocalizationMode();
        RCLCPP_INFO(this->get_logger(), "Localization mode enabled at startup");
    }

    pubs_.init(this);

    syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);
}

MonoInertialNode::~MonoInertialNode()
{
    syncThread_->join();
    delete syncThread_;

    // Save trajectory before Shutdown so we get KeyFrameTrajectory.txt even if Shutdown() segfaults later
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM_->SaveTrajectoryEuRoC("FrameTrajectory.txt");
    // Atlas is saved inside Shutdown() after LocalMapping/LoopClosing threads have stopped
    SLAM_->Shutdown();
}

void MonoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    const double t = Utility::StampToSec(msg->header.stamp) + imuTimeOffset_;
    if (lastImuTs_ >= 0.0 && t <= lastImuTs_)
    {
        droppedOutOfOrderImu_.fetch_add(1);
        return;
    }
    if (lastImuTs_ >= 0.0)
    {
        double dt = t - lastImuTs_;
        if (dt > 0)
            imuFreqSamples_.push_back(1.0 / dt);
    }
    lastImuTs_ = t;
    const double now = this->get_clock()->now().seconds();
    if (imuLastLogTime_ < 0)
        imuLastLogTime_ = now;
    if (now - imuLastLogTime_ >= 1.0 && !imuFreqSamples_.empty())
    {
        double minHz = *std::min_element(imuFreqSamples_.begin(), imuFreqSamples_.end());
        double maxHz = *std::max_element(imuFreqSamples_.begin(), imuFreqSamples_.end());
        double mean = 0;
        for (double f : imuFreqSamples_)
            mean += f;
        mean /= imuFreqSamples_.size();
        double var = 0;
        for (double f : imuFreqSamples_)
            var += (f - mean) * (f - mean);
        double stdHz = (imuFreqSamples_.size() > 1) ? std::sqrt(var / (imuFreqSamples_.size() - 1)) : 0;
        if (stdHz > 3.0)
        {
            RCLCPP_WARN(this->get_logger(), "IMU topic freq (1s): min=%.2f max=%.2f std=%.2f Hz", minHz, maxHz, stdHz);
        }
        imuFreqSamples_.clear();
        imuLastLogTime_ = now;
    }

    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonoInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    const double t = Utility::StampToSec(msg->header.stamp) + cameraTimeOffset_;
    if (lastImageTs_ >= 0.0 && t <= lastImageTs_)
    {
        droppedOutOfOrderImage_.fetch_add(1);
        return;
    }
    if (lastImageTs_ >= 0.0)
    {
        double dt = t - lastImageTs_;
        if (dt > 0)
            imageFreqSamples_.push_back(1.0 / dt);
    }
    lastImageTs_ = t;
    const double now = this->get_clock()->now().seconds();
    if (imageLastLogTime_ < 0)
        imageLastLogTime_ = now;
    if (now - imageLastLogTime_ >= 1.0 && !imageFreqSamples_.empty())
    {
        double minHz = *std::min_element(imageFreqSamples_.begin(), imageFreqSamples_.end());
        double maxHz = *std::max_element(imageFreqSamples_.begin(), imageFreqSamples_.end());
        double mean = 0;
        for (double f : imageFreqSamples_)
            mean += f;
        mean /= imageFreqSamples_.size();
        double var = 0;
        for (double f : imageFreqSamples_)
            var += (f - mean) * (f - mean);
        double stdHz = (imageFreqSamples_.size() > 1) ? std::sqrt(var / (imageFreqSamples_.size() - 1)) : 0;
        if (stdHz > 3.0)
        {
            RCLCPP_WARN(this->get_logger(), "Image topic freq (1s): min=%.2f max=%.2f std=%.2f Hz", minHz, maxHz, stdHz);
        }
        imageFreqSamples_.clear();
        imageLastLogTime_ = now;
    }

    bufMutexImg_.lock();

    if (!imgBuf_.empty())
    {
        imageOverwriteCount_.fetch_add(1);
        imgBuf_.pop();
    }
    imgBuf_.push(msg);

    bufMutexImg_.unlock();
}

cv::Mat MonoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonoInertialNode::SyncWithImu()
{
    while (rclcpp::ok())
    {
        cv::Mat im;
        double tIm = 0;
        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            tIm = Utility::StampToSec(imgBuf_.front()->header.stamp) + cameraTimeOffset_;
            if (tIm > Utility::StampToSec(imuBuf_.back()->header.stamp) + imuTimeOffset_)
            {
                waitingForImuCount_.fetch_add(1);
                std::chrono::milliseconds tSleep(1);
                std::this_thread::sleep_for(tSleep);
                continue;
            }

            bufMutexImg_.lock();
            im = GetImage(imgBuf_.front());
            imgBuf_.pop();
            bufMutexImg_.unlock();

            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                vImuMeas.clear();
                while (!imuBuf_.empty() && (Utility::StampToSec(imuBuf_.front()->header.stamp) + imuTimeOffset_) <= tIm)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp) + imuTimeOffset_;
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (vImuMeas.empty())
            {
                emptyImuWindows_.fetch_add(1);
                std::chrono::milliseconds tSleep(1);
                std::this_thread::sleep_for(tSleep);
                continue;
            }

            if (bClahe_)
            {
                clahe_->apply(im, im);
            }

            Sophus::SE3f Tcw = SLAM_->TrackMonocular(im, tIm, vImuMeas);
            pubs_.publish(SLAM_, Tcw, tIm);

            ORB_SLAM3::Atlas* pAtlas = SLAM_->GetAtlas();
            const bool imuInitialized = pAtlas != nullptr && pAtlas->isImuInitialized();
            const int trackingState = SLAM_->GetTrackingState();
            const bool isLost = SLAM_->isLost();

            if (trackingState == ORB_SLAM3::Tracking::OK && !initMonoLogged_)
            {
                RCLCPP_INFO(this->get_logger(), "[INIT] Mono initialized");
                initMonoLogged_ = true;
            }

            if (!imuInitStartLogged_ && pAtlas != nullptr && trackingState == ORB_SLAM3::Tracking::OK &&
                pAtlas->KeyFramesInMap() >= 8 && !imuInitialized)
            {
                RCLCPP_INFO(this->get_logger(), "[INIT] IMU initialization started");
                imuInitStartLogged_ = true;
            }

            if (imuInitialized && !imuReadyLogged_)
            {
                RCLCPP_INFO(this->get_logger(), "[INIT] IMU initialization success, scale=computed");
                RCLCPP_INFO(this->get_logger(), "[INIT] Switch to VIO");
                imuReadyLogged_ = true;
            }

            if (isLost && !lastLostState_)
            {
                RCLCPP_WARN(this->get_logger(), "[RESET] Tracking lost, reinitializing");
                pubs_.reset();
                initMonoLogged_ = false;
                imuInitStartLogged_ = false;
                imuReadyLogged_ = false;
            }
            lastLostState_ = isLost;

            const double now = this->get_clock()->now().seconds();
            if (obsLastLogTime_ < 0.0)
            {
                obsLastLogTime_ = now;
            }
            if (now - obsLastLogTime_ >= 1.0)
            {
                const int emptyWindows = emptyImuWindows_.exchange(0);
                const int droppedImu = droppedOutOfOrderImu_.exchange(0);
                const int droppedImage = droppedOutOfOrderImage_.exchange(0);
                const int overwrittenImage = imageOverwriteCount_.exchange(0);
                const int waitingCount = waitingForImuCount_.exchange(0);
                // RCLCPP_INFO(this->get_logger(),
                //             "Obs(1s): empty_imu_windows=%d dropped_imu_ooo=%d dropped_img_ooo=%d overwritten_img=%d wait_for_imu=%d",
                //             emptyWindows, droppedImu, droppedImage, overwrittenImage, waitingCount);
                obsLastLogTime_ = now;
            }

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
        else
        {
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
