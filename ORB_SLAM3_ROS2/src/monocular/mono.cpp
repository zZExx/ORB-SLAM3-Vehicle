#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"


int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam3 mono path_to_vocabulary path_to_settings [visualization]" << std::endl;
        return 1;
    }

    bool visualization = false;
    if (argc >= 4)
    {
        std::string vizArg = argv[3];
        visualization = (vizArg == "true" || vizArg == "1");
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
