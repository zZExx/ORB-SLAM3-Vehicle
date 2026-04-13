#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "mono-inertial-node.hpp"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam3 mono-inertial path_to_vocabulary path_to_settings [do_equalize]" << std::endl;
        return 1;
    }

    std::string doEqual = "false";
    if (argc >= 4)
        doEqual = argv[3];

    // 4th arg (after do_equalize) controls visualization: "true"/"false"
    bool visualization = false;
    if (argc >= 5)
    {
        std::string vizArg = argv[4];
        visualization = (vizArg == "true" || vizArg == "1");
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<MonoInertialNode>(argv[1], argv[2], doEqual, visualization);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
