#include "CubeEyeCameraNode.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubeEyeCameraNode>();

    RCLCPP_INFO(node->get_logger(), "cubeeye camera node started");

    rclcpp::spin(node);
    rclcpp::shutdown();

    node->shutdown();
    RCLCPP_INFO(node->get_logger(), "cubeeye camera node stopped");
    return 0;
}

