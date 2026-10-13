#include "robstride_usb_can_bridge/robstride_usb_can_bridge_node.hpp"

#include <cstdlib>
#include <exception>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = EXIT_SUCCESS;
    try
    {
        rclcpp::spin(std::make_shared<robstride_usb_can_bridge::RobStrideUsbCanBridgeNode>());
    }
    catch (const std::exception& error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("robstride_usb_can_bridge"), "%s", error.what());
        result = EXIT_FAILURE;
    }
    rclcpp::shutdown();
    return result;
}
