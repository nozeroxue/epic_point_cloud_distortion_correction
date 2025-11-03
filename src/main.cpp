#include "rclcpp/rclcpp.hpp"
#include "correction_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    auto node = std::make_shared<CorrectionNode>(options);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}