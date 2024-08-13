#include <rclcpp/rclcpp.hpp>
#include "follow_me_path_builder/path_builder.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // Create the joystick node
    auto path_builder = std::make_shared<follow_me::PathBuilder>("path_builder");

    // Run the node(s)
    executor->add_node(path_builder);
    executor->spin();

    // Exit
    rclcpp::shutdown();
    return 0;
}