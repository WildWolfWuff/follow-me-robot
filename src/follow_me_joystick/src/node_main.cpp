#include <rclcpp/rclcpp.hpp>
#include "follow_me_joystick/mecanum_joystick.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // Create the joystick node
    auto joystick = std::make_shared<follow_me::mecanum::teleop::MecanumJoystick>("joystick");

    // Run the node(s)
    executor->add_node(joystick);
    executor->spin();

    // Exit
    rclcpp::shutdown();
    return 0;
}