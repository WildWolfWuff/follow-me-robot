#include <rclcpp/rclcpp.hpp>
#include "followme_teleop/mecanum_teleop.hpp"
#include "followme_teleop/mecanum_joystick.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // Create the joystick node
    auto teleop   = std::make_shared<followme::mecanum::teleop::MecanumTeleop>("teleop");
    auto joystick = std::make_shared<followme::mecanum::teleop::MecanumJoystick>("joystick");

    // Run the node(s)
    executor->add_node(teleop);
    executor->add_node(joystick);
    executor->spin();

    // Exit
    rclcpp::shutdown();
    return 0;
}