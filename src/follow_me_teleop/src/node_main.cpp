#include <rclcpp/rclcpp.hpp>
#include "follow_me_teleop/mecanum_teleop.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // Create the joystick node
    auto teleop   = std::make_shared<follow_me::mecanum::teleop::MecanumTeleop>("teleop");

    // Run the node(s)
    executor->add_node(teleop);
    executor->spin();

    // Exit
    rclcpp::shutdown();
    return 0;
}