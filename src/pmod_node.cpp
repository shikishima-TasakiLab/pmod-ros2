#include "pmod_ros/pmod.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PMOD>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
