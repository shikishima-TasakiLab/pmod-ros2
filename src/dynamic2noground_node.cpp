#include "pmod_ros/dynamic2noground.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dynamic2Noground>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
