#include "pmod_ros/pmod.hpp"

PMOD::PMOD()
: rclcpp::Node("pmod") {
    this->declare_parameter("hz", 5.0);
    this->declare_parameter("height", this->_shape.height);
    this->declare_parameter("width", this->_shape.width);
    this->declare_parameter("sub_queue_size", 10);
    this->declare_parameter("pub_queue_size", 2);
    this->declare_parameter("use_optical_frame", this->_use_optical_frame);
    this->declare_parameter("camera_frame_id", this->_camera_frame_id);
    this->declare_parameter("pub_seg_color", true);

    this->_default_options = torch::TensorOptions().dtype(torch::kFloat32);
    if (torch::hasCUDA() == true) {
        RCLCPP_INFO(this->get_logger(), "Use CUDA.");
        this->_device = torch::kCUDA;
    }
    this->_default_options = this->_default_options.device(this->_device);

    double hz;
    int sub_queue_size, pub_queue_size;
    bool pub_seg_color = true;
    this->get_parameter("hz", hz);
    this->get_parameter("height", this->_shape.height);
    this->get_parameter("width", this->_shape.width);
    this->get_parameter("sub_queue_size", sub_queue_size);
    this->get_parameter("pub_queue_size", pub_queue_size);
    this->get_parameter("use_optical_frame", this->_use_optical_frame);
    if (this->_use_optical_frame == false) {
        this->get_parameter("camera_frame_id", this->_camera_frame_id);
    }
    this->get_parameter("pub_seg_color", pub_seg_color);

}
