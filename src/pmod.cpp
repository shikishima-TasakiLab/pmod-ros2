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

    yaml::parse_yaml("/workspace/src/pmod_ros/config/pmod-5class.yaml");

    // CameraParam SubPub
    this->_pub_camerainfo = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "sparse_depth/camera_info", rclcpp::QoS(sub_queue_size)
    );
    this->_sub_camerainfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera/camera_info", rclcpp::QoS(sub_queue_size),
        std::bind(&PMOD::_camerainfo_callback, this, std::placeholders::_1)
    );
}

void PMOD::_camerainfo_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg
) {
    double rate_x = static_cast<double>(this->_shape.width) / static_cast<double>(msg->width);
    double rate_y = static_cast<double>(this->_shape.height) / static_cast<double>(msg->height);

    sensor_msgs::msg::CameraInfo::SharedPtr camerainfo(new sensor_msgs::msg::CameraInfo(*msg));
    camerainfo->k[0] *= rate_x;
    camerainfo->k[2] *= rate_x;
    camerainfo->k[4] *= rate_y;
    camerainfo->k[5] *= rate_y;
    camerainfo->height = static_cast<uint32_t>(this->_shape.height);
    camerainfo->width = static_cast<uint32_t>(this->_shape.width);

    this->_pub_camerainfo->publish(*camerainfo);

    std::lock_guard<std::mutex> lock(this->_mtx);
    this->_camerainfo_msg = camerainfo;
}
