#include "pmod_ros/pmod.hpp"

PMOD::PMOD()
: rclcpp::Node("pmod") {
    this->declare_parameter("hz", 5.0);
    this->declare_parameter("sub_queue_size", 10);
    this->declare_parameter("pub_queue_size", 2);
    this->declare_parameter("use_optical_frame", this->_use_optical_frame);
    this->declare_parameter("camera_frame_id", this->_camera_frame_id);
    this->declare_parameter("pmod_config_path", "/workspace/src/pmod_ros/config/pmod-5class.yaml");

    this->_default_options = torch::TensorOptions().dtype(torch::kFloat32);
    if (torch::hasCUDA() == true) {
        RCLCPP_INFO(this->get_logger(), "Use CUDA.");
        this->_device = torch::kCUDA;
    }
    this->_default_options = this->_default_options.device(this->_device);

    double hz;
    int sub_queue_size, pub_queue_size;
    this->get_parameter("hz", hz);
    this->get_parameter("sub_queue_size", sub_queue_size);
    this->get_parameter("pub_queue_size", pub_queue_size);
    this->get_parameter("use_optical_frame", this->_use_optical_frame);
    if (this->_use_optical_frame == false) {
        this->get_parameter("camera_frame_id", this->_camera_frame_id);
    }

    this->_create_publisher<sensor_msgs::msg::Image>(this->_pub_seg_color, "seg_color", pub_queue_size);
    this->_create_publisher<sensor_msgs::msg::Image>(this->_pub_seg_id, "seg_id", pub_queue_size);
    this->_create_publisher<sensor_msgs::msg::Image>(this->_pub_depth, "depth", pub_queue_size);
    this->_create_publisher<sensor_msgs::msg::PointCloud2>(this->_pub_points_ground, "points_ground", pub_queue_size);
    this->_create_publisher<sensor_msgs::msg::PointCloud2>(this->_pub_points_no_ground, "points_no_ground", pub_queue_size);
    this->_create_publisher<sensor_msgs::msg::PointCloud2>(this->_pub_points_static, "points_static", pub_queue_size);
    this->_create_publisher<sensor_msgs::msg::PointCloud2>(this->_pub_points_dynamic, "points_dynamic", pub_queue_size);

    // Load PMOD Config (YAML)
    std::string pmod_config_path;
    this->get_parameter("pmod_config_path", pmod_config_path);
    this->_load_pmod_config(pmod_config_path);

    // CameraParam SubPub
    this->_pub_camerainfo = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "sparse_depth/camera_info", rclcpp::QoS(sub_queue_size)
    );
    this->_sub_camerainfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera/camera_info", rclcpp::QoS(sub_queue_size),
        std::bind(&PMOD::_camerainfo_callback, this, std::placeholders::_1)
    );
}

void PMOD::_load_pmod_config(
    const std::string &path
) {
    YAML::Node pmod_config;
    try {
        pmod_config = YAML::LoadFile(path);
    } catch (YAML::BadFile &e) {
        throw std::runtime_error("Failed to load \"" + path + "\".");
    }

    this->_shape = cv::Size(
        pmod_config["width"].as<int>(),
        pmod_config["height"].as<int>()
    );

    for (size_t i = 0UL; i < pmod_config["seg_labels"].size(); i++) {
        seg_label label_config;
        label_config.id = pmod_config["seg_labels"][i]["id"].as<int>();
        label_config.is_dynamic = pmod_config["seg_labels"][i]["is_dynamic"].as<bool>();
        label_config.is_ground = pmod_config["seg_labels"][i]["is_ground"].as<bool>();
        label_config.color = torch::tensor({
            static_cast<uint8_t>(pmod_config["seg_labels"][i]["color"]["b"].as<int>()),
            static_cast<uint8_t>(pmod_config["seg_labels"][i]["color"]["g"].as<int>()),
            static_cast<uint8_t>(pmod_config["seg_labels"][i]["color"]["r"].as<int>())
        }, torch::TensorOptions().dtype(torch::kUInt8).device(this->_device));
        this->_seg_labels[label_config.id] = label_config;
    }

    // load checkpoint
    std::string checkpoint_path = pmod_config["checkpoint"].as<std::string>();
    this->_module.reset(new torch::jit::script::Module(
        torch::jit::load(checkpoint_path, this->_device)
    ));
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

template<class msg_T>
void PMOD::_create_publisher(
    std::shared_ptr<rclcpp::Publisher<msg_T> > &pub_ptr,
    const std::string &topic,
    int queue_size,
    bool pub_on
) {
    this->declare_parameter("pub_" + topic, false);
    this->get_parameter("pub_" + topic, pub_on);
    if (pub_on == true) {
        pub_ptr = this->create_publisher<msg_T>(topic, rclcpp::QoS(queue_size));
    }
}
