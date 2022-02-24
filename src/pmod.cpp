#include "pmod_ros/pmod.hpp"

PMOD::PMOD()
: rclcpp::Node("pmod") {
    this->declare_parameter("hz", 5.0);
    this->declare_parameter("sub_queue_size", 10);
    this->declare_parameter("pub_queue_size", 2);
    this->declare_parameter("use_optical_frame", this->_use_optical_frame);
    this->declare_parameter("camera_frame_id", this->_camera_frame_id);
    this->declare_parameter("pmod_config_path", "/workspace/src/pmod_ros/config/pmod-5class.yaml");
    this->declare_parameter("use_sync", this->_use_sync);

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

    if (this->_use_sync == true) {
        this->_sub_camera_rgb_sync = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(
            this, "camera", rclcpp::QoS(sub_queue_size).get_rmw_qos_profile()
        );
        // this->_sub_camera_rgb_sync->subscribe(this, "camera", rclcpp::QoS(sub_queue_size).get_rmw_qos_profile());
        this->_sub_sparse_depth_sync = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(
            this, "sparse_depth", rclcpp::QoS(sub_queue_size).get_rmw_qos_profile()
        );
        this->_sub_camera_sync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> >(
            *this->_sub_camera_rgb_sync, *this->_sub_sparse_depth_sync,
            sub_queue_size
        );
        this->_sub_camera_sync->registerCallback(
            &PMOD::_camera_sync_callback, this
        );
    }
    else {
        this->_sub_camera_rgb = this->create_subscription<sensor_msgs::msg::Image>(
            "camera", rclcpp::QoS(sub_queue_size),
            std::bind(&PMOD::_camera_callback, this, std::placeholders::_1)
        );
        this->_sub_sparse_depth = this->create_subscription<sensor_msgs::msg::Image>(
            "sparse_depth", rclcpp::QoS(sub_queue_size),
            std::bind(&PMOD::_sparse_depth_callback, this, std::placeholders::_1)
        );
    }

    this->_timer = this->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int>(1e9 / hz)),
        std::bind(&PMOD::_timer_callback, this)
    );
}

void PMOD::_camera_sync_callback(
    const sensor_msgs::msg::Image::SharedPtr rgb,
    const sensor_msgs::msg::Image::SharedPtr sparse_depth
) {
    std::lock_guard<std::mutex> lock(this->_mtx);
    this->_camera_msg = rgb;
    this->_sparse_depth_msg = sparse_depth;
}

void PMOD::_camera_callback(
    const sensor_msgs::msg::Image::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(this->_mtx);
    this->_camera_msg = msg;
}

void PMOD::_sparse_depth_callback(
    const sensor_msgs::msg::Image::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(this->_mtx);
    this->_sparse_depth_msg = msg;
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Label: Loaded");

    // load checkpoint
    std::string checkpoint_path = pmod_config["checkpoint"].as<std::string>();
    RCLCPP_INFO_STREAM(this->get_logger(), "Checkpoint: " << checkpoint_path);
    this->_module = boost::make_shared<torch::jit::script::Module>(
        torch::jit::load(checkpoint_path, this->_device)
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

void PMOD::_broadcast_static_tf(
    const std::string &parent,
    std::string &child
) {
    child = parent + "/base";
    this->_static_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped tfstamp_msg;
    tfstamp_msg.header.frame_id = parent;
    tfstamp_msg.child_frame_id = child;
    tfstamp_msg.transform.rotation.w = 0.5;
    tfstamp_msg.transform.rotation.x = 0.5;
    tfstamp_msg.transform.rotation.y = -0.5;
    tfstamp_msg.transform.rotation.z = 0.5;
    this->_static_br->sendTransform(tfstamp_msg);
}

void PMOD::_timer_callback()
{
    torch::NoGradGuard no_grad;

    sensor_msgs::msg::CameraInfo::SharedPtr camerainfo;
    sensor_msgs::msg::Image::SharedPtr camera_msg, sparse_depth_msg;
    {
        std::lock_guard<std::mutex> lock(this->_mtx);
        camerainfo = this->_camerainfo_msg;
        camera_msg = this->_camera_msg;
        sparse_depth_msg = this->_sparse_depth_msg;
    }

    if (camera_msg == nullptr || sparse_depth_msg == nullptr) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            30000,
            "No \"Image\" msgs subscribed."
        );
        return;
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        30000,
        "Processing..."
    );

    std::vector<torch::jit::IValue> inputs;

    cv_bridge::CvImagePtr rgb_cv = cv_bridge::toCvCopy(camera_msg, camera_msg->encoding);
    if (rgb_cv->encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::cvtColor(rgb_cv->image, rgb_cv->image, CV_BGRA2BGR);
    }
    rgb_cv->image.convertTo(rgb_cv->image, CV_32FC3);
    inputs.push_back(torch::from_blob(
        rgb_cv->image.data,
        {1, rgb_cv->image.rows, rgb_cv->image.cols, 3},
        torch::kFloat32
    ).to(this->_default_options).permute({0,3,1,2}));

    cv_bridge::CvImageConstPtr sparse_depth_cv = cv_bridge::toCvShare(sparse_depth_msg, sparse_depth_msg->encoding);
    inputs.push_back(torch::from_blob(
        sparse_depth_cv->image.data,
        {1, this->_shape.height, this->_shape.width, 1},
        torch::kFloat32
    ).to(this->_default_options).permute({0,3,1,2}));

    if (
        rgb_cv->image.rows != this->_shape.height ||
        rgb_cv->image.cols != this->_shape.width
    ) {
        inputs[0] = at::upsample_bilinear2d(
            inputs[0].toTensor(),
            {this->_shape.height, this->_shape.width},
            false
        );
    }
    inputs[0] = inputs[0].toTensor() / 255.0F;
    inputs[1] = torch::where(
        ((0.0F < inputs[1].toTensor()) & (inputs[1].toTensor() < this->_range_norm)),
        inputs[1].toTensor() / this->_range_norm,
        torch::full_like(inputs[1].toTensor(), 2.0F, this->_default_options)
    );

    std::vector<c10::IValue> outputs = this->_module->forward(inputs).toTuple()->elements();

    torch::Tensor out_seg = outputs[0].toTensor().argmax(1).to(torch::kUInt8).squeeze_();
    torch::Tensor out_depth = torch::where(
        ((outputs[1].toTensor() < 0.0F) | (1.0F < outputs[1].toTensor())),
        torch::full_like(outputs[1].toTensor(), INFINITY, this->_default_options),
        outputs[1].toTensor() * this->_range_norm
    ).squeeze_();

    if (this->_pub_seg_id != nullptr) {
        torch::Tensor label_tensor_cpu = out_seg.to(torch::kCPU);
        cv_bridge::CvImage label_cv(
            camera_msg->header,
            sensor_msgs::image_encodings::TYPE_8UC1,
            cv::Mat(this->_shape, CV_8UC1, label_tensor_cpu.data_ptr())
        );
        sensor_msgs::msg::Image::SharedPtr label_msg = label_cv.toImageMsg();
        label_msg->header.stamp = camera_msg->header.stamp;
        this->_pub_seg_id->publish(*label_msg);
    }

    if (this->_pub_seg_color != nullptr) {
        torch::Tensor label_tensor = torch::zeros(
            {this->_shape.height, this->_shape.width, 3},
            torch::TensorOptions().dtype(torch::kUInt8).device(this->_device)
        );

        for (
            auto seg_labels_itr = this->_seg_labels.begin();
            seg_labels_itr != this->_seg_labels.end();
            seg_labels_itr++
        ) {
            std::vector<torch::Tensor> label_idx = torch::where(out_seg == seg_labels_itr->second.id);
            label_tensor = label_tensor.index_put_(label_idx, seg_labels_itr->second.color);
        }

        torch::Tensor label_tensor_cpu = label_tensor.to(torch::kCPU);
        cv_bridge::CvImage label_cv(
            camera_msg->header,
            sensor_msgs::image_encodings::BGR8,
            cv::Mat(this->_shape, CV_8UC3, label_tensor_cpu.data_ptr())
        );
        sensor_msgs::msg::Image::SharedPtr label_msg = label_cv.toImageMsg();
        label_msg->header.stamp = camera_msg->header.stamp;
        this->_pub_seg_color->publish(*label_msg);
    }

    if (this->_pub_depth != nullptr) {
        torch::Tensor depth_tensor_cpu = out_depth.to(torch::kCPU);
        cv_bridge::CvImage depth_cv(
            camera_msg->header,
            sensor_msgs::image_encodings::TYPE_32FC1,
            cv::Mat(this->_shape, CV_32FC1, depth_tensor_cpu.data_ptr())
        );
        sensor_msgs::msg::Image::SharedPtr depth_msg = depth_cv.toImageMsg();
        depth_msg->header.stamp = camera_msg->header.stamp;
        this->_pub_depth->publish(*depth_msg);
    }

    if (
        this->_pub_points_dynamic == nullptr &&
        this->_pub_points_static == nullptr &&
        this->_pub_points_ground == nullptr &&
        this->_pub_points_no_ground == nullptr
    ) {
        return;
    }
    if (camerainfo == nullptr) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            30000,
            "No \"CameraInfo\" msgs subscribed."
        );
        return;
    }

    torch::Tensor image_u = torch::arange(
        this->_shape.width, this->_default_options
    ).expand({this->_shape.height, -1});
    torch::Tensor image_v = torch::arange(
        this->_shape.height, this->_default_options
    ).expand({this->_shape.width, -1}).permute({1, 0});
    torch::Tensor points_x = (
        out_depth * (image_u - static_cast<float>(camerainfo->k[2])) / static_cast<float>(camerainfo->k[0])
    ).to(torch::kCPU);
    torch::Tensor points_y = (
        out_depth * (image_v - static_cast<float>(camerainfo->k[5])) / static_cast<float>(camerainfo->k[4])
    ).to(torch::kCPU);
    torch::Tensor points_z = out_depth.to(torch::kCPU);
    torch::Tensor label_tensor_cpu = out_seg.to(torch::kCPU);

    cv::Mat points_x_cv(this->_shape, CV_32FC1, points_x.data_ptr());
    cv::Mat points_y_cv(this->_shape, CV_32FC1, points_y.data_ptr());
    cv::Mat points_z_cv(this->_shape, CV_32FC1, points_z.data_ptr());
    cv::Mat label_cv(this->_shape, CV_8UC1, label_tensor_cpu.data_ptr());

    cv::Mat label_edge;
    cv::Laplacian(label_cv, label_edge, CV_8U, 5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_no_ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_dynamic;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_static;

    if (this->_use_optical_frame == true) {
        this->_camera_frame_id = camerainfo->header.frame_id;
    }
    else if (this->_can_transform == false) {
        tf2_ros::Buffer tf_buffer(this->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);
        if (this->_camera_frame_id == "") {
            this->_broadcast_static_tf(
                camerainfo->header.frame_id,
                this->_camera_frame_id
            );
        }
        else if (tf_buffer.canTransform(this->_camera_frame_id, camerainfo->header.frame_id, tf2::TimePoint()) == false) {
            this->_broadcast_static_tf(
                camerainfo->header.frame_id,
                this->_camera_frame_id
            );
        }
        this->_can_transform = true;
    }

    if (this->_pub_points_ground != nullptr) {
        points_ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
        points_ground->header.frame_id = this->_camera_frame_id;
    }
    if (this->_pub_points_no_ground != nullptr) {
        points_no_ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
        points_no_ground->header.frame_id = this->_camera_frame_id;
    }
    if (this->_pub_points_dynamic != nullptr) {
        points_dynamic.reset(new pcl::PointCloud<pcl::PointXYZ>);
        points_dynamic->header.frame_id = this->_camera_frame_id;
    }
    if (this->_pub_points_static != nullptr) {
        points_static.reset(new pcl::PointCloud<pcl::PointXYZ>);
        points_static->header.frame_id = this->_camera_frame_id;
    }

    for (int y = 0; y < this->_shape.height; y++) {
        float *points_x_cv_ptr = points_x_cv.ptr<float>(y);
        float *points_y_cv_ptr = points_y_cv.ptr<float>(y);
        float *points_z_cv_ptr = points_z_cv.ptr<float>(y);
        uint8_t *label_cv_ptr = label_cv.ptr<uint8_t>(y);
        uint8_t *label_edge_ptr = label_edge.ptr<uint8_t>(y);

        for (int x = 0; x < this->_shape.width; x++) {
            if (std::isinf(points_z_cv_ptr[x]) == true) continue;
            if (label_edge_ptr[x] != 0) continue;
            auto seg_label_itr = this->_seg_labels.find(static_cast<int>(label_cv_ptr[x]));
            if (seg_label_itr == this->_seg_labels.end()) continue;

            pcl::PointXYZ point;
            if (this->_use_optical_frame == true) {
                point.x = points_x_cv_ptr[x];
                point.y = points_y_cv_ptr[x];
                point.z = points_z_cv_ptr[x];
            }
            else {
                point.x = points_z_cv_ptr[x];
                point.y = -points_x_cv_ptr[x];
                point.z = -points_y_cv_ptr[x];
            }
            if (
                seg_label_itr->second.is_ground == true &&
                this->_pub_points_ground != nullptr
            ) points_ground->points.push_back(point);
            else if (
                seg_label_itr->second.is_ground == false &&
                this->_pub_points_no_ground != nullptr
            ) points_no_ground->points.push_back(point);
            if (
                seg_label_itr->second.is_dynamic == true &&
                this->_pub_points_dynamic != nullptr
            ) points_dynamic->points.push_back(point);
            else if (
                seg_label_itr->second.is_dynamic == false &&
                this->_pub_points_static != nullptr
            ) points_static->points.push_back(point);
        }
    }

    if (this->_pub_points_ground != nullptr) {
        sensor_msgs::msg::PointCloud2 points_ground_msg;
        pcl::toROSMsg(*points_ground, points_ground_msg);
        points_ground_msg.header.stamp = camera_msg->header.stamp;
        this->_pub_points_ground->publish(points_ground_msg);
    }

    if (this->_pub_points_no_ground != nullptr) {
        sensor_msgs::msg::PointCloud2 points_no_ground_msg;
        pcl::toROSMsg(*points_no_ground, points_no_ground_msg);
        points_no_ground_msg.header.stamp = camera_msg->header.stamp;
        this->_pub_points_no_ground->publish(points_no_ground_msg);
    }

    if (this->_pub_points_dynamic != nullptr) {
        sensor_msgs::msg::PointCloud2 points_dynamic_msg;
        pcl::toROSMsg(*points_dynamic, points_dynamic_msg);
        points_dynamic_msg.header.stamp = camera_msg->header.stamp;
        this->_pub_points_dynamic->publish(points_dynamic_msg);
    }

    if (this->_pub_points_static != nullptr) {
        sensor_msgs::msg::PointCloud2 points_static_msg;
        pcl::toROSMsg(*points_static, points_static_msg);
        points_static_msg.header.stamp = camera_msg->header.stamp;
        this->_pub_points_static->publish(points_static_msg);
    }
}
