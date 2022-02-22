#pragma once
#include <rclcpp/rclcpp.hpp>
#include <torch/script.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <mutex>
#include <yaml-cpp/yaml.h>

struct seg_label {
    int id;
    torch::Tensor color;
    bool is_ground;
    bool is_dynamic;
};

// class CameraInfoPubSub : public Subscriber<sensor_msgs::msg::CameraInfo>
// {
// public:
//     CameraInfoPubSub(
//         const rclcpp::Node::SharedPtr &node,
//         const std::string &sub_topic,
//         const std::string &pub_topic,
//         const uint32_t height,
//         const uint32_t width,
//         const rclcpp::QoS &queue_size
//     ) : Subscriber(node, sub_topic, queue_size) {
//         this->_height = height;
//         this->_width = width;
//         this->_height_d = static_cast<double>(height);
//         this->_width_d = static_cast<double>(width);
//         this->_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
//             pub_topic, queue_size
//         );
//     }

// protected:
//     uint32_t _height;
//     uint32_t _width;
//     double _height_d;
//     double _width_d;
//     rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _pub;

//     void _callback(
//         const sensor_msgs::msg::CameraInfo::SharedPtr &msg
//     ) override {
//         double rate_x = this->_width_d / static_cast<double>(msg->width);
//         double rate_y = this->_height_d / static_cast<double>(msg->height);

//         sensor_msgs::msg::CameraInfo::SharedPtr camerainfo(new sensor_msgs::msg::CameraInfo(*msg));
//         camerainfo->k[0] *= rate_x;
//         camerainfo->k[2] *= rate_x;
//         camerainfo->k[4] *= rate_y;
//         camerainfo->k[5] *= rate_y;
//         camerainfo->height = this->_height;
//         camerainfo->width = this->_width;

//         this->_pub->publish(*camerainfo);

//         std::lock_guard<std::mutex> lock(this->_mtx);
//         this->_msg = camerainfo;
//     }
// };

class PMOD : public rclcpp::Node
{
public:
    PMOD();

protected:
    std::mutex _mtx;

    bool _use_sync = false;
    bool _use_optical_frame = false;

    std::unordered_map<int, seg_label> _seg_labels;
    std::string _camera_frame_id = "";
    cv::Size _shape = cv::Size(512, 256);
    float _range_norm = 80.0F;
    torch::DeviceType _device = torch::kCPU;
    torch::TensorOptions _default_options;

    rclcpp::TimerBase::SharedPtr _timer;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _sub_camerainfo;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_camera_rgb;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_camera_depth;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _pub_camerainfo;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_seg_id;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_seg_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_depth;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_points_ground;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_points_no_ground;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_points_static;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_points_dynamic;

    bool _can_transform = false;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_br;

    sensor_msgs::msg::CameraInfo::SharedPtr _camerainfo_msg;

    std::shared_ptr<torch::jit::script::Module> _module;

    void _camerainfo_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg
    );
    void _timer_callback();
    void _broadcast_static_tf(
        const std::string &parent,
        const std::string &child
    );
    void _load_pmod_config(const std::string &path);

    template<class msg_T>
    void _create_publisher(
        std::shared_ptr<rclcpp::Publisher<msg_T> > &pub_ptr,
        const std::string &topic,
        int queue_size,
        bool pub_on = false
    );
};
