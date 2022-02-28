#pragma once
#include <torch/script.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

struct seg_label {
    int id;
    torch::Tensor color;
    bool is_ground;
    bool is_dynamic;
};

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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_sparse_depth;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > _sub_camera_rgb_sync;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > _sub_sparse_depth_sync;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> > _sub_camera_sync;

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
    sensor_msgs::msg::Image::SharedPtr _camera_msg;
    sensor_msgs::msg::Image::SharedPtr _sparse_depth_msg;

    std::shared_ptr<torch::jit::script::Module> _module;

    void _camerainfo_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg
    );
    void _camera_sync_callback(
        const sensor_msgs::msg::Image::SharedPtr rgb,
        const sensor_msgs::msg::Image::SharedPtr sparse_depth
    );
    void _camera_callback(
        const sensor_msgs::msg::Image::SharedPtr msg
    );
    void _sparse_depth_callback(
        const sensor_msgs::msg::Image::SharedPtr msg
    );
    void _timer_callback();
    void _broadcast_static_tf(
        const std::string &parent,
        std::string &child
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
