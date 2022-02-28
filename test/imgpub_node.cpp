#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("imgpub");

    std::string camera_path, sparse_depth_path;
    camera_path = node->declare_parameter("camera_path", camera_path);
    sparse_depth_path = node->declare_parameter("sparse_depth_path", sparse_depth_path);

    sensor_msgs::msg::CameraInfo camerainfo_msg;
    camerainfo_msg.k[0] = node->declare_parameter("Fx", camerainfo_msg.k[0]);
    camerainfo_msg.k[2] = node->declare_parameter("Cx", camerainfo_msg.k[2]);
    camerainfo_msg.k[4] = node->declare_parameter("Fy", camerainfo_msg.k[4]);
    camerainfo_msg.k[5] = node->declare_parameter("Cy", camerainfo_msg.k[5]);
    camerainfo_msg.header.frame_id = node->declare_parameter("frame_id", camerainfo_msg.header.frame_id);

    cv_bridge::CvImage camera_cv, sparse_depth_cv;
    camera_cv.image = cv::imread(camera_path, cv::IMREAD_ANYCOLOR);
    if (camera_cv.image.empty() == true) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Load Failed: \"" << camera_path << "\"");
        return EXIT_FAILURE;
    }
    camera_cv.encoding = sensor_msgs::image_encodings::BGR8;
    camerainfo_msg.height = camera_cv.image.rows;
    camerainfo_msg.width = camera_cv.image.cols;
    sensor_msgs::msg::Image::SharedPtr camera_msg = camera_cv.toImageMsg();

    sparse_depth_cv.image = cv::imread(sparse_depth_path, cv::IMREAD_UNCHANGED);
    if (sparse_depth_cv.image.empty() == true) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Load Failed: \"" << sparse_depth_path << "\"");
        return EXIT_FAILURE;
    }
    sparse_depth_cv.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sensor_msgs::msg::Image::SharedPtr sparse_depth_msg = sparse_depth_cv.toImageMsg();

    auto camerainfo_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 1);
    auto camera_pub = node->create_publisher<sensor_msgs::msg::Image>("camera", 1);
    auto sparse_depth_pub = node->create_publisher<sensor_msgs::msg::Image>("sparse_depth", 1);

    rclcpp::WallRate loop_rate(1s);

    while (rclcpp::ok()) {
        camerainfo_msg.header.stamp = node->get_clock()->now();
        camera_msg->header = camerainfo_msg.header;
        sparse_depth_msg->header = camerainfo_msg.header;

        camerainfo_pub->publish(camerainfo_msg);
        camera_pub->publish(*camera_msg);
        sparse_depth_pub->publish(*sparse_depth_msg);

        RCLCPP_INFO_ONCE(node->get_logger(), "publishing and latching message. Press ctrl-C to terminate.");

        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
