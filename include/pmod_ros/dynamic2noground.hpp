#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>

#define THROTTLE_PERIOD 30000
#define DEFAULT_OCTREE_RESOLUTION 0.3F
#define DEFAULT_OCTREE_RADIUS 10.0F
#define DEFAULT_FRAME_ID "map"

class Dynamic2Noground : public rclcpp::Node
{
public:
    Dynamic2Noground();

protected:
    std::mutex _mtx;

    double _hz = 10.0;
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _noground_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _static_noground_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _dynamic_sub;
    rclcpp::TimerBase::SharedPtr _timer;

    sensor_msgs::msg::PointCloud2::SharedPtr _dynamic_msg;

    std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZL> > _octree;
    float _octree_resolution = DEFAULT_OCTREE_RESOLUTION;
    float _radius = DEFAULT_OCTREE_RADIUS;
    pcl::PointCloud<pcl::PointXYZL>::Ptr _map;
    pcl::PointCloud<pcl::PointXYZL>::Ptr _filtered_map;
    pcl::PointXYZL _search_point;
    std::string _frame_id = DEFAULT_FRAME_ID;

    pcl::PointCloud<pcl::PointXYZL>::ConstPtr _get_map(
        const geometry_msgs::msg::TransformStamped &tf_map2camera
    );
    void _transform_pointcloud(
        const pcl::PointCloud<pcl::PointXYZL> &src,
        pcl::PointCloud<pcl::PointXYZL> &dst,
        const geometry_msgs::msg::TransformStamped &tf_src2dst
    );
    void _dynamic_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg
    );
    void _map_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg
    );
    void _timer_callback();
};
