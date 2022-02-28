#include "pmod_ros/dynamic2noground.hpp"

Dynamic2Noground::Dynamic2Noground()
: rclcpp::Node("dynamic2noground")
{
    std::vector<std::string> init_maps;
    this->_hz = this->declare_parameter("hz", this->_hz);
    init_maps = this->declare_parameter("init_maps", init_maps);
    this->_frame_id = this->declare_parameter("init_frame_id", this->_frame_id);
    this->_radius = this->declare_parameter("radius", this->_radius);
    this->_octree_resolution = this->declare_parameter("resolution", this->_octree_resolution);

    this->_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZL>(this->_octree_resolution));

    if (init_maps.size() > 0) {
        pcl::PointCloud<pcl::PointXYZL> init_map;

        for (size_t i = 0UL; i < init_maps.size(); i++) {
            pcl::PointCloud<pcl::PointXYZL> tmp_map;
            if (pcl::io::loadPCDFile(init_maps[i], tmp_map) == -1) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't read file \"" << init_maps[i] << "\".");
                continue;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Load \"" << init_maps[i] << "\".");

            init_map += tmp_map;
        }

        this->_map.reset(new pcl::PointCloud<pcl::PointXYZL>(init_map));

        this->_octree->setInputCloud(this->_map);
        this->_octree->addPointsFromInputCloud();
    }

    this->_tf_buffer.reset(new tf2_ros::Buffer(this->get_clock()));
    this->_tf_listener.reset(new tf2_ros::TransformListener(*this->_tf_buffer));

    this->_noground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "points_no_ground", 1
    );

    this->_static_noground_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_map/noground", 1,
        std::bind(&Dynamic2Noground::_map_callback, this, std::placeholders::_1)
    );
    this->_dynamic_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_dynamic", 1,
        std::bind(&Dynamic2Noground::_dynamic_callback, this, std::placeholders::_1)
    );

    this->_timer = this->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int>(1e9 / this->_hz)),
        std::bind(&Dynamic2Noground::_timer_callback, this)
    );
}

pcl::PointCloud<pcl::PointXYZL>::ConstPtr Dynamic2Noground::_get_map(
    const geometry_msgs::msg::TransformStamped &tf_map2camera
) {
    Eigen::Vector3f tr(tf2::transformToEigen(tf_map2camera).translation().cast<float>());
    pcl::PointXYZL search_point;
    search_point.x = tr.x();
    search_point.y = tr.y();
    search_point.z = tr.z();
    search_point.label = 0;

    if (
        this->_search_point.x == search_point.x &&
        this->_search_point.y == search_point.y &&
        this->_search_point.z == search_point.z
    ) {
        std::lock_guard<std::mutex> lock(this->_mtx);
        return this->_filtered_map;
    }
    this->_search_point = search_point;

    pcl::ExtractIndices<pcl::PointXYZL> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    std::vector<float> point_squared_distance;

    {
        std::lock_guard<std::mutex> lock(this->_mtx);

        this->_octree->radiusSearch(search_point, this->_radius, inliers->indices, point_squared_distance);
        extract.setInputCloud(this->_map);
        extract.setIndices(inliers);
        this->_filtered_map.reset(new pcl::PointCloud<pcl::PointXYZL>);
        extract.filter(*this->_filtered_map);

        return this->_filtered_map;
    }
}

void Dynamic2Noground::_transform_pointcloud(
    const pcl::PointCloud<pcl::PointXYZL> &src,
    pcl::PointCloud<pcl::PointXYZL> &dst,
    const geometry_msgs::msg::TransformStamped &tf_src2dst
) {
    size_t len = src.points.size();

    if (len == 0ul) {
        dst = src;
        return;
    }

    Eigen::Isometry3d tmp_tf = tf2::transformToEigen(tf_src2dst);
    Eigen::Quaternionf tmp_q = Eigen::Quaternionf(tmp_tf.rotation().cast<float>()).conjugate();
    Eigen::Vector3f tmp_tr(-(tmp_q * tmp_tf.translation().cast<float>()));

    if (
        tmp_tr == Eigen::Vector3f::Zero() &&
        tmp_q.x() == 0.0F &&
        tmp_q.y() == 0.0F &&
        tmp_q.z() == 0.0F &&
        tmp_q.w() == 1.0F
    ) {
        dst = src;
        return;
    }

    if (&src != &dst) {
        dst.points.resize(len);
        dst.width = src.width;
        dst.height = src.height;
        dst.is_dense = src.is_dense;
        dst.header = src.header;
    }

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif
    for (size_t i = 0UL; i < len; i++) {
        pcl::PointXYZL src_point_pcl = src.points[i];
        Eigen::Vector3f src_point_eigen(src_point_pcl.x, src_point_pcl.y, src_point_pcl.z);
        Eigen::Vector3f dst_point_eigen = tmp_q * src_point_eigen + tmp_tr;

        dst.points[i].x = dst_point_eigen[0];
        dst.points[i].y = dst_point_eigen[1];
        dst.points[i].z = dst_point_eigen[2];
        dst.points[i].label = src_point_pcl.label;
    }
}

void Dynamic2Noground::_dynamic_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(this->_mtx);
    this->_dynamic_msg = msg;
}

void Dynamic2Noground::_map_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(this->_mtx);

    this->_frame_id = msg->header.frame_id;
    this->_map.reset(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::fromROSMsg(*msg, *this->_map);

    this->_octree.reset(
        new pcl::octree::OctreePointCloudSearch<pcl::PointXYZL>(this->_octree_resolution)
    );
    this->_octree->setInputCloud(this->_map);
    this->_octree->addPointsFromInputCloud();
}

void Dynamic2Noground::_timer_callback()
{
    std::string map_frame_id;
    sensor_msgs::msg::PointCloud2::SharedPtr dynamic_msg;
    {
        std::lock_guard<std::mutex> lock(this->_mtx);
        map_frame_id = this->_frame_id;
        dynamic_msg = this->_dynamic_msg;
    }

    if (dynamic_msg == nullptr) {
        RCLCPP_WARN_STREAM_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            THROTTLE_PERIOD,
            "No \"PointCloud2\" msgs subscribed."
        );
        return;
    }

    geometry_msgs::msg::TransformStamped tf_map2camera;
    try {
        tf_map2camera = this->_tf_buffer->lookupTransform(
            map_frame_id,
            dynamic_msg->header.frame_id,
            dynamic_msg->header.stamp
        );
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN_STREAM_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            THROTTLE_PERIOD,
            "[map(" << map_frame_id << ") -> camera (" << dynamic_msg->header.frame_id << ")] Transform Error: " << e.what()
        );
        return;
    }

    RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        THROTTLE_PERIOD,
        "Processing..."
    );

    pcl::PointCloud<pcl::PointXYZL> noground_map;
    pcl::fromROSMsg(*dynamic_msg, noground_map);

    pcl::PointCloud<pcl::PointXYZL>::ConstPtr static_noground_map = this->_get_map(tf_map2camera);
    pcl::PointCloud<pcl::PointXYZL> static_noground_camera;
    this->_transform_pointcloud(*static_noground_map, static_noground_camera, tf_map2camera);

    noground_map += static_noground_camera;

    sensor_msgs::msg::PointCloud2 noground_msg;
    pcl::toROSMsg(noground_map, noground_msg);
    noground_msg.header.stamp = dynamic_msg->header.stamp;
    noground_msg.header.frame_id = dynamic_msg->header.frame_id;

    this->_noground_pub->publish(noground_msg);
}
