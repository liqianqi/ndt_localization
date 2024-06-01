#ifndef MAP_LOADER_H
#define MAP_LOADER_H

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

// C++
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <memory>

class MapLoader : public rclcpp::Node
{
public:
    MapLoader(const rclcpp::NodeOptions &options);
    ~MapLoader();

private:
    std::vector<std::string> file_list_;
    std::string pcd_file_path;
    std::string map_topic;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_ = nullptr;
    float tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_; 

    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2 out_msg;

    void init_tf_params();
    sensor_msgs::msg::PointCloud2 CreatePcd();
    sensor_msgs::msg::PointCloud2 TransformMap(sensor_msgs::msg::PointCloud2 & in);
    void SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr);
    void publish_map_cloud();
};












#endif // MAP_LOADER_H