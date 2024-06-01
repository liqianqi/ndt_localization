#pragma once

// C++
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <sstream>
#include <memory>
#include <thread>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

class NDTLocalizer : public rclcpp::Node
{
public:
    NDTLocalizer() = delete;
    NDTLocalizer(const rclcpp::NodeOptions &options);
    ~NDTLocalizer();

private:
    // ROS2 pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr exe_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr transform_probability_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iteration_num_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

    // ROS2 sub
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

    // 必须指针，否则会出现编译错误, tf2_ros::Buffer 没有默认的构造函数
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

    Eigen::Matrix4f base_to_sensor_matrix_;
    Eigen::Matrix4f pre_trans, delta_trans;
    bool init_pose = false;

    std::string base_frame_;
    std::string map_frame_;

    // init guess for ndt
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_cov_msg_;

    std::mutex ndt_map_mtx_;

    double converged_param_transform_probability_;
    std::thread diagnostic_thread_;
    std::map<std::string, std::string> key_value_stdmap_;

    // function
    void init_params();
    void timer_diagnostic();

    bool get_transform(const std::string & target_frame, const std::string & source_frame,
                       const geometry_msgs::msg::TransformStamped::Ptr & transform_stamped_ptr,
                       rclcpp::Time & time_stamp);
    bool get_transform(const std::string & target_frame, 
                       const std::string & source_frame,
                       const geometry_msgs::msg::TransformStamped::Ptr & transform_stamped_ptr);
    void publish_tf(const std::string & frame_id, const std::string & child_frame_id,
                    const geometry_msgs::msg::PoseStamped & pose_msg);

    // calback 函数不可以是引用形式，去掉&
    void callback_pointsmap(const sensor_msgs::msg::PointCloud2::ConstPtr pointcloud2_msg_ptr);
    void callback_init_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr pose_conv_msg_ptr);
    void callback_pointcloud(const sensor_msgs::msg::PointCloud2::ConstPtr pointcloud2_msg_ptr);

};
