#include "ndt_localization/map_loader.h"

MapLoader::MapLoader(const rclcpp::NodeOptions &options) : Node("ndt_map_loader", options)
{
    this->declare_parameter("pcd_file_path", "/home/liqianqi/pb_rmsimulation/src/rm_localization/FAST_LIO/PCD/map.pcd");
    this->declare_parameter("map_topic", "points_map");

    this->get_parameter("pcd_file_path", pcd_file_path);
    this->get_parameter("map_topic", map_topic);

    init_tf_params();

    pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, 10);

    // TODO: load map cloud from pcd file
    file_list_.push_back(pcd_file_path);
    auto pc_msg = CreatePcd();
    out_msg = TransformMap(pc_msg);

    if(out_msg.width != 0)
    {
        out_msg.header.frame_id = "map";
        pub_map_cloud_->publish(out_msg);
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MapLoader::publish_map_cloud, this));
}

MapLoader::~MapLoader()
{


}

void MapLoader::publish_map_cloud()
{
    // TODO: publish map cloud
    if(out_msg.width != 0)
    {
        out_msg.header.stamp = this->get_clock()->now();
        pub_map_cloud_->publish(out_msg);
    }
}

void MapLoader::init_tf_params()
{
    // TODO: init tf params
    this->declare_parameter("x", 0.0);
    this->declare_parameter("y", 0.0);
    this->declare_parameter("z", 0.0);
    this->declare_parameter("roll", 0.0);
    this->declare_parameter("pitch", 0.0);
    this->declare_parameter("yaw", 0.0);

    this->get_parameter("x", tf_x_);
    this->get_parameter("y", tf_y_);
    this->get_parameter("z", tf_z_);
    this->get_parameter("roll", tf_roll_);
    this->get_parameter("pitch", tf_pitch_);
    this->get_parameter("yaw", tf_yaw_);

    RCLCPP_INFO(this->get_logger(), "init x %f", tf_x_);
    RCLCPP_INFO(this->get_logger(), "init y %f", tf_y_);
    RCLCPP_INFO(this->get_logger(), "init z %f", tf_z_);
    RCLCPP_INFO(this->get_logger(), "init roll %f", tf_roll_);
    RCLCPP_INFO(this->get_logger(), "init pitch %f", tf_pitch_);
    RCLCPP_INFO(this->get_logger(), "init yaw %f", tf_yaw_);
}

sensor_msgs::msg::PointCloud2 MapLoader::CreatePcd()
{
    // TODO: load pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading point cloud file");
        return sensor_msgs::msg::PointCloud2();
    }else
    {
        RCLCPP_INFO(this->get_logger(), "load pcd finished!!!");
    }

    sensor_msgs::msg::PointCloud2 pc_msg;
    pcl::toROSMsg(*cloud, pc_msg);
    return pc_msg;
}

sensor_msgs::msg::PointCloud2 MapLoader::TransformMap(sensor_msgs::msg::PointCloud2 &in)
{
    // TODO: transform map cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(in, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Translation3f tl_m2w(tf_x_, tf_y_, tf_z_);
    Eigen::AngleAxisf rot_x_m2w(tf_roll_, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_m2w(tf_pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_m2w(tf_yaw_, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f tf_m2w = (tl_m2w * rot_z_m2w * rot_y_m2w * rot_x_m2w).matrix();

    pcl::transformPointCloud(*cloud, *cloud_trans, tf_m2w);

    // SaveMap(cloud_trans);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud_trans, out_msg);
    return out_msg;
}

void MapLoader::SaveMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // TODO: save map cloud
    pcl::io::savePCDFileASCII("map.pcd", *cloud);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  
    auto options = rclcpp::NodeOptions();  
    auto node = std::make_shared<MapLoader>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
