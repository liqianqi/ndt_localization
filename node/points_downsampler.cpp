#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// #include "points_downsampler.h"

#define MAX_MEASUREMENT_RANGE 120.0

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub;

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static bool _output_log = false;
static std::ofstream ofs;
static std::string filename;

static std::string POINTS_TOPIC;

static pcl::PointCloud<pcl::PointXYZ> removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZ> narrowed_scan;
    narrowed_scan.header = scan.header;

    if (min_range >= max_range)
    {
        printf("min_range>=max_range @(%lf, %lf)", min_range, max_range);
        return scan;
    }

    double square_min_range = min_range * min_range;
    double square_max_range = max_range * max_range;

    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
    {
        const pcl::PointXYZ &p = *iter;
        double square_distance = p.x * p.x + p.y * p.y;

        if (square_min_range <= square_distance && square_distance <= square_max_range)
        {
            narrowed_scan.points.push_back(p);
        }
    }

    return narrowed_scan;
}

void scan_callback(const sensor_msgs::msg::PointCloud2::ConstPtr input)
{
    pcl::PointCloud<pcl::PointXYZ> scan;
    pcl::fromROSMsg(*input, scan);
    scan = removePointsByRange(scan, 0, MAX_MEASUREMENT_RANGE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    sensor_msgs::msg::PointCloud2 filtered_msg;

    // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
    if (voxel_leaf_size >= 0.1)
    {
        // Downsampling the velodyne scan using VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_grid_filter.setInputCloud(scan_ptr);
        voxel_grid_filter.filter(*filtered_scan_ptr);
        pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
    }
    else
    {
        pcl::toROSMsg(*scan_ptr, filtered_msg);
    }

    filtered_msg.header = input->header;
    filtered_points_pub->publish(filtered_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("points_downsampler");

    filtered_points_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);

    node->declare_parameter<double>("voxel_leaf_size", 2.0);
    voxel_leaf_size = node->get_parameter("voxel_leaf_size").get_value<double>();

    node->declare_parameter<std::string>("points_topic", "/points_raw");
    POINTS_TOPIC = node->get_parameter("points_topic").get_value<std::string>();

    node->declare_parameter<bool>("output_log", false);
    _output_log = node->get_parameter("output_log").get_value<bool>();

    if (_output_log)
    {
        filename = "points_downsampler.log";
        ofs.open(filename.c_str(), std::ios::out);
    }

    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(POINTS_TOPIC, 10, scan_callback);

    rclcpp::spin(node);

    if (_output_log)
    {
        ofs.close();
    }

    return 0;
}
