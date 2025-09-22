#ifndef EVELACTION_HPP_
#define EVELACTION_HPP_


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <thread>
#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <eigen3/Eigen/Core>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <vector>

class Elevation_Node : public rclcpp::Node
{
public:
    Elevation_Node();
    virtual ~Elevation_Node();

private:
    // 回调函数
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // 主处理线程
    void elevation_thread();
    
    // 辅助函数
    void shiftMapWithMove(std::vector<std::vector<double>>& map, int move_x, int move_y, int MAP_WIDTH, int MAP_HEIGHT);
    
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevation_map_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
    
    // 线程管理
    std::thread elevation_thread_; 
    
    // 数据同步
    std::mutex mutex_pointcloud_;
    std::mutex mutex_odom_;
    
    // 状态变量
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_point_cloud_;
    Eigen::Vector3d last_odom_pos_;
    Eigen::Vector2i grid_cell_movement_;
    
    // 配置参数
    double max_height_;
    double map_resolution_m_;
    int map_width_in_cells_;
    double downsample_leaf_size_;
    std::string pointcloud_frame_id_;
    double percentage_;  // 修正拼写(原prsentage_)
    bool downsample_;
    
    // 裁剪盒参数
    Eigen::Vector4f crop_box_min_pt_;
    Eigen::Vector4f crop_box_max_pt_;
    
    // 地图数据
    int total_grid_cells_;
    std::vector<std::vector<double>> elevation_grid_;
    std::vector<float> elevation_map_;  // 改名以避免与C标准库冲突(原map)
};

#endif // EVELACTION_HPP_