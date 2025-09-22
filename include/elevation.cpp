#include "elevation.hpp"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cmath>
#include <string>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>

Elevation_Node::Elevation_Node() : Node("elevation_node"),
    // --- 使用成员初始化列表初始化所有参数 ---
    max_height_(this->declare_parameter<double>("max_height", 1.0)),
    map_resolution_m_(this->declare_parameter<double>("map_resolution", 0.04)),
    map_width_in_cells_(this->declare_parameter<int>("map_width_in_cells", 51)),
    downsample_leaf_size_(this->declare_parameter<double>("downsample_leaf_size", 0.02)),
    pointcloud_frame_id_(this->declare_parameter<std::string>("out_frame_id", "odom")),
    percentage_(this->declare_parameter("height_percentage", 0.25)),  // 修正拼写
    downsample_(this->declare_parameter("downsample", false))
{
    RCLCPP_INFO(this->get_logger(), "Starting Elevation_Node initialization...");
    
    // 从参数服务器获取裁剪盒参数
    double x_min = this->declare_parameter("crop_box.x_min", -10.0);
    double y_min = this->declare_parameter("crop_box.y_min", -10.0);
    double z_min = this->declare_parameter("crop_box.z_min", -1.0);
    double x_max = this->declare_parameter("crop_box.x_max", 10.0);
    double y_max = this->declare_parameter("crop_box.y_max", 10.0);
    double z_max = this->declare_parameter("crop_box.z_max", 1.0);
    
    // 设置裁剪盒参数
    crop_box_min_pt_ << x_min, y_min, z_min, 1.0f;
    crop_box_max_pt_ << x_max, y_max, z_max, 1.0f;

    // 初始化状态变量
    last_odom_pos_.setZero();
    grid_cell_movement_.setZero();

    // 初始化地图数据结构
    total_grid_cells_ = map_width_in_cells_ * map_width_in_cells_;
    elevation_grid_.resize(total_grid_cells_);
    elevation_map_.resize(total_grid_cells_, -2.0);  // 默认值初始化为-2.0
    
    // 获取话题名称参数
    auto pointcloud_topic = this->declare_parameter<std::string>("pointcloud_in", "pointcloud_in");
    auto odom_topic = this->declare_parameter<std::string>("odom_in", "odom");
    auto elevation_map_topic = this->declare_parameter<std::string>("elevation_map_out", "elevation_map_cloud");
    auto ground_cloud_topic = this->declare_parameter<std::string>("ground_cloud_out", "ground_cloud");

    // 设置QoS策略
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // 创建订阅者
    sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic, sensor_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->pointcloud_callback(msg); }
    );
    
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->odom_callback(msg); }
    );

    // 创建发布者
    elevation_map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(elevation_map_topic, 10);
    ground_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ground_cloud_topic, 10);

    // 启动处理线程
    elevation_thread_ = std::thread(&Elevation_Node::elevation_thread, this);

    RCLCPP_INFO(this->get_logger(), "Elevation node has been started successfully.");
}

Elevation_Node::~Elevation_Node()
{
    // 安全停止线程
    if (elevation_thread_.joinable())
    {
        elevation_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "Elevation node has been shut down.");
}

void Elevation_Node::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 转换ROS消息为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // 应用裁剪盒过滤
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);
    crop_box.setMin(crop_box_min_pt_);
    crop_box.setMax(crop_box_max_pt_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    crop_box.filter(*cloud_filtered);
    
    // 如果启用了降采样则应用体素网格过滤
    if(downsample_)
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud_filtered);
        voxel_grid.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_grid.filter(*cloud_filtered_voxel);
        
        {
            std::lock_guard<std::mutex> lock(mutex_pointcloud_);
            latest_point_cloud_ = cloud_filtered_voxel;
        }
    }
    else
    {
        std::lock_guard<std::mutex> lock(mutex_pointcloud_);
        latest_point_cloud_ = cloud_filtered;
    }
}

void Elevation_Node::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 提取里程计消息中的位置
    Eigen::Vector3d odom_pos;
    Eigen::Vector2i move;
    odom_pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    
    // 检查移动是否超过一个平面体素大小
    double move_y = odom_pos.y() - last_odom_pos_.y();
    double move_x = odom_pos.x() - last_odom_pos_.x();
    
    if(std::abs(move_y) >= map_resolution_m_)
    {
        move.y() = std::round(move_y / map_resolution_m_);
        last_odom_pos_.y() = odom_pos.y();
    }
    
    if(std::abs(move_x) >= map_resolution_m_)
    {
        move.x() = std::round(move_x / map_resolution_m_);
        last_odom_pos_.x() = odom_pos.x();
    }
    
    // 线程安全地更新移动信息
    {
        std::lock_guard<std::mutex> lock(mutex_odom_);
        grid_cell_movement_ = move;
    }
}


void Elevation_Node::elevation_thread()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    Eigen::Vector2i move;
    int planarVoxelHalfWidth = (map_width_in_cells_-1) / 2;
    
    // 预计算常用值以避免重复计算
    float half_width = planarVoxelHalfWidth * map_resolution_m_;
    float half_resolution = map_resolution_m_ / 2.0f;

    while(rclcpp::ok())
    {
        // 安全获取最新数据
        {
            std::lock_guard<std::mutex> lock1(mutex_pointcloud_);
            std::lock_guard<std::mutex> lock2(mutex_odom_);
            cloud = latest_point_cloud_;
            move = grid_cell_movement_;
            latest_point_cloud_ = nullptr;
            grid_cell_movement_.setZero();
        }
        
        // 根据需要处理地图移动
        if((move.x() != 0 || move.y() != 0) && (!elevation_grid_.empty()))
        {
            shiftMapWithMove(elevation_grid_, move.x(), move.y(), map_width_in_cells_, map_width_in_cells_);
            move.setZero();
        }
        
        if(cloud != nullptr)
        {
            // 创建点云容器并预分配内存
            pcl::PointCloud<pcl::PointXYZ>::Ptr map_point(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>());
            
            // 预分配容量以避免动态重分配
            map_point->points.reserve(cloud->points.size());
            obstacleCloud->points.reserve(cloud->points.size());
            groundCloud->points.reserve(total_grid_cells_);
            
            // 处理点云以收集高程数据
            for(const auto& point : cloud->points)
            {
                // 跳过过高的点
                // if(point.z >= max_height_) continue;
                
                int grid_x = std::floor((point.x + half_width) / map_resolution_m_);
                int grid_y = std::floor((point.y + half_width) / map_resolution_m_);

                if(grid_x >= 0 && grid_x < map_width_in_cells_ && 
                   grid_y >= 0 && grid_y < map_width_in_cells_)
                {
                    int index = grid_x + grid_y * map_width_in_cells_;
                    elevation_grid_[index].push_back(point.z);
                    map_point->points.push_back(point);
                }
            }

            // 计算每个单元格的高程
            for(int i = 0; i < total_grid_cells_; i++)
            {
                if(!elevation_grid_[i].empty())
                {
                    std::sort(elevation_grid_[i].begin(), elevation_grid_[i].end());
                    int num = elevation_grid_[i].size();
                    int mid = static_cast<int>(num * percentage_);
                    elevation_map_[i] = elevation_grid_[i][mid];
                }
                else
                {
                    elevation_map_[i] = -2.0; // 无点则设为-2.0
                }
            }

            // 处理点以识别障碍物
            for(const auto& point : map_point->points)
            {
                int grid_x = std::floor((point.x + half_width) / map_resolution_m_);
                int grid_y = std::floor((point.y + half_width) / map_resolution_m_);
                
                if(grid_x >= 0 && grid_x < map_width_in_cells_ && 
                   grid_y >= 0 && grid_y < map_width_in_cells_)
                {
                    int index = grid_x + grid_y * map_width_in_cells_;
                    float height_diff = point.z - elevation_map_[index];
                    
                    // 高于地面但低于最大高度的点
                    if(height_diff >= -0.2f && height_diff <= max_height_)
                    {
                        pcl::PointXYZI obstacle_point;
                        obstacle_point.x = point.x;
                        obstacle_point.y = point.y;
                        obstacle_point.z = point.z;
                        obstacle_point.intensity = height_diff;
                        obstacleCloud->points.push_back(obstacle_point);
                    }
                }
            }

            // 生成地面表面点云
            for(int i = 0; i < total_grid_cells_; i++)
            {
                // 修正：检查-2.0而非-1.0，保持一致性
                // if(elevation_map_[i] != -2.0)
                // {
                    int grid_x = i % map_width_in_cells_;
                    int grid_y = i / map_width_in_cells_;
                    pcl::PointXYZ ground_point;
                    ground_point.x = (grid_x - planarVoxelHalfWidth) * map_resolution_m_ + half_resolution;
                    ground_point.y = (grid_y - planarVoxelHalfWidth) * map_resolution_m_ + half_resolution;
                    ground_point.z = elevation_map_[i];
                    groundCloud->points.push_back(ground_point);
                // }
            }

            // 发布地面点云
            sensor_msgs::msg::PointCloud2 ground_cloud_msg;
            pcl::toROSMsg(*groundCloud, ground_cloud_msg);
            ground_cloud_msg.header.frame_id = pointcloud_frame_id_;
            ground_cloud_msg.header.stamp = this->now();
            ground_cloud_pub_->publish(ground_cloud_msg);

            // 发布高程地图点云
            sensor_msgs::msg::PointCloud2 elevation_map_cloud_msg;
            pcl::toROSMsg(*obstacleCloud, elevation_map_cloud_msg);
            elevation_map_cloud_msg.header.frame_id = pointcloud_frame_id_;
            elevation_map_cloud_msg.header.stamp = this->now();
            elevation_map_cloud_pub_->publish(elevation_map_cloud_msg);
        }
    }
}



void Elevation_Node::shiftMapWithMove(std::vector<std::vector<double>>& map, int move_x, int move_y, int MAP_WIDTH, int MAP_HEIGHT)
{
    // 处理行移动（Y方向）
    if (move_y != 0) {
        int step = std::abs(move_y);
        bool shift_down = move_y < 0;
        
        int start = shift_down ? MAP_HEIGHT - 1 : 0;
        int end = shift_down ? step - 1 : MAP_HEIGHT - step;
        int dir = shift_down ? -1 : 1;
        int clear_start = shift_down ? 0 : MAP_HEIGHT - step;
        int clear_end = shift_down ? step : MAP_HEIGHT;
        
        std::cout << "地图" << (shift_down ? "向下" : "向上") << "移动 " << step << " 行.\n";
        
        // 移动已有数据
        for (int i = start; i != end; i += dir) {
            int source_row = shift_down ? i - step : i + step;
            for (int j = 0; j < MAP_WIDTH; ++j) {
                map[i * MAP_WIDTH + j] = std::move(map[source_row * MAP_WIDTH + j]);
            }
        }
        
        // 清空新暴露的行
        for (int i = clear_start; i < clear_end; ++i) {
            for (int j = 0; j < MAP_WIDTH; ++j) {
                map[i * MAP_WIDTH + j].clear();
            }
        }
    }
    
    // 处理列移动（X方向）
    if (move_x != 0) {
        int step = std::abs(move_x);
        bool shift_right = move_x < 0;
        
        int start = shift_right ? MAP_WIDTH - 1 : 0;
        int end = shift_right ? step - 1 : MAP_WIDTH - step;
        int dir = shift_right ? -1 : 1;
        int clear_start = shift_right ? 0 : MAP_WIDTH - step;
        int clear_end = shift_right ? step : MAP_WIDTH;
        
        std::cout << "地图" << (shift_right ? "向右" : "向左") << "移动 " << step << " 列.\n";
        
        // 移动已有数据
        for (int i = 0; i < MAP_HEIGHT; ++i) {
            for (int j = start; j != end; j += dir) {
                int source_col = shift_right ? j - step : j + step;
                map[i * MAP_WIDTH + j] = std::move(map[i * MAP_WIDTH + source_col]);
            }
            
            // 清空新暴露的列
            for (int j = clear_start; j < clear_end; ++j) {
                map[i * MAP_WIDTH + j].clear();
            }
        }
    }
}