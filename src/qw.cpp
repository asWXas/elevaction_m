#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
// odometry 头文件
// 引入 PCL 的 CropBox 滤波器
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <thread>
#include <chrono>
#include <nav_msgs/msg/odometry.hpp>


class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor() : Node("pointcloud_processor")
    {
        // 订阅话题 名称
        std::string cloud_topic = this->declare_parameter<std::string>("cloud_topic", "/livox/lidar");
        std::string odometry_topic = this->declare_parameter<std::string>("odometry_topic", "/Odometry");
        std::string output_topic = this->declare_parameter<std::string>("output_topic", "/cropped_cloud");
        output_topic_frame = this->declare_parameter<std::string>("output_topic_frame", "livox_frame");

        // 裁减的点云的长宽高
        double point_min_x = this->declare_parameter<double>("point_min_x", -4.0);
        double point_max_x = this->declare_parameter<double>("point_max_x", 4.0);
        double point_min_y = this->declare_parameter<double>("point_min_y", -4.0);
        double point_max_y = this->declare_parameter<double>("point_max_y", 4.0);
        double point_min_z = this->declare_parameter<double>("point_min_z", -1.0);
        double point_max_z = this->declare_parameter<double>("point_max_z", 1.0);

        min_point << point_min_x, point_min_y, point_min_z, 1.0;
        max_point << point_max_x, point_max_y, point_max_z, 1.0;

        // 体素滤波器 大小
        // 是否 降采样
        downsample = this->declare_parameter<bool>("downsample", true);
        voxel_size_ = this->declare_parameter<double>("voxel_size", 0.01);

        // 2.5D 地图的相关参数
        planarVoxelSize = this->declare_parameter<double>("planar_voxel_size", 0.05); // 2.5D 地图的体素大小
        planarVoxelWidth = this->declare_parameter<int>("planar_voxel_width", 51); // 2.5D 地图整体的宽度
        maxObstacleHeight = this->declare_parameter<double>("max_obstacle_height", 1.5);// 最大障碍物高度
        maxGroundHeight = this->declare_parameter<double>("max_ground_height", 0.1);// 最大地面高度
        planarVoxelHalfWidth = (planarVoxelWidth-1) / 2;
        kPlanarVoxelNum = planarVoxelWidth * planarVoxelWidth;

        map  = std::vector<float>(kPlanarVoxelNum, -1);                       // 2.5D 地图数据
        voxelMap = std::vector<std::vector<float>>(kPlanarVoxelNum);          // 体素地图数据

        // 声明订阅者，订阅 /input_cloud 话题的点云消息
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic , 10, std::bind(&PointCloudProcessor::cloud_callback, this, std::placeholders::_1));
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 10, std::bind(&PointCloudProcessor::odom_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        publisher_obstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/groud_cloud", 10);
        RCLCPP_INFO(this->get_logger(), "Point cloud processor node has started.");

        thread_ = std::thread(&PointCloudProcessor::task, this);
        thread_.detach();
    }

private:

    void task()
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_task;
        Eigen::Vector3d last_position;
        Eigen::Vector3d current_position;
        last_position << 0, 0, 0;
        current_position << 0, 0, 0;
        int move_x = 0;
        int move_y = 0;

        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(cloud.has_value())
                {
                    pcl_task = cloud.value();
                    cloud.reset();
                }
            }

            {
                std::lock_guard<std::mutex> lock(mutex_odom_);
                current_position = position;
            }

            {
                move_x = static_cast<int>((current_position[0] - last_position[0])/planarVoxelSize);
                move_y = static_cast<int>((current_position[1] - last_position[1])/planarVoxelSize);
                if((move_x != 0 || move_y != 0) &&!voxelMap.empty())
                {
                    // 移动map 数据： 以中心点为原点，反向移动 move_x, move_y 步长
                    std::cout << "move_x: " << move_x << " move_y: " << move_y << std::endl;
                    shiftMapWithMove(voxelMap, move_x, move_y, planarVoxelWidth, planarVoxelWidth);
                    if(move_x!=0)
                    {
                        last_position[0] = current_position[0];
                    }
                    if(move_y!=0)
                    {
                        last_position[1] = current_position[1];
                    }
                }
            }


            if(pcl_task)
            {
                for(const auto& point : pcl_task->points)
                {
                    int ix = static_cast<int>((point.x / planarVoxelSize) + planarVoxelHalfWidth);
                    int iy = static_cast<int>((point.y / planarVoxelSize) + planarVoxelHalfWidth);

                    if (ix < 0 || ix >= planarVoxelWidth || iy < 0 || iy >= planarVoxelWidth)
                    {
                        continue;
                    }
                    else
                    {
                        int index = ix + iy * planarVoxelWidth;
                        voxelMap[index].push_back(point.z);
                    }
                }

                for(int i = 0; i < kPlanarVoxelNum; i++)
                {
                    if(voxelMap[i].empty())
                    {
                        map[i] = -1;
                    }
                    else
                    {
                        // 密度分布，判断地面高度
                        std::sort(voxelMap[i].begin(), voxelMap[i].end());
                        int num = voxelMap[i].size();
                        int mid = static_cast<int>(num * 0.25);
                        float height = voxelMap[i][mid];
                        map[i] = height;
                    }
                }

                // 点云容器
                pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZI>());
                for(const auto& point : pcl_task->points)
                {
                    int ix = static_cast<int>((point.x / planarVoxelSize) + planarVoxelHalfWidth);
                    int iy = static_cast<int>((point.y / planarVoxelSize) + planarVoxelHalfWidth);

                    if (ix < 0 || ix >= planarVoxelWidth || iy < 0 || iy >= planarVoxelWidth)
                    {
                        continue;
                    }
                    else
                    {
                        int index = ix + iy * planarVoxelWidth;
                        float height = map[index];
                        float absth = point.z - height;
                        if(maxGroundHeight<absth < maxObstacleHeight)
                        {
                            pcl::PointXYZI obstaclePoint;
                            obstaclePoint.x = point.x;
                            obstaclePoint.y = point.y;
                            obstaclePoint.z = point.z;
                            obstaclePoint.intensity = absth;
                            obstacleCloud->points.push_back(obstaclePoint);
                        }
                    }
                }

                // 构建同样方格距离的点云，发布地面点云
                pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>());
                for(int i = 0; i < kPlanarVoxelNum; i++)
                {
                    if(map[i] != -1)
                    {
                        float x = (i % planarVoxelWidth - planarVoxelHalfWidth) * planarVoxelSize;
                        float y = (i / planarVoxelWidth - planarVoxelHalfWidth) * planarVoxelSize;
                        pcl::PointXYZ point;
                        point.x = x;
                        point.y = y;
                        point.z = map[i];
                        groundCloud->points.push_back(point);
                    }
                }

                // 发布地面点云
                sensor_msgs::msg::PointCloud2 ground_msg;
                pcl::toROSMsg(*groundCloud, ground_msg);
                
                // header
                ground_msg.header.frame_id = output_topic_frame;
                publisher_->publish(ground_msg);

                // 发布障碍物点云
                sensor_msgs::msg::PointCloud2 obstacle_msg;
                pcl::toROSMsg(*obstacleCloud, obstacle_msg);
                
                // header
                obstacle_msg.header.frame_id = output_topic_frame;
                publisher_->publish(obstacle_msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));

            }
        }
    }

    void shiftMapWithMove(std::vector<std::vector<float>>& map, int move_x, int move_y, int MAP_WIDTH, int MAP_HEIGHT) {
        
        // --- 第一步：处理 Y 方向的滚动 (上下移动行) ---

        // 车辆向上移动 (move_y < 0)，地图数据向下滚动
        if (move_y < 0) {
            int step = -move_y;
            std::cout << "Shifting map DOWN by " << step << " rows.\n";
            // 从下往上遍历，避免在移动过程中覆盖还需使用的源数据
            for (int i = MAP_HEIGHT - 1; i >= step; --i) {
                for (int j = 0; j < MAP_WIDTH; ++j) {
                    map[i * MAP_WIDTH + j] = std::move(map[(i - step) * MAP_WIDTH + j]);
                }
            }
            // 清空顶部新出现的行 (这些行的旧数据已被移走，现在是空状态)
            for (int i = 0; i < step; ++i) {
                for (int j = 0; j < MAP_WIDTH; ++j) {
                    map[i * MAP_WIDTH + j].clear();
                }
            }
        }
        // 车辆向下移动 (move_y > 0)，地图数据向上滚动
        else if (move_y > 0) {
            int step = move_y;
            std::cout << "Shifting map UP by " << step << " rows.\n";
            // 从上往下遍历
            for (int i = 0; i < MAP_HEIGHT - step; ++i) {
                for (int j = 0; j < MAP_WIDTH; ++j) {
                    map[i * MAP_WIDTH + j] = std::move(map[(i + step) * MAP_WIDTH + j]);
                }
            }
            // 清空底部新出现的行
            for (int i = MAP_HEIGHT - step; i < MAP_HEIGHT; ++i) {
                for (int j = 0; j < MAP_WIDTH; ++j) {
                    map[i * MAP_WIDTH + j].clear();
                }
            }
        }

        // --- 第二步：处理 X 方向的滚动 (左右移动列) ---

        // 车辆向左移动 (move_x < 0)，地图数据向右滚动
        if (move_x < 0) {
            int step = -move_x;
            std::cout << "Shifting map RIGHT by " << step << " columns.\n";
            // 遍历每一行
            for (int i = 0; i < MAP_HEIGHT; ++i) {
                // 从右往左遍历列，避免覆盖源数据
                for (int j = MAP_WIDTH - 1; j >= step; --j) {
                    map[i * MAP_WIDTH + j] = std::move(map[i * MAP_WIDTH + (j - step)]);
                }
                // 清空左侧新出现的列
                for (int j = 0; j < step; ++j) {
                    map[i * MAP_WIDTH + j].clear();
                }
            }
        }
        // 车辆向右移动 (move_x > 0)，地图数据向左滚动
        else if (move_x > 0) {
            int step = move_x;
            std::cout << "Shifting map LEFT by " << step << " columns.\n";
            // 遍历每一行
            for (int i = 0; i < MAP_HEIGHT; ++i) {
                // 从左往右遍历列
                for (int j = 0; j < MAP_WIDTH - step; ++j) {
                    map[i * MAP_WIDTH + j] = std::move(map[i * MAP_WIDTH + (j + step)]);
                }
                // 清空右侧新出现的列
                for (int j = MAP_WIDTH - step; j < MAP_WIDTH; ++j) {
                    map[i * MAP_WIDTH + j].clear();
                }
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_odom_);
            position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        // 1. 将 ROS 2 PointCloud2 消息转换为 PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // 2. 应用 CropBox 滤波器
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(pcl_cloud);

        // 设置裁剪盒子的边界
        // 定义裁剪盒子的最小和最大点
        // 这里以机器人为中心，裁剪一个 4x4x1 米的区域

        crop_box_filter.setMin(min_point);
        crop_box_filter.setMax(max_point);
        crop_box_filter.filter(*pcl_cloud_cropped);

        if(downsample)
        {
            // 4. 体素滤波器
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
            voxel_grid_filter.setInputCloud(pcl_cloud_cropped);
            voxel_grid_filter.setLeafSize(voxel_size_,voxel_size_,voxel_size_);
            voxel_grid_filter.filter(*pcl_cloud_cropped);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            cloud = pcl_cloud_cropped;
        }  
    }

    std::mutex mutex_;
    std::mutex mutex_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_obstacle_;
    std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud;
    std::string output_topic_frame;
    std::thread thread_;

    // 裁减盒子的边界
    Eigen::Vector4f min_point;
    Eigen::Vector4f max_point;

    // 体素滤波器的大小
    double voxel_size_;


    // 2.5D 地图的相关参数
    float planarVoxelSize ;
    int planarVoxelWidth; // map 宽度
    float maxObstacleHeight; // 障碍物的最大高度
    float maxGroundHeight; // 地面高度
    int planarVoxelHalfWidth;
    int kPlanarVoxelNum; 
    bool downsample;

    // 位置信息
    Eigen::Vector3d position;
    // 容器指针
    std::vector<std::vector<float>> voxelMap;
    std::vector<float> map;
    bool is_first_frame;

    // tf 变换 
    // xyz 
    Eigen::Vector4d transform_imu_to_baselink;
     


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
