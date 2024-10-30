#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ctime>
using namespace std;

ros::Time last_saved_time;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // 声明存储原始数据与滤波后的数据的点云的格式
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // 转化为PCL中的点云的数据格式
    pcl_conversions::toPCL(*input, *cloud);



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
    cloud1.reset (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud1);

    // 保存点云到文件，每0.5秒（2Hz）保存一次
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_saved_time).toSec() >= 0.5)
    {
        std::time_t now = std::time(0);
        char buf[80];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
        std::string filename = "./data/lidar/pointcloud_" + std::string(buf) + ".pcd";
        pcl::io::savePCDFileASCII(filename, *cloud1);
        ROS_INFO_STREAM("Saved point cloud to " << filename);
        last_saved_time = current_time;
    }
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl");
    ros::NodeHandle nh;

    // 获取参数
    std::string input_topic;
    nh.param<std::string>("input_topic", input_topic, "/rslidar_points");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe(input_topic, 10, cloud_cb);
    // 初始化最后保存时间
    last_saved_time = ros::Time::now();

    // Spin
    ros::spin();

    return 0;
}

