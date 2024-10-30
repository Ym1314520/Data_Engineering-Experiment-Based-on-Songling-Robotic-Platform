#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>  
#include <cmath>

// 定义一个结构体来表示数据a,b
struct Odom {  
    float a;  
    float b;  
};

// 定义一个结构体来表示点  
struct Point {  
    float x;  
    float y;  
}; 

float slope = 1.0, intercept = 0.0; 

// 计算线性拟合的斜率和截距  
void linearFit(const std::vector<Point>& points, float& slope, float& intercept) 
{  
    float sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXSquare = 0.0;  
    int n = points.size();  
  
    for (const auto& point : points) {  
        sumX += point.x;  
        sumY += point.y;  
        sumXY += point.x * point.y;  
        sumXSquare += point.x * point.x;  
    }  
  
    // 计算均值
    float xMean = sumX / n;  
    float yMean = sumY / n;  
  
    // 计算斜率和截距
    slope = (n * sumXY - sumX * sumY) / (n * sumXSquare - sumX * sumX);  
    intercept = yMean - slope * xMean;  
} 

float distance = 0.0;

void doCount(const nav_msgs::Odometry::ConstPtr& msg)
{
    float speed_x=0;    
    speed_x = msg->twist.twist.linear.x;  

    if(fabs(speed_x)<0.05)
    {
        speed_x=0;
    }
    ROS_INFO("X_速度：%.3f ", speed_x);

    distance += 0.1 * speed_x;
    ROS_INFO("里程计的距离为：%.6f",distance);

    float calculate = (distance-intercept)/slope;
    ROS_INFO("建立模型后的计算值为：%.6f",calculate);
}

int main(int argc, char  *argv[])
{
    // n=1时的测试数据
    std::vector<Odom> odoms_1 = {
        {2.813656,3.734071}, {9.151440,10.082789},
        {10.972377,11.867435}, {11.867435,12.7972540},
        {12.7972540,13.754729}
    };

    // n=2时的测试数据
    std::vector<Odom> odoms_2 = {
        {3.734071,5.202457}, {13.770550,15.839819},
        {15.839819,17.740211}, {17.740211,19.768019},
        {19.768019,21.663891}
    };

    // n=3时的测试数据
    std::vector<Odom> odoms_3 = {
        {5.202457,7.881470}, {7.881470,10.557323},
        {13.501240,16.332977}, {16.332977,18.804277},
        {18.804277,21.663891}
    };

    // 计算并定义点集 
    std::vector<Point> points;
    for(int i=0;i<odoms_1.size();i++){
        float y = odoms_1[i].b-odoms_1[i].a;
        points.push_back({1,y});
    }
    for(int i=0;i<odoms_2.size();i++){
        float y = odoms_2[i].b-odoms_2[i].a;
        points.push_back({2,y});
    }
    for(int i=0;i<odoms_3.size();i++){
        float y = odoms_3[i].b-odoms_3[i].a;
        points.push_back({3,y});
    }
  
    // 进行线性拟合  
    linearFit(points, slope, intercept);  

    setlocale(LC_ALL,"");
    
    // 初始化节点名称 
    ros::init(argc, argv, "odom_calculate");
    ros::NodeHandle nh;

    // 声明订阅者，创建一个订阅者sub
    ros::Subscriber sub = nh.subscribe("/odom",1000,doCount);

    // 用于调用后台函数，等待接收消息。在接收到消息时执行后台函数。
    ros::spin();

    return 0;
}