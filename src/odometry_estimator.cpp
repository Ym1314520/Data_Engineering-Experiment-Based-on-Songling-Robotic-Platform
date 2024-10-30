#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
float distance = 0.0;
void doMsg(const nav_msgs::Odometry::ConstPtr& msg)
{
    float speed_x=0;    
    speed_x = msg->twist.twist.linear.x;    
    if(fabs(speed_x)<0.05)
    {
        speed_x=0;
    }
    ROS_INFO("X_速度：%.3f ", speed_x);
    distance += 0.1 * speed_x;
    ROS_INFO("里程计的距离为：%.6f",distance );
   
}
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "odom_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom",1000,doMsg);
    ros::spin();
    return 0;
}
