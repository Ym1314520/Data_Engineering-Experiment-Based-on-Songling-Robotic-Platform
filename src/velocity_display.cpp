#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

void doMsg(const nav_msgs::Odometry::ConstPtr& msg)
{
    float linear_x=0; 
    float angular_z=0;
    linear_x = msg->twist.twist.linear.x;
    angular_z = msg->twist.twist.angular.z;    
    if(fabs(linear_x)<0.01)
    {
        linear_x=0;
    }
    if(fabs(angular_z)<0.01)
    {
        angular_z=0;
    }
    ROS_INFO("x_velocity：%.3f ", linear_x);
    ROS_INFO("z_angular：%.3f",angular_z);
   
}
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "velocity_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom",1000,doMsg);
    ros::spin();
    return 0;
}
