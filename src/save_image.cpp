#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <vector>
#include <string>
#include <sstream>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_color;
  image_transport::Subscriber image_sub_depth;
  int frame_count_color_;
  int frame_count_depth_;
  ros::Time last_save_time_color_; // Time for saving color image
  ros::Time last_save_time_depth_; // Time for saving depth image
  double save_frequency_; // Save frequency (2Hz)

public:
  ImageConverter(const std::string& color_topic, const std::string& depth_topic, double frequency)
    : it_(nh_), frame_count_color_(0), frame_count_depth_(0), save_frequency_(frequency)
  {
    image_sub_color = it_.subscribe(color_topic, 1, &ImageConverter::imageCb, this);
    image_sub_depth = it_.subscribe(depth_topic, 1, &ImageConverter::depthCb, this);
    last_save_time_color_ = ros::Time::now();
    last_save_time_depth_ = ros::Time::now();
    boost::filesystem::create_directories("./data/color_image");
    boost::filesystem::create_directories("./data/depth_image");
  }

  // Callback function for color images
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Only save color image at the defined frequency (2Hz)
    if (ros::Time::now() - last_save_time_color_ >= ros::Duration(1.0 / save_frequency_))
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // Generate filename for the color image
      std::stringstream ss;
      ss << "./data/color_image/color_frame_" << frame_count_color_++ << ".png";
      std::string filename = ss.str();

      // Save the color image
      cv::imwrite(filename, cv_ptr->image);
      ROS_INFO("Saved color image: %s", filename.c_str());

      last_save_time_color_ = ros::Time::now(); // Update the last save time for color
    }
  }

  // Callback function for depth images
  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Only save depth image at the defined frequency (2Hz)
    if (ros::Time::now() - last_save_time_depth_ >= ros::Duration(1.0 / save_frequency_))
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // Normalize the depth image for saving
      cv::Mat depth_image_normalized;
      cv::normalize(cv_ptr->image, depth_image_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

      // Generate filename for the depth image
      std::stringstream ss;
      ss << "./data/depth_image/depth_frame_" << frame_count_depth_++ << ".png";
      std::string filename = ss.str();

      // Save the depth image
      cv::imwrite(filename, depth_image_normalized);
      ROS_INFO("Saved depth image: %s", filename.c_str());

      last_save_time_depth_ = ros::Time::now(); // Update the last save time for depth
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  std::string color_topic = "/camera/color/image_raw";
  std::string depth_topic = "/camera/depth/image_rect_raw";
  double save_frequency = 2.0; // 2Hz saving frequency

  if (argc == 4)
  {
    color_topic = argv[1];
    depth_topic = argv[2];
    save_frequency = atof(argv[3]);
  }

  ImageConverter ic(color_topic, depth_topic, save_frequency);

  // Start the ROS spinning
  ros::spin();

  return 0;
}

