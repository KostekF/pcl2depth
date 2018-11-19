#ifndef __PointCloud2ToDepth_h
#define __PointCloud2ToDepth_h


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <pcl_ros/point_cloud.h> //supports message passing with PCL native data types. 
//This header allows you to publish and subscribe pcl::PointCloud<T> objects as ROS messages.
#include <pcl/common/projection_matrix.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include <string>
//#include<boost>
#include<camera_info_manager/camera_info_manager.h>


class PointCloud2ToDepth
{
public:
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
 
 PointCloud2ToDepth();
 
private:
  ros::NodeHandle nh_;
 // sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string depthImage_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
  
  cv::Mat depthImage_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_; //camera_info pointer
  ros::NodeHandle camera_nh_;
  ros::Publisher cam_info_pub_; //camera_info message publisher
 
};

#endif