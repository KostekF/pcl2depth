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




class PointCloud2ToDepth
{
public:
  void
  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
  {
   
    try
    {
     // pcl::toROSMsg (*cloud, image_); //convert the cloud
        
         pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*ptr_cloud);
    
     if (!depthImage_.empty())
        depthImage_.release();
    
    
    int depthImageHeight=ptr_cloud->height /4;//96 shouldnt this be 24? 15degree/0.625=24
    int depthImageWidth=ptr_cloud->width;//924
     depthImage_.create(depthImageHeight,depthImageWidth,CV_32F);  //było cv32F
    
     int count =0;
     

     ///#pragma omp parallel for ---co to oznacza? sprawdz
    for(int i=0;i<depthImage_.rows;++i)
        for(int j=0;j<depthImage_.cols;++j)
        {
           
           float valOfDepthImagePixel=cbrt(pow(ptr_cloud->points.at(count).z,2)+pow(ptr_cloud->points.at(count).x,2)+pow(ptr_cloud->points.at(count).y,2)) *1000;
            
            if(valOfDepthImagePixel>0)
            {
            depthImage_.at<float>(i,j)=valOfDepthImagePixel;
            }
            else
                 depthImage_.at<float>(i,j)=0;
            
            
            ++count;
        }
    std::cout<<depthImage_.rows<<std::endl;
    depthImage_.convertTo(depthImage_,CV_16U);
        
    
    //changing cv::Mat to sensor_msgs/Image
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, depthImage_);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    image_pub_.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
            
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to depth image message: "
                        << e.what());
    }
    
    
    
   // image_pub_.publish (image_); //publish our cloud image
    
    
  }
  PointCloud2ToDepth () : cloud_topic_("sick_mrs6xxx/cloud"),depthImage_topic_("depthFromPcl")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloud2ToDepth::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (depthImage_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (depthImage_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }
private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string depthImage_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
  
  cv::Mat depthImage_;
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pcl2_to_depth");
  PointCloud2ToDepth pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}





/*
void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)

{
 // ROS_INFO("I heard: ");
   
   for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x"), iter_y(*cloud, "y"), iter_z(*cloud, "z"); iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
   {
    std::cout<<"x: "<<*iter_x<<std::endl;
    std::cout<<"y: "<<*iter_y<<std::endl;
    std::cout<<"z: "<<*iter_z<<std::endl;
    }
  
   
   

     //sensor_msgs::Image image;
    // pcl::toROSMsg(*cloud,image);
     //std::cout<<*image<<std::endl;
 
}

cv::Mat depthImage;

void pclToDepthCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)

{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*ptr_cloud);
   
    
     if (!depthImage.empty())
        depthImage.release();
    
    
    int depthImageHeight=ptr_cloud->height;//96
    int depthImageWidth=ptr_cloud->width;//924
     depthImage.create(depthImageHeight,depthImageWidth,CV_32F);  //było cv32F
    
     int count =0;
     
    for(int i=0;i<depthImage.rows;++i)
        for(int j=0;j<depthImage.cols;++j)
        {
            depthImage.at<float>(i,j)=ptr_cloud->points.at(count).z *1000;
            ++count;
        }
    
    depthImage.convertTo(depthImage,CV_8U);
    
    for(size_t i=0; i<ptr_cloud->points.size();++i)
    {
       
       float x = ptr_cloud->points[i].x;
        float y = ptr_cloud->points[i].y;
        float z = ptr_cloud->points[i+2].z;
      ///  std::cout<<pt_cloud->isOrganized()<<std::endl;
        std::cout<<x<<", "<<y<<", "<<z<<std::endl;
    }
   // 
    
  //  double depthVal=sqrt(pow(pt_cloud->points[i].x,2)+pow(pt_cloud->points[i].y,2)+pow(pt_cloud->points[i].z,2));
    // std::cout<<x<<", "<<y<<", "<<z<<std::endl;
    
        
   // }
   
   
   
    
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "pcl2depth");

 
  ros::NodeHandle n;

 
  ros::Subscriber sub = n.subscribe("sick_mrs6xxx/cloud", 1000, pclToDepthCallback);

 
  ros::spin();

  return 0;
}
*/

