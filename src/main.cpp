


#include "PointCloud2ToDepth.h"




int main (int argc, char **argv)
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
*/
