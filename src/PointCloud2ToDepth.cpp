#include "PointCloud2ToDepth.h"

     void PointCloud2ToDepth::createDepthImage(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_cloud)
     {
         
        if (!depthImage_.empty())
            depthImage_.release();
        
        //nadanie pustemu obrazowi glebi wymiarow
        int depthImageHeight=ptr_cloud->height /4;// dzięki funkcji Multi-Echo otrzymujemy 4 echa, z czego korzystam z jednego, głównego  //SICK
        //int depthImageHeight=ptr_cloud->height; //KINECT
        int depthImageWidth=ptr_cloud->width;//924
        depthImage_.create(depthImageHeight,depthImageWidth,CV_32F);  

      
        
        const int m_to_mm=1000;
               
        //   #pragma omp parallel for //multithreading
        for(int v=0;v<depthImage_.rows;++v)
        
        {
            
            for(int u=0;u<depthImage_.cols;++u)
        
            {
                //float valOfDepthImagePixel=sqrt(pow(ptr_cloud->points.at(count).z,2)+pow(ptr_cloud->points.at(count).x,2)+pow(ptr_cloud->points.at(count).y,2)) *1000;
                float valOfDepthImagePixel=ptr_cloud->points.at(v*924 +u).x*m_to_mm;
           
                 if(valOfDepthImagePixel>0)
                {
                depthImage_.at<float>(v,u)=valOfDepthImagePixel; //przypisanie odleglosci do danego piksela obrazu glebi 
                }
                else
                    depthImage_.at<float>(v,u)=0;         
            } 
        }
        
        depthImage_.convertTo(depthImage_,CV_16U);
     }
  void PointCloud2ToDepth::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
  {
   
    try
    {
    
        //zamiana chmury z postaci PointCloud2 na pcl pointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*ptr_cloud);
    
    createDepthImage(ptr_cloud);

    //changing cv::Mat to sensor_msgs/Image
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    
    header.frame_id="virtual_camera";//delete?
    
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depthImage_);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
  
    
    sensor_msgs::CameraInfoPtr camInfoPtr(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo())); //get current cameraInfo data
    camInfoPtr->header.stamp=header.stamp;
    
    camInfoPtr->header.frame_id="virtual_camera"; //delete?
    image_pub_.publish(img_msg); 
 
    //publish camera_info
    cam_info_pub_.publish(camInfoPtr);
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to depth image message: "
                        << e.what());
    }
 
  }

    PointCloud2ToDepth::PointCloud2ToDepth() : cloud_topic_("cloud_in"),depthImage_topic_("depth_image"),camInfo_topic_("camera_info") 
  {
      ///DO i need two node handlers for two topics? or another node handle for camera info?
      
       
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloud2ToDepth::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (depthImage_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (depthImage_topic_);
     std::string camInfo_it = nh_.resolveName (camInfo_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
    
   ROS_INFO_STREAM("Publishing camera_info on topic " << camInfo_it );
    
    //creating camera_info for depth_nav_tools package
    cinfo_=std::make_shared<camera_info_manager::CameraInfoManager>(camera_nh_);
    cinfo_->setCameraName("fakeSickCamera");
    if(cinfo_->validateURL("package://pcl2depth/fakeCameraInfo.yaml"))
    {  
        std::cout<<"camera_info URL validated successfully"<<std::endl;
        if(cinfo_->loadCameraInfo("package://pcl2depth/fakeCameraInfo.yaml"))
        {
             std::cout<<"CameraInfo loaded successfully"<<std::endl;
        }
        
    }
    else
        std::cout<<"error validating url"<<std::endl;
    
     cam_info_pub_ = camera_nh_.advertise<sensor_msgs::CameraInfo> (camInfo_topic_, 30);
    
            
    
  }
