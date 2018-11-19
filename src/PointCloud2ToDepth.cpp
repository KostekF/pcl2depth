#include "PointCloud2ToDepth.h"


  void PointCloud2ToDepth::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
  {
   
    try
    {
    
        //zamiana chmury z postaci PointCloud2 na pcl pointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*ptr_cloud);
    
     if (!depthImage_.empty())
        depthImage_.release();
    
    //nadanie pustemu obrazowi glebi wymiarow
    int depthImageHeight=ptr_cloud->height /4;// dzięki funkcji Multi-Echo otrzymujemy 4 echa, z czego korzystam z jednego, głównego
    int depthImageWidth=ptr_cloud->width;//924
    depthImage_.create(depthImageHeight,depthImageWidth,CV_32F);  
    
     int count =0;
     

  //   #pragma omp parallel for //multithreading

    for(int i=0;i<depthImage_.rows;++i)
        for(int j=0;j<depthImage_.cols;++j)
        {
           //Wartosc valOfDepthImagePixel czyli odleglosc od przeszkody w danym pikselu obrazu jest dana w mm
           float valOfDepthImagePixel=sqrt(pow(ptr_cloud->points.at(count).z,2)+pow(ptr_cloud->points.at(count).x,2)+pow(ptr_cloud->points.at(count).y,2)) *1000;
          
            if(valOfDepthImagePixel>0)
            {
            depthImage_.at<float>(i,j)=valOfDepthImagePixel; //przypisanie odleglosci do danego piksela obrazu glebi 
            }
            else
                 depthImage_.at<float>(i,j)=0;
            
            
            ++count;
        }
  //  std::cout<<depthImage_.rows<<std::endl;
    depthImage_.convertTo(depthImage_,CV_16U);
        
    
    //changing cv::Mat to sensor_msgs/Image
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, depthImage_);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      image_pub_.publish(img_msg); 
            
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to depth image message: "
                        << e.what());
    }
    
  
    
    
    
  }

 
 PointCloud2ToDepth::PointCloud2ToDepth() : cloud_topic_("sick_mrs6xxx/cloud"),depthImage_topic_("depthFromPcl")
  {
      ///DO i need two node handlers for two topics? or another node handle for camera info?
      
       
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloud2ToDepth::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (depthImage_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (depthImage_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
    
  
    
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
    
   //  cam_info_pub_ = camera_nh_.advertise<sensor_msgs::Image> (depthImage_topic_, 30);
    
            
    
  }