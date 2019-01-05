#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>



using std::cout;
using std::endl;
using std::stringstream;
using std::string;

using namespace pcl;

unsigned int filesNum = 0;
unsigned int filesNum1 = 0;
bool saveCloud(false);
bool saveImage(false);

boost::shared_ptr<pcl::visualization::CloudViewer> viewer;

void cloudCB(const sensor_msgs::PointCloud2& input)
{
    //pcl::PointCloud<pcl::PointXYZRGBA> cloud; // with color
    pcl::PointCloud<pcl::PointXYZI> cloud;  //with intensity

    pcl::fromROSMsg(input, cloud); // sensor_msgs::PointCloud2 ----> pcl::PointCloud<T>
///////////////////////////测试点云数据开始/////////////////////////////////////////////////////////////////////////////////////
std::cout<<std::endl<<"start get pointCloud data"<<std::endl;
std::cout<<"cloud.size: "<<cloud.size()<<std::endl;







///////////////////////////测试点云数据结束/////////////////////////////////////////////////////////////////////////////////////
    if(! viewer->wasStopped()) viewer->showCloud(cloud.makeShared());
    
    if(saveCloud)
    {
        stringstream stream;
        stringstream stream1;
        
        char numstr[5];
        sprintf(numstr,"%04d",filesNum);  
        stream  << "/home/andyoyo/calib_ws/DATA/pcd/" << numstr << ".pcd";
        string filename = stream.str();
        //if(io::savePCDFile(filename, cloud, true) == 0)//第三个参数表示按二进制存储
        // if(io::savePCDFile(filename, cloud, false) == 0)
        if(io::savePCDFile(filename,input,Eigen::Vector4f::Zero(),Eigen::Quaternionf::Identity(),false ) == 0)
        {
            filesNum++;
            cout << filename<<" had saved."<<endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());

        saveCloud = false;

    }

}


void savecsv(const sensor_msgs::PointCloud2::ConstPtr& inputCloud)
{
    std::cout<<"stamp:   "<<inputCloud->header.stamp<<std::endl;
    std::cout<<"frame_id:"<<inputCloud->header.frame_id<<std::endl;

}


void imageCB(const sensor_msgs::ImageConstPtr& msg)
{

  cv::imshow("Show RgbImage", cv_bridge::toCvShare(msg,"rgb8")->image);
  cv::waitKey(10);
  if(saveImage)
  {
    stringstream stream1;
    char numstr1[5];
    sprintf(numstr1,"%04d",filesNum1);
    stream1 <<"/home/andyoyo/calib_ws/DATA/img/" << numstr1 <<".png";
    string filename1 = stream1.str();
    cv::imwrite(filename1,cv_bridge::toCvShare(msg)->image);
    saveImage = false;
    filesNum1++;
    cout << filename1 << " had Saved."<< endl;
  }
}



void keyboardEventOccured(const visualization::KeyboardEvent& event, void* nothing)
{
    if(event.getKeySym() == "space"&& event.keyDown())
    {
        saveCloud = true;
        saveImage = true;
    }
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer> createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("OpenNI viewer"));
    v->registerKeyboardCallback(keyboardEventOccured);

    return(v);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pcl_write");
    ros::NodeHandle nh;
    cv::namedWindow("Show RgbImage");
    cv::startWindowThread();
    cout<< "Press space to record point cloud to a file."<<endl;
   
    viewer = createViewer();

    image_transport::ImageTransport it(nh);

   
    
    image_transport::Subscriber img_sub = it.subscribe("/camera/image", 1, imageCB);

    ros::Subscriber pclpcd_sub = nh.subscribe("/velodyne_points", 1, cloudCB);
    ros::Subscriber pclcsv_sub = nh.subscribe("/velodyne_points", 1, savecsv);

    ros::Rate rate(30.0);

    while (ros::ok() && ! viewer->wasStopped())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

