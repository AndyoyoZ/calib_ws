#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <math.h>
#include <fstream>
using namespace std;


int filesNum = 0;


float calcu_distance_m(float x,float y,float z)
{
    return(sqrt(x*x+y*y+z*z));
}

void save_csv(const char* filename,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  std::ofstream outfile;
  outfile.open(filename);//打开文件
  
  /////////写文件头
  outfile<<"\"Points_m_XYZ:0\",\"Points_m_XYZ:1\",\"Points_m_XYZ:2\",\"intensity\",\"laser_id\",\"azimuth\",\"distance_m\",\"adjustedtime\",\"timestamp\""<<"\n";
  
  int intensity=15;
  int laser_id;
  int azimuth=0;
  float distance_m;
  int64_t adjustedtime=315902177;
  int64_t timestamp   =315902177;
  for(size_t i=0;i<cloud->points.size();++i)
  {
    laser_id = i;
    distance_m = calcu_distance_m(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
    outfile <<cloud->points[i].x<<","<<cloud->points[i].y<<","<<cloud->points[i].z<<","
            <<intensity<<","<<laser_id<<","<<azimuth<<","<<distance_m<<","<<adjustedtime<<","<<timestamp<<"\n";
  }
  
  outfile.close();
  std::cout<<".csv file saved."<<std::endl;

}



int main (int argc, char **argv)
{
  ros::init (argc, argv, "pcd2csv");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  ros::Rate loop_rate(30);
  while (ros::ok()&& !viewer.wasStopped())
  {

　  std::stringstream stream;
    char numstr[5];
    sprintf(numstr,"%04d",filesNum);
    stream  << "/home/andyoyo/calib_ws/DATA/pcd/" << numstr << ".pcd";
    std::string filename = stream.str();
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud)==-1)//读取pcd文件
    {
        std::cout << "Cloud reading failed." << std::endl;
	    return (-1);
    }
    std::cout <<"cloud->width :"<< cloud->width << std::endl;
    std::cout <<"cloud->height:"<< cloud->height<< std::endl;
    viewer.showCloud(cloud);
    std::stringstream stream1;
    stream1<< "/home/andyoyo/calib_ws/DATA/pcd/" << numstr << ".csv";
    std::string filename1 = stream1.str();
    std::ofstream outfile;
    const char* filename_csv=filename1.c_str();
    save_csv(filename_csv,cloud);//将点云数据保存为.csv格式
　　 filesNum++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


	

