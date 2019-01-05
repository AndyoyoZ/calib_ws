#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "config.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int count=0;
std::vector<PointT> checkerBoardPoints;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr checkerBoardPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZ>());
float x_max=0,x_min=0,y_max=0,y_min=0,z_max=0,z_min=0;
bool segment_flag=false;

std::string filepath;
std::string filenumber;
std::string filetype;
// Mutex: //
boost::mutex cloud_mutex;

struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
    checkerBoardPoints.push_back(current_point);
    count++;
    if(count==1)
    {
        x_max=current_point.x;
        x_min=current_point.x;
        y_max=current_point.y;
        y_min=current_point.y;
        z_max=current_point.z;
        z_min=current_point.z;
    }
    else //获取鼠标选择的点云的坐标范围
    {
        x_max=current_point.x>x_max?current_point.x:x_max;
        x_min=current_point.x<x_min?current_point.x:x_min;
        y_max=current_point.y>y_max?current_point.y:y_max;
        y_min=current_point.y<y_min?current_point.y:y_min;
        z_max=current_point.z>z_max?current_point.z:z_max;
        z_min=current_point.z<z_min?current_point.z:z_min;
    }
    /////////////////////////////////////////////////////////////////////////////////////////
    if(count>=4)//获取到四个点后进行点云分割
    {
        std::vector<int> indexs;
        for(auto iter=checkerBoardPoints.cbegin();iter!=checkerBoardPoints.cend();iter++)
        {
            std::cout<<"Point="<<*iter<<std::endl;
        }
        std::cout<<"x_max:"<<x_max<<std::endl;
        std::cout<<"x_min:"<<x_min<<std::endl;
        std::cout<<"y_max:"<<y_max<<std::endl;
        std::cout<<"y_min:"<<y_min<<std::endl;
        std::cout<<"z_max:"<<z_max<<std::endl;
        std::cout<<"z_min:"<<z_min<<std::endl;

        for(size_t i=0;i<tempcloud->points.size();i++)
        {
            if(tempcloud->points[i].x>(x_min-0.05)&&tempcloud->points[i].x<x_max+0.05
            &&tempcloud->points[i].y>(y_min-0.05)&&tempcloud->points[i].y<y_max+0.05
            &&tempcloud->points[i].z>(z_min-0.05)&&tempcloud->points[i].z<z_max+0.05)
            {
                // std::cout<<tempcloud->points[i]<<std::endl;
                indexs.push_back(i);//保存满足条件的点云索引
            }   
        }
        pcl::copyPointCloud(*tempcloud,indexs,*checkerBoardPointCloud);//按索引号拷贝点云
        //对分割后的点云着色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> 
        target_color(checkerBoardPointCloud, 255, 255, 0);
        data->viewerPtr->removePointCloud("checkerboardCloud");
        data->viewerPtr->addPointCloud(checkerBoardPointCloud,target_color,"checkerboardCloud");
        data->viewerPtr->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                           1, "checkerboardCloud");
        //将结果保存到PCD文件
        std::stringstream stream;
        stream  << filepath<<filenumber<<"_s"<<filetype;
        std::string savefilename = stream.str();
	    pcl::io::savePCDFileASCII(savefilename, *checkerBoardPointCloud);//将点云保存到PCD文件中
	    std::cerr << "Saved " << checkerBoardPointCloud->points.size() << " data points to "<<savefilename<< std::endl;
    }

    /////////////////////////////////////////////////////////////////////////////////////////
}

int main(int argc,char **argv)
{
    //从配置文件读取pcd文件路径
	const char ConfigFile[]= "../config/config.txt";   
    Config configSettings(ConfigFile);
    filepath = configSettings.Read("pcdFilePath",filepath);
    filenumber = configSettings.Read("pcdFileNumber",filenumber);
    filetype = configSettings.Read("pcdFileType",filetype);
    std::stringstream stream;
    stream  << filepath<<filenumber<<filetype;
    std::string filename = stream.str();
    // std::string filename="../data/0000.pcd";
    //visualizer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

    if (pcl::io::loadPCDFile(filename, *cloud))
    {
        std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
        return -1;
    }
    std::cout << cloud->points.size() << std::endl;
    pcl::copyPointCloud(*cloud,*tempcloud);
    cloud_mutex.lock();    // for not overwriting the point cloud

    // Display pointcloud:
    viewer->addPointCloud(cloud, "PointCloud");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;


    // Spin until 'Q' is pressed:
    viewer->spin();
    std::cout << "done." << std::endl;

    cloud_mutex.unlock();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}




