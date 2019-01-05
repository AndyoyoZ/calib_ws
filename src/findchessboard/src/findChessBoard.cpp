#include <iostream>
#include <config.h>
#include <pcl/io/pcd_io.h>//pcd输入输出
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <boost/thread/thread.hpp>//多线程
#include <pcl/common/common_headers.h>
#include <pcl/visualization/range_image_visualizer.h>//深度图可视化的头文件
#include <pcl/visualization/pcl_visualizer.h>//PCL可视化的头文件
#include <pcl/console/parse.h>//命令行参数解析
#include <opencv2/opencv.hpp>
#include <quickSort.h>
#include <convertion.h>

#define PatSize_width  9 //定义棋盘格角点数 m
#define PatSize_height 7

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
std::string filepath;
std::string filenumber;
std::string filetype;

cv::Size PatSize=cv::Size(PatSize_width,PatSize_height);
double camD[9];
double distCoeffD[5];
cv::Mat cameraMatrix=(cv::Mat_<double>(3,3)<<520.9,0,325.8,0,521.7,250.8,0,0,1);//相机内参
cv::Mat distortionCoefficients;
std::vector<PointT> chessboardPoints3d_pcl;
std::vector<cv::Point3f>   chessboardPoints3d_cv;


int main()
{
    //从配置文件读取png文件路径
	const char ConfigFile[]= "../config/config.txt";   
    Config configSettings(ConfigFile);
    
    filenumber = configSettings.Read("pcdFileNumber",filenumber);
    filepath = configSettings.Read("pngFilePath",filepath);
    filetype = configSettings.Read("pngFileType",filetype);
    std::stringstream stream,stream1;
    stream<<filepath<<filenumber<<filetype;
    std::string pngfilename = stream.str();
    
    filepath = configSettings.Read("pcdFilePath",filepath);
    filetype=configSettings.Read("pcdFileType",filetype);
    stream1<<filepath<<filenumber<<filetype;
    std::string pcdfilename = stream1.str();

	cv::Mat srcimage;
	cv::Mat grayimage;
	std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> checkerpointcloud;


	srcimage = cv::imread(pngfilename);
    if(!srcimage.data)
    {
        std::cerr<<"can not open"<<pngfilename<<std::endl;
    }
    cv::imshow("srcImage", srcimage);
    if(!cv::findChessboardCorners(srcimage, PatSize, corners,cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE))
    {
        std::cout<<"can not find chessboard corners..."<<std::endl;
        cv::waitKey(0);
        return -1;
    }
    else
    {
        cv::Mat chessBoardGray;
        cv::cvtColor(srcimage,chessBoardGray,cv::COLOR_RGB2GRAY);
        //亚像素精确化
        cv::cornerSubPix(chessBoardGray,corners,cv::Size(5,5),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        //cv::find4QuadCornerSubpix(chessBoardGray,corners,cv::Size(9,9));
        cv::drawChessboardCorners(chessBoardGray, PatSize, cv::Mat(corners), true);
	    cv::imshow("chessboard corners", chessBoardGray);
        cv::waitKey(30);
        // for(size_t corners_count=0;corners_count<corners.size();corners_count++)
        // {
        //     std::cout<<"corner_"<<corners_count<<":"<<corners.at(corners_count)<<std::endl;//按行存储
        // }
        
        // std::stringstream stream２;
        // stream２  << filepath<<filenumber<<"_c.png"; 
        // std::string savefilename = stream2.str();
        // cv::imwrite(savefilename,chessBoardGray);

        //加载点云文件
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
        //添加坐标系，红色是X轴，绿色是Y轴，蓝色是Z轴
        viewer->addCoordinateSystem(); 

        if (pcl::io::loadPCDFile(pcdfilename, *cloud))
        {
            std::cerr << "ERROR: Cannot open file " << pcdfilename << "! Aborting..." << std::endl;
            return -1;
        }
        // std::cout<<"cloud->points.size():"<<cloud->points.size()<<std::endl;
        cv::Mat pointcloud_mat;
        pointCloudPtr2Mat(cloud,pointcloud_mat);//pcl::PointCloud<pcl::PointXYZ>::Ptr转cv::Mat
        // std::cout<<"pointcloud_mat:\n"<<pointcloud_mat<<std::endl;
        // QuickSort(pointcloud_mat,0,pointcloud_mat.rows-1,1);
        // QuickSort(pointcloud_mat,0,pointcloud_mat.rows-1,2);

        pcl::RangeImage range_image;
        pointCloud2rangeImage(*cloud,range_image);//pcd点云转深度图
        // std::cout<<"range_image:\n"<<range_image<<std::endl;//输出深度图信息

        int width=range_image.width;
        int height=range_image.height;
        int width_step=(int)(0.5+width/(PatSize_width+1));//四舍五入取整
        int height_step=(int)(0.5+height/(PatSize_height+1));

        int count1=1,count2=0;
        size_t distcount=0;
        bool flag1=false,flag2=false;  
        cv::Vec6f line_para;
        cv::Mat pointcloud3d_mat=cv::Mat::zeros(cv::Size(PatSize_width,PatSize_height),CV_32FC3);
                PointCloudT::Ptr showCloudP;
        PointCloudT showCloud ;
        PointT showPoint;

        for(size_t i=0;i<width;i++)
        {
            std::vector< my3dPointType > boardPoints;
            std::vector< cv::Point3f > z_linePoints;
            if(i==width_step*count1)
                {
                    flag1=true;
                    count1++;
                }
            for(size_t j=0;j<height;j++)
            {           
                if(range_image.isValid(i,j)&&flag1)//&&(j-distcount>height_step/2)判断目标位置的点是否有限
                {   distcount=j; 
                    Eigen::Vector3f vectorPoint;
                    cv::Point3f cvPoint;               
                    range_image.calculate3DPoint(i,j,vectorPoint);//根据深度图计算3D点
                    // std::cout<<i<<","<<j<<std::endl;
                    // std::cout<<"vectorPoint:\n"<<vectorPoint<<std::endl;
                    Vector3f2cvPoint(vectorPoint,cvPoint);//3D点类型转为cv::Point3f
                    // z_linePoints.push_back(cvPoint);
                    my3dPointType boardPoint;
                    boardPoint.PointIndex.x=i;
                    boardPoint.PointIndex.y=j;
                    boardPoint.PointData=cvPoint;
                    boardPoints.push_back(boardPoint);
                }
            }


            flag1=false;
            // if(z_linePoints.size()>=3)//大于３个点才拟合直线
            if(boardPoints.size()>=2)//大于2个点才拟合直线
            {
                cv::Point3f endPoint1,endPoint2;               
                double x_step,y_step,z_step;
                //计算端点和步长
                my3dPointType endPoint1withIndex,endPoint2withIndex;
                getEndPoints(boardPoints,height,endPoint1withIndex,endPoint2withIndex);
                endPoint1=endPoint1withIndex.PointData;
                endPoint2=endPoint2withIndex.PointData;

                // std::cout<<"count1-2:"<<count1 -2<<std::endl;               
                // std::cout<<"endPoint1:"<<endPoint1<<std::endl;
                // std::cout<<"endPoint2:"<<endPoint2<<std::endl;

                x_step=(endPoint2.x-endPoint1.x)/(PatSize_height+1);
                y_step=(endPoint2.y-endPoint1.y)/(PatSize_height+1);
                z_step=(endPoint2.z-endPoint1.z)/(PatSize_height+1);

                // std::cout<<" x_step: "<<x_step<<" y_step: "<<y_step<<" z_step: "<<z_step<<std::endl;
                // cvPoint2pclPoint(endPoint1,showPoint);
                // showCloud.push_back(showPoint);
                // cvPoint2pclPoint(endPoint2,showPoint);
                // showCloud.push_back(showPoint);

                //按步长计算棋盘格角点对应的3D点坐标
                for(size_t j=0;j<PatSize_height;j++)
                {
                    cv::Point3f ccc=cv::Point3f((endPoint1.x+(j+1)*x_step),(endPoint1.y+(j+1)*y_step),(endPoint1.z+(j+1)*z_step));
                    // std::cout<<"ccc:"<<ccc<<std::endl;
                    pointcloud3d_mat.at<cv::Point3f>(j,count1 -2)=ccc;
                }
            }
           
        }

        // std::cout<<"pointcloud3d_mat:"<<pointcloud3d_mat<<std::endl;

        //将棋盘格点云存为vector，方便与图像提取的角点匹配
        for(size_t r=0;r<pointcloud3d_mat.rows;r++)
        {
            for(size_t c=0;c<pointcloud3d_mat.cols;c++)
            {
                checkerpointcloud.push_back(pointcloud3d_mat.at<cv::Point3f>(r,c));
            }
        }

        //此时已得到棋盘格角点在图像与激光点云中的匹配点对，分别存储在 std::vector<cv::Point2f> corners 与 std::vector<cv::Point3f> checkerpointcloud 中
        //采用pnp求解旋转与平移
        cv::Mat rvec,tvec,RMat;
        if(cv::solvePnP(checkerpointcloud,corners,cameraMatrix,cv::Mat(),rvec,tvec,false,cv::SOLVEPNP_EPNP));
        {
            cv::Rodrigues(rvec,RMat);
            std::cout<<"RMat:"<<RMat<<std::endl;
            std::cout<<"tvec:"<<tvec<<std::endl;
        }

        //////////////////////////////////////////////////////////////
        //    接下来要完成的任务是对多帧数据进行BA优化
        //    未完待续......2018/10/12

        //将棋盘格对应的3D点坐标转为点云，方便显示
        for(size_t i=0;i<pointcloud3d_mat.rows;i++)
        {
            for(size_t j=0;j<pointcloud3d_mat.cols;j++)
            {
                cvPoint2pclPoint(pointcloud3d_mat.at<cv::Point3f>(i,j),showPoint);
                showCloud.push_back(showPoint);
            }
        }
        showCloudP= showCloud.makeShared();
        
        // for(size_t i=0;i<boardPoints.size();i++)
        // {
        //     std::cout<<boardPoints.at(i).PointIndex<<std::endl;
        // }
        // std::cout<<"width:"<<width<<std::endl;
        // std::cout<<"height:"<<height<<std::endl;
        // std::cout<<"width_step:"<<width_step<<std::endl;
        // std::cout<<"height_step:"<<height_step<<std::endl;
        // std::cout<<boardPoints.at(1).PointData<<std::endl;

        //以图像的形式显示深度图像，深度值作为颜色显示
        pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
        range_image_widget.showRangeImage (range_image);

        // Display pointcloud:
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> 
        red(cloud, 255, 0, 0);
        viewer->addPointCloud(cloud,red,"PointCloud");

        //对分割后的点云着色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> 
        target_color(showCloudP, 255, 255, 0);
        viewer->removePointCloud("checkerboardCloud");
        viewer->addPointCloud(showCloudP,target_color,"checkerboardCloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                           5, "checkerboardCloud");

        viewer->setCameraPosition(-1,0,0,0,0,0,0,0,1);//设置pcl_viewer的视角为激光坐标系视角
        // Spin until 'Q' is pressed:
        viewer->spin();
        std::cout << "done." << std::endl;
        while (!viewer->wasStopped())
            {
                range_image_widget.spinOnce (100);
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
                // scene_sensor_pose = viewer.getViewerPose();
                // range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
                //                         pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                //                         scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
                // range_image_widget.showRangeImage (range_image);
            }
        }

	// cv::waitKey(0);
    return 0;
}

