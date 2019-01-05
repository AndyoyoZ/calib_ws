//fileName:convertion.h
//
#ifndef _CONVERTION_H
#define _CONVERTION_H
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>    //关于深度图像的头文件


typedef struct my3dpointtype
{
    cv::Point2d PointIndex;
    cv::Point3f PointData;
}my3dPointType;

typedef struct my2dpointtype
{
    cv::Point2d PointIndex;
    cv::Point2f PointData;
}my2dPointType;


// ///************************************************************************************/
// /// 函数名：corners2Mat
// /// 输入：std::vector<cv::Point2f> &corners,cv::Mat &outputMat
// /// 输出：无
// /// 状态：调试中........
// /// 作者：Andyoyo@swust
// /// 时间：2018/10/12
// ///************************************************************************************/
// void corners2Mat (std::vector<cv::Point2f> &corners,cv::Mat &outputMat)
// {
//     cv::Mat temp=cv::Mat::zeros(cv::Size(PatSize_width,PatSize_height),CV_32FC2);
//     for(size_t i=0;i<PatSize_height;i++)
//     {
//         for(size_t j=0;j<PatSize_width;j++)
//         {
//              temp.at< cv::Point2f >(i,j)=corners.at(i*PatSize_height+j);
//         }
//     }
//     temp.copyTo(outputMat);
// }


///************************************************************************************/
/// 函数名：pclPoint2cvPoint
/// 输入：pcl::PointXYZ &inputpclPoint,cv::Point3f &outputcvPoint
/// 输出：无
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
void pclPoint2cvPoint(pcl::PointXYZRGBA &inputpclPoint,cv::Point3f &outputcvPoint)
{
    outputcvPoint.x=inputpclPoint.x;
    outputcvPoint.y=inputpclPoint.y;
    outputcvPoint.z=inputpclPoint.z;
    // std::cout<<"inputpclPoint.x"<<inputpclPoint.x<<std::endl;
    // std::cout<<"outputcvPoint.x"<<outputcvPoint.x<<std::endl;
}
///************************************************************************************/
/// 函数名：cvPoint2pclPoint
/// 输入：cv::Point3f &inputcvPoint,pcl::PointXYZ  &outputpclPoint
/// 输出：无
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
void cvPoint2pclPoint(cv::Point3f &inputcvPoint,pcl::PointXYZRGBA  &outputpclPoint)
{
    outputpclPoint.x=inputcvPoint.x;
    outputpclPoint.y=inputcvPoint.y;
    outputpclPoint.z=inputcvPoint.z;
}

///************************************************************************************/
/// 函数名：Vector3f2cvPoint
/// 输入：Eigen::Vector3f &input,cv::Point3f &output
/// 输出：无
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
void Vector3f2cvPoint(Eigen::Vector3f &input,cv::Point3f &output)
{
    output.x=input(0,0);
    output.y=input(1,0);
    output.z=input(2,0);
}

///************************************************************************************/
/// 函数名：Vector3f2pclPoint
/// 输入：Eigen::Vector3f &input,pcl::PointXYZ &output
/// 输出：无
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
void Vector3f2pclPoint(Eigen::Vector3f &input,pcl::PointXYZRGBA &output)
{
    output.x=input(0,0);
    output.y=input(1,0);
    output.z=input(2,0);
}


///************************************************************************************/
/// 函数名：pointCloudPtr2Mat
/// 输入：pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,cv::Mat &outputCloud
/// 输出：无
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
void pointCloudPtr2Mat(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputCloud,cv::Mat &outputCloud)
{
    cv::Mat temp=cv::Mat::zeros(inputCloud->points.size(),3,CV_64F);
    for(size_t i=0;i<inputCloud->points.size();i++)
        {
            temp.at<double>(i,0)=inputCloud->points[i].x;
            temp.at<double>(i,1)=inputCloud->points[i].y;
            temp.at<double>(i,2)=inputCloud->points[i].z;
        }
    outputCloud=temp;
}
///************************************************************************************/
/// 函数名：pointCloud2Mat
/// 输入：pcl::PointCloud<pcl::PointXYZ> &inputCloud,cv::Mat &outputCloud
/// 输出：无
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
void pointCloud2Mat(pcl::PointCloud<pcl::PointXYZRGBA> &inputCloud,cv::Mat &outputCloud)
{
    cv::Mat temp=cv::Mat::zeros(inputCloud.size(),3,CV_64F);
    for(size_t i=0;i<inputCloud.size();i++)
        {
            temp.at<double>(i,0)=inputCloud.at(i).x;
            temp.at<double>(i,1)=inputCloud.at(i).y;
            temp.at<double>(i,2)=inputCloud.at(i).z;
        }
    outputCloud=temp;
}

///************************************************************************************/
/// 函数名：pointCloud2rangeImage
/// 输入：pcl::PointCloud<pcl::PointXYZ> &point_cloud,pcl::RangeImage &range_image
/// 输出：无
/// 说明：从点云中获取深度图
//       关于range_image.createFromPointCloud（）参数的解释 （涉及的角度都为弧度） ：
//       point_cloud为创建深度图像所需要的点云
//       angular_resolution_x深度传感器X方向的角度分辨率
//       angular_resolution_y深度传感器Y方向的角度分辨率
//       pcl::deg2rad (360.0f)深度传感器的水平最大采样角度
//       pcl::deg2rad (180.0f)垂直最大采样角度
//       scene_sensor_pose设置的模拟传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换
//       coordinate_frame定义坐标系，默认为CAMERA_FRAME
//       noise_level  获取深度图像深度时，邻近点对查询点距离值的影响水平
//       min_range 设置最小的获取距离，小于最小的获取距离的位置为传感器的盲区
//       border_size  设置获取深度图像边缘的宽度 默认为0 
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/10
///************************************************************************************/
void pointCloud2rangeImage(pcl::PointCloud<pcl::PointXYZRGBA> &point_cloud,pcl::RangeImage &range_image)
{
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());   //申明传感器的位置是一个4*4的仿射变换
    //传感器的采样位姿  就是获取点云的传感器的平移与旋转向量
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
    //设置深度图像遵循坐标系统
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;//CAMERA_FRAME;//LASER_FRAME
    //设置基本参数
    float angular_resolution_x = 0.1f,//angular_resolution为深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
    angular_resolution_y = angular_resolution_x;
    angular_resolution_x = pcl::deg2rad (angular_resolution_x);//角度值转为弧度值
    angular_resolution_y = pcl::deg2rad (angular_resolution_y);
    float noise_level = 0.01f;
    float min_range = 0.0f;
    int border_size = 1;
    range_image.createFromPointCloud (point_cloud, angular_resolution_x, 
                                        angular_resolution_y,pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                        scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size); 
}


///************************************************************************************/
/// 函数名：getEndPoints
/// 输入：std::vector< my3dPointType > inputPoints,int rangeImg_height,my3dPointType &endPoint1,my3dPointType &endPoint2
/// 输出：无
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/12
///************************************************************************************/
void getEndPoints(std::vector< my3dPointType > inputPoints,int rangeImg_height,my3dPointType &endPoint1,my3dPointType &endPoint2)
{
    cv::Point3f point_1= inputPoints.at(0).PointData;
    cv::Point3f point_2= inputPoints.at(inputPoints.size()-1).PointData;
    int point1_y_index=inputPoints.at(0).PointIndex.y;
    int point2_y_index=inputPoints.at(inputPoints.size()-1).PointIndex.y;
    double index_diff=(double)(point2_y_index-point1_y_index);
    double x_step=(point_2.x-point_1.x)/index_diff;
    double y_step=(point_2.y-point_1.y)/index_diff;
    double z_step=(point_2.z-point_1.z)/index_diff;
    cv::Point3f pointdata1,pointdata2;
    pointdata1=cv::Point3f(point_1.x+x_step*(0-point1_y_index),point_1.y+y_step*(0-point1_y_index),point_1.z+z_step*(0-point1_y_index));
    pointdata2=cv::Point3f(point_2.x+x_step*((rangeImg_height-1)-point2_y_index),
                                            point_2.y+y_step*((rangeImg_height-1)-point2_y_index),
                                            point_2.z+z_step*((rangeImg_height-1)-point2_y_index));
    endPoint1.PointIndex=cv::Point2d(inputPoints.at(0).PointIndex.x,0);
    endPoint2.PointIndex=cv::Point2d(inputPoints.at(0).PointIndex.x,rangeImg_height-1);
    endPoint1.PointData=pointdata1;
    endPoint2.PointData=pointdata2;
}



#endif


//endfile