
///************************************************************************************/
/// 文件名：quickSort.h
/// 功能：实现cv::Mat中第二列元素按从小到大排序，排序过程中实现整行数据交换
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
#ifndef _QUICKSORT_H
#define _QUICKSORT_H

#include <assert.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <config.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

///************************************************************************************/
/// 函数名：swapRow
/// 输入：cv::Mat &inputMat,size_t num1,size_t num2
/// 输出：无
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
void swapRow(cv::Mat &inputMat,size_t num1,size_t num2)
{
    // std::cout<<"inputMat"<<inputMat<<std::endl;
    cv::Mat row1=inputMat.rowRange(num1,num1+1).clone();//获取矩阵num1行元素
    // std::cout<<"row1"<<row1<<std::endl;
    cv::Mat row2=inputMat.rowRange(num2,num2+1).clone();//获取矩阵num２行元素
    // std::cout<<"row2"<<row2<<std::endl;
    cv::Mat temp1=inputMat.row(num1);//temp1指向inputMat的num２行
    row2.copyTo(temp1);//将row２拷贝给temp1，则inputMat的num２行数据也随之改变
    cv::Mat temp2=inputMat.row(num2);
    row1.copyTo(temp2);
    // std::cout<<"inputMat"<<inputMat<<std::endl;
}

///************************************************************************************/
/// 函数名：GetMid
/// 输入：cv::Mat &array,int left,int right
/// 输出：无
/// 说明：三数取中
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
int GetMid(cv::Mat &array,int left,int right,int channel)
{
    int mid = left + ((right - left)>>1);
    if(array.at<double>(left,channel) <= array.at<double>(right,channel))
    {
        if(array.at<double>(mid,channel) <  array.at<double>(left,channel))
            return left;
        else if(array.at<double>(mid,channel) > array.at<double>(right,channel))
            return right;
        else
            return mid;
    }
    else
    {
        if(array.at<double>(mid,channel) < array.at<double>(right,channel))
            return right;
        else if(array.at<double>(mid,channel) > array.at<double>(left,channel))
            return left;
        else
            return mid;
    }

}

///************************************************************************************/
/// 函数名：PartSort3
/// 输入：cv::Mat &array,int left,int right
/// 输出：无
/// 说明：前后指针法
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
int PartSort3(cv::Mat &array,int left,int right,int channel)
{
    int mid = GetMid(array,left,right,channel);
    swapRow(array,mid,right);
    if(left < right){
        double key = array.at<double>(right,channel);
        int cur = left;
        int pre = left - 1;
        while(cur < right)
        {
             while(array.at<double>(cur,channel) < key && ++pre != cur)
             {
                swapRow(array,cur,pre);
             }
                ++cur;
        }
            swapRow(array,++pre,right);
            return pre;
    }
    return -1;
}

///************************************************************************************/
/// 函数名：QuickSort
/// 输入：cv::Mat &array,int left,int right
/// 输出：无
/// 说明：前后指针法快排
/// 状态：调试完成
/// 作者：Andyoyo@swust
/// 时间：2018/10/9
///************************************************************************************/
void QuickSort(cv::Mat &pointcloud,int left,int right,int channel)
{
    if(left >= right)   
        return;       
    int index = PartSort3(pointcloud,left,right,channel);
    QuickSort(pointcloud,left,index-1,channel);
    QuickSort(pointcloud,index+1,right,channel);
}


#endif