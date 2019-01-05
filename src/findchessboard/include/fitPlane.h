#ifndef _FITPLANE_H
#define _FITPLANE_H

#include <iostream>
#include <opencv2/opencv.hpp>

//Ax+By+Cz=D  
void cvFitPlane(const CvMat* points, float* plane){  
    // Estimate geometric centroid.  
    int nrows = points->rows;  
    int ncols = points->cols;  
    int type = points->type;  
    CvMat* centroid = cvCreateMat(1, ncols, type);  
    cvSet(centroid, cvScalar(0));  
    for (int c = 0; c<ncols; c++){  
        for (int r = 0; r < nrows; r++)  
        {  
            centroid->data.fl[c] += points->data.fl[ncols*r + c];  
        }  
        centroid->data.fl[c] /= nrows;  
    }  
    // Subtract geometric centroid from each point.  
    CvMat* points2 = cvCreateMat(nrows, ncols, type);  
    for (int r = 0; r<nrows; r++)  
        for (int c = 0; c<ncols; c++)  
            points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];  
    // Evaluate SVD of covariance matrix.  
    CvMat* A = cvCreateMat(ncols, ncols, type);  
    CvMat* W = cvCreateMat(ncols, ncols, type);  
    CvMat* V = cvCreateMat(ncols, ncols, type);  
    cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);  
    cvSVD(A, W, NULL, V, CV_SVD_V_T);  
    // Assign plane coefficients by singular vector corresponding to smallest singular value.  
    plane[ncols] = 0;  
    for (int c = 0; c<ncols; c++){  
        plane[c] = V->data.fl[ncols*(ncols - 1) + c];  
        plane[ncols] += plane[c] * centroid->data.fl[c];  
    }  
    // Release allocated resources.  
    cvReleaseMat(&centroid);  
    cvReleaseMat(&points2);  
    cvReleaseMat(&A);  
    cvReleaseMat(&W);  
    cvReleaseMat(&V);  
}  

// 使用方法：
// CvMat*points_mat = cvCreateMat(X_vector.size(), 3, CV_32FC1);//定义用来存储需要拟合点的矩阵 
// 		for (int i=0;i < X_vector.size(); ++i)
// 		{
// 			points_mat->data.fl[i*3+0] = X_vector[i];//矩阵的值进行初始化   X的坐标值
// 			points_mat->data.fl[i * 3 + 1] = Y_vector[i];//  Y的坐标值
// 			points_mat->data.fl[i * 3 + 2] = Z_vector[i];<span style="font-family: Arial, Helvetica, sans-serif;">//  Z的坐标值</span>

// 		}
// 		float plane12[4] = { 0 };//定义用来储存平面参数的数组 
// 		cvFitPlane(points_mat, plane12);//调用方程 
// 拟合出来的方程： Ax+By+Cz=D
// 其中 A=plane12[0],    B=plane12[1],   C=plane12[2],   D=plane12[3],
#endif 