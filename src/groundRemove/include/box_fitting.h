#ifndef BOX_FITTING_H
#define BOX_FITTING_H

#include <array>
#include <vector>
#include "cloud.h"
#include "param.h"

#include "box_type.h"
#include "convex_hull.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <GL/gl.h>
#include <GL/glu.h>

// #include <opencv2/core/eigen.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui.hpp>
using namespace std;
// using namespace cv;

extern float picScale; // picScale * roiM = 30 * 30
//const float picScale = 30;
extern int ramPoints;
extern float lSlopeDist;
extern int lnumPoints;

extern float tHeightMin;
extern float tHeightMax;
extern float tWidthMin;
extern float tWidthMax;
extern float tLenMin;
extern float tLenMax;
extern float tAreaMax;
extern float tRatioMin;
extern float tRatioMax;
extern float minLenRatio;
extern float tPtPerM3;

// 图像做的
void getBoundingBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr> & bbPoints);
// 使用原始点做的
void getBBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr> & bbPoints,
                    Cloud::Ptr & markPoints,
                    Cloud::Ptr & lShapePoints,
                    std::unordered_map<int, int> & bboxToCluster,
                    const float & lShapeHorizonResolution,
                    const int & debugID = -1);        
// 论文
// An Orientation Corrected Bounding Box Fit Based on the Convex Hull under Real Time Constraintes
// std::vector<Vertex> CloudToVertexs(const Cloud::Ptr & cloud, float & minZ, float & maxZ);

void CloudToVertexs(const Cloud::Ptr & cloud, 
            std::vector<Vertex> & bottomVertexs//,
            // std::vector<Vertex> & topVertexs
            );

// 最小二乘法拟合直线， 返回斜率 K
float fitLine(const std::vector<cv::Point2f> & points);

// RANSAC 算法拟合
float fitLineRansac(const vector<cv::Point2f>& clouds, 
                    const int & num_iterations,
                    const float & tolerance,
                    float & correct,
                    const bool & debug = false
                    );

void getOrientedBBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr> & bbPoints);    

// 根据方向拟合点云 RECT
void fitRect(const float & k, const Cloud & cloud, std::vector<cv::Point2f> & rect);
// 利用各种方法求对称点云的方向
float direction(const Cloud & cloud);
float direction(const std::vector<cv::Point2f> & cloud);

// 对单个 Cluster 进行边缘点提取
void getLShapePoints(Cloud & cluster, 
                    const float & maxDx,
                    const float & maxDy,
                    const bool & needFixed,
                    const bool & debugBool,
                    const float & lShapeHorizonResolution);

int ColFromAngle(float angle_cols, 
                    float minAngle, 
                    float maxAngle, 
                    const size_t & numSegment = 20,
                    const bool & debug = false);

// 根据俩主边的长度来划分边缘点选取的密度
int ColFromAngle(const float & angle_cols, 
                    const float & minAngle, 
                    const float & maxAngle, 
                    const float & OAngle,
                    const float & minToOStep,
                    const float & maxToOStep,
                    const int & numMinSample,
                    const bool & debug = false);
                    
// 俩个点之间的距离
float distTwoPoint(const point & p1, const point & p2);

// 遮挡检测
void setShapeOcclusionCheck(
                std::vector<float> & shapeToDist,
                std::vector<int> & shapeToClusterID,
                std::vector<float> & shapeAngleRef,
                const Cloud & cluster,
                const float & lshapeResRad
            );
#endif //MY_PCL_TUTORIAL_BOX_FITTING_H
