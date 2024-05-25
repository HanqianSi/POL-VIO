#pragma once

#include <cstdio>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <string>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>
#include <ctime>


#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
 #include <pcl/point_cloud.h>
 #include <pcl/kdtree/kdtree_flann.h>

#include "parameters.h"
#include "line_feature/line_feature.h"
#include "tic_toc.h"
#include "line_feature_tracker/lines_devide.h"
//#define pi 3.1415926

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
bool inBorder(const cv::line_descriptor::KeyLine &line);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

/**
* @class FeatureTracker
* @Description 对每个相机进行角点LK光流跟踪
*/
class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time,vector<float> &array_startx, vector<float> &array_starty, vector<float> &array_endx, vector<float> &array_endy,int cnt_img,cv_bridge::CvImageConstPtr ptr);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();
    
    cv::Mat mask;//图像掩码
    cv::Mat fisheye_mask;//鱼眼相机mask，用来去除边缘噪点

    cv::Mat prev_img, cur_img, forw_img,cur_img_line, forw_img_line;

    vector<cv::Point2f> n_pts;//每一帧中新提取的特征点

    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;

    vector<cv::Point2f> prev_un_pts, cur_un_pts;//归一化相机坐标系下的坐标
    
    vector<cv::Point2f> pts_velocity;//当前帧相对前一帧特征点沿x,y方向的像素移动速度

    vector<int> ids;//能够被跟踪到的特征点的id

    vector<int> track_cnt;//当前帧forw_img中每个特征点被追踪的时间次数

    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;

    camodocal::CameraPtr m_camera;

    double cur_time;
    double prev_time;

    static int n_id;//用来作为特征点id，每检测到一个新的特征点，就将n_id作为该特征点的id，然后n_id加1

    //-------------------------------------下面是我添加的和线相关的变量----------------------------------------
    LineFeature m_line_feature;

    vector<cv::Vec4f> matched_lines, cur_lines, forw_lines;

    cv::Mat cur_ldesc, forw_ldesc;

    vector<int> line_ids;

    vector<int> line_ids_2;
    vector<int> line_ids_3;
    vector<int> last_line_id;

    vector<int> line_track_cnt;
    vector<int> line_track_cnt_2;
    vector<int> line_track_cnt_3;

    static int n_line_id;

    static int n_line_id_2;

    void undistortedline(Vector3d& un_pts_s, Vector3d& un_pts_e, cv::Vec4f line);

    std::vector<float>  forw_line_para_a,forw_line_para_b,forw_line_para_c,cur_line_para_a,cur_line_para_b,cur_line_para_c;
    std::vector<cv::Point2f>  forw_line_hough,cur_line_hough,forw_mid_point,cur_mid_point;
    std::vector<cv::Point2f> forw_point_all,cur_point_all,forw_point_flow;
    std::vector<int> cur_flag;
   
    cv_bridge::CvImageConstPtr cur_ptr,forw_ptr;
};
