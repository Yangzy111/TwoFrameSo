
/**
 *   P_Converter.h
 *   
 *   add by tu li gen   2020.2.19
 * 
 */
#ifndef __PCONVERTER_H_H_
#define __PCONVERTER_H_H_
#include "P_Types.h"

#include<Eigen/Dense>
#include"g2o/g2o/types/types_six_dof_expmap.h"
#include"g2o/g2o/types/types_seven_dof_expmap.h"

namespace Position
{
    //坐标转换
    class PConverter
    {
    public:

       //cv/eigen data convert
       static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

       static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
       static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

       static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
       static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
       static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
       static cv::Mat toCvMat(const Eigen::Matrix3d &m);
       static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
       static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

       static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
       static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
       static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

       static FloatVector toQuaternion(const cv::Mat &M);

       //mat -> string 
       static std::string toString(const cv::Mat &mat);
       //string -> mat
       static cv::Mat     str2CVMat(const std::string &str,bool ispt = false);
    };
}


#endif