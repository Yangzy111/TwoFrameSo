#pragma once
#include <opencv2/core.hpp>
#include <map>
#include <vector>
#include <string>

struct ImageExtrinsics
{
      int seqNum;              
      char name[200];        //图像名
      int gpsFixType;
      double BLH[3];
      double Xs;
      double Ys;
      double Zs;
      double ratation[9];
      double speed[2];
};

typedef cv::Matx<double, 3, 1> Mat31;
typedef cv::Matx<double, 3, 3> Mat33;
typedef std::string ImageID;

struct OriRT
{
      Mat33 R;
      Mat31 T;
      float  cfd;    //置信度
};

namespace Position
{
      class TwoFrameRTImpl;
      class TwoFrameCalcRT
      {
      public:
            //TwoFrameCalcRT();
            TwoFrameCalcRT(std::string configPath);//配置文件路径
            ~TwoFrameCalcRT();
            //出错返回-1.正常返回1
            int ImageOritation(const std::map<ImageID, cv::Mat> imgs,  //原始图像
                            const std::map<ImageID, cv::Mat> imgSegments,                 //语义分割图
                            const Mat33 InternalParam,
                            const std::vector<ImageExtrinsics> cameraTrc,                      //轨迹数据
                            std::map<ImageID, OriRT> &rts,                                          //位姿输出结果, 位姿输出都是相对于第一帧
                            int type=1);
      private:
            TwoFrameRTImpl * _impl;
      };
}