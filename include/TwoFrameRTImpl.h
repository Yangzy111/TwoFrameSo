#ifndef __TWOFRAMERTIMPL_H_
#define __TWOFRAMERTIMPL_H_

#include "P_Interface.h"
#include "P_Map.h"
#include "P_Optimizer.h"
#include "TwoFrameCalcRT.h"
namespace Position
{
    class TwoFrameRTImpl
    {
    public:
        TwoFrameRTImpl(string configPath);

        //调用函数形式
        int ImageOritationImpl(const std::map<ImageID, cv::Mat> imgs,  //原始图像
                            const std::map<ImageID, cv::Mat> imgSegments,                 //语义分割图
                            const Mat33 InternalParam,
                            const std::vector<ImageExtrinsics> cameraTrc,                      //轨迹数据
                            std::map<ImageID, OriRT> &rts,                                          //位姿输出结果, 位姿输出都是相对于第一帧
                            int type=1);

        ~TwoFrameRTImpl();
    private:
        cv::Mat GernerateMask(const cv::Mat &seg, cv::Rect rect);
    private:
        std::shared_ptr<Position::IConfig>   mpCfg;
        std::shared_ptr<IFeature>            mpFeature;
        std::shared_ptr<IFeatureMatcher>     mpFeatureMatcher;
        std::shared_ptr<IPoseEstimation>     mpEst;
        std::shared_ptr<IOptimizer>          mpOptimizer;
        CameraParam                          mCam;
        int                                  mFtSearchRadius;
        float                                mMatchratio;
        std::shared_ptr<IMap>                mpMap;

        int mmaskuse;
        cv::Rect mrect;

        IFrame                     *mpCurrent;
        IFrame                     *mpLast;
    
        IKeyFrame                  *mpCurrentKeyFm;
        IKeyFrame                  *mpLastKeyFm;
    };
}

#endif