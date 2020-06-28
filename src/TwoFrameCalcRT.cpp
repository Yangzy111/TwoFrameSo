#include "TwoFrameCalcRT.h"
#include "TwoFrameRTImpl.h"

namespace Position
{
    TwoFrameCalcRT::TwoFrameCalcRT(std::string configPath)
    {
        _impl = new TwoFrameRTImpl(configPath);
    }
    TwoFrameCalcRT::~TwoFrameCalcRT()
    {    
        _impl->~TwoFrameRTImpl();
         
    }
    int TwoFrameCalcRT::ImageOritation(const std::map<ImageID, cv::Mat> imgs,  //原始图像
                            const std::map<ImageID, cv::Mat> imgSegments,                 //语义分割图
                            const Mat33 InternalParam,
                            const std::vector<ImageExtrinsics> cameraTrc,                      //轨迹数据
                            std::map<ImageID, OriRT> &rts,                                          //位姿输出结果, 位姿输出都是相对于第一帧
                            int type)
    {
        return _impl->ImageOritationImpl(imgs, imgSegments, InternalParam, cameraTrc, rts, type);

    }
    //TwoFrameCalcRT::TwoFrameCalcRT(){}
}

