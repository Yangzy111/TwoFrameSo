/**
 *   P_Feature.h
 *   
 *   add by tu li gen   2020.2.13
 * 
 */
#ifndef __PFEATURE_H_H_
#define __PFEATURE_H_H_
#include "P_Interface.h"

namespace Position
{
    //特征提取
    class PFeature : public IFeature
    {
    public:
        PFeature()
        {
        }
        //设置配置文件
        PFeature(const std::shared_ptr<IConfig> &pcfg):mCfg(pcfg)
        {
        }

        //计算特征点
        virtual bool detect(const FrameData &frame,KeyPtVector &keys, Mat &descript)
        {
            assert(NULL);
            return false;
        }

         //返回sigma参数(主要用于优化 信息矩阵)
        virtual const FloatVector& getSigma2() const 
        {
            assert(NULL);
            return FloatVector();
        }
    protected:
        std::shared_ptr<IConfig> mCfg;
    };
}

#endif