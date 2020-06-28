#include "TwoFrameRTImpl.h"
#include "P_SemanticGraph.h"
#include "P_Config.h"
#include "FeatureQuadTree.h"
#include <opencv2/xfeatures2d.hpp>
#include "P_PoseEstimation.h"
#include "P_FeatureMatcher.h"
namespace Position
{
    class MultFeature : public Position::IFeature
    {
    public:
        MultFeature():mfast(FastFeatureDetector::create(20, true, 2)), mSift(cv::xfeatures2d::SIFT::create(2000,8,0.05,15,1.4))
        {
            Position::FloatVector  mvScaleFactor(4,0);
            Position::FloatVector  mvLevelSigma2(4,0);
            mSigmaVector.resize(4);
            mvLevelSigma2[0]= 1.0f;
            mSigmaVector[0] = 1.0f;

            for(int i=1; i < 4; i++)
            {
                mvScaleFactor[i]=mvScaleFactor[i-1]*1.4;
                mvLevelSigma2[i]= 1.0 /(mvScaleFactor[i]*mvScaleFactor[i]);
            }

        }

        //计算特征点
        virtual bool detect(const Position::FrameData &frame, Position::KeyPtVector &keys, Mat &descript) 
        {
            Position::KeyPtVector keys1,keys2; 
            Mat descript1,descript2;
            mfast->detect(frame._img, keys1);
            mSift->compute(frame._img,keys1,descript1);
        
            mSift->detect(frame._img, keys2);
            mSift->compute(frame._img,keys2,descript2);

            keys.insert(keys.end(),keys1.begin(),keys1.end());
            keys.insert(keys.end(),keys2.begin(),keys2.end());

            cv::vconcat(descript1, descript2, descript);
            return true;
        }

        //返回sigma参数(主要用于优化 信息矩阵)
        virtual const Position::FloatVector& getSigma2() const
        {
            return mSigmaVector;
        }

    protected:
        Position::FloatVector mSigmaVector;
        Ptr<FastFeatureDetector> mfast;
        Ptr<cv::xfeatures2d::SIFT> mSift;
    };

    TwoFrameRTImpl::TwoFrameRTImpl(std::string configPath)
    {
        //载入参数文件
        mpCfg = std::shared_ptr<Position::IConfig>(new PConfig());
        mpCfg->load(configPath + "/config.yaml");
        SemanticGraph::Instance()->loadObjInfos(configPath + "/semgraph.cfg");

        mFtSearchRadius = GETCFGVALUE(mpCfg,SearchRadius,int);
        mMatchratio = GETCFGVALUE(mpCfg,MatchRatio,float);
        //mask
        mmaskuse = GETCFGVALUE(mpCfg,MaskEnable,int);
        int roiLx = GETCFGVALUE(mpCfg,Lx,int);
        int roiLy = GETCFGVALUE(mpCfg,Ly,int);
        int roiW  = GETCFGVALUE(mpCfg,Wd,int);
        int roiH  = GETCFGVALUE(mpCfg,Ht,int);
        mrect = cv::Rect(roiLx, roiLy, roiW, roiH);

        //初始化
        mpOptimizer = std::shared_ptr<Position::IOptimizer>(new Position::G2oOptimizer());
        mpMap = std::shared_ptr<Position::IMap>(new Position::PMap);
        mpEst = std::shared_ptr<Position::IPoseEstimation>(new Position::ORBPoseEstimation());
        mpFeatureMatcher = std::shared_ptr<Position::IFeatureMatcher>(new Position::PKnnMatcher);

        //特征算子选择
        if(GETCFGVALUE(mpCfg,MultFeatDect,int))
        {
            mpFeature = std::shared_ptr<MultFeature>(new MultFeature());
        }
        else
        {
            mpFeature = std::shared_ptr<Position::IFeature>(new FeatureQuadTree(GETCFGVALUE(mpCfg,FeatureCnt,int)));
        }

    }
    
    TwoFrameRTImpl::~TwoFrameRTImpl()
    {
            
    }

    cv::Mat TwoFrameRTImpl::GernerateMask(const cv::Mat &seg, cv::Rect rect)
    {
        if(seg.empty())
        {
            PROMT_S("Mask Gernerate err!");
            return cv::Mat();
        }
        cv::Mat maskim = cv::Mat::ones(seg.rows, seg.cols,CV_8UC1);
        for(size_t i = 0; i<seg.rows; i++)
            for(size_t j = 0; j<seg.cols; j++)
            {
                cv::Point pt = Point(j,i);
                if(SemanticGraph::Instance()->isDynamicObj(pt,seg)||rect.contains(pt))
                {
                    maskim.at<uchar>(pt) = 0;
                }           
            }    
        return maskim;
    }

    int TwoFrameRTImpl::ImageOritationImpl(const std::map<ImageID, cv::Mat> imgs,     //原始图像
                        const std::map<ImageID, cv::Mat> imgSegments,                 //语义分割图
                        const Mat33 InternalParam,                                    //内参
                        const std::vector<ImageExtrinsics> cameraTrc,                 //轨迹数据
                        std::map<ImageID, OriRT> &rts,                                //位姿输出结果, 位姿输出都是相对于第一帧
                        int type)
    {
        if(!imgs.size()||!imgSegments.size()||!cameraTrc.size())
        {
            PROMT_S("two frame data input err!");
            return -1;
        }
            
        //计算时间间隔
        int fps = GETCFGVALUE(mpCfg,ImgFps,int);
        if(fps == 0)
        {
            PROMT_S("check fps setting!");
            return -1;
        }      
        double ti = 1.0/(double)fps;

        //内参
        mCam.K = (Mat_<double>(3,3)<<  InternalParam(0,0),InternalParam(0,1),InternalParam(0,2),
                                        InternalParam(1,0),InternalParam(1,1),InternalParam(1,2),
                                        InternalParam(2,0),InternalParam(2,1),InternalParam(2,2));
        
        Position::FrameHelper::initParams(GETCFGVALUE(mpCfg,ImgWd,int),GETCFGVALUE(mpCfg,ImgHg,int),&mCam);
        mpOptimizer->setCamera(mCam);
        mpEst->setCamera(mCam);
        Mat Tcw = Mat::eye(4,4,MATCVTYPE);
        //判断模式type
        if(type)
        {
            std::map<ImageID, cv::Mat>::const_iterator iter0;
            std::map<ImageID, cv::Mat>::const_iterator iter1;
            std::map<ImageID, cv::Mat>::const_iterator iter2;
            
            for(size_t i = 0;i<cameraTrc.size();++i)
            { 
                std::string imname = cameraTrc[i].name;
                iter1 = imgs.find(imname);
                iter2 = imgSegments.find(imname);

                //生成mask 
                Mat mask;  
                if(i>0)
                {
                    std::string prename = cameraTrc[i-1].name;
                    iter0 = imgSegments.find(prename);
                    if(mmaskuse && iter0 != imgs.end())
                    {   
                        mask = GernerateMask(iter0->second, mrect);
                    }
                }
                int preIndex, curIndex;
                if(iter1 != imgs.end() && iter2 != imgSegments.end())
                {
                    Position::FrameData curdata;
                    curdata._img = iter1->second;
                    curdata._pos._sp = cameraTrc[i].speed[1];
                    curIndex = cameraTrc[i].seqNum;
                    mpCurrent = new Position::PFrame(curdata,mpFeature, mpMap->frameCount());
                    Position::FrameHelper::assignFeaturesToGrid(mpCurrent);
                    
                    if(GETCFGVALUE(mpCfg,MultFeatDect,int))
                    {
                        std::shared_ptr<FeatureQuadTree>         pQuadTree    (new FeatureQuadTree(GETCFGVALUE(mpCfg,FeatureCnt,int))); 
                        pQuadTree->GenerateFeatureQuadTree(curdata._img,mpCurrent->mKeypts,mpCurrent->mKeypts);
                        pQuadTree->compute(curdata._img,mpCurrent->mKeypts,mpCurrent->mDescript);
                    }

                    OriRT rtres;
                    if(i == 0)
                    {
                        preIndex = curIndex;
                        mpLast = mpCurrent;
                        rtres.cfd = 1;
                        rtres.R = Mat33::eye();
                        rtres.T = Mat31::zeros();
                        rts.insert(pair<ImageID, OriRT>(iter1->first, rtres));
                        Mat pose1 = Mat::eye(4,4,CV_64F);
                        mpLast->setPose(pose1);
                        mpLastKeyFm = mpMap->createKeyFrame(mpLast);                                   
                    }
                    else
                    {
                        assert(mpLast);
                        assert(mpCurrent);
                       
                        Position::MatchVector good_matches =  mpFeatureMatcher->match(mpLast,mpCurrent,mFtSearchRadius,mMatchratio);
                        vector<DMatch>::iterator it = good_matches.begin();
                        vector<DMatch>::iterator ed = good_matches.end();

                        Position::MatchVector matches;
                        matches.reserve(good_matches.size());
                        for(; it != ed; ++it)
                        {
                            Point2f pt = mpLast->getKeys()[it->queryIdx].pt;
                            //当mask未设置,或者设置量 有值的时候(权重问题 后续再考虑)
                            if(mask.empty() || mask.at<uchar>(pt.y,pt.x))
                            {
                                matches.emplace_back(*it);
                            } 
                        }
                        good_matches.swap(matches);

                        mpEst->setFrames(mpLast,mpCurrent);
                        Mat R,t;
                        Position::Pt3Vector pt3ds;
                        

                        Mat estF;
                        //cout<<"good_matches: "<<good_matches.size()<<endl;
                        if(mpEst->estimate(R,t,good_matches,pt3ds,estF))
                        {
                            Mat pose2 = Mat::eye(4,4,CV_64F);
                            R.copyTo(pose2.rowRange(0,3).colRange(0,3));
                            t.copyTo(pose2.rowRange(0,3).col(3));
                            mpCurrent->setPose(pose2);
                            mpCurrentKeyFm = mpMap->createKeyFrame(mpCurrent);
                            assert(mpLastKeyFm);
                            assert(mpCurrentKeyFm);
                            
                            for(auto item : good_matches)
                            {
                                const Point3f fpt = pt3ds[item.queryIdx];
                                Mat mpt = (Mat_<MATTYPE>(4,1) << fpt.x,fpt.y,fpt.z,1.0);
                                mpt = mpt / mpt.at<MATTYPE>(3);
                                Position::IMapPoint *mppt = mpMap->createMapPoint(mpt); 
                                mpLastKeyFm->addMapPoint(mppt,item.queryIdx);
                                mpCurrentKeyFm->addMapPoint(mppt,item.trainIdx);
                            }
                            mpOptimizer->frameOptimization(mpCurrentKeyFm,mpFeature->getSigma2());

                            double len = curdata._pos._sp*ti*abs(curIndex-preIndex);
                            Mat T = mpCurrentKeyFm->getPose().clone();
                            T.rowRange(0,3).col(3)*=len;

                            Tcw = T * Tcw;
                            
                            Mat Twc = Tcw.inv();
                            rtres.cfd = 1;
                            rtres.R = Twc.rowRange(0,3).colRange(0,3).clone();
                            rtres.T = Twc.rowRange(0,3).col(3).clone();  

                            rts.insert(pair<ImageID, OriRT>(iter1->first, rtres));
                            
                            //preframe->release();
                            preIndex = curIndex;
                            mpLast = mpCurrent;
                            mpLastKeyFm = mpCurrentKeyFm;
                            
                            mpCurrent = NULL;
                            mpCurrentKeyFm = NULL;
                        }
                        else
                        {
                            mpCurrentKeyFm->release();
                            mpLastKeyFm->release();
                            PROMTD_S("pose estimate failed!!!");
                            if(i==1)
                                return -1;
                            else
                            {
                                for(size_t j = i;j<cameraTrc.size();++j)
                                {
                                    std::string imname = cameraTrc[i].name;
                
                                    std::map<ImageID, cv::Mat>::const_iterator iter;
                                    iter = imgs.find(imname);
                                    rtres.cfd = 0;
                                    rts.insert(pair<ImageID, OriRT>(iter->first, rtres));
                                }
                                break;
                            }
                        }
                    }                   
                }
                else
                {
                    PROMT_S("two frame Image or segImage err!");
                    return -1;
                }
                
            }
            
                
        }
        else
        {
            //to do...
        }
        return 1;
    }

}