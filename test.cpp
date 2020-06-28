#include "TwoFrameCalcRT.h"
#include <iostream>
#include <opencv2/highgui.hpp>

void run(const std::map<ImageID, cv::Mat> imgs,  //原始图像
                            const std::map<ImageID, cv::Mat> imgSegments,                 //语义分割图
                            const Mat33 InternalParam,
                            const std::vector<ImageExtrinsics> cameraTrcs,                      //轨迹数据
                            std::map<ImageID, OriRT> &rts, std::string configPath)                                         //位姿输出结果, 位姿输出都是相对于第一帧
                            
{
    Position::TwoFrameCalcRT a(configPath);
    std::cout<<"begin ... "<<std::endl;
    if(a.ImageOritation(imgs,imgSegments,InternalParam,cameraTrcs,rts)>0)
    {
        std::map<ImageID, OriRT>::const_iterator iter = rts.begin();
        for(;iter != rts.end(); ++iter)
        {
            std::cout<<iter->first<<" , "<<iter->second.cfd<<std::endl;
            std::cout<<iter->first<<" , "<<iter->second.R<<std::endl;
            std::cout<<iter->first<<" , "<<iter->second.T<<std::endl;
        }
        std::cout<<"finish ... "<<std::endl;
    }
    else
    {
        std::cout<<" calc err! "<<std::endl;
    }
    
    
}


int main(int argc, char** argv)
{
    if(argc<4)
    {
        std::cout<<"input para err!"<<std::endl;
        return -1;
    }
    std::string imPath = argv[1];
    std::string segimPath = argv[2];
    std::string configPath = argv[3];
    std::vector<ImageExtrinsics> cameraTrcs;
    std::map<ImageID, cv::Mat> imgs;
    std::map<ImageID, cv::Mat> imgSegments;

    ImageExtrinsics cameraTrc;
    Mat33 InternalParam = Mat33(1197.087918053, 0, 992.219633496, 0, 1197.559963210, 578.406356694, 0, 0, 1);
    
    strcpy(cameraTrc.name, "0-006437-467-0007818");
    cameraTrc.speed[1] = 27.832800000;
    cameraTrc.seqNum = 0;
    std::string str = cameraTrc.name;
    //rts.insert(pair<ImageID, OriRT>(iter1->first, rtres));
    cv::Mat im = cv::imread(imPath+'/'+str+".jpg");
    cv::Mat seg = cv::imread(segimPath+'/'+str+".png");
    
    imgs.insert(std::pair<ImageID, cv::Mat>(str,im));
    imgSegments.insert(std::pair<ImageID, cv::Mat>(str,seg));
    cameraTrcs.push_back(cameraTrc);

    strcpy(cameraTrc.name, "0-006437-515-0007819");
    cameraTrc.speed[1] = 27.830500000;
    cameraTrc.seqNum = 1;
    str = cameraTrc.name;
    //rts.insert(pair<ImageID, OriRT>(iter1->first, rtres));
    im = cv::imread(imPath+'/'+str+".jpg");
    seg = cv::imread(segimPath+'/'+str+".png");

    imgs.insert(std::pair<ImageID, cv::Mat>(str,im));
    imgSegments.insert(std::pair<ImageID, cv::Mat>(str,seg));
    cameraTrcs.push_back(cameraTrc);

    strcpy(cameraTrc.name, "0-006437-608-0007821");
    cameraTrc.speed[1] = 27.823900000;
    cameraTrc.seqNum = 3;
    str = cameraTrc.name;
    //rts.insert(pair<ImageID, OriRT>(iter1->first, rtres));
    im = cv::imread(imPath+'/'+str+".jpg");
    seg = cv::imread(segimPath+'/'+str+".png");

    imgs.insert(std::pair<ImageID, cv::Mat>(str,im));
    imgSegments.insert(std::pair<ImageID, cv::Mat>(str,seg));
    cameraTrcs.push_back(cameraTrc);

    strcpy(cameraTrc.name, "0-006437-717-0007823");
    cameraTrc.speed[1] = 27.817200000;
    cameraTrc.seqNum = 5;
    str = cameraTrc.name;
    
    im = cv::imread(imPath+'/'+str+".jpg");
    seg = cv::imread(segimPath+'/'+str+".png");

    imgs.insert(std::pair<ImageID, cv::Mat>(str,im));
    imgSegments.insert(std::pair<ImageID, cv::Mat>(str,seg));
    cameraTrcs.push_back(cameraTrc);

    strcpy(cameraTrc.name, "0-006437-765-0007824");
    cameraTrc.speed[1] = 27.813900000;
    cameraTrc.seqNum = 6;
    str = cameraTrc.name;
    
    im = cv::imread(imPath+'/'+str+".jpg");
    seg = cv::imread(segimPath+'/'+str+".png");

    imgs.insert(std::pair<ImageID, cv::Mat>(str,im));
    imgSegments.insert(std::pair<ImageID, cv::Mat>(str,seg));
    cameraTrcs.push_back(cameraTrc);

    int N = 7000;
    while(N>0)
    {
        std::map<ImageID, OriRT> rts;
        std::cout<<"run "<<7001-N<<" times"<<std::endl;
        run(imgs,imgSegments,InternalParam,cameraTrcs,rts,configPath);
        N--;
    }
    
    return 0;
}