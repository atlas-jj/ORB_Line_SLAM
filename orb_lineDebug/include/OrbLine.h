/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBLINE_H
#define ORBLINE_H
#include <iostream>
#include <list>
#include <stdlib.h>
#include <string>
#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#define MATCHES_DIST_THRESHOLD 15
//#include <set>

//#include <mutex>



namespace ORB_SLAM2
{

class OrbLine
{
public:
    OrbLine(cv::Mat _previousImage, cv::Mat _currentImage,
      cv::Mat _previousTWC, cv::Mat _currentTWC, double fx, double fy, double cx, double cy);

    //void AddKeyFrame(KeyFrame* pKF);
    //void AddMapPoint(MapPoint* pMP);
    // void AddLineMapPoints(MapPoint* lMP);//----------------chris-------------orb_line
    // void EraseMapPoint(MapPoint* pMP);
    // void EraseKeyFrame(KeyFrame* pKF);
    // void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    // void InformNewBigChange();
    // int GetLastBigChangeIdx();
    //
    // std::vector<KeyFrame*> GetAllKeyFrames();
    // std::vector<MapPoint*> GetAllMapPoints();
    // std::vector<MapPoint*> GetAllLineMapPoints();//----------------chris-------------orb_line
    // std::vector<MapPoint*> GetReferenceMapPoints();
    //
    // long unsigned int MapPointsInMap();
    // long unsigned  KeyFramesInMap();
    //
    // long unsigned int GetMaxKFid();
    //
    // void clear();
    //
    // vector<KeyFrame*> mvpKeyFrameOrigins;
    //
    // std::mutex mMutexMapUpdate;
    //
    // // This avoid that two points are created simultaneously in separate threads (id conflict)
    // std::mutex mMutexPointCreation;
    //std::vector<MapPoint*>  GenerateLineMapPoints();
    std::vector<std::vector<cv::Point3d> > GenerateLineMapPoints();
protected:
    cv::Mat K;//camera's intrinsic parameters.
    double invfx;
    double invfy;
    double cx;
    double cy;
    double matchingConfidence=0;
    cv::Mat TWC1;
    cv::Mat TWC2;

    int maxKFid=0;
    int adjustInterval=0;
    cv::Mat previousKFImage;
    cv::Mat currentKFImage;
    cv::Mat previousTWC;
    //cv::Mat currentTWC;
    cv::Mat currentTWC;//(4,4,cv::DataType<double>::type);

    bool currentTWCValid=false;
    cv::Mat imageMat2;
    int image_count=0;
    int Max_count=20;

    //std::vector<string> split(string str, char delimiter) ;
    cv::Mat getSkew(cv::Mat vec);
    cv::Point2d getPointInLine2(double D, cv::Point2f p1, cv::Point2f p2);
    cv::Point nccMatching(cv::Mat image, cv::Mat templ);//////////////

    std::vector<cv::Point2d> refineSearchPoints(cv::Point2d pointInLine, cv::Mat rightLine, cv::Mat F);
    //std::vector<MapPoint*> lineMapPoints;
    std::vector<cv::Point3d> generatePointsInLine(cv::Point3d p1, cv::Point3d p2);

    // std::set<MapPoint*> mspMapPoints;
    // std::set<MapPoint*> lineMapPoints;//----------------chris-------------orb_line
    // std::set<KeyFrame*> mspKeyFrames;
    //
    // std::vector<MapPoint*> mvpReferenceMapPoints;
    //
    // long unsigned int mnMaxKFid;
    //
    // // Index related to a big change in the map (loop closure, global BA)
    // int mnBigChangeIdx;
    //
    // std::mutex mMutexMap;
    //
    // std::mutex mMutexLineMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
