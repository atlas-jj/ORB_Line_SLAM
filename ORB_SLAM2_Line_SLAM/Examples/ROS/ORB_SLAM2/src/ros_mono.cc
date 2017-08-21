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


#include "std_msgs/String.h"
#include <iostream>
#include <cstdlib>

#include<algorithm>
#include<fstream>
#include<chrono>
#include<set>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>



#include <image_transport/image_transport.h>


#include"../../../include/System.h"
#include "../../../include/KeyFrame.h"
#include "../../../include/MapPoint.h"
#include "../../../include/Converter.h"
#include "../../../include/Map.h"
#include "../../../include/OrbLine.h"
#include "../../../include/MapPoint.h"
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};
//
ros::Publisher pub;
//ros::Publisher pose_pub;
ros::Publisher pubTask;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;

    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    // Create a publisher obmZect.
    //pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/cloud_in", 1);
    //cout<<"\nros pub cloud_in created"<<endl;
    //pubTask = nodeHandler.advertise<std_msgs::String>("/chris/twc", 1);
    //

    //pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/tf",1);
    ros::spin();
    cout<<"ros after spin"<<endl;
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}
//
int maxKFid=0;
int adjustInterval=0;
cv::Mat previousKFImage;
cv::Mat currentKFImage;
cv::Mat previousTWC;
cv::Mat currentTWC;
cv::Mat Ow1;
cv::Mat Ow2;
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    //
    int nowMaxId=mpSLAM->mpMap->GetMaxKFid();
    if(nowMaxId>(maxKFid+adjustInterval))//new key frame detected
    {
      //if(maxKFid==0)  //the first key frame
      //get all keyframes, select kfid>(maxKFid+adjustInterval) keyframes
    	const vector<ORB_SLAM2::KeyFrame*> vpKFs = mpSLAM->mpMap->GetAllKeyFrames();
    	  for(size_t i=0; i<vpKFs.size(); i++)
        {
    	     ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
           int nowpkid=pKF->mnId;
           if(nowpkid==nowMaxId)  //get frame pose
           {
    	        cv::Mat TWC = pKF->GetPoseInverse();//return TWC
              cv::Mat cameraPose=pKF->GetCameraCenter();
              cout<<"camera center: "<<cameraPose<<endl;
            //cout<<"MyCurrentFrame.mTcw: "<<endl<<mpSLAM->mpTracker->mCurrentFrame.mTcw.inv()<<endl;
            //frame image and frame pose are all ready now.
              if(maxKFid==0) //the first key frame
              {
                previousKFImage=cv_ptr->image;
                previousTWC=TWC;
                Ow1=cameraPose;
              }
              else
              {
                currentKFImage=cv_ptr->image;
                currentTWC=TWC;
                Ow2=cameraPose;
                //check the baseline
                cv::Mat vBaseline = Ow2-Ow1;
                const float baseline = cv::norm(vBaseline);
                cout<<"baseline: "<<baseline<<endl;
                if(baseline>0.1)
                {
                    //process orb_line points
                    ORB_SLAM2::OrbLine myorbline(previousKFImage,currentKFImage,previousTWC,
                      currentTWC,pKF->fx,pKF->fy,pKF->cx,pKF->cy, 9, 0.1,3);//line match distance threshold, reprojection error threshold, 3D line length threshold
                    std::vector<std::vector<cv::Point3d> > outPoints;
                    try{
                    outPoints=myorbline.GenerateLineMapPoints();
                    }
                    catch(...){}
                    //mpSLAM->mpMap->setLineMatchImage(myorbline.matchedImage);
                   for(int m=0;m<outPoints.size();m++)
                   {
                     std::vector<cv::Point3d> linePoint1=outPoints[m];
                     ORB_SLAM2::MapPoint* pMP = new ORB_SLAM2::MapPoint(linePoint1[0], linePoint1[1]);
                    //check existing lines, compute similarity distances
                    //if distance is very small, regard them as the same line, use fusion (EKF) algorithm to merge them together.
                    //TODO
                    //..
                    //..
                     mpSLAM->mpMap->AddLineMapPoints(pMP);
                   }
                    //if baseline is long enough,
                    //set current to previous ones
                    previousKFImage=currentKFImage;
                    previousTWC=currentTWC;
                    Ow1=Ow2;
              }
            }//end of else
           }
    	   }//for end
    	maxKFid=nowMaxId;
    	//cout<<maxKFid<<endl;
    }//end if (nowMaxId>.....)
    //cout<<"ros message published<!"<<endl;
}
