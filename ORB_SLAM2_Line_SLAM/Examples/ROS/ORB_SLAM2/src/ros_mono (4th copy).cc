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
    pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/cloud_in", 1);
    cout<<"\nros pub cloud_in created"<<endl;
    pubTask = nodeHandler.advertise<std_msgs::String>("/chris/twc", 1);
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
    if(nowMaxId>(maxKFid+adjustInterval))//new key frame here
    {
        //get all keyframes, select kfid>(maxKFid+adjustInterval) keyframes
    	const vector<ORB_SLAM2::KeyFrame*> vpKFs = mpSLAM->mpMap->GetAllKeyFrames();
    	for(size_t i=0; i<vpKFs.size(); i++)
        {
    	   ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
           int nowpkid=pKF->mnId;
           if(nowpkid>(maxKFid+adjustInterval))  //get frame pose, publish transform tf
           {
    	        cv::Mat TWC = pKF->GetPoseInverse();

                if(TWC.rows>2)
                {
			//cout<<"MyCurrentFrame.mTcw: "<<endl<<mpSLAM->mpTracker->mCurrentFrame.mTcw.inv()<<endl;
                	std_msgs::String msg;
                		std::stringstream ss;
                		for(int ti=0;ti<TWC.rows;ti++)
                		{
                			for(int tj=0;tj<TWC.cols;tj++)
                			{
                				std::ostringstream ssss;
                				ssss << TWC.at<float>(ti,tj);
                				ss<<ssss.str()<<",";
                			}

                			ss << ";";
                		}

                		msg.data = ss.str();
                		pubTask.publish(msg);

	    		cv::Mat RWC= TWC.rowRange(0,3).colRange(0,3);
	    		cv::Mat tWC= TWC.rowRange(0,3).col(3);
			//cout<<"RWC: "<<endl<<RWC<<endl;
			//cout<<"TWC: "<<endl<<tWC<<endl;
	    		tf::Matrix3x3 M(RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
	    			RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
	    			RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2));
			//cout<<"M constructed"<<endl;
	    		tf::Vector3 V(tWC.at<float>(0), tWC.at<float>(2), tWC.at<float>(1));
			//cout<<"V constructed"<<endl;
	    		tf::Quaternion q;
	    		M.getRotation(q);
			//cout<<"q obtained"<<endl;
			//static tf2_ros::TransformBroadcaster br;
	    		static tf::TransformBroadcaster br;
	    		tf::Transform transform = tf::Transform(M, V);
			//cout<<"transform constructed "<<endl;
	    		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "cameraPose"));

		}

                //get all map points
    	        const std::set<ORB_SLAM2::MapPoint*> MapPointsInKF=pKF->GetMapPoints();

    	        sensor_msgs::PointCloud2 lidarscan;
		lidarscan.header.stamp = ros::Time::now();
    	        lidarscan.header.frame_id="cameraPose";
    	        lidarscan.width=MapPointsInKF.size();
    	        lidarscan.height=1;
		//lidarscan.is_dense = true;


    	        sensor_msgs::PointCloud2Modifier modifier(lidarscan);

    	        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
    	             	                         "y", 1, sensor_msgs::PointField::FLOAT32,
    	             	                         "z", 1, sensor_msgs::PointField::FLOAT32);
    	        sensor_msgs::PointCloud2Iterator<float> iter_x(lidarscan, "x");
    	        sensor_msgs::PointCloud2Iterator<float> iter_y(lidarscan, "y");
    	        sensor_msgs::PointCloud2Iterator<float> iter_z(lidarscan, "z");


    	        for (std::set<ORB_SLAM2::MapPoint*>::iterator i = MapPointsInKF.begin(); i != MapPointsInKF.end();
			i++, ++iter_x, ++iter_y, ++iter_z)
    	        {

    	            ORB_SLAM2::MapPoint* element = *i;
    	            if(element->isBad())
    	                continue;

    	            cv::Mat pos = element->GetWorldPos();
                    //cout<<pos<<endl;

    	            *iter_x = pos.at<float>(0);
    	            *iter_y = pos.at<float>(2);
    	            *iter_z = pos.at<float>(1);

    	        }

    	        pub.publish(lidarscan);//publish laser scan cloud point
    	        //cout<<"ros message published!"<<endl;
           }
    	}//for end
    	maxKFid=nowMaxId;
    	//cout<<maxKFid<<endl;
    }


    //cout<<"ros message published<!"<<endl;

}
