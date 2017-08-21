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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "../../../include/KeyFrame.h"
#include "../../../include/MapPoint.h"


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};
ros::Publisher pub;
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


    pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000);
    cout<<"ros pub created"<<endl;

    ros::spin();
    cout<<"ros after spin"<<endl;
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}
int maxKFid=0;
int adjustInterval=-5;
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
    int nowMaxId=mpSLAM->mpMap->GetMaxKFid();
    if(nowMaxId>(maxKFid+adjustInterval))
    {
        //get all keyframes, select kfid>(maxKFid+adjustInterval) keyframes
    	const vector<ORB_SLAM2::KeyFrame*> vpKFs = mpSLAM->mpMap->GetAllKeyFrames();
    	for(size_t i=0; i<vpKFs.size(); i++)
        {
    		ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
           int nowpkid=pKF->mnId;
           if(nowpkid>(maxKFid+adjustInterval))  //get frame pose, publish transforme tf
           {
    	       cv::Mat Tcw = pKF->GetPose();

    	       cv::Mat R = pKF->GetRotation().t();
    	       vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
    	        cv::Mat t = pKF->GetCameraCenter();
    	        tf::Vector3 tfv3(t.at<float>(0), t.at<float>(1), t.at<float>(2));
    	        tf::Quaternion tfq(q[0],q[1],q[2],q[3]);
    	       static tf::TransformBroadcaster br;
    	         tf::Transform transform(tfq,tfv3);
    	         std::ostringstream stfs;
    	        stfs<<(nowpkid);
    	         string mnid=stfs.str();
    	         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "cameraPose"));//publish tf topic


                 //get all map points
    	         const std::set<ORB_SLAM2::MapPoint*> MapPointsInKF=pKF->GetMapPoints();

    	         sensor_msgs::PointCloud2 lidarscan;
    	         lidarscan.header.frame_id=mnid;
    	         lidarscan.width=3;
    	         lidarscan.height=1;

    	          sensor_msgs::PointCloud2Modifier modifier(lidarscan);

    	          modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
    	             	                                                  "y", 1, sensor_msgs::PointField::FLOAT32,
    	             	                                                  "z", 1, sensor_msgs::PointField::FLOAT32);
    	          sensor_msgs::PointCloud2Iterator<float> iter_x(lidarscan, "x");
    	          sensor_msgs::PointCloud2Iterator<float> iter_y(lidarscan, "y");
    	          sensor_msgs::PointCloud2Iterator<float> iter_z(lidarscan, "z");

    	         for (std::set<ORB_SLAM2::MapPoint*>::iterator i = MapPointsInKF.begin(); i != MapPointsInKF.end(); i++)
    	         {
    	        	 ORB_SLAM2::MapPoint* element = *i;
    	             if(element->isBad())
    	                 continue;

    	             cv::Mat pos = element->GetWorldPos();

    	             *iter_x = pos.at<float>(0);
    	             *iter_y = pos.at<float>(2);
    	             *iter_z = pos.at<float>(1);
                     //for debugging.
    	             std::ostringstream d2sx;
    	             std::ostringstream d2sy;
    	             std::ostringstream d2sz;
    	             d2sx<<pos.at<float>(0);
    	             d2sy<<pos.at<float>(2);
    	             d2sz<<pos.at<float>(1);
    	             string xt=d2sx.str();
    	             string yt=d2sy.str();
    	             string zt=d2sz.str();
    	             cout<<"X:"<<xt<<endl;
    	             cout<<"Y:"<<yt<<endl;
    	             cout<<"Z:"<<zt<<endl;
    	         }
    	         pub.publish(lidarscan);//publish laser scan cloud point
    	         cout<<"ros message published<!"<<endl;
           }
    	}
    	maxKFid=nowMaxId;
    	cout<<maxKFid<<endl;
    }


    //cout<<"ros message published<!"<<endl;

}


