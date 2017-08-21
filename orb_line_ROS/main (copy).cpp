#include <iostream>
#include <list>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#define MATCHES_DIST_THRESHOLD 9

using namespace cv;
using namespace cv::line_descriptor;
using namespace std;
double matchingConfidence=0;
ros::Publisher pubpcl;


static const char* keys =
{ "{@image_path1 | | Image path 1 }"
    "{@image_path2 | | Image path 2 }" };

static void help()
{
  std::cout << "\nThis example shows the functionalities of lines extraction " << "and descriptors computation furnished by BinaryDescriptor class\n"
            << "Please, run this sample using a command in the form\n" << "./example_line_descriptor_compute_descriptors <path_to_input_image 1>"
            << "<path_to_input_image 2>" << std::endl;

}

vector<string> split(string str, char delimiter) {
  vector<string> internal;
  stringstream ss(str); // Turn the string into a stream.
  string tok;

  while(getline(ss, tok, delimiter)) {
    internal.push_back(tok);
  }

  return internal;
}

cv::Mat getSkew(cv::Mat vec)
{
  cv::Mat K=(Mat_<double>(3,3) <<0,-vec.at<double>(2,0),vec.at<double>(1,0),vec.at<double>(2,0),0,-vec.at<double>(0,0),-vec.at<double>(1,0),vec.at<double>(0,0),0);
  return K;
}

cv::Point2d getPointInLine2(double D, cv::Point2f p1, cv::Point2f p2) //x1 !=x2;
{
  if(abs(p2.x-p1.x)<5)
  {
    if(p2.y>p1.y)
      return cv::Point2d(p1.x,p1.y+D);
    else
      return cv::Point2d(p1.x,p1.y-D);
  }
  else
  {
    double deltaY=p2.y-p1.y;
  double deltaX=p2.x-p1.x;
  double theta=atan(deltaY/deltaX);
  //cout<<"theta::"<<theta<<endl;
  double sintheta=sin(theta);
 // cout<<"sin::"<<sintheta<<endl;
  double costheta=cos(theta);
  //cout<<"cos::"<<costheta<<endl;
    double newX=p1.x+D*costheta;
    double newY=p1.y+D*sintheta;
    if(deltaX<0&&deltaY>0)
    {
      newX=p1.x-D*costheta;
      newY=p1.y-D*sintheta;
    }
    if(deltaX<0&&deltaY<0)
    {
      newX=p1.x-D*costheta;
      newY=p1.y-D*sintheta;
    }
    return cv::Point2d(newX,newY);
  }

}
cv::Mat TWC1;
cv::Mat TWC2;
cv::Point nccMatching(cv::Mat image, cv::Mat templ)
{
 // cv::Mat image;  // Your input image
//cv::Mat templ;  // Your template image of the screw
cv::Mat result; // Result correlation will be placed here

// Do template matching across whole image
cv::matchTemplate(image, templ, result, CV_TM_CCORR_NORMED);

// Find a best match:
double minVal, maxVal;
cv::Point minLoc, maxLoc;
cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

// Regards to documentation the best match is in maxima location
// (http://opencv.willowgarage.com/documentation/cpp/object_detection.html)

// Move center of detected screw to the correct position:
cv::Point screwCenter = maxLoc + cv::Point(templ.cols/2, templ.rows/2);
matchingConfidence=maxVal;
return screwCenter;
}
std::list<cv::Mat> previousImages;
std::list<cv::Mat> previousTWC1s;
cv::Mat currentTWC(4,4,cv::DataType<double>::type);
bool currentTWCValid=false;
cv::Mat imageMat2;
int image_count=0;
int Max_count=5;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     
	  //cout<<"intrisicMat"<<endl<<K<<endl;
	        // TWC1=(Mat_<double>(4,4)<<0.99650651, -0.011492188, 0.082721241, 0.020527277,0.01427226, 0.99935025, -0.033095218, -0.0034712953,-0.082287155, 0.034160219, 0.99602294, 0.53971553,0, 0, 0, 1);
	     //    TWC2=(Mat_<double>(4,4)<<0.99149781, -0.007802105, 0.1298897, 0.029513706,0.0095642013, 0.99987048, -0.012947837, -0.0044715595, -0.12977183, 0.014080041, 0.99144381, 0.64686686,0, 0, 0, 1);
    try
     {
        cv::Mat myImage=cv_bridge::toCvShare(msg, "bgr8")->image;//using for simulation display
        //imshow("view",myImage);
        if(currentTWCValid)
        {
           previousImages.push_back(myImage);
           previousTWC1s.push_back(currentTWC);
           //previousImages.pop_front();
          // previousTWC1s.pop_front();
image_count++;
        }

        if(image_count>=Max_count)
        {

         /* load image */
         std::list<cv::Mat>::iterator it=previousImages.begin();
         cv::Mat imageMat1 = *it;
         cv::Mat imageMat2 = myImage;
         std::list<cv::Mat>::iterator it2=previousTWC1s.begin();
         TWC1=*it2;
         TWC2=currentTWC;
       //   //rectify image, recover distortion
       //   cv::Mat imageMat1;
       // cv::Mat imageMat2;
       //
       // cv::Mat intrisicMat(3,3,cv::DataType<double>::type);
       //      intrisicMat.at<double>(0,0)=929.996179;
       //      intrisicMat.at<double>(0,2)=300.952418;
       //      intrisicMat.at<double>(1,1)=931.956932;
       //      intrisicMat.at<double>(1,2)=251.008379;
       //      intrisicMat.at<double>(2,2)=1;
       //
       //      cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector, all set to 0
       //            distCoeffs.at<double>(0) = 0.244575;
       //      distCoeffs.at<double>(1) = -0.170874;
       //      distCoeffs.at<double>(2) = -0.016775;
       //      distCoeffs.at<double>(3) = 0.031988;
       //      distCoeffs.at<double>(4) = 0;
       //      cv::Mat newCameraMatrix;
       //      cv::undistort(im1, imageMat1, intrisicMat, distCoeffs, newCameraMatrix);
       //      cv::undistort(im2, imageMat2, intrisicMat, distCoeffs, newCameraMatrix);
       // waitKey();
          if( imageMat1.data == NULL || imageMat2.data == NULL )
         {
           std::cout << "Error, images could not be loaded. Please, check their path" << std::endl;
         }

         /* create binary masks */
         cv::Mat mask1 = Mat::ones( imageMat1.size(), CV_8UC1 );
         cv::Mat mask2 = Mat::ones( imageMat2.size(), CV_8UC1 );
        //cout<<"all set"<<endl;
         /* create a pointer to a BinaryDescriptor object with default parameters */
         Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor(  );
       // cout<<"all set2"<<endl;
         /* compute lines and descriptors */
         std::vector<KeyLine> keylines1, keylines2;
        //cout<<"all set3"<<endl;
         cv::Mat descr1, descr2;

         ( *bd )( imageMat1, mask1, keylines1, descr1, false, false );
         ( *bd )( imageMat2, mask2, keylines2, descr2, false, false );

         /* select keylines from first octave and their descriptors */
         std::vector<KeyLine> lbd_octave1, lbd_octave2;
         Mat left_lbd, right_lbd;
         for ( int i = 0; i < (int) keylines1.size(); i++ )
         {
           if( keylines1[i].octave == 0 )
           {
             lbd_octave1.push_back( keylines1[i] );
             left_lbd.push_back( descr1.row( i ) );
           }
         }

         for ( int j = 0; j < (int) keylines2.size(); j++ )
         {
           if( keylines2[j].octave == 0 )
           {
             lbd_octave2.push_back( keylines2[j] );
             right_lbd.push_back( descr2.row( j ) );
           }
         }

         /* create a BinaryDescriptorMatcher object */
         Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

         /* require match */
         std::vector<DMatch> matches;
         bdm->match( left_lbd, right_lbd, matches );

         /* select best matches */
         std::vector<DMatch> good_matches;
         for ( int i = 0; i < (int) matches.size(); i++ )
         {
           if( matches[i].distance < MATCHES_DIST_THRESHOLD )
             good_matches.push_back( matches[i] );
         }




         /* plot matches */
         cv::Mat outImg;
         cv::Mat scaled1, scaled2;
         std::vector<char> mask( matches.size(), 1 );
         drawLineMatches( imageMat1, lbd_octave1, imageMat2, lbd_octave2, good_matches, outImg, Scalar::all( -1 ), Scalar::all( -1 ), mask,
                            DrawLinesMatchesFlags::DEFAULT );

         imshow( "view", outImg );

         //using epipolar constraints to check details of matches.
          std::vector<KeyLine> crosKl1, crosKl2;//store matched lines
         for (std::vector<cv::DMatch>::
                const_iterator it= good_matches.begin();
                it!= good_matches.end(); ++it) {
               // Get the position of left keypoints
               KeyLine kl1=keylines1[it->queryIdx];
               KeyLine kl2=keylines2[it->trainIdx];
               crosKl1.push_back(kl1);
               crosKl2.push_back(kl2);
         }

         cv::Mat K=(Mat_<double>(3,3) <<797.355792,0,247.244316,0,791.795971,276.411966,0,0,1);
         double invfx=1/797.355792;
         double invfy=1/791.795971;
         double cx=247.244316;
         double cy=276.411966;


         Mat R1 = TWC1.rowRange(0,3).colRange(0,3);//(Mat_<double>(3,3) << 0.99650651, -0.011492188, 0.082721241, 0.01427226, 0.99935025, -0.033095218, -0.082287155,  0.034160219, 0.99602294);
         //cout<<"R1"<<endl<<R1<<endl;
         Mat T1= TWC1.rowRange(0,3).col(3);//(Mat_<double>(3,1) << 0.020527277, -0.0034712953, 0.53971553);
         //cout<<"T1"<<endl<<T1<<endl;
         Mat R2=TWC2.rowRange(0,3).colRange(0,3);//(Mat_<double>(3,3) <<0.99149781, -0.007802105,0.1298897, 0.0095642013,  0.99987048,  -0.012947837, -0.12977183, 0.014080041, 0.99144381);
         //cout<<"R2"<<endl<<R2<<endl;
         Mat T2=TWC2.rowRange(0,3).col(3);//(Mat_<double>(3,1) <<0.029513706, -0.0044715595,  0.64686686);
         //cout<<"T2"<<endl<<T2<<endl;

         //compute F
         Mat R12=R1*(R2.t());
        // cout<<"R12"<<endl<<R12<<endl;
         Mat t12=(-R1)*(R2.t())*T2+T1;
        // cout<<"T12"<<endl<<t12<<endl;
         Mat t12x=getSkew(t12);
        // cout<<"skew T12"<<endl<<t12x<<endl;
         Mat invK=(K.t()).inv();
         Mat F=invK*t12x*R12*(K.inv());
        // cout<<"F"<<endl<<F<<endl;
           Mat Tcw1=TWC1.inv();
           Mat Tcw2=TWC2.inv();
            Mat P1=Tcw1.rowRange(0,3).colRange(0,4);
           Mat P2=Tcw2.rowRange(0,3).colRange(0,4);
           std::vector<cv::Point3d> outputPoints;
           cout<<"lines detected  "<<crosKl1.size()<<endl;
         for(int i=0;i<crosKl1.size();i++)//crosKl1.size()
         {

            //test , select one point in line1, find its best matches using epipolar constraints
         cv::Point2f startP1=crosKl1[i].getStartPoint();
         cv::Point2f endP1=crosKl1[i].getEndPoint();
         cv::Point2f startP2=crosKl2[i].getStartPoint();
         cv::Point2f endP2=crosKl2[i].getEndPoint();
         //use every point in line startP1 to endP1
         double distance=sqrt((startP1.x-endP1.x)*(startP1.x-endP1.x)+(startP1.y-endP1.y)*(startP1.y-endP1.y));
         double DIndex=0;
         std::vector<cv::Point2d> leftPoints;
         std::vector<cv::Point2d> rightPoints;
         while(DIndex<distance)
         {
           //process
           cv::Point2d pointInLine=getPointInLine2(DIndex,startP1,endP1);
           //cout<<"left p"<<endl<<pointInLine<<endl;
           Mat lP=(Mat_<double>(3,1)<<pointInLine.x, pointInLine.y,1);
           Mat epline=F*lP;
           Mat startp2=(Mat_<double>(3,1)<<startP2.x, startP2.y,1);
           Mat endp2=(Mat_<double>(3,1)<<endP2.x, endP2.y,1);
           Mat rightLine=startp2.cross(endp2);
           Mat rightPoint=epline.cross(rightLine);
           cv::Point2d rp(rightPoint.at<double>(0,0)/rightPoint.at<double>(2,0),rightPoint.at<double>(1,0)/rightPoint.at<double>(2,0));
           //cout<<"right p"<<endl<<rp<<endl;

           //do NCC template matching
             //use patch around lP, and search around rp, find best matches
          cv::Rect leftROI(pointInLine.x-7, pointInLine.y-7, 15, 15);

           cv::Rect rightROI(rp.x-50, rp.y-50, 101, 101);
           if(pointInLine.x>7&&pointInLine.y>7&&rp.x>50&&rp.y>50&&imageMat2.rows>300&&imageMat1.rows>300)
           {
        	  // cout<<"try to crop image"<<endl;
        	   try{
           cv::Mat rightPatch = imageMat2(rightROI);
           cv::Mat leftPatch = imageMat1(leftROI);
                     //imshow("leftPatch",leftPatch);
           //cout<<"image cropped!"<<endl;
           cv::Point matchedPointT=nccMatching(rightPatch,leftPatch);
           cv::Point2d newRP(matchedPointT.x+rp.x-50, matchedPointT.y+rp.y-50);
          // cout<<"right p Re"<<endl<<newRP<<endl;
           if(matchingConfidence>=0.95)
           {
           cv::Point2d xn1((pointInLine.x-cx)*invfx, (pointInLine.y-cy)*invfy);
           cv::Point2d xn2((newRP.x-cx)*invfx, (newRP.y-cy)*invfy);

           leftPoints.push_back(xn1);
           rightPoints.push_back(xn2);
           }
           }
        	   catch(const std::exception&){}
           }

           DIndex+=2;
          // break;
         }
         //construct the Points
         int pointsCount=leftPoints.size();
         if(pointsCount>0)
         {
         cv::Mat lpts(2,pointsCount,cv::DataType<double>::type);
         cv::Mat rpts(2,pointsCount,cv::DataType<double>::type);
         for(int m=0;m<leftPoints.size();m++)
         {
           lpts.at<double> (0,m)=leftPoints[m].x;
           lpts.at<double> (1,m)=leftPoints[m].y;
           rpts.at<double> (0,m)=rightPoints[m].x;
           rpts.at<double> (1,m)=rightPoints[m].y;
         }

        // cout<<"lptf"<<endl<<lpts<<endl;
        // cout<<"rpts"<<endl<<rpts<<endl;
        // cout<<"P1"<<endl<<P1<<endl;
        // cout<<"P2"<<endl<<P2<<endl;
          cv::Mat x3D;

          cv::triangulatePoints(P1,P2,lpts,rpts,x3D);
          // cout<<"x3d"<<endl<<x3D<<endl;

           for(int n=0;n<x3D.cols;n++)
           {
               double x=x3D.at<double>(0,n)/x3D.at<double>(3,n);
       	double y=x3D.at<double>(1,n)/x3D.at<double>(3,n);
       	double z=x3D.at<double>(2,n)/x3D.at<double>(3,n);
       	outputPoints.push_back(Point3d(x,y,z));
           }
         }
        }
      cout<<"3d point count  "<<"-----------------------"<<outputPoints.size()<<endl;
       //cout<<"points 3d "<<endl<<outputPoints<<endl;
        //publish the point cloud:
       sensor_msgs::PointCloud2 lidarscan;
      		lidarscan.header.stamp = ros::Time::now();
          	        lidarscan.header.frame_id="cameraPose";
          	        lidarscan.width=outputPoints.size();
          	        lidarscan.height=1;
      		//lidarscan.is_dense = true;


          	        sensor_msgs::PointCloud2Modifier modifier(lidarscan);

          	        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
          	             	                         "y", 1, sensor_msgs::PointField::FLOAT32,
          	             	                         "z", 1, sensor_msgs::PointField::FLOAT32);
          	        sensor_msgs::PointCloud2Iterator<float> iter_x(lidarscan, "x");
          	        sensor_msgs::PointCloud2Iterator<float> iter_y(lidarscan, "y");
          	        sensor_msgs::PointCloud2Iterator<float> iter_z(lidarscan, "z");

          	      for (size_t ptit=0; ptit<outputPoints.size();ptit++, ++iter_x, ++iter_y, ++iter_z)
          	         	        {

                                    cv::Point3d opts3d=outputPoints[ptit];

          	         	            *iter_x = opts3d.x;
          	         	            *iter_y = opts3d.z;
          	         	            *iter_z = opts3d.y;

          	         	        }

          	    pubpcl.publish(lidarscan);//publish laser scan cloud point




       image_count=0;
       previousImages.pop_front();
       if(currentTWCValid)
        {
                 previousImages.push_back(myImage);
                 previousTWC1s.push_back(currentTWC);
                 previousImages.pop_front();
                 previousTWC1s.pop_front();
                 image_count++;
        }


        }//end of if (count>=maxcount)

       // cout<<"image received"<<endl;
cv::waitKey(30);
     }//end of try
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }

  }

void nextTask_callback(const std_msgs::String::ConstPtr& msg)
 {
    std::string str= msg->data.c_str();
    //cout<<"new str "<<str<<endl;
    //update current TWC1;
    vector<string> sta3=split(str,';');
       for(size_t i=0;i<sta3.size();i++)
       {
       	string str3=sta3[i];
       	vector<string> sta4=split(str3,',');

       	for(size_t j=0;j<sta4.size();j++)
   	{
   	  double tempt=atof(sta4[j].c_str());
   	  currentTWC.at<double>(i,j)=tempt;
   	}
       }
       currentTWCValid=true;
   // cout<<"current TWC "<<endl<<currentTWC<<endl;
}

int main( int argc, char** argv )
{

 // Initialize the ROS system and become a node.
  ros::init(argc, argv, "orb_line");
  ros::NodeHandle nh;
  pubpcl = nh.advertise<sensor_msgs::PointCloud2>("/cloud_in", 1);

cv::namedWindow("view");
     cv::startWindowThread();
     image_transport::ImageTransport it(nh);
     image_transport::Subscriber subImage = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
     ros::Subscriber sub2 = nh.subscribe("/chris/twc", 1000, nextTask_callback);

  //waitKey();

ros::spin();
    //declaration of function
	cv::destroyWindow("view");
	return 0;

}
