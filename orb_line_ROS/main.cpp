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
bool currentTWCValid=true;
cv::Mat imageMat2;
int image_count=0;
int Max_count=2;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     cout<<"ass"<<endl;
     image_count++;
     //currentTWC=true;
	  //cout<<"intrisicMat"<<endl<<K<<endl;
	        // TWC1=(Mat_<double>(4,4)<<0.99650651, -0.011492188, 0.082721241, 0.020527277,0.01427226, 0.99935025, -0.033095218, -0.0034712953,-0.082287155, 0.034160219, 0.99602294, 0.53971553,0, 0, 0, 1);
	     //    TWC2=(Mat_<double>(4,4)<<0.99149781, -0.007802105, 0.1298897, 0.029513706,0.0095642013, 0.99987048, -0.012947837, -0.0044715595, -0.12977183, 0.014080041, 0.99144381, 0.64686686,0, 0, 0, 1);
    try
     {

        cv::Mat myImage=cv_bridge::toCvShare(msg, "bgr8")->image;//using for simulation display
        imshow("view1",myImage);
        //if(currentTWCValid)
        {
           previousImages.push_back(myImage);
           //previousTWC1s.push_back(currentTWC);
           //previousImages.pop_front();
          // previousTWC1s.pop_front();

        }

        if(image_count>=Max_count)
        {
         cout<<"1212"<<endl;
         /* load image */
         std::list<cv::Mat>::iterator it=previousImages.begin();
         cv::Mat imageMat1 = *it;
         cv::Mat imageMat2 = myImage;

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

         image_count=0;
         previousImages.pop_front();
         //if(currentTWCValid)
          {
                   previousImages.push_back(myImage);
                  // previousTWC1s.push_back(currentTWC);
                   previousImages.pop_front();
                   //previousTWC1s.pop_front();
                   image_count++;
          }
       // cout<<"image received"<<endl;
         cv::waitKey(30);
         }
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
       //currentTWCValid=true;
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
     //ros::Subscriber sub2 = nh.subscribe("/chris/twc", 1000, nextTask_callback);

  //waitKey();

ros::spin();
    //declaration of function
	cv::destroyWindow("view");
	return 0;

}
