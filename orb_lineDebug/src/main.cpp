
#include <iostream>  
#include <vector> 

#include <fstream>
#include <list>
#include <stdlib.h>
#include <string>
#include "../include/OrbLine.h"
using namespace std; 
using namespace cv;

int main(int argc, char** argv) 
{ 
  cv::Mat F1=(Mat_<double>(3,3) <<0, -7.294960901307637e-156, 6.866257146355473e-16,
 4.986646371546321e-156, 0, 2.819307881891166e-20,
 -1.769047946293168e-13, -1.062616224012667e-17, -1.96829937256708e-13);
  
  cv::Mat R12=(Mat_<double>(3,3) <<0.99573439, -0.091915414, 0.0080302451,
 0.091982454, 0.99572504, -0.0084201032,
 -0.0072219777, 0.0091228299, 0.99993229);
  
  cv::Mat F1R12=F1*R12;
  
  cout<<F1R12<<endl;
  
  cv::Point3d p1(0,0,0);
  cv::Point3d p2(1,1,2);
  double distance=cv::norm(p1,p2);
  cout<<distance<<endl;
  
  Mat _preImage=imread("1.png");
  Mat _currentImage=imread("2.png");
  imshow("im1", _preImage);
  imshow("im2", _currentImage);
  double fx=797.355792;
  double fy=791.795971;
  double cx=247.244316;
  double cy=276.411966;
  
  cv::Mat TWC1=(Mat_<double>(4,4) <<0.99826926, -0.0075122733, 0.058327463, 0.0021332994,
  0.0079663275, 0.99993974, -0.0075559565, -0.0013332696,
  -0.05826718, 0.0080075348, 0.99826896, 0.18146452,
  0, 0, 0, 1);
  
  cv::Mat TWC2=(Mat_<double>(4,4) <<0.99772024, -0.0045941761, 0.067329571, 0.006641984,
  0.0044574128, 0.99998766, 0.0021813312, -0.0023586091,
  -0.067338765, -0.0018762427, 0.99772853, 0.35187256,
  0, 0, 0, 1);
  
  ORB_SLAM2::OrbLine myorbline(_preImage,_currentImage,TWC1,TWC2,fx,fy,cx,cy);
  std::vector<std::vector<cv::Point3d> > outPoints=myorbline.GenerateLineMapPoints();     
  
  int lineCount=outPoints.size();
  cout<<"line count"<<lineCount<<endl;
  std::vector<cv::Point3d> displayOuts;
  for(int i=0;i<lineCount;i++)
  {
    std::vector<cv::Point3d> linePoint=outPoints[i];
    for(int j=0;j<linePoint.size();j++)
    {
      displayOuts.push_back(linePoint[j]);
    }
    cout<<"line"<<i<<":"<<linePoint.size()<<endl;
  }
    cout<<"3D points: "<<displayOuts<<endl;
    // Wait for a keystroke in the window
    waitKey(0);
    return 0;

}