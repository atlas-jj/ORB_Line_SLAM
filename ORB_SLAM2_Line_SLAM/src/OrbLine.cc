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

#include "OrbLine.h"

//#include<mutex>


namespace ORB_SLAM2
{
  using namespace cv;
  using namespace cv::line_descriptor;
  using namespace std;
  OrbLine::OrbLine(cv::Mat _previousImage, cv::Mat _currentImage,
    cv::Mat _previousTWC, cv::Mat _currentTWC, double _fx, double _fy, double _cx, double _cy, double lineMatchDistanceThres
    , double reprojectionErrorThres, double pDistThres)
    {
      previousKFImage=_previousImage;
      currentKFImage=_currentImage;
      previousTWC=_previousTWC;
      currentTWC=_currentTWC;
      invfx=1/_fx;
      invfy=1/_fy;
      cx=_cx;
      cy=_cy;
      K=(cv::Mat_<double>(3,3) <<_fx,0,cx,0,_fy,cy,0,0,1);
      MATCHES_DIST_THRESHOLD=lineMatchDistanceThres;
      reprojectionErrorThreshold=reprojectionErrorThres;
      pointDistanceThreshold=pDistThres;
    }

  std::vector<std::vector<cv::Point3d> >  OrbLine::GenerateLineMapPoints()
  {
    std::vector<std::vector<cv::Point3d> > rtPoints;

  //  cout<<"std 1"<<endl;
      try{
	//imshow("view",currentKFImage);
        /* create binary masks */
        cv::Mat mask1 = cv::Mat::ones( previousKFImage.size(), CV_8UC1 );
        cv::Mat mask2 = cv::Mat::ones( currentKFImage.size(), CV_8UC1 );
        //cout<<"previous image size"<<previousKFImage.size()<<endl;
        //cout<<"current image size"<<currentKFImage.size()<<endl;
      //  cout<<"all set"<<endl;
        /* create a pointer to a BinaryDescriptor object with default parameters */
        Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor(  );
      //  cout<<"all set2"<<endl;
        /* compute lines and descriptors */
        std::vector<KeyLine> keylines1, keylines2;
      //  cout<<"all set3"<<endl;
        cv::Mat descr1, descr2;

        ( *bd )( previousKFImage, mask1, keylines1, descr1, false, false );
        ( *bd )( currentKFImage, mask2, keylines2, descr2, false, false );

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
        Ptr<cv::line_descriptor::BinaryDescriptorMatcher> bdm = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
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

        //  /* plot matches */
        //  cv::Mat outImg;
        //  cv::Mat scaled1, scaled2;
        //  std::vector<char> mask( matches.size(), 1 );
        //  drawLineMatches( previousKFImage, lbd_octave1, currentKFImage, lbd_octave2, good_matches, outImg, Scalar::all( -1 ), Scalar::all( -1 ), mask,
        //                     DrawLinesMatchesFlags::DEFAULT );
        //  matchedImage=outImg;
         //imshow( "view", outImg );


        //using epipolar constraints to check details of matches.
        std::vector<cv::line_descriptor::KeyLine> crosKl1, crosKl2;//store matched lines
        for (std::vector<cv::DMatch>::
               const_iterator it= good_matches.begin();
               it!= good_matches.end(); ++it) {
              // Get the position of left keypoints
              KeyLine kl1=keylines1[it->queryIdx];
              KeyLine kl2=keylines2[it->trainIdx];
              crosKl1.push_back(kl1);
              crosKl2.push_back(kl2);
        }
        //cout<<"all set 4"<<endl;
        Mat R1 = previousTWC.rowRange(0,3).colRange(0,3);//(Mat_<double>(3,3) << 0.99650651, -0.011492188, 0.082721241, 0.01427226, 0.99935025, -0.033095218, -0.082287155,  0.034160219, 0.99602294);
      //  cout<<"R1"<<endl<<R1<<endl;
        Mat T1= previousTWC.rowRange(0,3).col(3);//(Mat_<double>(3,1) << 0.020527277, -0.0034712953, 0.53971553);
      //  cout<<"T1"<<endl<<T1<<endl;
        Mat R2=currentTWC.rowRange(0,3).colRange(0,3);//(Mat_<double>(3,3) <<0.99149781, -0.007802105,0.1298897, 0.0095642013,  0.99987048,  -0.012947837, -0.12977183, 0.014080041, 0.99144381);
      //  cout<<"R2"<<endl<<R2<<endl;
        Mat T2=currentTWC.rowRange(0,3).col(3);//(Mat_<double>(3,1) <<0.029513706, -0.0044715595,  0.64686686);
      //  cout<<"T2"<<endl<<T2<<endl;
        //compute F
        Mat R12=R1*(R2.t());
      //  cout<<"R12"<<endl<<R12<<endl;
        Mat t12=(-R1)*(R2.t())*T2+T1;
      //  cout<<"T12"<<endl<<t12<<endl;
        Mat t12x=getSkew(t12);
      //  cout<<"skew T12"<<endl<<t12x<<endl;
        Mat invK=(K.t()).inv();
      //  cout<<"invK"<<endl<<invK<<endl;
        invK.convertTo(invK, CV_64F);
        t12x.convertTo(t12x,CV_64F);
        R12.convertTo(R12,CV_64F);
        K.convertTo(K,CV_64F);
        R1.convertTo(R1,CV_64F);
        T1.convertTo(T1,CV_64F);
        R2.convertTo(R2, CV_64F);
        T2.convertTo(T2,CV_64F);
      //  cout<<"converto finished!"<<endl;

        Mat F=invK*t12x*R12*(K.inv());
        //cout<<"F"<<endl<<F<<endl;
        Mat Tcw1=previousTWC.inv();
        Mat Tcw2=currentTWC.inv();
        Mat P1=Tcw1.rowRange(0,3).colRange(0,4);
        Mat P2=Tcw2.rowRange(0,3).colRange(0,4);

        std::cout<<"lines found  "<<crosKl1.size()<<std::endl;
	      int newPointsCreated=0;
        for(int i=0;i<crosKl1.size();i++)//crosKl1.size()
        {
	        std::vector<cv::Point3d> outputPoints;

          //test , select one point in line1, find its best matches using epipolar constraints
           cv::Point2f startP1=crosKl1[i].getStartPoint();
           cv::Point2f endP1=crosKl1[i].getEndPoint();
           //startP2 and endP2 are used to compute line equations.
           cv::Point2f startP2=crosKl2[i].getStartPoint();
           cv::Point2f endP2=crosKl2[i].getEndPoint();
           Mat startp2=(cv::Mat_<double>(3,1)<<startP2.x, startP2.y,1);
           Mat endp2=(cv::Mat_<double>(3,1)<<endP2.x, endP2.y,1);
           Mat rightLine=startp2.cross(endp2);

           std::vector<cv::Point2d> leftPoints;
           std::vector<cv::Point2d> rightPoints;

           std::vector<cv::Point2d> startPairs;
           std::vector<cv::Point2d> endPairs;
           startPairs=refineSearchPoints(startP1, rightLine, F, R1, R2);
           endPairs=refineSearchPoints(endP1, rightLine, F, R1, R2);
           if(startPairs.size()>1 && endPairs.size()>1)
           {
             leftPoints.push_back(startPairs[0]);
             leftPoints.push_back(endPairs[0]);
             rightPoints.push_back(startPairs[1]);
             rightPoints.push_back(endPairs[1]);
           }

           //change stragety, only compute start point and end point
           //then recover the 3D line points, intensity 1mm

          //construct the Points
          int pointsCount=leftPoints.size();

          if(pointsCount>1)
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

               bool pointValid=true;
              //  if(y<-0.3)
              //     pointValid=false;
               //check triangulation in front of camera
               cv::Mat Rcw1=R1.t();
               cv::Mat Rcw2=R2.t();
               cv::Mat x3Dt = (cv::Mat_<double>(1,3) << x, y, z);
               cv::Mat tcw1= Tcw1.rowRange(0,3).col(3).clone();
               cv::Mat tcw2= Tcw2.rowRange(0,3).col(3).clone();

               float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
               float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
               if(z1<=0||z2<=0)
                  pointValid=false;
               //check reprojection error in first frame
               cv::Mat x4D = (cv::Mat_<double>(4,1) << x, y, z,1);
               P1.convertTo(P1, CV_64F);
               P2.convertTo(P2, CV_64F);
               cv::Mat reprojImg1=P1*x4D;
               double u1=reprojImg1.at<double>(0,0)/reprojImg1.at<double>(2,0);
               double v1=reprojImg1.at<double>(1,0)/reprojImg1.at<double>(2,0);
               double ou1=leftPoints[n].x;
               double ov1=leftPoints[n].y;
               double deltaU1=u1-ou1;
               double deltaV1=v1-ov1;
               double error1=sqrt(deltaU1*deltaU1+deltaV1*deltaV1);
               cout<<"left reprojection error: "<<error1<<endl;
              //check reprojection error in second frame
               cv::Mat reprojImg2=P2*x4D;
               double u2=reprojImg2.at<double>(0,0)/reprojImg2.at<double>(2,0);
               double v2=reprojImg2.at<double>(1,0)/reprojImg2.at<double>(2,0);
               double ou2=rightPoints[n].x;
               double ov2=rightPoints[n].y;
               double deltaU2=u2-ou2;
               double deltaV2=v2-ov2;
               double error2=sqrt(deltaU2*deltaU2+deltaV2*deltaV2);
               cout<<"right reprojection error: "<<error2<<endl;
               if(error1>reprojectionErrorThreshold||error2>reprojectionErrorThreshold)
                  pointValid=false;

               //check scale consistent???
               //TODO
               if(pointValid)
                 outputPoints.push_back(cv::Point3d(x,y,z));
            }
            std::vector<cv::Point3d> pointsAlongLine;
	    if(outputPoints.size()>1)
	    {
        double pDistance=calcuPoint3dDistance(outputPoints[0], outputPoints[1]);
        cout<<"point distance"<<pDistance<<endl;
        if(pDistance<=pointDistanceThreshold&&pDistance>0.1)
        {
  	      pointsAlongLine=generatePointsInLine(outputPoints[0], outputPoints[1]);
  	      rtPoints.push_back(pointsAlongLine);
  	      newPointsCreated+=pointsAlongLine.size();
  	      cout<<"crossk line"<<i<<" 3D P: "<<endl<<pointsAlongLine<<endl;
        }
        else
          cout<<"distance too long, scale inconsistent, line has been deleted!"<<endl;
	    }
          }//end of if(pointsCount>0)



       }//end of for
       std::cout<<"3d point count  "<<"-----------------------"<<newPointsCreated<<std::endl;

      }
      catch (...)
      {
        //TODO with exceptions.
      }


      return rtPoints;
  }
  double OrbLine::calcuPoint3dDistance(cv::Point3d p1, cv::Point3d p2)
  {
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
  }

  std::vector<cv::Point3d> OrbLine::generatePointsInLine(cv::Point3d p1, cv::Point3d p2)
  {
    std::vector<cv::Point3d> outputPoints;
    outputPoints.push_back(p1);
    outputPoints.push_back(p2);
    //TODO, compute all points between p1 and p2.
    return outputPoints;
  }

  std::vector<cv::Point2d> OrbLine::refineSearchPoints(cv::Point2d pointInLine, cv::Mat rightLine,cv::Mat F, cv::Mat Rtwc1, cv::Mat Rtwc2)
  {
    std::vector<cv::Point2d> matchedPoints;
    try
    {
    //cout<<"left p"<<endl<<pointInLine<<endl;
    cv::Mat lP=(cv::Mat_<double>(3,1)<<pointInLine.x, pointInLine.y,1);
    cv::Mat epline=F*lP;
    Mat rightPoint=epline.cross(rightLine);
    cv::Point2d rp(rightPoint.at<double>(0,0)/rightPoint.at<double>(2,0),rightPoint.at<double>(1,0)/rightPoint.at<double>(2,0));
    //cout<<"right p"<<endl<<rp<<endl;

    cv::Rect leftROI(pointInLine.x-7, pointInLine.y-7, 15, 15);
    cv::Rect rightROI(rp.x-50, rp.y-50, 101, 101);
    if(pointInLine.x>7&&pointInLine.y>7&&rp.x>50&&rp.y>50
      &&currentKFImage.rows>300&&previousKFImage.rows>300)
    {
     // cout<<"try to crop image"<<endl;

          cv::Mat rightPatch = currentKFImage(rightROI);
          cv::Mat leftPatch = previousKFImage(leftROI);
          //imshow("leftPatch",leftPatch);
          //cout<<"image cropped!"<<endl;
          cv::Point matchedPointT=nccMatching(rightPatch,leftPatch);
          cv::Point2d newRP(matchedPointT.x+rp.x-50, matchedPointT.y+rp.y-50);
         // cout<<"right p Re"<<endl<<newRP<<endl;
	  //cout<<"line imageP: "<<endl<<"left  "<<pointInLine<<endl;
	  //cout<< "right  "<<newRP<<endl;
          if(matchingConfidence>=0.95)
          {
              cv::Point2d xn1((pointInLine.x-cx)*invfx, (pointInLine.y-cy)*invfy);
              cv::Point2d xn2((newRP.x-cx)*invfx, (newRP.y-cy)*invfy);
              cv::Mat mxn1 = (cv::Mat_<double>(3,1) << xn1.x, xn1.y, 1.0);
              cv::Mat mxn2 = (cv::Mat_<double>(3,1) << xn2.x, xn2.y, 1.0);
              // Check parallax between rays
              cv::Mat ray1=Rtwc1*mxn1;
              cv::Mat ray2=Rtwc2*mxn2;
              const double cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));
              //double cosParallaxStereo = cosParallaxRays+1;
              if(cosParallaxRays>0 && cosParallaxRays<0.9998)
              {
                  matchedPoints.push_back(xn1);
                  matchedPoints.push_back(xn2);
              }
          }
     }

    }//end of if
    catch(...){}
    return matchedPoints;
  }

//   std::vector<std::string> OrbLine::split(std::string str, char delimiter)
//   {
//     std::vector<std::string> internal;
//     std::stringstream ss(str); // Turn the string into a stream.
//     std::string tok;
//
//     while(std::getline(ss, tok, delimiter)) {
//       internal.push_back(tok);
//     }
//     return internal;
//   }

  cv::Mat OrbLine::getSkew(cv::Mat vec)
  {
    cv::Mat K=(cv::Mat_<double>(3,3) <<0,-vec.at<double>(2,0),vec.at<double>(1,0),vec.at<double>(2,0),0,-vec.at<double>(0,0),-vec.at<double>(1,0),vec.at<double>(0,0),0);
    return K;
  }

  cv::Point2d OrbLine::getPointInLine2(double D, cv::Point2f p1, cv::Point2f p2)
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

  // cv::Mat image;  // Your input image
  //cv::Mat templ;  // Your template image of the screw
  cv::Point OrbLine::nccMatching(cv::Mat image, cv::Mat templ)
  {
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

} //namespace ORB_SLAM
