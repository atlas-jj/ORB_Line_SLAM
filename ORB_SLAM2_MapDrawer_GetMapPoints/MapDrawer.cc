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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <fstream>
#include <iostream>
#include "Converter.h"
//#include<ros/ros.h>
//#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

///////////////chris/////////////////////////////////////////////////////////////////////////////////////
//void MapDrawer::setPublisher(ros::Publisher & _pub)
//{
//	myPub=_pub;
//}

void MapDrawer::DrawMapPoints()
{
//

   // ofstream file;////////
   // file.open("chris_points.txt",ios::app);	/////
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    string strf="Fid=[";
    string strx="Mx=[";
    string stry="My=[";
    string strz="Mz=[";
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        std::map<KeyFrame*,size_t> obsvs=vpMPs[i]->GetObservations();
        string strFids="";
         for (std::map<KeyFrame*,size_t>::const_iterator it = obsvs.begin(); it != obsvs.end(); ++it) {        
            if (it->first == NULL)
            {
                cout << "{INFO} - Empty observation " << endl;
            }
            else
            {
                KeyFrame* pKF=it->first;
                cv::Mat Tcw = pKF->GetPose();
                std::ostringstream stfs;
                stfs<<(pKF->mTimeStamp);
                string mnid=stfs.str();
                strFids +=mnid+",";
            }
         }
        
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        std::ostringstream d2sx;
        std::ostringstream d2sy;
	std::ostringstream d2sz;
        std::ostringstream stFrame;
	d2sx<<pos.at<float>(0);
	d2sy<<pos.at<float>(1);
	d2sz<<pos.at<float>(2);
        

        string xt=d2sx.str();
        string yt=d2sy.str();
        string zt=d2sz.str();
        //long unsigned int frid=vpMPs[i]->mnId;
        stFrame<<(vpMPs[i]->mnId);
        string mnid=stFrame.str();
        
        if(i<(iend-1))
        {
            xt+=";";
            yt+=";";
            zt+=";";
            mnid+=";";
            strFids+=";";
        }
        strx+=xt;
        stry+=yt;
        strz+=zt;
        strf+=strFids;
        //cout<<"FrameID:"<<strFids<<endl;
    }

    //write to text file ---by Chris
    strx+="];";
    stry+="];";
    strz+="];";
    strf+="];";
    //cout<<"strx:"<<strx<<endl;
    //cout<<"stry:"<<stry<<endl;
    //cout<<"strz:"<<strz<<endl;

    //file<<strx<<endl;
   // file<<stry<<endl;
    //file<<strz<<endl;
   // file<<strf<<endl;

    //file.close();
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
       // ofstream file;////////
       // file.open("chris_trajectory.txt",ios::app);	/////
        string strf="Fid=[";
        string strpx="px=[";
        string strpy="py=[";
        string strpz="pz=[";
        string strqx="qx=[";
        string strqy="qy=[";
        string strqz="qz=[";
        string strqw="qw=[";
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Tcw = pKF->GetPose();
            std::ostringstream sfid;
            std::ostringstream spx;
            std::ostringstream spy;
	    std::ostringstream spz;
            std::ostringstream sqx;
            std::ostringstream sqy;
            std::ostringstream sqz;
            std::ostringstream sqw;
	spx<<Tcw.at<float>(0,3);
	spy<<Tcw.at<float>(1,3);
	spz<<Tcw.at<float>(2,3);
        vector<float> Qcw = Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
        sqx<<Qcw[0];
        sqy<<Qcw[1];
        sqz<<Qcw[2];
        sqw<<Qcw[3];
        sfid<<(pKF->mnId);
        string pxt=spx.str();
        string pyt=spy.str();
        string pzt=spz.str();
        string qxt=sqx.str();
        string qyt=sqy.str();   
        string qzt=sqz.str();   
        string qwt=sqw.str();   
        string fidt=sfid.str();
           
        if(i<(vpKFs.size()-1))
        {
            pxt+=";";
            pyt+=";";
            pzt+=";";
            qxt+=";";
            qyt+=";";
            qzt+=";";
            qwt+=";";
            fidt+=";";
        }
        strpx+=pxt;
        strpy+=pyt;
        strpz+=pzt;
        strqx+=qxt;
        strqy+=qyt;
        strqz+=qzt;
        strqw+=qwt;  
        strf+=fidt;

            cv::Mat Twc = pKF->GetPoseInverse().t();
            //cout<<"kfid:"<<(pKF->mTimeStamp)<<endl;
            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }//end for
         //write to text file ---by Chris
    strpx+="];";
    strpy+="];";
    strpz+="];";
    strqx+="];";
    strqy+="];";
    strqz+="];";
    strqw+="];";
    strf+="];";
    //cout<<"strx:"<<strx<<endl;
    //cout<<"stry:"<<stry<<endl;
    //cout<<"strz:"<<strz<<endl;

   // file<<strpx<<endl;
  // // file<<strpy<<endl;
  //  file<<strpz<<endl;
  //  file<<strqx<<endl;
   // file<<strqy<<endl;
   // file<<strqz<<endl;
  //  file<<strqw<<endl;
  //  file<<strf<<endl;

   // file.close();


    }// end if

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
