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
#include<string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

#include "AllConfig.h"
#include "System.h"
#include "Converter.h"

using namespace std;
string bg,config;
bool enable_ros;
ORB_SLAM2::Send *sendpc;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
	enable_ros=true;
	ros::init(argc, argv, "RGBD");
	ros::start();
	ros::NodeHandle nh;
	ORB_SLAM2::System *SLAM;

	if(argc != 3)
	{
		ROS_ERROR("enter ros");
		ros::NodeHandle n_private("~");
		ORB_SLAM2::AllConfig* con=new ORB_SLAM2::AllConfig();
		con->setCon(n_private);
		sendpc=new ORB_SLAM2::Send(con->world,con->base,con->odom);
		SLAM = new ORB_SLAM2::System(con,ORB_SLAM2::System::RGBD);
	}   
	else
	{
		bg=argv[1];
		config=argv[2];
		SLAM = new ORB_SLAM2::System(bg,config,ORB_SLAM2::System::RGBD,true);
	} 
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ImageGrabber igb(SLAM);


	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/rgb", 10);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/depth", 10);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
	message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
	sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

	ros::spin();

	// Stop all threads
	SLAM->Shutdown();

	// Save camera trajectory
	SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	ros::shutdown();

	return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
	// Copy the ros image message to cv::Ma
	static tf::TransformListener tf_listener;
	cv_bridge::CvImageConstPtr cv_ptrRGB;
	try
	{
		cv_ptrRGB = cv_bridge::toCvCopy(msgRGB,sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv_bridge::CvImageConstPtr cv_ptrD;
	try
	{
		cv_ptrD = cv_bridge::toCvCopy(msgD);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	const ros::Time time=cv_ptrRGB->header.stamp;	
	cv::Mat pose=mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
	bool enable_ros=false;
	if(enable_ros)
	{
	    if(!sendpc->getloop())
	    {
		if(pose.rows==4&&pose.cols==4)
		{
		    cv::Mat Rwc(3,3,CV_32F);
		    cv::Mat twc(3,1,CV_32F);
		    Rwc = pose.rowRange(0,3).colRange(0,3).t();
		    twc = -Rwc*pose.rowRange(0,3).col(3);
		    float a[]={
			Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),twc.at<float>(0),
			Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),twc.at<float>(1),
			Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2),twc.at<float>(2),
			0,0,0,1
		    };
		    cv::Mat currentm(4,4,CV_32F,a);
		    cv::Mat finalm=ORB_SLAM2::Converter::toRight(currentm);
		    sendpc->sendout(mpSLAM->getCurrentFrame().cloud,finalm,time);
		}
	    }
	}	
	mpSLAM->getCurrentFrame().deletecloud();
}


