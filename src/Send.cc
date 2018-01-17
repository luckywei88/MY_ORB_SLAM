#include "Send.h"

using namespace std;
using namespace sensor_msgs;

namespace ORB_SLAM2
{

Send::Send(string world,string base,string odom)
:	world(world),base(base),odom(odom),i(0),j(0)
{
	ros::NodeHandle nh;
	loop=false;
	pclpub=nh.advertise<sensor_msgs::PointCloud2>("/tmp/pointcloud",1);	
	framepub=nh.advertise<sensor_msgs::Image>("/tmp/frame",1);	
	keypub=nh.advertise<sensor_msgs::Image>("/tmp/keyframe",1);	
	cmdpub=nh.advertise<std_msgs::String>("/tmp/cmd",1);
	cmdsub=nh.subscribe<std_msgs::String>("/tmp/cmd",1,&Send::cmdcallback,this);	
}

void Send::Frame(cv::Mat image)
{
	cv_bridge::CvImage rgb;
	sensor_msgs::Image rgb_image;
	rgb.encoding=sensor_msgs::image_encodings::BGR8;
	rgb.image=image;
	rgb.toImageMsg(rgb_image);
	framepub.publish(rgb_image);
}

void Send::KeyFrame(cv::Mat image)
{
	cv_bridge::CvImage rgb;
	sensor_msgs::Image rgb_image;
	rgb.encoding=sensor_msgs::image_encodings::BGR8;
	rgb.image=image;
	rgb.toImageMsg(rgb_image);
	keypub.publish(rgb_image);
}

void Send::sendout(PointC::Ptr pc,cv::Mat pose)
{
	/*
	tf::StampedTransform obtf;
	try
	{
		tf_listener.waitForTransform(odom,base,time,ros::Duration(0.05));
		tf_listener.lookupTransform(odom,base,time,obtf);
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("error %s",ex.what());
		return;
	}
	*/
	//cv::Mat finalk(4,4,CV_32F);
//	finalk=matrix;
	/*
	origin.setValue(
			finalk.at<float>(2,3),-finalk.at<float>(0,3),-finalk.at<float>(1,3)+1);
	tf::Matrix3x3 tf3d;
	tf3d.setValue(
			finalk.at<float>(0,0),finalk.at<float>(0,1),finalk.at<float>(0,2),
			finalk.at<float>(1,0),finalk.at<float>(1,1),finalk.at<float>(1,2),
			finalk.at<float>(2,0),finalk.at<float>(2,1),finalk.at<float>(2,2));
	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);
	tf::Transform rostf;
	tf::Quaternion tfq(-tfqt.getZ(),tfqt.getX(),tfqt.getY(),-tfqt.getW());
	rostf.setOrigin(origin);
	rostf.setRotation(tfq);
	*/

/*
	Eigen::Matrix4f ptm(4,4);
           ptm<<
               pose.at<float>(0,0),pose.at<float>(0,1),pose.at<float>(0,2),pose.at<float>(0,3),
               pose.at<float>(1,0),pose.at<float>(1,1),pose.at<float>(1,2),pose.at<float>(1,3),
               pose.at<float>(2,0),pose.at<float>(2,1),pose.at<float>(2,2),pose.at<float>(2,3),
               0,0,0,1;
	pcl::transformPointCloud(*pc,*pc,ptm);
*/	
	tf::Vector3 origin1;
	origin1.setValue(
			pose.at<float>(0,3),pose.at<float>(1,3),pose.at<float>(2,3)+1);
	tf::Matrix3x3 tf3d1;
	tf3d1.setValue(
			pose.at<float>(0,0),pose.at<float>(0,1),pose.at<float>(0,2),
			pose.at<float>(1,0),pose.at<float>(1,1),pose.at<float>(1,2),
			pose.at<float>(2,0),pose.at<float>(2,1),pose.at<float>(2,2));
	tf::Transform rostf1(tf3d1,origin1);

	tf::Vector3 origin;
	origin.setValue(
			0,0,1);
	tf::Matrix3x3 tf3d;
	tf3d.setValue(
			1,0,0,
			0,1,0,
			0,0,1);
	tf::Transform rostf(tf3d,origin);


	//tf::Transform result=rostf*obtf.inverse();

	sensor_msgs::PointCloud2 pcl2;
	toROSMsg(*pc,pcl2);
	pcl2.header.seq=i;
	ros::Time time=ros::Time::now();
	pcl2.header.stamp=time;
	pcl2.header.frame_id=base;
	pclpub.publish(pcl2);
	//br.sendTransform(tf::StampedTransform(result,time,world,odom));
	br.sendTransform(tf::StampedTransform(rostf,time,world,base));
	br.sendTransform(tf::StampedTransform(rostf1,time,world,odom));
	i++;
	cout<<"send complete"<<endl;
}

bool Send::getloop()
{
	return loop;
}

void Send::startloop()
{
	std_msgs::String msg;
	string s="loop start";
	msg.data=s.c_str();
	cmdpub.publish(msg);
	loop=true;
}

void Send::cmdcallback(const std_msgs::String::ConstPtr& msg)
{
	string cmd=msg->data;
	if(cmd=="OK"&&!clouddeq.empty())
	{
		auto cloudmsg=clouddeq.begin();
		pointcloud_type cloud;
		pcl::io::loadPCDFile<point_type>(cloudmsg->first,cloud);
		Eigen::Matrix4f pt(4,4);
		pt<<
			0, 0, 1, 0,
			-1, 0, 0, 0,
			0,-1, 0, 0,
			0, 0, 0, 1;
		pcl::transformPointCloud(cloud,cloud,pt);
		sensor_msgs::PointCloud2 pcl2;
		toROSMsg(cloud,pcl2);
		pcl2.header.seq=j;
		pcl2.header.stamp=ros::Time::now();
		pcl2.header.frame_id="loop";
		pclpub.publish(pcl2);
		br.sendTransform(tf::StampedTransform(cloudmsg->second,pcl2.header.stamp,world,"loop"));
		clouddeq.pop_front();
		j++;
		if(clouddeq.empty())
		{
			std_msgs::String msg;
			string s="loop stop";
			msg.data=s.c_str();
			cmdpub.publish(msg);
			loop=false;
			j=0;
		}
	}
}

void Send::loopsendout(string file,cv::Mat matrix)
{
	cv::Mat finalk(4,4,CV_32F);
	finalk=matrix;
	tf::Vector3 origin;
	origin.setValue(
			finalk.at<float>(2,3),-finalk.at<float>(0,3),-finalk.at<float>(1,3)+0.58);
	tf::Matrix3x3 tf3d;
	tf3d.setValue(
			finalk.at<float>(0,0),finalk.at<float>(0,1),finalk.at<float>(0,2),
			finalk.at<float>(1,0),finalk.at<float>(1,1),finalk.at<float>(1,2),
			finalk.at<float>(2,0),finalk.at<float>(2,1),finalk.at<float>(2,2));
	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);
	tf::Transform rostf;
	tf::Quaternion tfq(-tfqt.getZ(),tfqt.getX(),tfqt.getY(),-tfqt.getW());
	rostf.setOrigin(origin);
	rostf.setRotation(tfq);
	clouddeq.push_back(make_pair(file,rostf));
}
}
