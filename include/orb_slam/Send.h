#ifndef SEND_H
#define SEND_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>


#include <string>
#include <vector>
#include <deque>
#include <thread>
#include <iostream>
#include "Frame.h"

namespace ORB_SLAM2
{

	class Send
	{
		public:
			typedef pcl::PointXYZRGB point_type;
			typedef pcl::PointCloud<point_type> pointcloud_type;
			typedef pcl::PointXYZRGB PointT;
			typedef pcl::PointCloud<PointT> PointC;
			Send(std::string world,std::string base,std::string odom);
			~Send();
			void Frame(cv::Mat image);
			void KeyFrame(cv::Mat image);
			void sendout(PointC::Ptr pc,cv::Mat matrix);
			void loopsendout(string file,cv::Mat matrix);
			void cmdcallback(const std_msgs::String::ConstPtr& msg);
			void startloop();
			bool getloop();

			ros::Publisher pclpub,looppub,cmdpub,framepub,keypub;
			ros::Subscriber cmdsub;
			tf::TransformBroadcaster br;
			tf::TransformListener tf_listener;
			std::string world,base,odom;
			size_t i,j;
			bool loop;
			std::deque<std::pair<string, tf::Transform>> clouddeq;
	};

}

#endif
