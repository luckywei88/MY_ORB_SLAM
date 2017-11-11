#ifndef ALLCONFIG_H
#define ALLCONFIG_H

#include <iostream>
#include <string>
#include <ros/ros.h>


namespace ORB_SLAM2
{
    class AllConfig
    {
	public:
	    AllConfig();
	    std::string bg;
	    std::string config;
	    std::string yolo_data;
	    std::string yolo_weight;
	    std::string yolo_cfg;
	    std::string yolo_label;
	    std::string world;
	    std::string base;
	    std::string odom;
	    std::string data_dir;
	    std::string associate;
	    bool enable_ros;
	    bool gui;
	    int type;
	    void setCon(ros::NodeHandle& n);	    
	    void setCon(char **argv);
    };
}

#endif
