#include "AllConfig.h"

using namespace std;

namespace ORB_SLAM2
{
    AllConfig::AllConfig()
    {
	bg="";
	config="";
	yolo_data="";
	yolo_weight="";
	yolo_cfg="";

	world="";
	base="";
	odom="";

	data_dir="";
	associate="";

	enable_ros=false;
	gui=false;
	type=0;
	
    }

    void AllConfig::setCon(ros::NodeHandle & n_private)
    {
	n_private.param("bag_of_word",bg,bg);
	n_private.param("config",config,config);

	n_private.param("yolo_data",yolo_data,yolo_data);
	n_private.param("yolo_weight",yolo_weight,yolo_weight);
	n_private.param("yolo_cfg",yolo_cfg,yolo_cfg);

	n_private.param("world_tf",world,world);
	n_private.param("base_tf",base,base);
	n_private.param("odom_tf",odom,odom);

	n_private.param("ros",enable_ros,enable_ros);
	n_private.param("use_gui",gui,gui);
	n_private.param("type",type,type);
    }

    void AllConfig::setCon(char **argv)
    {
	bg=argv[1];
	config=argv[2];

	yolo_data=argv[3];
	yolo_weight=argv[4];
	yolo_cfg=argv[5];

	world=argv[6];
	base=argv[7];
	odom=argv[8];

	data_dir=argv[9];
	associate=argv[10];

	string enable=argv[11];
	if(enable=="true")
	    enable_ros=true;
	string gu=argv[12];
	if(gu=="true")
	    gui=true;
    }
}
