#ifndef OBJECT_H
#define OBJECT_H

#include "KeyFrame.h"
#include <map>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>
#include <set>

namespace ORB_SLAM2
{
    class KeyFrame;


    class Object
    {
	public:
    	    typedef pcl::PointXYZRGB PointT;
    	    typedef pcl::PointCloud<PointT> PointC;

	    Object(KeyFrame* kf, PointC::Ptr pc, float* prob, int n);
	    ~Object();
	    //probility
	    std::vector<float> probs;
	    //max prob
	    float maxProb;
	    //max prob type
	    int clazz;
	    //obstime
	    int nObs;
	    //keyFrames PC
	    std::map<KeyFrame*,PointC::Ptr> pcmap;
	    //All pc
	    PointC::Ptr AllPC;
	    //KdTree
	    pcl::KdTreeFLANN<PointT> *kdtree;

	    bool compare(int type, PointC::Ptr pc);
	    void add(KeyFrame* kf, PointC::Ptr pc, float* prob);
	    void remove(KeyFrame*);
	    float getProb();
	    int getType();
	    PointC::Ptr GetPC();
    };
}

#endif
