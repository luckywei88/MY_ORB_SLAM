#ifndef OBJECTS_H
#define OBJECTS_H

#include "Yolo.h"
#include "Object.h"
//#include "Cpf.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "KeyFrameDrawer.h"
#include <list>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

namespace ORB_SLAM2
{
    class Yolo;
 //   class CPF;
    class Object;
    class KeyFrame;
    class KeyFrameDrawer;

    class Objects
    {
    public:
    	typedef pcl::PointXYZRGB PointT;
    	typedef pcl::PointCloud<PointT> PointC;

	Objects(std::string yolo_d,std::string yolo_c,std::string yolo_w, std::string yolo_l);
	void detection(KeyFrame* kf);
        int max_index(float *a , int n);	
	void addDataBase(Object* obj);
	void computeICP(KeyFrame* kf,set<Object*>& objs);
	void Reset();
	void SetKeyFrameDrawer(KeyFrameDrawer* keyframedrawer);
	std::list<Object*> GetAllObjects();	

	//all object
	std::list<Object*> vector;

	//detect result
	std::vector<int> tmpBox;
	std::vector<int> tmpTypes;
	std::list<PointC::Ptr> tmpPCs;
	std::list<float*> tmpProbs;

	std::string getName(int i);	

	//detect type
	int classes;
	char **names;

    private:
	Yolo* yolo;
	KeyFrameDrawer* keyframedrawer;
//	CPF* cpf;
	pcl::VoxelGrid<PointT>* vg;
	pcl::StatisticalOutlierRemoval<PointT>* sor;
	pcl::IterativeClosestPoint<PointT,PointT>* icp;

	Eigen::Matrix4f* pt;
	int abc;
	//lock
    };
}

#endif
