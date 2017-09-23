#include "Objects.h"

using namespace std;

namespace ORB_SLAM2
{

Objects::Objects(string yolo_d,string yolo_c,string yolo_w)
{
    yolo=new Yolo();
    char* data=const_cast<char*>(yolo_d.c_str());
    char* config=const_cast<char*>(yolo_c.c_str());
    char* weight=const_cast<char*>(yolo_w.c_str());
    yolo->yolo_load(data,
	    config,
	    weight,
	    0.24,
	    0.5);
//    cpf=new CPF();
    vg=new pcl::VoxelGrid<PointT>();
    vg->setLeafSize(0.005,0.005,0.005);
    icp=new pcl::IterativeClosestPoint<PointT,PointT>();
    icp->setMaximumIterations(20);
    icp->setEuclideanFitnessEpsilon(0.005);
    icp->setTransformationEpsilon(1e-8);
    icp->setMaxCorrespondenceDistance(0.01);
}

int Objects::max_index(float *a, int n)
{
    if(n <= 0) return -1; 
    int i, max_i = 0;
    float max = a[0];
    for(i = 1; i < n; ++i){
	if(a[i] > max){
	    max = a[i];
	    max_i = i;
	}
    }   
    return max_i;

}

void Objects::detection(KeyFrame* kf)
{
    cv::Mat rgb=kf->mRGB;
    cv::Mat depth=kf->mDepth;
    IplImage* img=new IplImage(rgb);
    yolo->yolo_detect(img);

    classes=yolo->classes;	//类别数量
    names=yolo->names;	//类别

    int total=yolo->total;	//检测结果数量
    float **probs=yolo->probs;	//检测结果概率
    box *boxes=yolo->boxes;	//检测结果位置

    int w=rgb.cols;
    int h=rgb.rows;
    printf("start print result\n");
    tmpTypes.clear();
    tmpProbs.clear();
    tmpPCs.clear();
    float fxinv=kf->invfx;
    float fyinv=kf->invfy;
    float cx=kf->cx;
    float cy=kf->cy;

    //pointcloud
    /*PointC* cloud=new PointC();
    for(int i=0;i<h;i++)
    {
	for(int j=0;j<w;j++)
	{
	    float z=depth.at<float>(i,j);
	    if(z<0.01||z>5)
	    {
		pc[i][j]=NULL;
		continue;
	    }
	    float y=(i-cy)*z*fyinv;
	    float x=(j-cx)*z*fxinv;
	    cv::Vec3b color=rgb.at<cv::Vec3b>(i,j);
	    PointT* pt=new PointT();
	    pt->x=x;
	    pt->y=y;
	    pt->z=z;
	    unsigned char r=(unsigned char)color[0];
	    unsigned char g=(unsigned char)color[1];
	    unsigned char b=(unsigned char)color[2];
	    unsigned int col=(r<<16)|(g<<8)|b;
	    pt->rgb=*reinterpret_cast<float*>(&col);
	    cloud->push_back(*pt);
	}
    }
    cout<<"cloud "<<cloud->size()<<endl;
    kf->WriteCloud(cloud);
    */
    
    // hehe
    for(int i=0;i<total;i++)
    {
	int clazz = max_index(probs[i],classes);
	float prob=probs[i][clazz];
	if(prob>yolo->thresh)
	{
	    box b=boxes[i];
	    int left  = (b.x-b.w/2.)*w;
	    int right = (b.x+b.w/2.)*w;
	    int top   = (b.y-b.h/2.)*h;
	    int bot   = (b.y+b.h/2.)*h;

	    if(left < 0) left = 0;
	    if(right > w-1) right = w-1;
	    if(top < 0) top = 0;
	    if(bot > h-1) bot = h-1;

	    printf("%d %s : %.0f%%\n", i,names[clazz],prob*100);		
	    string s(names[clazz]);
	    if(s=="person")
	    {
		//delete unstable feature
		kf->DeleteFeature(left,right,top,bot);

	    }
	    else
	    {
		//segmentation
		PointC::Ptr cloud(new PointC);
		for(int n=top;n<bot+1;n++)
		{
		    for(int j=left;j<right+1;j++)
		    {
			float z=depth.at<float>(n,j);
			if(z<0.01||z>5)
			{
			    continue;
			}
			float y=(n-cy)*z*fyinv;
			float x=(j-cx)*z*fxinv;
			cv::Vec3b color=rgb.at<cv::Vec3b>(n,j);
			PointT pt;
			pt.x=x;
			pt.y=y;
			pt.z=z;
			unsigned char r=(unsigned char)color[0];
			unsigned char g=(unsigned char)color[1];
			unsigned char b=(unsigned char)color[2];
			unsigned int col=(r<<16)|(g<<8)|b;
			pt.rgb=*reinterpret_cast<float*>(&col);
			cloud->push_back(pt);	 
		    }
		}
		vg->setInputCloud(cloud);
		vg->filter(*cloud);

		Eigen::Matrix4f pt(4,4);
		pt<<
		    0, 0, 1, 0,
		    -1, 0, 0, -0.05,
		    0,-1, 0, 0,
		    0, 0, 0, 1;
		pcl::transformPointCloud(*cloud,*cloud,pt);
		//	    cpf->Segment(cloud);
		//kf->writeCloud(cpf->Segment(cloud),i);
		//kf->WriteCloud(cloud,i);

		//add result
		tmpTypes.push_back(clazz);
		tmpPCs.push_back(cloud);
		float* tmpProb=(float*)calloc(classes,sizeof(float));
		for(int j=0;j<classes;j++)
		{
		    tmpProb[j]=probs[i][j]*100.0;
		}
		tmpProbs.push_back(tmpProb);
	    }
	}
    }
    yolo->delet();
}

void Objects::addDataBase(Object* obj)
{
    vector.push_back(obj);
}

void Objects::computeICP(KeyFrame* kf,set<Object*>& objs)
{
    if(objs.size()==0||tmpPCs.size()==0)
	return;
    set<Object*>::iterator tmpobjstart=objs.begin();
    set<Object*>::iterator tmpobjend=objs.end();
    int k=0;
    PointC::Ptr target(new PointC);
    while(tmpobjstart!=tmpobjend)
    {
	Object* tmpobj=*tmpobjstart;
	auto mapstart=tmpobj->pcmap.begin();
	auto mapend=tmpobj->pcmap.end();
	PointC::Ptr totalpc(new PointC);
	while(mapstart!=mapend)
	{
	    KeyFrame* kf=mapstart->first;
	    PointC::Ptr pc=mapstart->second;

	    cv::Mat pose=kf->GetPoseRight();
	    Eigen::Matrix4f ptm=Converter::toMatrix4d(pose);

	    PointC::Ptr tmppc(new PointC);
	    pcl::transformPointCloud(*pc,*tmppc,ptm);
	    *totalpc+=*tmppc;
	    mapstart++;
	}
	tmpobjstart++;
	k++;
	*target+=*totalpc;
    }

    PointC::Ptr input(new PointC);
    list<PointC::Ptr>::iterator PCsBegin=tmpPCs.begin();
    list<PointC::Ptr>::iterator PCsEnd=tmpPCs.end();
    while(PCsBegin!=PCsEnd)
    {
	PointC::Ptr tmppc=*PCsBegin;
	*input+=*tmppc;
	PCsBegin++;
    }
    cv::Mat pose=kf->GetPoseRight();
    Eigen::Matrix4f ptm=Converter::toMatrix4d(pose);

    pcl::transformPointCloud(*input,*input,ptm);

    icp->setInputSource(input);
    icp->setInputTarget(target);
    PointC::Ptr finalpc(new PointC);
    icp->align(*finalpc);

    Eigen::Matrix4f finalm=icp->getFinalTransformation();
    cv::Mat finalpose=Converter::toCvMat(finalm);
    kf->CorrectMapPoint(finalpose);
    finalm=finalm*ptm;
    //finalm=ptm*finalm;
    finalpose=Converter::toCvMat(finalm);
    kf->SetPoseByRight(finalpose);
}
}
