#include "Objects.h"

using namespace std;

namespace ORB_SLAM2
{

	Objects::Objects(string yolo_d,string yolo_c,string yolo_w,string yolo_l)
	{
		yolo=new Yolo();
		char* data=const_cast<char*>(yolo_d.c_str());
		char* config=const_cast<char*>(yolo_c.c_str());
		char* weight=const_cast<char*>(yolo_w.c_str());
		char* label=const_cast<char*>(yolo_l.c_str());
		yolo->yolo_load(data,
				config,
				weight,
				label,	
				0.35,
				0.5);
		//    cpf=new CPF();  0.35
		vg=new pcl::VoxelGrid<PointT>();
		vg->setLeafSize(0.005,0.005,0.005);

                sor=new pcl::StatisticalOutlierRemoval<PointT>();
		sor->setMeanK(50);
		sor->setStddevMulThresh(1.0);

		icp=new pcl::IterativeClosestPoint<PointT,PointT>();
		icp->setMaximumIterations(20);
		icp->setEuclideanFitnessEpsilon(0.005);
		icp->setTransformationEpsilon(1e-8);
		icp->setMaxCorrespondenceDistance(0.01);

		pt=new Eigen::Matrix4f(4,4);
		*pt<<
			0, 0, 1, 0,
			-1, 0, 0, -0.05,
			0,-1, 0, 0,
			0, 0, 0, 1;
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

	void Objects::SetKeyFrameDrawer(KeyFrameDrawer* tkeyframedrawer)
	{
		keyframedrawer=tkeyframedrawer;
	}

	list<Object*> Objects::GetAllObjects()
	{
		return std::list<Object*>(vector.begin(),vector.end());
	}

	string Objects::getName(int i)
	{
		string name(names[i]);
		return name;
	}

	void Objects::Reset()
	{
		for(auto it=vector.begin(); it!= vector.end(); it++)
			delete *it;
		vector.clear();
		tmpTypes.clear();
		tmpPCs.clear();
		tmpProbs.clear();
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
		tmpTypes.clear();
		tmpPCs.clear();
		tmpProbs.clear();
		float fxinv=kf->invfx;
		float fyinv=kf->invfy;
		float cx=kf->cx;
		float cy=kf->cy;

		//vector<int> tmpbox;
		std::vector<int> tmpbox,tmpbox1,tmpTypes1;

		//pointcloud
		PointC::Ptr cloud=boost::make_shared<PointC>();
		for(int i=0;i<h;i++)
		{
			for(int j=0;j<w;j++)
			{
				float z=depth.at<float>(i,j);
				if(z<0.01)
				{
					PointT pt;
					pt.x=pt.y=pt.z=0;	
					cloud->push_back(pt);
					continue;
				}
				float y=(i-cy)*z*fyinv;
				float x=(j-cx)*z*fxinv;
				cv::Vec3b color=rgb.at<cv::Vec3b>(i,j);
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
		pcl::transformPointCloud(*cloud,*cloud,*pt);
		cout<<"cloud "<<cloud->size()<<endl;

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
					PointC::Ptr cloud1=boost::make_shared<PointC>();
					//segmentation
					for(int n=top;n<bot+1;n++)
					{
						for(int j=left;j<right+1;j++)
						{
							PointT pt=(*cloud)[n*w+j];
							cloud1->push_back(pt);
							/*	
							unsigned char r=255;
							unsigned char g=0;
							unsigned char b=0;
							unsigned int col=(r<<16)|(g<<8)|b;
							(*cloud)[n*w+j].rgb=*reinterpret_cast<float*>(&col);
							*/
						}
					}

					vg->setInputCloud(cloud1);
					vg->filter(*cloud1);
					sor->setInputCloud(cloud1);
					sor->filter(*cloud1);

					//	    cpf->Segment(cloud);
					//kf->writeCloud(cpf->Segment(cloud),i);
					//kf->WriteCloud(cloud,i);

					//add result
					tmpbox.push_back(left);
					tmpbox.push_back(right);
					tmpbox.push_back(top);
					tmpbox.push_back(bot);
					tmpTypes.push_back(clazz);
					tmpPCs.push_back(cloud1);
					float* tmpProb=(float*)calloc(classes,sizeof(float));
					for(int j=0;j<classes;j++)
					{
						tmpProb[j]=probs[i][j]*100.0;
					}
					tmpProbs.push_back(tmpProb);

				}
				tmpbox1.push_back(left);
				tmpbox1.push_back(right);
				tmpbox1.push_back(top);
				tmpbox1.push_back(bot);
				tmpTypes1.push_back(clazz);

			}
		}
		vg->setInputCloud(cloud);
		vg->filter(*cloud);
		sor->setInputCloud(cloud);
		sor->filter(*cloud);
		kf->SetPointCloud(cloud);

		keyframedrawer->Update(img,names, yolo->alphabet,classes,tmpbox1,tmpTypes1);
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
		PointC::Ptr target=boost::make_shared<PointC>();
		while(tmpobjstart!=tmpobjend)
		{
			Object* tmpobj=*tmpobjstart;
			auto totalpc=tmpobj->GetPC();	
			tmpobjstart++;
			*target+=*totalpc;
		}

		PointC::Ptr input=boost::make_shared<PointC>();
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
		PointC::Ptr finalpc=boost::make_shared<PointC>();
		icp->align(*finalpc);

		Eigen::Matrix4f finalm=icp->getFinalTransformation();
		cv::Mat finalpose=Converter::toCvMat(finalm);
		kf->CorrectMapPoint(finalpose);
		//	cout<<"correct"<<endl;
		//	cout<<finalm<<endl;
		finalm=finalm*ptm;
		//	finalm=ptm*finalm;
		finalpose=Converter::toCvMat(finalm);
		kf->SetPoseByRight(finalpose);
	}



}
